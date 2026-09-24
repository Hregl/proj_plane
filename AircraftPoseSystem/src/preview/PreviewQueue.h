#pragma once

// ============================================================================
//  src/preview/PreviewQueue.h
//
//  依据：SYS-09 §2.1 / §13.1、ENG-05 §2.1 / §13.1、ENG-01 §7
//
//  作用：预览流水线的**跨线程通道**。采集侧（相机线程）写入，
//  PreviewWorker 线程读出。SYS-09 §2.1 把系统划为双流水线：
//      Preview     —— 低延迟、可丢帧、不影响测量
//      Measurement —— 数据可靠、时间一致
//  本类只服务前者，故策略是**丢旧保新**（SYS-09 §13.1）：
//  满载时丢掉最旧的一帧。若改成丢新保旧，相机比显示快时队列会永远
//  停在最早的那几帧，预览显示的是几秒前的画面——而"实时"正是本通路
//  存在的唯一理由，宁可少显示一帧也不能显示过期的帧。
//
//  ⚠ 它是**有界**队列且**不阻塞**写入方：push() 永不等待。
//  这是 SYS-09 §2.1"预览不影响测量"的落点 —— 三相机共用一个采集线程时，
//  预览侧的阻塞会直接拖慢测量侧的取流节拍。
//
//  ⚠ 数据所有权（SYS-09 §14 要求图像用 shared_ptr<ImageFrame>）：
//  本队列按值保存 data::PreviewFrame，其 image 是 cv::Mat ——
//  cv::Mat 的拷贝是**浅拷贝**（引用计数 +1，像素缓冲不复制），
//  故"每帧复制一张百万像素图"的代价并不存在，§14 的意图（避免大块
//  像素的深拷贝）由 cv::Mat 自身的语义满足。
//  之所以不用 shared_ptr<ImageFrame>：ENG-09 §5.7 冻结的
//  `PreviewFrame { ImageFrame frame; uint64_t displayTimestamp; }` 就是
//  值语义，改用指针会与冻结定义冲突，且丢旧保新时释放的时机反而更难掌握。
//
//  本类是 header-only（6.md §五 亦如此）：它没有需要隐藏的实现细节，
//  全部内容都是模板化的容器操作与同步原语。
// ============================================================================

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "data/PreviewFrame.h"

namespace aircraft
{
namespace preview
{

/// 实时显示缓存，满载丢旧保新（SYS-09 §13.1）。
class PreviewQueue
{
public:
    /// SYS-09 §13.1 冻结的容量区间。
    static constexpr std::size_t kMinCapacity     = 3;
    static constexpr std::size_t kMaxCapacity     = 5;
    static constexpr std::size_t kDefaultCapacity = 5;

    /// @param capacity 期望容量。越界值被夹取到 [kMinCapacity, kMaxCapacity]，
    ///        实际生效值可用 capacity()、是否发生过夹取可用 clamped() 查询
    ///        —— 夹取是降级行为，必须可观测（"不得静默降级"）。
    explicit PreviewQueue(std::size_t capacity = kDefaultCapacity)
        : capacity_(clampCapacity(capacity))
        , clamped_(capacity_ != capacity)
    {
    }

    PreviewQueue(const PreviewQueue&)            = delete;
    PreviewQueue& operator=(const PreviewQueue&) = delete;

    /// 写入一帧。满载时丢弃**最旧**的一帧。永不阻塞。
    void push(const data::PreviewFrame& frame)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (queue_.size() >= capacity_)
            {
                // ⚠ 先判空再 pop_front：capacity_ 已被夹取到 >= 3，
                // 此处其实不会为空；但容量来源是外部配置，一旦夹取逻辑
                // 被改动，空 deque 上的 pop_front 是未定义行为。
                // 用一次判空的代价换掉这个隐患。
                if (!queue_.empty())
                {
                    queue_.pop_front();
                    ++dropped_;
                }
            }

            queue_.push_back(frame);
            ++pushed_;
        }

        // 只唤醒一个等待者：本队列的消费者是 PreviewWorker 单线程
        // （SYS-09 §2.1 只有一条预览流水线）。notify_all 会引入无谓的
        // 惊群，而若将来真出现多消费者，必须改的就是这里而不是队列结构。
        cv_.notify_one();
    }

    /// 非阻塞取出一帧（最旧）。@return false 表示当前为空。
    bool pop(data::PreviewFrame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (queue_.empty())
        {
            return false;
        }

        frame = queue_.front();
        queue_.pop_front();
        return true;
    }

    /// 阻塞取出一帧，至多等待 timeoutMs 毫秒。
    ///
    /// PreviewWorker 用这个而不是 pop()：轮询空队列会让预览线程以
    /// CPU 满载空转（在国产 CPU 平台上这是实打实的功耗与温升），
    /// 而 waitPop 在无帧时真正休眠、有帧时被 push() 立刻唤醒，
    /// 既不空转也不引入轮询间隔的延迟。
    ///
    /// 超时的存在是**为了关闭流程**：stop() 置位运行标志后，
    /// 线程最多再过 timeoutMs 就自然退出，不必依赖 wakeAll() 一定被调用。
    /// @return false 表示超时（或队列为空且超时）。
    bool waitPop(data::PreviewFrame& frame, int timeoutMs)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        const bool ready = cv_.wait_for(
            lock, std::chrono::milliseconds(timeoutMs),
            [this] { return !queue_.empty() || woken_; });

        if (!ready)
        {
            return false;  // 超时
        }

        // 唤醒标记是**一次性**的：无论本次是否真取到帧，都要清掉，
        // 否则 wakeAll() 之后剩下的每一次 waitPop 都会立刻返回，
        // 关闭流程之外的那些调用会退化成忙轮询。
        woken_ = false;

        if (queue_.empty())
        {
            return false;  // 被 wakeAll() 唤醒（关闭流程），队列已空
        }

        frame = queue_.front();
        queue_.pop_front();
        return true;
    }

    /// 取出最新一帧但**不**移除它（供诊断与"取当前画面"使用）。
    bool latest(data::PreviewFrame& frame) const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (queue_.empty())
        {
            return false;
        }

        frame = queue_.back();
        return true;
    }

    /// 唤醒所有等待者。用于关闭流程，使 waitPop 立即返回 false
    /// 而不必等满一个超时周期。
    void wakeAll()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            woken_ = true;
        }
        cv_.notify_all();
    }

    /// 清空队列。**不清零计数** —— 丢帧总数是跨整个运行期的观测量，
    /// 清零点会让"运行了多久、丢了多少"无法回答。
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    std::size_t capacity() const { return capacity_; }

    /// 构造时是否发生过容量夹取（降级可见性，见构造函数说明）。
    bool clamped() const { return clamped_; }

    /// 因满载被丢弃的帧数（丢旧保新的实际发生次数）。
    /// SYS-09 §20 的稳定性要求队列积压可观测：若该值长期增长，
    /// 说明预览消费跟不上采集，应降低分辨率或帧率，而不是加大容量。
    std::uint64_t droppedCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

    /// 累计写入帧数（含后来被丢弃的）。
    std::uint64_t pushedCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pushed_;
    }

private:
    static std::size_t clampCapacity(std::size_t c)
    {
        if (c < kMinCapacity) { return kMinCapacity; }
        if (c > kMaxCapacity) { return kMaxCapacity; }
        return c;
    }

    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    std::deque<data::PreviewFrame> queue_;
    std::size_t             capacity_;
    bool                    clamped_;
    bool                    woken_ = false;

    std::uint64_t pushed_  = 0;
    std::uint64_t dropped_ = 0;
};

}  // namespace preview
}  // namespace aircraft
