// ============================================================================
//  src/preview/PreviewWorker.cpp
//
//  依据：SYS-09 §2.1 / §11 / §13.1、ENG-05 §11
// ============================================================================

#include "preview/PreviewWorker.h"

namespace aircraft
{
namespace preview
{

PreviewWorker::PreviewWorker(PreviewManager& manager)
    : manager_(manager)
{
}

PreviewWorker::~PreviewWorker()
{
    // 析构必须 join（见头文件的生命周期约定）：本线程访问的 manager_
    // 与 queue_ 都活在 PreviewWorker 之外，若线程比它们活得久，
    // 就是必然的 use-after-free。
    stop();
}

bool PreviewWorker::start()
{
    if (running_.load())
    {
        return false;  // 已在运行，幂等
    }

    running_ = true;

    try
    {
        thread_ = std::thread(&PreviewWorker::run, this);
    }
    catch (...)
    {
        // 线程创建失败（资源耗尽等）时必须回滚 running_，
        // 否则对象会停在"自称在运行、实际没有线程"的状态：
        // running() 返回 true 骗过上层，而预览永远不更新。
        running_ = false;
        throw;
    }

    return true;
}

void PreviewWorker::stop()
{
    // ⚠ 无条件置位，不做 "if (!running_) return;" 的短路：
    // running_ 为 false 并不代表 thread_ 不可 join —— start() 建好线程后
    // 状态可能已被别处清过。漏掉一次 join，std::thread 的析构就会
    // 调 std::terminate（进程直接死，不留日志），而这个分支只有在
    // 特定交错下才走到，属于"偶发崩溃"里最难查的一类。
    running_ = false;

    // 唤醒可能正在 waitPop 里休眠的线程：否则要等满一个
    // pollTimeoutMs 才轮得到它检查 running_。
    manager_.queue().wakeAll();

    if (thread_.joinable())
    {
        thread_.join();
    }
}

bool PreviewWorker::running() const
{
    return running_.load();
}

std::uint64_t PreviewWorker::processedCount() const
{
    return processed_.load();
}

std::uint64_t PreviewWorker::publishedCount() const
{
    return published_.load();
}

std::uint64_t PreviewWorker::skippedCount() const
{
    return skipped_.load();
}

void PreviewWorker::setPollTimeoutMs(int ms)
{
    pollTimeoutMs_ = (ms > 0) ? ms : kDefaultPollTimeoutMs;
}

int PreviewWorker::pollTimeoutMs() const
{
    return pollTimeoutMs_.load();
}

// ---------------------------------------------------------------------------

void PreviewWorker::run()
{
    data::PreviewFrame frame;

    while (running_.load())
    {
        // waitPop 带超时而不是无条件阻塞：这样即使 wakeAll() 因为任何
        // 原因没被调用（例如 stop() 与 start() 交错），线程也会在
        // 一个超时周期内自己发现 running_ 已清零并退出。
        // 把"能退出"建立在自己的超时上，而不是建立在对唤醒的信任上。
        if (!manager_.queue().waitPop(frame, pollTimeoutMs_.load()))
        {
            continue;  // 超时或关闭唤醒
        }

        // 取出后再确认一次运行状态：stop() 可能在 waitPop 返回的瞬间
        // 发生，此时不应再向 Manager 发布帧（Manager 可能正在被销毁）。
        if (!running_.load())
        {
            break;
        }

        ++processed_;

        switch (manager_.workerDeliver(frame))
        {
        case PreviewManager::DeliverResult::PUBLISHED:
            ++published_;
            break;
        case PreviewManager::DeliverResult::SKIPPED_ROLE_MISMATCH:
            // 不是错误：用户切了相机，队列里还留着上一焦段的在途帧。
            // 丢掉它们正是"切换后不闪一下旧画面"的实现方式。
            ++skipped_;
            break;
        }
    }

    // 退出前不发布"最后一帧"：stop() 意味着显示通路即将随上层一并停止，
    // 此时再写一次显示槽，可能与 UI 自己的析构竞态。
    // 预览帧是"可丢"的（SYS-09 §2.1），少显示最后一帧没有任何后果。
}

}  // namespace preview
}  // namespace aircraft
