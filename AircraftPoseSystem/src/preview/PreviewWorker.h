#pragma once

// ============================================================================
//  src/preview/PreviewWorker.h
//
//  依据：SYS-09 §2.1 / §11 / §13.1、ENG-05 §2.1 / §11 / §13.1
//        ENG-02 §9.2 / ENG-04 §9.2（类设计）、ENG-01 §7（文件清单）
//
//  作用：预览流水线的**消费线程**（SYS-09 §11：输入 PreviewQueue，
//  输出 PreviewFrame）。它是"采集侧快、显示侧慢"这个矛盾的缓冲区搬运用工：
//  相机线程只管 submitFrame() 后立刻返回，慢的那一头由本线程承担。
//
//  ---- 为什么用 std::thread 而不是 QThread（与 SYS-09 §15 的偏离）----
//
//  SYS-09 §15 / ENG-05 §15 冻结的线程实现规范是"QThread + QObject Worker，
//  不继承 QThread"。本模块**不链接 Qt**，依据是 ENG-03 §12.4
//  （preview 的依赖为 data + optical，无 Qt）与 ENG-01 §17 的依赖链
//  （Qt 位于 ui 层）。两条都是冻结条目且相互冲突，取舍理由写在
//  PreviewManager.h 的文件头，此处不重复。
//
//  该条件里被明确禁止的那一项 —— "不采用：继承 QThread" —— 本实现
//  完全遵守（本模块根本不用 QThread）。剩余意图（生命周期清晰、
//  易测试）用 std::thread 同样达成，且 tests/preview 无需
//  QCoreApplication 或事件循环即可跑完整链路。
//
//  ---- 生命周期约定（RAII）----
//
//  start() 创建线程；stop() 置位停止标志、唤醒队列、并 **join** ——
//  stop() 返回后线程一定已经退出，不会在对象析构后继续访问 manager_。
//  析构函数调用 stop()，故"忘记 stop 就析构"不会导致 use-after-free。
//  这是刻意的：预览线程访问的 manager_ 与 queue_ 都活在别处，
//  线程比它们活得久就是必然崩溃。
// ============================================================================

#include <atomic>
#include <cstdint>
#include <thread>

#include "preview/PreviewManager.h"

namespace aircraft
{
namespace preview
{

/// 预览线程（SYS-09 §11、ENG-02 §9.2）。
class PreviewWorker
{
public:
    /// 队列空时的等待上限。
    ///
    /// 它同时是两个量的折中：
    ///   · 关闭延迟 —— stop() 后线程最多再等这么久就自然退出；
    ///   · 空转代价 —— 无帧时线程真正休眠（condition_variable），
    ///     不是忙等，故这个值不需要很小。
    /// 50 ms 使关闭在任何情况下都不超过一个可感知的瞬间，
    /// 而 20 fps 的预览也不会有可见延迟（有帧时会被立即唤醒，
    /// 与这个值无关）。测试可经 setPollTimeoutMs() 调小以加快收敛。
    static constexpr int kDefaultPollTimeoutMs = 50;

    /// @param manager 预览管理器，本线程从中取队列、向其中发布显示帧。
    ///        用引用而非 6.md §九 的 `PreviewManager*`：空指针会让
    ///        start() 静默地什么都不做（或崩溃在另一处），
    ///        而"预览线程启动了但没反应"是极难定位的现象。
    ///        调用方须保证 manager 的生命周期长于本对象。
    explicit PreviewWorker(PreviewManager& manager);

    /// 析构时自动 stop()（见文件头生命周期约定）。
    ~PreviewWorker();

    PreviewWorker(const PreviewWorker&)            = delete;
    PreviewWorker& operator=(const PreviewWorker&) = delete;

    /// 启动线程。已在运行时返回 false（幂等，不重复建线程）。
    bool start();

    /// 停止并**等待**线程退出（join）。未在运行时直接返回。
    void stop();

    bool running() const;

    /// 从队列取出的帧数（含被角色过滤丢弃的）。
    std::uint64_t processedCount() const;

    /// 实际发布为显示帧的数量。
    std::uint64_t publishedCount() const;

    /// 因不属于当前显示源而被丢弃的帧数。
    ///
    /// 与 PreviewManager::skippedCount() 是两个不同的量：
    /// 本值是**本线程**的计数（含线程重启后清零），
    /// 后者是 Manager 的累计值。二者不相等时，说明有多条预览线程
    /// 或队列被别处消费过 —— 这是个值得警惕的信号。
    std::uint64_t skippedCount() const;

    /// 设置队列等待上限，单位 ms。测试用（加快 stop 收敛）。
    /// 传入 <= 0 时回退为 kDefaultPollTimeoutMs，避免忙轮询。
    void setPollTimeoutMs(int ms);
    int pollTimeoutMs() const;

private:
    /// 线程体：循环"取帧 → 交付"，直到 running_ 被清零。
    void run();

    PreviewManager& manager_;

    /// ⚠ 用 std::thread 而非 joinable 标志：std::thread 自身就能回答
    /// "是否需要在析构时 join"，多存一个 bool 只会给二者不一致留出机会。
    std::thread thread_;

    std::atomic<bool> running_{false};

    std::atomic<std::uint64_t> processed_{0};
    std::atomic<std::uint64_t> published_{0};
    std::atomic<std::uint64_t> skipped_{0};

    std::atomic<int> pollTimeoutMs_{kDefaultPollTimeoutMs};
};

}  // namespace preview
}  // namespace aircraft
