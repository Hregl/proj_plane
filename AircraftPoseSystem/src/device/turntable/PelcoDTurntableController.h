#pragma once

// ============================================================================
//  src/device/turntable/PelcoDTurntableController.h
//
//  依据：SYS-10 V1.1 §6.3～§6.6、§17（线程）、§18（tick 影响）、
//        SYS-06 V2.7 §11.2、ENG-09 V2.7 §4.3、裁决 C-22/C-23
//
//  作用：`ITurntableController` 的真实 Pelco-D/UDP 后端，并实现并列的
//  `ITurntableDiagnostics`。
//  · `PelcoDSession` 负责协议与状态机（无线程）；
//  · 本类负责 socket 生命周期、**单所有者 worker**、停止屏障与快照发布。
//
//  ⚠ 线程/锁约定（裁决 C-23）：
//    · worker 是启动后的**唯一 I/O 所有者**。`initialize()` 在 worker
//      启动前完成（它是启动期事务，不参与运行期抢占）。
//    · `move()==true` 仅表示**命令已受理**：调用线程只把命令写入单槽
//      （带单调序号）后立即返回，**不等待回显**。发送/回显结果通过
//      `diagnostics().lastCommandOutcome` 与命令序号给出。
//    · `stop()` 分配更高的停止序号、清空待执行运动槽，并等待 worker
//      处理到该停止（有界）。worker 在每次发送前、每轴切换前、每个
//      接收切片后检查停止序号；屏障之前的旧命令一律作废，停止后不得
//      恢复执行。
//    · `state()` / `diagnostics()` 只读原子快照 + 短锁，不拿 I/O 锁、
//      不做 I/O，可在每 16 ms 的主 tick 里安全调用。
//    · 析构：停止 worker → join → 必要时停止 → close。
//
//  ⚠ 单所有者 worker 是**选定方案**，不是唯一可能的方案。SYS-09 记录
//    选择理由与测试条件，不写“唯一方案”或未经实测的时延绝对保证。
//
//  ⚠ 诊断 sink 由 worker 线程调用，**只能**接 `Logger`（自身持锁）；
//    本类在锁外调用并捕获异常 —— sink 抛异常不得导致 std::terminate。
// ============================================================================

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "data/ErrorInfo.h"
#include "data/TurntableConfig.h"
#include "data/TurntableDiagnostics.h"
#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/ITurntableController.h"
#include "device/turntable/ITurntableDiagnostics.h"

namespace aircraft
{
namespace device
{

class PelcoDSession;

class PelcoDTurntableController final : public ITurntableController,
                                        public ITurntableDiagnostics
{
public:
    /// @param transport 设备层内部 seam；为空时创建 POSIX UDP 实现。
    explicit PelcoDTurntableController(
        const data::TurntableConfig& config,
        std::shared_ptr<IPelcoTransport> transport = nullptr);
    ~PelcoDTurntableController() override;

    PelcoDTurntableController(const PelcoDTurntableController&) = delete;
    PelcoDTurntableController& operator=(const PelcoDTurntableController&) =
        delete;

    bool initialize() override;

    /// ⚠ 返回 true 只表示**已受理**，不表示设备已接收或到位。
    bool move(const data::TurntableCommand& command) override;

    data::TurntableState state() override;
    void stop() override;

    /// 诊断快照（实现 `ITurntableDiagnostics`）。
    data::TurntableDiagnostics diagnostics() const override;

    /// 设置诊断输出（worker 线程调用）。必须在 `initialize()` 之前设置。
    void setDiagnosticSink(std::function<void(const std::string&)> sink);

    /// 注入单调时钟（测试用）。必须在 `initialize()` 之前设置。
    void setClock(std::function<std::uint64_t()> clock);

    std::string    lastErrorText() const;
    data::ErrorInfo lastError() const;

    bool initialized() const { return initialized_.load(); }

    /// ⚠ 单元测试专用：仅用于在 fake transport 上验证停止/故障路径。
    /// 生产装配不调用；它不改变 WorkMode 可信度，也不改变只读门禁。
    void setTestMotionGate(bool allow);

private:
    struct PendingMove
    {
        data::TurntableCommand command;
        std::uint64_t          seq = 0;
    };

    void workerLoop();
    void handleCommand(const PendingMove& move);
    void setCommandOutcome(std::uint64_t seq,
                           data::TurntableCommandOutcome outcome,
                           const std::string& message);
    bool commandSuperseded(std::uint64_t seq) const;

    data::TurntableConfig            config_;
    std::shared_ptr<IPelcoTransport> transport_;
    std::unique_ptr<PelcoDSession>   session_;

    // ---- 命令槽（move() 只写这里，立即返回）----
    mutable std::mutex                commandMutex_;
    std::condition_variable           commandCv_;
    std::optional<PendingMove>        pendingMove_;
    std::uint64_t                     nextCommandSeq_   = 0;
    std::uint64_t                     latestCommandSeq_ = 0;

    // ---- 停止屏障 ----
    std::atomic<std::uint64_t>        stopRequestSeq_{0};
    std::atomic<std::uint64_t>        stopCompletedSeq_{0};
    std::mutex                        stopWaitMutex_;
    std::condition_variable           stopWaitCv_;

    // ---- 生命周期 ----
    std::thread             worker_;
    std::atomic<bool>       running_{false};
    std::atomic<bool>       initialized_{false};

    // ---- 诊断 ----
    mutable std::mutex diagMutex_;
    std::uint64_t      lastCommandSeq_ = 0;
    data::TurntableCommandOutcome lastCommandOutcome_ =
        data::TurntableCommandOutcome::None;
    std::string        lastCommandMessage_;

    std::function<void(const std::string&)> diagnosticSink_;
    std::function<std::uint64_t()>          clock_;

    // 仅 worker 线程访问：诊断去重，避免每 50 ms 重复同一条错误。
    std::string lastReportedError_;
};

}  // namespace device
}  // namespace aircraft
