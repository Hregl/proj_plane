#pragma once

// ============================================================================
//  src/device/turntable/PelcoDTurntableController.h
//
//  依据：SYS-10 V1.1 §6.3～§6.6、SYS-06 V2.7 §11.2、ENG-09 V2.7 §4.3、C-22
//
//  作用：`ITurntableController` 的真实 Pelco-D/UDP 后端。
//  · `PelcoDSession` 负责协议与状态机（无线程）；
//  · 本类负责 socket 生命周期、后台 I/O 线程、`ioMutex_` 与快照发布。
//
//  ⚠ 线程/锁约定：
//    · `initialize()` / `move()` / `stop()` 与 worker 的 `pollOnce()`
//      共用 `ioMutex_` —— 否则轮询回复会被 `move()` 的等回显抢走；
//    · `state()` **只读原子快照**，不拿锁、不做 I/O，可在每 16 ms 的
//      主 tick 里安全调用；它绝不会因为 UDP 超时而阻塞界面；
//    · 析构：停止 worker → join → 必要时尽力 stop → close。
//
//  ⚠ 诊断 sink 由 worker 线程调用，**只能**接 `Logger`（自身持锁），
//    不得接 `SystemInitializer::warn()`（会改 `warnings_`，与主线程竞争）。
// ============================================================================

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "data/ErrorInfo.h"
#include "data/TurntableConfig.h"
#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/ITurntableController.h"

namespace aircraft
{
namespace device
{

class PelcoDSession;

class PelcoDTurntableController final : public ITurntableController
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
    bool move(const data::TurntableCommand& command) override;
    data::TurntableState state() override;
    void stop() override;

    /// 设置诊断输出（worker 线程调用）。必须在 `initialize()` 之前设置。
    void setDiagnosticSink(std::function<void(const std::string&)> sink);

    /// 注入单调时钟（测试用）。必须在 `initialize()` 之前设置。
    void setClock(std::function<std::uint64_t()> clock);

    std::string    lastErrorText() const;
    data::ErrorInfo lastError() const;

    bool initialized() const { return initialized_.load(); }

private:
    void workerLoop();

    data::TurntableConfig            config_;
    std::shared_ptr<IPelcoTransport> transport_;
    std::unique_ptr<PelcoDSession>   session_;

    mutable std::mutex ioMutex_;

    std::thread             worker_;
    std::mutex              waitMutex_;
    std::condition_variable waitCv_;
    std::atomic<bool>       running_{false};
    std::atomic<bool>       initialized_{false};

    std::function<void(const std::string&)> diagnosticSink_;
    std::function<std::uint64_t()>          clock_;

    // 仅 worker 线程访问：诊断去重，避免每 50 ms 重复同一条错误。
    std::string lastReportedError_;
};

}  // namespace device
}  // namespace aircraft
