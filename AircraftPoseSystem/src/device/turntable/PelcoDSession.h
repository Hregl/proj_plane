#pragma once

// ============================================================================
//  src/device/turntable/PelcoDSession.h
//
//  依据：SYS-10 V1.1 §6.3～§6.6、ENG-09 V2.7 §6.3、裁决 C-22
//
//  作用：Pelco-D 的**协议与会话状态**：查询、移动回显、重试、STABLE 判据、
//  看门狗与错误锁存。**不持有线程、不做 socket**：真实后端的 I/O 线程
//  在 `PelcoDTurntableController` 里，测试可以直接调用 `pollOnce()`。
//
//  ⚠ 线程/锁约定（必须遵守，否则 state() 会阻塞主 tick）：
//    · `initialize()` / `move()` / `pollOnce()` / `stop()` 由调用方在
//      **同一把锁**（控制器的 `ioMutex_`）内串行调用；
//    · `snapshot()` **不加锁、不做 I/O**，只读原子快照；
//    · `lastError()` / `lastErrorText()` 只在持有控制器 `ioMutex_` 时调用。
//
//  ⚠ 本批**不发送** 0xC5/0xDD/0xE1，也不把它们的回传用于位置或稳定判据：
//    · 0xC5 回的是**指定目标角**，不是实测角；
//    · 0xDD 的刹车标志在角度仍变化时也可能置位；
//    · 0xE1 实时角度流本批不开启。
// ============================================================================

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "data/ErrorInfo.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableState.h"
#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/PelcoDFrame.h"

namespace aircraft
{
namespace device
{

class PelcoDSession
{
public:
    /// @param transport 不允许为空；控制器在构造前创建。
    PelcoDSession(const data::TurntableConfig& config,
                  std::shared_ptr<IPelcoTransport> transport);

    PelcoDSession(const PelcoDSession&) = delete;
    PelcoDSession& operator=(const PelcoDSession&) = delete;

    /// 注入单调时钟（测试用）。默认使用 data::monotonicNowNs()。
    void setClock(std::function<std::uint64_t()> clock);

    /// 打开传输、查询两轴角度与工作模式。
    /// 失败时关闭传输、锁存错误，返回 false。
    bool initialize(std::string& error);

    /// 下发绝对定位（两轴）。**不阻塞到到位**，只等设备回显。
    bool move(const data::TurntableCommand& command, std::string& error);

    /// 执行一个轮询周期：查询两轴、更新快照、推进 STABLE/看门狗。
    void pollOnce();

    /// 尽力停止。仅在 MOVING 或有锁存错误时发送，且每次错误/动作只发一次。
    /// 不清除已锁存的 ERROR（清除只能由新的 initialize() 完成）。
    void stop();

    /// 无线程安全的只读快照（原子读）。
    data::TurntableState snapshot() const;

    /// 最近一次错误（仅在持有 ioMutex_ 时调用）。
    data::ErrorInfo lastError() const;
    std::string    lastErrorText() const;

    bool errorLatched() const { return errorLatched_; }
    bool initialized() const { return initialized_; }

private:
    // ---- 单次收发的辅助 ----
    std::uint64_t nowNs() const;
    bool sendFrame(const PelcoDFrame& frame, std::string& error);
    bool waitForEcho(const PelcoDFrame& sent, std::string& error, bool& rejected);
    bool sendAndAwaitEcho(const PelcoDFrame& frame, std::string& error,
                          bool& rejected);
    /// @param retries 额外重试次数（总尝试 = retries + 1）。初始化用 2，
    ///                轮询用 0 —— 轮询的下一个周期本身就是重试。
    bool queryAngle(bool horizontal, double& out, std::string& error,
                    int retries);
    bool queryWorkMode(std::uint8_t& out, std::string& error, int retries);

    // ---- 状态维护 ----
    void setError(int code, const std::string& text, bool latch);
    void bestEffortStop();
    void updateMotion(double az, double el);
    bool resolveAzimuth(double command, double& resolved) const;
    bool resolveElevation(double command, double& resolved) const;
    double selectedSpeedDegPerSec(double maxDeltaDeg) const;
    void armWatchdog(double targetAz, double targetEl);

    data::TurntableConfig            config_;
    std::shared_ptr<IPelcoTransport> transport_;
    std::function<std::uint64_t()>   clock_;

    // 协议/状态（由调用方的 ioMutex_ 串行保护）
    bool     initialized_      = false;
    bool     moving_           = false;
    bool     errorLatched_     = false;
    bool     stopSent_         = false;
    bool     hasLastRead_      = false;
    int      stableCount_      = 0;
    int      consecutiveFailures_ = 0;
    double   targetAz_         = 0.0;
    double   targetEl_         = 0.0;
    double   lastAz_           = 0.0;
    double   lastEl_           = 0.0;
    std::uint64_t moveDeadlineNs_ = 0;
    data::ErrorInfo lastError_;

    // 快照（可被 state() 无锁读取）
    std::atomic<double>                    azimuth_{0.0};
    std::atomic<double>                    elevation_{0.0};
    std::atomic<data::TurntableMotionState> motion_{
        data::TurntableMotionState::IDLE};
};

}  // namespace device
}  // namespace aircraft
