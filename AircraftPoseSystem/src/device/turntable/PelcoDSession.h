#pragma once

// ============================================================================
//  src/device/turntable/PelcoDSession.h
//
//  依据：SYS-10 V1.1 §6.3～§6.6、§12/§13、ENG-09 V2.7 §6.3、裁决 C-22/C-23
//
//  作用：Pelco-D 的**协议与会话状态**：查询、移动回显、重试、STABLE 判据、
//  看门狗、停止语义与错误锁存。**不持有线程、不做 socket**：真实后端的
//  I/O 线程在 `PelcoDTurntableController` 里，测试可以直接调用 `pollOnce()`。
//
//  ⚠ 线程/锁约定：
//    · 所有**变更**方法（initialize/move/pollOnce/stop）必须由**同一个
//      I/O 所有者**串行调用（控制器 worker；initialize 在 worker 启动前）；
//    · `snapshot()` / `diagnostics()` **不做 I/O**，只读原子量 + 短暂锁
//      （仅用于首因诊断文本），可在主 tick 里调用；
//    · `lastError()` / `lastErrorText()` 可跨线程调用（内部短锁）。
//
//  ⚠ 本批不发送 0xC5/0xDD/0xE1，也不把它们的回传用于位置或稳定判据：
//    · 0xC5 回的是**指定目标角**，不是实测角；
//    · 0xDD 的刹车标志在角度仍变化时也可能置位；
//    · 0xE1 实时角度流本批不开启。
//
//  ⚠ 只读门禁（C1）：`sendPolicy == ReadOnly` 时，**所有路径**（初始化、
//    轮询、失联、错误锁存、退出、stop()）都不得发送控制帧；stop() 是
//    无帧 no-op。本类不提供任何能把门禁改成 MotionEnabled 的方法。
// ============================================================================

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "data/ErrorInfo.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableDiagnostics.h"
#include "data/TurntableSample.h"
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

    /// 打开传输、查询两轴角度与工作模式；两轴必须在同一事务内成功，
    /// 且工作模式回复的**地址**必须与配置一致（C3）。
    /// 失败时关闭传输、锁存错误，返回 false。
    /// 成功时清锁存、清 motionUncertain —— 这是唯一的重臂点。
    bool initialize(std::string& error);

    /// 下发绝对定位（两轴）。**在 worker 线程内**等待回显（≤10 ms 切片）。
    /// @param shouldAbort 停止屏障回调；返回 true 时放弃等待且不恢复旧命令。
    bool move(const data::TurntableCommand& command, std::string& error);
    bool move(const data::TurntableCommand& command, std::string& error,
              const std::function<bool()>& shouldAbort);

    /// 执行一个轮询周期：查询两轴（**同一轮次**）、更新快照、推进
    /// STABLE/看门狗与停止后静止观测。
    void pollOnce();

    /// 显式停止：允许重新武装停止重试。ReadOnly 下为无帧 no-op。
    void stop();

    /// 故障收口使用的停止：保留原始首因，记录停止结果；停止未交付/
    /// 未观测到静止时升级为 2005/2006 并锁存 motionUncertain。
    void stopForFault(int faultCode, const std::string& message,
                      const std::string& context);

    /// 无线程安全的只读快照（原子读）。样本过期或无效时 motion 报 ERROR。
    data::TurntableState snapshot() const;

    /// 诊断快照（不做 I/O）。
    data::TurntableDiagnostics diagnostics() const;

    /// 最近一次错误（短锁）。
    data::ErrorInfo lastError() const;
    std::string    lastErrorText() const;

    bool errorLatched() const { return errorLatched_.load(); }
    bool initialized() const { return initialized_.load(); }

    /// 运动门禁是否正在拒绝运动（供上层诊断/测试）。
    bool motionInhibited() const;

    /// ⚠ 单元测试专用：允许 MotionEnabled 会话在 fake transport 上执行
    /// 运动指令，以验证停止/故障路径。生产装配从调用点、配置解析和
    /// 默认值三处都不可达。**不改变 WorkMode 可信度**。
    void setTestMotionGate(bool allow) { testMotionGate_ = allow; }

private:
    // ---- 单次收发的辅助 ----
    std::uint64_t nowNs() const;
    bool sendFrame(const PelcoDFrame& frame, std::string& error);
    bool waitForEcho(const PelcoDFrame& sent, std::string& error, bool& rejected,
                     const std::function<bool()>& shouldAbort);
    bool sendAndAwaitEcho(const PelcoDFrame& frame, std::string& error,
                          bool& rejected,
                          const std::function<bool()>& shouldAbort);
    /// 地址错误的回复按“没有有效回复”处理（C3）。
    bool queryAngle(bool horizontal, double& out, std::string& error,
                    int retries);
    /// @param echoOnly 收到的回复与查询帧逐字节相同（回显歧义）。
    bool queryWorkMode(std::uint8_t& out, bool& echoOnly, std::string& error,
                       int retries);

    // ---- 状态维护 ----
    void setError(int code, const std::string& text, bool latch,
                  const std::string& context = std::string());
    void setFirstFault(int code, const std::string& text,
                       const std::string& context);
    void publishSample(double az, double el, std::uint64_t now, bool valid);
    void checkStaleLocked() const;
    bool withinTolerance(double az, double el) const;
    void updateMotion(double az, double el);
    void handleStopObservation(double az, double el, std::uint64_t now);
    bool performStop(int faultCode, const std::string& faultMessage,
                     const std::string& context, bool recordFirstFault);
    bool resolveAzimuth(double command, double& resolved) const;
    bool resolveElevation(double command, double& resolved) const;
    double selectedSpeedDegPerSec(double maxDeltaDeg) const;
    void armWatchdog(double targetAz, double targetEl);
    bool finiteConfig(std::string& error) const;

    data::TurntableConfig            config_;
    std::shared_ptr<IPelcoTransport> transport_;
    std::function<std::uint64_t()>   clock_;
    bool                             testMotionGate_ = false;

    // ---- I/O 所有者线程独占的状态 ----
    bool     moving_              = false;
    bool     hasLastRead_         = false;
    int      stableCount_         = 0;
    int      consecutiveFailures_ = 0;
    double   targetAz_            = 0.0;
    double   targetEl_            = 0.0;
    double   lastAz_              = 0.0;
    double   lastEl_              = 0.0;
    std::uint64_t moveDeadlineNs_ = 0;

    // 停止状态机（I/O 所有者线程独占）
    bool     stopPending_              = false;
    bool     stopSent_                 = false;
    bool     stopRetryArmed_           = false;
    int      stopStationaryCount_      = 0;
    bool     stopHasLast_              = false;
    double   stopLastAz_               = 0.0;
    double   stopLastEl_               = 0.0;
    std::uint64_t stopObservationDeadlineNs_ = 0;

    // ---- 跨线程诊断/错误（短锁）----
    mutable std::mutex faultMutex_;
    data::ErrorInfo    lastError_;
    data::TurntableFault firstFault_;

    // ---- 快照（无锁原子量）----
    std::atomic<bool>     initialized_{false};
    std::atomic<bool>     errorLatched_{false};
    std::atomic<double>   azimuth_{0.0};
    std::atomic<double>   elevation_{0.0};
    std::atomic<std::uint64_t> sampleTimestampNs_{0};
    std::atomic<std::uint64_t> sampleCycleId_{0};
    std::atomic<bool>     sampleValid_{false};
    std::atomic<data::TurntableMotionState> motion_{
        data::TurntableMotionState::IDLE};
    std::atomic<data::WorkModeTrust> workModeTrust_{
        data::WorkModeTrust::Unknown};
    std::atomic<bool> stopFrameDelivered_{false};
    std::atomic<bool> stopObservedStationary_{false};
    std::atomic<bool> motionUncertain_{false};
};

}  // namespace device
}  // namespace aircraft
