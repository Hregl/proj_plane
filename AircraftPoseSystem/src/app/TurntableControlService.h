#pragma once

// ============================================================================
//  src/app/TurntableControlService.h
//
//  作用：主程序里**操作员定位/停止入口**的协调层。
//
//  ⚠ 这不是第二套控制器，也不发协议帧：
//    · 真正的发送、门禁、重试、停止屏障仍在 `ITurntableController`
//      的实现里（`PelcoDTurntableController` / `PelcoDSession` /
//      `VirtualTurntable`）；
//    · 本类只做四件事：
//        1. 在调用 `move()` 前做即时预检（机械行程/新鲜样本/许可范围/
//           位移），让明显不合法的输入不进入命令槽；
//        2. 把“受理 / 交付 / 反馈到位”三个阶段从既有诊断中拆开呈现；
//        3. 在一条命令结束前拒绝第二条命令（连续操作不混命令）；
//        4. 在界面层超时或故障时按**现有**停止入口收口，不重试、不补发、
//           不自动回零。
//
//  ⚠ 线程约定：
//    · `requestMove()` / `poll()` / `view()` 由 GUI 线程调用；
//    · `requestStop()` 把 `controller.stop()` 放到独立线程执行，避免
//      停止屏障的有界等待（`stop_retry_window_ms + 1000 ms`）冻结界面。
//      停止请求本身在 `stop()` 入口立即登记，因此“已发出”不依赖线程
//      何时被调度；
//    · 析构/`shutdown()` 必须 join 该线程，且必须先于控制器销毁。
// ============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "data/TurntableConfig.h"
#include "device/turntable/ITurntableController.h"
#include "device/turntable/ITurntableDiagnostics.h"
#include "ui/TurntableControlView.h"

namespace aircraft
{
namespace app
{

class TurntableControlService
{
public:
    /// @param controller   已初始化的转台控制器（所有权不在本类）。
    /// @param config       该转台的配置（机械行程、到位容差、速度）。
    /// @param diagnostics  并列诊断接口；虚拟后端可为 nullptr。
    TurntableControlService(device::ITurntableController& controller,
                            const data::TurntableConfig& config,
                            device::ITurntableDiagnostics* diagnostics);
    ~TurntableControlService();

    TurntableControlService(const TurntableControlService&) = delete;
    TurntableControlService& operator=(const TurntableControlService&) = delete;

    /// 请求一次绝对定位。
    ///
    /// @return false 表示**未受理**，原因写入 `error`；此时不会调用
    ///         `move()`，也不会消耗验收许可次数。
    ///         true 仅表示 `move()` 已受理（命令进入命令槽），不表示
    ///         设备已收到或已到位。
    bool requestMove(double azimuthDeg, double elevationDeg,
                     std::string& error);

    /// 请求停止（异步执行 `controller.stop()`，内部有界等待）。
    /// 重复请求在停止尚未完成时忽略。
    void requestStop();

    /// 推进本类的阶段判断与超时检查。由 app 的 tick 调用（GUI 线程）。
    void poll();

    /// 停止线程 join；幂等。必须在控制器销毁之前调用。
    void shutdown();

    /// 是否有未结束的定位命令或停止请求。
    bool busy() const;

    /// 生成界面快照。
    /// @param measurementActive 测量任务是否正在运行（运行中禁止手动定位）。
    ui::TurntableControlView view(bool measurementActive = false) const;

private:
    bool precheckLocked(double azimuthDeg, double elevationDeg,
                        std::string& error) const;
    void updateCommandLocked(bool& needStop);
    void updateStopLocked();
    static bool withinTolerance(double az, double el, double targetAz,
                                double targetEl, double toleranceDeg);
    static double circularDeltaDeg(double a, double b);

    device::ITurntableController& controller_;
    const data::TurntableConfig   config_;
    device::ITurntableDiagnostics* diagnostics_ = nullptr;

    mutable std::mutex mutex_;

    // ---- 定位命令 ----
    bool        commandActive_ = false;
    bool        arrived_       = false;
    bool        leftTolerance_ = false;
    bool        hasTarget_     = false;
    double      targetAz_      = 0.0;
    double      targetEl_      = 0.0;
    std::uint64_t acceptedSeq_ = 0;
    std::chrono::steady_clock::time_point commandDeadline_{};

    ui::TurntableCommandStage commandStage_ =
        ui::TurntableCommandStage::None;
    std::string commandStageText_;
    std::string commandDetail_;

    // ---- 停止 ----
    bool                    stopRequested_ = false;
    std::atomic<bool>       stopInFlight_{false};
    std::thread             stopThread_;
    ui::TurntableStopStage  stopStage_ = ui::TurntableStopStage::None;
    std::string             stopStageText_;
    std::string             stopDetail_;

    // ---- 操作反馈 ----
    std::string lastAction_;
};

}  // namespace app
}  // namespace aircraft
