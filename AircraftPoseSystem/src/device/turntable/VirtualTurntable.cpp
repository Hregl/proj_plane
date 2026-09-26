// ============================================================================
//  src/device/turntable/VirtualTurntable.cpp
//
//  依据：SYS-06 §12、SYS-08 §7.7（能力边界不重试）、ENG-09 §5.9 / §6.3
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "device/turntable/VirtualTurntable.h"

#include <algorithm>
#include <cmath>

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// 到达目标附近的容差，单位 deg。
/// 转台实际无法精确停在某个角度上，总会有一个由编码器分辨率与
/// 机械回差决定的残差。虚拟转台若精确到位，则依赖"角度是否完全相等"
/// 的代码在真机上才会第一次暴露问题。
///
/// 0.001 deg = 3.6 角秒，远小于 1 角分的验收指标（60 角秒），
/// 故不会掩盖指标级的问题，但足以让"精确相等比较"失效 ——
/// 这正是此处想要暴露的。
constexpr double kAngleEpsilonDeg = 0.001;

/// 由角度差与速度计算行程时间，返回 ns。
uint64_t travelTimeNs(double deltaDeg, double speedDegPerSec)
{
    if (speedDegPerSec <= 0.0)
    {
        // 速度为 0 或负：配置错误。返回 0 会表现为"瞬间到位"，
        // 把配置缺陷伪装成正常行为；返回一个极大值又会让调用方挂死。
        // 此处返回 0 并由调用方（move()）另作检查，见 move() 中的说明。
        return 0;
    }
    const double seconds = std::fabs(deltaDeg) / speedDegPerSec;
    return static_cast<uint64_t>(seconds * 1e9);
}
}  // namespace

VirtualTurntable::VirtualTurntable(const data::TurntableConfig& config)
    : config_(config)
{
}

void VirtualTurntable::setClock(std::function<uint64_t()> clock)
{
    clock_ = std::move(clock);
}

uint64_t VirtualTurntable::nowNs() const
{
    return clock_ ? clock_() : data::monotonicNowNs();
}

bool VirtualTurntable::initialize()
{
    lastError_ = data::ErrorInfo{};

    // 校验配置自洽性。此处检查是必要的，因为虚拟转台是**第一阶段唯一
    // 可用的转台**，若配置有误（如上下限颠倒、速度为 0）而此处不报，
    // 则错误会在第一次 move() 时以"超行程"或"瞬间到位"的形式出现，
    // 排查方向会被引向对准算法而非配置。
    if (config_.azimuthMax <= config_.azimuthMin ||
        config_.elevationMax <= config_.elevationMin)
    {
        motion_                = data::TurntableMotionState::ERROR;
        lastError_.code        = data::kErrTurntableComm;
        lastError_.message     = "转台行程配置无效（上限不大于下限）";
        lastError_.timestampNs = nowNs();
        return false;
    }

    if (config_.coarseSpeed <= 0.0 || config_.fineSpeed <= 0.0)
    {
        motion_                = data::TurntableMotionState::ERROR;
        lastError_.code        = data::kErrTurntableComm;
        lastError_.message     = "转台速度配置无效（粗调或精调速度 <= 0）";
        lastError_.timestampNs = nowNs();
        return false;
    }

    // 初始角度取行程中点：这是一个必然在限位内的值，
    // 使对象在 initialize() 后立刻可 move()，而不必先摆位。
    azimuth_   = (config_.azimuthMin + config_.azimuthMax) / 2.0;
    elevation_ = (config_.elevationMin + config_.elevationMax) / 2.0;

    motion_      = data::TurntableMotionState::IDLE;
    initialized_ = true;
    return true;
}

bool VirtualTurntable::move(const data::TurntableCommand& command)
{
    lastError_ = data::ErrorInfo{};

    if (!initialized_)
    {
        lastError_.code        = data::kErrTurntableComm;
        lastError_.message     = "转台未初始化即收到移动命令";
        lastError_.timestampNs = nowNs();
        return false;
    }

    // ---- 行程限位检查（SYS-08 §7.7 能力边界，不重试）----
    //
    // ⚠ 必须在**命令值**上检查，而不是在运动结束后检查：
    //   运动结束后才发现超限意味着转台已经撞到机械限位，
    //   这在真机上可能造成损伤。命令下发前拒绝是唯一安全的时机。
    if (command.azimuthCommand < config_.azimuthMin ||
        command.azimuthCommand > config_.azimuthMax)
    {
        motion_                = data::TurntableMotionState::ERROR;
        lastError_.code        = data::kErrTurntableOverTravel;  // 2002
        lastError_.message     = "方位角命令超出行程（能力边界，不重试）";
        lastError_.timestampNs = nowNs();
        return false;
    }

    if (command.elevationCommand < config_.elevationMin ||
        command.elevationCommand > config_.elevationMax)
    {
        motion_                = data::TurntableMotionState::ERROR;
        lastError_.code        = data::kErrTurntableOverTravel;  // 2002
        lastError_.message     = "俯仰角命令超出行程（能力边界，不重试）";
        lastError_.timestampNs = nowNs();
        return false;
    }

    const double dAz = command.azimuthCommand - azimuth_;
    const double dEl = command.elevationCommand - elevation_;

    // 已到位（在容差内）：不进入 MOVING。
    // 若此处仍置 MOVING 并等待一个 0 长度的行程，则 state() 会在
    // 下一次轮询才转为 STABLE，使 STABILIZE 状态平白多等一个周期；
    // 更麻烦的是"命令值等于当前值"这一常见情形（对准已收敛时
    // 仍会下发一次命令）会被误报成"转台在动"。
    if (std::fabs(dAz) < kAngleEpsilonDeg && std::fabs(dEl) < kAngleEpsilonDeg)
    {
        azimuth_   = command.azimuthCommand;
        elevation_ = command.elevationCommand;
        motion_    = data::TurntableMotionState::STABLE;
        return true;
    }

    // 速度选择：大角度用粗调，小角度用精调。
    // 阈值取常量 1.0 deg —— 这是一个**虚拟设备的模拟参数**，
    // 不是系统级配置项。理由：真实转台由自身控制器决定加减速曲线，
    // 软件只下发目标角度；把"何时切换粗/精调"做成可配置项，
    // 会诱导实现者以为真实转台也由软件分段控制，而 SYS-10 §6 的
    // 真实协议不含该能力。故此处不做成配置，只用于让虚拟运动
    // 的时间量级接近真机。
    constexpr double kCoarseThresholdDeg = 1.0;
    const double maxDelta = std::max(std::fabs(dAz), std::fabs(dEl));
    const double speed    = maxDelta > kCoarseThresholdDeg ? config_.coarseSpeed
                                                           : config_.fineSpeed;

    const uint64_t travelNs = travelTimeNs(maxDelta, speed);

    targetAzimuth_   = command.azimuthCommand;
    targetElevation_ = command.elevationCommand;
    arrivalNs_       = nowNs() + travelNs;
    motion_          = data::TurntableMotionState::MOVING;

    return true;
}

data::TurntableState VirtualTurntable::state()
{
    // 在查询时推进运动状态，而不是靠独立的定时器线程。
    // 理由：虚拟设备不需要真实的时间驱动 —— 它只需要"被观察时
    // 表现出与真实设备一致的时序"。用定时器线程反而引入了一个
    // 只有测试才会遇到的并发问题（定时器与轮询的竞态），
    // 而真机上并不存在这个线程。
    if (motion_ == data::TurntableMotionState::MOVING && nowNs() >= arrivalNs_)
    {
        azimuth_   = targetAzimuth_;
        elevation_ = targetElevation_;
        motion_    = data::TurntableMotionState::STABLE;
    }

    data::TurntableState s;
    s.azimuth   = azimuth_;    // deg
    s.elevation = elevation_;  // deg
    s.motion    = motion_;
    return s;
}

void VirtualTurntable::stop()
{
    // 停在当前位置，而不是停在命令目标上 —— 这是"停止"的正确语义，
    // 也是 SYS-08 §7.3 人工取消（9003）时期望的行为：
    // 转台就地停下，便于人工检查，而不是先冲到一个未到达的目标。
    if (motion_ == data::TurntableMotionState::MOVING)
    {
        motion_ = data::TurntableMotionState::IDLE;
    }

    // ERROR 状态不被 stop() 清除：故障需要显式处理（重新 initialize()
    // 或由上层判定失败），一次 stop() 不应把故障抹掉 ——
    // 否则 2002 超行程的记录会在任务收尾的 stop() 调用中消失。
}

void VirtualTurntable::setInitialAngles(double azimuth, double elevation)
{
    azimuth_   = azimuth;    // deg
    elevation_ = elevation;  // deg

    // 位置一旦被直接设定，进行中的运动即失效 ——
    // 否则到达时刻一到，state() 会把角度覆盖回之前的目标值，
    // 使 setInitialAngles() 的效果在下一次查询时被静默撤销。
    if (motion_ == data::TurntableMotionState::MOVING)
    {
        motion_ = data::TurntableMotionState::IDLE;
    }
}

data::ErrorInfo VirtualTurntable::lastError() const
{
    return lastError_;
}

}  // namespace device
}  // namespace aircraft
