// ============================================================================
//  src/app/TurntableControlService.cpp
//
//  见头文件说明。
// ============================================================================

#include "app/TurntableControlService.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <system_error>

#include "data/TurntableCommand.h"
#include "data/TurntableDiagnostics.h"
#include "data/TurntableMotionState.h"
#include "device/turntable/PelcoDTurntableController.h"

namespace aircraft
{
namespace app
{

namespace
{

const char* commandStageName(ui::TurntableCommandStage stage)
{
    switch (stage)
    {
    case ui::TurntableCommandStage::None:        return "未命令";
    case ui::TurntableCommandStage::Accepted:    return "请求已受理";
    case ui::TurntableCommandStage::Delivered:   return "指令已交付";
    case ui::TurntableCommandStage::Arrived:     return "反馈已到位";
    case ui::TurntableCommandStage::Rejected:    return "未受理";
    case ui::TurntableCommandStage::Failed:      return "失败";
    case ui::TurntableCommandStage::Superseded:  return "被新命令覆盖";
    case ui::TurntableCommandStage::Cancelled:   return "被停止取消";
    case ui::TurntableCommandStage::Timeout:     return "未在期限内到位";
    case ui::TurntableCommandStage::LeftTolerance:
        return "已离开到位容差";
    }
    return "?";
}

const char* stopStageName(ui::TurntableStopStage stage)
{
    switch (stage)
    {
    case ui::TurntableStopStage::None:               return "未请求停止";
    case ui::TurntableStopStage::Requested:          return "停止请求已发出";
    case ui::TurntableStopStage::FrameDelivered:     return "停止帧已交付";
    case ui::TurntableStopStage::ObservedStationary: return "已观测到静止";
    case ui::TurntableStopStage::Uncertain:          return "运动不确定";
    case ui::TurntableStopStage::NoFrame:            return "未发送停止帧";
    }
    return "?";
}

const char* motionName(data::TurntableMotionState motion)
{
    switch (motion)
    {
    case data::TurntableMotionState::IDLE:   return "IDLE";
    case data::TurntableMotionState::MOVING: return "MOVING";
    case data::TurntableMotionState::STABLE: return "STABLE";
    case data::TurntableMotionState::ERROR:  return "ERROR";
    }
    return "?";
}

std::string faultText(const data::TurntableDiagnostics& d)
{
    if (d.firstFault.code == 0 && d.firstFault.message.empty())
    {
        return std::string();
    }
    std::string text;
    if (d.firstFault.code != 0)
    {
        text += std::to_string(d.firstFault.code);
        if (!d.firstFault.message.empty())
        {
            text += " ";
        }
    }
    text += d.firstFault.message;
    return text;
}

}  // namespace

TurntableControlService::TurntableControlService(
    device::ITurntableController& controller,
    const data::TurntableConfig& config,
    device::ITurntableDiagnostics* diagnostics)
    : controller_(controller), config_(config), diagnostics_(diagnostics)
{
}

TurntableControlService::~TurntableControlService()
{
    shutdown();
}

double TurntableControlService::circularDeltaDeg(double a, double b)
{
    double d = std::fmod(a - b + 180.0, 360.0);
    if (d < 0.0)
    {
        d += 360.0;
    }
    return std::fabs(d - 180.0);
}

bool TurntableControlService::withinTolerance(double az, double el,
                                              double targetAz,
                                              double targetEl,
                                              double toleranceDeg)
{
    return circularDeltaDeg(az, targetAz) <= toleranceDeg &&
           std::fabs(el - targetEl) <= toleranceDeg;
}

bool TurntableControlService::precheckLocked(double azimuthDeg,
                                             double elevationDeg,
                                             std::string& error) const
{
    if (!std::isfinite(azimuthDeg) || !std::isfinite(elevationDeg))
    {
        error = "目标角必须是有限数";
        return false;
    }
    if (stopInFlight_.load())
    {
        error = "停止尚未完成，拒绝新的定位";
        return false;
    }
    if (commandActive_)
    {
        error = "上一条定位尚未结束，拒绝新的定位";
        return false;
    }
    if (azimuthDeg < config_.azimuthMin ||
        azimuthDeg > config_.azimuthMax)
    {
        error = "目标方位角超出机械行程";
        return false;
    }
    if (elevationDeg < config_.elevationMin ||
        elevationDeg > config_.elevationMax)
    {
        error = "目标俯仰角超出机械行程";
        return false;
    }

    const data::TurntableState state = controller_.state();
    if (state.motion == data::TurntableMotionState::ERROR)
    {
        error = "转台处于 ERROR 状态，拒绝定位";
        return false;
    }

    if (diagnostics_ == nullptr)
    {
        // 虚拟后端没有并列诊断：状态本身就是唯一判据。
        if (state.motion == data::TurntableMotionState::MOVING)
        {
            error = "转台正在运动（可能有其他命令），拒绝新的定位";
            return false;
        }
        return true;
    }

    const data::TurntableDiagnostics d = diagnostics_->diagnostics();
    if (d.errorLatched || d.motionUncertain)
    {
        error = "转台故障锁存或运动不确定，拒绝定位";
        return false;
    }
    if (!d.sample.valid || !d.sampleFresh)
    {
        error = "无新鲜有效双轴样本，拒绝定位（无法核对当前位移）";
        return false;
    }
    if (d.motionInhibited)
    {
        error = "运动门禁拒绝：无有效验收许可、许可过期/次数用尽或只读模式";
        return false;
    }
    if (state.motion == data::TurntableMotionState::MOVING)
    {
        error = "转台正在运动（可能有其他命令），拒绝新的定位";
        return false;
    }

    if (d.acceptancePermitInstalled)
    {
        if (!d.acceptancePermitHasAbsolutePosition)
        {
            error = "验收许可未授权绝对定位";
            return false;
        }
        const double tol = 1e-9;
        if (azimuthDeg < d.acceptancePermitAzimuthMinDeg - tol ||
            azimuthDeg > d.acceptancePermitAzimuthMaxDeg + tol)
        {
            error = "目标方位角超出验收许可范围";
            return false;
        }
        if (elevationDeg < d.acceptancePermitElevationMinDeg - tol ||
            elevationDeg > d.acceptancePermitElevationMaxDeg + tol)
        {
            error = "目标俯仰角超出验收许可范围";
            return false;
        }
        const double azDelta =
            circularDeltaDeg(azimuthDeg, d.sample.azimuth);
        const double elDelta =
            std::fabs(elevationDeg - d.sample.elevation);
        if (azDelta > d.acceptancePermitMaxDisplacementDeg + tol ||
            elDelta > d.acceptancePermitMaxDisplacementDeg + tol)
        {
            error = "目标相对当前实测角的位移超出验收许可";
            return false;
        }
    }
    return true;
}

bool TurntableControlService::requestMove(double azimuthDeg,
                                          double elevationDeg,
                                          std::string& error)
{
    std::lock_guard<std::mutex> lock(mutex_);

    error.clear();
    if (!precheckLocked(azimuthDeg, elevationDeg, error))
    {
        commandStage_ = ui::TurntableCommandStage::Rejected;
        commandStageText_ = commandStageName(commandStage_);
        commandDetail_ = error;
        lastAction_ = "定位未受理：" + error;
        return false;
    }

    data::TurntableCommand command;
    command.azimuthCommand = azimuthDeg;
    command.elevationCommand = elevationDeg;

    const data::TurntableState before = controller_.state();
    if (!controller_.move(command))
    {
        std::string detail = "转台拒绝定位命令（门禁或发送失败）";
        if (diagnostics_ != nullptr)
        {
            const data::TurntableDiagnostics d = diagnostics_->diagnostics();
            if (!d.lastCommandMessage.empty())
            {
                detail = d.lastCommandMessage;
            }
        }
        commandStage_ = ui::TurntableCommandStage::Rejected;
        commandStageText_ = commandStageName(commandStage_);
        commandDetail_ = detail;
        error = detail;
        lastAction_ = "定位未受理：" + detail;
        return false;
    }

    acceptedSeq_ = 0;
    if (auto* pelco =
            dynamic_cast<device::PelcoDTurntableController*>(&controller_))
    {
        acceptedSeq_ = pelco->lastAcceptedCommandSeq();
    }

    // 新命令开始时，上一次停止结果不再对应当前运动。不清掉会出现
    // “已观测到静止”或（事务号变化后）“未发送停止帧”被误读为本条
    // 定位的停止状态。
    stopRequested_ = false;
    stopStage_ = ui::TurntableStopStage::None;
    stopStageText_ = stopStageName(stopStage_);
    stopDetail_.clear();

    commandActive_ = true;
    arrived_       = false;
    leftTolerance_ = false;
    hasTarget_     = true;
    targetAz_      = azimuthDeg;
    targetEl_      = elevationDeg;
    commandStage_  = ui::TurntableCommandStage::Accepted;
    commandStageText_ = commandStageName(commandStage_);
    commandDetail_ = "命令已进入命令槽，等待发送/回显";

    const double travel =
        std::max(circularDeltaDeg(azimuthDeg, before.azimuth),
                 std::fabs(elevationDeg - before.elevation));
    double speed = config_.fineSpeed;
    if (config_.useSpeedControl)
    {
        speed = travel > 1.0 ? config_.coarseSpeed : config_.fineSpeed;
    }
    if (!std::isfinite(speed) || speed <= 0.0)
    {
        speed = 1.2;   // 与生产会话的缺省速度量级一致，仅用于界面超时
    }
    double seconds = travel / speed * 1.5 + 5.0 + 2.0;
    if (!std::isfinite(seconds) || seconds < 1.0)
    {
        seconds = 1.0;
    }
    if (seconds > 600.0)
    {
        seconds = 600.0;
    }
    commandDeadline_ = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(
            static_cast<std::int64_t>(seconds * 1000.0));

    lastAction_ = "定位请求已受理（不表示设备已收到或已到位）";
    return true;
}

void TurntableControlService::requestStop()
{
    // 上一次停止线程已结束则回收；只有 GUI 线程访问 stopThread_。
    if (stopThread_.joinable() && !stopInFlight_.load())
    {
        stopThread_.join();
    }

    // 虚拟转台没有诊断接口，stop() 是就地状态切换、不涉及 I/O；
    // 同步执行可避免与 view()/state() 并发访问其非线程安全状态。
    const bool synchronous = diagnostics_ == nullptr;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopInFlight_.load())
        {
            return;   // 停止请求在途，重复点击不叠加
        }
        stopRequested_ = true;
        stopInFlight_.store(!synchronous);
        stopStage_ = ui::TurntableStopStage::Requested;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ = "停止请求已交给控制器，等待处理结果";
        if (commandActive_)
        {
            commandActive_ = false;
            commandStage_ = ui::TurntableCommandStage::Cancelled;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ = "定位命令被停止请求取消";
        }
        lastAction_ = "停止请求已发出（不表示设备已停止）";
    }

    if (synchronous)
    {
        controller_.stop();
        return;
    }

    try
    {
        stopThread_ = std::thread([this]() {
            controller_.stop();
            stopInFlight_.store(false);
        });
    }
    catch (const std::system_error&)
    {
        // 线程起不来时退化为同步停止：宁可界面有界卡顿，也不能让
        // “停止”按钮看起来发出而实际没有进入控制器。
        controller_.stop();
        stopInFlight_.store(false);
    }
}

void TurntableControlService::updateCommandLocked(bool& needStop)
{
    if (!commandActive_ && !arrived_)
    {
        return;
    }

    const data::TurntableState state = controller_.state();

    if (diagnostics_ != nullptr)
    {
        const data::TurntableDiagnostics d = diagnostics_->diagnostics();
        if (d.errorLatched || d.motionUncertain)
        {
            commandActive_ = false;
            commandStage_ = ui::TurntableCommandStage::Failed;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ = d.motionUncertain
                                 ? "转台运动状态不确定（停止未交付或未观测到静止）"
                                 : "转台错误已锁存";
            const std::string f = faultText(d);
            if (!f.empty())
            {
                commandDetail_ += "：" + f;
            }
            return;
        }

        if (acceptedSeq_ != 0)
        {
            if (d.lastCommandSeq == acceptedSeq_)
            {
                switch (d.lastCommandOutcome)
                {
                case data::TurntableCommandOutcome::Failed:
                    commandActive_ = false;
                    commandStage_ = ui::TurntableCommandStage::Failed;
                    commandStageText_ = commandStageName(commandStage_);
                    commandDetail_ = d.lastCommandMessage.empty()
                                         ? "发送/回显失败"
                                         : d.lastCommandMessage;
                    return;
                case data::TurntableCommandOutcome::Superseded:
                    commandActive_ = false;
                    commandStage_ =
                        ui::TurntableCommandStage::Superseded;
                    commandStageText_ = commandStageName(commandStage_);
                    commandDetail_ = "命令被更新的运动命令覆盖";
                    return;
                case data::TurntableCommandOutcome::Cancelled:
                    commandActive_ = false;
                    commandStage_ =
                        ui::TurntableCommandStage::Cancelled;
                    commandStageText_ = commandStageName(commandStage_);
                    commandDetail_ = "命令被停止屏障取消";
                    return;
                case data::TurntableCommandOutcome::Delivered:
                    if (commandActive_)
                    {
                        commandStage_ =
                            ui::TurntableCommandStage::Delivered;
                        commandStageText_ =
                            commandStageName(commandStage_);
                        commandDetail_ = "两轴指令均已交付（收到回显），等待反馈到位";
                    }
                    break;
                case data::TurntableCommandOutcome::None:
                    break;
                }
            }
            else if (d.lastCommandSeq > acceptedSeq_)
            {
                commandActive_ = false;
                commandStage_ = ui::TurntableCommandStage::Superseded;
                commandStageText_ = commandStageName(commandStage_);
                commandDetail_ = "命令序号已被其他来源推进";
                return;
            }
        }

        const bool fresh = d.sampleFresh && d.sample.valid;
        const bool inTolerance =
            fresh && withinTolerance(d.sample.azimuth, d.sample.elevation,
                                     targetAz_, targetEl_,
                                     config_.arrivalToleranceDeg);
        if (commandActive_ && state.motion == data::TurntableMotionState::STABLE &&
            inTolerance)
        {
            commandActive_ = false;
            arrived_ = true;
            commandStage_ = ui::TurntableCommandStage::Arrived;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ =
                "新鲜双轴样本满足到位容差与 STABLE；仅设备反馈，非独立物理位置";
        }
        else if (!commandActive_ && arrived_ && !leftTolerance_ && fresh &&
                 !inTolerance)
        {
            leftTolerance_ = true;
            commandStage_ = ui::TurntableCommandStage::LeftTolerance;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ = "到位后反馈已偏离到位容差（原样显示，不重新命令）";
        }
    }
    else
    {
        // 虚拟后端：move() 受理即执行，state() 是唯一反馈。
        if (state.motion == data::TurntableMotionState::ERROR)
        {
            commandActive_ = false;
            commandStage_ = ui::TurntableCommandStage::Failed;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ = "虚拟转台进入 ERROR 状态";
            return;
        }
        if (commandActive_)
        {
            if (state.motion == data::TurntableMotionState::STABLE &&
                withinTolerance(state.azimuth, state.elevation,
                                targetAz_, targetEl_,
                                config_.arrivalToleranceDeg))
            {
                commandActive_ = false;
                arrived_ = true;
                commandStage_ = ui::TurntableCommandStage::Arrived;
                commandStageText_ = commandStageName(commandStage_);
                commandDetail_ =
                    "虚拟转台反馈满足到位容差与 STABLE；不是实机证据";
            }
            else if (state.motion == data::TurntableMotionState::MOVING)
            {
                commandStage_ = ui::TurntableCommandStage::Delivered;
                commandStageText_ = commandStageName(commandStage_);
                commandDetail_ = "虚拟转台正在运动，等待到位";
            }
            else if (state.motion == data::TurntableMotionState::IDLE)
            {
                commandActive_ = false;
                commandStage_ = ui::TurntableCommandStage::Failed;
                commandStageText_ = commandStageName(commandStage_);
                commandDetail_ = "虚拟转台回到 IDLE，未观测到到位";
            }
        }
        else if (arrived_ && !leftTolerance_ &&
                 !withinTolerance(state.azimuth, state.elevation,
                                  targetAz_, targetEl_,
                                  config_.arrivalToleranceDeg))
        {
            leftTolerance_ = true;
            commandStage_ = ui::TurntableCommandStage::LeftTolerance;
            commandStageText_ = commandStageName(commandStage_);
            commandDetail_ = "到位后反馈已偏离到位容差（原样显示）";
        }
    }

    if (commandActive_ &&
        std::chrono::steady_clock::now() > commandDeadline_)
    {
        commandActive_ = false;
        commandStage_ = ui::TurntableCommandStage::Timeout;
        commandStageText_ = commandStageName(commandStage_);
        commandDetail_ =
            "未在界面期限内到位；按现有停止入口请求停止，不重试、不回零";
        needStop = true;
    }
}

void TurntableControlService::updateStopLocked()
{
    if (!stopRequested_)
    {
        stopStage_ = ui::TurntableStopStage::None;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_.clear();
        return;
    }

    if (stopInFlight_.load())
    {
        stopStage_ = ui::TurntableStopStage::Requested;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ = "停止请求已交给控制器，等待处理结果";
        return;
    }

    if (diagnostics_ == nullptr)
    {
        const data::TurntableState state = controller_.state();
        if (state.motion == data::TurntableMotionState::MOVING)
        {
            stopStage_ = ui::TurntableStopStage::Requested;
            stopStageText_ = stopStageName(stopStage_);
            stopDetail_ = "停止已请求，虚拟转台仍为 MOVING";
        }
        else
        {
            stopStage_ = ui::TurntableStopStage::NoFrame;
            stopStageText_ = stopStageName(stopStage_);
            stopDetail_ =
                "虚拟转台已就地停止（无协议帧；不是设备 ACK）";
        }
        return;
    }

    const data::TurntableDiagnostics d = diagnostics_->diagnostics();
    const bool factsCurrent =
        d.stopFactsTransactionId == d.motionTransactionId;

    if (d.motionUncertain)
    {
        stopStage_ = ui::TurntableStopStage::Uncertain;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ =
            "停止未交付或未在规定窗口内观测到静止；运动不确定";
        return;
    }
    if (factsCurrent && d.stopObservedStationary)
    {
        stopStage_ = ui::TurntableStopStage::ObservedStationary;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ =
            "停止后连续新鲜样本满足静止判据；不是设备 ACK 或制动认证";
        return;
    }
    if (factsCurrent && d.stopFrameDelivered)
    {
        stopStage_ = ui::TurntableStopStage::FrameDelivered;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ = "停止帧已交给传输层；尚未观测到静止";
        return;
    }
    if (d.stopRequestSeq == 0)
    {
        stopStage_ = ui::TurntableStopStage::NoFrame;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ =
            "控制器未发送停止帧（只读模式或无在途运动）";
        return;
    }
    if (d.stopCompletedSeq >= d.stopRequestSeq)
    {
        // 停止事务已完成，但当前的停止结果字段属于更早的运动事务
        // （通常是有新的运动命令把事务号推进了）。不能把旧结果冒充
        // 为当前运动的停止结论，也不能反过来说“未发送停止帧”。
        stopStage_ = ui::TurntableStopStage::NoFrame;
        stopStageText_ = stopStageName(stopStage_);
        stopDetail_ =
            "上一次停止请求已由控制器处理；当前运动事务已更新，"
            "旧停止结果不适用于本条运动（不是设备 ACK）";
        return;
    }

    stopStage_ = ui::TurntableStopStage::Requested;
    stopStageText_ = stopStageName(stopStage_);
    stopDetail_ = "停止请求已登记，等待控制器处理";
}

void TurntableControlService::poll()
{
    bool needStop = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        updateCommandLocked(needStop);
        updateStopLocked();
    }
    if (needStop)
    {
        requestStop();
    }
}

bool TurntableControlService::busy() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return commandActive_ || stopInFlight_.load();
}

void TurntableControlService::shutdown()
{
    if (stopThread_.joinable())
    {
        stopThread_.join();
    }
    stopInFlight_.store(false);
}

ui::TurntableControlView TurntableControlService::view(
    bool measurementActive) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    ui::TurntableControlView v;
    v.available = true;
    v.azimuthMin = config_.azimuthMin;
    v.azimuthMax = config_.azimuthMax;
    v.elevationMin = config_.elevationMin;
    v.elevationMax = config_.elevationMax;

    const data::TurntableState state = controller_.state();
    v.azimuth = state.azimuth;
    v.elevation = state.elevation;
    v.motionText = motionName(state.motion);

    if (diagnostics_ != nullptr)
    {
        const data::TurntableDiagnostics d = diagnostics_->diagnostics();
        v.sampleValid = d.sample.valid;
        v.sampleFresh = d.sampleFresh;
        v.hasSample = d.sample.valid && d.sample.sampleTimestampNs != 0;
        if (d.sample.valid)
        {
            v.azimuth = d.sample.azimuth;
            v.elevation = d.sample.elevation;
        }
        v.errorLatched = d.errorLatched;
        v.motionUncertain = d.motionUncertain;
        v.faultText = faultText(d);

        if (d.errorLatched || d.motionUncertain ||
            state.motion == data::TurntableMotionState::ERROR)
        {
            v.connection = ui::TurntableConnectionState::Fault;
            v.connectionText = "故障";
            v.connectionDetail = v.faultText.empty()
                                     ? "转台报 ERROR 或诊断锁存"
                                     : v.faultText;
        }
        else if (!v.hasSample)
        {
            v.connection = ui::TurntableConnectionState::WaitingSample;
            v.connectionText = "等待首轮样本";
            v.connectionDetail = "已初始化，尚未收到有效双轴回复";
        }
        else if (!d.sample.valid || !d.sampleFresh)
        {
            v.connection = ui::TurntableConnectionState::Stale;
            v.connectionText = "样本无效/过期";
            v.connectionDetail =
                "无新鲜有效双轴读数：不能据旧值定位或判到位";
        }
        else
        {
            v.connection = ui::TurntableConnectionState::Connected;
            v.connectionText = "已连接";
            v.connectionDetail = "有新鲜有效双轴样本";
        }

        v.motionEnabled = !d.motionInhibited && !d.errorLatched &&
                          !d.motionUncertain && d.sample.valid &&
                          d.sampleFresh &&
                          state.motion != data::TurntableMotionState::ERROR;

        if (!d.acceptancePermitInstalled)
        {
            v.permitText = "无验收许可：只读/运动禁用";
        }
        else
        {
            v.permitText = "许可 " + d.acceptancePermitScope;
            v.permitText += d.acceptancePermitExpired ? "（已过期）"
                                                      : "（有效）";
            v.permitText += "，剩余 " +
                            std::to_string(d.acceptancePermitRemaining) +
                            " 次";
        }
    }
    else
    {
        // 虚拟转台没有并列诊断；明确标注，避免把虚拟反馈当实机证据。
        v.connection = state.motion == data::TurntableMotionState::ERROR
                           ? ui::TurntableConnectionState::Fault
                           : ui::TurntableConnectionState::Connected;
        v.connectionText =
            state.motion == data::TurntableMotionState::ERROR
                ? "虚拟转台 ERROR"
                : "已连接（虚拟转台）";
        v.connectionDetail = "虚拟后端：不是实机链路证据";
        v.sampleValid = true;
        v.sampleFresh = true;
        v.hasSample = true;
        v.motionEnabled = state.motion != data::TurntableMotionState::ERROR;
        v.permitText = "虚拟转台：不适用验收许可";
    }

    v.busy = commandActive_ || stopInFlight_.load();
    v.positioningEnabled = v.motionEnabled && !v.busy && !measurementActive;
    v.stopEnabled = true;

    if (measurementActive)
    {
        v.gateText = "测量任务进行中：先停止测量";
    }
    else if (commandActive_)
    {
        v.gateText = "上一条定位尚未结束";
    }
    else if (stopInFlight_.load())
    {
        v.gateText = "停止尚未完成";
    }
    else if (!v.motionEnabled)
    {
        v.gateText = (diagnostics_ != nullptr &&
                      (!v.sampleValid || !v.sampleFresh))
                         ? "无新鲜有效双轴样本"
                         : v.permitText;
    }

    v.commandStage = commandStage_;
    v.commandStageText = commandStageText_.empty()
                             ? commandStageName(commandStage_)
                             : commandStageText_;
    v.commandDetail = commandDetail_;
    v.hasTarget = hasTarget_;
    v.targetAzimuth = targetAz_;
    v.targetElevation = targetEl_;

    v.stopStage = stopStage_;
    v.stopStageText = stopStageText_.empty()
                          ? stopStageName(stopStage_)
                          : stopStageText_;
    v.stopDetail = stopDetail_;
    v.lastActionText = lastAction_;
    return v;
}

}  // namespace app
}  // namespace aircraft
