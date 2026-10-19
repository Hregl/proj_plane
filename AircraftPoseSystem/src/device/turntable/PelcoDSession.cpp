// ============================================================================
//  src/device/turntable/PelcoDSession.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、§12/§13、ENG-09 V2.7 §6.3、
//        裁决 C-22、C-23
// ============================================================================

#include "device/turntable/PelcoDSession.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <thread>

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// 指令重试次数：总尝试 = kCommandRetries + 1 = 3。
constexpr int kCommandRetries = 2;

/// 接收切片上限。停止屏障在**每个切片之后**都能被 worker 检查到；
/// 单个切片超时只是继续等，**不是**整个事务超时。
constexpr int kReceiveSliceMs = 10;

constexpr int kMaxDatagramBytes = 2048;

/// 看门狗估算：未启用转速控制时按 12 °/s 保守估计（SYS-10 §6.5）。
constexpr double kDefaultSpeedDegPerSec = 12.0;

std::uint64_t msToNs(int ms)
{
    return static_cast<std::uint64_t>(ms) * 1000000ULL;
}

bool frameEquals(const PelcoDFrame& a, const PelcoDFrame& b)
{
    return a.address == b.address && a.data1 == b.data1 && a.data2 == b.data2 &&
           a.data3 == b.data3 && a.data4 == b.data4;
}

/// 始终返回 [0, 180] 的绝对角差。
///
/// ⚠ 旧实现先取 |a-b|，仅在 delta > 180 时折成 360-delta。当两角相差
///    超过 360°（例如 720°）时结果变成负数，而“<= tolerance”对负值恒真 ——
///    到位判据会在转台根本没动时成立。先取模再折回，范围才封闭。
double circularDeltaDeg(double a, double b)
{
    if (!std::isfinite(a) || !std::isfinite(b))
    {
        return 180.0;  // 非有限输入绝不能判成“到位”
    }
    double delta = std::fmod(a - b, 360.0);
    if (delta < 0.0)
    {
        delta = -delta;
    }
    if (delta > 180.0)
    {
        delta = 360.0 - delta;
    }
    return delta;
}

std::string hexByte(std::uint8_t value)
{
    char buffer[8] = {};
    std::snprintf(buffer, sizeof(buffer), "0x%02X", value);
    return std::string(buffer);
}

const char* workModeTrustName(data::WorkModeTrust trust)
{
    switch (trust)
    {
    case data::WorkModeTrust::Unknown:
        return "Unknown";
    case data::WorkModeTrust::EchoOnly:
        return "EchoOnly";
    case data::WorkModeTrust::ReplyNonZero:
        return "ReplyNonZero";
    case data::WorkModeTrust::NoReply:
        return "NoReply";
    }
    return "Unknown";
}

}  // namespace

PelcoDSession::PelcoDSession(const data::TurntableConfig& config,
                             std::shared_ptr<IPelcoTransport> transport)
    : config_(config), transport_(std::move(transport))
{
}

void PelcoDSession::setClock(std::function<std::uint64_t()> clock)
{
    clock_ = std::move(clock);
}

std::uint64_t PelcoDSession::nowNs() const
{
    return clock_ ? clock_() : data::monotonicNowNs();
}

bool PelcoDSession::sendFrame(const PelcoDFrame& frame, std::string& error)
{
    const auto encoded = encodePelcoD(frame);
    if (!transport_)
    {
        error = "转台传输未创建";
        return false;
    }
    return transport_->send(encoded.data(), encoded.size(), error);
}

bool PelcoDSession::waitForEcho(const PelcoDFrame& sent, std::string& error,
                                bool& rejected,
                                const std::function<bool()>& shouldAbort)
{
    rejected = false;
    const std::uint64_t deadline = nowNs() + msToNs(config_.replyTimeoutMs);
    std::uint8_t buffer[kMaxDatagramBytes] = {};

    for (;;)
    {
        if (shouldAbort && shouldAbort())
        {
            error = "指令被停止屏障取消";
            return false;
        }

        const std::uint64_t now = nowNs();
        if (now >= deadline)
        {
            error = "等待指令回显超时";
            return false;
        }

        const std::uint64_t remainingNs = deadline - now;
        int remainingMs = static_cast<int>(remainingNs / 1000000ULL);
        if (remainingMs <= 0)
        {
            remainingMs = 1;
        }
        // 切片：单次 receive 不跨越停止检查周期。切片超时后继续循环，
        // 直到 deadline —— 切片超时不等于事务超时。
        const int sliceMs = std::min(remainingMs, kReceiveSliceMs);

        const auto result =
            transport_->receive(buffer, sizeof(buffer), sliceMs);
        if (result.status == TransportStatus::Timeout)
        {
            continue;  // 本切片无数据；deadline 判断在循环顶部
        }
        if (result.status == TransportStatus::Error)
        {
            error = result.error.empty() ? "等待指令回显失败" : result.error;
            return false;
        }

        const auto replies = parsePelcoDDatagram(buffer, result.size);
        for (const auto& reply : replies)
        {
            if (reply.kind == PelcoDReplyKind::Rejected)
            {
                rejected = true;
                error = "设备拒绝指令（EE EE EE EE）";
                return false;
            }
            if (reply.kind == PelcoDReplyKind::ChecksumError)
            {
                error = "设备报告校验错误（FF FF FF FF）";
                return false;
            }
            if (reply.kind == PelcoDReplyKind::Echo &&
                frameEquals(reply.frame, sent))
            {
                return true;
            }
            // 角度/工作模式等其它帧：丢弃，继续等到超时。
        }
    }
}

bool PelcoDSession::sendAndAwaitEcho(
    const PelcoDFrame& frame, std::string& error, bool& rejected,
    const std::function<bool()>& shouldAbort)
{
    rejected = false;
    for (int attempt = 0; attempt <= kCommandRetries; ++attempt)
    {
        if (shouldAbort && shouldAbort())
        {
            error = "指令被停止屏障取消";
            return false;
        }
        if (!sendFrame(frame, error))
        {
            return false;
        }
        if (waitForEcho(frame, error, rejected, shouldAbort))
        {
            return true;
        }
        if (shouldAbort && shouldAbort())
        {
            error = "指令被停止屏障取消";
            return false;
        }
        // EE 是设备明确拒绝，重发同一帧没有意义（SYS-10 V1.1 §6.5）。
        if (rejected)
        {
            return false;
        }
        error.clear();
    }
    error += "（已尝试 " + std::to_string(kCommandRetries + 1) + " 次）";
    return false;
}

bool PelcoDSession::queryAngle(bool horizontal, double& out,
                               std::string& error, int retries)
{
    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame query =
        horizontal ? makeQueryHorizontal(address) : makeQueryVertical(address);
    std::uint8_t buffer[kMaxDatagramBytes] = {};

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
        if (!sendFrame(query, error))
        {
            return false;
        }

        const std::uint64_t deadline =
            nowNs() + msToNs(config_.replyTimeoutMs);
        bool retry = false;
        while (!retry)
        {
            const std::uint64_t now = nowNs();
            if (now >= deadline)
            {
                error = horizontal ? "查询水平角超时" : "查询垂直角超时";
                retry = true;
                break;
            }

            int remainingMs =
                static_cast<int>((deadline - now) / 1000000ULL);
            if (remainingMs <= 0)
            {
                remainingMs = 1;
            }
            const int sliceMs = std::min(remainingMs, kReceiveSliceMs);

            const auto result =
                transport_->receive(buffer, sizeof(buffer), sliceMs);
            if (result.status == TransportStatus::Timeout)
            {
                continue;
            }
            if (result.status == TransportStatus::Error)
            {
                error = result.error.empty() ? "查询角度失败" : result.error;
                return false;
            }

            const auto replies = parsePelcoDDatagram(buffer, result.size);
            for (const auto& reply : replies)
            {
                const bool wanted =
                    (horizontal &&
                     reply.kind == PelcoDReplyKind::AngleHorizontal) ||
                    (!horizontal &&
                     reply.kind == PelcoDReplyKind::AngleVertical);
                if (wanted)
                {
                    // C3：回复地址必须与配置一致。地址不符的回复按
                    // “没有有效回复”处理，继续等到本事务超时。
                    if (reply.frame.address != address)
                    {
                        continue;
                    }
                    out = reply.angleDeg;
                    return true;
                }
                if (reply.kind == PelcoDReplyKind::Rejected)
                {
                    error = "设备拒绝角度查询（EE EE EE EE）";
                    return false;
                }
                if (reply.kind == PelcoDReplyKind::ChecksumError)
                {
                    error = "角度回复校验错误（FF FF FF FF）";
                    retry = true;
                    break;
                }
                if (reply.kind == PelcoDReplyKind::InvalidAngle)
                {
                    error = "角度回复数值越界";
                    retry = true;
                    break;
                }
            }
        }
    }

    error += "（已尝试 " + std::to_string(retries + 1) + " 次）";
    return false;
}

bool PelcoDSession::queryWorkMode(std::uint8_t& out, bool& echoOnly,
                                  std::string& error, int retries)
{
    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame query = makeQueryWorkMode(address);
    std::uint8_t buffer[kMaxDatagramBytes] = {};
    echoOnly = false;

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
        if (!sendFrame(query, error))
        {
            return false;
        }

        const std::uint64_t deadline =
            nowNs() + msToNs(config_.replyTimeoutMs);
        bool retry = false;
        while (!retry)
        {
            const std::uint64_t now = nowNs();
            if (now >= deadline)
            {
                error = "查询工作模式超时";
                retry = true;
                break;
            }

            int remainingMs =
                static_cast<int>((deadline - now) / 1000000ULL);
            if (remainingMs <= 0)
            {
                remainingMs = 1;
            }
            const int sliceMs = std::min(remainingMs, kReceiveSliceMs);

            const auto result =
                transport_->receive(buffer, sizeof(buffer), sliceMs);
            if (result.status == TransportStatus::Timeout)
            {
                continue;
            }
            if (result.status == TransportStatus::Error)
            {
                error = result.error.empty() ? "查询工作模式失败"
                                             : result.error;
                return false;
            }

            const auto replies = parsePelcoDDatagram(buffer, result.size);
            for (const auto& reply : replies)
            {
                if (reply.kind == PelcoDReplyKind::WorkMode)
                {
                    // C3：地址不符不能完成工作模式事务。
                    if (reply.frame.address != address)
                    {
                        continue;
                    }
                    out      = reply.workMode;
                    echoOnly = frameEquals(reply.frame, query);
                    return true;
                }
                if (reply.kind == PelcoDReplyKind::Rejected)
                {
                    error = "设备拒绝工作模式查询（EE EE EE EE）";
                    return false;
                }
                if (reply.kind == PelcoDReplyKind::ChecksumError)
                {
                    error = "工作模式回复校验错误（FF FF FF FF）";
                    retry = true;
                    break;
                }
            }
        }
    }

    error += "（已尝试 " + std::to_string(retries + 1) + " 次）";
    return false;
}

bool PelcoDSession::finiteConfig(std::string& error) const
{
    if (!(config_.azimuthMin < config_.azimuthMax) ||
        !(config_.elevationMin < config_.elevationMax))
    {
        error = "转台行程配置无效（上限不大于下限）";
        return false;
    }
    if (!std::isfinite(config_.azimuthMin) ||
        !std::isfinite(config_.azimuthMax) ||
        !std::isfinite(config_.elevationMin) ||
        !std::isfinite(config_.elevationMax))
    {
        error = "转台行程配置含非有限值";
        return false;
    }
    if (!std::isfinite(config_.coarseSpeed) ||
        !std::isfinite(config_.fineSpeed) || config_.coarseSpeed <= 0.0 ||
        config_.fineSpeed <= 0.0)
    {
        error = "转台速度配置无效（非有限或 <= 0）";
        return false;
    }
    if (!std::isfinite(config_.stableThresholdDeg) ||
        !std::isfinite(config_.arrivalToleranceDeg) ||
        config_.stableThresholdDeg < 0.0 || config_.arrivalToleranceDeg < 0.0)
    {
        error = "转台到位判定参数含非有限/负值";
        return false;
    }
    if (config_.pelcoAddress < 1 || config_.pelcoAddress > 255)
    {
        error = "Pelco-D 设备地址必须在 1..255";
        return false;
    }
    if (config_.port < 1 || config_.port > 65535)
    {
        error = "转台 UDP 端口必须在 1..65535";
        return false;
    }
    if (config_.replyTimeoutMs < 1 || config_.stableReadCount < 1 ||
        config_.commFailureThreshold < 1 || config_.sampleStaleMs < 1 ||
        config_.stopTimeoutMs < 1 || config_.stopRetryCount < 0 ||
        config_.stopRetryCount > 5 || config_.stopRetryWindowMs < 1 ||
        config_.stopBackoffMs < 0)
    {
        error = "转台通信/停止/样本有效性参数无效";
        return false;
    }
    return true;
}

bool PelcoDSession::initialize(std::string& error)
{
    error.clear();
    {
        std::lock_guard<std::mutex> lock(faultMutex_);
        lastError_  = data::ErrorInfo{};
        firstFault_ = data::TurntableFault{};
    }
    errorLatched_         = false;
    initialized_          = false;
    moving_               = false;
    stableCount_          = 0;
    hasLastRead_          = false;
    consecutiveFailures_  = 0;
    stopPending_          = false;
    stopSent_             = false;
    stopRetryArmed_       = false;
    stopStationaryCount_  = 0;
    stopHasLast_          = false;
    stopObservationDeadlineNs_ = 0;
    motionUncertain_      = false;
    stopFrameDelivered_   = false;
    stopObservedStationary_ = false;
    sampleValid_          = false;
    sampleTimestampNs_    = 0;
    sampleCycleId_        = 0;
    workModeTrust_        = data::WorkModeTrust::Unknown;
    motion_               = data::TurntableMotionState::IDLE;

    // ---- 配置自洽性。错误必须在打开 socket 之前暴露。----
    if (!finiteConfig(error))
    {
        setError(data::kErrTurntableComm, error, true, "initialize/config");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (!transport_)
    {
        setError(data::kErrTurntableComm, "转台传输未创建", true,
                 "initialize/transport");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (!transport_->open(config_.host, static_cast<std::uint16_t>(config_.port),
                          error))
    {
        setError(data::kErrTurntableComm,
                 "打开转台 UDP 连接失败：" + error, true,
                 "initialize/open");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    double azimuth   = 0.0;
    double elevation = 0.0;
    std::uint8_t workMode = 0;
    bool echoOnly = false;

    // 两轴与工作模式在同一初始化事务内查询；每一类回复都核验地址。
    // ⚠ 保持原有的短路顺序：只有两轴都成功才查询工作模式。
    const bool gotAzimuth   = queryAngle(true, azimuth, error, kCommandRetries);
    const bool gotElevation = gotAzimuth &&
                              queryAngle(false, elevation, error,
                                         kCommandRetries);
    const bool gotWorkMode  = gotElevation &&
                              queryWorkMode(workMode, echoOnly, error,
                                            kCommandRetries);
    if (!gotAzimuth || !gotElevation || !gotWorkMode)
    {
        // 只有“确实发起了工作模式查询且没有有效回复”才记 NoReply ——
        // 两轴失败时工作模式事务根本没发生，不能替它下结论。
        if (gotElevation && !gotWorkMode)
        {
            workModeTrust_ = data::WorkModeTrust::NoReply;
        }
        transport_->close();
        setError(data::kErrTurntableComm, "转台初始化查询失败：" + error, true,
                 "initialize/query");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (workMode != 0x00)
    {
        workModeTrust_ = data::WorkModeTrust::ReplyNonZero;
        transport_->close();
        setError(data::kErrTurntableComm,
                 "转台工作模式不是常规模式（当前 " + hexByte(workMode) +
                     "），拒绝初始化；不自动清故障、不逃逸限位",
                 true, "initialize/work_mode");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    // ⚠ 不设 ConfirmedNormal：回复 0x00 但字节与查询不同，只能证明
    //    “报文不同”，不能证明它是独立、有效的工作模式回复。
    workModeTrust_ = echoOnly ? data::WorkModeTrust::EchoOnly
                              : data::WorkModeTrust::Unknown;

    if (azimuth < config_.azimuthMin - 1e-9 ||
        azimuth > config_.azimuthMax + 1e-9 ||
        elevation < config_.elevationMin - 1e-9 ||
        elevation > config_.elevationMax + 1e-9)
    {
        transport_->close();
        setError(data::kErrTurntableComm,
                 "转台实测角超出配置行程，拒绝初始化", true,
                 "initialize/range");
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    sampleCycleId_.store(0);
    publishSample(azimuth, elevation, nowNs(), true);  // cycle = 1
    initialized_ = true;
    motion_.store(data::TurntableMotionState::IDLE);
    return true;
}

bool PelcoDSession::move(const data::TurntableCommand& command,
                         std::string& error)
{
    return move(command, error, std::function<bool()>());
}

bool PelcoDSession::move(const data::TurntableCommand& command,
                         std::string& error,
                         const std::function<bool()>& shouldAbort)
{
    error.clear();

    if (!initialized_.load())
    {
        setError(data::kErrTurntableComm, "转台未初始化即收到移动命令", false,
                 "move/init");
        error = lastErrorText();
        return false;
    }
    if (errorLatched_.load())
    {
        error = lastErrorText();
        if (error.empty())
        {
            error = "转台处于锁存错误状态";
        }
        return false;
    }
    if (motionUncertain_.load())
    {
        setError(data::kErrTurntableMotionInhibited,
                 "转台运动状态不确定（停止未确认），必须先显式停止或重新初始化",
                 true, "move/motion_uncertain");
        error = lastErrorText();
        return false;
    }

    // 只有**即将开始一次新命令**时才清当前错误；锁存错误与运动不确定
    // 分支必须保留原错误码/文本（修复：先前无条件清零会让上层看到空错误）。
    {
        std::lock_guard<std::mutex> lock(faultMutex_);
        lastError_ = data::ErrorInfo{};  // 首因 firstFault_ 始终保留
    }

    // ---- 发送门禁（C1/A4）----
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        setError(data::kErrTurntableMotionInhibited,
                 "只读模式拒绝运动指令（控制帧恒为 0）", true,
                 "move/read_only");
        error = lastErrorText();
        return false;
    }
    if (!testMotionGate_)
    {
        // 本批没有可放行运动的协议证据：WorkMode 不构成许可。
        setError(data::kErrTurntableMotionInhibited,
                 std::string("运动门禁拒绝：WorkMode 可信度 ") +
                     workModeTrustName(workModeTrust_.load()) +
                     "，无运动授权",
                 true, "move/work_mode_gate");
        error = lastErrorText();
        return false;
    }

    // ---- 本地输入边界（C1/NaN）：非法命令一个字节都不发 ----
    if (!std::isfinite(command.azimuthCommand) ||
        !std::isfinite(command.elevationCommand))
    {
        setError(data::kErrTurntableInvalidCommand,
                 "移动命令含非有限角，指令未发送", true, "move/nonfinite");
        motion_.store(data::TurntableMotionState::ERROR);
        error = lastErrorText();
        return false;
    }

    double azimuthTarget = 0.0;
    double elevationTarget = 0.0;
    if (!resolveAzimuth(command.azimuthCommand, azimuthTarget) ||
        !resolveElevation(command.elevationCommand, elevationTarget))
    {
        setError(data::kErrTurntableOverTravel,
                 "目标角度超出行程，指令未发送", true, "move/over_travel");
        motion_.store(data::TurntableMotionState::ERROR);
        error = lastErrorText();
        return false;
    }
    if (!std::isfinite(azimuthTarget) || !std::isfinite(elevationTarget))
    {
        setError(data::kErrTurntableInvalidCommand,
                 "解析后的目标角非有限，指令未发送", true,
                 "move/resolve_nonfinite");
        motion_.store(data::TurntableMotionState::ERROR);
        error = lastErrorText();
        return false;
    }

    const data::TurntableState current = snapshot();
    const double azDelta = circularDeltaDeg(azimuthTarget, current.azimuth);
    const double elDelta = std::fabs(elevationTarget - current.elevation);
    const double maxDelta = std::max(azDelta, elDelta);
    const double speed = selectedSpeedDegPerSec(maxDelta);

    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    PelcoDFrame horizontalFrame;
    PelcoDFrame verticalFrame;
    const bool framed =
        config_.useSpeedControl
            ? (makeAbsolutePositionWithSpeedChecked(address, true,
                                                    azimuthTarget, speed,
                                                    horizontalFrame) &&
               makeAbsolutePositionWithSpeedChecked(address, false,
                                                    elevationTarget, speed,
                                                    verticalFrame))
            : (makeAbsolutePositionChecked(address, true, azimuthTarget,
                                           horizontalFrame) &&
               makeAbsolutePositionChecked(address, false, elevationTarget,
                                           verticalFrame));
    if (!framed)
    {
        setError(data::kErrTurntableInvalidCommand,
                 "定位帧编码失败（非有限角/速度），指令未发送", true,
                 "move/encode");
        motion_.store(data::TurntableMotionState::ERROR);
        error = lastErrorText();
        return false;
    }

    stopSent_       = false;
    stopRetryArmed_ = false;

    bool rejected = false;
    std::string failure;

    if (!sendAndAwaitEcho(horizontalFrame, failure, rejected, shouldAbort))
    {
        if (shouldAbort && shouldAbort())
        {
            // 停止屏障已取消本次命令：不恢复、不再自行发停止帧
            //（停止帧由屏障路径发送一次）。
            error = failure.empty() ? "指令被停止屏障取消" : failure;
            return false;
        }
        // 回显超时/校验错误时设备**可能已经接受**了指令。
        // 不能假装"什么都没发生"：发起规定的停止处理并记录已知结果。
        stopForFault(data::kErrTurntableComm,
                     "水平轴指令失败：" + failure, "move/axis=horizontal");
        error = lastErrorText();
        return false;
    }

    if (!sendAndAwaitEcho(verticalFrame, failure, rejected, shouldAbort))
    {
        if (shouldAbort && shouldAbort())
        {
            error = failure.empty() ? "指令被停止屏障取消" : failure;
            return false;
        }
        // 部分轴已接受：必须请求停止，不能留下单轴运动。
        stopForFault(data::kErrTurntableComm,
                     "垂直轴指令失败：" + failure, "move/axis=vertical");
        error = lastErrorText();
        return false;
    }

    targetAz_    = azimuthTarget;
    targetEl_    = elevationTarget;
    moving_      = true;
    stableCount_ = 0;
    hasLastRead_ = false;
    consecutiveFailures_ = 0;
    stopPending_ = false;
    motion_.store(data::TurntableMotionState::MOVING);
    armWatchdog(azimuthTarget, elevationTarget);
    return true;
}

void PelcoDSession::pollOnce()
{
    if (!initialized_.load())
    {
        return;
    }

    const std::uint64_t now = nowNs();

    // 停止观测窗口到期：无论本轮采样是否成功，都要如实锁存。
    if (stopPending_ && stopFrameDelivered_ &&
        now >= stopObservationDeadlineNs_)
    {
        stopPending_ = false;
        motionUncertain_ = true;
        setFirstFault(data::kErrTurntableStopNotObserved,
                      "停止帧已交付，但观测窗口内未观察到静止",
                      "stop/observe_timeout");
        setError(data::kErrTurntableStopNotObserved,
                 "停止帧已交付，但观测窗口内未观察到静止", true,
                 "stop/observe_timeout");
        motion_.store(data::TurntableMotionState::ERROR);
        return;
    }

    if (moving_ && !stopPending_ && moveDeadlineNs_ != 0 &&
        now >= moveDeadlineNs_)
    {
        moveDeadlineNs_ = 0;
        // 2008 是“命令未在时限内完成”的事实，不是一次通信失败。
        stopForFault(data::kErrTurntableMotionWatchdog,
                     "移动看门狗超时", "watchdog");
        setError(data::kErrTurntableMotionWatchdog, "移动看门狗超时", true,
                 "watchdog");
        motion_.store(data::TurntableMotionState::ERROR);
        return;
    }

    double azimuth = 0.0;
    double elevation = 0.0;
    std::string azError;
    std::string elError;

    // 轮询周期内**不重试**：下一个周期就是重试。一次丢包不该把
    // 50 ms 的周期拖成 150 ms。
    const bool gotAzimuth = queryAngle(true, azimuth, azError, 0);
    const bool gotElevation = queryAngle(false, elevation, elError, 0);

    // C4：两轴必须同轮。任一轴缺失 ⇒ 整个样本无效，**不得**把
    // 新方位角与旧俯仰角拼在一起发布。
    if (!gotAzimuth || !gotElevation)
    {
        sampleCycleId_.fetch_add(1);
        sampleTimestampNs_.store(now);
        sampleValid_.store(false);
        ++consecutiveFailures_;
        stableCount_ = 0;
        hasLastRead_ = false;

        // 没有新鲜样本时不能继续声称 STABLE。
        if (motion_.load() == data::TurntableMotionState::STABLE)
        {
            motion_.store(data::TurntableMotionState::IDLE);
        }

        if (consecutiveFailures_ >= config_.commFailureThreshold &&
            !errorLatched_.load())
        {
            const std::string text =
                "连续 " + std::to_string(consecutiveFailures_) +
                " 个轮询周期无有效角度回复";
            setFirstFault(data::kErrTurntableComm, text, "poll/lost_link");
            stopForFault(data::kErrTurntableComm, text, "poll/lost_link");
            setError(data::kErrTurntableComm, text, true, "poll/lost_link");
            motion_.store(data::TurntableMotionState::ERROR);
        }
        return;
    }

    consecutiveFailures_ = 0;
    publishSample(azimuth, elevation, now, true);

    if (stopPending_ && stopFrameDelivered_)
    {
        handleStopObservation(azimuth, elevation, now);
        return;
    }

    updateMotion(azimuth, elevation);
}

bool PelcoDSession::performStop(int faultCode, const std::string& faultMessage,
                                const std::string& context,
                                bool recordFirstFault)
{
    // 只读模式：所有路径均不得产生控制帧（C1）。
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return false;
    }

    if (stopSent_ && !stopRetryArmed_)
    {
        return true;  // 已经交付过，等待观测；不自动重发
    }
    if (stopRetryArmed_)
    {
        stopSent_ = false;
    }

    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame stopFrame = makeStop(address);

    const std::uint64_t deadline =
        nowNs() + msToNs(config_.stopRetryWindowMs);
    bool delivered = false;
    std::string lastFailure;

    for (int attempt = 0; attempt <= config_.stopRetryCount; ++attempt)
    {
        if (sendFrame(stopFrame, lastFailure))
        {
            delivered = true;
            break;
        }
        if (attempt >= config_.stopRetryCount)
        {
            break;
        }
        const std::uint64_t now = nowNs();
        if (now + msToNs(config_.stopBackoffMs) > deadline)
        {
            break;
        }
        if (config_.stopBackoffMs > 0)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.stopBackoffMs));
        }
    }

    stopSent_ = delivered;
    if (delivered)
    {
        stopFrameDelivered_     = true;
        stopObservedStationary_ = false;
        stopPending_            = true;
        stopStationaryCount_    = 0;
        stopHasLast_            = false;
        stopObservationDeadlineNs_ =
            nowNs() + msToNs(config_.stopTimeoutMs);
        moving_ = false;
        // 停止帧已交付 ≠ 已观察到静止。在观测窗口内保持 MOVING，
        // 不谎报 IDLE/STABLE。
        if (!errorLatched_.load())
        {
            motion_.store(data::TurntableMotionState::MOVING);
        }
        return true;
    }

    // 窗口耗尽仍无法交付：2005 + motionUncertain，停止自动重试。
    motionUncertain_      = true;
    stopFrameDelivered_   = false;
    stopObservedStationary_ = false;
    stopPending_          = true;
    (void)recordFirstFault;
    if (faultCode != 0)
    {
        // 首因优先：原始故障（如 2001/2008）先写入；setFirstFault
        // 只写一次，后续的 2005 不会覆盖它。
        setFirstFault(faultCode, faultMessage, context);
    }
    setFirstFault(data::kErrTurntableStopNotDelivered,
                  "停止帧未能在窗口内交付：" + lastFailure,
                  context + ";attempts<= " +
                      std::to_string(config_.stopRetryCount + 1));
    setError(data::kErrTurntableStopNotDelivered,
             "停止帧未能在窗口内交付：" + lastFailure, true,
             context);
    motion_.store(data::TurntableMotionState::ERROR);
    return false;
}

void PelcoDSession::stopForFault(int faultCode, const std::string& message,
                                 const std::string& context)
{
    if (!initialized_.load())
    {
        return;
    }
    setFirstFault(faultCode, message, context);
    // 只读模式：故障也不得产生控制帧。原故障照实锁存。
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        setError(faultCode, message, true, context);
        return;
    }
    const bool delivered = performStop(faultCode, message, context, false);
    if (delivered)
    {
        // 原始故障保留为首因；当前错误如实记为原始码（非锁存），
        // 由观测窗口决定是否升级为 2006。
        setError(faultCode, message, false, context);
    }
}

void PelcoDSession::stop()
{
    if (!initialized_.load())
    {
        return;
    }
    // ReadOnly：无帧 no-op（C1）。
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return;
    }
    if (!moving_ && !errorLatched_.load() && !motionUncertain_.load() &&
        !stopPending_)
    {
        return;
    }

    // 显式停止是唯一允许“重新武装停止重试”的入口之一
    //（另一处是成功的 initialize()）。只有**上次未交付**时才重新武装；
    // 已交付的停止再次调用 stop() 不重复发帧。
    stopRetryArmed_ = motionUncertain_.load() && !stopFrameDelivered_.load();
    performStop(0, "显式停止", "stop/explicit", false);
    stopRetryArmed_ = false;

    if (motionUncertain_.load() && !errorLatched_.load())
    {
        motion_.store(data::TurntableMotionState::ERROR);
    }
}

void PelcoDSession::publishSample(double az, double el, std::uint64_t now,
                                  bool valid)
{
    azimuth_.store(az);
    elevation_.store(el);
    sampleTimestampNs_.store(now);
    sampleValid_.store(valid);
    if (valid)
    {
        sampleCycleId_.fetch_add(1);
    }
}

bool PelcoDSession::withinTolerance(double az, double el) const
{
    return circularDeltaDeg(az, targetAz_) <=
               config_.arrivalToleranceDeg &&
           std::fabs(el - targetEl_) <= config_.arrivalToleranceDeg;
}

void PelcoDSession::handleStopObservation(double az, double el,
                                          std::uint64_t now)
{
    const bool unchanged =
        stopHasLast_ &&
        std::fabs(az - stopLastAz_) <= config_.stableThresholdDeg &&
        std::fabs(el - stopLastEl_) <= config_.stableThresholdDeg;

    if (unchanged)
    {
        ++stopStationaryCount_;
    }
    else
    {
        stopStationaryCount_ = 0;
    }

    stopHasLast_ = true;
    stopLastAz_  = az;
    stopLastEl_  = el;

    if (stopStationaryCount_ >= std::max(1, config_.stableReadCount))
    {
        stopObservedStationary_ = true;
        stopPending_            = false;
        if (!errorLatched_.load())
        {
            motion_.store(data::TurntableMotionState::IDLE);
        }
        return;
    }

    if (now >= stopObservationDeadlineNs_)
    {
        stopPending_     = false;
        motionUncertain_ = true;
        setFirstFault(data::kErrTurntableStopNotObserved,
                      "停止后未在观测窗口内观察到静止",
                      "stop/observe_timeout");
        setError(data::kErrTurntableStopNotObserved,
                 "停止后未在观测窗口内观察到静止", true,
                 "stop/observe_timeout");
        motion_.store(data::TurntableMotionState::ERROR);
    }
}

void PelcoDSession::updateMotion(double az, double el)
{
    if (errorLatched_.load() || motionUncertain_.load())
    {
        return;
    }

    const bool inTolerance = withinTolerance(az, el);

    if (!moving_)
    {
        // C2：已到位后读数明显偏离 ⇒ **撤销** STABLE，不得一次置位后
        // 永久保持。STABLE 必须是“本轮样本满足判据”的结论。
        if (motion_.load() == data::TurntableMotionState::STABLE &&
            !inTolerance)
        {
            motion_.store(data::TurntableMotionState::IDLE);
        }
        lastAz_      = az;
        lastEl_      = el;
        hasLastRead_ = true;
        return;
    }

    const bool unchanged =
        hasLastRead_ &&
        std::fabs(az - lastAz_) <= config_.stableThresholdDeg &&
        std::fabs(el - lastEl_) <= config_.stableThresholdDeg;

    if (inTolerance && unchanged)
    {
        ++stableCount_;
    }
    else
    {
        stableCount_ = 0;
    }

    lastAz_      = az;
    lastEl_      = el;
    hasLastRead_ = true;

    if (inTolerance &&
        stableCount_ >= std::max(1, config_.stableReadCount))
    {
        moving_ = false;
        motion_.store(data::TurntableMotionState::STABLE);
    }
    else
    {
        motion_.store(data::TurntableMotionState::MOVING);
    }
}

data::TurntableState PelcoDSession::snapshot() const
{
    data::TurntableState state;
    state.azimuth   = azimuth_.load();
    state.elevation = elevation_.load();
    state.motion    = motion_.load();

    if (initialized_.load())
    {
        const std::uint64_t now = nowNs();
        const std::uint64_t ts  = sampleTimestampNs_.load();
        const bool stale =
            !sampleValid_.load() || ts == 0 || now < ts ||
            (now - ts) > msToNs(config_.sampleStaleMs);
        if (stale)
        {
            // 过期/无效样本不得再被当作“可采集的稳定状态”。
            state.motion = data::TurntableMotionState::ERROR;
        }
    }
    return state;
}

data::TurntableDiagnostics PelcoDSession::diagnostics() const
{
    data::TurntableDiagnostics d;
    d.sample.azimuth         = azimuth_.load();
    d.sample.elevation       = elevation_.load();
    d.sample.sampleTimestampNs = sampleTimestampNs_.load();
    d.sample.cycleId         = sampleCycleId_.load();
    d.sample.valid           = sampleValid_.load();
    d.sample.source = d.sample.valid ? data::TurntableSampleSource::Poll
                                     : data::TurntableSampleSource::None;

    const std::uint64_t now = nowNs();
    const std::uint64_t ts  = d.sample.sampleTimestampNs;
    d.sampleFresh = initialized_.load() && d.sample.valid && ts != 0 &&
                    now >= ts &&
                    (now - ts) <= msToNs(config_.sampleStaleMs);

    d.stopFrameDelivered     = stopFrameDelivered_.load();
    d.stopObservedStationary = stopObservedStationary_.load();
    d.motionUncertain        = motionUncertain_.load();
    d.errorLatched           = errorLatched_.load();
    d.workModeTrust          = workModeTrust_.load();
    d.motionInhibited        = motionInhibited();

    {
        std::lock_guard<std::mutex> lock(faultMutex_);
        d.firstFault = firstFault_;
    }
    return d;
}

data::ErrorInfo PelcoDSession::lastError() const
{
    std::lock_guard<std::mutex> lock(faultMutex_);
    return lastError_;
}

std::string PelcoDSession::lastErrorText() const
{
    std::lock_guard<std::mutex> lock(faultMutex_);
    return lastError_.message;
}

bool PelcoDSession::motionInhibited() const
{
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return true;
    }
    // 本批没有可放行运动的协议证据；单元测试门是唯一例外。
    return !testMotionGate_;
}

void PelcoDSession::setError(int code, const std::string& text, bool latch,
                             const std::string& context)
{
    {
        std::lock_guard<std::mutex> lock(faultMutex_);
        lastError_.code        = code;
        lastError_.message     = text;
        lastError_.timestampNs = nowNs();
        (void)context;
    }
    if (latch)
    {
        errorLatched_ = true;
    }
}

void PelcoDSession::setFirstFault(int code, const std::string& text,
                                  const std::string& context)
{
    std::lock_guard<std::mutex> lock(faultMutex_);
    if (firstFault_.code != 0)
    {
        return;  // 首因只写一次
    }
    firstFault_.code        = code;
    firstFault_.message     = text;
    firstFault_.context     = context;
    firstFault_.timestampNs = nowNs();
}

bool PelcoDSession::resolveAzimuth(double command, double& resolved) const
{
    if (!std::isfinite(command))
    {
        return false;
    }
    const double tolerance = 1e-9;
    auto inRange = [&](double value) {
        return std::isfinite(value) &&
               value >= config_.azimuthMin - tolerance &&
               value <= config_.azimuthMax + tolerance;
    };

    if (inRange(command))
    {
        resolved = command;
        return true;
    }

    // 同一物理角可以写成 command + k×360。选落在行程内、且与原命令
    // 数值差最小的那个；都不可行才报 2002。
    double normalized = std::fmod(command, 360.0);
    if (normalized > 180.0)
    {
        normalized -= 360.0;
    }
    else if (normalized <= -180.0)
    {
        normalized += 360.0;
    }

    double candidates[] = {command + 360.0, command - 360.0, normalized,
                           normalized + 360.0, normalized - 360.0};

    bool found = false;
    double bestDistance = std::numeric_limits<double>::max();
    for (double candidate : candidates)
    {
        if (!inRange(candidate))
        {
            continue;
        }
        const double distance = std::fabs(candidate - command);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            resolved     = candidate;
            found        = true;
        }
    }
    return found;
}

bool PelcoDSession::resolveElevation(double command, double& resolved) const
{
    if (!std::isfinite(command))
    {
        return false;
    }
    if (command < config_.elevationMin - 1e-9 ||
        command > config_.elevationMax + 1e-9)
    {
        return false;
    }
    resolved = command;
    return true;
}

double PelcoDSession::selectedSpeedDegPerSec(double maxDeltaDeg) const
{
    if (!config_.useSpeedControl)
    {
        return 0.0;  // 表 3.5 不携带速度
    }
    return maxDeltaDeg > 1.0 ? config_.coarseSpeed : config_.fineSpeed;
}

void PelcoDSession::armWatchdog(double targetAz, double targetEl)
{
    const data::TurntableState current = snapshot();
    const double travel =
        std::max(circularDeltaDeg(targetAz, current.azimuth),
                 std::fabs(targetEl - current.elevation));

    double speed = config_.useSpeedControl
                       ? selectedSpeedDegPerSec(travel)
                       : kDefaultSpeedDegPerSec;
    if (!std::isfinite(speed) || speed <= 0.0)
    {
        speed = kDefaultSpeedDegPerSec;
    }

    double seconds = travel / speed * 1.5 + 5.0;
    if (!std::isfinite(seconds) || seconds > 600.0)
    {
        seconds = 600.0;
    }
    if (seconds < 0.001)
    {
        seconds = 0.001;
    }

    // 先做有限性与范围检查，再转整数：直接对 NaN/Inf 强转会得到
    // 未定义的截止时刻。
    const double ns = seconds * 1e9;
    if (!std::isfinite(ns) || ns < 0.0)
    {
        moveDeadlineNs_ = nowNs() + msToNs(600000);
        return;
    }
    moveDeadlineNs_ = nowNs() + static_cast<std::uint64_t>(ns);
}

}  // namespace device
}  // namespace aircraft
