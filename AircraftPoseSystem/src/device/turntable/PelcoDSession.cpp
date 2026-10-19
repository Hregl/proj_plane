// ============================================================================
//  src/device/turntable/PelcoDSession.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、ENG-09 V2.7 §6.3、裁决 C-22
// ============================================================================

#include "device/turntable/PelcoDSession.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// 指令重试次数：总尝试 = kCommandRetries + 1 = 3。
/// 查询在轮询周期里不重试（下一次 poll 就是重试），否则一个丢包会把
/// 一个 50 ms 的周期拖成 150 ms，worker 会持续落后。
constexpr int kCommandRetries = 2;

constexpr int kMaxDatagramBytes = 2048;

/// 看门狗估算：未启用转速控制时按 12 °/s 保守估计（SYS-10 V1.1 §6.5）。
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

double circularDeltaDeg(double a, double b)
{
    double delta = std::fabs(a - b);
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
                                bool& rejected)
{
    rejected = false;
    const std::uint64_t deadline = nowNs() + msToNs(config_.replyTimeoutMs);
    std::uint8_t buffer[kMaxDatagramBytes] = {};

    for (;;)
    {
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

        const auto result =
            transport_->receive(buffer, sizeof(buffer), remainingMs);
        if (result.status == TransportStatus::Timeout)
        {
            error = "等待指令回显超时";
            return false;
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
            // 设备可能在回显前后夹带轮询回复，不能把"收到任何报文"
            // 当成"指令被接受"。
        }
    }
}

bool PelcoDSession::sendAndAwaitEcho(const PelcoDFrame& frame,
                                     std::string& error, bool& rejected)
{
    rejected = false;
    for (int attempt = 0; attempt <= kCommandRetries; ++attempt)
    {
        if (!sendFrame(frame, error))
        {
            return false;
        }
        if (waitForEcho(frame, error, rejected))
        {
            return true;
        }
        // EE 是设备明确拒绝，重发同一帧没有意义（SYS-10 V1.1 §6.5）。
        if (rejected)
        {
            return false;
        }
    }
    error += "（已尝试 3 次）";
    return false;
}

bool PelcoDSession::queryAngle(bool horizontal, double& out,
                               std::string& error, int retries)
{
    const PelcoDFrame query = horizontal ? makeQueryHorizontal(
                                               static_cast<std::uint8_t>(
                                                   config_.pelcoAddress))
                                         : makeQueryVertical(
                                               static_cast<std::uint8_t>(
                                                   config_.pelcoAddress));
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

            const auto result =
                transport_->receive(buffer, sizeof(buffer), remainingMs);
            if (result.status == TransportStatus::Timeout)
            {
                error = horizontal ? "查询水平角超时" : "查询垂直角超时";
                retry = true;
                break;
            }
            if (result.status == TransportStatus::Error)
            {
                error = result.error.empty() ? "查询角度失败" : result.error;
                return false;
            }

            const auto replies = parsePelcoDDatagram(buffer, result.size);
            for (const auto& reply : replies)
            {
                if (horizontal &&
                    reply.kind == PelcoDReplyKind::AngleHorizontal)
                {
                    out = reply.angleDeg;
                    return true;
                }
                if (!horizontal &&
                    reply.kind == PelcoDReplyKind::AngleVertical)
                {
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

bool PelcoDSession::queryWorkMode(std::uint8_t& out, std::string& error,
                                  int retries)
{
    const PelcoDFrame query =
        makeQueryWorkMode(static_cast<std::uint8_t>(config_.pelcoAddress));
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

            const auto result =
                transport_->receive(buffer, sizeof(buffer), remainingMs);
            if (result.status == TransportStatus::Timeout)
            {
                error = "查询工作模式超时";
                retry = true;
                break;
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
                    // ⚠ 若设备把查询帧原样回显（0xE0 00 00 00），它会与
                    // WorkMode=0x00 的回复同形。手册表 3.41 规定设备回
                    // 当前模式，故这里按回复解析；M5 只读实测会核对。
                    out = reply.workMode;
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

bool PelcoDSession::initialize(std::string& error)
{
    error.clear();
    lastError_        = data::ErrorInfo{};
    errorLatched_     = false;
    initialized_      = false;
    moving_           = false;
    stableCount_      = 0;
    hasLastRead_      = false;
    consecutiveFailures_ = 0;
    stopSent_         = false;
    motion_.store(data::TurntableMotionState::IDLE);

    // ---- 配置自洽性。错误必须在打开 socket 之前暴露。----
    if (!(config_.azimuthMin < config_.azimuthMax) ||
        !(config_.elevationMin < config_.elevationMax))
    {
        setError(data::kErrTurntableComm, "转台行程配置无效（上限不大于下限）",
                 true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }
    if (config_.coarseSpeed <= 0.0 || config_.fineSpeed <= 0.0)
    {
        setError(data::kErrTurntableComm, "转台速度配置无效（粗调或精调 <= 0）",
                 true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }
    if (config_.pelcoAddress < 1 || config_.pelcoAddress > 255)
    {
        setError(data::kErrTurntableComm, "Pelco-D 设备地址必须在 1..255",
                 true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }
    if (config_.port < 1 || config_.port > 65535)
    {
        setError(data::kErrTurntableComm, "转台 UDP 端口必须在 1..65535", true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }
    if (config_.replyTimeoutMs < 1 || config_.stableReadCount < 1 ||
        config_.commFailureThreshold < 1 || config_.stableThresholdDeg < 0.0 ||
        config_.arrivalToleranceDeg < 0.0)
    {
        setError(data::kErrTurntableComm, "转台通信/到位判定参数无效", true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (!transport_)
    {
        setError(data::kErrTurntableComm, "转台传输未创建", true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (!transport_->open(config_.host, static_cast<std::uint16_t>(config_.port),
                          error))
    {
        setError(data::kErrTurntableComm,
                 "打开转台 UDP 连接失败：" + error, true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    double azimuth   = 0.0;
    double elevation = 0.0;
    std::uint8_t workMode = 0;

    if (!queryAngle(true, azimuth, error, kCommandRetries) ||
        !queryAngle(false, elevation, error, kCommandRetries) ||
        !queryWorkMode(workMode, error, kCommandRetries))
    {
        transport_->close();
        setError(data::kErrTurntableComm, "转台初始化查询失败：" + error, true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (workMode != 0x00)
    {
        transport_->close();
        setError(data::kErrTurntableComm,
                 "转台工作模式不是常规模式（当前 " + hexByte(workMode) +
                     "），拒绝初始化；不自动清故障、不逃逸限位",
                 true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    if (azimuth < config_.azimuthMin - 1e-9 ||
        azimuth > config_.azimuthMax + 1e-9 ||
        elevation < config_.elevationMin - 1e-9 ||
        elevation > config_.elevationMax + 1e-9)
    {
        transport_->close();
        setError(data::kErrTurntableComm,
                 "转台实测角超出配置行程，拒绝初始化", true);
        motion_.store(data::TurntableMotionState::ERROR);
        return false;
    }

    azimuth_.store(azimuth);
    elevation_.store(elevation);
    motion_.store(data::TurntableMotionState::IDLE);
    initialized_ = true;
    return true;
}

bool PelcoDSession::move(const data::TurntableCommand& command,
                         std::string& error)
{
    error.clear();
    lastError_ = data::ErrorInfo{};

    if (!initialized_)
    {
        setError(data::kErrTurntableComm, "转台未初始化即收到移动命令", false);
        error = lastError_.message;
        return false;
    }
    if (errorLatched_)
    {
        error = lastError_.message.empty() ? "转台处于锁存错误状态"
                                           : lastError_.message;
        return false;
    }

    double azimuthTarget = 0.0;
    double elevationTarget = 0.0;
    if (!resolveAzimuth(command.azimuthCommand, azimuthTarget) ||
        !resolveElevation(command.elevationCommand, elevationTarget))
    {
        // 本地超行程：**一个字节都不发**。发出去再拦已经太晚 ——
        // 真机上这可能意味着已经撞到机械限位。
        setError(data::kErrTurntableOverTravel,
                 "目标角度超出行程，指令未发送", true);
        motion_.store(data::TurntableMotionState::ERROR);
        error = lastError_.message;
        return false;
    }

    const data::TurntableState current = snapshot();
    const double azDelta = circularDeltaDeg(azimuthTarget, current.azimuth);
    const double elDelta = std::fabs(elevationTarget - current.elevation);
    const double maxDelta = std::max(azDelta, elDelta);
    const double speed = selectedSpeedDegPerSec(maxDelta);

    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame horizontalFrame =
        config_.useSpeedControl
            ? makeAbsolutePositionWithSpeed(address, true, azimuthTarget, speed)
            : makeAbsolutePosition(address, true, azimuthTarget);
    const PelcoDFrame verticalFrame =
        config_.useSpeedControl
            ? makeAbsolutePositionWithSpeed(address, false, elevationTarget,
                                            speed)
            : makeAbsolutePosition(address, false, elevationTarget);

    stopSent_ = false;

    bool rejected = false;
    std::string failure;

    if (!sendAndAwaitEcho(horizontalFrame, failure, rejected))
    {
        // 回显超时/校验错误时设备**可能已经接受**了指令。
        // 此时不能假装"什么都没发生"，先尽力停止。
        bestEffortStop();
        setError(data::kErrTurntableComm, "水平轴指令失败：" + failure, false);
        error = lastError_.message;
        return false;
    }

    if (!sendAndAwaitEcho(verticalFrame, failure, rejected))
    {
        // 部分轴已接受：必须立即尽力停止，不能留下单轴运动。
        bestEffortStop();
        setError(data::kErrTurntableComm, "垂直轴指令失败：" + failure, false);
        error = lastError_.message;
        return false;
    }

    targetAz_    = azimuthTarget;
    targetEl_    = elevationTarget;
    moving_      = true;
    stableCount_ = 0;
    hasLastRead_ = false;
    consecutiveFailures_ = 0;
    motion_.store(data::TurntableMotionState::MOVING);
    armWatchdog(azimuthTarget, elevationTarget);
    return true;
}

void PelcoDSession::pollOnce()
{
    if (!initialized_ || errorLatched_)
    {
        return;
    }

    if (moving_ && moveDeadlineNs_ != 0 && nowNs() >= moveDeadlineNs_)
    {
        bestEffortStop();
        moving_ = false;
        setError(data::kErrTurntableComm, "移动看门狗超时，已尽力停止", true);
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

    if (gotAzimuth)
    {
        azimuth_.store(azimuth);
    }
    if (gotElevation)
    {
        elevation_.store(elevation);
    }

    if (!gotAzimuth || !gotElevation)
    {
        ++consecutiveFailures_;
        stableCount_ = 0;
        hasLastRead_ = false;

        if (consecutiveFailures_ >= config_.commFailureThreshold)
        {
            bestEffortStop();
            moving_ = false;
            setError(data::kErrTurntableComm,
                     "连续 " + std::to_string(consecutiveFailures_) +
                         " 个轮询周期无有效角度回复",
                     true);
            motion_.store(data::TurntableMotionState::ERROR);
        }
        return;
    }

    consecutiveFailures_ = 0;
    updateMotion(azimuth, elevation);
}

void PelcoDSession::updateMotion(double azimuth, double elevation)
{
    const bool withinTolerance =
        circularDeltaDeg(azimuth, targetAz_) <= config_.arrivalToleranceDeg &&
        std::fabs(elevation - targetEl_) <= config_.arrivalToleranceDeg;

    if (!moving_)
    {
        lastAz_      = azimuth;
        lastEl_      = elevation;
        hasLastRead_ = true;
        return;
    }

    const bool unchanged =
        hasLastRead_ &&
        std::fabs(azimuth - lastAz_) <= config_.stableThresholdDeg &&
        std::fabs(elevation - lastEl_) <= config_.stableThresholdDeg;

    if (withinTolerance && unchanged)
    {
        ++stableCount_;
    }
    else
    {
        stableCount_ = 0;
    }

    lastAz_      = azimuth;
    lastEl_      = elevation;
    hasLastRead_ = true;

    if (withinTolerance &&
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

void PelcoDSession::stop()
{
    if (!initialized_)
    {
        return;
    }
    // 只读启动/退出时既没有未完成动作、也没有锁存错误 ⇒ **不发任何帧**。
    if (!moving_ && !errorLatched_)
    {
        return;
    }
    if (stopSent_)
    {
        return;
    }

    std::string ignored;
    if (sendFrame(makeStop(static_cast<std::uint8_t>(config_.pelcoAddress)),
                  ignored))
    {
        stopSent_ = true;
    }

    if (moving_)
    {
        moving_ = false;
        if (!errorLatched_)
        {
            motion_.store(data::TurntableMotionState::IDLE);
        }
    }
}

data::TurntableState PelcoDSession::snapshot() const
{
    data::TurntableState state;
    state.azimuth   = azimuth_.load();
    state.elevation = elevation_.load();
    state.motion    = motion_.load();
    return state;
}

data::ErrorInfo PelcoDSession::lastError() const
{
    return lastError_;
}

std::string PelcoDSession::lastErrorText() const
{
    return lastError_.message;
}

void PelcoDSession::setError(int code, const std::string& text, bool latch)
{
    lastError_.code        = code;
    lastError_.message     = text;
    lastError_.timestampNs = nowNs();
    errorLatched_          = latch;
}

void PelcoDSession::bestEffortStop()
{
    if (stopSent_)
    {
        return;
    }
    std::string ignored;
    if (sendFrame(makeStop(static_cast<std::uint8_t>(config_.pelcoAddress)),
                  ignored))
    {
        stopSent_ = true;
    }
}

bool PelcoDSession::resolveAzimuth(double command, double& resolved) const
{
    const double tolerance = 1e-9;
    auto inRange = [&](double value) {
        return value >= config_.azimuthMin - tolerance &&
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
    const double maxDelta = travel;

    double speed = config_.useSpeedControl
                       ? selectedSpeedDegPerSec(maxDelta)
                       : kDefaultSpeedDegPerSec;
    if (speed <= 0.0)
    {
        speed = kDefaultSpeedDegPerSec;
    }

    double seconds = travel / speed * 1.5 + 5.0;
    if (seconds > 600.0)
    {
        seconds = 600.0;
    }

    moveDeadlineNs_ = nowNs() + static_cast<std::uint64_t>(seconds * 1e9);
}

}  // namespace device
}  // namespace aircraft
