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

void PelcoDSession::setWallClockForTest(std::function<std::int64_t()> clock)
{
    wallClock_ = std::move(clock);
}

std::int64_t PelcoDSession::wallNowMs() const
{
    if (wallClock_)
    {
        return wallClock_();
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool PelcoDSession::setAcceptancePermit(
    const data::TurntableAcceptancePermit& permit, std::string& error)
{
    error.clear();

    if (initialized_.load())
    {
        error = "验收许可必须在 initialize() 之前安装";
        return false;
    }
    if (acceptancePermitInstalled_)
    {
        error = "验收许可只能安装一次，不可替换";
        return false;
    }

    if (permit.scopeId.empty())
    {
        error = "许可缺少验收范围编号（scope_id）";
        return false;
    }
    // 两个操作授权互相独立：只授权静止停止帧的许可合法（用于 T1），
    // 但它不构成任何运动授权 —— move() 的许可门禁仍会拒绝。
    if (!permit.allowAbsolutePosition && !permit.allowStopFrameAtRest)
    {
        error = "许可没有允许任何操作（绝对定位与静止停止帧均未授权）";
        return false;
    }
    if (permit.deviceHost != config_.host ||
        permit.devicePort != config_.port ||
        permit.pelcoAddress != config_.pelcoAddress)
    {
        // 不回显地址值：只说明不一致，避免把设备地址写进日志。
        error = "许可设备（host/port/pelco_address）与配置不一致";
        return false;
    }

    const double tol = 1e-9;
    if (!std::isfinite(permit.azimuthMinDeg) ||
        !std::isfinite(permit.azimuthMaxDeg) ||
        !std::isfinite(permit.elevationMinDeg) ||
        !std::isfinite(permit.elevationMaxDeg) ||
        !std::isfinite(permit.maxDisplacementDeg))
    {
        error = "许可角度范围或最大位移含非有限值";
        return false;
    }
    if (permit.azimuthMinDeg > permit.azimuthMaxDeg ||
        permit.elevationMinDeg > permit.elevationMaxDeg)
    {
        error = "许可角度范围下限大于上限";
        return false;
    }
    if (permit.azimuthMinDeg < config_.azimuthMin - tol ||
        permit.azimuthMaxDeg > config_.azimuthMax + tol ||
        permit.elevationMinDeg < config_.elevationMin - tol ||
        permit.elevationMaxDeg > config_.elevationMax + tol)
    {
        error = "许可角度范围超出配置的机械行程";
        return false;
    }
    if (permit.maxDisplacementDeg <= 0.0)
    {
        error = "许可最大位移必须大于 0";
        return false;
    }
    if (permit.maxExecutions <= 0)
    {
        error = "许可执行次数必须大于 0";
        return false;
    }
    if (permit.validFromEpochMs <= 0 ||
        permit.validUntilEpochMs <= permit.validFromEpochMs)
    {
        error = "许可有效期无效（必须满足 from>0 且 until>from）";
        return false;
    }
    if (wallNowMs() >= permit.validUntilEpochMs)
    {
        error = "许可已过期，拒绝安装";
        return false;
    }

    acceptancePermit_          = permit;
    acceptancePermitInstalled_ = true;
    acceptancePermitRemaining_.store(permit.maxExecutions);
    acceptancePermitConsumed_.store(0);
    return true;
}

bool PelcoDSession::acceptancePermitUsableNow() const
{
    if (!acceptancePermitInstalled_ ||
        !acceptancePermit_.allowAbsolutePosition)
    {
        return false;
    }
    const std::int64_t now = wallNowMs();
    return now >= acceptancePermit_.validFromEpochMs &&
           now < acceptancePermit_.validUntilEpochMs &&
           acceptancePermitRemaining_.load() > 0;
}

bool PelcoDSession::acceptancePermitAllows(double azimuthTarget,
                                           double elevationTarget,
                                           std::string& denial) const
{
    denial.clear();

    if (!acceptancePermitInstalled_)
    {
        denial = "无验收许可（acceptance permit）：运动指令未受理";
        return false;
    }
    if (!acceptancePermit_.allowAbsolutePosition)
    {
        denial = "验收许可未授权绝对定位：运动指令未受理";
        return false;
    }

    // 许可不是“工作模式正常”的证明，也不能绕过明确异常/无回复的模式。
    // ReplyNonZero/NoReply 在 initialize() 已失败，这里是第二道防线。
    const data::WorkModeTrust trust = workModeTrust_.load();
    if (trust == data::WorkModeTrust::ReplyNonZero ||
        trust == data::WorkModeTrust::NoReply)
    {
        denial = "工作模式可信度为异常/无回复，许可不能放行：运动指令未受理";
        return false;
    }

    const std::int64_t now = wallNowMs();
    if (now < acceptancePermit_.validFromEpochMs)
    {
        denial = "验收许可尚未生效：运动指令未受理";
        return false;
    }
    if (now >= acceptancePermit_.validUntilEpochMs)
    {
        denial = "验收许可已过期：新的运动指令未受理（停止不受影响）";
        return false;
    }
    if (acceptancePermitRemaining_.load() <= 0)
    {
        denial = "验收许可执行次数已用尽：运动指令未受理";
        return false;
    }

    // 许可不绕过过期/无效样本：没有同一轮的新鲜双轴读数，就无法核对
    // 当前位移，也无法保证目标相对“当前实际位置”是安全的。
    double currentAz = 0.0;
    double currentEl = 0.0;
    std::uint64_t ts    = 0;
    std::uint64_t cycle = 0;
    bool valid = false;
    data::TurntableMotionState motion = data::TurntableMotionState::IDLE;
    loadSample(currentAz, currentEl, ts, cycle, valid, motion);

    const std::uint64_t nowNsValue = nowNs();
    const bool fresh =
        valid && ts != 0 && nowNsValue >= ts &&
        (nowNsValue - ts) <= msToNs(config_.sampleStaleMs);
    if (!fresh)
    {
        denial = "无新鲜有效双轴样本，无法核对许可范围/位移："
                 "运动指令未受理";
        return false;
    }

    const double tol = 1e-9;
    if (azimuthTarget < acceptancePermit_.azimuthMinDeg - tol ||
        azimuthTarget > acceptancePermit_.azimuthMaxDeg + tol)
    {
        denial = "目标方位角超出验收许可范围：运动指令未受理";
        return false;
    }
    if (elevationTarget < acceptancePermit_.elevationMinDeg - tol ||
        elevationTarget > acceptancePermit_.elevationMaxDeg + tol)
    {
        denial = "目标俯仰角超出验收许可范围：运动指令未受理";
        return false;
    }

    const double azDelta = circularDeltaDeg(azimuthTarget, currentAz);
    const double elDelta = std::fabs(elevationTarget - currentEl);
    if (azDelta > acceptancePermit_.maxDisplacementDeg + tol ||
        elDelta > acceptancePermit_.maxDisplacementDeg + tol)
    {
        denial = "目标相对当前实测角的位移超出验收许可：运动指令未受理";
        return false;
    }
    return true;
}

bool PelcoDSession::motionPreauthorized() const
{
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return false;
    }
    if (testMotionGate_)
    {
        return true;
    }
    return acceptancePermitUsableNow();
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
    const std::function<bool()>& shouldAbort, bool& frameSent,
    bool& permitExpired)
{
    rejected      = false;
    frameSent     = false;
    permitExpired = false;
    for (int attempt = 0; attempt <= kCommandRetries; ++attempt)
    {
        if (shouldAbort && shouldAbort())
        {
            error = "指令被停止屏障取消";
            return false;
        }
        // 许可有效期必须在**每一个定位帧**发送前复核，重传也一样。
        // 只在 move() 入口核对一次会留下“第一轴发出后到期、第二轴或
        // 重传仍继续发”的缺口。次数已在命令入口按逻辑命令扣减一次，
        // 这里只核对期限与操作授权，不重复扣减。
        if (acceptancePermitInstalled_)
        {
            const std::int64_t nowWall = wallNowMs();
            if (!acceptancePermit_.allowAbsolutePosition ||
                nowWall < acceptancePermit_.validFromEpochMs ||
                nowWall >= acceptancePermit_.validUntilEpochMs)
            {
                error = "验收许可在发送过程中失效（过期或未授权定位）："
                        "定位帧未发送";
                permitExpired = true;
                return false;
            }
        }
        if (!sendFrame(frame, error))
        {
            return false;
        }
        // 从这一刻起设备**可能**已经接受指令：取消时必须补发停止帧。
        frameSent = true;
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
                               std::string& error, int retries,
                               const std::function<bool()>& shouldAbort,
                               bool* aborted)
{
    if (aborted != nullptr)
    {
        *aborted = false;
    }
    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame query =
        horizontal ? makeQueryHorizontal(address) : makeQueryVertical(address);
    std::uint8_t buffer[kMaxDatagramBytes] = {};

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
        // 停止屏障优先于整轮查询：每轴最多等 replyTimeoutMs，不能
        // 让停止请求排在它后面（否则一次双轴轮询可拖住 ~400 ms）。
        if (shouldAbort && shouldAbort())
        {
            if (aborted != nullptr)
            {
                *aborted = true;
            }
            error = "角度查询被停止屏障取消";
            return false;
        }
        if (!sendFrame(query, error))
        {
            return false;
        }

        const std::uint64_t deadline =
            nowNs() + msToNs(config_.replyTimeoutMs);
        bool retry = false;
        while (!retry)
        {
            if (shouldAbort && shouldAbort())
            {
                if (aborted != nullptr)
                {
                    *aborted = true;
                }
                error = "角度查询被停止屏障取消";
                return false;
            }

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
    stopBudgetSpent_      = false;
    stopStationaryCount_  = 0;
    stopHasLast_          = false;
    stopObservationDeadlineNs_ = 0;
    motionUncertain_      = false;
    motionTransactionId_  = 0;
    stopFactsTransactionId_ = 0;
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
    publishSample(azimuth, elevation, nowNs(), true,
                  data::TurntableMotionState::IDLE);  // cycle = 1
    initialized_ = true;
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

    // ---- 运动门禁：WorkMode 不构成许可 ----
    // 单元测试门只用于 fake transport；生产与验收路径必须由显式安装的
    // 验收许可放行。所有判据（设备/范围/位移/次数/期限）都在**发送前**
    // 核对，拒绝时一个运动帧都不发。
    // 许可一旦安装就**始终**参与判据，单元测试门也不能绕开它。
    if (acceptancePermitInstalled_)
    {
        std::string denial;
        if (!acceptancePermitAllows(azimuthTarget, elevationTarget, denial))
        {
            // 不锁存：修正目标或等到许可有效期内可以重新提交；
            // 但任何一次发送前都会重新核对，越界/过期永远发不出去。
            setError(data::kErrTurntableMotionInhibited, denial, false,
                     "move/acceptance_gate");
            error = lastErrorText();
            return false;
        }
    }
    else if (!testMotionGate_)
    {
        setError(data::kErrTurntableMotionInhibited,
                 std::string("无验收许可（acceptance permit），"
                             "WorkMode 可信度 ") +
                     workModeTrustName(workModeTrust_.load()) +
                     " 不构成运动授权：运动指令未受理",
                 true, "move/work_mode_gate");
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

    // 新的运动事务从这里开始：上一停止事务的“已交付/已观测静止”
    // 是**上一次**运动的事实，不能挂在新命令上。停止事实由
    // stopFrameDelivered/stopObservedStationary 表达，二者必须随
    // 新事务清零；否则上层会看到“正在 MOVING 且已观测到静止”。
    stopSent_               = false;
    stopRetryArmed_         = false;
    stopBudgetSpent_        = false;
    stopPending_            = false;
    stopStationaryCount_    = 0;
    stopHasLast_            = false;
    stopFrameDelivered_     = false;
    stopObservedStationary_ = false;
    // 事务归属：新运动开始即分配新编号。上一停止事务的编号保留在
    // stopFactsTransactionId_ 中，读端可据此识别“事实属于旧事务”。
    motionTransactionId_.fetch_add(1);

    // 许可次数在**进入发送阶段**时扣减。发送/回显失败不退款：
    // 许可约束的是尝试次数，不是成功次数；否则反复失败会突破上限。
    if (acceptancePermitInstalled_)
    {
        acceptancePermitRemaining_.fetch_sub(1);
        acceptancePermitConsumed_.fetch_add(1);
    }

    bool rejected         = false;
    bool horizontalSent   = false;
    bool verticalSent     = false;
    bool permitExpired    = false;
    std::string failure;

    if (!sendAndAwaitEcho(horizontalFrame, failure, rejected, shouldAbort,
                          horizontalSent, permitExpired))
    {
        if (permitExpired)
        {
            // 许可在发送阶段失效。若第一轴已经发出，设备**可能**已经
            // 开始运动：必须进入停止收口，不能按“未受理”静默返回。
            // 停止本身不受许可期限限制。
            setError(data::kErrTurntableMotionInhibited, failure, false,
                     "move/permit_expired");
            if (horizontalSent)
            {
                performStop(0, "许可到期后补发停止",
                            "move/permit_expired", false);
            }
            error = failure;
            return false;
        }
        if (shouldAbort && shouldAbort())
        {
            // 停止屏障已取消本次命令。⚠ 若第一轴已经发出，设备**可能
            // 已经开始运动** —— 取消不等于“什么都没发生”。必须补发
            // 停止帧；否则屏障里的 session_->stop() 会因 moving_ 仍为
            // false 而直接返回，最终一条停止帧都没有。
            if (horizontalSent)
            {
                performStop(0, "部分发送后被取消", "move/cancel_horizontal",
                            false);
            }
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

    if (!sendAndAwaitEcho(verticalFrame, failure, rejected, shouldAbort,
                          verticalSent, permitExpired))
    {
        if (permitExpired)
        {
            // 水平轴已经交付：部分发送后许可失效，必须补发停止帧。
            setError(data::kErrTurntableMotionInhibited, failure, false,
                     "move/permit_expired");
            performStop(0, "许可到期后补发停止", "move/permit_expired",
                        false);
            error = failure;
            return false;
        }
        if (shouldAbort && shouldAbort())
        {
            // 水平轴已交付（或至少已发出）：必须请求停止，不能留下
            // 单轴运动。这正是“部分发送取消”的反例路径。
            performStop(0, "部分发送后被取消", "move/cancel_vertical", false);
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
    motion_.store(data::TurntableMotionState::MOVING);
    armWatchdog(azimuthTarget, elevationTarget);
    return true;
}

void PelcoDSession::pollOnce(const std::function<bool()>& shouldAbort)
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
        // stopForFault 按“停止是否交付”写当前错误：交付时是原始码，
        // 未交付时是更严重的 2005。只有交付时才把 2008 置为锁存；
        // 否则会用 2008 覆盖 2005，把“停止没发出去”伪装成单纯超时。
        stopForFault(data::kErrTurntableMotionWatchdog,
                     "移动看门狗超时", "watchdog");
        if (stopFrameDelivered_.load())
        {
            setError(data::kErrTurntableMotionWatchdog,
                     "移动看门狗超时", true, "watchdog");
        }
        motion_.store(data::TurntableMotionState::ERROR);
        return;
    }

    double azimuth = 0.0;
    double elevation = 0.0;
    std::string azError;
    std::string elError;

    // 轮询周期内**不重试**：下一个周期就是重试。一次丢包不该把
    // 50 ms 的周期拖成 150 ms。
    //
    // 停止屏障在**每个接收切片**前检查：一次双轴轮询的最坏时长是
    // 2 × replyTimeoutMs，不能让它把停止请求整段拖住。
    bool aborted = false;
    const bool gotAzimuth =
        queryAngle(true, azimuth, azError, 0, shouldAbort, &aborted);
    if (aborted)
    {
        return;   // 停止优先；本轮不发布、不计失败
    }
    const bool gotElevation =
        queryAngle(false, elevation, elError, 0, shouldAbort, &aborted);
    if (aborted)
    {
        return;
    }

    // C4：两轴必须同轮。任一轴缺失 ⇒ 整个样本无效，**不得**把
    // 新方位角与旧俯仰角拼在一起发布。
    if (!gotAzimuth || !gotElevation)
    {
        ++consecutiveFailures_;
        stableCount_ = 0;
        hasLastRead_ = false;

        // 没有新鲜样本时不能继续声称 STABLE。
        data::TurntableMotionState nextMotion = motion_.load();
        if (nextMotion == data::TurntableMotionState::STABLE)
        {
            nextMotion = data::TurntableMotionState::IDLE;
        }

        if (consecutiveFailures_ >= config_.commFailureThreshold &&
            !errorLatched_.load())
        {
            const std::string text =
                "连续 " + std::to_string(consecutiveFailures_) +
                " 个轮询周期无有效角度回复";
            setFirstFault(data::kErrTurntableComm, text, "poll/lost_link");
            // 同看门狗：stopForFault 已按“停止是否交付”写好当前错误；
            // 停止未交付（2005）优先于本故障码，不能被它覆盖。
            stopForFault(data::kErrTurntableComm, text, "poll/lost_link");
            // 与看门狗同一收口：停止已交付时把失联错误本身锁存，
            // 否则锁存的是 stopForFault 写下的更严重 2005。
            // 先前这里无条件不锁存，导致恢复读后 motion 回到 IDLE、
            // move() 又被接受 —— 失联故障没有留下任何拒绝依据。
            if (stopFrameDelivered_.load())
            {
                setError(data::kErrTurntableComm, text, true,
                         "poll/lost_link");
            }
            nextMotion = data::TurntableMotionState::ERROR;
        }
        // 无效样本与它对应的 motion 一起发布：读端不会拿到“上一轮角度
        // 仍然有效”的样本配一个已经撤销的 STABLE。
        publishSample(azimuth_.load(), elevation_.load(), now, false,
                      nextMotion);
        return;
    }

    consecutiveFailures_ = 0;
    data::TurntableMotionState nextMotion;
    if (stopPending_ && stopFrameDelivered_)
    {
        nextMotion = handleStopObservation(azimuth, elevation, now);
    }
    else
    {
        nextMotion = updateMotion(azimuth, elevation);
    }
    // 本轮样本与由它得出的 motion 在同一 seqlock 窗口发布。
    publishSample(azimuth, elevation, now, true, nextMotion);
}

bool PelcoDSession::performStop(int faultCode, const std::string& faultMessage,
                                const std::string& context,
                                bool recordFirstFault, bool singleAttempt)
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
    if (stopBudgetSpent_ && !stopRetryArmed_)
    {
        // 本次停止请求的预算已经耗尽（例如取消补帧已用掉全部重试）。
        // 停止屏障与取消补帧共享一份预算，不能因为换了一个调用点
        // 就再发一轮 —— 只有新的外部 stop(true) 才能重新武装。
        return false;
    }
    if (stopRetryArmed_)
    {
        stopSent_ = false;
    }

    // 这次停止处理属于当前运动事务。即使发送失败，归属也要如实记录，
    // 读端才能区分“上个事务的已停止”与“本事务的停止结果”。
    stopFactsTransactionId_.store(motionTransactionId_.load());

    const std::uint8_t address =
        static_cast<std::uint8_t>(config_.pelcoAddress);
    const PelcoDFrame stopFrame = makeStop(address);

    const std::uint64_t deadline =
        nowNs() + msToNs(config_.stopRetryWindowMs);
    bool delivered = false;
    std::string lastFailure;

    // 验收静止停止是“证明线上能发出这一帧”的验收操作，不是故障停止：
    // 只尝试一次，首次失败即返回；不得借用普通停止的重试预算。
    const int maxAttempts =
        singleAttempt ? 1 : (config_.stopRetryCount + 1);
    for (int attempt = 0; attempt < maxAttempts; ++attempt)
    {
        if (sendFrame(stopFrame, lastFailure))
        {
            delivered = true;
            break;
        }
        if (attempt + 1 >= maxAttempts)
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
        stopBudgetSpent_        = false;
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

    if (singleAttempt)
    {
        // 单次发送失败：不重发、不把已在静止的会话置为 motionUncertain，
        // 也不占用普通停止的预算；只如实记录本次未交付。
        stopSent_               = false;
        stopFrameDelivered_     = false;
        stopObservedStationary_ = false;
        stopPending_            = false;
        setError(data::kErrTurntableStopNotDelivered,
                 "验收静止停止帧未交付（单次发送失败）：" + lastFailure,
                 false, context);
        return false;
    }

    // 窗口耗尽仍无法交付：2005 + motionUncertain，停止自动重试。
    stopBudgetSpent_      = true;
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

bool PelcoDSession::sendAcceptanceStopFrameAtRest(std::string& error)
{
    error.clear();
    if (!initialized_.load())
    {
        error = "会话未初始化，不能发送验收静止停止帧";
        return false;
    }
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        error = "只读会话不发送任何控制帧（含静止停止帧）";
        return false;
    }
    if (!acceptancePermitInstalled_)
    {
        error = "无验收许可：静止停止帧未授权";
        return false;
    }
    if (!acceptancePermit_.allowStopFrameAtRest)
    {
        error = "验收许可未授权静止停止帧（allow_stop_at_rest=0）";
        return false;
    }
    if (moving_ || stopPending_ || errorLatched_.load() ||
        motionUncertain_.load() ||
        motion_.load() == data::TurntableMotionState::MOVING ||
        motion_.load() == data::TurntableMotionState::ERROR)
    {
        error = "当前不是静止/无故障状态：请使用正常 stop()";
        return false;
    }

    // “静止”必须由同一轮新鲜有效双轴样本支持；不能只看旧的 motion 标志。
    double az = 0.0;
    double el = 0.0;
    std::uint64_t ts = 0;
    std::uint64_t cycle = 0;
    bool valid = false;
    data::TurntableMotionState motion = data::TurntableMotionState::IDLE;
    loadSample(az, el, ts, cycle, valid, motion);
    const std::uint64_t now = nowNs();
    if (!valid || ts == 0 || now < ts ||
        (now - ts) > msToNs(config_.sampleStaleMs))
    {
        error = "无新鲜有效双轴样本，不能确认静止";
        return false;
    }

    if (stopSent_ || acceptanceStopAtRestDelivered_)
    {
        error = "本次会话已交付过静止停止帧，不重复补帧";
        return false;
    }

    if (!performStop(0, "验收静止停止", "stop/acceptance_at_rest", false,
                     /*singleAttempt=*/true))
    {
        error = lastErrorText();
        if (error.empty())
        {
            error = "验收静止停止帧未交付";
        }
        return false;
    }
    // 独立于 stopSent_ 保留：中间发生过运动并清零 stopSent_ 后，
    // 本会话仍然不得再发第二帧 T1 停止帧。
    acceptanceStopAtRestDelivered_ = true;
    return true;
}

void PelcoDSession::stop(bool allowRearm)
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
    //
    // allowRearm=false 用于停止屏障：该屏障只是落实**同一次**停止请求，
    // 其取消补帧已经用掉预算时不得再发一轮。
    stopRetryArmed_ = allowRearm && motionUncertain_.load() &&
                      !stopFrameDelivered_.load();
    if (stopRetryArmed_)
    {
        stopBudgetSpent_ = false;
    }
    performStop(0, "显式停止", "stop/explicit", false);
    stopRetryArmed_ = false;

    if (motionUncertain_.load() && !errorLatched_.load())
    {
        motion_.store(data::TurntableMotionState::ERROR);
    }
}

void PelcoDSession::publishSample(double az, double el, std::uint64_t now,
                                  bool valid,
                                  data::TurntableMotionState motion)
{
    // seqlock：奇数为“写入中”。读端若在写入过程中取到奇序号，或前后
    // 序号不同，就重读 —— 因此不可能读到 (新方位角, 旧俯仰角) 这种
    // 跨轮拼接，即使每个字段本身都是原子的、数值也都合法。
    const std::uint64_t seq = sampleSeq_.load(std::memory_order_relaxed);
    sampleSeq_.store(seq + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    azimuth_.store(az, std::memory_order_relaxed);
    if (samplePublishHook_)
    {
        // 仅测试：确定性地把读端放进“方位角已写、俯仰角未写”的窗口。
        samplePublishHook_();
    }
    elevation_.store(el, std::memory_order_relaxed);
    sampleTimestampNs_.store(now, std::memory_order_relaxed);
    sampleValid_.store(valid, std::memory_order_relaxed);
    sampleCycleId_.store(sampleCycleId_.load(std::memory_order_relaxed) + 1,
                         std::memory_order_relaxed);
    // motion 由本轮样本得出，必须和样本字段处在**同一个**写入窗口：
    // 若在释放 seqlock 之后再单独写 motion_，读端可以读到本轮角度却
    // 配上下一轮的 STABLE/MOVING。
    motion_.store(motion, std::memory_order_relaxed);

    sampleSeq_.store(seq + 2, std::memory_order_release);

    if (samplePublishedHook_)
    {
        // 仅测试：完整发布之后进入。固定实现的 motion 已在窗口内写好；
        // 若被移到发布之后，本钩子期间读端会看到新样本 + 旧 motion。
        samplePublishedHook_();
    }
}

void PelcoDSession::loadSample(double& az, double& el, std::uint64_t& now,
                               std::uint64_t& cycle, bool& valid,
                               data::TurntableMotionState& motion) const
{
    for (;;)
    {
        const std::uint64_t begin =
            sampleSeq_.load(std::memory_order_acquire);
        if ((begin & 1u) != 0u)
        {
            continue;   // 写入中
        }

        az    = azimuth_.load(std::memory_order_relaxed);
        el    = elevation_.load(std::memory_order_relaxed);
        now   = sampleTimestampNs_.load(std::memory_order_relaxed);
        cycle = sampleCycleId_.load(std::memory_order_relaxed);
        valid = sampleValid_.load(std::memory_order_relaxed);
        motion = motion_.load(std::memory_order_relaxed);

        std::atomic_thread_fence(std::memory_order_acquire);
        if (sampleSeq_.load(std::memory_order_relaxed) == begin)
        {
            return;
        }
    }
}

bool PelcoDSession::withinTolerance(double az, double el) const
{
    return circularDeltaDeg(az, targetAz_) <=
               config_.arrivalToleranceDeg &&
           std::fabs(el - targetEl_) <= config_.arrivalToleranceDeg;
}

data::TurntableMotionState PelcoDSession::handleStopObservation(
    double az, double el, std::uint64_t now)
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
        return errorLatched_.load() ? motion_.load()
                                    : data::TurntableMotionState::IDLE;
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
        return data::TurntableMotionState::ERROR;
    }

    // 仍在观测窗口内：保持 MOVING，不谎报 IDLE/STABLE。
    return motion_.load();
}

data::TurntableMotionState PelcoDSession::updateMotion(double az, double el)
{
    if (errorLatched_.load() || motionUncertain_.load())
    {
        return motion_.load();
    }

    const bool inTolerance = withinTolerance(az, el);

    if (!moving_)
    {
        // C2：已到位后读数明显偏离 ⇒ **撤销** STABLE，不得一次置位后
        // 永久保持。STABLE 必须是“本轮样本满足判据”的结论。
        const data::TurntableMotionState current = motion_.load();
        const bool demote =
            current == data::TurntableMotionState::STABLE && !inTolerance;
        lastAz_      = az;
        lastEl_      = el;
        hasLastRead_ = true;
        return demote ? data::TurntableMotionState::IDLE : current;
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
        return data::TurntableMotionState::STABLE;
    }
    return data::TurntableMotionState::MOVING;
}

data::TurntableState PelcoDSession::snapshot() const
{
    data::TurntableState state;
    std::uint64_t ts    = 0;
    std::uint64_t cycle = 0;
    bool valid = false;
    data::TurntableMotionState motion = data::TurntableMotionState::IDLE;
    // 一次读出**同一轮**的方位/俯仰/时刻/有效性与 motion：两轴不得
    // 跨发布拼接，motion 也不得来自另一次发布（否则会出现“新角度 +
    // 上一轮的 STABLE”）。
    loadSample(state.azimuth, state.elevation, ts, cycle, valid, motion);
    if (snapshotReadHook_)
    {
        // 仅测试：确定性地把读端停在“样本与 motion 都已读出、尚未
        // 返回”的窗口，验证后续发布不会改变本次读数的语义。
        snapshotReadHook_();
    }
    state.motion = motion;

    if (initialized_.load())
    {
        const std::uint64_t now = nowNs();
        const bool stale =
            !valid || ts == 0 || now < ts ||
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
    data::TurntableMotionState motion = data::TurntableMotionState::IDLE;
    loadSample(d.sample.azimuth, d.sample.elevation,
               d.sample.sampleTimestampNs, d.sample.cycleId,
               d.sample.valid, motion);
    d.sample.source = d.sample.valid ? data::TurntableSampleSource::Poll
                                     : data::TurntableSampleSource::None;

    const std::uint64_t now = nowNs();
    const std::int64_t nowWall = wallNowMs();
    const std::uint64_t ts  = d.sample.sampleTimestampNs;
    d.sampleFresh = initialized_.load() && d.sample.valid && ts != 0 &&
                    now >= ts &&
                    (now - ts) <= msToNs(config_.sampleStaleMs);

    d.motionTransactionId    = motionTransactionId_.load();
    d.stopFactsTransactionId = stopFactsTransactionId_.load();
    d.stopFrameDelivered     = stopFrameDelivered_.load();
    d.stopObservedStationary = stopObservedStationary_.load();
    d.motionUncertain        = motionUncertain_.load();
    d.errorLatched           = errorLatched_.load();
    d.workModeTrust          = workModeTrust_.load();
    d.motionInhibited        = motionInhibited();

    d.acceptancePermitInstalled = acceptancePermitInstalled_;
    if (acceptancePermitInstalled_)
    {
        d.acceptancePermitScope     = acceptancePermit_.scopeId;
        d.acceptancePermitExpired   = nowWall >=
                                      acceptancePermit_.validUntilEpochMs;
        d.acceptancePermitRemaining =
            acceptancePermitRemaining_.load();
    }

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
    // 只读、无有效许可、许可未生效/过期/次数用尽、或单元测试门未开。
    return !motionPreauthorized();
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
