// ============================================================================
//  src/device/turntable/PelcoDTurntableController.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、§17（线程）、§18（tick 影响）、C-22/C-23
// ============================================================================

#include "device/turntable/PelcoDTurntableController.h"

#include <chrono>
#include <cmath>
#include <system_error>
#include <utility>

#include "device/turntable/PelcoDSession.h"

namespace aircraft
{
namespace device
{

PelcoDTurntableController::PelcoDTurntableController(
    const data::TurntableConfig& config,
    std::shared_ptr<IPelcoTransport> transport)
    : config_(config),
      transport_(transport ? std::move(transport) : makeUdpPelcoTransport())
{
    session_ = std::make_unique<PelcoDSession>(config_, transport_);
}

PelcoDTurntableController::~PelcoDTurntableController()
{
    running_.store(false);
    commandCv_.notify_all();
    if (worker_.joinable())
    {
        worker_.join();
    }

    // worker 已退出：此后的 session 调用不再有并发方。
    session_->stop();
    transport_->close();
    initialized_.store(false);
}

void PelcoDTurntableController::setDiagnosticSink(
    std::function<void(const std::string&)> sink)
{
    std::lock_guard<std::mutex> lock(diagMutex_);
    diagnosticSink_ = std::move(sink);
}

void PelcoDTurntableController::setClock(std::function<std::uint64_t()> clock)
{
    clock_ = std::move(clock);
    session_->setClock(clock_);
}

void PelcoDTurntableController::setTestMotionGate(bool allow)
{
    session_->setTestMotionGate(allow);
}

bool PelcoDTurntableController::initialize()
{
    if (initialized_.load())
    {
        return true;
    }

    std::string error;
    if (!session_->initialize(error))
    {
        return false;
    }

    initialized_.store(true);
    running_.store(true);

    try
    {
        worker_ = std::thread(&PelcoDTurntableController::workerLoop, this);
    }
    catch (const std::system_error&)
    {
        // 线程起不来 ⇒ 没有轮询，就没有 STABLE，也没有通信失败检测。
        // 与其返回 true 让上层以为"能等到位"，不如如实失败。
        running_.store(false);
        initialized_.store(false);
        session_->stop();
        transport_->close();
        return false;
    }
    return true;
}

bool PelcoDTurntableController::move(
    const data::TurntableCommand& command)
{
    if (!initialized_.load())
    {
        return false;
    }

    // 只读模式：不产生任何控制帧，连命令槽都不写。
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return false;
    }

    const bool finite = std::isfinite(command.azimuthCommand) &&
                        std::isfinite(command.elevationCommand);

    std::uint64_t seq = 0;
    std::optional<PendingMove> superseded;
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        seq = ++nextCommandSeq_;
        latestCommandSeq_ = seq;
        if (finite)
        {
            if (pendingMove_)
            {
                superseded = pendingMove_;  // 旧命令必须得到终结结果
            }
            pendingMove_ = PendingMove{command, seq};
        }
    }

    if (superseded)
    {
        setCommandOutcome(superseded->seq,
                          data::TurntableCommandOutcome::Superseded,
                          "命令被更新的运动命令覆盖");
    }
    if (!finite)
    {
        setCommandOutcome(seq, data::TurntableCommandOutcome::Failed,
                          "命令含非有限角，未受理");
        return false;
    }

    commandCv_.notify_all();
    // ⚠ 这里**不**等待回显：true 仅表示已受理。
    return true;
}

data::TurntableState PelcoDTurntableController::state()
{
    // 无锁、无 I/O：这是主 tick 每 16 ms 调用的路径。
    return session_->snapshot();
}

void PelcoDTurntableController::stop()
{
    if (!initialized_.load())
    {
        return;
    }
    // 只读模式：stop() 是无帧 no-op（C1）。
    if (config_.sendPolicy == data::TurntableSendPolicy::ReadOnly)
    {
        return;
    }

    std::optional<PendingMove> cancelled;
    std::uint64_t seq = 0;
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        seq = ++stopRequestSeq_;
        if (pendingMove_)
        {
            // 屏障之前的旧命令一律作废，停止后不得恢复执行。
            cancelled = pendingMove_;
            pendingMove_.reset();
        }
    }
    if (cancelled)
    {
        setCommandOutcome(cancelled->seq,
                          data::TurntableCommandOutcome::Cancelled,
                          "命令被停止屏障取消");
    }

    commandCv_.notify_all();

    // 等 worker 处理到该停止。上限由重试窗口给出；不把“观测到静止”
    // 的窗口算进来 —— 那是停止之后的事实，不是 stop() 返回的前提。
    const auto waitMs = std::chrono::milliseconds(
        config_.stopRetryWindowMs > 0 ? config_.stopRetryWindowMs + 1000
                                      : 1000);
    std::unique_lock<std::mutex> lock(stopWaitMutex_);
    stopWaitCv_.wait_for(lock, waitMs, [this, seq] {
        return stopCompletedSeq_.load() >= seq;
    });
}

data::TurntableDiagnostics PelcoDTurntableController::diagnostics() const
{
    data::TurntableDiagnostics d = session_->diagnostics();
    {
        std::lock_guard<std::mutex> lock(diagMutex_);
        d.lastCommandSeq     = lastCommandSeq_;
        d.lastCommandOutcome = lastCommandOutcome_;
        d.lastCommandMessage = lastCommandMessage_;
    }
    d.stopRequestSeq   = stopRequestSeq_.load();
    d.stopCompletedSeq = stopCompletedSeq_.load();
    return d;
}

std::string PelcoDTurntableController::lastErrorText() const
{
    return session_->lastErrorText();
}

data::ErrorInfo PelcoDTurntableController::lastError() const
{
    return session_->lastError();
}

bool PelcoDTurntableController::commandSuperseded(std::uint64_t seq) const
{
    std::lock_guard<std::mutex> lock(commandMutex_);
    return latestCommandSeq_ > seq;
}

void PelcoDTurntableController::setCommandOutcome(
    std::uint64_t seq, data::TurntableCommandOutcome outcome,
    const std::string& message)
{
    std::lock_guard<std::mutex> lock(diagMutex_);
    lastCommandSeq_     = seq;
    lastCommandOutcome_ = outcome;
    lastCommandMessage_ = message;
}

void PelcoDTurntableController::handleCommand(const PendingMove& move)
{
    std::string error;
    const bool ok = session_->move(
        move.command, error, [this, &move] {
            return stopRequestSeq_.load() > stopCompletedSeq_.load() ||
                   commandSuperseded(move.seq);
        });

    if (stopRequestSeq_.load() > stopCompletedSeq_.load())
    {
        setCommandOutcome(move.seq, data::TurntableCommandOutcome::Cancelled,
                          "命令被停止屏障取消");
        return;
    }
    if (commandSuperseded(move.seq))
    {
        setCommandOutcome(move.seq,
                          data::TurntableCommandOutcome::Superseded,
                          "命令被更新的运动命令覆盖");
        return;
    }
    if (ok)
    {
        setCommandOutcome(move.seq, data::TurntableCommandOutcome::Delivered,
                          "两轴指令已交付");
    }
    else
    {
        setCommandOutcome(move.seq, data::TurntableCommandOutcome::Failed,
                          error.empty() ? std::string("转台指令失败")
                                        : error);
    }
}

void PelcoDTurntableController::workerLoop()
{
    const int pollMs = config_.pollIntervalMs > 0 ? config_.pollIntervalMs : 1;

    while (running_.load())
    {
        {
            std::unique_lock<std::mutex> lock(commandMutex_);
            commandCv_.wait_for(lock, std::chrono::milliseconds(pollMs), [this] {
                return !running_.load() || pendingMove_.has_value() ||
                       stopRequestSeq_.load() > stopCompletedSeq_.load();
            });
            if (!running_.load())
            {
                break;
            }
        }

        // ---- 停止屏障优先：清槽 → 停止 → 通告完成 ----
        const std::uint64_t stopSeq = stopRequestSeq_.load();
        if (stopSeq > stopCompletedSeq_.load())
        {
            std::optional<PendingMove> cancelled;
            {
                std::lock_guard<std::mutex> lock(commandMutex_);
                if (pendingMove_)
                {
                    cancelled = pendingMove_;
                    pendingMove_.reset();
                }
            }
            if (cancelled)
            {
                setCommandOutcome(cancelled->seq,
                                  data::TurntableCommandOutcome::Cancelled,
                                  "命令被停止屏障取消");
            }

            session_->stop();
            stopCompletedSeq_.store(stopSeq);
            stopWaitCv_.notify_all();
            continue;
        }

        // ---- 运动命令 ----
        std::optional<PendingMove> move;
        {
            std::lock_guard<std::mutex> lock(commandMutex_);
            if (pendingMove_)
            {
                move = pendingMove_;
                pendingMove_.reset();
            }
        }
        if (move)
        {
            handleCommand(*move);
            continue;  // 接着轮询，让角度快照尽快反映设备
        }

        // ---- 轮询 ----
        // 停止请求/退出必须在**每个接收切片**后可见：一次双轴轮询
        // 最坏等 2 × replyTimeoutMs，不能让它整段挡住停止屏障。
        session_->pollOnce([this] {
            return stopRequestSeq_.load() > stopCompletedSeq_.load() ||
                   !running_.load();
        });

        std::string errorText = session_->lastErrorText();
        std::function<void(const std::string&)> sink;
        {
            std::lock_guard<std::mutex> lock(diagMutex_);
            sink = diagnosticSink_;
        }

        // sink 在锁外调用：它可能写日志、拿 Logger 自己的锁。
        if (sink && !errorText.empty() && errorText != lastReportedError_)
        {
            lastReportedError_ = errorText;
            try
            {
                sink(errorText);
            }
            catch (...)
            {
                // 诊断回调抛异常不得导致 std::terminate。
            }
        }
    }

    // ---- 退出路径 ----
    session_->stop();
    transport_->close();
}

}  // namespace device
}  // namespace aircraft
