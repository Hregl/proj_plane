// ============================================================================
//  src/device/turntable/PelcoDTurntableController.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、§17（线程）、§18（tick 影响）、C-22
// ============================================================================

#include "device/turntable/PelcoDTurntableController.h"

#include <chrono>
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
    // session_ 在这里创建：其生命周期与传输一致，且构造只做值拷贝。
    session_ = std::make_unique<PelcoDSession>(config_, transport_);
}

PelcoDTurntableController::~PelcoDTurntableController()
{
    running_.store(false);
    waitCv_.notify_all();
    if (worker_.joinable())
    {
        worker_.join();
    }

    {
        std::lock_guard<std::mutex> lock(ioMutex_);
        // 仅在 MOVING 或有锁存错误时才会真正发帧（见 PelcoDSession::stop）。
        session_->stop();
        transport_->close();
    }
    initialized_.store(false);
}

void PelcoDTurntableController::setDiagnosticSink(
    std::function<void(const std::string&)> sink)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    diagnosticSink_ = std::move(sink);
}

void PelcoDTurntableController::setClock(std::function<std::uint64_t()> clock)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    clock_ = std::move(clock);
    session_->setClock(clock_);
}

bool PelcoDTurntableController::initialize()
{
    std::lock_guard<std::mutex> lock(ioMutex_);

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
    lastReportedError_.clear();

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

bool PelcoDTurntableController::move(const data::TurntableCommand& command)
{
    if (!initialized_.load())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(ioMutex_);
    std::string error;
    return session_->move(command, error);
}

data::TurntableState PelcoDTurntableController::state()
{
    // 无锁、无 I/O：这是主 tick 每 16 ms 调用的路径。
    return session_->snapshot();
}

void PelcoDTurntableController::stop()
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    session_->stop();
}

std::string PelcoDTurntableController::lastErrorText() const
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    return session_->lastErrorText();
}

data::ErrorInfo PelcoDTurntableController::lastError() const
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    return session_->lastError();
}

void PelcoDTurntableController::workerLoop()
{
    const int pollMs = config_.pollIntervalMs > 0 ? config_.pollIntervalMs : 1;

    while (running_.load())
    {
        std::string errorText;
        std::function<void(const std::string&)> sink;

        {
            std::lock_guard<std::mutex> lock(ioMutex_);
            session_->pollOnce();
            errorText = session_->lastErrorText();
            sink      = diagnosticSink_;
        }

        // sink 在锁外调用：它可能写日志、拿 Logger 自己的锁，
        // 不应因此把主线程的 move()/state() 挡在 ioMutex_ 外。
        if (sink && !errorText.empty() && errorText != lastReportedError_)
        {
            lastReportedError_ = errorText;
            sink(errorText);
        }

        std::unique_lock<std::mutex> wait(waitMutex_);
        waitCv_.wait_for(wait, std::chrono::milliseconds(pollMs),
                         [this] { return !running_.load(); });
    }
}

}  // namespace device
}  // namespace aircraft
