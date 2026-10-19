// ============================================================================
//  tests/device/PelcoDTurntableControllerTest.cpp
//
//  覆盖后端线程模型（SYS-10 V1.1 §17/§18、裁决 C-23）：
//  · state() 只读快照 —— worker 阻塞在 UDP 上时也不阻塞主 tick；
//  · move() 只表示“已受理”，不在调用线程等回显；
//  · 停止屏障优先于排队命令，被取消的命令有终结结果；
//  · 诊断快照合并会话诊断与命令终结结果；
//  · sink 在锁外调用，抛异常不得 terminate；
//  · 只读控制器所有路径控制帧为 0；
//  · 析构 join 并关闭传输。
// ============================================================================

#include "FakePelcoTransport.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "gtest/gtest.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTimer>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace aircraft
{
namespace device
{
namespace
{

using test::FakePelcoDevice;
using test::FakePelcoTransport;

data::TurntableConfig makeConfig(bool motion = false)
{
    data::TurntableConfig cfg;
    cfg.protocol          = "pelco_d_udp";
    cfg.host              = "127.0.0.1";
    cfg.port              = 6666;
    cfg.pelcoAddress      = 1;
    cfg.azimuthMin        = -180.0;
    cfg.azimuthMax        = 180.0;
    cfg.elevationMin      = -90.0;
    cfg.elevationMax      = 90.0;
    cfg.coarseSpeed       = 5.0;
    cfg.fineSpeed         = 1.0;
    cfg.replyTimeoutMs    = 20;
    cfg.pollIntervalMs    = 5;
    cfg.stableReadCount   = 2;
    cfg.stableThresholdDeg    = 0.02;
    cfg.arrivalToleranceDeg   = 0.2;
    cfg.commFailureThreshold  = 2;
    cfg.useSpeedControl       = false;
    cfg.sampleStaleMs         = 500;
    cfg.stopTimeoutMs         = 200;
    cfg.stopRetryCount        = 2;
    cfg.stopRetryWindowMs     = 200;
    cfg.stopBackoffMs         = 0;
    cfg.sendPolicy = motion ? data::TurntableSendPolicy::MotionEnabled
                            : data::TurntableSendPolicy::ReadOnly;
    return cfg;
}

void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

template <typename Predicate>
bool waitFor(Predicate predicate, int timeoutMs = 1500)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (predicate())
        {
            return true;
        }
        sleepMs(2);
    }
    return predicate();
}

bool isControlFrame(const PelcoDFrame& frame)
{
    if (frame.data1 == 0x00 && frame.data2 == 0x00 &&
        frame.data3 == 0x00 && frame.data4 == 0x00)
    {
        return true;  // stop
    }
    return (frame.data1 == 0x00 &&
            (frame.data2 == 0x4B || frame.data2 == 0x4D)) ||
           ((frame.data1 == 0x4B || frame.data1 == 0x4D) &&
            frame.data2 != 0x00);
}

TEST(PelcoDTurntableControllerTest, StateIsCheapWhileWorkerBlocksOnTransport)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    // 阈值设得足够大：worker 会持续轮询并反复阻塞，
    // 而不是几个周期后就锁存 ERROR、不再进入 receive()。
    data::TurntableConfig cfg = makeConfig();
    cfg.commFailureThreshold  = 1000;

    PelcoDTurntableController controller(cfg, fake);
    ASSERT_TRUE(controller.initialize());

    fake->receiveBlockMs.store(200);
    dev.dropHorizontal = true;
    dev.dropVertical   = true;
    sleepMs(30);

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i)
    {
        const data::TurntableState state = controller.state();
        (void)state;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    // 100 次快照应接近 0 ms。若 state() 去拿 I/O 锁，worker 正持锁
    // 阻塞在 200 ms 的 UDP 等待上，这 100 次调用会被拖过 100 ms。
    EXPECT_LT(elapsed.count(), 100);
}

TEST(PelcoDTurntableControllerTest, MoveIsAcceptedWithoutWaitingForEcho)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    // 让 worker 阻塞在轮询接收上，确保 move() 无法搭便车等回显。
    fake->receiveBlockMs.store(100);
    dev.dropHorizontal = true;
    dev.dropVertical   = true;
    sleepMs(20);

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 5; ++i)
    {
        data::TurntableCommand command;
        command.azimuthCommand   = static_cast<double>(i);
        command.elevationCommand = 0.0;
        EXPECT_TRUE(controller.move(command));
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    // true 仅表示已受理：调用线程不得等待回显/停止。
    EXPECT_LT(elapsed.count(), 100);
    EXPECT_GT(controller.diagnostics().lastCommandSeq, 0u);
}

TEST(PelcoDTurntableControllerTest, StopBarrierCancelsQueuedMove)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    // worker 正在慢速轮询：此时排队的命令尚未执行。
    fake->receiveBlockMs.store(200);
    dev.dropHorizontal = true;
    dev.dropVertical   = true;
    sleepMs(20);

    data::TurntableCommand command;
    command.azimuthCommand   = 2.0;
    command.elevationCommand = 0.0;
    ASSERT_TRUE(controller.move(command));

    controller.stop();

    const auto d = controller.diagnostics();
    EXPECT_EQ(d.lastCommandOutcome, data::TurntableCommandOutcome::Cancelled);
    EXPECT_GE(d.stopCompletedSeq, d.stopRequestSeq);
    EXPECT_GT(d.stopRequestSeq, 0u);
}

TEST(PelcoDTurntableControllerTest, DiagnosticsReportDeliveredCommand)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    ASSERT_TRUE(controller.move(command));

    const bool delivered = waitFor([&controller] {
        return controller.diagnostics().lastCommandOutcome ==
               data::TurntableCommandOutcome::Delivered;
    });
    EXPECT_TRUE(delivered);

    const auto d = controller.diagnostics();
    EXPECT_GT(d.lastCommandSeq, 0u);
    EXPECT_EQ(d.stopRequestSeq, 0u);
    EXPECT_TRUE(d.sample.valid);
}

TEST(PelcoDTurntableControllerTest, WorkerDrivesMotionToStable)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    ASSERT_TRUE(controller.move(command));

    const bool stable = waitFor([&controller] {
        return controller.state().motion == data::TurntableMotionState::STABLE;
    });
    EXPECT_TRUE(stable);
}

TEST(PelcoDTurntableControllerTest, ReadOnlyControllerSendsNoControlFrames)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    {
        PelcoDTurntableController controller(makeConfig(), fake);
        ASSERT_TRUE(controller.initialize());

        fake->sent.clear();
        dev.dropHorizontal = true;
        dev.dropVertical   = true;
        sleepMs(100);       // 触发失联锁存
        controller.stop();  // 只读：无帧 no-op
    }

    EXPECT_EQ(fake->countSent(isControlFrame), 0);
    EXPECT_EQ(dev.stopFrames, 0);
}

TEST(PelcoDTurntableControllerTest, DestructorJoinsWorkerAndClosesTransport)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    data::TurntableConfig cfg = makeConfig();
    cfg.commFailureThreshold  = 1000;

    const auto start = std::chrono::steady_clock::now();
    {
        PelcoDTurntableController controller(cfg, fake);
        ASSERT_TRUE(controller.initialize());
        fake->receiveBlockMs.store(100);
        dev.dropHorizontal = true;
        dev.dropVertical   = true;
        sleepMs(20);
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    EXPECT_LT(elapsed.count(), 2000);
    EXPECT_GT(fake->closeCalls, 0);
}

TEST(PelcoDTurntableControllerTest, DiagnosticSinkReportsLatchedErrorOnce)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(), fake);

    std::atomic<int> calls{0};
    std::mutex messageMutex;
    std::string message;
    controller.setDiagnosticSink([&](const std::string& text) {
        {
            std::lock_guard<std::mutex> lock(messageMutex);
            message = text;
        }
        calls.fetch_add(1);
    });

    ASSERT_TRUE(controller.initialize());

    dev.dropHorizontal = true;
    dev.dropVertical   = true;

    EXPECT_TRUE(waitFor([&calls] { return calls.load() > 0; }));
    sleepMs(30);  // 错误文本未变时不得重复上报
    EXPECT_EQ(calls.load(), 1);
    std::lock_guard<std::mutex> lock(messageMutex);
    EXPECT_FALSE(message.empty());
    EXPECT_TRUE(message.find("连续") != std::string::npos);
}

TEST(PelcoDTurntableControllerTest, ThrowingSinkDoesNotTerminateWorker)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(), fake);

    std::atomic<int> calls{0};
    controller.setDiagnosticSink([&calls](const std::string&) {
        calls.fetch_add(1);
        throw std::runtime_error("sink 故意抛异常");
    });

    ASSERT_TRUE(controller.initialize());
    dev.dropHorizontal = true;
    dev.dropVertical   = true;

    EXPECT_TRUE(waitFor([&calls] { return calls.load() > 0; }));

    // worker 仍存活：快照仍可读，且锁存的是通信错误。
    const data::TurntableState state = controller.state();
    EXPECT_EQ(state.motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(controller.lastError().code, data::kErrTurntableComm);
}

TEST(PelcoDTurntableControllerTest, ConcurrentMoveAndWorkerDoNotDeadlock)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    for (int i = 0; i < 10; ++i)
    {
        data::TurntableCommand command;
        command.azimuthCommand   = (i % 2 == 0) ? 1.0 : 0.5;
        command.elevationCommand = 0.0;
        EXPECT_TRUE(controller.move(command));
        sleepMs(2);
        if (i % 3 == 0)
        {
            controller.stop();
        }
    }

    // 没有死锁：停止屏障都得到应答。
    const auto d = controller.diagnostics();
    EXPECT_GE(d.stopCompletedSeq, d.stopRequestSeq);
}

TEST(PelcoDTurntableControllerTest, MoveBeforeInitializeFails)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    EXPECT_FALSE(controller.move(command));

    const data::TurntableState state = controller.state();
    EXPECT_EQ(state.motion, data::TurntableMotionState::IDLE);
    EXPECT_DOUBLE_EQ(state.azimuth, 0.0);
}

// A3 / S2：生产调用路径在真实 Qt 事件循环里。
//   · 界面线程调用 move() 必须立即返回（不在调用线程等回显）；
//   · 同一事件循环中的定时器仍能派发 stop() 请求；
//   · 后台 worker 阻塞在接收上不影响上述两点。
// 只测两个后台线程的并发不能覆盖“GUI 线程是否被回显等待卡住”。
TEST(PelcoDTurntableControllerTest, QtEventLoopDispatchesStopWhileMoveInFlight)
{
    int argc = 1;
    char appName[] = "PelcoDTest";
    char* argv[] = {appName, nullptr};
    QCoreApplication app(argc, argv);

    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(true), fake);
    controller.setTestMotionGate(true);
    ASSERT_TRUE(controller.initialize());

    // worker 阻塞在轮询接收上：若 move() 在调用线程等回显，界面线程会被拖住。
    fake->receiveBlockMs.store(20);
    dev.dropHorizontal = true;
    dev.dropVertical   = true;
    sleepMs(15);

    std::atomic<int> heartbeats{0};
    std::atomic<long long> moveMs{-1};
    std::atomic<long long> stopMs{-1};
    std::atomic<bool> stopDispatched{false};

    QTimer heartbeat;
    heartbeat.setInterval(5);
    QObject::connect(&heartbeat, &QTimer::timeout,
                     [&heartbeats]() { ++heartbeats; });
    heartbeat.start();

    QTimer::singleShot(0, [&]() {
        data::TurntableCommand command;
        command.azimuthCommand   = 1.0;
        command.elevationCommand = 0.0;
        QElapsedTimer timer;
        timer.start();
        EXPECT_TRUE(controller.move(command));
        moveMs.store(timer.elapsed());
    });

    // 关键：停止请求由事件循环派发，而不是在 move() 返回前同步调用。
    QTimer::singleShot(20, [&]() {
        stopDispatched.store(true);
        QElapsedTimer timer;
        timer.start();
        controller.stop();
        stopMs.store(timer.elapsed());
    });

    QTimer::singleShot(200, [&]() { app.quit(); });
    app.exec();
    heartbeat.stop();

    EXPECT_TRUE(stopDispatched.load());
    EXPECT_GE(moveMs.load(), 0);
    EXPECT_LT(moveMs.load(), 50);   // 立即返回，不等 20 ms 接收切片后的回显
    EXPECT_GE(stopMs.load(), 0);
    EXPECT_LT(stopMs.load(), 400);  // 屏障在 worker 处理到停止后返回
    EXPECT_GE(heartbeats.load(), 3);

    const auto d = controller.diagnostics();
    EXPECT_GT(d.stopRequestSeq, 0u);
    EXPECT_GE(d.stopCompletedSeq, d.stopRequestSeq);
}

}  // namespace
}  // namespace device
}  // namespace aircraft
