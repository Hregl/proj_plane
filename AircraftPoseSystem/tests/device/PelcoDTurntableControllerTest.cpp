// ============================================================================
//  tests/device/PelcoDTurntableControllerTest.cpp
//
//  覆盖后端线程模型（SYS-10 V1.1 §17/§18）：
//  · state() 只读快照 —— worker 阻塞在 UDP 上时也不阻塞主 tick；
//  · worker 轮询推进 STABLE；
//  · 析构 join、关闭传输；
//  · 诊断 sink 只在 worker 线程、错误文本变化时调用一次。
// ============================================================================

#include "FakePelcoTransport.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "gtest/gtest.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
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

data::TurntableConfig makeConfig()
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
    return cfg;
}

void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
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

    // 让下一次轮询真正阻塞在 receive() 上。
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

    // 100 次快照应接近 0 ms。若 state() 去拿 ioMutex_，worker 正持锁
    // 阻塞在 200 ms 的 UDP 等待上，这 100 次调用会被拖过 100 ms。
    EXPECT_LT(elapsed.count(), 100);
}

TEST(PelcoDTurntableControllerTest, WorkerDrivesMotionToStable)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(), fake);
    ASSERT_TRUE(controller.initialize());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    ASSERT_TRUE(controller.move(command));

    bool stable = false;
    for (int i = 0; i < 200; ++i)
    {
        if (controller.state().motion == data::TurntableMotionState::STABLE)
        {
            stable = true;
            break;
        }
        sleepMs(5);
    }
    EXPECT_TRUE(stable);
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
    EXPECT_TRUE(fake->closeCalls > 0);
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

    for (int i = 0; i < 200 && calls.load() == 0; ++i)
    {
        sleepMs(5);
    }

    EXPECT_EQ(calls.load(), 1);
    std::lock_guard<std::mutex> lock(messageMutex);
    EXPECT_FALSE(message.empty());
    EXPECT_TRUE(message.find("连续") != std::string::npos);
}

TEST(PelcoDTurntableControllerTest, ConcurrentMoveAndWorkerDoNotDeadlock)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(), fake);
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
    EXPECT_EQ(controller.lastError().code, 0);
}

TEST(PelcoDTurntableControllerTest, MoveBeforeInitializeFails)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    PelcoDTurntableController controller(makeConfig(), fake);

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    EXPECT_FALSE(controller.move(command));

    const data::TurntableState state = controller.state();
    EXPECT_EQ(state.motion, data::TurntableMotionState::IDLE);
    EXPECT_DOUBLE_EQ(state.azimuth, 0.0);
}

}  // namespace
}  // namespace device
}  // namespace aircraft
