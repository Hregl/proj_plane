// ============================================================================
//  tests/device/PelcoDSessionTest.cpp
//
//  覆盖 SYS-10 V1.1 §6.5 的失败策略表：
//  · initialize 的成功/WorkMode 故障/实测超行程/配置非法；
//  · move 的回显、FF 重试、EE 不重试、部分轴失败立即 stop、本地超行程零发送；
//  · pollOnce 的 MOVING→STABLE、假稳定、单轴丢包、连续失败锁存、看门狗；
//  · stop 的幂等与"不清除锁存 ERROR"。
// ============================================================================

#include "FakePelcoTransport.h"
#include "device/turntable/PelcoDSession.h"
#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

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
    cfg.replyTimeoutMs    = 50;
    cfg.pollIntervalMs    = 50;
    cfg.stableReadCount   = 3;
    cfg.stableThresholdDeg    = 0.02;
    cfg.arrivalToleranceDeg   = 0.2;
    cfg.commFailureThreshold  = 5;
    cfg.useSpeedControl       = false;
    return cfg;
}

struct Fixture
{
    std::shared_ptr<FakePelcoTransport> fake =
        std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    std::unique_ptr<PelcoDSession> session;

    explicit Fixture(const data::TurntableConfig& cfg)
    {
        dev.transport = fake.get();
        dev.install();
        session = std::make_unique<PelcoDSession>(cfg, fake);
    }

    bool init()
    {
        std::string error;
        return session->initialize(error);
    }

    data::TurntableState snap() const { return session->snapshot(); }
};

bool isPositioning(const PelcoDFrame& frame)
{
    return (frame.data1 == 0x00 &&
            (frame.data2 == 0x4B || frame.data2 == 0x4D)) ||
           ((frame.data1 == 0x4B || frame.data1 == 0x4D) &&
            frame.data2 != 0x00);
}

bool isControl(const PelcoDFrame& frame)
{
    if (frame.data1 == 0x00 && frame.data2 == 0x00 &&
        frame.data3 == 0x00 && frame.data4 == 0x00)
    {
        return true;  // stop
    }
    return isPositioning(frame);
}

// ---------------------------------------------------------------------------
//  initialize
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, InitializeReadsBothAxesAndWorkMode)
{
    Fixture f(makeConfig());
    f.dev.azimuth   = 4.72;
    f.dev.elevation = -14.51;
    f.dev.workMode  = 0x00;

    EXPECT_TRUE(f.init());
    EXPECT_TRUE(f.session->initialized());
    EXPECT_FALSE(f.session->errorLatched());

    const auto state = f.snap();
    EXPECT_DOUBLE_EQ(state.azimuth, 4.72);
    EXPECT_DOUBLE_EQ(state.elevation, -14.51);
    EXPECT_EQ(state.motion, data::TurntableMotionState::IDLE);

    // 三帧查询，且没有一帧是控制指令 —— 只读启动的硬要求。
    EXPECT_EQ(f.fake->sent.size(), 3u);
    EXPECT_EQ(f.fake->countSent(isControl), 0);
    EXPECT_EQ(f.dev.stopFrames, 0);
}

TEST(PelcoDSessionTest, InitializeFailsOnNonZeroWorkMode)
{
    Fixture f(makeConfig());
    f.dev.workMode = 0x01;

    EXPECT_FALSE(f.init());
    EXPECT_FALSE(f.session->initialized());
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);
    EXPECT_TRUE(f.fake->closeCalls > 0);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

TEST(PelcoDSessionTest, InitializeFailsWhenMeasuredAngleOutsideRange)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.azimuthMin = -90.0;
    cfg.azimuthMax = 90.0;

    Fixture f(cfg);
    f.dev.azimuth = 100.0;

    EXPECT_FALSE(f.init());
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_TRUE(f.fake->closeCalls > 0);
}

TEST(PelcoDSessionTest, InitializeRejectsInconsistentConfigBeforeOpeningSocket)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.azimuthMax = cfg.azimuthMin;

    Fixture f(cfg);
    EXPECT_FALSE(f.init());
    EXPECT_EQ(f.fake->openCalls, 0);
    EXPECT_TRUE(f.session->errorLatched());
}

TEST(PelcoDSessionTest, MoveBeforeInitializeFailsWithoutSending)
{
    Fixture f(makeConfig());
    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 1.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
}

// ---------------------------------------------------------------------------
//  move
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, MoveSendsBothAxesAndAcceptsEcho)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_TRUE(f.session->move(command, error));

    EXPECT_EQ(f.fake->sent.size(), 2u);
    const PelcoDFrame h = FakePelcoTransport::decode(f.fake->sent[0]);
    const PelcoDFrame v = FakePelcoTransport::decode(f.fake->sent[1]);
    EXPECT_EQ(h.data1, 0x00);
    EXPECT_EQ(h.data2, 0x4B);
    EXPECT_EQ(v.data1, 0x00);
    EXPECT_EQ(v.data2, 0x4D);

    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
}

TEST(PelcoDSessionTest, MoveRetriesOnChecksumErrorMarker)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.positioningChecksumErrors = 1;  // 第一次定位回 FF FF FF FF

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_TRUE(f.session->move(command, error));

    int horizontalSends = 0;
    for (const auto& bytes : f.fake->sent)
    {
        const PelcoDFrame frame = FakePelcoTransport::decode(bytes);
        if (frame.data1 == 0x00 && frame.data2 == 0x4B)
        {
            ++horizontalSends;
        }
    }
    EXPECT_EQ(horizontalSends, 2);
    EXPECT_EQ(f.dev.stopFrames, 0);
}

TEST(PelcoDSessionTest, MoveDoesNotRetryOnRejectedMarker)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.rejectMoveAxis = 1;  // 水平定位回 EE EE EE EE

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));

    int horizontalSends = 0;
    for (const auto& bytes : f.fake->sent)
    {
        const PelcoDFrame frame = FakePelcoTransport::decode(bytes);
        if (frame.data1 == 0x00 && frame.data2 == 0x4B)
        {
            ++horizontalSends;
        }
    }
    EXPECT_EQ(horizontalSends, 1);
    EXPECT_EQ(f.dev.stopFrames, 1);
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);
    EXPECT_FALSE(f.session->errorLatched());
    EXPECT_NE(f.snap().motion, data::TurntableMotionState::MOVING);
}

TEST(PelcoDSessionTest, PartialAxisFailureSendsStop)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.rejectMoveAxis = 2;  // 水平接受，垂直拒绝

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_EQ(f.dev.stopFrames, 1);
    EXPECT_NE(f.snap().motion, data::TurntableMotionState::MOVING);
}

TEST(PelcoDSessionTest, LocalOverTravelSendsNoFrameAtAll)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.azimuthMin = -10.0;
    cfg.azimuthMax = 10.0;

    Fixture f(cfg);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 50.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableOverTravel);
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(f.dev.stopFrames, 0);
}

TEST(PelcoDSessionTest, AzimuthCommandWrapsIntoRange)
{
    Fixture f(makeConfig());  // 行程 -180..180
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 190.0;  // 等价于 -170
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_TRUE(f.session->move(command, error));

    const PelcoDFrame h = FakePelcoTransport::decode(f.fake->sent[0]);
    const std::uint16_t raw =
        static_cast<std::uint16_t>((h.data3 << 8) | h.data4);
    EXPECT_EQ(raw, 19000);
    EXPECT_DOUBLE_EQ(f.dev.azimuth, -170.0);
}

TEST(PelcoDSessionTest, SpeedControlUsesTable365AndRpm10)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.useSpeedControl = true;
    cfg.coarseSpeed     = 5.0;  // 5 °/s ⇒ 8.333 r/min ⇒ 8

    Fixture f(cfg);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_TRUE(f.session->move(command, error));
    ASSERT_EQ(f.fake->sent.size(), 2u);

    const PelcoDFrame h = FakePelcoTransport::decode(f.fake->sent[0]);
    EXPECT_EQ(h.data1, 0x4B);
    EXPECT_EQ(h.data2, 8);
}

// ---------------------------------------------------------------------------
//  pollOnce / STABLE / 看门狗
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, PollReachesStableAfterConfiguredReadCount)
{
    Fixture f(makeConfig());  // stableReadCount = 3
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // 第 1 次：建立 lastRead，计数仍为 0
    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    // 第 2、3 次：计数 1、2
    f.session->pollOnce();
    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    // 第 4 次：计数达到 3 ⇒ STABLE
    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::STABLE);
}

TEST(PelcoDSessionTest, PollDoesNotCallStableWhenFarFromTarget)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;

    data::TurntableCommand command;
    command.azimuthCommand   = 5.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // 读数恒定但离目标 1°：只满足"没变"，不满足"到位"。
    f.dev.azimuth = 4.0;
    for (int i = 0; i < 6; ++i)
    {
        f.session->pollOnce();
        EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    }
}

TEST(PelcoDSessionTest, PollSingleAxisDropDoesNotLatchError)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());

    f.dev.dropVertical = true;
    f.session->pollOnce();
    EXPECT_FALSE(f.session->errorLatched());
    EXPECT_NE(f.snap().motion, data::TurntableMotionState::ERROR);

    f.dev.dropVertical = false;
    f.session->pollOnce();
    EXPECT_FALSE(f.session->errorLatched());
}

TEST(PelcoDSessionTest, PollLatchesErrorAfterConsecutiveFailures)
{
    Fixture f(makeConfig());  // commFailureThreshold = 5
    ASSERT_TRUE(f.init());

    f.dev.dropHorizontal = true;
    f.dev.dropVertical   = true;

    for (int i = 0; i < 4; ++i)
    {
        f.session->pollOnce();
        EXPECT_FALSE(f.session->errorLatched());
    }

    f.session->pollOnce();  // 第 5 次
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);
    EXPECT_EQ(f.dev.stopFrames, 1);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

TEST(PelcoDSessionTest, WatchdogLatchesErrorAndStops)
{
    data::TurntableConfig cfg = makeConfig();
    Fixture f(cfg);

    std::uint64_t now = 0;
    f.session->setClock([&now]() { return now; });

    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // 10° / 12 °/s × 1.5 + 5 s = 6.25 s
    now = 7000000000ULL;  // 7 s
    f.session->pollOnce();

    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(f.dev.stopFrames, 1);
}

// ---------------------------------------------------------------------------
//  stop
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, StopIsNoOpWhenIdle)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.session->stop();
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.dev.stopFrames, 0);
}

TEST(PelcoDSessionTest, StopIsIdempotentAndDoesNotClearLatchedError)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.azimuthMax = 10.0;

    Fixture f(cfg);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 50.0;  // 本地拒绝 ⇒ 锁存 ERROR
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_FALSE(f.session->move(command, error));
    ASSERT_TRUE(f.session->errorLatched());

    f.session->stop();
    EXPECT_EQ(f.dev.stopFrames, 1);
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);

    f.session->stop();
    EXPECT_EQ(f.dev.stopFrames, 1);  // 不重复发送
    EXPECT_TRUE(f.session->errorLatched());
}

}  // namespace
}  // namespace device
}  // namespace aircraft
