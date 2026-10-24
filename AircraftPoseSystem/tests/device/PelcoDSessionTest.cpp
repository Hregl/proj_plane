// ============================================================================
//  tests/device/PelcoDSessionTest.cpp
//
//  覆盖裁决 C-23 的验收项：
//  · C1 只读模式在正常/失联/错误/退出路径上控制帧恒为 0；
//  · C2 偏离/单轴失效/过期均撤销 STABLE，过期报 ERROR；
//  · C3 三类回复（水平角/俯仰角/工作模式）的地址都必须核对；
//  · C4 两轴跨轮样本不得发布，样本带轮次/时刻/有效性；
//  · NaN/Inf、宽行程、停止 2005/2006、重试窗口、看门狗、首因与重臂。
// ============================================================================

#include "FakePelcoTransport.h"
#include "device/turntable/PelcoDSession.h"
#include "gtest/gtest.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
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
    cfg.stableReadCount   = 3;
    cfg.stableThresholdDeg    = 0.02;
    cfg.arrivalToleranceDeg   = 0.2;
    cfg.commFailureThreshold  = 5;
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

struct Fixture
{
    std::shared_ptr<FakePelcoTransport> fake =
        std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    std::unique_ptr<PelcoDSession> session;

    explicit Fixture(const data::TurntableConfig& cfg, bool motion = false)
    {
        dev.transport = fake.get();
        dev.install();
        session = std::make_unique<PelcoDSession>(cfg, fake);
        session->setTestMotionGate(motion);
    }

    bool init()
    {
        std::string error;
        return session->initialize(error);
    }

    data::TurntableState snap() const { return session->snapshot(); }
    data::TurntableDiagnostics diag() const { return session->diagnostics(); }
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
//  initialize / C3 地址校验
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

    const auto d = f.diag();
    EXPECT_TRUE(d.sample.valid);
    EXPECT_TRUE(d.sampleFresh);
    EXPECT_EQ(d.sample.cycleId, 1u);
    EXPECT_GT(d.sample.sampleTimestampNs, 0u);
    // 0xE0 回复与查询逐字节相同 ⇒ 只能是 EchoOnly，不得升级为“确认正常”。
    EXPECT_EQ(d.workModeTrust, data::WorkModeTrust::EchoOnly);

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
    EXPECT_EQ(f.diag().workModeTrust, data::WorkModeTrust::ReplyNonZero);
    EXPECT_TRUE(f.fake->closeCalls > 0);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

// 0x00 且与查询字节不同：只能证明“报文不同”，不能升级为正常模式。
TEST(PelcoDSessionTest, IndependentWorkModeReplyStaysUnknown)
{
    Fixture f(makeConfig());
    f.dev.workMode            = 0x00;
    f.dev.workModeReplyData3  = 0x01;  // 与查询不再逐字节相同

    EXPECT_TRUE(f.init());
    EXPECT_EQ(f.diag().workModeTrust, data::WorkModeTrust::Unknown);
}

// 工作模式事务没有有效回复（地址错/丢包）⇒ NoReply，且初始化失败。
TEST(PelcoDSessionTest, MissingWorkModeReplyReportsNoReply)
{
    data::TurntableConfig cfg = makeConfig();
    cfg.replyTimeoutMs = 10;   // 缩短重试等待

    Fixture f(cfg);
    f.dev.dropWorkMode = true;

    EXPECT_FALSE(f.init());
    EXPECT_FALSE(f.session->initialized());
    EXPECT_EQ(f.diag().workModeTrust, data::WorkModeTrust::NoReply);
    EXPECT_TRUE(f.session->errorLatched());
}

// Unknown 不是运动许可：即使 MotionEnabled，没有测试授权也不得发运动帧。
TEST(PelcoDSessionTest, UnknownWorkModeCannotMoveWithoutAuthorization)
{
    Fixture f(makeConfig(true));   // MotionEnabled，但不打开测试门
    f.dev.workMode           = 0x00;
    f.dev.workModeReplyData3 = 0x01;
    ASSERT_TRUE(f.init());
    EXPECT_EQ(f.diag().workModeTrust, data::WorkModeTrust::Unknown);
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
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

// C3：请求地址 1、只收到地址 2 的回复 ⇒ 初始化不得成功。
TEST(PelcoDSessionTest, HorizontalReplyFromWrongAddressCannotInitialize)
{
    Fixture f(makeConfig());
    f.dev.horizontalReplyAddress = 0x02;

    EXPECT_FALSE(f.init());
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_GT(f.fake->closeCalls, 0);
}

TEST(PelcoDSessionTest, VerticalReplyFromWrongAddressCannotInitialize)
{
    Fixture f(makeConfig());
    f.dev.verticalReplyAddress = 0x02;

    EXPECT_FALSE(f.init());
    EXPECT_TRUE(f.session->errorLatched());
}

TEST(PelcoDSessionTest, WorkModeReplyFromWrongAddressCannotInitialize)
{
    Fixture f(makeConfig());
    f.dev.workModeReplyAddress = 0x02;

    EXPECT_FALSE(f.init());
    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_FALSE(f.session->initialized());
}

TEST(PelcoDSessionTest, MoveBeforeInitializeFailsWithoutSending)
{
    Fixture f(makeConfig(true), true);
    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 1.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
}

// ---------------------------------------------------------------------------
//  C1：只读门禁 —— 控制帧恒为 0
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, ReadOnlyMoveIsRejectedWithoutAnyControlFrame)
{
    Fixture f(makeConfig());  // ReadOnly
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    EXPECT_FALSE(f.session->move(command, error));

    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.dev.stopFrames, 0);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
    EXPECT_EQ(f.diag().lastCommandOutcome,
              data::TurntableCommandOutcome::None);
}

TEST(PelcoDSessionTest, ReadOnlyLostLinkSendsNoControlFrame)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.dev.dropHorizontal = true;
    f.dev.dropVertical   = true;
    for (int i = 0; i < 6; ++i)
    {
        f.session->pollOnce();
    }

    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);

    // 失联后**上层会走失败收口**（MeasurementController::failWith →
    // turntable_.stop()）。错误已锁存时 stop() 不再命中"未运动"的早退，
    // 故这里必须单独验证：只读模式下这条路径同样零控制帧。
    f.session->stop();

    EXPECT_EQ(f.fake->countSent(isControl), 0);
    EXPECT_EQ(f.dev.stopFrames, 0);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

TEST(PelcoDSessionTest, ReadOnlyStopIsFrameFreeNoOp)
{
    Fixture f(makeConfig());
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.session->stop();
    f.session->stop();

    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.dev.stopFrames, 0);
    EXPECT_FALSE(f.diag().stopFrameDelivered);
}

// ---------------------------------------------------------------------------
//  MotionEnabled（仅 fake transport 单测）：命令与停止
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, MotionGateRejectsWithoutTestAuthorization)
{
    Fixture f(makeConfig(true));  // MotionEnabled，但不打开测试门
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
}

TEST(PelcoDSessionTest, MoveSendsBothAxesAndAcceptsEcho)
{
    Fixture f(makeConfig(true), true);
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
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.positioningChecksumErrors = 1;

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
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.rejectMoveAxis = 1;

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
    // 停止帧已交付但尚未观测到静止：不得谎报 IDLE/STABLE。
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    EXPECT_TRUE(f.diag().stopFrameDelivered);
    EXPECT_FALSE(f.diag().stopObservedStationary);
}

TEST(PelcoDSessionTest, PartialAxisFailureSendsStop)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.dev.rejectMoveAxis = 2;  // 水平接受，垂直拒绝

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_EQ(f.dev.stopFrames, 1);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
}

TEST(PelcoDSessionTest, LocalOverTravelSendsNoFrameAtAll)
{
    data::TurntableConfig cfg = makeConfig(true);
    cfg.azimuthMin = -10.0;
    cfg.azimuthMax = 10.0;

    Fixture f(cfg, true);
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

// 原报告第六条：错误已锁存，再调用 move() 会把错误码和文本清空。
// 锁存后必须拒绝运动、零运动帧，且原错误码/文本原样保留。
TEST(PelcoDSessionTest, MoveAfterLatchedErrorPreservesOriginalError)
{
    data::TurntableConfig cfg = makeConfig(true);
    cfg.azimuthMax = 10.0;

    Fixture f(cfg, true);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 50.0;  // 本地越程 ⇒ 2002 锁存
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_FALSE(f.session->move(command, error));
    ASSERT_TRUE(f.session->errorLatched());

    const data::ErrorInfo first = f.session->lastError();
    ASSERT_EQ(first.code, data::kErrTurntableOverTravel);
    ASSERT_FALSE(first.message.empty());
    f.fake->sent.clear();

    command.azimuthCommand = 1.0;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.dev.stopFrames, 0);
    EXPECT_EQ(error, first.message);

    const data::ErrorInfo after = f.session->lastError();
    EXPECT_EQ(after.code, first.code);
    EXPECT_EQ(after.message, first.message);
    EXPECT_EQ(after.timestampNs, first.timestampNs);
}

TEST(PelcoDSessionTest, NaNCommandIsRejectedBeforeAnyByteIsSent)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = std::numeric_limits<double>::quiet_NaN();
    command.elevationCommand = 0.0;
    std::string error;
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableInvalidCommand);
    EXPECT_TRUE(f.session->errorLatched());

    command.azimuthCommand   = 0.0;
    command.elevationCommand = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(f.session->move(command, error));
    EXPECT_TRUE(f.fake->sent.empty());
}

TEST(PelcoDSessionTest, NaNSpeedConfigFailsBeforeOpeningSocket)
{
    data::TurntableConfig cfg = makeConfig(true);
    cfg.useSpeedControl = true;
    cfg.coarseSpeed     = std::numeric_limits<double>::quiet_NaN();

    Fixture f(cfg, true);
    EXPECT_FALSE(f.init());
    EXPECT_EQ(f.fake->openCalls, 0);
    EXPECT_TRUE(f.session->errorLatched());
}

TEST(PelcoDSessionTest, AzimuthCommandWrapsIntoRange)
{
    Fixture f(makeConfig(true), true);
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
    data::TurntableConfig cfg = makeConfig(true);
    cfg.useSpeedControl = true;
    cfg.coarseSpeed     = 5.0;  // 5 °/s ⇒ 8.333 r/min ⇒ 8

    Fixture f(cfg, true);
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

TEST(PelcoDSessionTest, LowSpeedOnePointFiveDegreeUsesFieldValueTwo)
{
    // T3 预备：1.5° 位移在 >1.0° 分支选 coarse_speed。现场若要 1.2 °/s，
    // coarse 与 fine 都必须配成 1.2；帧内转速字段应为 2，而不是 12。
    data::TurntableConfig cfg = makeConfig(true);
    cfg.useSpeedControl = true;
    cfg.coarseSpeed     = 1.2;
    cfg.fineSpeed       = 1.2;

    Fixture f(cfg, true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    data::TurntableCommand command;
    command.azimuthCommand   = 1.5;
    command.elevationCommand = 0.0;

    std::string error;
    EXPECT_TRUE(f.session->move(command, error));
    ASSERT_EQ(f.fake->sent.size(), 2u);

    const PelcoDFrame h = FakePelcoTransport::decode(f.fake->sent[0]);
    EXPECT_EQ(h.data1, 0x4B);
    EXPECT_EQ(h.data2, 2);
    const PelcoDFrame v = FakePelcoTransport::decode(f.fake->sent[1]);
    EXPECT_EQ(v.data1, 0x4D);
    EXPECT_EQ(v.data2, 2);
}

// ---------------------------------------------------------------------------
//  C2：STABLE 撤销 / 样本过期
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, PollReachesStableAfterConfiguredReadCount)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    f.session->pollOnce();
    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);
    f.session->pollOnce();
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::STABLE);
}

TEST(PelcoDSessionTest, StableIsRevokedWhenMeasuredAngleDeviates)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    for (int i = 0; i < 4; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_EQ(f.snap().motion, data::TurntableMotionState::STABLE);

    // 已到位后读数明显偏离：STABLE 必须撤销，不得锁存。
    f.dev.azimuth = 5.0;
    f.session->pollOnce();
    EXPECT_NE(f.snap().motion, data::TurntableMotionState::STABLE);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::IDLE);
}

TEST(PelcoDSessionTest, SingleAxisInvalidRevokesStable)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));
    for (int i = 0; i < 4; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_EQ(f.snap().motion, data::TurntableMotionState::STABLE);

    f.dev.dropVertical = true;
    f.session->pollOnce();

    EXPECT_NE(f.snap().motion, data::TurntableMotionState::STABLE);
    EXPECT_FALSE(f.diag().sample.valid);
    EXPECT_FALSE(f.diag().sampleFresh);
}

TEST(PelcoDSessionTest, StaleSampleReportsErrorAndIsNotFresh)
{
    Fixture f(makeConfig(true), true);
    std::uint64_t now = 1000000000ULL;
    f.session->setClock([&now]() { return now; });
    ASSERT_TRUE(f.init());

    now += 600ULL * 1000000ULL;  // sample_stale_ms = 500
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
    EXPECT_FALSE(f.diag().sampleFresh);
}

// ---------------------------------------------------------------------------
//  C4：两轴跨轮 / 轮次与时刻
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, CrossRoundSampleIsNeverPublished)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    const std::uint64_t cycle0 = f.diag().sample.cycleId;
    const double az0 = f.snap().azimuth;

    // 水平轴读到新值、垂直轴缺回复：整轮无效，不得把新 az 配旧 el。
    f.dev.azimuth    = 5.0;
    f.dev.dropVertical = true;
    f.session->pollOnce();

    const auto d = f.diag();
    EXPECT_FALSE(d.sample.valid);
    EXPECT_GT(d.sample.cycleId, cycle0);  // 轮次推进，证明这是新的一轮
    EXPECT_DOUBLE_EQ(f.snap().azimuth, az0);

    // 下一轮两轴齐全：才允许发布。
    f.dev.dropVertical = false;
    f.session->pollOnce();
    EXPECT_TRUE(f.diag().sample.valid);
    EXPECT_DOUBLE_EQ(f.snap().azimuth, 5.0);
    EXPECT_GT(f.diag().sample.cycleId, d.sample.cycleId);
}

// ---------------------------------------------------------------------------
//  停止：2005 / 2006 / 重臂
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, StopNotDeliveredLatches2005AndExhaustsRetries)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    // 先让会话处于“有未完成动作”的状态。
    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    f.fake->failStopSends = 100;
    f.fake->stopSendAttempts = 0;
    f.session->stop();

    // 总尝试 = 1 + stop_retry_count = 3。
    EXPECT_EQ(f.fake->stopSendAttempts.load(), 3);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableStopNotDelivered);
    const auto d = f.diag();
    EXPECT_TRUE(d.motionUncertain);
    EXPECT_FALSE(d.stopFrameDelivered);
    EXPECT_FALSE(d.stopObservedStationary);
    EXPECT_EQ(d.firstFault.code, data::kErrTurntableStopNotDelivered);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);

    // 显式 stop() 是唯一重新武装点：恢复发送后正好再尝试一次。
    f.fake->failStopSends = 0;
    f.fake->stopSendAttempts = 0;
    f.session->stop();
    EXPECT_EQ(f.fake->stopSendAttempts.load(), 1);
    EXPECT_TRUE(f.diag().stopFrameDelivered);
}

TEST(PelcoDSessionTest, StopDeliveredButNotStationaryLatches2006)
{
    data::TurntableConfig cfg = makeConfig(true);
    Fixture f(cfg, true);
    std::uint64_t now = 1000000000ULL;
    f.session->setClock([&now]() { return now; });
    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    f.session->stop();
    ASSERT_TRUE(f.diag().stopFrameDelivered);
    ASSERT_FALSE(f.diag().stopObservedStationary);

    now += 300ULL * 1000000ULL;  // stop_timeout_ms = 200
    f.session->pollOnce();

    const auto d = f.diag();
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableStopNotObserved);
    EXPECT_TRUE(d.motionUncertain);
    EXPECT_TRUE(d.stopFrameDelivered);
    EXPECT_FALSE(d.stopObservedStationary);
    EXPECT_EQ(d.firstFault.code, data::kErrTurntableStopNotObserved);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

TEST(PelcoDSessionTest, StopObservedStationaryRequiresFreshStableSamples)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    f.session->stop();
    ASSERT_TRUE(f.diag().stopFrameDelivered);

    for (int i = 0; i < 4; ++i)
    {
        f.session->pollOnce();
    }

    const auto d = f.diag();
    EXPECT_TRUE(d.stopObservedStationary);
    EXPECT_FALSE(d.motionUncertain);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::IDLE);
}

TEST(PelcoDSessionTest, StopIsNoOpWhenIdle)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.session->stop();
    EXPECT_TRUE(f.fake->sent.empty());
    EXPECT_EQ(f.dev.stopFrames, 0);
}

TEST(PelcoDSessionTest, StopIsIdempotentAndDoesNotClearLatchedError)
{
    data::TurntableConfig cfg = makeConfig(true);
    cfg.azimuthMax = 10.0;

    Fixture f(cfg, true);
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
    EXPECT_EQ(f.dev.stopFrames, 1);  // 已交付的停止不重复发送
    EXPECT_TRUE(f.session->errorLatched());
}

// ---------------------------------------------------------------------------
//  看门狗 / 首因
// ---------------------------------------------------------------------------

TEST(PelcoDSessionTest, WatchdogLatches2008AndRequestsStop)
{
    Fixture f(makeConfig(true), true);
    std::uint64_t now = 1000000000ULL;
    f.session->setClock([&now]() { return now; });
    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // 10° / 12 °/s × 1.5 + 5 s = 6.25 s
    now += 7000000000ULL;
    f.session->pollOnce();

    EXPECT_TRUE(f.session->errorLatched());
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionWatchdog);
    EXPECT_EQ(f.diag().firstFault.code,
              data::kErrTurntableMotionWatchdog);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(f.dev.stopFrames, 1);
}

TEST(PelcoDSessionTest, FirstFaultIsPreservedAcrossStopEscalation)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    f.fake->failStopSends = 100;
    f.session->stop();

    // 首因是显式 stop() 引发的 2005（没有更早的故障）；
    // 这里验证的是“先写者不被后续升级覆盖”的机制。
    ASSERT_EQ(f.diag().firstFault.code,
              data::kErrTurntableStopNotDelivered);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableStopNotDelivered);

    // 再次触发不会改写首因。
    f.session->stop();
    EXPECT_EQ(f.diag().firstFault.code,
              data::kErrTurntableStopNotDelivered);
}

// ---------------------------------------------------------------------------
//  评审 2026-10-20：跨轮拼接 / 部分发送取消 / 停止故障码被覆盖
// ---------------------------------------------------------------------------

/// 两轴快照不得跨两次发布拼接。
///
/// 用 `setSamplePublishHookForTest` 把读端**确定性地**放进
/// “方位角已写、俯仰角未写”的窗口：旧实现会读出 (30,20) 且 valid=true；
/// seqlock 下读端必须等到整轮发布完成，只能读到 (10,20) 或 (30,40)。
TEST(PelcoDSessionTest, TwoAxisSnapshotNeverMixesRounds)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());

    f.dev.azimuth   = 10.0;
    f.dev.elevation = 20.0;
    f.session->pollOnce();
    ASSERT_TRUE(f.diag().sample.valid);
    EXPECT_DOUBLE_EQ(f.diag().sample.azimuth, 10.0);
    EXPECT_DOUBLE_EQ(f.diag().sample.elevation, 20.0);

    f.dev.azimuth   = 30.0;
    f.dev.elevation = 40.0;

    std::atomic<bool> inPublish{false};
    f.session->setSamplePublishHookForTest([&inPublish] {
        inPublish.store(true);
        // 给读端足够时间进入读取路径；seqlock 让它等待，旧实现让它读撕裂值。
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    std::atomic<int>  mixed{0};
    std::atomic<bool> readerDone{false};
    double seenAz = 0.0;
    double seenEl = 0.0;

    std::thread reader([&] {
        while (!inPublish.load())
        {
            std::this_thread::yield();
        }
        const data::TurntableState s = f.session->snapshot();
        const data::TurntableDiagnostics d = f.session->diagnostics();
        // 任何一侧出现 “新方位 + 旧俯仰” 都是跨轮拼接。
        if ((s.azimuth == 30.0 && s.elevation == 20.0) ||
            (d.sample.azimuth == 30.0 && d.sample.elevation == 20.0))
        {
            mixed.fetch_add(1);
        }
        seenAz = s.azimuth;
        seenEl = s.elevation;
        readerDone.store(true);
    });

    f.session->pollOnce();
    reader.join();
    f.session->setSamplePublishHookForTest({});

    ASSERT_TRUE(readerDone.load());
    EXPECT_EQ(mixed.load(), 0)
        << "读端得到跨轮拼接的样本：(" << seenAz << ", " << seenEl << ")";
    EXPECT_TRUE((seenAz == 10.0 && seenEl == 20.0) ||
                (seenAz == 30.0 && seenEl == 40.0))
        << "快照必须完整属于某一轮，实际 (" << seenAz << ", " << seenEl
        << ")";
}

/// 首轴已发送后取消：停止帧必须真的发出去。
TEST(PelcoDSessionTest, CancelledPartialMoveStillSendsStopFrame)
{
    Fixture f(makeConfig(true), true);
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    // ⚠ 必须制造“第一轴已发出、仍在等回显”的窗口（评审反例 1）。
    // 若改成第二轴，走的是另一条取消分支，测不到水平轴的漏停止路径。
    f.fake->dropMoveEchoAxis.store(1);

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;

    const std::function<bool()> abort = [&f] {
        // 第一帧定位已发出 ⇒ 设备可能已经开始运动。
        return f.fake->countSent(isPositioning) >= 1;
    };

    std::string error;
    EXPECT_FALSE(f.session->move(command, error, abort));

    EXPECT_EQ(f.dev.stopFrames, 1)
        << "部分发送后取消必须补发停止帧，不能留下单轴运动";
    const auto d = f.diag();
    EXPECT_TRUE(d.stopFrameDelivered);
    EXPECT_FALSE(d.motionUncertain);
    EXPECT_NE(f.snap().motion, data::TurntableMotionState::IDLE)
        << "停止帧刚交付、尚未观测到静止，不得谎报 IDLE";
}

/// 看门狗触发停止、但停止帧未交付：当前错误必须是 2005，
/// 首因仍是 2008。旧实现随后用 2008 覆盖了 2005。
TEST(PelcoDSessionTest, WatchdogStopNotDeliveredKeeps2005AsCurrentError)
{
    Fixture f(makeConfig(true), true);
    std::uint64_t now = 1000000000ULL;
    f.session->setClock([&now] { return now; });
    ASSERT_TRUE(f.init());

    f.dev.snapToTarget = false;
    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // 三次停止发送全部失败：停止未交付。
    f.fake->failStopSends.store(100);

    now += 7000000000ULL;   // 越过看门狗期限
    f.session->pollOnce();

    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableStopNotDelivered)
        << "停止未交付的 2005 被原始故障码覆盖了";
    EXPECT_EQ(f.diag().firstFault.code,
              data::kErrTurntableMotionWatchdog)
        << "首因必须仍是看门狗 2008";
    const auto d = f.diag();
    EXPECT_TRUE(d.motionUncertain);
    EXPECT_FALSE(d.stopFrameDelivered);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);
}

/// 失联锁存回归：停止帧已交付时，失联故障本身必须锁存。旧实现只在
/// stopForFault 里写“当前错误”（不锁存），于是恢复读后 motion 回到
/// IDLE、move() 又被接受 —— 失联没有留下任何拒绝依据。
TEST(PelcoDSessionTest, LostLinkFaultStaysLatchedAfterRecovery)
{
    data::TurntableConfig cfg = makeConfig(true);
    cfg.commFailureThreshold = 2;

    Fixture f(cfg, true);
    f.dev.azimuth   = 3.0;
    f.dev.elevation = -2.0;
    ASSERT_TRUE(f.init());

    f.dev.dropHorizontal = true;
    f.dev.dropVertical   = true;
    f.session->pollOnce();
    f.session->pollOnce();

    EXPECT_TRUE(f.session->errorLatched())
        << "连续失联达到阈值必须锁存";
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);
    EXPECT_TRUE(f.diag().stopFrameDelivered)
        << "本用例的前提是停止帧已交付（未交付时应保留 2005）";
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR);

    // 链路恢复：锁存不得自动清除，motion 不得回到 IDLE。
    f.dev.dropHorizontal = false;
    f.dev.dropVertical   = false;
    for (int i = 0; i < 5; ++i)
    {
        f.session->pollOnce();
    }

    EXPECT_TRUE(f.session->errorLatched())
        << "恢复通信不得自动清除失联锁存";
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::ERROR)
        << "恢复读后 motion 回到了 IDLE";

    data::TurntableCommand command;
    command.azimuthCommand   = 1.0;
    command.elevationCommand = 0.0;
    std::string error;
    EXPECT_FALSE(f.session->move(command, error))
        << "失联锁存后新运动命令必须被拒绝";
    EXPECT_EQ(f.session->lastError().code, data::kErrTurntableComm);
}

/// 样本与 motion 必须同轮发布：读端不能拿到旧一轮的角度却配上新一轮
/// 得出的 STABLE。
TEST(PelcoDSessionTest, SnapshotNeverPairsOldSampleWithNewMotion)
{
    Fixture f(makeConfig(true), true);
    f.dev.azimuth   = 25.0;
    f.dev.elevation = 0.0;
    ASSERT_TRUE(f.init());
    EXPECT_DOUBLE_EQ(f.snap().azimuth, 25.0);
    ASSERT_EQ(f.snap().motion, data::TurntableMotionState::IDLE);

    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));

    // stableReadCount = 3：三轮 MOVING 之后，下一轮才会得出 STABLE。
    for (int i = 0; i < 3; ++i)
    {
        f.session->pollOnce();
    }
    EXPECT_DOUBLE_EQ(f.snap().azimuth, 10.0);
    ASSERT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);

    std::atomic<bool> entered{false};
    f.session->setSnapshotReadHookForTest([&entered] {
        entered.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    data::TurntableState observed;
    std::thread reader([&f, &observed] { observed = f.session->snapshot(); });

    while (!entered.load())
    {
        std::this_thread::yield();
    }

    // 读端已取走上一轮 (10, MOVING) 并停在钩子里；此时完整发布
    // 新一轮 (10, STABLE)。固定实现返回旧一轮的**一对**值。
    f.session->pollOnce();

    reader.join();
    f.session->setSnapshotReadHookForTest(nullptr);

    EXPECT_DOUBLE_EQ(observed.azimuth, 10.0);
    EXPECT_EQ(observed.motion, data::TurntableMotionState::MOVING)
        << "旧样本配上了新发布的 STABLE：motion 未与样本同窗发布";
    EXPECT_DOUBLE_EQ(f.snap().azimuth, 10.0);
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::STABLE);
}

/// 写端：motion 必须在样本 seqlock 窗口**之内**写入。若发布完成后再写
/// motion，读端会在“新样本已可见、motion 还是上一轮”的窗口里拿到
/// 10° + MOVING，而该样本本轮其实已经判为 STABLE。
TEST(PelcoDSessionTest, MotionIsPublishedInsideSampleWindow)
{
    Fixture f(makeConfig(true), true);
    f.dev.azimuth   = 25.0;
    f.dev.elevation = 0.0;
    ASSERT_TRUE(f.init());

    data::TurntableCommand command;
    command.azimuthCommand   = 10.0;
    command.elevationCommand = 0.0;
    std::string error;
    ASSERT_TRUE(f.session->move(command, error));
    for (int i = 0; i < 3; ++i)
    {
        f.session->pollOnce();
    }
    EXPECT_EQ(f.snap().motion, data::TurntableMotionState::MOVING);

    std::atomic<bool> entered{false};
    f.session->setSamplePublishedHookForTest([&entered] {
        entered.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    data::TurntableState observed;
    std::thread reader([&entered, &observed, &f] {
        while (!entered.load())
        {
            std::this_thread::yield();
        }
        observed = f.session->snapshot();
    });

    f.session->pollOnce();  // 完整发布 (10, STABLE) 并进入钩子

    reader.join();
    f.session->setSamplePublishedHookForTest(nullptr);

    EXPECT_DOUBLE_EQ(observed.azimuth, 10.0);
    EXPECT_EQ(observed.motion, data::TurntableMotionState::STABLE)
        << "motion 在样本发布之后才写入：读端拿到新样本配旧 motion";
}

}  // namespace
}  // namespace device
}  // namespace aircraft
