// ============================================================================
//  tests/device/PelcoDAcceptancePermitTest.cpp
//
//  覆盖“仅限实机验收”的运动许可（V1.3）：
//  · 无许可/设备不符/范围越界/位移超限/次数用尽/过期 ⇒ **零运动帧**；
//  · 许可不能覆盖只读策略；
//  · 许可不改变 WorkModeTrust（EchoOnly 仍是 EchoOnly）；
//  · 许可不绕过错误锁存与过期样本；
//  · 许可过期后拒绝新运动，但**停止路径仍然可用**；
//  · 控制器在无许可时不把命令写入命令槽。
//
//  ⚠ 本文件不连接任何真实 UDP 地址；所有“设备”都是 FakePelcoTransport。
// ============================================================================

#include "FakePelcoTransport.h"
#include "device/turntable/PelcoDSession.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "gtest/gtest.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
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

data::TurntableConfig makeConfig(bool motion = true)
{
    data::TurntableConfig cfg;
    cfg.protocol          = "pelco_d_udp";
    cfg.host              = "127.0.0.1";
    cfg.port              = 6666;
    cfg.pelcoAddress      = 1;
    cfg.azimuthMin        = -90.0;
    cfg.azimuthMax        = 90.0;
    cfg.elevationMin      = -60.0;
    cfg.elevationMax      = 60.0;
    cfg.coarseSpeed       = 5.0;
    cfg.fineSpeed         = 1.0;
    cfg.replyTimeoutMs    = 10;
    cfg.pollIntervalMs    = 2;
    cfg.stableReadCount   = 3;
    cfg.stableThresholdDeg    = 0.02;
    cfg.arrivalToleranceDeg   = 0.2;
    cfg.commFailureThreshold  = 5;
    cfg.useSpeedControl       = false;
    cfg.sampleStaleMs         = 500;
    cfg.stopTimeoutMs         = 50;
    cfg.stopRetryCount        = 2;
    cfg.stopRetryWindowMs     = 100;
    cfg.stopBackoffMs         = 0;
    cfg.sendPolicy = motion ? data::TurntableSendPolicy::MotionEnabled
                            : data::TurntableSendPolicy::ReadOnly;
    return cfg;
}

data::TurntableAcceptancePermit makePermit(
    const data::TurntableConfig& cfg)
{
    data::TurntableAcceptancePermit permit;
    permit.scopeId              = "unit-test-scope";
    permit.deviceHost           = cfg.host;
    permit.devicePort           = cfg.port;
    permit.pelcoAddress         = cfg.pelcoAddress;
    permit.allowAbsolutePosition = true;
    permit.azimuthMinDeg        = -5.0;
    permit.azimuthMaxDeg        = 5.0;
    permit.elevationMinDeg      = -5.0;
    permit.elevationMaxDeg      = 5.0;
    permit.maxDisplacementDeg   = 5.0;
    permit.maxExecutions        = 3;
    permit.validFromEpochMs     = 1000000;
    permit.validUntilEpochMs    = 2000000;
    return permit;
}

bool isControl(const PelcoDFrame& frame)
{
    const bool positioning =
        (frame.data1 == 0x00 &&
         (frame.data2 == 0x4B || frame.data2 == 0x4D)) ||
        ((frame.data1 == 0x4B || frame.data1 == 0x4D) &&
         frame.data2 != 0x00);
    const bool stop = frame.data1 == 0x00 && frame.data2 == 0x00 &&
                      frame.data3 == 0x00 && frame.data4 == 0x00;
    return positioning || stop;
}

int controlFrames(const FakePelcoTransport& fake)
{
    return fake.countSent(isControl);
}

int stopFrames(const FakePelcoTransport& fake)
{
    return fake.countSent([](const PelcoDFrame& frame) {
        return frame.data1 == 0x00 && frame.data2 == 0x00 &&
               frame.data3 == 0x00 && frame.data4 == 0x00;
    });
}

int positioningFrames(const FakePelcoTransport& fake)
{
    return fake.countSent([](const PelcoDFrame& frame) {
        return (frame.data1 == 0x00 &&
                (frame.data2 == 0x4B || frame.data2 == 0x4D)) ||
               ((frame.data1 == 0x4B || frame.data1 == 0x4D) &&
                frame.data2 != 0x00);
    });
}

/// 会话直接驱动的夹具：可选注入墙钟与单调时钟，让“过期/样本过期”
/// 成为确定性判据，而不是依赖真实等待。
struct SessionFixture
{
    std::shared_ptr<FakePelcoTransport> fake =
        std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    std::int64_t wallMs = 1500000;            // 位于许可有效期内
    std::uint64_t monoNs = 1000000000ULL;     // 1 s
    std::unique_ptr<PelcoDSession> session;   // 最后声明 → 最先析构

    /// @param injectMono 仅在需要**确定性样本过期**的用例中注入固定
    ///        单调时钟。注入后时钟不会自行前进，因此不能用于会等待
    ///        回显超时的路径（例如设备故意不回包）。
    explicit SessionFixture(const data::TurntableConfig& cfg,
                            bool injectMono = false)
    {
        dev.transport = fake.get();
        dev.install();
        session = std::make_unique<PelcoDSession>(cfg, fake);
        session->setWallClockForTest([this] { return wallMs; });
        if (injectMono)
        {
            session->setClock([this] { return monoNs; });
        }
    }

    bool init()
    {
        std::string err;
        return session->initialize(err);
    }

    bool install(const data::TurntableAcceptancePermit& permit)
    {
        installError.clear();
        return session->setAcceptancePermit(permit, installError);
    }

    bool moveTo(double az, double el, std::string& err)
    {
        data::TurntableCommand command;
        command.azimuthCommand   = az;
        command.elevationCommand = el;
        return session->move(command, err);
    }

    std::string installError;
};

// ---------------------------------------------------------------------------
// 1. 无许可 / 设备不符 / 字段非法
// ---------------------------------------------------------------------------

TEST(PelcoDAcceptancePermitTest, MotionEnabledWithoutPermitSendsNoFrame)
{
    SessionFixture f(makeConfig(/*motion=*/true));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);

    const auto d = f.session->diagnostics();
    EXPECT_FALSE(d.acceptancePermitInstalled);
    EXPECT_TRUE(d.motionInhibited);
}

TEST(PelcoDAcceptancePermitTest, PermitForDifferentDeviceIsRejected)
{
    const data::TurntableConfig cfg = makeConfig();

    auto tryPermit = [&](const data::TurntableAcceptancePermit& permit) {
        SessionFixture f(cfg);
        return f.install(permit);
    };

    auto permit = makePermit(cfg);
    EXPECT_TRUE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.deviceHost = "127.0.0.2";
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.devicePort = 6667;
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.pelcoAddress = 2;
    EXPECT_FALSE(tryPermit(permit));
}

TEST(PelcoDAcceptancePermitTest, StructurallyInvalidPermitIsRejected)
{
    const data::TurntableConfig cfg = makeConfig();

    auto tryPermit = [&](const data::TurntableAcceptancePermit& permit) {
        SessionFixture f(cfg);
        return f.install(permit);
    };

    auto permit = makePermit(cfg);
    permit.scopeId.clear();
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.allowAbsolutePosition = false;
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.maxDisplacementDeg = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.maxExecutions = 0;
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.validUntilEpochMs = permit.validFromEpochMs;
    EXPECT_FALSE(tryPermit(permit));

    // 许可范围必须落在机械行程内。
    permit = makePermit(cfg);
    permit.azimuthMaxDeg = cfg.azimuthMax + 1.0;
    EXPECT_FALSE(tryPermit(permit));

    permit = makePermit(cfg);
    permit.elevationMinDeg = cfg.elevationMin - 1.0;
    EXPECT_FALSE(tryPermit(permit));
}

TEST(PelcoDAcceptancePermitTest, AlreadyExpiredPermitIsRejectedAtInstall)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    f.wallMs = 2500000;  // 晚于 validUntil

    EXPECT_FALSE(f.install(makePermit(cfg)));
    EXPECT_FALSE(f.installError.empty());
}

// ---------------------------------------------------------------------------
// 2. 目标范围 / 位移 / 次数 / 期限
// ---------------------------------------------------------------------------

TEST(PelcoDAcceptancePermitTest, TargetOutsidePermitRangeSendsNoFrame)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.azimuthMinDeg   = -1.0;
    permit.azimuthMaxDeg   = 1.0;
    permit.elevationMinDeg = -1.0;
    permit.elevationMaxDeg = 1.0;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_FALSE(f.moveTo(2.0, 0.0, error));  // 在机械行程内、在许可外
    EXPECT_EQ(controlFrames(*f.fake), 0);

    EXPECT_FALSE(f.moveTo(0.0, 2.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
}

TEST(PelcoDAcceptancePermitTest, DisplacementBeyondPermitSendsNoFrame)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.maxDisplacementDeg = 1.0;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_FALSE(f.moveTo(2.0, 0.0, error));  // 范围允许，位移不允许
    EXPECT_EQ(controlFrames(*f.fake), 0);

    // 边界内仍允许，证明拒绝来自位移而不是“许可一律拒绝”。
    EXPECT_TRUE(f.moveTo(0.5, 0.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 2);
}

TEST(PelcoDAcceptancePermitTest, ExecutionCountIsConsumedAndExhausted)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.maxExecutions = 2;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_TRUE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 2);
    EXPECT_EQ(f.session->diagnostics().acceptancePermitRemaining, 1);

    EXPECT_TRUE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 4);
    EXPECT_EQ(f.session->diagnostics().acceptancePermitRemaining, 0);

    // 第三次必须零帧拒绝，且不能因为“次数为 0”把会话锁死到连诊断都不给。
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 4);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
}

TEST(PelcoDAcceptancePermitTest, ExpiryAfterMotionRejectsNewMotionButStopStillSends)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    ASSERT_TRUE(f.moveTo(1.0, 1.0, error));   // 仍然有效
    ASSERT_EQ(controlFrames(*f.fake), 2);

    f.wallMs = 2000001;                       // 刚过 validUntil（左闭右开）
    EXPECT_FALSE(f.moveTo(1.5, 1.5, error));  // 新运动必须拒绝
    EXPECT_EQ(controlFrames(*f.fake), 2);
    EXPECT_TRUE(f.session->diagnostics().acceptancePermitExpired);

    // 停止不是许可门禁的一部分：已经开始的运动必须仍能停止。
    f.session->stop();
    EXPECT_EQ(stopFrames(*f.fake), 1);
    EXPECT_TRUE(f.session->diagnostics().stopFrameDelivered);
}

TEST(PelcoDAcceptancePermitTest, PermitExpiryBetweenAxesStopsWithoutSecondAxis)
{
    // 反例：许可在第一轴发出后到期。第二轴不得再发，且已发出的
    // 第一轴必须进入停止收口。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    const auto previous = f.fake->onSend;
    f.fake->onSend = [&](const std::vector<std::uint8_t>& bytes) {
        const PelcoDFrame frame = FakePelcoTransport::decode(bytes);
        if (frame.data1 == 0x00 && frame.data2 == 0x4B)
        {
            f.wallMs = 2000001;  // 水平定位帧已发出 ⇒ 许可随即到期
        }
        previous(bytes);
    };

    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_NE(error.find("失效"), std::string::npos);
    EXPECT_EQ(positioningFrames(*f.fake), 1);   // 只有水平帧
    EXPECT_EQ(stopFrames(*f.fake), 1);          // 部分发送 ⇒ 停止收口
    EXPECT_TRUE(f.session->diagnostics().stopFrameDelivered);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
}

TEST(PelcoDAcceptancePermitTest, PermitExpiryDuringEchoWaitDoesNotRetransmit)
{
    // 反例：第一轴等待回显期间到期，旧实现会在随后的重传里继续发帧。
    // 每个定位帧发送前复核后，只允许已发出的那 1 帧，然后停止收口。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.fake->dropMoveEchoAxis = 1;  // 水平轴不回显，迫使进入重传分支

    const auto previous = f.fake->onSend;
    f.fake->onSend = [&](const std::vector<std::uint8_t>& bytes) {
        const PelcoDFrame frame = FakePelcoTransport::decode(bytes);
        if (frame.data1 == 0x00 && frame.data2 == 0x4B)
        {
            f.wallMs = 2000001;
        }
        previous(bytes);
    };

    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(positioningFrames(*f.fake), 1);  // 没有重传帧
    EXPECT_EQ(stopFrames(*f.fake), 1);
}

// ---------------------------------------------------------------------------
// 3. 只读、WorkMode、锁存、样本新鲜度
// ---------------------------------------------------------------------------

TEST(PelcoDAcceptancePermitTest, ReadOnlyPolicyWinsOverPermit)
{
    const data::TurntableConfig cfg = makeConfig(/*motion=*/false);
    SessionFixture f(cfg);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_TRUE(f.session->diagnostics().acceptancePermitInstalled);
    EXPECT_TRUE(f.session->diagnostics().motionInhibited);
}

TEST(PelcoDAcceptancePermitTest, PermitDoesNotChangeWorkModeTrust)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);  // 默认 fake：0xE0 回复与查询逐字节相同
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());

    ASSERT_EQ(f.session->diagnostics().workModeTrust,
              data::WorkModeTrust::EchoOnly);

    std::string error;
    ASSERT_TRUE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(f.session->diagnostics().workModeTrust,
              data::WorkModeTrust::EchoOnly);
}

TEST(PelcoDAcceptancePermitTest, PermitDoesNotBypassLatchedFault)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());

    // 水平轴从此刻起不回包；连续阈值周期后锁存 2001。
    f.dev.dropHorizontal = true;
    for (int i = 0; i < cfg.commFailureThreshold; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_TRUE(f.session->errorLatched());
    // ⚠ 失联锁存会同时把样本置为无效，因此本用例**不能**区分
    // “锁存门禁拒绝”与“无新鲜样本拒绝”。可鉴别路径见下一个用例。
    EXPECT_FALSE(f.session->diagnostics().sampleFresh);

    f.fake->sent.clear();
    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
}

TEST(PelcoDAcceptancePermitTest,
     PermitDoesNotBypassLatchedFaultWithFreshSample)
{
    // T5b 的离线**可鉴别**路径：看门狗锁存（2008）发生在样本仍 fresh
    // 时，拒绝只能来自锁存门禁，而不是样本过期。实机 T5b（拔线）无法
    // 提供这一区分：拔线后工具的新鲜度预检会先拒绝。
    data::TurntableConfig cfg = makeConfig();
    cfg.sampleStaleMs = 10000;  // 让 6 s 后的旧样本仍算 fresh
    SessionFixture f(cfg, /*injectMono=*/true);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.dev.snapToTarget = false;  // 设备不动 ⇒ 看门狗到期
    f.fake->sent.clear();

    std::string error;
    ASSERT_TRUE(f.moveTo(1.0, 1.0, error));  // travel=1° ⇒ 截止约 5.1 s
    f.monoNs += 6ULL * 1000ULL * 1000ULL * 1000ULL;
    f.session->pollOnce();

    ASSERT_TRUE(f.session->errorLatched());
    ASSERT_TRUE(f.session->diagnostics().sampleFresh);  // 关键鉴别条件
    ASSERT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionWatchdog);
    EXPECT_EQ(stopFrames(*f.fake), 1);  // 故障收口的停止帧已交付

    f.fake->sent.clear();
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionWatchdog);
}

TEST(PelcoDAcceptancePermitTest, StaleSampleBlocksPermitMotion)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg, /*injectMono=*/true);
    ASSERT_TRUE(f.install(makePermit(cfg)));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    // 不轮询，只把单调时钟推过 sample_stale_ms。
    f.monoNs += 1000ULL * 1000ULL * 1000ULL;  // +1 s > 500 ms

    std::string error;
    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_EQ(f.session->lastError().code,
              data::kErrTurntableMotionInhibited);
}

// ---------------------------------------------------------------------------
// 4. 验收静止停止帧（T1）：获准且静止时**实际发一帧**，其余路径零帧
// ---------------------------------------------------------------------------

TEST(PelcoDAcceptancePermitTest, AtRestStopWithoutPermitSendsNoFrame)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    ASSERT_TRUE(f.init());  // 未安装许可
    f.fake->sent.clear();

    const int before = stopFrames(*f.fake);
    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), before);
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(f.session->diagnostics().stopFrameDelivered);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopWithoutFlagSendsNoFrame)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = false;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    const int before = stopFrames(*f.fake);
    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), before);
    EXPECT_NE(error.find("allow_stop_at_rest"), std::string::npos);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopReadOnlyWinsOverPermit)
{
    const data::TurntableConfig cfg = makeConfig(/*motion=*/false);
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(controlFrames(*f.fake), 0);
    EXPECT_FALSE(f.session->diagnostics().stopFrameDelivered);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopAllowedSendsExactlyOneStopFrame)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    ASSERT_TRUE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(stopFrames(*f.fake), 1);
    EXPECT_TRUE(f.session->diagnostics().stopFrameDelivered);
    EXPECT_FALSE(f.session->diagnostics().motionUncertain);

    // 轮询到观测静止后再次请求：不得重复补帧（stopPending_ 已清除，
    // 拒绝只能来自 stopSent_）。
    for (int i = 0; i < cfg.stableReadCount + 2; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_TRUE(f.session->diagnostics().stopObservedStationary);

    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), 1);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopRejectedWhileMoving)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());

    std::string error;
    ASSERT_TRUE(f.moveTo(1.0, 1.0, error));  // 刚交付，尚未观测到位
    f.fake->sent.clear();

    const int before = stopFrames(*f.fake);
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), before);
    EXPECT_NE(error.find("静止"), std::string::npos);

    f.session->stop();
}

TEST(PelcoDAcceptancePermitTest, AtRestStopRejectedWhenSampleStale)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg, /*injectMono=*/true);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.monoNs += 1000ULL * 1000ULL * 1000ULL;  // +1 s > sample_stale_ms

    const int before = stopFrames(*f.fake);
    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), before);
    EXPECT_NE(error.find("新鲜"), std::string::npos);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopRejectedAfterFaultLatch)
{
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());

    f.dev.dropHorizontal = true;
    for (int i = 0; i < cfg.commFailureThreshold; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_TRUE(f.session->errorLatched());

    const int before = stopFrames(*f.fake);
    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), before);
    EXPECT_FALSE(error.empty());
}

TEST(PelcoDAcceptancePermitTest, AtRestStopAllowedAfterPermitExpiry)
{
    // 静止停止属于停止/安全收口，不是新运动：许可过期只拒绝新运动，
    // 不阻止本次会话补发停止帧。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    f.wallMs = 2000001;  // 已过期
    std::string error;
    EXPECT_TRUE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), 1);
    EXPECT_TRUE(f.session->diagnostics().acceptancePermitExpired);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopSingleAttemptNoAutomaticRetry)
{
    // 反例：首次发送失败时旧实现沿用 stop_retry_count 自动重发。
    // T1 必须只有一次发送；失败返回 false，不置 motionUncertain。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();
    f.fake->failStopSends = 1;

    std::string error;
    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(f.fake->stopSendAttempts.load(), 1);  // 不是 3
    EXPECT_EQ(stopFrames(*f.fake), 0);
    EXPECT_FALSE(f.session->diagnostics().stopFrameDelivered);
    EXPECT_FALSE(f.session->diagnostics().motionUncertain);
    EXPECT_FALSE(error.empty());

    // 显式再调用一次（不是自动重试）：本次发送成功，恰好 1 帧。
    EXPECT_TRUE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(f.fake->stopSendAttempts.load(), 2);
    EXPECT_EQ(stopFrames(*f.fake), 1);
    EXPECT_TRUE(f.session->diagnostics().stopFrameDelivered);
}

TEST(PelcoDAcceptancePermitTest, AtRestStopOnlyOncePerSessionAcrossMotion)
{
    // 反例：T1 → 观测静止 → 正常运动 → 再次 T1。旧实现复用会被新运动
    // 清零的 stopSent_，第二次仍发帧。会话级记录必须独立保留。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowStopFrameAtRest = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    ASSERT_TRUE(f.session->sendAcceptanceStopFrameAtRest(error));
    ASSERT_EQ(stopFrames(*f.fake), 1);
    for (int i = 0; i < cfg.stableReadCount + 2; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_TRUE(f.session->diagnostics().stopObservedStationary);

    ASSERT_TRUE(f.moveTo(1.0, 1.0, error));  // 新运动清零 stopSent_
    for (int i = 0; i < cfg.stableReadCount + 2; ++i)
    {
        f.session->pollOnce();
    }
    ASSERT_EQ(f.session->snapshot().motion,
              data::TurntableMotionState::STABLE);

    EXPECT_FALSE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), 1);  // 仍只有第一次的 1 帧
    EXPECT_FALSE(f.session->diagnostics().motionUncertain);
}

TEST(PelcoDAcceptancePermitTest, StopOnlyPermitInstallsButNeverAuthorizesMotion)
{
    // 两个授权必须真正独立：只授权静止停止帧的许可可以安装并执行 T1，
    // 但绝对定位始终被拒且零定位帧。
    const data::TurntableConfig cfg = makeConfig();
    SessionFixture f(cfg);
    auto permit = makePermit(cfg);
    permit.allowAbsolutePosition = false;
    permit.allowStopFrameAtRest  = true;
    ASSERT_TRUE(f.install(permit));
    ASSERT_TRUE(f.init());
    f.fake->sent.clear();

    std::string error;
    EXPECT_TRUE(f.session->sendAcceptanceStopFrameAtRest(error));
    EXPECT_EQ(stopFrames(*f.fake), 1);

    EXPECT_FALSE(f.moveTo(1.0, 1.0, error));
    EXPECT_EQ(positioningFrames(*f.fake), 0);
    EXPECT_NE(error.find("绝对定位"), std::string::npos);
    EXPECT_TRUE(f.session->diagnostics().motionInhibited);
}

// ---------------------------------------------------------------------------
// 5. 控制器层：无许可时连命令槽都不写；静止停止帧经过 worker 只发一帧
// ---------------------------------------------------------------------------

TEST(PelcoDAcceptancePermitTest, ControllerRefusesQueueWithoutPermit)
{
    const data::TurntableConfig cfg = makeConfig(/*motion=*/true);
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    {
        PelcoDTurntableController controller(cfg, fake);
        ASSERT_TRUE(controller.initialize());
        fake->sent.clear();

        data::TurntableCommand command;
        command.azimuthCommand   = 1.0;
        command.elevationCommand = 1.0;
        EXPECT_FALSE(controller.move(command));
        EXPECT_EQ(controlFrames(*fake), 0);

        const auto d = controller.diagnostics();
        EXPECT_EQ(d.lastCommandOutcome,
                  data::TurntableCommandOutcome::Failed);
    }
}

TEST(PelcoDAcceptancePermitTest, ControllerAtRestStopSendsOneFrameAndRejectsSecond)
{
    const data::TurntableConfig cfg = makeConfig();
    auto fake = std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    dev.transport = fake.get();
    dev.install();

    {
        PelcoDTurntableController controller(cfg, fake);
        auto permit = makePermit(cfg);
        permit.allowStopFrameAtRest = true;
        // 控制器没有注入墙钟的测试入口；用宽期限覆盖真实当前时间。
        permit.validFromEpochMs  = 1;
        permit.validUntilEpochMs = 4102444800000LL;  // 2100-01-01
        std::string error;
        ASSERT_TRUE(controller.enableAcceptancePermit(permit, error));
        ASSERT_TRUE(controller.initialize());
        fake->sent.clear();

        ASSERT_TRUE(controller.sendAcceptanceStopFrameAtRest(error));
        EXPECT_TRUE(error.empty());
        EXPECT_EQ(stopFrames(*fake), 1);
        EXPECT_TRUE(controller.diagnostics().stopFrameDelivered);

        EXPECT_FALSE(controller.sendAcceptanceStopFrameAtRest(error));
        EXPECT_FALSE(error.empty());
        EXPECT_EQ(stopFrames(*fake), 1);
    }

    // 析构/退出路径不得再补发停止帧。
    EXPECT_EQ(stopFrames(*fake), 1);
}

}  // namespace
}  // namespace device
}  // namespace aircraft
