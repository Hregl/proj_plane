// ============================================================================
//  tests/integration/TurntableControlServiceTest.cpp
//
//  覆盖主程序“受控转台操作入口”（app::TurntableControlService）在本批
//  实际改动上的行为：
//    · 受理 / 交付 / 反馈到位三个阶段分开；
//    · 前一条命令未结束时拒绝第二条；
//    · 越出验收许可/机械行程的目标在发送前被拒，且不消耗许可次数；
//    · 运动中停止走既有 stop()，结果区分“帧已交付/观测到静止”；
//    · 失联/样本过期拒绝定位，不补发定位帧；
//    · 回显超时、错误回包 → 失败，不自动重试；
//    · 到位后可继续下一条命令（连续操作）；
//    · shutdown() join 停止线程，退出后不在 close 后补发停止帧。
//
//  ⚠ 本套件**不发任何真实网络帧**：控制器使用 tests/device 的
//    FakePelcoTransport / FakePelcoDevice；许可期限用宽区间覆盖真实钟。
//  ⚠ 只回归本批改动（控制服务与其入口），不重新铺开已验证关闭的
//    T3/T4 驱动用例。
// ============================================================================

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "../device/FakePelcoTransport.h"
#include "app/TurntableControlService.h"
#include "data/TurntableAcceptancePermit.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableDiagnostics.h"
#include "data/TurntableMotionState.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "ui/TurntableControlView.h"

namespace aircraft
{
namespace app
{
namespace
{

using device::PelcoDTurntableController;
using device::test::FakePelcoDevice;
using device::test::FakePelcoTransport;
using S = data::TurntableMotionState;
using Stage = ui::TurntableCommandStage;
using StopStage = ui::TurntableStopStage;

constexpr double kAngleEpsilon = 1e-6;

void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

data::TurntableConfig makeConfig(bool snapToTarget,
                                 int commFailureThreshold = 2)
{
    data::TurntableConfig cfg;
    cfg.protocol         = "pelco_d_udp";
    cfg.host             = "127.0.0.1";
    cfg.port             = 6666;
    cfg.pelcoAddress     = 1;
    cfg.azimuthMin       = -180.0;
    cfg.azimuthMax       = 180.0;
    cfg.elevationMin     = -60.0;
    cfg.elevationMax     = 60.0;
    cfg.coarseSpeed      = 5.0;
    cfg.fineSpeed        = 1.2;
    cfg.replyTimeoutMs   = 20;
    cfg.pollIntervalMs   = 5;
    cfg.stableReadCount  = 2;
    cfg.stableThresholdDeg   = 0.02;
    cfg.arrivalToleranceDeg  = 0.2;
    cfg.commFailureThreshold = commFailureThreshold;
    cfg.useSpeedControl      = false;
    cfg.sampleStaleMs        = 500;
    cfg.stopTimeoutMs        = 300;
    cfg.stopRetryCount       = 2;
    cfg.stopRetryWindowMs    = 300;
    cfg.stopBackoffMs        = 0;
    cfg.sendPolicy = data::TurntableSendPolicy::MotionEnabled;
    (void)snapToTarget;   // 只影响 fake 设备，保留参数方便阅读
    return cfg;
}

data::TurntableAcceptancePermit makePermit(const data::TurntableConfig& cfg,
                                           int maxExecutions = 5)
{
    data::TurntableAcceptancePermit permit;
    permit.scopeId               = "control-service-test";
    permit.deviceHost            = cfg.host;
    permit.devicePort            = cfg.port;
    permit.pelcoAddress          = cfg.pelcoAddress;
    permit.allowAbsolutePosition = true;
    permit.azimuthMinDeg         = -5.0;
    permit.azimuthMaxDeg         = 5.0;
    permit.elevationMinDeg       = -5.0;
    permit.elevationMaxDeg       = 5.0;
    // 位移上限刻意设得比目标范围宽：让“目标角越界”这一条成为
    // 唯一可能拒绝 6.0° 的判据，而不是被位移判据掩住。
    permit.maxDisplacementDeg    = 10.0;
    permit.maxExecutions         = maxExecutions;
    // 控制器没有注入墙钟的入口；用宽期限覆盖真实当前时间。
    permit.validFromEpochMs  = 1;
    permit.validUntilEpochMs = 4102444800000LL;   // 2100-01-01
    return permit;
}

bool isStopFrame(const device::PelcoDFrame& f)
{
    return f.data1 == 0x00 && f.data2 == 0x00 && f.data3 == 0x00 &&
           f.data4 == 0x00;
}

bool isPositioningFrame(const device::PelcoDFrame& f)
{
    return (f.data1 == 0x00 && (f.data2 == 0x4B || f.data2 == 0x4D)) ||
           ((f.data1 == 0x4B || f.data1 == 0x4D) && f.data2 != 0x00);
}

int stopFrames(const FakePelcoTransport& fake)
{
    return fake.countSent(isStopFrame);
}

int positioningFrames(const FakePelcoTransport& fake)
{
    return fake.countSent(isPositioningFrame);
}

/// 一套“真实控制器 + fake 设备 + 控制服务”的夹具。
struct Fixture
{
    std::shared_ptr<FakePelcoTransport> fake =
        std::make_shared<FakePelcoTransport>();
    FakePelcoDevice dev;
    data::TurntableConfig cfg;
    std::unique_ptr<PelcoDTurntableController> controller;
    std::unique_ptr<TurntableControlService> service;

    explicit Fixture(bool snapToTarget, int commFailureThreshold = 2)
        : cfg(makeConfig(snapToTarget, commFailureThreshold))
    {
        dev.transport = fake.get();
        dev.snapToTarget = snapToTarget;
        dev.install();

        controller = std::make_unique<PelcoDTurntableController>(cfg, fake);
        std::string error;
        EXPECT_TRUE(controller->enableAcceptancePermit(makePermit(cfg), error))
            << error;
        EXPECT_TRUE(controller->initialize());
        service = std::make_unique<TurntableControlService>(
            *controller, cfg, controller.get());
    }

    ~Fixture()
    {
        if (service)
        {
            service->shutdown();
        }
    }

    /// 等到首轮新鲜双轴样本可用（服务预检要求它不是过期值）。
    bool waitConnected(int timeoutMs = 2000)
    {
        return pump(
            [](const ui::TurntableControlView& v) {
                return v.connection == ui::TurntableConnectionState::Connected &&
                       v.sampleValid && v.sampleFresh;
            },
            timeoutMs);
    }

    bool pump(const std::function<bool(const ui::TurntableControlView&)>& pred,
              int timeoutMs)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeoutMs);
        while (std::chrono::steady_clock::now() < deadline)
        {
            service->poll();
            if (pred(service->view()))
            {
                return true;
            }
            sleepMs(2);
        }
        service->poll();
        return pred(service->view());
    }

    /// 发一条命令并等它进入指定阶段。
    bool moveAndWait(double az, double el, Stage stage,
                     std::string& error, int timeoutMs = 3000)
    {
        if (!service->requestMove(az, el, error))
        {
            return false;
        }
        return pump([stage](const ui::TurntableControlView& v) {
                        return v.commandStage == stage;
                    },
                    timeoutMs);
    }
};

// ---------------------------------------------------------------------------
// 1. 受理 → 交付 → 反馈到位；到位后可继续下一条命令
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 定位按受理交付到位推进并可连续操作)
{
    Fixture f(/*snapToTarget=*/true);
    ASSERT_TRUE(f.waitConnected());

    std::string error;
    ASSERT_TRUE(f.service->requestMove(1.0, 0.5, error)) << error;
    EXPECT_TRUE(f.service->view().busy);
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Arrived;
        },
        3000)) << f.service->view().commandDetail;

    const ui::TurntableControlView first = f.service->view();
    EXPECT_EQ(first.commandStage, Stage::Arrived);
    EXPECT_NEAR(first.azimuth, 1.0, kAngleEpsilon);
    EXPECT_NEAR(first.elevation, 0.5, kAngleEpsilon);
    EXPECT_FALSE(first.busy);
    EXPECT_TRUE(first.positioningEnabled);
    EXPECT_EQ(f.controller->state().motion, S::STABLE);
    EXPECT_EQ(positioningFrames(*f.fake), 2)
        << "一条双轴定位命令应恰好发送两帧定位指令";

    // 到位后第二条命令必须被受理（连续操作，不因上一条结束而被卡住）。
    ASSERT_TRUE(f.moveAndWait(-1.0, -0.5, Stage::Arrived, error, 3000))
        << error;
    const ui::TurntableControlView second = f.service->view();
    EXPECT_NEAR(second.azimuth, -1.0, kAngleEpsilon);
    EXPECT_NEAR(second.elevation, -0.5, kAngleEpsilon);
}

// ---------------------------------------------------------------------------
// 2. 前一条未结束时拒绝第二条；不得产生新的定位帧
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 上一条未结束时拒绝第二条命令)
{
    // 设备不跟随目标：命令交付后仍停留在原角，服务保持“未结束”。
    Fixture f(/*snapToTarget=*/false);
    ASSERT_TRUE(f.waitConnected());

    std::string error;
    ASSERT_TRUE(f.service->requestMove(1.0, 0.0, error)) << error;
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Delivered;
        },
        2000)) << f.service->view().commandDetail;

    const int framesBefore = positioningFrames(*f.fake);
    EXPECT_FALSE(f.service->requestMove(2.0, 0.0, error));
    EXPECT_NE(error.find("上一条定位尚未结束"), std::string::npos)
        << "拒绝原因应明确指出上一条未结束：" << error;
    EXPECT_EQ(positioningFrames(*f.fake), framesBefore)
        << "被拒的第二条命令不得产生任何定位帧";
    EXPECT_EQ(f.service->view().commandStage, Stage::Rejected);
}

// ---------------------------------------------------------------------------
// 3. 目标越出许可范围：发送前拒绝，不消耗许可次数，不发帧
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 超出许可范围在发送前被拒且不消耗次数)
{
    Fixture f(/*snapToTarget=*/true);
    ASSERT_TRUE(f.waitConnected());

    const int remainingBefore =
        f.controller->diagnostics().acceptancePermitRemaining;
    ASSERT_EQ(remainingBefore, 5);

    std::string error;
    EXPECT_FALSE(f.service->requestMove(6.0, 0.0, error));
    EXPECT_NE(error.find("超出验收许可范围"), std::string::npos) << error;
    EXPECT_EQ(positioningFrames(*f.fake), 0);
    EXPECT_EQ(f.controller->diagnostics().acceptancePermitRemaining,
              remainingBefore)
        << "预检拒绝不得消耗许可次数";
    EXPECT_FALSE(f.service->view().busy);
}

// ---------------------------------------------------------------------------
// 4. 运动中停止：走既有 stop()，区分帧交付与观测静止
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 运动中停止显示帧交付与观测静止)
{
    Fixture f(/*snapToTarget=*/false);
    ASSERT_TRUE(f.waitConnected());

    std::string error;
    ASSERT_TRUE(f.service->requestMove(2.0, 0.0, error)) << error;
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Delivered;
        },
        2000)) << f.service->view().commandDetail;
    ASSERT_EQ(f.controller->state().motion, S::MOVING);

    f.service->requestStop();
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.stopStage == StopStage::ObservedStationary;
        },
        3000)) << f.service->view().stopDetail;

    const ui::TurntableControlView v = f.service->view();
    EXPECT_EQ(v.commandStage, Stage::Cancelled)
        << "停止屏障应让在途命令有明确终结结果";
    EXPECT_EQ(v.stopStage, StopStage::ObservedStationary);
    EXPECT_NE(v.stopDetail.find("不是设备 ACK"), std::string::npos);
    EXPECT_EQ(stopFrames(*f.fake), 1);
    const data::TurntableDiagnostics d = f.controller->diagnostics();
    EXPECT_TRUE(d.stopFrameDelivered);
    EXPECT_TRUE(d.stopObservedStationary);
    EXPECT_FALSE(d.motionUncertain);
    EXPECT_FALSE(f.service->busy());

    // 停止完成后再定位：上一次停止结果不得继续冒充本条运动的停止
    // 状态。运动事务号推进后，旧 stopFacts 不再适用于当前命令。
    ASSERT_TRUE(f.service->requestMove(1.0, 0.0, error)) << error;
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Delivered;
        },
        3000)) << f.service->view().commandDetail;
    EXPECT_EQ(f.service->view().stopStage, StopStage::None)
        << "新定位开始后，停止栏应回到“未请求停止”";
    EXPECT_TRUE(f.service->view().stopDetail.empty());
}

// ---------------------------------------------------------------------------
// 5a. 回显超时：失败，不自动重试
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 回显超时失败且不自动重试)
{
    Fixture f(/*snapToTarget=*/true);
    ASSERT_TRUE(f.waitConnected());
    f.fake->dropMoveEchoAxis.store(1);   // 水平轴永远不回显

    std::string error;
    ASSERT_TRUE(f.service->requestMove(1.0, 0.0, error)) << error;
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Failed;
        },
        3000)) << f.service->view().commandDetail;

    // 停止帧已交付时，原始通信故障保留为首因但不锁存（由观测窗口
    // 决定是否升级为 2006）；命令本身的终结结果必须是 Failed。
    EXPECT_EQ(f.controller->diagnostics().lastCommandOutcome,
              data::TurntableCommandOutcome::Failed);
    EXPECT_NE(f.service->view().faultText.find("2001"), std::string::npos)
        << "回显超时应记录通信故障首因：" << f.service->view().faultText;

    // 失败后只允许既有故障收口的那一帧停止；不得补发定位帧。
    const int positioning = positioningFrames(*f.fake);
    const int stops = stopFrames(*f.fake);
    sleepMs(300);
    EXPECT_EQ(positioningFrames(*f.fake), positioning)
        << "失败后不得自动重试定位";
    EXPECT_EQ(stopFrames(*f.fake), stops)
        << "失败后不得自动重发停止";
    EXPECT_GE(stops, 1) << "失败收口必须留下停止尝试的记录";
}

// ---------------------------------------------------------------------------
// 5b. 错误回包：失败，不自动重试
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 错误回包失败且不自动重试)
{
    Fixture f(/*snapToTarget=*/true);
    ASSERT_TRUE(f.waitConnected());
    f.dev.rejectMoveAxis = 3;   // 两轴都回 EE EE EE EE

    std::string error;
    ASSERT_TRUE(f.service->requestMove(1.0, 0.0, error)) << error;
    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.commandStage == Stage::Failed;
        },
        3000)) << f.service->view().commandDetail;

    EXPECT_EQ(f.controller->diagnostics().lastCommandOutcome,
              data::TurntableCommandOutcome::Failed);
    EXPECT_NE(f.service->view().faultText.find("2001"), std::string::npos)
        << "错误回包应记录通信故障首因：" << f.service->view().faultText;
    const int positioning = positioningFrames(*f.fake);
    sleepMs(300);
    EXPECT_EQ(positioningFrames(*f.fake), positioning)
        << "错误回包后不得自动重试定位";
}

// ---------------------------------------------------------------------------
// 6. 失联/样本过期：拒绝定位，不补发定位帧
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 样本过期时拒绝定位且不补发定位帧)
{
    // 阈值调高：让本轮走到“样本过期”而不是先锁存通信故障。
    Fixture f(/*snapToTarget=*/true, /*commFailureThreshold=*/1000);
    ASSERT_TRUE(f.waitConnected());
    f.dev.dropHorizontal = true;
    f.dev.dropVertical   = true;

    ASSERT_TRUE(f.pump(
        [](const ui::TurntableControlView& v) {
            return v.connection == ui::TurntableConnectionState::Stale ||
                   !v.sampleFresh;
        },
        3000)) << "样本在 sample_stale_ms 后应显示为过期";

    const int positioning = positioningFrames(*f.fake);
    std::string error;
    EXPECT_FALSE(f.service->requestMove(1.0, 0.0, error));
    // 会话把“过期/无效样本”映射为 state().motion == ERROR（旧读数不得
    // 冒充当前角度）；服务预检因此在 ERROR 处拒绝。界面仍必须给出
    // “无新鲜有效双轴样本”，不能只显示一个笼统的故障。
    EXPECT_TRUE(error.find("ERROR") != std::string::npos ||
                error.find("无新鲜有效双轴样本") != std::string::npos)
        << error;
    EXPECT_TRUE(f.service->view().gateText.find("无新鲜有效双轴样本") !=
                std::string::npos)
        << f.service->view().gateText;
    EXPECT_EQ(positioningFrames(*f.fake), positioning)
        << "过期样本不得用于定位，也不得补发定位帧";
    EXPECT_FALSE(f.service->view().positioningEnabled);
}

// ---------------------------------------------------------------------------
// 7. 退出收尾：shutdown() join 停止线程；close 之后不发停止帧
// ---------------------------------------------------------------------------
TEST(TurntableControlServiceTest, 退出收尾join停止线程且不在close后补发)
{
    auto fake = std::make_shared<FakePelcoTransport>();
    {
        FakePelcoDevice dev;
        dev.transport = fake.get();
        dev.snapToTarget = false;
        dev.install();

        data::TurntableConfig cfg = makeConfig(false);
        PelcoDTurntableController controller(cfg, fake);
        std::string error;
        ASSERT_TRUE(controller.enableAcceptancePermit(makePermit(cfg), error))
            << error;
        ASSERT_TRUE(controller.initialize());

        TurntableControlService service(controller, cfg, &controller);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            service.poll();
            if (service.view().connection ==
                ui::TurntableConnectionState::Connected) {
                break;
            }
            sleepMs(2);
        }

        ASSERT_TRUE(service.requestMove(2.0, 0.0, error)) << error;
        bool delivered = false;
        const auto deliverDeadline = std::chrono::steady_clock::now() +
                                     std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deliverDeadline) {
            service.poll();
            if (service.view().commandStage == Stage::Delivered) {
                delivered = true;
                break;
            }
            sleepMs(2);
        }
        ASSERT_TRUE(delivered);

        service.requestStop();
        service.shutdown();   // 必须 join 停止线程，不能留下后台访问控制器

        EXPECT_EQ(stopFrames(*fake), 1);
        EXPECT_FALSE(service.busy());
    }

    // 控制器析构/传输关闭后，不得再尝试发送停止帧。
    EXPECT_EQ(fake->stopSendsAfterClose.load(), 0);
}

}  // namespace
}  // namespace app
}  // namespace aircraft
