// ============================================================================
//  tests/device/DeviceLayerTest.cpp
//
//  依据：ENG-06（测试工程设计）、ENG-08 §13
//        SYS-06（设备抽象层）、SYS-08 §7.5（降级表）、§10（收敛性测试）
//
//  覆盖范围（本文件聚焦**设备层自身的契约**）：
//    · MultiCameraManager 的三种相机数量情形（3 / 2 / ≤1），
//      对应 SYS-08 §7.5 降级表的三行；
//    · triggerTimestamp 的最大值规则（ENG-09 §2.5）；
//    · VirtualTurntable 的行程限位（SYS-08 §7.7 能力边界，不重试）
//      与运动时序（MOVING → STABLE）；
//    · 触发失效的错误码（SYS-08 §7.5 的 3001）；
//    · 真实设备桩必须如实报告不可用。
//
//  ⚠ 不在本文件覆盖的内容：
//    · SYS-08 §10 的 7 个**状态机收敛性**用例 —— 它们需要
//      application 层的 StateMachine 与 RetryManager（010 阶段），
//      属于 tests/integration/ 的范围；
//    · 三相机同步判据（syncToleranceNs）—— 归 optical 层的
//      CameraSynchronizer，属于 tests/optical/ 的范围。
//    在此重复实现会让同一判据有两份实现，而两份实现迟早会不一致。
// ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "data/CameraConfig.h"
#include "data/ErrorInfo.h"
#include "data/TriggerConfig.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "device/camera/ImvCameraBackend.h"
#include "device/camera/MultiCameraManager.h"
#include "device/camera/VirtualCameraBackend.h"
#include "device/turntable/PekoTurntableController.h"
#include "device/turntable/VirtualTurntable.h"
#include "device/trigger/HardwareTriggerController.h"
#include "device/trigger/VirtualTriggerController.h"

namespace aircraft
{
namespace device
{
namespace
{

// ---------------------------------------------------------------------------
//  测试夹具辅助
// ---------------------------------------------------------------------------

data::CameraConfig makeCameraConfig(data::CameraRole role, const std::string& id)
{
    data::CameraConfig c;
    c.cameraId     = id;
    c.role         = role;
    c.width        = 640;
    c.height       = 480;
    c.exposureTime = 0.005;  // s
    c.gain         = 3.0;    // dB
    c.triggerMode  = true;
    return c;
}

std::shared_ptr<VirtualCameraBackend> makeVirtualCamera(data::CameraRole role,
                                                        const std::string& id)
{
    return std::make_shared<VirtualCameraBackend>(makeCameraConfig(role, id));
}

/// 构造三路虚拟相机的管理器。
std::unique_ptr<MultiCameraManager> makeManager()
{
    return std::make_unique<MultiCameraManager>(
        makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
        makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
        makeVirtualCamera(data::CameraRole::CAM100, "cam100"));
}

data::TurntableConfig makeTurntableConfig()
{
    data::TurntableConfig tc;
    tc.protocol       = "sdk";
    tc.azimuthMin     = -30.0;   // deg
    tc.azimuthMax     = 30.0;    // deg
    tc.elevationMin   = -10.0;   // deg
    tc.elevationMax   = 40.0;   // deg
    tc.coarseSpeed    = 5.0;     // deg/s
    tc.fineSpeed      = 0.5;     // deg/s
    tc.centerThreshold = 50.0;   // pixel
    return tc;
}

/// 按**脚本**报出 frameId 的相机替身（裁决 D-C02-6 的验证手段）。
///
/// ⚠ 为什么不用 `VirtualCameraBackend`：它的计数器恒从 0 起且只增，
/// 于是 `MultiCameraManager` 的"首帧基准锁定"退化为恒等映射
/// （基准恒为 0，减法与不减法得到同一个值），而**真机上必然出现**的
/// 两种情形在测试里根本无法出现：
///   · 三台相机的计数器各有任意起点（相差成千上万）；
///   · 相机重连 / 计数器回绕导致帧号**回退**。
/// 只测虚拟相机会得到一个"看起来通过"的结论。
///
/// 脚本用尽后重复最后一个值 —— 用于构造"帧号不前进"的后端
/// （即 `ImvCameraBackend` 尚未填 frameId 时的真实表现：恒为 0）。
class ScriptedCameraBackend : public ICameraBackend
{
public:
    ScriptedCameraBackend(data::CameraRole role, std::vector<uint64_t> frameIds)
        : role_(role)
        , frameIds_(std::move(frameIds))
    {
    }

    /// 令接下来的 N 次 grab 失败（构造"该路中途掉线"）。
    int failNextGrabs = 0;

    bool initialize() override { return true; }
    bool start() override { return true; }
    void stop() override {}

    bool setTriggerMode(bool enable) override { return enable; }

    bool grab(data::ImageFrame& frame) override
    {
        if (failNextGrabs > 0)
        {
            --failNextGrabs;
            return false;
        }
        if (frameIds_.empty())
        {
            return false;
        }

        const std::size_t i = std::min(cursor_, frameIds_.size() - 1);
        ++cursor_;

        frame            = data::ImageFrame{};
        frame.role       = role_;
        frame.cameraId   = "scripted";
        frame.frameId    = frameIds_[i];
        frame.timestampNs = 1000;
        frame.exposureTime = 0.005;
        frame.image      = cv::Mat::ones(8, 8, CV_8UC1);
        return true;
    }

private:
    data::CameraRole       role_;
    std::vector<uint64_t>  frameIds_;
    std::size_t            cursor_ = 0;
};

/// 用脚本相机装配一个三路管理器（三路起点互不相同，模拟真机）。
std::unique_ptr<MultiCameraManager> makeScriptedManager(
    std::vector<uint64_t> cam25,
    std::vector<uint64_t> cam50,
    std::vector<uint64_t> cam100)
{
    return std::make_unique<MultiCameraManager>(
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM25,
                                               std::move(cam25)),
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM50,
                                               std::move(cam50)),
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM100,
                                               std::move(cam100)));
}

// ---------------------------------------------------------------------------
//  MultiCameraManager —— SYS-08 §7.5 降级表第 1 行：3 台正常
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, ThreeCamerasCaptureAllChannels)
{
    auto mgr = makeManager();

    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    EXPECT_EQ(mgr->availableCameraCount(), 3);
    EXPECT_FALSE(mgr->degraded());
    EXPECT_EQ(mgr->state(), data::DeviceState::RUNNING);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame));

    EXPECT_FALSE(frame.cam25.image.empty());
    EXPECT_FALSE(frame.cam50.image.empty());
    EXPECT_FALSE(frame.cam100.image.empty());
}

TEST(MultiCameraManagerTest, CaptureFillsRoleFieldOnEveryChannel)
{
    // role 由管理器按通道补齐，而非依赖 backend 自行填写。
    // 若缺失，会出现"frame.cam25 里装的是 CAM100 的图"这类
    // 无法从数据本身察觉的错位，而三相机评分链完全依赖它。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame));

    EXPECT_EQ(frame.cam25.role, data::CameraRole::CAM25);
    EXPECT_EQ(frame.cam50.role, data::CameraRole::CAM50);
    EXPECT_EQ(frame.cam100.role, data::CameraRole::CAM100);
}

TEST(MultiCameraManagerTest, TriggerTimestampIsMaxOfChannelTimestamps)
{
    // ENG-09 §2.5 冻结：triggerTimestamp = 各相机 timestampNs 的最大值。
    // 取最大而非均值，保证它不早于任何一路的实际曝光时刻。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame));

    const uint64_t expected = std::max({frame.cam25.timestampNs,
                                       frame.cam50.timestampNs,
                                       frame.cam100.timestampNs});
    EXPECT_EQ(frame.triggerTimestamp, expected);
}

// ---------------------------------------------------------------------------
//  SYS-08 §7.5 降级表第 2 行：2 台 → 降级继续
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, TwoCamerasDegradeButCaptureSucceeds)
{
    // 关键判据：capture() **必须返回 true**。
    // 把"少一台"当作失败会白白丢弃 §7.5 明确允许的降级能力，
    // 让一次本可完成的测量变成 FAILED。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    ASSERT_TRUE(mgr->disableChannel(data::CameraRole::CAM100));

    EXPECT_EQ(mgr->availableCameraCount(), 2);
    EXPECT_TRUE(mgr->degraded());

    data::MultiCameraFrame frame;
    EXPECT_TRUE(mgr->capture(frame));

    // 不可用的通道留默认构造的 ImageFrame，以 image.empty() 判定 ——
    // 而不是引入哨兵枚举值（见 CameraRole.h 的说明）。
    EXPECT_TRUE(frame.cam100.image.empty());
    EXPECT_FALSE(frame.cam25.image.empty());
    EXPECT_FALSE(frame.cam50.image.empty());
}

TEST(MultiCameraManagerTest, CaptureMidwayFaultDisablesChannelWithoutRetrying)
{
    // SYS-08 §7.5：采集途中掉线属硬件故障，**不重试**，直接禁用。
    // 若在此处重试，一个物理上不可恢复的故障会消耗掉 §7.3 的重试预算，
    // 使真正属于瞬态类的失败失去重试机会。
    auto cam100 = makeVirtualCamera(data::CameraRole::CAM100, "cam100");
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
                           cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    cam100->simulateFault(true);

    data::MultiCameraFrame frame;
    EXPECT_TRUE(mgr.capture(frame));  // 2 台仍可用 → 降级继续
    EXPECT_EQ(mgr.availableCameraCount(), 2);
    EXPECT_TRUE(mgr.degraded());
}

// ---------------------------------------------------------------------------
//  SYS-08 §7.5 降级表第 3 行：≤1 台 → 直接失败，code=1001
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, InsufficientCamerasFailsWithoutConsumingBudget)
{
    auto cam50  = makeVirtualCamera(data::CameraRole::CAM50, "cam50");
    auto cam100 = makeVirtualCamera(data::CameraRole::CAM100, "cam100");
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           cam50, cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    cam50->simulateFault(true);
    cam100->simulateFault(true);

    data::MultiCameraFrame frame;
    EXPECT_FALSE(mgr.capture(frame));
    EXPECT_EQ(mgr.availableCameraCount(), 1);

    // ENG-09 §5.27：代码中不得出现裸错误码字面量，一律用具名常量
    EXPECT_EQ(mgr.lastError().code, data::kErrCameraInsufficient);
}

TEST(MultiCameraManagerTest, NullBackendsCountAsUnavailable)
{
    // 只注入两路：这是"现场只装了两台相机"的正常配置，
    // 应降级工作而非初始化失败。
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
                           nullptr);
    EXPECT_TRUE(mgr.initializeAll());
    EXPECT_EQ(mgr.availableCameraCount(), 2);
    EXPECT_TRUE(mgr.degraded());
}

TEST(MultiCameraManagerTest, CaptureBeforeStartFails)
{
    // 未 start 就采集：拒绝，而不是把 SDK 的未定义行为泄漏给上层。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());

    data::MultiCameraFrame frame;
    EXPECT_FALSE(mgr->capture(frame));
}

// ---------------------------------------------------------------------------
//  MultiCameraFrame::exposureIndex —— 裁决 D-C02-6
//
//  这组用例的全部意义在于：`exposureIndex` 是"这几张 raw 是不是同一次
//  曝光"的**唯一**判据，而它一旦取错（用绝对帧号、用某一路的帧号、
//  帧号回退时不重锁），错误值**看起来仍然是一个合法的正整数** ——
//  没有任何下游会报错。故必须用可控的帧号序列把每条边都走到。
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, ExposureIndexStartsAtOneAndIgnoresDeviceFrameIdOrigin)
{
    // 三台相机的计数器起点互不相同（真机上必然如此，SYS-06 §8 只要求
    // 各机自身单调，从未要求三机同起点）。
    auto mgr = makeScriptedManager({100, 101, 102}, {7, 8, 9}, {0, 1, 2});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame));

    // 一次 `capture()` 恰对应一次触发曝光 ⇒ 第一次采集的序号必须是 1。
    // 0 的含义是**未定**（MultiCameraFrame 的字段说明），不得占用。
    EXPECT_EQ(frame.exposureIndex, 1u)
        << "首次曝光应为 1；若等于某一路的绝对帧号（100/7/0），"
           "说明基准锁定没有生效";

    ASSERT_TRUE(mgr->capture(frame));
    EXPECT_EQ(frame.exposureIndex, 2u) << "第二次曝光应为 2";
    ASSERT_TRUE(mgr->capture(frame));
    EXPECT_EQ(frame.exposureIndex, 3u);

    // 基准锁定是正常路径，不得借"降级"标记蒙混过关
    // （它一旦置位，调用方就没有别的办法判断序号是否可信）。
    EXPECT_FALSE(mgr->exposureIndexDegraded())
        << "帧号单调前进时不应报降级";
}

TEST(MultiCameraManagerTest, ExposureIndexFlagsDegradedOnFrameIdRollbackWithoutLosingMonotonicity)
{
    // 相机重连 / 计数器回绕：第 3 帧的帧号回到 3（小于基准 5）。
    auto mgr = makeScriptedManager({5, 6, 3, 4}, {0, 1, 2, 3}, {0, 1, 2, 3});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    uint64_t previous = 0;
    const uint64_t expected[4] = {1, 2, 3, 4};

    for (int i = 0; i < 4; ++i)
    {
        ASSERT_TRUE(mgr->capture(frame)) << "第 " << (i + 1) << " 次采集失败";

        // 不变量：序号**严格递增**（调用方靠它区分"同一次曝光"与"下一次"）。
        EXPECT_GT(frame.exposureIndex, previous)
            << "第 " << (i + 1) << " 次采集的序号没有前进";
        EXPECT_EQ(frame.exposureIndex, expected[i])
            << "第 " << (i + 1) << " 次采集：帧号回退后应就地重锁基准，"
               "而不是下溢成一个巨大的序号";
        previous = frame.exposureIndex;
    }

    // 重锁是"序号已不能直接反映设备帧号"这件事，必须可见。
    EXPECT_TRUE(mgr->exposureIndexDegraded())
        << "帧号回退后未置降级标记 —— 序号来源已经换人却无人知晓";
}

TEST(MultiCameraManagerTest, ExposureIndexFlagsDegradedWhenFrameIdDoesNotAdvance)
{
    // 后端不填 frameId（`ImvCameraBackend` 实施前的真实表现：恒为 0）。
    // 此时减法结果恒等于 1 —— 而 1 恰好是一个"合法"的序号，
    // 于是"这几张 raw 是不是同一次曝光"的判断会**恒为真**。
    auto mgr = makeScriptedManager({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame));
    EXPECT_EQ(frame.exposureIndex, 1u);

    ASSERT_TRUE(mgr->capture(frame));
    ASSERT_TRUE(mgr->capture(frame));

    // 保住单调性（这是调用方依赖的不变量），同时把"序号不再来自设备帧号"
    // 记录成可见事实。
    EXPECT_EQ(frame.exposureIndex, 3u)
        << "帧号不前进时序号不得停在 1（恒值会让同源判断永远为真）";
    EXPECT_TRUE(mgr->exposureIndexDegraded())
        << "帧号不前进却未报降级";
}

TEST(MultiCameraManagerTest, ExposureIndexKeepsAdvancingWhenReferenceChannelDrops)
{
    // 第 3 次采集起 CAM25 掉线：参考路按 CAM25→CAM50→CAM100 顺序换成
    // CAM50，而 CAM50 的计数器起点与 CAM25 不同（10 vs 0）。
    auto cam25 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM25, std::vector<uint64_t>{10, 11, 12, 13});
    auto cam50 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM50, std::vector<uint64_t>{0, 1, 2, 3});
    auto cam100 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM100, std::vector<uint64_t>{0, 1, 2, 3});

    MultiCameraManager mgr(cam25, cam50, cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr.capture(frame));
    EXPECT_EQ(frame.exposureIndex, 1u);
    ASSERT_TRUE(mgr.capture(frame));
    EXPECT_EQ(frame.exposureIndex, 2u);

    cam25->failNextGrabs = 2;   // 第 3 次起 CAM25 连续丢帧

    uint64_t previous = frame.exposureIndex;
    for (int i = 0; i < 2; ++i)
    {
        ASSERT_TRUE(mgr.capture(frame)) << "两路可用仍应采集成功（§7.5 降级）";
        EXPECT_GT(frame.exposureIndex, previous)
            << "换参考路后序号必须继续前进，否则两次曝光会被判为同一次";
        previous = frame.exposureIndex;
    }

    EXPECT_TRUE(mgr.exposureIndexDegraded())
        << "参考路换人后未置降级标记";
}

// ---------------------------------------------------------------------------
//  VirtualTurntable
// ---------------------------------------------------------------------------

TEST(VirtualTurntableTest, MoveTransitionsToMovingThenStable)
{
    // SYS-08 的 ALIGN → STABILIZE 两状态切分假定 move() **不阻塞到到位**，
    // 到位与否由轮询 state() 判断。本用例固定该契约。
    uint64_t fakeNowNs = 0;

    VirtualTurntable turntable(makeTurntableConfig());
    turntable.setClock([&fakeNowNs] { return fakeNowNs; });

    ASSERT_TRUE(turntable.initialize());
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::IDLE);

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 10.0;  // deg
    cmd.elevationCommand = 5.0;   // deg

    ASSERT_TRUE(turntable.move(cmd));
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::MOVING);

    // 快进足够长的时间（5 s 远超 20 deg / 5 deg/s = 4 s）
    fakeNowNs += 5ULL * 1000000000ULL;

    const data::TurntableState s = turntable.state();
    EXPECT_EQ(s.motion, data::TurntableMotionState::STABLE);
    EXPECT_NEAR(s.azimuth, 10.0, 1e-9);
    EXPECT_NEAR(s.elevation, 5.0, 1e-9);
}

TEST(VirtualTurntableTest, OutOfTravelCommandIsRejectedAsCapabilityLimit)
{
    // SYS-08 §7.7：转台超行程属**能力边界**，不重试。
    // 检查必须发生在**命令下发前** —— 运动结束后才发现超限，
    // 意味着真机上转台已经撞到机械限位。
    VirtualTurntable turntable(makeTurntableConfig());
    ASSERT_TRUE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 45.0;  // 超出 [-30, 30] deg
    cmd.elevationCommand = 0.0;

    EXPECT_FALSE(turntable.move(cmd));
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(turntable.lastError().code, data::kErrTurntableOverTravel);
}

TEST(VirtualTurntableTest, InvalidLimitsAreRejectedAtInitialize)
{
    // 上限不大于下限属配置错误。若不在此处拒绝，错误会推迟到第一次
    // move() 时以"超行程"形式出现，排查方向会被引向对准算法而非配置。
    data::TurntableConfig bad = makeTurntableConfig();
    bad.azimuthMax = bad.azimuthMin;

    VirtualTurntable turntable(bad);
    EXPECT_FALSE(turntable.initialize());
}

TEST(VirtualTurntableTest, StopDoesNotClearErrorState)
{
    // 故障需要显式处理。一次 stop() 不应把 2002 的记录抹掉 ——
    // 否则任务收尾时的 stop() 调用会让超行程的事实消失。
    VirtualTurntable turntable(makeTurntableConfig());
    ASSERT_TRUE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 999.0;
    cmd.elevationCommand = 0.0;
    ASSERT_FALSE(turntable.move(cmd));

    turntable.stop();
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::ERROR);
}

// ---------------------------------------------------------------------------
//  触发
// ---------------------------------------------------------------------------

TEST(VirtualTriggerControllerTest, TriggerRequiresEnable)
{
    data::TriggerConfig cfg;
    cfg.source          = "virtual";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    VirtualTriggerController trigger(cfg);
    ASSERT_TRUE(trigger.initialize());

    EXPECT_FALSE(trigger.trigger());  // 未使能
    ASSERT_TRUE(trigger.enable());
    EXPECT_TRUE(trigger.trigger());
    EXPECT_TRUE(trigger.trigger());
    EXPECT_EQ(trigger.triggerCount(), 2u);
}

TEST(VirtualTriggerControllerTest, SimulatedFailureReportsDegradeCode)
{
    // SYS-08 §7.5：触发故障 → 降级为软触发并记录 code=3001。
    // 该路径必须可测，否则在只有虚拟设备的第一阶段永远不被执行，
    // 其正确性只能等现场接入真实硬触发时才知道。
    data::TriggerConfig cfg;
    cfg.source          = "virtual";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    VirtualTriggerController trigger(cfg);
    ASSERT_TRUE(trigger.initialize());
    ASSERT_TRUE(trigger.enable());

    trigger.simulateFailure(true);
    EXPECT_FALSE(trigger.trigger());
    EXPECT_EQ(trigger.lastError().code, data::kErrTriggerDegraded);
}

// ---------------------------------------------------------------------------
//  真实设备桩：必须如实报告不可用
// ---------------------------------------------------------------------------

TEST(ImvCameraBackendTest, ReportsUnavailableWithoutSdk)
{
    // 桩若"假装成功"，会让可用相机数虚高为 3，使 §7.5 的降级判据失效，
    // 而 grab() 失败又会被当成"采集途中掉线"触发降级 —— 两条路径
    // 互相矛盾，表现为"三台都在却反复降级"。
    ImvCameraBackend backend(makeCameraConfig(data::CameraRole::CAM25, "real"));

    EXPECT_FALSE(backend.initialize());
    EXPECT_EQ(backend.state(), data::DeviceState::ERROR);

    data::ImageFrame frame;
    EXPECT_FALSE(backend.grab(frame));
}

TEST(PekoTurntableControllerTest, ReportsUnavailableAndDoesNotFakeMotion)
{
    // 转台无冗余（SYS-08 §7.5）。若桩假装成功，对准算法会在永远不动的
    // 转台上反复下发命令，直到耗尽 8 次 ALIGN 重试报 code=2003 ——
    // **错误码指向算法，根因却在设备桩**。
    data::TurntableConfig cfg = makeTurntableConfig();
    cfg.protocol = "pekod";

    PekoTurntableController turntable(cfg);
    EXPECT_FALSE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 5.0;
    cmd.elevationCommand = 5.0;
    EXPECT_FALSE(turntable.move(cmd));

    // state() 绝不回放 lastCommand_：那会让对准判据看到"已在目标上"，
    // 使一台没接的转台让整个流程继续走到给出错误结果。
    const data::TurntableState s = turntable.state();
    EXPECT_EQ(s.motion, data::TurntableMotionState::IDLE);
    EXPECT_DOUBLE_EQ(s.azimuth, 0.0);
}

TEST(HardwareTriggerControllerTest, ReportsUnavailable)
{
    data::TriggerConfig cfg;
    cfg.source          = "hardware";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    HardwareTriggerController trigger(cfg);
    EXPECT_FALSE(trigger.initialize());
    EXPECT_FALSE(trigger.enable());
    EXPECT_FALSE(trigger.trigger());
    EXPECT_EQ(trigger.lastError().code, data::kErrTriggerDegraded);
}

}  // namespace
}  // namespace device
}  // namespace aircraft
