// ============================================================================
//  tests/preview/PreviewLayerTest.cpp
//
//  依据：ENG-06 §10（PreviewManager / PreviewQueue 的测试要求）、
//        ENG-06 §6（预览测试目录）、ENG-08 §13
//        SYS-08 §8（AUTO 的状态→焦段映射表，冻结）
//        SYS-09 §2.1 / §11 / §13.1（双流水线、PreviewWorker、队列策略）
//        SYS-01 §12 / SYS-03 §7（默认 CAM25）
//
//  覆盖 005 Preview 的四个类：
//      PreviewQueue    容量夹取、丢旧保新、高帧率输入、阻塞取帧与唤醒
//      PreviewManager  AUTO/MANUAL、默认 CAM25、显示源切换、帧交付与取用
//      PreviewWorker   启停、取帧发布、角色过滤、RAII 退出
//      PreviewConfig   配置夹取与记录
//
//  ⚠ 本目录的测试**不需要 Qt**，也不需要 QCoreApplication 或事件循环
//  —— preview 模块不链接 Qt（ENG-03 §12.4）。这正是把 PreviewWorker
//  实现为 std::thread 而非 QThread 换来的可测性。
//
//  ⚠ 涉及线程的用例（PreviewWorkerTest）**不使用 sleep 做同步**：
//  一律轮询"期待的可观测量 + 超时上限"（waitUntil）。用固定 sleep 的
//  测试在负载高时会随机失败，而失败信息只会显示"某断言为假"，
//  排查方向被引向被测代码而不是测试本身。
//
//  ⚠ 不在本文件覆盖的内容：
//    · PreviewWorker 与真实 VirtualCameraBackend 的联调、以及
//      "预览不阻塞测量"的端到端时延 → tests/integration/（010）；
//    · 队列长时间积压下的内存增长 → tests/stability/。
// ============================================================================

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "data/CameraChannel.h"
#include "data/ImageFrame.h"
#include "data/MeasurementState.h"
#include "data/MultiCameraFrame.h"
#include "data/OpticalRigCalibration.h"
#include "data/PreviewFrame.h"
#include "optical/OpticalRig.h"
#include "preview/PreviewConfig.h"
#include "preview/PreviewManager.h"
#include "preview/PreviewQueue.h"
#include "preview/PreviewWorker.h"

namespace
{

using aircraft::data::CameraRole;
using aircraft::data::MeasurementState;
using aircraft::preview::PreviewConfig;
using aircraft::preview::PreviewManager;
using aircraft::preview::PreviewMode;
using aircraft::preview::PreviewQueue;
using aircraft::preview::PreviewWorker;

/// 轮询等待某个条件成立，带上限。返回条件是否在超时前成立。
///
/// 用它替代 sleep：条件成立即刻返回（快），不成立才等满超时（稳）。
/// 固定 sleep 的写法在 CI 负载高时必然出现随机失败。
template <typename Predicate>
bool waitUntil(Predicate pred, int timeoutMs = 2000)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (pred())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

/// 构造一个带灰度标记的预览帧（灰度值即帧序号，便于断言取到的是哪一帧）。
aircraft::data::PreviewFrame makeFrame(uint64_t seq,
                                       CameraRole role = CameraRole::CAM25,
                                       uint64_t timestampNs = 0)
{
    aircraft::data::PreviewFrame pf;
    pf.frame.image       = cv::Mat(4, 4, CV_8UC1, cv::Scalar(static_cast<double>(seq % 251)));
    pf.frame.frameId     = seq;
    pf.frame.timestampNs = timestampNs != 0 ? timestampNs : seq * 1000;
    pf.frame.role        = role;
    return pf;
}

uint8_t grayOf(const aircraft::data::PreviewFrame& pf)
{
    return pf.frame.image.at<unsigned char>(0, 0);
}

/// 构造一个三路都已填好图像的 MultiCameraFrame（各路的灰度互不相同）。
aircraft::data::MultiCameraFrame makeMultiFrame(uint8_t g25, uint8_t g50, uint8_t g100)
{
    aircraft::data::MultiCameraFrame mcf;
    mcf.cam25.image  = cv::Mat(4, 4, CV_8UC1, cv::Scalar(g25));
    mcf.cam50.image  = cv::Mat(4, 4, CV_8UC1, cv::Scalar(g50));
    mcf.cam100.image = cv::Mat(4, 4, CV_8UC1, cv::Scalar(g100));
    mcf.cam25.role   = CameraRole::CAM25;
    mcf.cam50.role   = CameraRole::CAM50;
    mcf.cam100.role  = CameraRole::CAM100;
    mcf.triggerTimestamp = 12345;
    return mcf;
}

/// 建一个 OpticalRig，并把指定焦段的通道置为 enabled=false。
aircraft::optical::OpticalRig makeRigWithDisabled(std::vector<CameraRole> disabled)
{
    aircraft::optical::OpticalRig rig;
    rig.initialize();

    aircraft::data::OpticalRigCalibration cal = rig.calibration();
    std::vector<aircraft::data::CameraChannel> channels = rig.cameras();

    for (auto& ch : channels)
    {
        for (CameraRole r : disabled)
        {
            if (ch.role == r)
            {
                ch.enabled = false;
            }
        }
    }

    rig.initialize(channels);  // 用改过 enabled 的表重建
    return rig;
}

}  // namespace

// ===========================================================================
//  PreviewQueue
// ===========================================================================

TEST(PreviewQueueTest, DefaultCapacityIsWithinFrozenRange)
{
    PreviewQueue q;

    // SYS-09 §13.1 / ENG-05 §13.1：capacity = 3~5
    EXPECT_EQ(q.capacity(), 5u);
    EXPECT_GE(q.capacity(), PreviewQueue::kMinCapacity);
    EXPECT_LE(q.capacity(), PreviewQueue::kMaxCapacity);
    EXPECT_FALSE(q.clamped());
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.size(), 0u);
}

TEST(PreviewQueueTest, CapacityIsClampedIntoFrozenRangeAndReported)
{
    // 越界容量被夹取，且夹取这件事可被查出（不得静默降级）。
    //
    // 为什么不干脆接受任意容量：容量的物理含义是"允许积压多少帧"，
    // 60 fps 下容量 100 意味着预览要比现实晚约 1.7 s 才显示，
    // "实时"预览就变成了录像回放。
    PreviewQueue tooSmall(1);
    EXPECT_EQ(tooSmall.capacity(), PreviewQueue::kMinCapacity);
    EXPECT_TRUE(tooSmall.clamped());

    PreviewQueue tooBig(100);
    EXPECT_EQ(tooBig.capacity(), PreviewQueue::kMaxCapacity);
    EXPECT_TRUE(tooBig.clamped());

    PreviewQueue zero(0);
    EXPECT_EQ(zero.capacity(), PreviewQueue::kMinCapacity);
    EXPECT_TRUE(zero.clamped());
}

TEST(PreviewQueueTest, DropOldestKeepsNewest)
{
    PreviewQueue q(3);

    // 压入 5 帧，容量 3：应留下**最后** 3 帧（3、4、5）
    for (uint64_t i = 1; i <= 5; ++i)
    {
        q.push(makeFrame(i));
    }

    EXPECT_EQ(q.size(), 3u);
    EXPECT_EQ(q.pushedCount(), 5u);
    EXPECT_EQ(q.droppedCount(), 2u);

    aircraft::data::PreviewFrame out;
    for (uint64_t expected = 3; expected <= 5; ++expected)
    {
        ASSERT_TRUE(q.pop(out)) << "应还能取出第 " << expected << " 帧";
        EXPECT_EQ(out.frame.frameId, expected) << "丢的必须是最旧的，留下的必须是最新的";
    }
    EXPECT_FALSE(q.pop(out));
}

TEST(PreviewQueueTest, PopOnEmptyReturnsFalse)
{
    PreviewQueue q(3);
    aircraft::data::PreviewFrame out;
    EXPECT_FALSE(q.pop(out));
}

TEST(PreviewQueueTest, LatestDoesNotRemove)
{
    PreviewQueue q(3);
    q.push(makeFrame(1));
    q.push(makeFrame(2));

    aircraft::data::PreviewFrame out;
    ASSERT_TRUE(q.latest(out));
    EXPECT_EQ(out.frame.frameId, 2u);
    EXPECT_EQ(q.size(), 2u) << "latest() 只读不取";
}

TEST(PreviewQueueTest, ClearEmptiesQueueButKeepsCounters)
{
    PreviewQueue q(3);
    for (uint64_t i = 1; i <= 5; ++i)
    {
        q.push(makeFrame(i));
    }
    ASSERT_EQ(q.droppedCount(), 2u);

    q.clear();
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.droppedCount(), 2u)
        << "丢帧计数是跨运行期的观测量，clear() 不应清零";
    EXPECT_EQ(q.pushedCount(), 5u);
}

TEST(PreviewQueueTest, WaitPopReturnsFalseOnTimeout)
{
    PreviewQueue q(3);
    aircraft::data::PreviewFrame out;

    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(q.waitPop(out, 30));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0).count();

    EXPECT_GE(elapsed, 25) << "应至少等满超时时间，实际 " << elapsed << " ms";
}

TEST(PreviewQueueTest, WaitPopIsWokenByPush)
{
    PreviewQueue q(3);
    aircraft::data::PreviewFrame out;

    std::atomic<bool> got{false};
    std::thread consumer([&] {
        aircraft::data::PreviewFrame f;
        if (q.waitPop(f, 3000))
        {
            out = f;
            got = true;
        }
    });

    // 让它先进入等待，再压入 —— 唤醒路径才被真正走到。
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.push(makeFrame(42));
    consumer.join();

    // 关键：被唤醒后立刻拿到帧，而不是等满 3 s。
    EXPECT_TRUE(got.load());
    EXPECT_EQ(out.frame.frameId, 42u);
}

TEST(PreviewQueueTest, WaitPopReturnsFalseAfterWakeAll)
{
    PreviewQueue q(3);
    aircraft::data::PreviewFrame out;

    const auto t0 = std::chrono::steady_clock::now();
    std::thread consumer([&] { EXPECT_FALSE(q.waitPop(out, 5000)); });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.wakeAll();  // 关闭流程
    consumer.join();

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0).count();
    EXPECT_LT(elapsed, 2000) << "wakeAll 应立即放行，而不是等满 5 s";
}

TEST(PreviewQueueTest, WakeIsOneShotNotSticky)
{
    PreviewQueue q(3);
    aircraft::data::PreviewFrame out;

    q.wakeAll();

    // 唤醒标记是一次性的：第一次 waitPop 立刻返回 false，
    // 第二次必须重新正常等待（否则关闭流程之外的调用会退化成忙轮询）。
    EXPECT_FALSE(q.waitPop(out, 20));

    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_FALSE(q.waitPop(out, 40));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0).count();
    EXPECT_GE(elapsed, 30) << "第二次应当真的等待，实际只等了 " << elapsed << " ms";
}

TEST(PreviewQueueTest, HighRateInputConservesEveryFrame)
{
    // ENG-06 §10 要求验证"高帧率输入"。
    // 最强的判据不是"没崩"，而是守恒：每一帧要么被消费者取走，
    // 要么作为最旧帧被丢弃，一帧不多一帧不少。
    PreviewQueue q(3);

    constexpr uint64_t kFrames = 20000;
    std::atomic<uint64_t> popped{0};

    std::thread consumer([&] {
        aircraft::data::PreviewFrame f;
        while (popped.load() + q.droppedCount() < kFrames)
        {
            if (q.waitPop(f, 5))
            {
                ++popped;
            }
        }
        // 收尾：把队里剩下的取空
        while (q.pop(f))
        {
            ++popped;
        }
    });

    for (uint64_t i = 1; i <= kFrames; ++i)
    {
        q.push(makeFrame(i));
    }

    consumer.join();

    EXPECT_EQ(q.pushedCount(), kFrames);
    EXPECT_EQ(popped.load() + q.droppedCount(), kFrames)
        << "取出 " << popped.load() << " + 丢弃 " << q.droppedCount()
        << " 应等于压入 " << kFrames;
    EXPECT_LE(q.size(), q.capacity()) << "队列长度不得超过容量";
    EXPECT_GT(q.droppedCount(), 0u) << "生产者远快于消费者，必然发生丢旧保新";
}

// ===========================================================================
//  PreviewManager — 默认值与模式
// ===========================================================================

TEST(PreviewManagerTest, DefaultsToCam25InAutoMode)
{
    // SYS-01 §12 / SYS-03 §7 冻结默认 CAM25；ENG-06 §10 要求验证这一点。
    PreviewManager pm;

    EXPECT_EQ(pm.mode(), PreviewMode::AUTO);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25);
    EXPECT_EQ(pm.camera(), CameraRole::CAM25);
    EXPECT_EQ(pm.autoCamera(), CameraRole::CAM25);
    EXPECT_TRUE(pm.configNotices().empty()) << "默认配置不应产生任何告警";

    // 尚未发布任何帧
    aircraft::data::PreviewFrame f;
    EXPECT_FALSE(pm.getFrame(f));
    EXPECT_FALSE(pm.latestFrame(f));
}

TEST(PreviewManagerTest, ManualModeFollowsSetCamera)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_EQ(pm.mode(), PreviewMode::MANUAL);

    EXPECT_TRUE(pm.setCamera(CameraRole::CAM50));
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM50);
    EXPECT_EQ(pm.camera(), CameraRole::CAM50);

    EXPECT_TRUE(pm.setCamera(CameraRole::CAM100));
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM100);
}

TEST(PreviewManagerTest, AutoModeFollowsTheFrozenStateTable)
{
    // SYS-08 §8 冻结的三行：SEARCH → CAM25、ALIGN → CAM50、
    // MEASURE（Selected Camera）→ 由选择算法给出的焦段。
    PreviewManager pm;
    pm.setMode(PreviewMode::AUTO);
    pm.setAutoCamera(CameraRole::CAM100);  // 假设选择算法选了 100mm

    pm.setMeasurementState(MeasurementState::SEARCH);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25) << "SEARCH 必须用大视场";

    pm.setMeasurementState(MeasurementState::ALIGN);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM50) << "ALIGN 用中焦";

    pm.setMeasurementState(MeasurementState::CAPTURE);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM100)
        << "测量阶段用 Selected Camera";
}

TEST(PreviewManagerTest, AutoModeHoldsSourceOnFailure)
{
    PreviewManager pm;
    pm.setMeasurementState(MeasurementState::ALIGN);  // → CAM50
    ASSERT_EQ(pm.displayCamera(), CameraRole::CAM50);

    // FAILED 时保持原显示源：故障时把画面切走，操作者就看不到
    // 出问题的那一路了。
    pm.setMeasurementState(MeasurementState::FAILED);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM50);
}

TEST(PreviewManagerTest, MapStateToCameraCoversSearchAlignMeasure)
{
    // 冻结的三行，逐行验证（此函数被单测暴露出来正是为了能这样验）。
    EXPECT_EQ(PreviewManager::mapStateToCamera(MeasurementState::SEARCH,
                                               CameraRole::CAM100,
                                               CameraRole::CAM50),
              CameraRole::CAM25);
    EXPECT_EQ(PreviewManager::mapStateToCamera(MeasurementState::ALIGN,
                                               CameraRole::CAM100,
                                               CameraRole::CAM25),
              CameraRole::CAM50);
    EXPECT_EQ(PreviewManager::mapStateToCamera(MeasurementState::CAPTURE,
                                               CameraRole::CAM100,
                                               CameraRole::CAM25),
              CameraRole::CAM100);
    EXPECT_EQ(PreviewManager::mapStateToCamera(MeasurementState::FAILED,
                                               CameraRole::CAM100,
                                               CameraRole::CAM50),
              CameraRole::CAM50) << "FAILED 回到 fallback，即保持原显示源";
}

TEST(PreviewManagerTest, ManualModeIsUnaffectedByMeasurementState)
{
    // SYS-08 §8：MANUAL 下"用户控制预览"，状态机不得改变显示源。
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM25));

    pm.setMeasurementState(MeasurementState::ALIGN);   // AUTO 下会切到 CAM50
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25)
        << "MANUAL 模式下状态推进不得改变显示源";

    // 但状态仍应被记录（UI 要显示当前处于哪个状态）
    EXPECT_EQ(pm.measurementState(), MeasurementState::ALIGN);
}

TEST(PreviewManagerTest, ManualChoiceSurvivesAutoAndTakesEffectOnReturn)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM100));

    // 切到 AUTO：显示源由状态机决定，用户的选择被记住但不生效
    pm.setMode(PreviewMode::AUTO);
    pm.setMeasurementState(MeasurementState::SEARCH);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25);
    EXPECT_EQ(pm.camera(), CameraRole::CAM100) << "用户的选择应被记住";

    // 切回 MANUAL：立即回到用户选择，不需要再点一次
    pm.setMode(PreviewMode::MANUAL);
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM100);
}

TEST(PreviewManagerTest, SetCameraRejectsDisabledChannel)
{
    aircraft::optical::OpticalRig rig = makeRigWithDisabled({CameraRole::CAM100});
    PreviewManager pm(PreviewConfig{}, &rig);

    pm.setMode(PreviewMode::MANUAL);
    ASSERT_EQ(pm.displayCamera(), CameraRole::CAM25);

    // 选择一个已禁用的焦段必须失败并保持原状，而不是"接受但显示不出来"。
    EXPECT_FALSE(pm.setCamera(CameraRole::CAM100));
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25);

    // 可用的焦段仍可切换
    EXPECT_TRUE(pm.setCamera(CameraRole::CAM50));
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM50);

    EXPECT_FALSE(pm.cameraEnabled(CameraRole::CAM100));
    EXPECT_TRUE(pm.cameraEnabled(CameraRole::CAM50));
}

TEST(PreviewManagerTest, WithoutRigEveryChannelIsAccepted)
{
    // rig 为空表示"不校验可用性"（见构造函数说明），此时应一律接受。
    PreviewManager pm;  // 无 rig
    pm.setMode(PreviewMode::MANUAL);
    EXPECT_TRUE(pm.setCamera(CameraRole::CAM100));
    EXPECT_TRUE(pm.cameraEnabled(CameraRole::CAM100));

    // 无 rig 时相机标识回退为固定名字，焦距为 0（UI 应显示"未知"）
    EXPECT_EQ(pm.cameraId(CameraRole::CAM100), "CAM100");
    EXPECT_DOUBLE_EQ(pm.focalLengthMetres(CameraRole::CAM100), 0.0);
}

TEST(PreviewManagerTest, ReportsCameraIdAndFocalLengthInMetres)
{
    aircraft::optical::OpticalRig rig;
    ASSERT_TRUE(rig.initialize());
    PreviewManager pm(PreviewConfig{}, &rig);

    // ENG-09 §2.3：焦距单位是米（100mm 镜头写作 0.1）
    EXPECT_DOUBLE_EQ(pm.focalLengthMetres(CameraRole::CAM25), 0.025);
    EXPECT_DOUBLE_EQ(pm.focalLengthMetres(CameraRole::CAM50), 0.05);
    EXPECT_DOUBLE_EQ(pm.focalLengthMetres(CameraRole::CAM100), 0.1);
    EXPECT_FALSE(pm.cameraId(CameraRole::CAM100).empty());
}

TEST(PreviewManagerTest, QueueSizeClampIsRecordedInNotices)
{
    PreviewConfig cfg;
    cfg.queueSize = 100;
    PreviewManager pm(cfg);

    // 夹取是降级行为，必须可被读出（"不得静默降级"）。
    EXPECT_TRUE(pm.queue().clamped());
    ASSERT_FALSE(pm.configNotices().empty());
    EXPECT_NE(pm.configNotices().front().find("queueSize"), std::string::npos);
}

// ===========================================================================
//  PreviewManager — 帧交付与取用
// ===========================================================================

TEST(PreviewManagerTest, WorkerDeliverFiltersByDisplayRole)
{
    PreviewManager pm;  // 默认 AUTO / CAM25

    // 不属于当前显示源的帧应被丢弃而不是发布
    EXPECT_EQ(pm.workerDeliver(makeFrame(1, CameraRole::CAM100)),
              PreviewManager::DeliverResult::SKIPPED_ROLE_MISMATCH);
    EXPECT_EQ(pm.skippedCount(), 1u);
    EXPECT_EQ(pm.displaySequence(), 0u);

    aircraft::data::PreviewFrame out;
    EXPECT_FALSE(pm.latestFrame(out));

    // 匹配的帧才发布
    EXPECT_EQ(pm.workerDeliver(makeFrame(2, CameraRole::CAM25)),
              PreviewManager::DeliverResult::PUBLISHED);
    EXPECT_EQ(pm.displaySequence(), 1u);
    ASSERT_TRUE(pm.latestFrame(out));
    EXPECT_EQ(out.frame.frameId, 2u);
}

TEST(PreviewManagerTest, GetFrameOnlyReturnsNewFrames)
{
    PreviewManager pm;
    aircraft::data::PreviewFrame out;

    ASSERT_EQ(pm.workerDeliver(makeFrame(1, CameraRole::CAM25)),
              PreviewManager::DeliverResult::PUBLISHED);

    // 第一次取到
    ASSERT_TRUE(pm.getFrame(out));
    EXPECT_EQ(out.frame.frameId, 1u);

    // 没有新帧时再取：返回 false（UI 据此跳过重绘）
    EXPECT_FALSE(pm.getFrame(out)) << "没有新帧时不得重复交付，否则界面会以 60Hz 重绘同一张图";

    // 有新帧后又能取到
    ASSERT_EQ(pm.workerDeliver(makeFrame(2, CameraRole::CAM25)),
              PreviewManager::DeliverResult::PUBLISHED);
    ASSERT_TRUE(pm.getFrame(out));
    EXPECT_EQ(out.frame.frameId, 2u);
}

TEST(PreviewManagerTest, LatestFrameIgnoresFreshness)
{
    PreviewManager pm;
    aircraft::data::PreviewFrame out;

    ASSERT_EQ(pm.workerDeliver(makeFrame(7, CameraRole::CAM25)),
              PreviewManager::DeliverResult::PUBLISHED);
    ASSERT_TRUE(pm.getFrame(out));  // 已消费

    // latestFrame 不受"是否已交付"影响，供按需取图使用
    ASSERT_TRUE(pm.latestFrame(out));
    EXPECT_EQ(out.frame.frameId, 7u);
}

TEST(PreviewManagerTest, SwitchingSourceInvalidatesPublishedFrame)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);

    ASSERT_EQ(pm.workerDeliver(makeFrame(1, CameraRole::CAM25)),
              PreviewManager::DeliverResult::PUBLISHED);
    const uint64_t gen0 = pm.displayGeneration();
    ASSERT_EQ(pm.displaySequence(), 1u);

    // 切到 CAM50
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM50));

    // ⚠ 显示源变了，上一路的帧必须作废：否则界面会用 CAM50 的标签
    // 配 CAM25 的图像，而这种错配看起来完全像"测量结果不对"。
    aircraft::data::PreviewFrame out;
    EXPECT_FALSE(pm.latestFrame(out)) << "切换后不得再把旧焦段的帧当当前画面";
    EXPECT_FALSE(pm.getFrame(out));
    EXPECT_GT(pm.displayGeneration(), gen0) << "代次应递增，使 UI 知道要清空控件";

    // 新焦段的第一帧到达后恢复正常
    ASSERT_EQ(pm.workerDeliver(makeFrame(2, CameraRole::CAM50)),
              PreviewManager::DeliverResult::PUBLISHED);
    ASSERT_TRUE(pm.latestFrame(out));
    EXPECT_EQ(out.frame.frameId, 2u);
    EXPECT_EQ(out.frame.role, CameraRole::CAM50);
}

TEST(PreviewManagerTest, SubmitFrameStampsDisplayTimestamp)
{
    PreviewManager pm;

    aircraft::data::PreviewFrame in = makeFrame(1, CameraRole::CAM25, 5000);
    in.displayTimestamp = 0;  // 调用方没填 / 填 0
    pm.submitFrame(in);

    aircraft::data::PreviewFrame out;
    ASSERT_TRUE(pm.queue().latest(out));
    EXPECT_GT(out.displayTimestamp, 0u)
        << "进入预览队列的时刻应由 submitFrame 统一盖写，否则时延统计失真";
    EXPECT_EQ(out.frame.timestampNs, 5000u) << "采集时刻不得被改动";
}

TEST(PreviewManagerTest, SubmitFromPicksTheDisplayedChannel)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM50));

    // 三路图像灰度互不相同，取哪一路一目了然
    ASSERT_TRUE(pm.submitFrom(makeMultiFrame(11, 22, 33)));

    aircraft::data::PreviewFrame out;
    ASSERT_TRUE(pm.queue().latest(out));
    EXPECT_EQ(grayOf(out), 22) << "应提交 CAM50 那一路（当前显示源）";
    EXPECT_EQ(out.frame.role, CameraRole::CAM50)
        << "role 必须与图像来源一致，否则角色过滤会放行错配的帧";
}

TEST(PreviewManagerTest, SubmitFromReturnsFalseWhenDisplayedChannelIsEmpty)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM100));

    // CAM100 降级/未采集：该路图像为空
    aircraft::data::MultiCameraFrame mcf = makeMultiFrame(11, 22, 33);
    mcf.cam100.image = cv::Mat();

    EXPECT_FALSE(pm.submitFrom(mcf));
    EXPECT_TRUE(pm.queue().empty())
        << "空帧不得进入队列：它会占掉容量并被当作正常数据发布，界面于是显示空白";
}

TEST(PreviewManagerTest, SubmitFromRefusesNonEightBitDisplayImage)
{
    // 边界：进入预览的 `image` 必须是**显示图**（8U）。
    // 一幅 16U 的图进到这里，说明某个后端把原始载荷当成了显示图发布
    // （ENG-09 V2.3 §5.28 第 6 条：`image` 永远是加工产物，不是载荷）。
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM25));

    aircraft::data::MultiCameraFrame mcf = makeMultiFrame(11, 22, 33);
    mcf.cam25.image = cv::Mat(4, 4, CV_16UC1, cv::Scalar(11));

    EXPECT_FALSE(pm.submitFrom(mcf))
        << "16U 图不得进入预览：它会被 Qt 转换按 8U 读，症状是"
           "颜色/亮度不对，而根因在取帧侧的格式映射";
    EXPECT_TRUE(pm.queue().empty());

    // ---- 正对照：同一份帧、同一路，只把深度换成 8U ⇒ 必须提交 ----
    // 没有这一条，"返回 false"也可能是因为别的判据（例如空图）
    // 在这里同样成立，于是这条用例证明不了深度判据存在。
    mcf.cam25.image = cv::Mat(4, 4, CV_8UC1, cv::Scalar(11));
    EXPECT_TRUE(pm.submitFrom(mcf));
    EXPECT_FALSE(pm.queue().empty());
}

TEST(PreviewManagerTest, SubmitFromDoesNotMislableOnSourceSwitch)
{
    // 回归：submitFrom 内部若把 displayCamera() 读两次，切换相机的窗口
    // 会让"旧焦段的图像"配上"新焦段的角色"，而角色过滤恰好会放行它。
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM25));

    ASSERT_TRUE(pm.submitFrom(makeMultiFrame(11, 22, 33)));
    for (int i = 0; i < 50; ++i)
    {
        ASSERT_TRUE(pm.submitFrom(makeMultiFrame(11, 22, 33)));
    }

    aircraft::data::PreviewFrame out;
    while (pm.queue().pop(out))
    {
        // 每一帧的角色与图像必须互相对应
        if (out.frame.role == CameraRole::CAM25)
        {
            EXPECT_EQ(grayOf(out), 11);
        }
    }
    SUCCEED();
}

// ===========================================================================
//  PreviewWorker
// ===========================================================================

TEST(PreviewWorkerTest, StartStopLifecycle)
{
    PreviewManager pm;
    PreviewWorker worker(pm);

    EXPECT_FALSE(worker.running());
    EXPECT_TRUE(worker.start());
    EXPECT_TRUE(worker.running());

    worker.stop();
    EXPECT_FALSE(worker.running());
    EXPECT_EQ(worker.processedCount(), 0u);
}

TEST(PreviewWorkerTest, DoubleStartIsRejected)
{
    PreviewManager pm;
    PreviewWorker worker(pm);

    ASSERT_TRUE(worker.start());
    EXPECT_FALSE(worker.start()) << "已在运行时 start() 应返回 false，不得再建一个线程";

    worker.stop();
    // stop 之后可以重新启动（同一对象可复用）
    EXPECT_TRUE(worker.start());
    worker.stop();
}

TEST(PreviewWorkerTest, StopIsIdempotent)
{
    PreviewManager pm;
    PreviewWorker worker(pm);

    worker.stop();          // 从未启动
    ASSERT_TRUE(worker.start());
    worker.stop();
    worker.stop();          // 重复 stop 不得挂起或崩溃
    EXPECT_FALSE(worker.running());
}

TEST(PreviewWorkerTest, PublishesFramesFromTheQueue)
{
    PreviewManager pm;  // 默认 AUTO / CAM25
    PreviewWorker worker(pm);
    ASSERT_TRUE(worker.start());

    for (uint64_t i = 1; i <= 5; ++i)
    {
        pm.submitFrame(makeFrame(i, CameraRole::CAM25));
    }

    ASSERT_TRUE(waitUntil([&] { return pm.displaySequence() >= 1; }))
        << "worker 应把队列里的帧发布为显示帧";

    EXPECT_TRUE(waitUntil([&] { return worker.publishedCount() >= 1; }));

    aircraft::data::PreviewFrame out;
    ASSERT_TRUE(pm.latestFrame(out));
    EXPECT_EQ(out.frame.role, CameraRole::CAM25);
    EXPECT_GT(out.displayTimestamp, 0u);

    worker.stop();
}

TEST(PreviewWorkerTest, FramesOfOtherRolesAreSkippedNotPublished)
{
    PreviewManager pm;  // 显示源 CAM25
    PreviewWorker worker(pm);
    ASSERT_TRUE(worker.start());

    for (uint64_t i = 1; i <= 5; ++i)
    {
        pm.submitFrame(makeFrame(i, CameraRole::CAM100));
    }

    ASSERT_TRUE(waitUntil([&] { return worker.skippedCount() >= 1; }))
        << "不属于当前显示源的帧应被丢弃";
    EXPECT_EQ(worker.publishedCount(), 0u);
    EXPECT_EQ(pm.displaySequence(), 0u);

    aircraft::data::PreviewFrame out;
    EXPECT_FALSE(pm.latestFrame(out));

    worker.stop();
}

TEST(PreviewWorkerTest, CameraSwitchDuringRunTakesEffect)
{
    PreviewManager pm;
    pm.setMode(PreviewMode::MANUAL);
    PreviewWorker worker(pm);
    ASSERT_TRUE(worker.start());

    pm.submitFrame(makeFrame(1, CameraRole::CAM25));
    ASSERT_TRUE(waitUntil([&] { return pm.displaySequence() >= 1; }));

    // 运行中切换显示源
    ASSERT_TRUE(pm.setCamera(CameraRole::CAM50));

    // 在途的 CAM25 帧应被丢弃（这正是"切换后不闪旧画面"的实现方式）
    for (uint64_t i = 2; i <= 4; ++i)
    {
        pm.submitFrame(makeFrame(i, CameraRole::CAM25));
    }

    // 新焦段的帧应能发布
    pm.submitFrame(makeFrame(10, CameraRole::CAM50));

    ASSERT_TRUE(waitUntil([&] {
        aircraft::data::PreviewFrame f;
        return pm.latestFrame(f) && f.frame.role == CameraRole::CAM50;
    })) << "切换后应能发布新焦段的帧";

    EXPECT_GT(worker.skippedCount(), 0u) << "在途的旧焦段帧应被计入丢弃";

    worker.stop();
}

TEST(PreviewWorkerTest, DestructorStopsTheThread)
{
    // RAII：worker 在作用域结束时析构，即使调用方忘了 stop() 也不得
    // 留下仍在访问 manager 的线程（那会在 manager 析构后 use-after-free）。
    PreviewManager pm;
    {
        PreviewWorker worker(pm);
        ASSERT_TRUE(worker.start());
        pm.submitFrame(makeFrame(1, CameraRole::CAM25));
        ASSERT_TRUE(waitUntil([&] { return pm.displaySequence() >= 1; }));
        // 故意不调用 stop()
    }

    // 能走到这里即说明析构已 join。再确认 manager 仍可用（未被悬挂线程破坏）。
    EXPECT_EQ(pm.displayCamera(), CameraRole::CAM25);
    aircraft::data::PreviewFrame out;
    EXPECT_TRUE(pm.latestFrame(out));
}

TEST(PreviewWorkerTest, ShortPollTimeoutStillProcessesFrames)
{
    PreviewManager pm;
    PreviewWorker worker(pm);
    worker.setPollTimeoutMs(1);  // 测试用：缩短等待

    EXPECT_EQ(worker.pollTimeoutMs(), 1);
    ASSERT_TRUE(worker.start());

    pm.submitFrame(makeFrame(1, CameraRole::CAM25));
    EXPECT_TRUE(waitUntil([&] { return pm.displaySequence() >= 1; }));

    worker.stop();

    // 非法值回退为默认，避免忙轮询
    worker.setPollTimeoutMs(0);
    EXPECT_EQ(worker.pollTimeoutMs(), PreviewWorker::kDefaultPollTimeoutMs);
    worker.setPollTimeoutMs(-5);
    EXPECT_EQ(worker.pollTimeoutMs(), PreviewWorker::kDefaultPollTimeoutMs);
}

TEST(PreviewWorkerTest, HighRateSubmissionDoesNotLoseTheThread)
{
    // 生产者远快于消费者时，worker 应持续工作且不卡死、不无限积压。
    PreviewManager pm;
    PreviewWorker worker(pm);
    ASSERT_TRUE(worker.start());

    for (uint64_t i = 1; i <= 5000; ++i)
    {
        pm.submitFrame(makeFrame(i, CameraRole::CAM25));
    }

    EXPECT_TRUE(waitUntil([&] { return pm.displaySequence() >= 1; }));
    worker.stop();

    EXPECT_LE(pm.queue().size(), pm.queue().capacity());
    EXPECT_EQ(pm.queue().pushedCount(), 5000u);
    EXPECT_EQ(worker.publishedCount() + worker.skippedCount(), worker.processedCount());
}
