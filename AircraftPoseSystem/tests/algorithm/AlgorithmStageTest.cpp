// ============================================================================
//  tests/algorithm/AlgorithmStageTest.cpp
//
//  覆盖（ENG-06 §8：algorithm 层测试 = 算法链各阶段的判据与数值）：
//    · 尺度估计   SYS-14 §10 的 Z = f·L/(l·s)（**f 必须是像素**）
//    · 通道选择   ENG-10 §3.2 的 E 式、§3.6 的归一化、§4.3 的冷启动先验、
//                 SYS-14 §6 的硬门槛
//    · 姿态验证   SYS-07 §12.2 的三项判据 + 姿态合理性
//    · PnP 解算   SYS-07 §10 的位姿恢复与 RANSAC 外点剔除
//
//  ⚠ 本文件承担一个超出普通单测的职责：把**冻结文档里没有写明的算式**
//  钉死。这些量在文档中只有名字（"E"、"confidence"、"M_hist"），实现里
//  必须有确定的定义，而**定义写错不会报错**，只会让选择/判定结果偏斜。
//  因此这些用例里的期望值是**按实现选定的定义逐项手算**的，不是从实现
//  里抄出来的：任何一处定义被改动，用例会失败并指出改了哪一项。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "algorithm/pose/PnPPoseEstimator.h"
#include "algorithm/scale/TargetScaleEstimator.h"
#include "algorithm/selection/MeasurementSelector.h"
#include "algorithm/validation/PoseValidator.h"
#include "data/CameraCalibration.h"
#include "data/CameraPose.h"
#include "data/DetectionResult.h"
#include "data/FeatureCorrespondence.h"
#include "data/ImageFrame.h"
#include "data/IMatchStatsStore.h"
#include "data/ImageQuality.h"
#include "data/MeasurementCandidate.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementSelectionResult.h"
#include "data/ShipPoseResult.h"
#include "data/TargetScaleEstimate.h"
#include "data/ValidationConfig.h"

#include "TestSupport.h"

using aircraft::algorithm::CvPnPPoseEstimator;
using aircraft::algorithm::MeasurementSelector;
using aircraft::algorithm::PinholeScaleEstimator;
using aircraft::algorithm::PnPPoseEstimator;
using aircraft::algorithm::PnpStats;
using aircraft::algorithm::PoseValidator;
using aircraft::data::CameraCalibration;
using aircraft::data::CameraPose;
using aircraft::data::CameraRole;
using aircraft::data::DetectionResult;
using aircraft::data::FeatureCorrespondence;
using aircraft::data::ImageQuality;
using aircraft::data::MeasurementCandidate;
using aircraft::data::MeasurementConfig;
using aircraft::data::MeasurementSelectionResult;
using aircraft::data::PoseValidationReason;
using aircraft::data::PoseValidationResult;
using aircraft::data::ShipPoseResult;
using aircraft::data::TargetScaleEstimate;
using aircraft::data::ValidationConfig;

// 共用夹具见 TestSupport.h（两个 algorithm 测试文件必须对"已配置"这个
// 前提持同一份定义，理由见该文件头）。
using namespace aps_test;


// ===========================================================================
//  一、尺度估计（SYS-14 §10）
// ===========================================================================

TEST(TargetScaleEstimator, PixelsFormGivesCorrectDistance)
{
    // 300 m 处 10 m 长的目标，25 mm 焦段：
    //   l = fx·L/Z = 7246.377 × 10 / 300 ≈ 241.55 pixel
    PinholeScaleEstimator estimator;
    const CameraCalibration calib = calibrationWithFx(kFx25mmPx, kFx25mmPx);

    DetectionResult det;
    det.found = true;
    det.bbox = cv::Rect(1000, 900, 242, 60);   // 长边 242 pixel

    TargetScaleEstimate out;
    ASSERT_TRUE(estimator.estimate(det, calib, 10.0, out));
    EXPECT_NEAR(out.distance, 300.0, 5.0);
    EXPECT_NEAR(out.targetPixelSize, 242.0, 1e-9);
}

TEST(TargetScaleEstimator, FocalLengthInMetresIsDetectablyWrong)
{
    // ⚠ 这是本工程记录在案的最危险的一处单位混淆（SYS-14 §10）：
    //   把 `CameraChannel::focalLength`（单位 m，100 mm 写作 0.1）
    //   放进 `f` 的位置，误差是 7246.4/0.025 ≈ 290 倍，**且不会报错** ——
    //   fx > 0 满足，距离为正满足，一切看起来都正常。
    //   本用例把该错误的**可检测特征**钉死：算出的距离会小 290 倍，
    //   即 300 m 变成约 1.03 m。有了这条断言，"单位写错"表现为一个
    //   荒谬但可判定的数值，而不是一个看似合理的错误距离。
    PinholeScaleEstimator estimator;
    const CameraCalibration wrong = calibrationWithFx(0.025, 0.025);
    (void)kFx25mmPx;

    DetectionResult det;
    det.found = true;
    det.bbox = cv::Rect(1000, 900, 242, 60);

    TargetScaleEstimate out;
    ASSERT_TRUE(estimator.estimate(det, wrong, 10.0, out));
    EXPECT_LT(out.distance, 5.0);   // 正确值是 300 m，此处置信为"明显错误"
}

TEST(TargetScaleEstimator, RejectsInvalidInputs)
{
    PinholeScaleEstimator estimator;
    TargetScaleEstimate out;

    DetectionResult det;
    det.found = true;
    det.bbox = cv::Rect(0, 0, 242, 60);

    // 目标真实尺寸未知 → 无从估计（且不得给猜测值：它会污染 M_hist 分桶）
    EXPECT_FALSE(estimator.estimate(det, calibrationWithFx(kFx25mmPx, kFx25mmPx),
                                    0.0, out));

    // 默认构造的内参（fx = fy = 1）必须被拒：它是一个"合法但不正确"的
    // 内参，直接使用会得出一个量级完全错误却依然"成功"的距离。
    const CameraCalibration uncalibrated;
    EXPECT_FALSE(estimator.estimate(det, uncalibrated, 10.0, out));

    // 检测框长边为 0 → 除零
    DetectionResult empty;
    empty.found = true;
    EXPECT_FALSE(estimator.estimate(empty, calibrationWithFx(kFx25mmPx, kFx25mmPx),
                                    10.0, out));
}

// ===========================================================================
//  二、通道选择（ENG-10 §3 / §4，SYS-14 §6/§7/§8）
// ===========================================================================

TEST(MeasurementSelector, EFollowsEng10Formula)
{
    // ENG-10 §3.2：σ_px_est = a + b·(1/S) + c·(1/C)，S = 清晰度、C = 对比度。
    // ENG-10 §3.3：N_est = M_hist × N_detect。
    // ENG-10 §3.2：E = σ_px_est·√12/(W·√N_est)   ——单位**角分**。
    MeasurementConfig cfg = configuredConfig();
    cfg.sigmaA = 0.1;
    cfg.sigmaB = 50.0;
    cfg.sigmaC = 5.0;

    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);

    MeasurementCandidate c;
    c.camera = CameraRole::CAM100;
    c.quality.sharpness = 100.0;
    c.quality.contrast = 0.5;
    c.quality.exposure = 0.8;
    c.nDetect = 200;
    c.featureSpreadPx = 400.0;
    c.scale.targetPixelSize = 400.0;
    c.scale.distance = 150.0;
    c.scale.confidence = 0.9;

    std::vector<MeasurementCandidate> candidates{c};
    ASSERT_TRUE(selector.scoreAll(candidates, "aircraft"));
    ASSERT_EQ(candidates.size(), static_cast<size_t>(1));

    const MeasurementCandidate& scored = candidates.front();

    // 手算（不引用实现的常量，逐项写出）：
    const double expectedSigma = 0.1 + 50.0 / 100.0 + 5.0 / 0.5;   // = 10.6
    const double expectedNEst = 0.5 * 200.0;                        // 冷启动先验 0.5
    const double expectedE =
        expectedSigma * std::sqrt(12.0) / (400.0 * std::sqrt(expectedNEst));

    EXPECT_NEAR(scored.sigmaPxEst, expectedSigma, 1e-9);
    EXPECT_NEAR(scored.nEst, expectedNEst, 1e-9);
    EXPECT_NEAR(scored.predictedError, expectedE, 1e-9);
    EXPECT_TRUE(scored.predictedErrorCalibrated);

    // E_ref = 1 角分（见 MeasurementSelector.cpp 的详细说明：冻结配置里
    // 没有以角分表达的误差容许字段，实现取系统自身的 Yaw 指标）。
    const double expectedENorm = 1.0 - std::min(1.0, expectedE / 1.0);
    EXPECT_NEAR(scored.eNorm, expectedENorm, 1e-9);

    // Q 的三项均值：清晰度分项取门槛值作饱和点 → 100/10 → clamp 1。
    const double expectedQ = (1.0 + 0.8 + 0.5) / 3.0;
    EXPECT_NEAR(scored.qNorm, expectedQ, 1e-9);
    EXPECT_NEAR(scored.fNorm, 1.0, 1e-9);          // min(1, 200/100)

    // ⚠ 四项**全部取正号**：归一化之后四项都是"越大越好"的优度。
    //   见 MeasurementSelector.cpp 中关于 SYS-14 §6.2 与 §8.2 冲突的说明：
    //   按字面取 "− w4·eNorm" 会让"预测误差最小"的候选得分最低。
    const double expectedScore =
        1.0 * expectedQ + 1.0 * 1.0 + 1.0 * 0.5 + 1.0 * expectedENorm;
    EXPECT_NEAR(scored.score, expectedScore, 1e-9);
}

TEST(MeasurementSelector, UncalibratedSigmasUseFallback)
{
    // ENG-10 §3.4：a,b,c 全 0 = 未标定 → 取保守值 sigmaPxFallback，
    // 且 `predictedErrorCalibrated` 必须为 false（该标记要写入 result.json，
    // 使"E 未生效"成为可见事实）。
    MeasurementConfig cfg = configuredConfig();
    cfg.sigmaPxFallback = 0.5;

    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);
    MeasurementCandidate c;
    c.quality.sharpness = 100.0;
    c.quality.contrast = 0.5;
    c.quality.exposure = 0.8;
    c.nDetect = 200;
    c.featureSpreadPx = 400.0;
    c.scale.targetPixelSize = 400.0;
    c.scale.distance = 150.0;
    c.scale.confidence = 0.9;

    std::vector<MeasurementCandidate> candidates{c};
    ASSERT_TRUE(selector.scoreAll(candidates, "aircraft"));
    EXPECT_NEAR(candidates.front().sigmaPxEst, 0.5, 1e-12);
    EXPECT_FALSE(candidates.front().predictedErrorCalibrated);
}

TEST(MeasurementSelector, GatedCandidatesAreErasedNotZeroScored)
{
    // SYS-14 §6 的三项硬门槛。**被淘汰的候选必须从候选集中删除**，
    // 而不是记 0 分：记 0 分时它仍参与 max 比较，于是当全部候选都不达标
    // 时，选择器会"成功地"选出一个不达标的通道 —— 而这恰好是
    // MEASURE_SELECT 最该失败（进而重试/切换相机）的场合。
    MeasurementConfig cfg = configuredConfig();
    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);

    MeasurementCandidate bad;
    bad.camera = CameraRole::CAM25;
    bad.quality.sharpness = 1.0;          // < minSharpness = 10
    bad.quality.exposure = 0.9;
    bad.quality.contrast = 0.9;
    bad.nDetect = 200;
    bad.featureSpreadPx = 400.0;
    bad.scale.targetPixelSize = 400.0;
    bad.scale.distance = 150.0;
    bad.scale.confidence = 0.9;

    std::vector<MeasurementCandidate> onlyBad{bad};
    EXPECT_FALSE(selector.scoreAll(onlyBad, "aircraft"));
    EXPECT_TRUE(onlyBad.empty());

    MeasurementCandidate good = bad;
    good.camera = CameraRole::CAM50;
    good.quality.sharpness = 100.0;

    std::vector<MeasurementCandidate> mixed{bad, good};
    ASSERT_TRUE(selector.scoreAll(mixed, "aircraft"));
    ASSERT_EQ(mixed.size(), static_cast<size_t>(1));
    EXPECT_EQ(mixed.front().camera, CameraRole::CAM50);
}

TEST(MeasurementSelector, MatchRatioGateDoesNotApplyWhenUnmeasured)
{
    // ⚠ 首次选择时 `ImageQuality::matchRatio` 还是默认值 0（匹配尚未发生，
    //   这正是 E 项要去掉 PnP 依赖的原因）。若无条件套用 `minMatchRatio`
    //   门槛，全部候选会被删除，现象是"永远没有可用通道"，
    //   而真实原因（门槛套错了阶段）不会出现在任何日志里。
    MeasurementConfig cfg = configuredConfig();
    cfg.minMatchRatio = 0.9;
    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);

    MeasurementCandidate c;
    c.camera = CameraRole::CAM50;
    c.quality.sharpness = 100.0;
    c.quality.contrast = 0.9;
    c.quality.exposure = 0.9;
    c.quality.matchRatio = 0.0;           // 尚未测量
    c.nDetect = 200;
    c.featureSpreadPx = 400.0;
    c.scale.targetPixelSize = 400.0;
    c.scale.distance = 150.0;
    c.scale.confidence = 0.9;

    std::vector<MeasurementCandidate> candidates{c};
    EXPECT_TRUE(selector.scoreAll(candidates, "aircraft"));
    EXPECT_EQ(candidates.size(), static_cast<size_t>(1));

    // 一旦测得了匹配率且低于门槛，则必须淘汰（重试路径上的行为）。
    MeasurementCandidate measured = c;
    measured.quality.matchRatio = 0.1;
    std::vector<MeasurementCandidate> retry{measured};
    EXPECT_FALSE(selector.scoreAll(retry, "aircraft"));
}

TEST(MeasurementSelector, DistanceBandsFollowFrozenEdges)
{
    // ENG-10 §4.2：0:<80 / 1:80~150 / 2:150~220 / 3:220~300+
    const MeasurementConfig cfg = configuredConfig();
    EXPECT_EQ(MeasurementSelector::distanceBandOf(40.0, cfg), 0);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(79.999, cfg), 0);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(80.0, cfg), 1);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(149.999, cfg), 1);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(150.0, cfg), 2);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(219.999, cfg), 2);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(220.0, cfg), 3);
    EXPECT_EQ(MeasurementSelector::distanceBandOf(300.0, cfg), 3);
}

TEST(MeasurementSelector, UnconfiguredIllumBandsFallBackToNormal)
{
    // ⚠ `illumBandEdges` 的默认值是 {0,0}（ENG-09 §6.5 的未标定哨兵）。
    //   照字面比较会把**所有**曝光值都判进 band 2（强光逆光），
    //   于是全部历史统计被灌进同一个偏斜的桶。
    MeasurementConfig cfg = configuredConfig();   // illumBandEdges 保持 {0,0}
    EXPECT_EQ(MeasurementSelector::illumBandOf(0.0, cfg), 1);
    EXPECT_EQ(MeasurementSelector::illumBandOf(0.5, cfg), 1);
    EXPECT_EQ(MeasurementSelector::illumBandOf(1.0, cfg), 1);

    cfg.illumBandEdges[0] = 0.3;
    cfg.illumBandEdges[1] = 0.7;
    EXPECT_EQ(MeasurementSelector::illumBandOf(0.1, cfg), 0);
    EXPECT_EQ(MeasurementSelector::illumBandOf(0.5, cfg), 1);
    EXPECT_EQ(MeasurementSelector::illumBandOf(0.9, cfg), 2);
}

TEST(MeasurementSelector, UnknownDistanceSkipsHistoryLookup)
{
    // 距离未知（尺度估计失败）时不得按 distance = 0 分桶：
    // "不知道距离"与"距离 0 m"是两件事，后者会查出最近距离带的历史。
    MeasurementConfig cfg = configuredConfig();
    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);

    MeasurementCandidate c;
    c.camera = CameraRole::CAM50;
    c.quality.sharpness = 100.0;
    c.quality.contrast = 0.9;
    c.quality.exposure = 0.9;
    c.nDetect = 200;
    c.featureSpreadPx = 400.0;
    c.scale.targetPixelSize = 400.0;
    // scale.distance / confidence 保持默认 0 = 未知

    std::vector<MeasurementCandidate> candidates{c};
    ASSERT_TRUE(selector.scoreAll(candidates, "aircraft"));
    ASSERT_EQ(selector.lastUsage().size(), static_cast<size_t>(1));
    EXPECT_EQ(selector.lastUsage().front().distanceBand, -1);
    EXPECT_FALSE(selector.lastUsage().front().fromStore);
    EXPECT_TRUE(selector.lastSelectionColdStart());
    EXPECT_NEAR(candidates.front().mNorm, cfg.matchStatsColdStartPrior, 1e-12);
}

namespace
{

/// 固定返回值的统计表桩，用于把 `M_hist` 与"查表命中"这两件事
/// 从冷启动先验里分离出来。
class FixedStatsStore : public aircraft::data::IMatchStatsStore
{
public:
    explicit FixedStatsStore(double v) : value_(v) {}

    double successRate(const std::string&, int, int, int) const override
    {
        ++queries;
        return value_;
    }

    void record(const std::string&, int, int, int, bool) override {}
    bool save() override { return true; }
    /// 本桩不接持久化介质；返回 true 而不是 false —— 后者在本工程的
    /// 约定里表示"表已损坏、已改名保留现场"，会被误读成一次异常。
    bool load() override { return true; }

    mutable int queries = 0;

private:
    double value_;
};

}  // namespace

TEST(MeasurementSelector, UsageTripleRecordsTheCamera)
{
    // ENG-10 §4.4：result.json 要记录每个候选使用的 M_hist 三元组
    // （机型 / 相机 / 距离带 / 光照带）。本用例守的是其中**唯一一处
    // 已有的用例都没覆盖的属性**：`camera`。
    //
    // ⚠ 候选取 **CAM100** 而不是 CAM25：`MatchStatsUsage::camera` 的默认值
    //   正是 CAM25，"忘了填 camera"这一缺陷在默认值上会**恰好通过**。
    //   取一个与默认值不同的角色，断言才有判别力。
    //
    // ⚠ 与 `UnknownDistanceSkipsHistoryLookup` 的分工：那一例查的是
    //   "距离未知时**不查表**"（负向），本用例查的是"距离已知时查到的
    //   三元组**如实写进追溯记录**"（正向）。两例的 `fromStore` 期望相反，
    //   故不能合并 —— 合并后必定有一半在错误的路径上断言。
    MeasurementConfig cfg = configuredConfig();
    cfg.illumBandEdges[0] = 0.3;
    cfg.illumBandEdges[1] = 0.7;

    FixedStatsStore store(0.75);   // 与冷启动先验（0.5）不同，可区分两条来源
    const MeasurementSelector selector(cfg, configuredValidation(), &store);

    MeasurementCandidate c;
    c.camera = CameraRole::CAM100;
    c.quality.sharpness = 100.0;
    c.quality.contrast = 0.9;
    c.quality.exposure = 0.9;              // → 光照带 2（强光/逆光侧）
    c.nDetect = 200;
    c.featureSpreadPx = 400.0;
    c.scale.targetPixelSize = 400.0;
    c.scale.distance = 200.0;              // → 距离带 2（150~220 m）
    c.scale.confidence = 0.9;

    std::vector<MeasurementCandidate> candidates{c};
    ASSERT_TRUE(selector.scoreAll(candidates, "aircraft"));
    ASSERT_EQ(store.queries, 1) << "距离已知时应查表一次";
    ASSERT_EQ(selector.lastUsage().size(), static_cast<size_t>(1));

    // ⚠ **按值**取一份，不能写成 `const MatchStatsUsage& u = ...front()`：
    //   `lastUsage()` 返回的是 vector **按值**（临时量），而 `front()` 是
    //   函数调用返回的引用 —— 生命周期延长在这里**不适用**，临时 vector
    //   在全表达式结束时即析构，`u` 随即悬空。现象是随机的：读到的字段
    //   有的还是原值、有的已被重写（本用例第一版就是这样，camera/两个带
    //   读到垃圾，而 mHist/fromStore 恰好还是原值）。
    const MeasurementSelector::MatchStatsUsage u = selector.lastUsage().front();
    EXPECT_EQ(u.camera, CameraRole::CAM100)
        << "三元组里的相机角色被写成了别的通道（或没写，留在默认的 CAM25）";
    EXPECT_EQ(u.distanceBand, 2);
    EXPECT_EQ(u.illumBand, 2);
    EXPECT_TRUE(u.fromStore) << "注入了统计表却被记为冷启动";
    EXPECT_FALSE(selector.lastSelectionColdStart());
    EXPECT_DOUBLE_EQ(u.mHist, 0.75) << "追溯值必须是**实际使用**的 M_hist";
    EXPECT_NEAR(candidates.front().mNorm, 0.75, 1e-12);
}

TEST(MeasurementSelector, SelectTakesMaximumAndKeepsPriorityOnTies)
{
    const MeasurementConfig cfg = configuredConfig();
    const MeasurementSelector selector(cfg, configuredValidation(), nullptr);

    auto make = [](CameraRole role, double spread) {
        MeasurementCandidate c;
        c.camera = role;
        c.quality.sharpness = 100.0;
        c.quality.contrast = 0.9;
        c.quality.exposure = 0.9;
        c.nDetect = 200;
        c.featureSpreadPx = spread;       // 展布越大 → E 越小 → 得分越高
        c.scale.targetPixelSize = spread;
        c.scale.distance = 150.0;
        c.scale.confidence = 0.9;
        return c;
    };

    std::vector<MeasurementCandidate> candidates{
        make(CameraRole::CAM25, 100.0),
        make(CameraRole::CAM100, 800.0),
        make(CameraRole::CAM50, 400.0),
    };
    ASSERT_TRUE(selector.scoreAll(candidates, "aircraft"));

    MeasurementSelectionResult out;
    ASSERT_TRUE(selector.select(candidates, out));
    EXPECT_EQ(out.selectedCamera, CameraRole::CAM100);
    EXPECT_NEAR(out.score, candidates[1].score, 1e-12);

    // 并列 → 保留先出现者（候选顺序即优先级顺序）。
    // SYS-04 §6.4 要求结果可离线复现，"并列时选哪个"必须有确定规则。
    std::vector<MeasurementCandidate> tied{
        make(CameraRole::CAM50, 400.0),
        make(CameraRole::CAM100, 400.0),
    };
    ASSERT_TRUE(selector.scoreAll(tied, "aircraft"));
    MeasurementSelectionResult tiedOut;
    ASSERT_TRUE(selector.select(tied, tiedOut));
    EXPECT_EQ(tiedOut.selectedCamera, CameraRole::CAM50);
}

// ===========================================================================
//  三、姿态验证（SYS-07 §12.2）
// ===========================================================================

namespace
{

ShipPoseResult validPose()
{
    ShipPoseResult r;
    r.success = true;
    r.yaw = 5.0;
    r.pitch = 1.0;
    r.roll = -0.5;
    r.reprojectionError = 0.5;
    return r;
}

}  // namespace

TEST(PoseValidator, PassesWhenAllCriteriaHold)
{
    const PoseValidator validator(configuredValidation());
    PoseValidationResult out;
    EXPECT_TRUE(validator.validate(validPose(), 0.9, 100, out));
    EXPECT_TRUE(out.valid);
    EXPECT_NEAR(out.inlierRatio, 0.9, 1e-12);
    EXPECT_NEAR(out.confidence,
                std::min(0.9, 1.0 - 0.5 / 2.0), 1e-12);
}

TEST(PoseValidator, FailsOnEachCriterionIndependently)
{
    const PoseValidator validator(configuredValidation());
    PoseValidationResult out;

    // ① 解算未成功
    ShipPoseResult failed = validPose();
    failed.success = false;
    EXPECT_FALSE(validator.validate(failed, 0.9, 100, out));

    // ② 重投影误差超限
    ShipPoseResult blurry = validPose();
    blurry.reprojectionError = 2.5;      // > maxReprojectionError = 2.0
    EXPECT_FALSE(validator.validate(blurry, 0.9, 100, out));
    EXPECT_NEAR(out.reprojectionError, 2.5, 1e-12);

    // ②' 重投影误差为 NaN：必须判**不合格**。
    //     若实现写成 `err > maxErr → 失败`，NaN 与任何值比较均为 false，
    //     于是 NaN 会被判为通过 —— 一个最该被拦住的输入反而被放行。
    ShipPoseResult nan = validPose();
    nan.reprojectionError = std::nan("");
    EXPECT_FALSE(validator.validate(nan, 0.9, 100, out));
    //     原因分类（裁决 C-008）：这不是"阈值没过"，是"值本身不是数"。
    EXPECT_EQ(out.reason, PoseValidationReason::NON_FINITE_VALUE);

    // ③ 内点比例不足
    EXPECT_FALSE(validator.validate(validPose(), 0.3, 100, out));

    // ③' 阈值未过时 reason **不得**是 NON_FINITE_VALUE。
    //     少了这一条，"永远填 NON_FINITE_VALUE"也能让上面的断言全绿 ——
    //     reason 会退化成一个常数，失去分类能力（与 C-006 的 9004 同理）。
    EXPECT_EQ(out.reason, PoseValidationReason::OK);

    // ④ 置信度不足（内点比例刚好压到与 minConfidence 相交处）
    EXPECT_FALSE(validator.validate(validPose(), 0.4, 100, out));
    EXPECT_EQ(out.reason, PoseValidationReason::OK);
}

TEST(PoseValidator, RejectsNonFinitePoseValues)
{
    // 裁决 C-008。判据是"非有限值必须在**阈值判据之前**被拦下"，
    // 而不是"恰好也没通过"—— 后者在下面每一个输入上都**不成立**：
    //
    //   · yaw   ：⑤ 的区间判据只在配置了 yawMin < yawMax 时生效，
    //             且 `NaN < min || NaN > max` 恒为 false → NaN **通过**；
    //   · pitch / roll：整个 validate() 里**没有任何判据看它们**；
    //   · aircraftToShip：同上。
    //
    // ⚠ 因此本用例的每个输入都必须是"若没有闸门就会一路通过"的那一个：
    //   只要某一项本来就会被别的判据拦住，它就证明不了闸门存在。
    //   `validPose()` + `configuredValidation()` 是**全判据通过**的基准，
    //   故只需改一个字段为 NaN，就能得到"除了闸门无人在看"的输入。
    const PoseValidator validator(configuredValidation());
    PoseValidationResult out;

    // 基准：同一份姿态在有限值下必须通过。否则下面的断言可能只是因为
    // 基准本身就不合格（那样每个输入都会"不合格"，用例毫无鉴别力）。
    ASSERT_TRUE(validator.validate(validPose(), 0.9, 100, out));
    EXPECT_EQ(out.reason, PoseValidationReason::OK)
        << "通过时 reason 必须是 OK，不得是'未识别的原因'";

    struct Case
    {
        const char* name;
        double      ShipPoseResult::*scalar;   // yaw / pitch / roll
    };
    const Case scalars[] = {
        {"yaw",   &ShipPoseResult::yaw},
        {"pitch", &ShipPoseResult::pitch},
        {"roll",  &ShipPoseResult::roll},
    };
    for (const Case& c : scalars)
    {
        ShipPoseResult pose = validPose();
        pose.*(c.scalar) = std::nan("");
        EXPECT_FALSE(validator.validate(pose, 0.9, 100, out))
            << c.name << " = NaN 被判为合格 —— 该姿态会一路走到 SAVE 并落盘";
        EXPECT_EQ(out.reason, PoseValidationReason::NON_FINITE_VALUE) << c.name;

        // +inf 与 -inf 同样是"不是数"，且它们能通过区间判据
        //（`inf > yawMax` 为 true 会被 ⑤ 拦住，但 `yawMin` 未配置时不会）。
        pose.*(c.scalar) = std::numeric_limits<double>::infinity();
        EXPECT_FALSE(validator.validate(pose, 0.9, 100, out)) << c.name << " = inf";
        EXPECT_EQ(out.reason, PoseValidationReason::NON_FINITE_VALUE) << c.name;
    }

    // 变换矩阵：9 个旋转元素 + 3 个平移元素，逐个置 NaN。
    // ⚠ 逐个而不是只测一个：闸门若写成"只查 rotation(0,0)"，
    //   只测一个的用例照样全绿。
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            ShipPoseResult pose = validPose();
            pose.aircraftToShip.rotation(r, c) = std::nan("");
            EXPECT_FALSE(validator.validate(pose, 0.9, 100, out))
                << "rotation(" << r << "," << c << ") = NaN 被放行";
            EXPECT_EQ(out.reason, PoseValidationReason::NON_FINITE_VALUE);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        ShipPoseResult pose = validPose();
        pose.aircraftToShip.translation[i] = std::nan("");
        EXPECT_FALSE(validator.validate(pose, 0.9, 100, out))
            << "translation[" << i << "] = NaN 被放行";
        EXPECT_EQ(out.reason, PoseValidationReason::NON_FINITE_VALUE);
    }

    // ⚠ 顺序判据：`success == false` 时其余字段按 ENG-09 §5.24 **无意义**
    //   （"false 时其余字段无意义"），此时不得报 NON_FINITE_VALUE ——
    //   那会把"算法没解出来"这个真实原因，覆盖成一个由垃圾值引发的误报，
    //   而两者的处置方向完全不同。
    ShipPoseResult unsolved = validPose();
    unsolved.success = false;
    unsolved.yaw = std::nan("");
    EXPECT_FALSE(validator.validate(unsolved, 0.9, 100, out));
    EXPECT_EQ(out.reason, PoseValidationReason::OK)
        << "解算失败时字段本无意义，不应据其报 NON_FINITE_VALUE";
}

TEST(PoseValidator, YawRangeCheckUsesNormalizedAngle)
{
    const PoseValidator validator(configuredValidation());   // ±30°
    PoseValidationResult out;

    // 359.5° 与 -0.5° 是同一个姿态。字面比较会把前者判为超出范围，
    // 使 VALIDATE 无谓重试直到 9001。
    ShipPoseResult wrapped = validPose();
    wrapped.yaw = 359.5;
    EXPECT_TRUE(validator.validate(wrapped, 0.9, 100, out));

    // -179.5° 归一化后仍超出 ±30° → 不合格
    ShipPoseResult outside = validPose();
    outside.yaw = -179.5;
    EXPECT_FALSE(validator.validate(outside, 0.9, 100, out));
}

TEST(PoseValidator, UnconfiguredYawRangeDisablesThatCriterion)
{
    // ValidationConfig 的默认 yawMin = yawMax = 0 是**未配置哨兵**。
    // 照字面执行"yaw ∈ [0,0]"会把任何非零 Yaw 都判不合格，
    // 而失败信息只表现为 VALIDATE 反复重试，与真实原因无关。
    ValidationConfig v;
    v.maxReprojectionError = 2.0;
    v.minInlierRatio = 0.0;
    v.minConfidence = 0.0;
    const PoseValidator validator(v);

    PoseValidationResult out;
    ShipPoseResult pose = validPose();
    pose.yaw = 17.0;
    EXPECT_TRUE(validator.validate(pose, 0.9, 100, out));
}

TEST(PoseValidator, ConfidenceIsWeakestLink)
{
    const PoseValidator validator(configuredValidation());

    // 重投影刚好在门槛上 → 该项 0 分；内点比例 1.0 → 置信度必须是 0，
    // 而不是被平均成一个好看的值。
    EXPECT_NEAR(validator.confidence(2.0, 1.0), 0.0, 1e-12);
    EXPECT_NEAR(validator.confidence(0.0, 1.0), 1.0, 1e-12);
    EXPECT_NEAR(validator.confidence(1.0, 0.8), 0.5, 1e-12);
}

// ===========================================================================
//  四、PnP 解算（SYS-07 §10 / §12.2）
// ===========================================================================


TEST(PnPPoseEstimator, RecoversKnownPoseFromNoiseFreeCorrespondences)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 1280, 1024);

    const std::vector<cv::Point3f> points = aircraftShapePoints();

    // 真值：绕 Z 转 30°、绕 Y 转 5°，目标在 (1.0, -0.5, 120.0) m。
    const cv::Vec3d truthRvec(0.05, 0.05, 30.0 * 3.14159265358979323846 / 180.0);
    const cv::Vec3d truthTvec(1.0, -0.5, 120.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRvec, rvecTrue);

    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), calib);

    MeasurementConfig cfg;
    cfg.minFeatureCount = 8;
    CvPnPPoseEstimator estimator(cfg);

    CameraPose pose;
    PnpStats stats;
    ASSERT_TRUE(estimator.estimate(correspondences, calib, pose, stats));

    EXPECT_TRUE(stats.converged);
    EXPECT_EQ(stats.inputCount, static_cast<int>(points.size()));
    EXPECT_NEAR(stats.inlierRatio, 1.0, 1e-9);

    // 无噪声对应的重投影误差**不是**机器精度量级，这与本实现无关：
    // 残差的下限由 cv::solvePnP 迭代求解自身的终止条件设定（默认
    // Criteria 的 eps 量级），实测本用例为 1.76e-5 pixel。
    // 该量级换算成角度约 8.8e-9 rad ≈ 3e-5 角分（fx = 2000），比本工程的
    // Yaw 指标（1 角分）低五个数量级，也远低于 E 项所用的保守单点
    // σ_px = 0.5 pixel —— 即**求解器不是限制项**，这正是此处要断言的。
    // 若改成要求 < 1e-6，测的将是 OpenCV 的内部容差（会随 OpenCV 版本变动），
    // 而不是本工程的正确性。
    EXPECT_LT(pose.reprojectionError, 1e-3);   // 比 σ_px 保守值低三个数量级

    // 旋转一致性误差用 atan 型量度，容差取 1e-5 rad ≈ 0.03 角分。
    // （不取更小的原因：由 trace 反算角度的式子在大角度分辨率上受
    //  双精度限制，1e-16 的迹误差会被放大成约 1e-8 rad 的角误差。）
    cv::Matx33d truthRotation;
    cv::Rodrigues(truthRvec, truthRotation);
    EXPECT_LT(rotationAngleBetween(pose.aircraftToCamera.rotation, truthRotation),
              1e-5);
    EXPECT_NEAR(pose.aircraftToCamera.translation(0), truthTvec(0), 1e-4);
    EXPECT_NEAR(pose.aircraftToCamera.translation(1), truthTvec(1), 1e-4);
    EXPECT_NEAR(pose.aircraftToCamera.translation(2), truthTvec(2), 1e-4);

    // 方向核对（裁决 C-06）：PnP 的输出**天然**是 aircraftToCamera。
    //   p_camera = R·p_aircraft + t
    // 取反（转置）不会有任何报错，只会让 Yaw 的符号与量值同时错。
    const cv::Vec3d check = pose.aircraftToCamera.rotation
                              * cv::Vec3d(points[0].x, points[0].y, points[0].z)
                          + pose.aircraftToCamera.translation;
    const cv::Vec3d expected =
        truthRotation * cv::Vec3d(points[0].x, points[0].y, points[0].z)
        + truthTvec;
    EXPECT_NEAR(check(0), expected(0), 1e-4);
    EXPECT_NEAR(check(1), expected(1), 1e-4);
    EXPECT_NEAR(check(2), expected(2), 1e-4);
}

TEST(PnPPoseEstimator, RejectsOutliersThroughRansac)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 1280, 1024);
    const std::vector<cv::Point3f> points = aircraftShapePoints();

    const cv::Vec3d truthRvec(0.0, 0.0, 0.1);
    const cv::Vec3d truthTvec(0.0, 0.0, 100.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRvec, rvecTrue);

    std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), calib);

    // 6 个外点：图像坐标被移到完全不相干的位置（负坐标 = 画面外）。
    // 对应关系是**结构性**错误的（三维点与图像点不匹配），
    // 这正是 RANSAC 要剔除的对象。
    for (int i = 0; i < 6; ++i)
    {
        FeatureCorrespondence bad;
        bad.objectPoint = points[static_cast<size_t>(i)];
        bad.imagePoint = cv::Point2f(-500.0f - 37.0f * i, -400.0f - 23.0f * i);
        correspondences.push_back(bad);
    }

    MeasurementConfig cfg;
    cfg.minFeatureCount = 8;
    CvPnPPoseEstimator estimator(cfg);

    CameraPose pose;
    PnpStats stats;
    ASSERT_TRUE(estimator.estimate(correspondences, calib, pose, stats));

    EXPECT_EQ(stats.inputCount, 18);
    EXPECT_GE(stats.inlierCount, 12);
    EXPECT_LE(stats.inlierCount, 12);       // 恰好 12 个真点
    EXPECT_NEAR(stats.inlierRatio, 12.0 / 18.0, 1e-9);

    cv::Matx33d truthRotation;
    cv::Rodrigues(truthRvec, truthRotation);
    EXPECT_LT(rotationAngleBetween(pose.aircraftToCamera.rotation, truthRotation),
              1e-4);
    EXPECT_NEAR(pose.aircraftToCamera.translation(2), 100.0, 1e-3);
}

TEST(PnPPoseEstimator, RefusesWhenTooFewCorrespondences)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 1280, 1024);

    MeasurementConfig cfg;
    cfg.minFeatureCount = 50;   // 门槛高于实际点数

    CvPnPPoseEstimator estimator(cfg);

    // 3 个点：低于 PnP 的几何最小点数 4。
    std::vector<FeatureCorrespondence> three;
    for (int i = 0; i < 3; ++i)
    {
        FeatureCorrespondence c;
        c.objectPoint = cv::Point3f(static_cast<float>(i), 0.0f, 0.0f);
        c.imagePoint = cv::Point2f(static_cast<float>(i), 0.0f);
        three.push_back(c);
    }

    CameraPose pose;
    PnpStats stats;
    // 必须返回 false 而不是"成功给出一个欠定方程组的解"：
    // 后者看起来与真解同型，会把错误一路推到最终 Yaw 输出。
    EXPECT_FALSE(estimator.estimate(three, calib, pose, stats));
    EXPECT_FALSE(stats.converged);
    EXPECT_EQ(stats.inlierCount, 0);
    // 失败时输出必须是**干净**的默认值（单位旋转 + 零平移），
    // 而不是被上一次或半途的计算结果污染 —— 后者会被上层当成
    // "解算成功但精度差"，从而走上一条错误的恢复路径。
    EXPECT_NEAR(pose.aircraftToCamera.translation(2), 0.0, 1e-12);
    EXPECT_NEAR(pose.reprojectionError, 0.0, 1e-12);
    EXPECT_NEAR(pose.aircraftToCamera.rotation(0, 1), 0.0, 1e-12);
}

TEST(PnPPoseEstimator, RejectsUncalibratedIntrinsics)
{
    const CameraCalibration uncalibrated;   // fx = fy = 1（ENG-09 §5.16 默认值）
    const std::vector<cv::Point3f> points = aircraftShapePoints();

    const cv::Vec3d truth(0.0, 0.0, 0.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truth, rvecTrue);

    // 用"未标定"的内参投影，再用同一内参解算 —— 数学上自洽，
    // 但物理上毫无意义（fx = 1 等价于视场接近 180°）。
    // 本实现选择拒绝：合法的做法是让启动流程在标定缺失时就失败，
    // 而不是让算法给出一个"看起来成功"的结果。
    const CameraCalibration fake = calibrationWithFx(1.0, 1.0, 1280, 1024);
    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(cv::Vec3d(0, 0, 100)), fake);

    MeasurementConfig cfg;
    cfg.minFeatureCount = 8;
    CvPnPPoseEstimator estimator(cfg);

    CameraPose pose;
    PnpStats stats;
    EXPECT_TRUE(estimator.estimate(correspondences, fake, pose, stats));
    EXPECT_FALSE(estimator.estimate(correspondences, uncalibrated, pose, stats));
}
