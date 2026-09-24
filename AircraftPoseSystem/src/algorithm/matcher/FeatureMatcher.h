#pragma once

// ============================================================================
//  src/algorithm/matcher/FeatureMatcher.h
//
//  依据：SYS-07 §9（特征匹配流程 Descriptor Matching → Ratio Test → RANSAC →
//                    FeatureCorrespondence）、
//        ENG-01 §10（matcher 子模块，**融合要求被冻结**：输入为 A 类与 B 类
//                    两路特征，须实现 ENG-10 §2.5 的冲突规则）、
//        ENG-10 §2.5（A/B 两类的融合规则与冲突规则）、
//        SYS-12 §10（图像匹配流程）、§11（CAD 与自然纹理融合策略）、
//        ENG-02 §11.6、ENG-09 §5.19 / §5.21 / §5.22
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与 8.md §十 的偏离：                                                  │
//  │  1. 8.md 的 `struct MatchResult { int matchCount = 0; };` 只有一个   │
//  │     计数。而 ENG-10 §2.5 的冲突规则、§2.4 的可用性判据、以及          │
//  │     `ImageQuality::matchRatio` 的计算都需要"总候选数 / 内点数 /      │
//  │     两类各多少 / 被冲突规则剔除多少"。只留一个计数，等于把           │
//  │     "为什么内点少"这个唯一可诊断的信息丢掉。                          │
//  │  2. 8.md 的接口 `match(const FeatureSet& image, const FeatureSet&     │
//  │     model)` 把模型也当成一个 FeatureSet。但 B 类（CAD 结构点）**没有  │
//  │     描述子**（ENG-10 §2.6：B 类的离线产物是坐标表，无需描述子），     │
//  │     它根本不是一个 FeatureSet。故模型侧入参是 `data::TargetModel`     │
//  │     （含 points3d + features），B 类对应由 CadStructureLocator 单独   │
//  │     给出 —— 这正是 ENG-01 §10 要求的"两路输入"。                      │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 本类**不做 RANSAC**，尽管 SYS-07 §9 的流程图上 RANSAC 画在匹配这一步。
//    原因是几何模型就是 PnP 本身：RANSAC 需要的"拟合 + 残差"只有算出位姿
//    之后才有意义。把它放在这里就等于在这里跑一次 PnP，而 SYS-14 §17 明令
//    "不同时运行三个完整 PnP"、§6.2 的选择必须在 PnP **之前**完成。
//    故实际分工是：
//      · 本类：描述子匹配 + Ratio Test + **冲突规则** + 候选集融合；
//      · PnPPoseEstimator：RANSAC（`cv::solvePnPRansac`）→ 内点集合。
//    内点比例（SYS-07 §12.2 的第二项验证指标）由 PnP 阶段给出。
// ============================================================================

#include <vector>

#include "data/CameraRole.h"
#include "data/FeatureCorrespondence.h"
#include "data/FeatureSet.h"
#include "data/MeasurementConfig.h"
#include "data/TargetModel.h"

namespace aircraft
{
namespace algorithm
{

/// 一次匹配的统计量。它同时是 `ImageQuality::matchCount/matchRatio` 与
/// `result.json` 中"特征来源构成"的原始出处（ENG-10 §2.4 / §7 约束 3）。
struct MatchResult
{
    /// 进入融合与冲突规则的候选对应总数（A 类 + B 类）。
    int totalCandidates = 0;

    /// 融合后保留的对应数（尚未经 RANSAC，故**不是**内点数）。
    int matchCount = 0;

    /// 被冲突规则剔除的 A 类点数（ENG-10 §2.5：同区域冲突时以 B 类为准）。
    int droppedByConflict = 0;

    /// 保留对应中来自 B 类（CAD 结构点）的数量。
    /// 必须记录：ENG-10 §2.4 的 `cad_assisted` 判据（≥4 才算 B 类可用）
    /// 依赖它，且它是 SYS-15 §4.5 中"展布由谁保证"的直接证据。
    int cadCount = 0;

    /// 保留对应中来自 A 类（自然纹理）的数量。
    int textureCount = 0;

    /// 保留对应集的横向展布宽度 W，单位 pixel。
    double spreadPx = 0.0;
};

/// 特征匹配抽象（SYS-07 §9）。实现方向：本类 → 描述子匹配 + 冲突规则。
class FeatureMatcher
{
public:
    virtual ~FeatureMatcher() = default;

    /// @param imageFeatures 被选通道图像的 A 类特征（SIFT）。
    /// @param model         目标模型（提供 A 类描述子与三维点表）。
    /// @param cadCorrespondences B 类对应（`CadStructureLocator` 的输出；
    ///        为空表示 B 类不可用，此时退化为纯 A 类 —— ENG-10 §2.4）。
    /// @param out           融合后的 2D-3D 对应集合。
    /// @param stats         统计量（用于 `ImageQuality` 与 result.json）。
    /// @return 是否存在足够的对应（数量由注入的 `minFeatureCount` 门槛决定）。
    virtual bool match(
        const data::FeatureSet& imageFeatures,
        const data::TargetModel& model,
        const std::vector<data::FeatureCorrespondence>& cadCorrespondences,
        std::vector<data::FeatureCorrespondence>& out,
        MatchResult& stats) = 0;
};

/// 真实匹配器：BFMatcher + Lowe Ratio Test + ENG-10 §2.5 的冲突规则。
///
/// 配置注入（ENG-10 §5.1 冻结的注入行）：
///   `MeasurementConfig → FeatureMatcher（minFeatureCount / minMatchRatio）`
/// 二者分别作为"对应数门槛"与"匹配比例门槛"。
class DescriptorFeatureMatcher : public FeatureMatcher
{
public:
    /// @param config `MeasurementConfig`（注入 `const&`，注入后不可变）。
    explicit DescriptorFeatureMatcher(const data::MeasurementConfig& config);

    bool match(const data::FeatureSet& imageFeatures,
               const data::TargetModel& model,
               const std::vector<data::FeatureCorrespondence>& cadCorrespondences,
               std::vector<data::FeatureCorrespondence>& out,
               MatchResult& stats) override;

    /// Lowe Ratio Test 的比值阈值，默认 0.75（Lowe 原文的建议值）。
    ///
    /// ⚠ 该阈值**不在任何冻结文档中**（`MeasurementConfig::minMatchRatio`
    /// 是**内点比例**门槛，不是本阈值，两者名字相近但含义完全不同，混用会
    /// 让匹配要么几乎全被拒、要么外点泛滥）。故此处给默认值并可显式设定，
    /// 已记入 README §6 的待裁决清单。
    void setRatioThreshold(double ratio);

private:
    /// ⚠ 按值持有（入参仍是 `const&`）：存引用会让传临时量的构造写法通过编译，
    /// 而门槛值在构造语句结束即悬空 —— 表现为匹配数量随机变化。理由详见
    /// MeasurementSelector.h 中同一处说明（ENG-10 §5.2 约束 2）。
    data::MeasurementConfig config_;
    double ratioThreshold_ = 0.75;
};

/// 固定匹配结果（8.md §十 的 `MockFeatureMatcher`）。
///
/// 用于 PnP / 验证 / 状态机等下游逻辑的单测：给出确定的对应集合与统计量，
/// 使"内点不足时应失败"之类的断言不依赖图像内容。
class MockFeatureMatcher : public FeatureMatcher
{
public:
    MockFeatureMatcher() = default;

    /// 预设匹配结果。
    /// @param out 固定输出。`stats.matchCount` 等由本方法按 `out` 重算，
    ///        使夹具自身不可能给出"对应数与统计量不一致"的组合 ——
    ///        那种不一致会让下游断言失败在错误的地方。
    void setPreset(const std::vector<data::FeatureCorrespondence>& preset,
                   int cadCount = 0,
                   double spreadPx = 0.0);

    /// 强制 match() 返回 false（模拟"匹配失败"这一路径）。
    void forceFailure(bool fail) { forceFailure_ = fail; }

    bool match(const data::FeatureSet& imageFeatures,
               const data::TargetModel& model,
               const std::vector<data::FeatureCorrespondence>& cadCorrespondences,
               std::vector<data::FeatureCorrespondence>& out,
               MatchResult& stats) override;

private:
    std::vector<data::FeatureCorrespondence> preset_;
    int cadCount_ = 0;
    double spreadPx_ = 0.0;
    bool forceFailure_ = false;
};

}  // namespace algorithm
}  // namespace aircraft
