#pragma once

// ============================================================================
//  src/algorithm/selection/MeasurementSelector.h
//
//  依据：SYS-14 §6（候选评价）/§7（评价指标）/§8（综合评分模型）、
//        SYS-07 §6（MeasurementSelector）、
//        ENG-01 §10（selection 子模块，**文件名被冻结为 MeasurementSelector.h**）、
//        ENG-02 §11.3、ENG-10 §3（G-2：E 项的数据来源）/§3.6（四分量归一化）/
//        §4（G-3：M_hist 的存储与生命周期）/§5.1（注入矩阵）
//        ENG-09 §5.16（MeasurementCandidate 字段）、§6.5（MeasurementConfig）、
//        §6.6（ValidationConfig）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类是"三焦段选一"的唯一决策点，也是全工程**最容易被静默做错**的一处：│
//  │                                                                      │
//  │  · SYS-07 §6.2 的评分式 `− w4·ReprojectionError` **已作废**（裁决 G-2）：│
//  │    ReprojectionError 是 PnP 的输出，而本评分发生在 PnP **之前**，      │
//  │    按原式实现就必须对三台相机各跑一次 PnP —— 那是循环依赖，           │
//  │    且违反 SYS-14 §17 的 <100 ms 预算。冻结为 ENG-10 §3.2 的 E。        │
//  │  · E 的计算路径上**不得出现任何 PnP 输出**（ENG-10 §7 约束 4）。       │
//  │    这是判断本实现是否正确的最简标准。                                  │
//  │  · 四个分项**必须先归一化到 [0,1] 再加权**（ENG-10 §3.6）。不归一化时  │
//  │    量级最大的分项实际决定结果，权重失去意义。                          │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 与 8.md §七 的偏离：8.md 的接口是
//        `data::CameraRole select(const std::vector<MeasurementCandidate>&)`
//    返回 CameraRole，并以 `CameraRole::UNKNOWN` 表示"无候选"。
//    ENG-09 §4.1 **只冻结了三个角色**（CAM25/CAM50/CAM100），没有 UNKNOWN，
//    理由见 CameraRole.h：哨兵值会经由默认构造悄悄出现在所有含该枚举的结构体
//    里，使"这台相机不存在"与"这台相机是 25mm"在内存中不可区分。
//    故改为 `bool select(..., MeasurementSelectionResult& out)` ——
//    用返回值表达失败，编译器强制调用方处理该分支。
//    该偏离已由 002 阶段的 data/MeasurementSelectionResult.h 记录在案。
//
//  ⚠ 8.md 的"取 score 最大者"被保留，但**打分之前先过 SYS-14 §6 的硬门槛**
//    （gating）。8.md 的实现直接从 candidates.front() 开始比较，
//    等于让完全过曝的图像凭 sharpness 参与竞争 —— 门槛正是为此存在。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <string>
#include <vector>

#include "data/CameraRole.h"
#include "data/IMatchStatsStore.h"
#include "data/MeasurementCandidate.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementSelectionResult.h"
#include "data/ValidationConfig.h"

namespace aircraft
{
namespace algorithm
{

/// 测量通道选择（SYS-14 §6 / SYS-07 §6）。
///
/// 配置注入（ENG-10 §5.1 冻结）：`MeasurementConfig` 与 `ValidationConfig`
/// 均**构造注入 const&**，注入后不可变（§5.2 约束 2：禁止运行时改配置）。
/// 本类**不访问 ConfigManager**（§5.2 约束 1）。
class MeasurementSelector
{
public:
    /// 一次选择中 M_hist 的实际用法，供 result.json 追溯。
    ///
    /// ENG-10 §4.4 要求"记录本次选择实际使用的 `M_hist` 三元组与表版本"。
    /// ⚠ 表版本 (`match_stats_version`) **无法**记录：冻结的
    /// `IMatchStatsStore`（data 层）只有 successRate/record/save/load 四个
    /// 方法，没有版本查询。该缺口已记入 README §6 的待裁决清单。
    struct MatchStatsUsage
    {
        data::CameraRole camera = data::CameraRole::CAM25;
        int    distanceBand = 0;      ///< [0,3]，边界见 distanceBandEdges
        int    illumBand = 0;         ///< [0,2]，边界见 illumBandEdges
        double mHist = 0.0;           ///< 本次实际使用的 M_hist ∈ [0,1]
        bool   fromStore = false;     ///< false = 未注入统计表，用的是先验
    };

    /// @param measurement `MeasurementConfig`（w1~w4、门槛、分桶边界、sigma 系数）。
    /// @param validation  `ValidationConfig`（E 归一化的参考值 E_ref，见 §3.6）。
    /// @param store       历史匹配成功率表；**可为 nullptr**（ENG-10 §4.3 的
    ///                    冷启动情形：第 1 天表是空的）。nullptr 时 M 取
    ///                    `matchStatsColdStartPrior`（默认 0.5），
    ///                    使 M 分项在冷启动时对所有候选等价，
    ///                    于是选择由 Q/F/E 决定 —— 这正是正确的：
    ///                    既然不知道哪台相机历史上匹配得好，就不该让它影响决策。
    MeasurementSelector(const data::MeasurementConfig& measurement,
                        const data::ValidationConfig& validation,
                        const data::IMatchStatsStore* store = nullptr);


    // ---- 打分 -------------------------------------------------------------

    /// 对候选集合打分，就地填写四分项、E 的三个输入、总分。
    ///
    /// 输入要求：每个候选项的 `camera` / `scale` / `quality` /
    /// `featureSpreadPx` / `nDetect` 已由调用方（`PosePipeline`）填好。
    /// 其余字段（qNorm…score）由本方法写入，入参中的旧值被覆盖。
    ///
    /// **不满足 SYS-14 §6 硬门槛的候选会被从 `candidates` 中删除**：
    ///   · `quality.sharpness`   < `minSharpness`
    ///   · `scale.targetPixelSize` < `minTargetPixelSize`
    ///   · `nDetect`             < `minFeatureCount`
    ///   · `quality.matchRatio`  < `minMatchRatio`
    /// 删除（而不是打 0 分）的理由：0 分仍会参与"最大值"比较，
    /// 当所有候选都不合格时，会让一个明确不合格的通道被选中，
    /// 而正确的行为是"无候选可用"→ MEASURE_SELECT 重试（SYS-08 §7.3）。
    ///
    /// @param targetModelId 机型 + 外形状态（`TargetModel::modelId`）。
    ///        ENG-10 §4.1：不同机型必须使用独立的统计表 —— 混用会让 M_hist
    ///        反映另一架飞机的纹理可匹配性（换涂装即失效）。
    /// @return 是否还有候选存活。false = 全部被门槛淘汰。
    bool scoreAll(std::vector<data::MeasurementCandidate>& candidates,
                  const std::string& targetModelId) const;

    // ---- 选择 -------------------------------------------------------------

    /// 在**已打分**的候选集合中选出总分最高者。
    ///
    /// @return false = 候选集合为空。
    ///
    /// ⚠ 得分式是 `Score = w1·qNorm + w2·fNorm + w3·mNorm + **+** w4·eNorm`
    ///   ——**四项皆为"越大越好"的归一化优度，全部取正号**，而不是
    ///   SYS-14 §6.2 字面上的 `− w4·E`。这不是笔误，而是 §8.2 / ENG-10 §3.6
    ///   （更具体的一节）在定义 `eNorm = 1 − min(1, E/E_ref)` 之后的必然结果：
    ///   把 §6.2 的减号照搬到**已归一化的优度**上（`− w4·eNorm`），
    ///   "预测误差最小"的最好候选反而得分最低。
    ///   （注意区分：§6.2 的 `− w4·E` 用的是**原始**误差，它与本式**排序方向
    ///   相同**，差别只在归一化；颠倒来自那个混合写法。）完整论证见
    ///   MeasurementSelector.cpp 中的注释与 README §6 的待裁决清单。
    ///   推论：得分不必为正（它是加权和，不是概率），也不是"越大越差"；
    ///   三者中总有一个相对最好，故不设"最低分"门槛 ——
    ///   "选了它但测量不达标"由 VALIDATE 与 SYS-08 §7.3 的重试处理，
    ///   比"三台都不选"更符合状态机的设计（后者的出口是 FAILED）。
    bool select(const std::vector<data::MeasurementCandidate>& candidates,
                data::MeasurementSelectionResult& out) const;

    // ---- 分桶（ENG-10 §4.2 冻结边界）--------------------------------------

    /// 距离带索引：0:<80 / 1:80~150 / 2:150~220 / 3:220+
    /// 边界取自 `MeasurementConfig::distanceBandEdges`，**不得另设**。
    static int distanceBandOf(double distanceM,
                              const data::MeasurementConfig& config);

    /// 光照带索引：0:低照度 / 1:正常 / 2:强光逆光
    /// 判据为 `ImageQuality::exposure`（ENG-10 §4.2 冻结），边界由
    /// `MeasurementConfig::illumBandEdges` 给定。
    static int illumBandOf(double exposure,
                           const data::MeasurementConfig& config);

    // ---- 追溯 -------------------------------------------------------------

    /// 最近一次 `scoreAll` 中每个存活候选使用的 M_hist 三元组。
    /// 供 result.json 记录（ENG-10 §4.4）。顺序与传入 `scoreAll` 的存活
    /// 候选顺序一致。
    ///
    /// ⚠ **当前状态：尚无生产消费方**（2026-09-23 的"死诊断接口清理"删掉了
    ///   `PosePipeline` 上暴露它的那个访问器，因为它属"内部状态查询"）。
    ///   这不是一条可以按"没人用"删掉的东西：
    ///   ENG-10 §4.4 要求把三元组 + 冷启动标记落进 result.json，
    ///   而本处是**该数据唯一的产出点**。删除它等于把这条尚未实施的需求
    ///   一并删掉。正确的实施方向是经推送通道（`IPipelineObserver`）
    ///   暴露，而不是恢复查询接口 —— 已登记为本批的开放项。
    ///   目前的消费方只有 `tests/algorithm/AlgorithmStageTest.cpp`
    ///   的 `UsageTripleRecordsTheCamera`（用例直连选择器，不经 Pipeline）。
    std::vector<MatchStatsUsage> lastUsage() const { return lastUsage_; }

    /// 最近一次选择是否处于历史统计冷启动（无表 / 样本不足先验）。
    /// 同上：`lastColdStart_` 是 §4.4 冷启动标记的唯一产出点，勿按"无消费方"删除。
    bool lastSelectionColdStart() const { return lastColdStart_; }

private:
    /// 单点定位标准差预测（ENG-10 §3.2）：
    ///     σ_px_est = a + b·(1/S) + c·(1/C)
    /// S = **清晰度**（`ImageQuality::sharpness`），C = 对比度
    /// （`ImageQuality::contrast`）—— 见 .cpp 中关于本式符号的一处文档冲突。
    /// 系数未标定（a=b=c=0）时返回 `sigmaPxFallback`（默认 0.5 pixel，保守值）。
    double sigmaPxEstimate(const data::ImageQuality& quality,
                           bool& calibrated) const;

    /// Q 分项：清晰度 / 曝光 / 对比度三项归一化后的均值。
    double qualityNorm(const data::ImageQuality& quality) const;

    // ⚠ 两份配置**按值**持有，而非 ENG-10 §5.1 字面上的 `const&`
    //   （构造函数入参仍是 `const&`，注入面没有变化）：
    //   存引用的后果是 `MeasurementSelector(cfg, configuredValidation(), nullptr)`
    //   这类**传临时量**的写法可以正常编译，而对象在构造函数所在的全表达式
    //   结束时即悬空 —— 之后每次读到的都是栈上的残留数据，表现为"验证结论
    //   随机正确/随机错误"，编译期与静态检查都不会报错（007 单测中已实际踩到，
    //   详见 tests/algorithm/AlgorithmStageTest.cpp 的验证器用例）。
    //   按值持有时 ENG-10 §5.2 约束 2"注入后不可变"成为**结构性事实**：
    //   外部再怎么改自己那份配置，本对象的决策依据也不会变。
    //   两份配置都是小尺寸 POD，拷贝代价可忽略。
    data::MeasurementConfig measurement_;
    data::ValidationConfig  validation_;
    const data::IMatchStatsStore*  store_;

    /// ⚠ `validation_` 目前**没有被读取**（这不是遗漏，是一个已登记的缺口）：
    /// ENG-10 §3.6 要求 `E_ref` 取 "ValidationConfig 允许的最大误差"，但冻结的
    /// `ValidationConfig`（ENG-09 §6.6）里唯一的误差字段是 `maxReprojectionError`，
    /// 单位是 **pixel**，而 E 的单位是**角分** —— 直接拿它当 E_ref 是单位混用。
    /// 故本实现暂取系统自身的 Yaw 指标（1 角分）为 E_ref，完整论证见
    /// MeasurementSelector.cpp 的 `kYawErrorRefArcmin`。该注入被保留是因为
    /// ENG-10 §5.1 的注入矩阵规定它必须存在，且一旦裁决补入
    /// `yawMaxErrorArcmin`，`E_ref` 就由它供给（届时删掉常量即可）。

    /// 追溯信息。`scoreAll` 在语义上是只读操作（不改本对象的决策依据），
    /// 故置 mutable —— 与 ENG-10 §5.2 约束 2"注入后不可变"不冲突：
    /// 被冻结的是**配置**，这里是本次选择的**记录**。
    mutable std::vector<MatchStatsUsage> lastUsage_;
    mutable bool lastColdStart_ = false;
};

}  // namespace algorithm
}  // namespace aircraft
