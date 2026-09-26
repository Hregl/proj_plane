#pragma once

// ============================================================================
//  src/algorithm/validation/PoseValidator.h
//
//  依据：ENG-01 §10（validation 子模块，类名 `PoseValidator` 被冻结）、
//        SYS-07 §12（结果验证：重投影误差 / 内点比例 / 姿态合理性）、
//        SYS-08 §5.9（VALIDATE 状态的输出）、
//        ENG-09 §5.25（PoseValidationResult）、§6.6（ValidationConfig）、
//        ENG-10 §3.6（E_ref 的出处）、§5.1（配置注入矩阵）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与冻结接口的一处冲突（本类的 `validate` 多两个入参）：                 │
//  │                                                                      │
//  │ `IPosePipeline::validate(const ShipPoseResult&, PoseValidationResult&)`│
//  │ 是 006 冻结的（SYS-04 §4.2 / §7 第 1 条：签名变更 = 不兼容变更）。     │
//  │ 但它要产出的 `PoseValidationResult`（ENG-09 §5.25）含 `inlierRatio`   │
//  │ 与 `confidence`，而**入参 `ShipPoseResult`（§5.24）里没有这两个量**： │
//  │ 它只有 success / yaw / pitch / roll / aircraftToShip /                │
//  │ reprojectionError。二者在冻结文档里无法闭合。                          │
//  │                                                                      │
//  │ 处理：**不改冻结签名**。`PosePipeline::validate` 从 006 的签名收      │
//  │ `ShipPoseResult`，把 `inlierRatio`/`matchCount` 这些"本类需要但        │
//  │ ShipPoseResult 没有"的量，从 Pipeline 在 POSE_SOLVE 阶段缓存的         │
//  │ `PnpStats`（见 pose/PnPPoseEstimator.h）中取出，再转交给本类的         │
//  │ `validate(result, inlierRatio, matchCount, out)`。                    │
//  │ 本类因此**不实现** IPosePipeline 的任何方法 —— 它是一个纯判据函数      │
//  │ 对象，由 Pipeline 持有（组合而非继承），这样接口层不增加任何            │
//  │ 冻结类型里不存在的东西。已记入 README §6 的偏离登记。                  │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ `PoseValidationResult::confidence` 的算式**没有任何冻结定义**。
//    SYS-07 §12 只列了"综合置信度 ≥ minConfidence"这条判据，未给算式；
//    ENG-09 §5.25 只说它是 [0,1]。故本实现的定义见 .cpp 中的详细说明，
//    并已记入 README §6 的待裁决清单 —— 这是一个**必须由裁决确定**的量，
//    因为它的算式直接决定 VALIDATE 状态下多少实测结果会判不合格。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"
#include "data/ValidationConfig.h"

namespace aircraft
{
namespace algorithm
{

/// 姿态验证器（SYS-07 §12）。
///
/// 配置注入（ENG-10 §5.1 冻结的注入行）：
///   `ValidationConfig → PoseValidator`
/// 注入后不可变（§5.2 约束 2：禁止运行时改配置）。
class PoseValidator
{
public:
    /// @param config 验证阈值（注入 `const&`）。
    explicit PoseValidator(const data::ValidationConfig& config);

    /// 逐条判据验证，并把三项判据值写入 `out`。
    ///
    /// @param result      待验证的姿态结果（重投影误差与 Yaw 从这里取）。
    /// @param inlierRatio 内点比例 [0,1]（来自 PnP 阶段的 `PnpStats`，
    ///        `ShipPoseResult` 里没有这个量，见文件头）。
    /// @param matchCount  匹配内点数（同样来自 PnP 阶段）。仅用于置信度的
    ///        构成与诊断，不单独设门槛（门槛是 `minInlierRatio`）。
    /// @param out         `PoseValidationResult`；无论通过与否，三个字段
    ///        （reprojectionError / inlierRatio / confidence）都会被填写 ——
    ///        SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求失败任务也记录失败原因，缺值会让 result.json
    ///        无法说明"为什么判不合格"。
    /// @return 是否通过全部判据（等于 out.valid）。
    bool validate(const data::ShipPoseResult& result,
                  double inlierRatio,
                  int matchCount,
                  data::PoseValidationResult& out) const;

    /// 综合置信度，[0,1]。独立暴露以便单测直接校验算式。
    double confidence(double reprojectionError, double inlierRatio) const;

private:
    // ⚠ 按值持有而非 ENG-10 §5.1 字面上的 `const&`（入参仍是 `const&`）：
    //   存引用会让 `PoseValidator(configuredValidation())` 这种传临时量的写法
    //   通过编译，而阈值在全表达式结束时随临时量析构 —— 之后每次 validate()
    //   都拿栈上的残留字节做比较，表现为"同一份输入时对时错"。
    //   007 单测正是被这一点绊住的（tests/algorithm/AlgorithmStageTest.cpp）。
    //   按值持有使 §5.2 约束 2"注入后不可变"成为结构性事实。
    data::ValidationConfig config_;
};

}  // namespace algorithm
}  // namespace aircraft
