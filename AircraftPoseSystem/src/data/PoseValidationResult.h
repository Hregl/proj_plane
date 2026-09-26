#pragma once

// ============================================================================
//  src/data/PoseValidationResult.h
//
//  依据：ENG-09 §4.1、§5.25（类型定义冻结）
//        SYS-07 §12（结果验证）
//
//  作用：PoseValidator 对 ShipPoseResult 的判定结论。SYS-08 的 VALIDATE
//  状态以 valid 决定前进到 SAVE 还是按 §7.3 重试（最多 2 次，
//  受 3 相机上限约束）。
//
//  ⚠ 三个判据与 ValidationConfig 的对应（ENG-09 §6.6）：
//      reprojectionError <= maxReprojectionError
//      inlierRatio       >= minInlierRatio
//      confidence        >= minConfidence
//  另有姿态合理性判据 yaw ∈ [yawMin, yawMax]（SYS-07 §12.2）。
//  阈值来自 ValidationConfig，**且必须随测量结果一同保存**（ENG-09 §6.8）：
//  同一批图像在不同版本阈值下会得出不同判定，不带阈值的回放无法复现判定。
//
//  ⚠ 裁决 C-008（V2.1-C01 §C-008，2026-09-23 批准）加入 reason 字段：
//  `valid == false` 只说明"没通过"，说明不了"**为什么**没通过"。
//  而这两件事的处置方向完全不同：阈值未过 → 调阈值或重采；
//  数值本身不是数（NaN）→ 上游算法出了问题，重采多少次都一样。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 判不合格的**原因分类**（裁决 C-008）。
///
/// ⚠ 只登记裁决明确给出的取值，不发明未裁决的分类：
///   `OK` 与 `NON_FINITE_VALUE` 两个够了，其余原因继续由
///   `reprojectionError` / `inlierRatio` / `confidence` 三个**数值**表达
///   （"差多少"比"属于哪一类"信息量更大）。凭想象切分阈值未过一类，
///   等于凭空发明一套 ENG-09 §5.27 表外的分类 ——
///   与错误码共用一个"必须有出处"的纪律。
enum class PoseValidationReason
{
    /// 通过（`valid == true`），或未识别的其它不合格原因。
    ///
    /// ⚠ 0 取"通过"而不是"未设置"：本枚举不是错误码表
    /// （ENG-09 §5.27 的 `code == 0` 才是"未设置"），
    /// 且 `PoseValidationResult{}` 的默认 `valid` 已是 false ——
    /// 若把 0 记作"未设置"，一个默认构造的结果会同时是
    /// "valid=false"与"reason=未设置"，读者得不到任何信息。
    OK = 0,

    /// 姿态或验证量中出现非有限值（NaN / ±inf），**先于全部阈值判据**拦下。
    ///
    /// 为什么必须单独一类：这些值在阈值比较里全都"通不过"或"通过得莫名其妙"
    /// （`NaN > max` 与 `NaN < min` 同为 false），无法归到任何阈值判据上；
    /// 而它指向的处置是"查上游算法/PnP 退化"，不是"调阈值"。
    NON_FINITE_VALUE,
};

/// 姿态验证结果（SYS-07 §12）。
struct PoseValidationResult
{
    /// 是否通过全部判据。false 时 SYS-08 的 VALIDATE 进入重试或 FAILED。
    bool valid = false;

    /// 判不合格的原因分类（裁决 C-008）。`valid == true` 时为 `OK`。
    ///
    /// ⚠ 与三个数值字段的分工：本字段回答"属不属于需要特殊处置的那一类"
    ///   （目前只有非有限值一类），数值字段回答"差多少"。
    ///   排查时**先看本字段**：`NON_FINITE_VALUE` 意味着后面的数值
    ///   全都不可信（它们可能本身就是 NaN，或在 NaN 参与下算出来的）。
    PoseValidationReason reason = PoseValidationReason::OK;

    /// 重投影误差，单位 pixel。
    double reprojectionError = 0.0;  // pixel

    /// 匹配内点比例，[0,1]。
    double inlierRatio = 0.0;

    /// 综合置信度，[0,1]。
    double confidence = 0.0;
};

}  // namespace data
}  // namespace aircraft
