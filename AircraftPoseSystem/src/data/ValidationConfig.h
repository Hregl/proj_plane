#pragma once

// ============================================================================
//  src/data/ValidationConfig.h
//
//  依据：ENG-09 §4.1、§6.6（类型定义冻结）、§2.2（角度单位）
//        SYS-07 §12（结果验证）
//
//  作用：PoseValidator 的判据阈值。**这些数值直接决定"结果是否有效"**。
//
//  ⚠ 必须随结果一同落盘（ENG-09 §6.8）：
//  SYS-05 §16 承诺"支持离线复现"，但复现的**判定结论**取决于阈值。
//  同一批图像在 maxReprojectionError = 2.0 与 2.5 下会得出
//  valid = true / false 两种结论，而两次都"复现成功"却给出相反答案，
//  这个承诺就是空的。因此 SYS-12 §17 / SYS-15 §16 只保存
//  calibration_id / model_id 是不够的 —— 必须保存 config_snapshot/。
//
//  ⚠ yawMin/yawMax 的用途（SYS-07 §12.2 姿态合理性）：
//  它们是**物理可达范围**的判据，不是精度判据。例如本系统目标 Yaw
//  限定在 ±30 度内。该判据的作用是拦截"解算发散了但碰巧三指标都过关"
//  的情形 —— 发散的 PnP 有时会给出一个重投影误差很小的解，
//  因为外点被 RANSAC 剔除后剩余点恰好拟合出一个错误但自洽的姿态。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 结果验证阈值（ENG-09 §6.6）。
struct ValidationConfig
{
    /// 重投影误差上限，单位 **pixel**。
    /// 判据：PoseValidationResult::reprojectionError <= maxReprojectionError
    double maxReprojectionError = 0.0;  // pixel

    /// 内点比例下限，[0,1]。
    /// 判据：PoseValidationResult::inlierRatio >= minInlierRatio
    double minInlierRatio = 0.0;

    /// 置信度下限，[0,1]。
    /// 判据：PoseValidationResult::confidence >= minConfidence
    double minConfidence = 0.0;

    /// Yaw 允许下限，单位 **deg**（ENG-09 §2.2）。
    double yawMin = 0.0;  // deg

    /// Yaw 允许上限，单位 **deg**。
    double yawMax = 0.0;  // deg
};

}  // namespace data
}  // namespace aircraft
