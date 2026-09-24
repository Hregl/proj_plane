#pragma once

// ============================================================================
//  src/data/TargetScaleEstimate.h
//
//  依据：ENG-09 §4.1、§5.14（类型定义冻结）、§2.3（长度单位）
//        SYS-14 §9 / §10（距离估计）
//
//  ⚠ 字段名是 targetPixelSize，**不是** 8.md §六 的 pixelSize。
//
//  ⚠ 用途限制（ENG-09 §5.14 明文约束）：
//  该值**不作为最终输出**（受 SYS-02 FR-004 约束），只允许被
//  MeasurementSelector 消费。
//  为什么必须写进头文件：本结构体里的 distance 是对目标距离的**估计**，
//  由目标像素尺寸与已知物理尺寸反算（SYS-14 §10：Z = f·L/(l·s)），
//  其误差量级远大于 1 角分对应的角度误差。若被误当作测量结果输出，
//  会让使用者以为系统提供了测距能力。最终输出只有 ShipPoseResult。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 目标尺度估计（SYS-14 §9）。
struct TargetScaleEstimate
{
    /// 目标距离估计值，单位 **m**（ENG-09 §2.3）。
    /// **仅用于通道选择**，不是测距输出。
    double distance = 0.0;  // m

    /// 目标在图像中的像素尺寸（检测框长边），单位 pixel。
    double targetPixelSize = 0.0;

    /// 估计置信度，[0,1]。
    double confidence = 0.0;
};

}  // namespace data
}  // namespace aircraft
