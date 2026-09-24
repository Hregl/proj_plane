#pragma once

// ============================================================================
//  src/algorithm/scale/TargetScaleEstimator.h
//
//  依据：SYS-07 §5（尺度估计，含 §5.2 的三项输入）、SYS-14 §9 / §10、
//        ENG-02 §11.2（文件路径）、ENG-01 §10（scale 子模块）、
//        ENG-09 §2.3（长度单位为米）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与 8.md §六 的两处偏离：                                              │
//  │  1. 8.md 定义了 `struct ScaleEstimate { distance; pixelSize;         │
//  │     confidence; }`。ENG-09 §5.14 已冻结 `data::TargetScaleEstimate`   │
//  │     （字段名是 **targetPixelSize**，不是 pixelSize），故用冻结类型。    │
//  │  2. 8.md 的签名是 `estimate(const DetectionResult&)` —— 只有检测框。  │
//  │     SYS-07 §5.2 明确列出三项输入：**bbox 尺寸、相机内参、飞机模型      │
//  │     尺寸**。内参与模型尺寸都不在检测结果里，故签名补齐。               │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 用途限制（ENG-09 §5.14 明文约束）：距离估计**仅用于通道选择与图像
//  质量评价**，不得作为最终输出，也不得进入姿态解算 —— 否则距离误差会以
//  **系统偏差**形式进入 Yaw（系统偏差在 SYS-15 §4 的合成规则中线性相加，
//  不像随机误差那样 RSS 相消）。
// ============================================================================

#include "data/CameraCalibration.h"
#include "data/DetectionResult.h"
#include "data/TargetScaleEstimate.h"

namespace aircraft
{
namespace algorithm
{

/// 目标尺度估计抽象（SYS-07 §5）。实现方向：本类 → 模型匹配尺度 / 多焦段比较。
class TargetScaleEstimator
{
public:
    virtual ~TargetScaleEstimator() = default;

    /// @param detection       检测结果（提供目标像素尺寸 l）。
    /// @param calibration     该焦段的标定（提供 f_x，单位 **pixel**）。
    ///        ⚠ 不用 `CameraChannel::focalLength`：ENG-09 §2.3 冻结它为**米**
    ///        （100 mm 镜头写作 0.1）。用米代入距离式会差 290 倍
    ///        （3.45 µm 像元），且**不报任何错**。
    /// @param targetRealSizeM 目标真实尺寸 L，单位 **m**（SYS-07 §5.2 的
    ///        "飞机模型尺寸"）。由 `TargetModelManager` 的模型外接尺寸给出；
    ///        本类不自行读模型文件（算法的文件访问集中在 TargetModelManager）。
    /// @param out             `data::TargetScaleEstimate`。
    /// @return 是否给出可用估计。检测框退化、内参不可用、模型尺寸为零 → false。
    virtual bool estimate(const data::DetectionResult& detection,
                          const data::CameraCalibration& calibration,
                          double targetRealSizeM,
                          data::TargetScaleEstimate& out) = 0;
};

/// 针孔模型尺度估计（SYS-14 §10 方法 1）。
///
/// 冻结公式（SYS-14 §10 的"严格形式"与它的等价简化式）：
///
///     Z = f · L / (l · s)          f 单位 m，s 为像元尺寸（m/pixel）
///     Z = f_x · L / l              f_x 单位 pixel（= f/s）
///
/// 本类实现**第二式**，因为 `CameraCalibration::cameraMatrix` 的 (0,0)
/// 元素正是 f_x（像素）；用第一式就必须知道像元尺寸 s，而 s 不在任何冻结
/// 结构体里。SYS-14 §10 已明文允许："若采用简化的 Z = f·L/l，则 f 必须
/// 理解为以像素为单位的 f_x" —— 两式数学上完全相同，区别只是把 s 折进 f_x。
///
/// SYS-14 §10 的笔误已裁决：原式 `Z = fL/(lD)` 分母里的 D 在说明中不存在，
/// 已按针孔模型更正为上式。
class PinholeScaleEstimator : public TargetScaleEstimator
{
public:
    bool estimate(const data::DetectionResult& detection,
                  const data::CameraCalibration& calibration,
                  double targetRealSizeM,
                  data::TargetScaleEstimate& out) override;
};

/// 固定值尺度估计（8.md §六 的 `MockTargetScaleEstimator`）。
///
/// 用于"选择逻辑本身"的单元测试：给定三个候选的固定距离，验证
/// `MeasurementSelector` 选出预期的通道，无需构造真实几何。
/// 生产路径不得使用。
class MockTargetScaleEstimator : public TargetScaleEstimator
{
public:
    explicit MockTargetScaleEstimator(double distanceM, double confidence = 1.0);

    bool estimate(const data::DetectionResult& detection,
                  const data::CameraCalibration& calibration,
                  double targetRealSizeM,
                  data::TargetScaleEstimate& out) override;

private:
    double distanceM_;
    double confidence_;
};

}  // namespace algorithm
}  // namespace aircraft
