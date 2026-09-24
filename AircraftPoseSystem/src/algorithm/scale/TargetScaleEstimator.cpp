// ============================================================================
//  src/algorithm/scale/TargetScaleEstimator.cpp
//
//  依据：SYS-14 §10（距离估计 Z = f·L/(l·s)，及其等价式 Z = f_x·L/l）、
//        SYS-07 §5、ENG-09 §2.3（长度单位为米）
// ============================================================================

#include "algorithm/scale/TargetScaleEstimator.h"

#include <algorithm>
#include <cmath>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 目标像素尺寸参考值，单位 pixel。
///
/// 取值来源：与 `MeasurementConfig::nRef` 同源的量级 —— SYS-15 §4.5 校核出的
/// "足够好"的特征量级（100 点）。**不是新引入的可调参数**（SYS-14 §20
/// 约束 3 反对无依据的可调量），而是复用同一份校核结论。
constexpr double kReferenceSizePx = 100.0;

/// 距离估计的置信度。
///
/// ⚠ 冻结文档**没有**给出置信度的定义（SYS-14 §9 只列出该字段）。
/// 本类取"目标在图像中的像素尺寸相对参考值"作为代理量：目标越大，
/// `l` 的相对量化误差越小，Z 越可信。
///
/// 该定义属本类自定，已记入 README §6 的待裁决清单（ENG-09 §8 的变更范畴）。
double scaleConfidence(double pixelSize)
{
    return std::min(1.0, std::max(0.0, pixelSize / kReferenceSizePx));
}

}  // namespace

// ---------------------------------------------------------------------------

bool PinholeScaleEstimator::estimate(const data::DetectionResult& detection,
                                     const data::CameraCalibration& calibration,
                                     double targetRealSizeM,
                                     data::TargetScaleEstimate& out)
{
    out = data::TargetScaleEstimate{};

    if (!detection.found)
    {
        return false;
    }

    // ---- 输入 ①：目标真实尺寸 L（SYS-07 §5.2 的"飞机模型尺寸"）----
    if (!(targetRealSizeM > 0.0) || !std::isfinite(targetRealSizeM))
    {
        // 模型外接尺寸为零：模型未加载，或 points3d.yaml 为空。
        // 此时 Z 无法计算 —— 返回 0 距离会让所有候选都落到最近的距离带，
        // 使通道选择整体错位，且不会报错。
        return false;
    }

    // ---- 输入 ②：内参 f_x（SYS-07 §5.2 的"相机内参"）----
    if (calibration.cameraMatrix.empty()
        || calibration.cameraMatrix.rows < 3
        || calibration.cameraMatrix.cols < 3
        || calibration.cameraMatrix.type() != CV_64F)
    {
        return false;
    }
    const double fx = calibration.cameraMatrix.at<double>(0, 0);
    if (!(fx > 0.0) || !std::isfinite(fx))
    {
        return false;
    }

    // ---- 输入 ②b：图像尺寸必须非零 ----
    //
    // ⚠ 这一条不是多余的防御，而是**区分"标定对象已填写"与"未填写"的唯一手段**。
    // ENG-09 §5.2 冻结的默认构造 `CameraCalibration` 是
    //     cameraMatrix = cv::Mat::eye(3,3,CV_64F)   // f_x = f_y = 1
    //     imageWidth = imageHeight = 0
    // 即 `fx = 1` **是一个合法正数**：若只校验 `fx > 0`，一个从未被赋值的
    // 标定对象会静默通过，算出 Z = 1·L/l —— 对本工程的量级而言距离恒 ≈0.02 m，
    // 于是所有候选都落进最近的距离带（ENG-10 §4.2 的带 0），通道选择整体错位，
    // 而**没有任何一处会报错**。真实标定（fx ≈ 7246 px）与未填写（fx = 1）
    // 相差三个量级，必须在这里断开。
    //
    // 用图像尺寸而非"fx 大于某个阈值"作为判据，是因为尺寸是有独立来源的
    // 客观量（相机分辨率，C-02 要求标定必须含图像尺寸），而阈值只能凭空指定。
    if (calibration.imageWidth <= 0 || calibration.imageHeight <= 0)
    {
        return false;
    }

    // ---- 输入 ③：目标像素尺寸 l ----
    //
    // SYS-14 §10 只说 l 是"目标在图像中的尺寸"，未指定取宽还是高。
    // 取**长边**的理由：飞机的图像投影长宽比通常接近 3:1，短边受姿态影响
    // 远大于长边（侧视转前视时长边缩小、短边几乎不变），故长边的相对变化
    // 更小，作为尺度基准更稳定。
    // 这与 ENG-10 §2.4 的"展布宽度 W ≥ 0.6 × 检测框长边"使用同一基准，
    // 避免同一帧内并存两套尺度参照。
    const int longSide = std::max(detection.bbox.width, detection.bbox.height);
    if (longSide <= 0)
    {
        return false;
    }

    // ---- Z = f_x · L / l ----
    const double z = fx * targetRealSizeM / static_cast<double>(longSide);
    if (!std::isfinite(z) || !(z > 0.0))
    {
        return false;
    }

    out.distance = z;                                       // m
    out.targetPixelSize = static_cast<double>(longSide);     // pixel
    out.confidence = scaleConfidence(out.targetPixelSize);
    return true;
}

// ---------------------------------------------------------------------------

MockTargetScaleEstimator::MockTargetScaleEstimator(double distanceM,
                                                   double confidence)
    : distanceM_(distanceM)
    , confidence_(confidence)
{
}

bool MockTargetScaleEstimator::estimate(const data::DetectionResult& detection,
                                        const data::CameraCalibration&,
                                        double,
                                        data::TargetScaleEstimate& out)
{
    out = data::TargetScaleEstimate{};
    if (!detection.found)
    {
        return false;
    }
    out.distance = distanceM_;
    out.targetPixelSize = static_cast<double>(
        std::max(detection.bbox.width, detection.bbox.height));
    out.confidence = confidence_;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
