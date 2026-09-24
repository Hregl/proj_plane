// ============================================================================
//  src/algorithm/pose/PnPPoseEstimator.cpp
//
//  依据：SYS-07 §10、§12.2、ENG-09 §5.23、裁决 C-06、ENG-10 §5.1
//
//  本文件含两个实现：CvPnPPoseEstimator（真实）与 MockPnPPoseEstimator
//  （测试夹具）。
// ============================================================================

#include "algorithm/pose/PnPPoseEstimator.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// PnP 的几何最小点数（SYS-07 §10）：少于 4 对对应无法唯一确定位姿。
constexpr int kMinPnpPoints = 4;

/// 内点集重解后的重投影误差口径：**内点上的 RMS**，单位 pixel。
///
/// 为什么是 RMS 而不是最大值或均值：
///   · 最大值对单个离群残差极敏感，会让"几乎解对了"与"解错了"看起来一样；
///   · 均值会与内点比例纠缠（内点越多、平均残差越小），不适合做收敛判据；
///   · RMS 与 SYS-15 §4.5 的 σ_px 口径一致（同为二次量纲的均值），
///     便于与误差预算对照。
/// 该口径是冻结文档未指定的一处选择，已记入 README §6。
double rmsReprojectionError(
    const std::vector<data::FeatureCorrespondence>& correspondences,
    const std::vector<int>& indices,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distortion,
    const cv::Mat& rvec,
    const cv::Mat& tvec)
{
    if (indices.empty())
    {
        return 0.0;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    objectPoints.reserve(indices.size());
    imagePoints.reserve(indices.size());
    for (int idx : indices)
    {
        objectPoints.push_back(correspondences[static_cast<size_t>(idx)]
                                   .objectPoint);
        imagePoints.push_back(correspondences[static_cast<size_t>(idx)]
                                  .imagePoint);
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortion,
                      projected);

    double sumSq = 0.0;
    for (size_t i = 0; i < projected.size(); ++i)
    {
        const double dx = static_cast<double>(projected[i].x)
                        - static_cast<double>(imagePoints[i].x);
        const double dy = static_cast<double>(projected[i].y)
                        - static_cast<double>(imagePoints[i].y);
        sumSq += dx * dx + dy * dy;
    }
    return std::sqrt(sumSq / static_cast<double>(projected.size()));
}

/// 内参可用性校验。默认构造的 `CameraCalibration` 的 cameraMatrix 是
/// `cv::Mat::eye(3,3,CV_64F)`（ENG-09 §5.16 冻结的默认值），即 fx = fy = 1 ——
/// 这是一个**合法但不正确**的内参，直接送进 solvePnP 会算出一个
/// 量级完全错误、却依然"收敛"的位姿。故此处显式拒绝。
bool validIntrinsics(const data::CameraCalibration& calibration)
{
    const cv::Mat& k = calibration.cameraMatrix;
    if (k.rows != 3 || k.cols != 3 || k.type() != CV_64F)
    {
        return false;
    }
    if (!std::isfinite(k.at<double>(0, 0))
        || !std::isfinite(k.at<double>(1, 1)))
    {
        return false;
    }
    if (!(k.at<double>(0, 0) > 0.0) || !(k.at<double>(1, 1) > 0.0))
    {
        return false;
    }

    // 图像尺寸必须非零 —— 与 PinholeScaleEstimator::estimate 的同一判据，
    // 理由相同：ENG-09 §5.2 冻结的默认构造 `CameraCalibration` 是
    // `cameraMatrix = Mat::eye(3,3,CV_64F)`（f_x = f_y = 1）+ 尺寸全 0，
    // 而 `f_x = 1` **能通过上面所有检查**。若在此放行，PnP 会用
    // "焦距 1 pixel" 求解，得到一个形状合法、数值荒谬的位姿：
    // 平移量被压到毫米级、旋转量由噪声主导，而 `PnpStats::converged` 仍为真
    // —— 错误从这一刻起进入 `ShipPoseResult`，后续 VALIDATE 只能看到
    // "重投影误差偏大"，无法归因到"标定根本没填"。
    if (calibration.imageWidth <= 0 || calibration.imageHeight <= 0)
    {
        return false;
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
//  CvPnPPoseEstimator
// ---------------------------------------------------------------------------

CvPnPPoseEstimator::CvPnPPoseEstimator(const data::MeasurementConfig& config)
    : config_(config)
{
}

void CvPnPPoseEstimator::setRansacReprojectionThreshold(double px)
{
    // 非正的阈值会让 RANSAC 找不到任何内点集（每个点都被判为外点）。
    // 夹到 1e-3 pixel 而不是忽略：误用会表现为"内点比例恒为 0"（可见）。
    ransacReprojectionThreshold_ = std::max(1e-3, px);
}

void CvPnPPoseEstimator::setRansacIterations(int iterations)
{
    ransacIterations_ = std::max(1, iterations);
}

void CvPnPPoseEstimator::setRansacConfidence(double confidence)
{
    if (!std::isfinite(confidence))
    {
        return;
    }
    // (0,1)：0 会让 OpenCV 的迭代次数公式除零，1 等价于无穷次迭代。
    ransacConfidence_ = std::min(0.999999, std::max(1e-6, confidence));
}

bool CvPnPPoseEstimator::estimate(
    const std::vector<data::FeatureCorrespondence>& correspondences,
    const data::CameraCalibration& calibration,
    data::CameraPose& out,
    PnpStats& stats)
{
    out = data::CameraPose{};
    stats = PnpStats{};

    // 门槛：几何最小值与配置值取大。
    // 配置的 `minFeatureCount` 默认 0（002 阶段的"未配置"哨兵），
    // 若直接当门槛使用，会让 PnP 去解一个欠定方程组并"成功"返回一个
    // 无意义的位姿 —— 这是本类最危险的失败模式，故必须夹取。
    const int required = std::max(kMinPnpPoints, config_.minFeatureCount);
    stats.inputCount = static_cast<int>(correspondences.size());
    if (stats.inputCount < required)
    {
        return false;
    }
    if (!validIntrinsics(calibration))
    {
        return false;
    }

    // 畸变系数可以为空（未标定畸变 = 无畸变模型）。非空时必须是
    // 可被 OpenCV 消费的浮点向量；cv::Mat 的空矩阵即"无畸变"。
    const cv::Mat& distortion = calibration.distortion;

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    objectPoints.reserve(correspondences.size());
    imagePoints.reserve(correspondences.size());
    for (const data::FeatureCorrespondence& c : correspondences)
    {
        // ENG-09 §2.3：objectPoint 的单位是 m。此处**不做任何单位换算** ——
        // 标定文件的 mm→m 已在 CalibrationManager::load() 内一次性完成，
        // 算法层再换算一次会把 290× 的错误悄悄乘回来。
        objectPoints.push_back(c.objectPoint);
        imagePoints.push_back(c.imagePoint);
    }

    cv::Mat rvec;
    cv::Mat tvec;
    std::vector<int> inliers;

    try
    {
        cv::solvePnPRansac(objectPoints, imagePoints, calibration.cameraMatrix,
                           distortion, rvec, tvec,
                           false,                  // useExtrinsicGuess
                           ransacIterations_,
                           ransacReprojectionThreshold_,
                           ransacConfidence_,
                           inliers,
                           cv::SOLVEPNP_ITERATIVE);
    }
    catch (const cv::Exception&)
    {
        // 退化构型（共线点、点全同）会让求解器抛异常。这不是"程序错误"，
        // 而是输入不可解 —— 按本接口的约定（ENG-09 §8 / IF-SW-02）
        // 返回 false 并让上层按 SYS-08 §7.3 重试。
        out = data::CameraPose{};
        stats = PnpStats{};
        stats.inputCount = static_cast<int>(correspondences.size());
        return false;
    }

    if (inliers.size() < static_cast<size_t>(kMinPnpPoints))
    {
        // RANSAC 没能凑出一致集。恢复到"未收敛"，不把 rvec/tvec 外泄 ——
        // 一个由外点撑起来的位姿看起来与真值同型，只会把错误推到下游。
        out = data::CameraPose{};
        stats = PnpStats{};
        stats.inputCount = static_cast<int>(correspondences.size());
        return false;
    }

    // ---- 只用内点重解（refine）----
    // RANSAC 的最小解只用了 4 点，噪声直接进入结果。内点集一旦确定，
    // 用全部内点做一次最小二乘迭代能显著降低重投影误差，
    // 这是 §12.2 第一项判据（重投影误差）能否达标的关键一步。
    std::vector<cv::Point3f> inlierObject;
    std::vector<cv::Point2f> inlierImage;
    inlierObject.reserve(inliers.size());
    inlierImage.reserve(inliers.size());
    for (int idx : inliers)
    {
        inlierObject.push_back(objectPoints[static_cast<size_t>(idx)]);
        inlierImage.push_back(imagePoints[static_cast<size_t>(idx)]);
    }

    cv::Mat refinedRvec = rvec.clone();
    cv::Mat refinedTvec = tvec.clone();
    try
    {
        cv::solvePnP(inlierObject, inlierImage, calibration.cameraMatrix,
                     distortion, refinedRvec, refinedTvec,
                     true,                   // useExtrinsicGuess
                     cv::SOLVEPNP_ITERATIVE);
    }
    catch (const cv::Exception&)
    {
        // refine 失败不是致命错误：RANSAC 的解仍然可用（只是精度略低）。
        // 静默回退到 RANSAC 的解，而不是把一次成功的解算变成失败。
        refinedRvec = rvec;
        refinedTvec = tvec;
    }

    // ---- 组装 ----
    cv::Matx33d rotation;
    cv::Rodrigues(refinedRvec, rotation);

    out.aircraftToCamera.rotation = rotation;
    out.aircraftToCamera.translation = cv::Vec3d(refinedTvec.at<double>(0),
                                                 refinedTvec.at<double>(1),
                                                 refinedTvec.at<double>(2));
    out.reprojectionError = rmsReprojectionError(correspondences, inliers,
                                                 calibration.cameraMatrix,
                                                 distortion, refinedRvec,
                                                 refinedTvec);

    stats.inlierCount = static_cast<int>(inliers.size());
    stats.inlierRatio = static_cast<double>(inliers.size())
                      / static_cast<double>(stats.inputCount);
    stats.reprojectionError = out.reprojectionError;
    stats.converged = true;

    if (!std::isfinite(out.reprojectionError)
        || !std::isfinite(out.aircraftToCamera.translation(2)))
    {
        out = data::CameraPose{};
        stats = PnpStats{};
        stats.inputCount = static_cast<int>(correspondences.size());
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
//  MockPnPPoseEstimator
// ---------------------------------------------------------------------------

void MockPnPPoseEstimator::setPreset(const data::CameraPose& pose,
                                     double inlierRatio)
{
    preset_ = pose;
    inlierRatio_ = std::min(1.0, std::max(0.0, inlierRatio));
    hasPreset_ = true;
}

bool MockPnPPoseEstimator::estimate(
    const std::vector<data::FeatureCorrespondence>& correspondences,
    const data::CameraCalibration&,
    data::CameraPose& out,
    PnpStats& stats)
{
    out = data::CameraPose{};
    stats = PnpStats{};
    stats.inputCount = static_cast<int>(correspondences.size());

    if (forceFailure_ || !hasPreset_)
    {
        return false;
    }

    out = preset_;
    stats.inlierCount = static_cast<int>(
        std::lround(inlierRatio_ * static_cast<double>(stats.inputCount)));
    // 自洽性：内点数由比例换算而来，两者不可能矛盾（手工填一个
    // "比例 0.9、内点 1 个"的组合会让下游断言失败在原因不明的地方）。
    stats.inlierRatio = stats.inputCount > 0
        ? static_cast<double>(stats.inlierCount)
              / static_cast<double>(stats.inputCount)
        : 0.0;
    stats.reprojectionError = preset_.reprojectionError;
    stats.converged = true;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
