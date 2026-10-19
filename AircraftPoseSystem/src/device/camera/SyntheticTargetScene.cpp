// ============================================================================
//  src/device/camera/SyntheticTargetScene.cpp
//
//  依据：批次 B（最小模拟成功流程）
// ============================================================================

#include "device/camera/SyntheticTargetScene.h"

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "data/SyntheticTargetGeometry.h"

namespace aircraft
{
namespace device
{

namespace
{

constexpr double kPi    = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

const data::CameraCalibration& calibrationOf(
    const data::OpticalRigCalibration& rig, data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return rig.cam25;
    case data::CameraRole::CAM50:  return rig.cam50;
    case data::CameraRole::CAM100: return rig.cam100;
    }
    return rig.cam25;
}

}  // namespace

SyntheticTargetScene::SyntheticTargetScene(
    const data::OpticalRigCalibration& rig,
    data::CameraRole role,
    const data::Transform& truthAircraftToShip)
    : calib_(calibrationOf(rig, role))
    , truth_(truthAircraftToShip)
    , points_(data::synthetic::outline())
{
    // camera → ship 的完整变换：
    //     p_ship = R_rs·(R_cr·p_cam + t_cr) + t_rs
    //            = (R_rs·R_cr)·p_cam + (R_rs·t_cr + t_rs)
    const cv::Matx33d rCamToShip =
        rig.rigToShip.rotation * calib_.cameraToRig.rotation;
    const cv::Vec3d tCamToShip =
        rig.rigToShip.rotation * calib_.cameraToRig.translation
        + rig.rigToShip.translation;

    // 反解：aircraftToCamera = (cameraToShip)^-1 · aircraftToShip。
    // 旋转矩阵是正交阵，逆等于转置（不做一般矩阵求逆）。
    const cv::Matx33d rShipToCam = rCamToShip.t();
    aircraftToCamera_.rotation = rShipToCam * truth_.rotation;
    aircraftToCamera_.translation =
        rShipToCam * (truth_.translation - tCamToShip);

    valid_ = !calib_.cameraMatrix.empty()
          && calib_.cameraMatrix.type() == CV_64F
          && calib_.cameraMatrix.rows == 3
          && calib_.cameraMatrix.cols == 3
          && calib_.imageWidth > 0
          && calib_.imageHeight > 0
          && !points_.empty();
}

bool SyntheticTargetScene::valid() const
{
    return valid_;
}

bool SyntheticTargetScene::renderInto(cv::Mat& bgr) const
{
    if (!valid_ || bgr.empty() || bgr.type() != CV_8UC3)
    {
        return false;
    }

    // 1. 投影：用**真值**位姿，不用粗姿态。粗姿态是求解器的输入，
    //    若渲染也用它，验收就退化成"求解器能否复现自己的输入"。
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(aircraftToCamera_.rotation), rvec);
    const cv::Vec3d t = aircraftToCamera_.translation;
    const cv::Mat tvec =
        (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);

    std::vector<cv::Point2f> projected;
    cv::projectPoints(points_, rvec, tvec, calib_.cameraMatrix,
                      calib_.distortion, projected);
    if (projected.size() != points_.size())
    {
        return false;
    }

    std::vector<cv::Point2i> polygon;
    polygon.reserve(projected.size());
    for (const cv::Point2f& p : projected)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
        {
            return false;
        }
        // 允许少量越界（部分可见），不影响 CAD 窗口；但完全在画面外的
        // 点会让多边形退化。此处不裁剪：projectPoints 已给出真实位置，
        // fillPoly 自己会按图像边界裁剪。
        polygon.emplace_back(static_cast<int>(std::lround(p.x)),
                             static_cast<int>(std::lround(p.y)));
    }

    // 2. 绘制：单一灰阶的实心多边形。
    //
    // ⚠ **不描白边**。2~3 px 的白边会在 Canny 下形成两条相距数像素的
    //    平行强边缘，CadStructureLocator 的倍角聚类会把它们并成一组，
    //    拟合出的直线落在两边缘之间 —— 交点随之偏移，而残差判据仍通过。
    //    单一步进边缘（235 vs 背景 40~100）只有一条边缘线。
    std::vector<std::vector<cv::Point>> polys{polygon};
    cv::fillPoly(bgr, polys, cv::Scalar(235, 235, 235), cv::LINE_8);

    return true;
}

data::Transform SyntheticTargetScene::defaultTruthAircraftToShip()
{
    const double e = coarseElevationDeg() * kDegToRad;
    const double c = std::cos(e);
    const double s = std::sin(e);

    // 与 PosePipeline::solvePose 的粗姿态同一构造：
    //   Ry(90°) 把机头 +X_aircraft 指向 -Z_camera（迎头）。
    const cv::Matx33d ry90(0.0, 0.0, 1.0,
                           0.0, 1.0, 0.0,
                           -1.0, 0.0, 0.0);
    const cv::Matx33d rx(1.0, 0.0, 0.0,
                         0.0, c, s,
                         0.0, -s, c);

    // 真值在粗姿态上再绕飞机 Z 轴转 1.2°：模拟"机头朝向与迎头假设
    // 不完全一致"。求解器若把粗姿态回填，就会与真值差 1.2°，验收可分辨。
    const double yaw = 1.2 * kDegToRad;
    const double cy  = std::cos(yaw);
    const double sy  = std::sin(yaw);
    const cv::Matx33d rz(cy, -sy, 0.0,
                         sy,  cy, 0.0,
                         0.0, 0.0, 1.0);

    data::Transform truth;
    truth.rotation    = rx * ry90 * rz;
    truth.translation = cv::Vec3d(0.0, 0.0, coarseDistanceM());
    return truth;
}

double SyntheticTargetScene::coarseElevationDeg()
{
    return 5.0;
}

double SyntheticTargetScene::coarseDistanceM()
{
    return 40.0;
}

}  // namespace device
}  // namespace aircraft
