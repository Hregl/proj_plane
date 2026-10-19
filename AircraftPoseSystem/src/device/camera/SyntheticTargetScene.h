#pragma once

// ============================================================================
//  src/device/camera/SyntheticTargetScene.h
//
//  依据：批次 B（最小模拟成功流程）
//
//  作用：把 `data::synthetic::outline()` 的六个结构点，按**已知相机内参**
//  与**已知真值位姿**投影到图像上，画成实心多边形。
//
//  ⚠ 本类**不**向算法输出任何二维点表。它只产生一张图像；图像侧的检测与
//    CAD 结构点定位必须从这张图里自己找。真值位姿只用于两处：
//      1. 渲染投影；
//      2. 验收比较（由调用方读回结果包后自行比较，不进入求解器）。
//
//  ⚠ 与 `VirtualCameraBackend` 的关系：场景是**可选叠层**。未设置场景时，
//    虚拟相机的既有行为（渐变背景 + 矩形目标）完全不变。
// ============================================================================

#include <vector>

#include <opencv2/core.hpp>

#include "data/CameraRole.h"
#include "data/OpticalRigCalibration.h"
#include "data/Transform.h"

namespace aircraft
{
namespace device
{

/// 合成目标的投影渲染器（Aircraft → Camera → 像素）。
class SyntheticTargetScene
{
public:
    /// @param rig   光机标定（提供该相机内参与 cameraToRig / rigToShip）。
    /// @param role  该场景用于哪一路相机。
    /// @param truthAircraftToShip 目标在舰体坐标系中的真值位姿。
    ///
    /// 构造时按 ENG-09 §2.1 的链方向反解出 aircraftToCamera：
    ///     aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
    /// 故同一份舰体系真值可以同时给三路相机各建一个场景，而三路看到的
    /// 是同一个物理目标。
    SyntheticTargetScene(const data::OpticalRigCalibration& rig,
                         data::CameraRole role,
                         const data::Transform& truthAircraftToShip);

    /// 内参与目标点是否足以投影。
    bool valid() const;

    /// 把目标叠画到 `bgr`（CV_8UC3）上。调用方负责先画背景。
    /// 图像类型不符时不做任何事（返回 false），由调用方决定如何报错。
    bool renderInto(cv::Mat& bgr) const;

    /// 构造时注入的真值（只读；**不得**传入求解器）。
    const data::Transform& truthAircraftToShip() const { return truth_; }

    /// 本场景使用的 aircraftToCamera（由真值与标定反解，渲染用）。
    const data::Transform& aircraftToCamera() const { return aircraftToCamera_; }

    /// 默认合成真值：R = Rx(-5°)·Ry(90°)·Rz(1.2°)，t = (0, 0, 40 m)。
    ///
    /// 选这一组数的理由：
    ///  · Ry(90°) 与 PosePipeline 的**粗姿态假设**同向（迎头），
    ///    故 CAD 结构点投影能落在渲染出的棱线附近；
    ///  · Rz(1.2°) 让真值**不等于**粗姿态，证明求解器确实在解算，
    ///    而不是把粗姿态原样回填；
    ///  · 原点投影到主点，检测框中心与图像中心重合，
    ///    ALIGN 不需要转台运动即可进入 STABILIZE（虚拟转台不会改变场景）。
    static data::Transform defaultTruthAircraftToShip();

    /// 与默认真值配套的粗姿态参数（供 `PosePipeline::setCoarseAttitude`）。
    static double coarseElevationDeg();
    static double coarseDistanceM();

private:
    data::CameraCalibration calib_;
    data::Transform          truth_;
    data::Transform          aircraftToCamera_;
    std::vector<cv::Point3f> points_;
    bool                     valid_ = false;
};

}  // namespace device
}  // namespace aircraft
