#pragma once

// ============================================================================
//  src/data/CameraPose.h
//
//  依据：ENG-09 §4.1、§5.23（类型定义冻结）、§2.1（变换方向）、裁决 C-06
//
//  裁决 C-06：SYS-05 §11.1 把 PnP 的直接输出命名为 `cameraToAircraft`
//  （camera→aircraft），与 SYS-13 §5 的坐标链方向相反。
//  本表裁决保留 SYS-13 的链方向，**修正字段名为 aircraftToCamera**。
//
//  方向核对（这是最容易出错的一处，实现前必须确认）：
//  PnP 的输入是"目标上的三维点（Aircraft Frame，即飞机本体坐标）"与
//  "这些点在图像上的像素坐标（Camera Frame）"。解算得到的是**要把飞机
//  坐标系的点变换到相机坐标系所需的刚体变换**，即
//      p_camera = aircraftToCamera · p_aircraft
//  因此 PnP 的输出天然就是 aircraftToCamera，而不是它的逆。
//  SYS-05 的原命名把它写反了 —— 而写反**不会有编译错误**，
//  只会让结果矩阵被转置，最终 Yaw 的正负号与量值同时错误。
//
//  与 CoordinateTransformer 的关系：本类型是 PnP 的单点产出，不参与
//  坐标链合成。CoordinateTransformer 消费它，按 ENG-09 §2.1 的链方向
//  合成出 aircraftToShip：
//      aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
// ============================================================================

#include "data/Transform.h"

namespace aircraft
{
namespace data
{

/// 单次 PnP 的姿态解算结果（SYS-07 §10）。
struct CameraPose
{
    /// Aircraft → Camera 的刚体变换。方向语义见 ENG-09 §2.1（文件头详解）。
    Transform aircraftToCamera;

    /// 重投影误差，单位 pixel。作为 PnP 是否收敛的判据，
    /// 也是 SYS-07 §12.2 三项验证指标之一。
    double reprojectionError = 0.0;  // pixel
};

}  // namespace data
}  // namespace aircraft
