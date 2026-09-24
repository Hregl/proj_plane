#pragma once

// ============================================================================
//  src/data/OpticalRigCalibration.h
//
//  依据：ENG-09 §4.1、§5.3（类型定义冻结）
//
//  作用：光机刚体标定的**完整产出**。SYS-11 的标定流程输出就是本结构体，
//  由 CalibrationManager 装载到 OpticalRig 中。
//
//  为什么三台相机的外参集中在一个结构体、而不是各自一份文件：
//  SYS-11 的标定是**联合**标定 —— 三台相机同时观测同一标靶，解出的
//  cameraToRig 以同一个光机刚体基准为参照。分开装载无法保证三者来自
//  同一次标定，而"三相机外参是否同源"直接决定多焦段测量的自洽性：
//  若 cam50 用的是第 5 次标定、cam100 用的是第 7 次，两者的
//  绝对位姿差会被错误地当作目标姿态变化。
//  故 rigToShip 也放在同一结构体内，使"标定版本"成为不可分割的原子单元，
//  对应 ENG-09 §6.1 的 OpticalRigConfig.calibrationId。
// ============================================================================

#include "data/CameraCalibration.h"

namespace aircraft
{
namespace data
{

/// 光机刚体标定数据（SYS-11 产出）。
struct OpticalRigCalibration
{
    CameraCalibration cam25;
    CameraCalibration cam50;
    CameraCalibration cam100;

    /// OpticalRig → Ship。方向语义见 ENG-09 §2.1（本字段方向经核对正确）。
    ///
    /// SYS-15 §4 的误差预算中，"舰体坐标标定"占 0.20 角分，是第二大项，
    /// 且属于**系统误差**（线性相加）。它的物理来源就是本字段 —— 光机
    /// 刚体安装到舰体时，机械基准与舰体坐标系之间的指向偏差。
    Transform rigToShip;
};

}  // namespace data
}  // namespace aircraft
