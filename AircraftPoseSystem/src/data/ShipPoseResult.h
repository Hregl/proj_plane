#pragma once

// ============================================================================
//  src/data/ShipPoseResult.h
//
//  依据：ENG-09 §4.1、§5.24（类型定义冻结）、§2.1 / §2.2（方向与角度单位）
//        裁决 C-06
//
//  裁决 C-06：SYS-05 §11.2 把合成结果字段命名为 `shipToAircraft`
//  （ship→aircraft），而 SYS-13 §5 坐标链算出的方向是 **aircraft→ship**
//  （T_ship^aircraft = T_ship^rig · T_rig^camera · T_camera^aircraft）。
//  两者互为逆变换。冻结字段名为 **aircraftToShip**。
//
//  ⚠ 这是本系统的**最终输出类型**。它的 yaw/pitch/roll 就是验收指标
//  "Yaw 误差 ≤ 1 角分"的受检量。
//
//  ⚠ 与工作流文档的偏离（3.md §八 的 ShipPoseResult 含 `double confidence`
//      且字段名为 shipToAircraft）：
//  ENG-09 §5.24 冻结的字段中没有 confidence —— 置信度在
//  PoseValidationResult.confidence 中（§5.25），由 PoseValidator 产出。
//  两处都存会出现"结果自身声称的置信度"与"验证器判定的置信度"不一致的状态，
//  而 VALIDATE 状态的通过与否以后者为准（SYS-07 §12）。
//
//  角度定义（ENG-09 §5.24）：
//  yaw = Aircraft Frame 相对 Ship Frame 绕 **Z_ship** 轴的旋转角，
//  ZYX 欧拉角顺序，Yaw → Pitch → Roll。
//  单位一律为 **deg**（§2.2），不是弧度。
// ============================================================================

#include "data/Transform.h"

namespace aircraft
{
namespace data
{

/// 最终输出的舰体坐标姿态（SYS-13 §6）。
struct ShipPoseResult
{
    /// 解算是否成功。false 时其余字段无意义。
    bool success = false;

    /// 偏航角，单位 **deg**。绕 Z_ship 轴，Aircraft 相对 Ship。
    /// **这是 1 角分指标的受检量。**
    double yaw = 0.0;    // deg

    /// 俯仰角，单位 **deg**。
    double pitch = 0.0;  // deg

    /// 滚转角，单位 **deg**。
    double roll = 0.0;   // deg

    /// Aircraft → Ship 的完整刚体变换（方向语义见 ENG-09 §2.1）。
    /// 与 yaw/pitch/roll 是同一旋转的两种表达，两者必须自洽 ——
    /// 冗余存储在此处是有意的：欧拉角便于比对指标，矩阵便于后续坐标换算，
    /// 且二者可互为校验（若不自洽说明分解代码有误）。
    Transform aircraftToShip;

    /// 重投影误差，单位 pixel。**不是角度**，不能与 yaw 直接比较。
    double reprojectionError = 0.0;  // pixel
};

}  // namespace data
}  // namespace aircraft
