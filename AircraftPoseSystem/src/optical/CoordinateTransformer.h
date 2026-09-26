#pragma once

// ============================================================================
//  src/optical/CoordinateTransformer.h
//
//  依据：ENG-09 §2.1（坐标变换方向冻结）、§5.23 / §5.24、裁决 C-06
//        SYS-13 §5（坐标链）、§6（输出）
//        ENG-01 §6、ENG-02 §2.3 / ENG-04 §2.3（依赖注入）
//        5.md §九
//
//  作用：完成 Camera → Rig → Ship 的坐标合成，产出最终姿态。
//
//  冻结的坐标链（ENG-09 §2.1，方向语义 `<A>To<B>` = 把 A 系的坐标转到 B 系）：
//
//      p_rig     = cameraToRig       · p_camera
//      p_ship    = rigToShip         · p_rig
//      p_camera  = aircraftToCamera  · p_aircraft
//
//  合成（SYS-13 §5 的 T_ship^aircraft）：
//
//      aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
//
//  ⚠ 矩阵乘法**顺序不可交换**，且此处极易写反（写反不会报错，
//  只会让结果矩阵变成另一种变换）。记忆要点：链式相乘时，
//  **最靠近待变换点的变换写在最右边**。上式中 aircraftToCamera
//  作用于 p_aircraft，故在最右；rigToShip 作用于最后结果，故在最左。
//
//  ⚠⚠ 与工作流文档 5.md §九 的一处**接口缺陷纠正**：
//
//  5.md 的签名是：
//      data::ShipPoseResult transform(const data::Transform& cameraPose);
//
//  它**没有指明这个 cameraPose 来自哪台相机** —— 而三台相机的
//  cameraToRig 外参各不相同（cam25_to_rig / cam50_to_rig /
//  cam100_to_rig 是三个独立文件，ENG-01 §3.4）。
//  因此该签名无法完成自己声明要做的事：
//    · 要么它固定用某一台的外参（另两台的测量结果就都错了，
//      而错误量级恰好是基线长度对应的视差，看起来像"姿态有小偏差"）；
//    · 要么它假定调用方已把 cameraToRig 预乘进去了
//      （那本类就只剩一次 rigToShip 乘法，"坐标链合成"的职责落空）。
//
//  冻结为带 CameraRole 的签名：`transform(CameraRole, const CameraPose&)`。
//  同时改为接收 CameraPose 而非裸 Transform —— CameraPose 携带
//  reprojectionError（ENG-09 §5.23），而 ShipPoseResult 也要输出它
//  （§5.24）。只传 Transform 会迫使调用方另行传递误差，
//  两处参数迟早会传错位（例如把上一次的误差配到这一次的姿态上）。
//
//  ⚠ 与 5.md 的第二处偏离：**不**在本类保存 rigToShip_ 副本。
//
//  5.md 用 `data::Transform rigToShip_;` 成员保存外参。但 rigToShip 的
//  权威来源是 CalibrationManager（装载自 rig_to_ship.yaml）。
//  存副本意味着同一事实有两份，一旦标定重新装载而本副本未更新，
//  合成的姿态会继续使用**旧的**舰体坐标变换 ——
//  而这类错误在数值上表现为"姿态整体偏移了一个常量"，
//  恰好与真实的机械安装偏差无法区分。
//  故改为持有 CalibrationManager 的引用，按需查询（单一数据源）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "data/CameraPose.h"
#include "data/CameraRole.h"
#include "data/ShipPoseResult.h"
#include "data/Transform.h"
#include "optical/CalibrationManager.h"

namespace aircraft
{
namespace optical
{

/// 坐标链合成（SYS-13 §5）。
class CoordinateTransformer
{
public:
    /// @param calibration 标定数据源。本类只读，不持有副本
    ///        （见文件头对偏离第二处的说明）。
    ///        调用方须保证 calibration 的生命周期长于本对象 ——
    ///        在 application 层由 ApplicationContext 统一持有。
    explicit CoordinateTransformer(const CalibrationManager& calibration);

    /// 取某台相机的 Camera → Rig 外参。
    /// 等价于 calibration_.getCalibration(role).cameraToRig，
    /// 保留此入口是为了让坐标链的全部环节都能从本类一次看全
    /// （5.md §九 亦将其列为本类职责）。
    data::Transform cameraToRig(data::CameraRole role) const;

    /// 合成最终姿态：aircraftToShip = rigToShip · cameraToRig · aircraftToCamera。
    ///
    /// @param role        产生该姿态的相机角色（决定用哪份外参）。
    /// @param cameraPose  PnP 的输出（ENG-09 §5.23），
    ///                    其 aircraftToCamera 方向为 Aircraft → Camera。
    /// @return 成功时 success=true，yaw/pitch/roll 单位为 **deg**（§2.2），
    ///         欧拉角顺序 ZYX（Yaw → Pitch → Roll），
    ///         yaw 为绕 Z_ship 轴的旋转角。
    ///         输入退化（旋转矩阵奇异、含非有限值）时 success=false，
    ///         其余字段为默认值 —— 此时 M 状态机的 POSE_SOLVE
    ///         按 SYS-08 §7.3 重试。
    data::ShipPoseResult transform(data::CameraRole role,
                                   const data::CameraPose& cameraPose) const;

    /// 由旋转矩阵提取 ZYX 欧拉角，单位为 **deg**。
    ///
    /// 独立暴露的原因：它是本类唯一有实质算法内容的步骤，
    /// 且是 SYS-07 §12.2 验证指标 yaw ∈ [yawMin, yawMax] 的输入，
    /// 需要能被单独测试（tests/optical/）。
    ///
    /// @param r           旋转矩阵。
    /// @param yawDeg      输出，单位 deg，绕 Z 轴。
    /// @param pitchDeg    输出，单位 deg，绕 Y 轴，范围 [-90, 90]。
    /// @param rollDeg     输出，单位 deg，绕 X 轴。
    /// @return false 表示矩阵奇异或含非有限值（见 .cpp 的万向锁处理）。
    static bool decomposeZyx(const cv::Matx33d& r,
                             double& yawDeg,
                             double& pitchDeg,
                             double& rollDeg);

private:
    const CalibrationManager& calibration_;
};

}  // namespace optical
}  // namespace aircraft
