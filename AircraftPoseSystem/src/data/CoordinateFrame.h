#pragma once

// ============================================================================
//  src/data/CoordinateFrame.h
//
//  依据：ENG-09 §4.1（类型清单）
//
//  用途：显式标注一个量"在哪个坐标系中表达"。ENG-09 §2.1 冻结了变换命名
//  方向（AToB 表示"把在 A 系中表达的坐标转换到 B 系"），但变换的**起点**
//  仍需一个可传递的标识——SYS-13 的坐标链有四系，日志与 result.json 里
//  若只写数值不写坐标系，事后无法判断该数值属于哪一系。
//
//  裁决 C-19：原 SYS-05 §3.3 的 ShipFrame 类型已删除（与
//  OpticalRigCalibration.rigToShip 语义重复）。坐标系标识由本枚举承担，
//  外参由 CalibrationManager 承担，两者不再各有一套。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 坐标链的四系（SYS-13）。ENG-09 §2.1 冻结的链方向：
///     p_rig    = cameraToRig       · p_camera
///     p_ship   = rigToShip         · p_rig
///     p_camera = aircraftToCamera  · p_aircraft
enum class CoordinateFrame
{
    CAMERA,       ///< 单相机坐标系（原点在光心，Z 轴沿光轴前向）
    OPTICAL_RIG,  ///< 光机刚体坐标系（三相机共同的机械基准）
    SHIP,         ///< 舰体坐标系（测量的目标参考系，最终输出所在系）
    AIRCRAFT      ///< 飞机坐标系（CAD 模型与目标模型三维点所在系）
};

}  // namespace data
}  // namespace aircraft
