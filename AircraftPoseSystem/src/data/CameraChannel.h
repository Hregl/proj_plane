#pragma once

// ============================================================================
//  src/data/CameraChannel.h
//
//  依据：ENG-09 §4.1、§5.4（类型定义冻结）、§2.3（长度单位）、
//        裁决 C-03、C-07、C-17
//
//  归属说明（裁决 C-07）：原规划在 src/optical/CameraChannel.h（ENG-01 §6），
//  随 CameraRole 一同下沉到 src/data/。理由：OpticalRig 的通道注册表是
//  data 层可描述的值类型，且 ui 层要读焦段角色做显示、algorithm 层要读
//  焦距做尺度估计 —— 若留在 optical，algorithm 不可达（ENG-03 §12.5
//  只允许 algorithm 依赖 data + OpenCV）。
//
//  裁决 C-03：ENG-02 §5.3 缺 enabled，**以 SYS-05 §4.4 为准，保留**。
//  三台相机支持单独禁用，是 SYS-08 §7.5 硬件降级的实现基础：
//  "2 台可用 → 降级继续，禁用故障相机，result.json 记录 degraded=true"。
//  若没有 enabled 字段，降级只能靠从 vector 中删除元素来表达，
//  这会使通道索引与数组下标失联，标定数据与通道的对应关系随之丢失。
//
//  裁决 C-17：CameraConfig 与 CameraChannel 字段曾被认为重叠，冻结为
//  拆分职责、两者均保留：
//    CameraConfig  = **运行时可调**参数（曝光、增益、触发模式）
//    CameraChannel = **静态描述 + 标定**（相机 ID、焦段角色、焦距、标定）
//  换句话说本结构体在一次测量任务内**不变**，CameraConfig 可以变
//  （但仍受 ENG-09 §6.8 的任务级快照约束）。
// ============================================================================

#include <string>

#include "data/CameraCalibration.h"
#include "data/CameraRole.h"

namespace aircraft
{
namespace data
{

/// 一个完整光学通道：相机 + 镜头 + 标定 + 角色。
struct CameraChannel
{
    /// 相机标识（序列号或配置中给定的名字），用于与 SDK 枚举到的设备对应。
    std::string cameraId;

    /// 焦段角色。光机刚体上固定，不随测量过程改变。
    CameraRole role = CameraRole::CAM25;

    /// 焦距，单位 **m**。
    ///
    /// ⚠ 最易出错的一处（ENG-09 §2.3 专门提示）：工程语言习惯说
    /// "100mm 镜头"，但本字段单位为米，故 **100mm 镜头应写作 0.1**。
    /// 写成 100.0 不会产生任何编译或运行错误，只会让 SYS-14 §10 的距离
    /// 估计式 Z = f·L/(l·s) 偏大 1000 倍，进而使焦段选择选错相机。
    double focalLength = 0.0;

    /// 该通道的内参 + 外参。与 OpticalRigCalibration 中同角色字段一致。
    CameraCalibration calibration;

    /// 是否启用。false = 该通道被禁用（调试或 SYS-08 §7.5 故障降级）。
    /// 禁用的通道不参与采集与测量选择，但**保留在注册表中**以维持
    /// 标定数据与通道角色的对应关系。
    bool enabled = true;
};

}  // namespace data
}  // namespace aircraft
