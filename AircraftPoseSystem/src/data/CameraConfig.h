#pragma once

// ============================================================================
//  src/data/CameraConfig.h
//
//  依据：ENG-09 §4.1、§6.1（类型定义冻结）、§2.2 / §2.3（单位）、裁决 C-17
//
//  裁决 C-17（职责划分，冻结）：
//      CameraConfig  = **运行时可调**参数（曝光、增益、触发模式）
//      CameraChannel = **静态描述 + 标定**（相机 ID、焦段角色、焦距、标定）
//
//  为什么必须分成两个类型（C-17 的理由，实现时不要合并）：
//  两者的**生命周期**根本不同。CameraConfig 从 cameras/*.yaml 读入后可在
//  运行中按场景调整（逆光时改曝光），且**不参与离线复现的判定**——
//  它只影响图像怎么拍出来的。CameraChannel 来自标定文件，一旦标定就固定，
//  且**直接决定测量结果是否正确**（焦距错了，角度就错了）。
//  若合并为一个类型，则"改曝光"与"改标定"会走同一个写回路径，
//  一次误操作就能把标定值覆盖掉，且没有任何机制能发现。
//
//  ⚠ cameraId 与 role 的关系：cameraId 是设备序列号（华睿 A7A20MU201 的
//  唯一标识），role 是焦段角色。两者**不是一一对应**的语义 ——
//  role 决定用哪份标定和哪份特征库，cameraId 决定打开哪个设备。
//  同一台物理相机换装到另一个焦段位置时，cameraId 不变而 role 变。
// ============================================================================

#include <string>

#include "data/CameraRole.h"

namespace aircraft
{
namespace data
{

/// 单台相机的运行时配置（ENG-09 §6.1）。
struct CameraConfig
{
    /// 设备序列号，与 CameraChannel::cameraId 对应。
    std::string cameraId;

    /// 焦段角色，决定标定与特征库的选择。
    CameraRole role = CameraRole::CAM25;

    /// 图像宽度，单位 pixel。必须与标定时的分辨率一致 ——
    /// 分辨率改变会直接平移 CameraCalibration::cameraMatrix 的主点，
    /// 而内参不会自动失效（见 CameraCalibration.h 的说明）。
    int width = 0;

    /// 图像高度，单位 pixel。
    int height = 0;

    /// 曝光时间，单位 **s**（ENG-09 §6.1 注释冻洁为 s，不是 ms 也不是 us）。
    /// 注意与 ImageFrame::exposureTime 同单位；与 ImageQuality::exposure
    /// 不同 —— 后者是 [0,1] 的合理性评分，不是时间。
    double exposureTime = 0.0;  // s

    /// 增益，单位 **dB**（ENG-09 §6.1 冻结）。
    double gain = 0.0;  // dB

    /// 触发模式：true = 硬触发，false = 软触发。
    /// 硬触发失效时按 SYS-08 §7.5 降级为软触发并置 kErrTriggerDegraded。
    bool triggerMode = true;
};

}  // namespace data
}  // namespace aircraft
