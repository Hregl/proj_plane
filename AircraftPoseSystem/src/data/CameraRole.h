#pragma once

// ============================================================================
//  src/data/CameraRole.h
//
//  依据：ENG-09 §4.1（类型清单）、§5（类型定义冻结）、裁决 C-07
//
//  归属说明（裁决 C-07，本项目最重要的一次归属修正）：
//  该枚举原规划在 src/optical/CameraRole.h（ENG-01 §6 / ENG-02 §5.2），
//  但 ImageFrame、DetectionResult、MeasurementCandidate 三个 **data 层**
//  结构体都含 CameraRole 成员。若枚举留在 optical，则 data 反过来依赖
//  optical，而 algorithm（按 ENG-03 §12.5 只依赖 data + OpenCV）拿不到该
//  枚举 —— data 与 algorithm 两个模块同时无法编译。
//  故下沉到 src/data/。同理下沉的还有 Transform / CoordinateFrame /
//  MeasurementState / DeviceState / TurntableMotionState。
//
//  ⚠ 与工作流文档的偏离（3.md §四 / 5.md §三 定义了 UNKNOWN 枚举值）：
//  ENG-09 §4.1 冻结的取值只有 CAM25 / CAM50 / CAM100 **三个**。
//  ENG-09 §1.1 规定"本文档与任何其他设计文档冲突时以本文档为准"，§8 规定
//  枚举值变更须先改 ENG-09。故此处**不增补 UNKNOWN**。
//  需要表达"无候选相机"时，用布尔返回值或计数器，不得引入哨兵枚举值——
//  哨兵值会被默认构造悄悄带进 ImageFrame / MeasurementCandidate，使
//  "这台相机不存在"与"这台相机是 25mm"在内存中不可区分。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 三焦段相机角色。光机刚体上固定装配，不可运行时改变。
enum class CameraRole
{
    CAM25,   ///< 25 mm 焦段：大视场，SEARCH 阶段使用
    CAM50,   ///< 50 mm 焦段：中距离测量
    CAM100   ///< 100 mm 焦段：远距离测量（40~300 m 的远端主力）
};

}  // namespace data
}  // namespace aircraft
