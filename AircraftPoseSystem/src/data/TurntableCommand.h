#pragma once

// ============================================================================
//  src/data/TurntableCommand.h
//
//  依据：ENG-09 §4.1、§5.10（类型定义冻结）、§2.2（角度单位）
//
//  ⚠ 字段名是 azimuthCommand / elevationCommand，**不是** 3.md §九 与
//  7.md 的 `azimuth` / `elevation`。
//  ENG-09 §5.10 冻结带 Command 后缀的字段名，理由是与 TurntableState 的
//  `azimuth` / `elevation`（**当前实际角度**）区分：
//  命令值是"要求转台去哪"，状态值是"转台现在在哪"，二者在同一段代码中
//  频繁同时出现（AlignmentController 计算命令、随后读状态判断是否到位）。
//  同名会在 `cmd.azimuth = state.azimuth` 这类语句中完全失去可读性，
//  而把两者写反**不会有任何编译错误**——只是转台永远停在原地。
//
//  SYS-09 §14：控制命令采用**值传递**（不是 shared_ptr）。
//  理由：命令是纯值、无共享像素缓冲，拷贝代价极小；而值语义使
//  TurntableCommandQueue 中的命令序列天然不可被发送方事后篡改。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 转台控制命令（SYS-10）。
///
/// 由 AlignmentController 依据 TargetOffset 计算，经 TurntableWorker 下发。
struct TurntableCommand
{
    /// 方位角指令，单位 **deg**（ENG-09 §2.2）。
    double azimuthCommand = 0.0;    // deg

    /// 俯仰角指令，单位 **deg**。
    double elevationCommand = 0.0;  // deg
};

}  // namespace data
}  // namespace aircraft
