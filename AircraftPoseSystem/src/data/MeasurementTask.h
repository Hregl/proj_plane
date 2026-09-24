#pragma once

// ============================================================================
//  src/data/MeasurementTask.h
//
//  依据：ENG-09 §4.1、§5.26（类型定义冻结）、裁决 C-13
//
//  作用：一次测量任务的完整记录，是 SAVE 状态的落盘对象，也是离线回放的
//  入口。RecorderWorker 产出的 measurement_xxx/ 目录与本结构体一一对应
//  （SYS-09 §12 的 result.json 即本结构体的序列化）。
//
//  ⚠ taskId 的用途不止是标识：ENG-09 §6.8 要求
//  measurement_xxx/config_snapshot/ 保存本次生效的全部 yaml，
//  目录名由 taskId 决定。配置快照与结果包同名同目录，才能保证
//  "拿到结果包就能复现判定"，而不会出现"包在但不知道当时用的哪份阈值"。
//
//  ⚠ 裁决 C-13 记录了归属决策：MeasurementState 定义在 SYS-08 §4，
//  但归属冻结为 src/data/ —— 因为本结构体在 data 层，而 data 不得依赖
//  application 层。同理本结构体引用的 ShipPoseResult / PoseValidationResult
//  也在 data 层，三者同层无循环。
// ============================================================================

#include <string>

#include "data/MeasurementState.h"
#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"

namespace aircraft
{
namespace data
{

/// 一次测量任务的记录（SYS-05 §12）。
struct MeasurementTask
{
    /// 任务标识。同时是结果目录名 measurement_xxx/ 的 xxx，
    /// 与 config_snapshot/ 的宿主（ENG-09 §6.8）。
    std::string taskId;

    /// 任务当前状态（SYS-08 §4 的 12 状态机）。
    MeasurementState state = MeasurementState::IDLE;

    /// 姿态结果（最终输出）。
    ShipPoseResult result;

    /// 验证结论。决定 VALIDATE 状态能否前进到 SAVE。
    PoseValidationResult validation;
};

}  // namespace data
}  // namespace aircraft
