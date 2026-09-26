#pragma once

// ============================================================================
//  src/data/AlignmentResult.h
//
//  依据：ENG-09 §4.1、§5.13（类型定义冻结）
//        SYS-08 §5.4（ALIGN 状态）、SYS-10 §7（对准判据）
//
//  作用：一次对准**迭代**的结果记录。注意它记录的是一次迭代，
//  不是整个 ALIGN 状态 —— ALIGN 最多尝试 8 次（SYS-08 §7.3），
//  每次迭代产出一个本结构体。
//
//  ⚠ iterations 与 SYS-08 §7.3 的"最大尝试次数 8"不是同一个计数器：
//  本字段由 AlignmentController 记录单次闭环内的迭代步数（视觉伺服通常
//  2~3 步收敛），而 8 次的尝试上限由 RetryManager 持有。
//  SYS-08 §11 约束 6 / ENG-09 §10 约束 8 明确：**重试计数只能由 RetryManager
//  持有**，AlignmentController 不得自持计数器（裁决 C-20 删除了
//  TurntableConfig::maxIterations 正是为了让唯一数据源成立）。
//  故本字段是**诊断量**，不得用于任何"是否继续重试"的判断。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "data/TargetOffset.h"

namespace aircraft
{
namespace data
{

/// 一次对准迭代的结果（SYS-10 §7）。
struct AlignmentResult
{
    /// 本次迭代是否达成对准（offset.centered 为真，且运动已执行）。
    bool success = false;

    /// 本次迭代结束时的目标像素偏差。
    TargetOffset offset;

    /// 本次迭代内的闭环步数（诊断用，见文件头说明，不参与重试决策）。
    int iterations = 0;

    /// 本次迭代结束时的残余误差，单位 pixel。
    /// 取 max(abs(offset.pixelX), abs(offset.pixelY))，与 SYS-10 §7 的
    /// 停止判据同一量纲，便于与 TurntableConfig::centerThreshold 直接比对。
    double finalErrorPixel = 0.0;
};

}  // namespace data
}  // namespace aircraft
