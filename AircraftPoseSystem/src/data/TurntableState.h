#pragma once

// ============================================================================
//  src/data/TurntableState.h
//
//  依据：ENG-09 §4.1、§5.9（类型定义冻结）、§2.2（角度单位）、裁决 C-04
//
//  裁决 C-04：本结构体**保留** TurntableState 名称，同名的枚举更名为
//  TurntableMotionState（见 TurntableMotionState.h 的裁决说明）。
//
//  两处变更（裁决 C-04）：
//  1. 原 SYS-05 §6.1 的 `bool moving` **删除**，由 `motion` 承担。
//     moving 与 `motion != IDLE && motion != STABLE` 完全等价，保留两者
//     会产生不一致状态（moving=true 而 motion=STABLE 无法被排除）。
//  2. `motion` 补充 ERROR 取值，使转台错误可随状态一起回传，
//     无需另开错误通道。
//
//  本结构体是 ITurntableController::state() 的返回类型（SYS-06 §10.1，
//  裁决 C-05 确认方法名为 state() 而非 getState()），
//  也是 StatusPanel / TurntablePanel 的显示数据源。
// ============================================================================

#include "data/TurntableMotionState.h"

namespace aircraft
{
namespace data
{

/// 两轴转台的当前状态。
struct TurntableState
{
    /// 方位角，单位 **deg**（ENG-09 §2.2：所有角度字段一律为度，非弧度）。
    ///
    /// 为什么冻结为度而不是弧度：验收指标以角分表述（Yaw ≤ 1 角分），
    /// 转台指标以度表述（±0.2°）。若内部用弧度，`if (yaw <= 1.0)` 这类
    /// 比较会极易把"1 弧度"当成"1 角分"（相差 3437 倍）。
    /// 需要弧度参与数学运算时在算法内部局部转换（deg * CV_PI / 180.0），
    /// 不得改变本字段单位。
    double azimuth = 0.0;    // deg

    /// 俯仰角，单位 **deg**。
    double elevation = 0.0;  // deg

    /// 运动状态。是判据"转台是否到位"的唯一依据 ——
    /// SYS-08 的 ALIGN / STABILIZE 两个状态都在等待它变为 STABLE。
    TurntableMotionState motion = TurntableMotionState::IDLE;
};

}  // namespace data
}  // namespace aircraft
