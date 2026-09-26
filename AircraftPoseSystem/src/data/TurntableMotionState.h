#pragma once

// ============================================================================
//  src/data/TurntableMotionState.h
//
//  依据：ENG-09 §4.1、§5.9（类型定义冻结）、裁决 C-04
//
//  裁决 C-04（原文档最严重的命名冲突）：
//  TurntableState 在同一版本中被定义为两个互不相容的类型：
//    · SYS-05 §6.1：struct TurntableState { azimuth; elevation; moving; }
//    · SYS-10 §10 ：enum class TurntableState { IDLE; MOVING; STABLE; ERROR; }
//  而 ITurntableController::state() 的返回类型正是它（SYS-06 §10.1），
//  两者不可能同名。
//
//  冻结方案：**结构体保留 TurntableState 名称**（它是跨模块传递的数据），
//  **枚举更名为 TurntableMotionState**，并补充 ERROR 取值 ——
//  使转台错误可随状态一起回传，无需另开错误通道。
//
//  为什么结构体保名、枚举让名：
//  跨模块接口（ITurntableController::state()）的返回类型是结构体，改名会
//  波及 ui / application / 持久化三处；枚举只在转台内部与其消费者做判断，
//  改名波及面小。冲突裁决优先动波及面小的一方。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 转台的运动状态（SYS-10 §10）。
///
/// 与 TurntableState.motion 配合使用：结构体携带"在哪"，
/// 本枚举携带"在动还是在停"。裁决 C-04 删除了原结构体中的 bool moving ——
/// 它与 `motion != IDLE && motion != STABLE` 完全等价，保留两者会产生
/// 不一致状态（moving=true 而 motion=STABLE 这类组合无法被排除）。
enum class TurntableMotionState
{
    IDLE,    ///< 静止且已到位
    MOVING,  ///< 运动进行中
    STABLE,  ///< 已到达指令位置并稳定（ALIGN / STABILIZE 状态等待的即为此）
    ERROR    ///< 转台故障（通信失败、超行程）。按 SYS-08 §7.5 直接 FAILED，不重试
};

}  // namespace data
}  // namespace aircraft
