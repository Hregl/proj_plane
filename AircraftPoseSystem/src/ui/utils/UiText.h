#pragma once

// ============================================================================
//  src/ui/utils/UiText.h
//
//  依据：ENG-09 §4.1（MeasurementState / CameraRole / TurntableMotionState
//        三者的取值冻结）、§2.2（角度单位）
//        9.md §六 / §七（PosePanel / StatusPanel 的显示项）
//        ENG-01 §13、ENG-04 §14（TurntablePanel 显示"方位 / 俯仰 / 状态"）
//
//  作用：枚举 → 界面文字的**唯一**转换点，以及角度的统一格式化。
//
//  为什么单独开一个文件（而不是各 panel 内各写一份 switch）：
//  application 层的 StateMachine.cpp / RetryManager.cpp / MeasurementStrategy.cpp
//  三处各留了一份 `stateName()`，理由是"目前只有那几个 .cpp 需要它，
//  而在 data 层新增 measurementStateName() 属 ENG-01 §5 清单变更"。
//  同样的理由在本层不成立：ui 是**同一层内、同一时刻**需要这些名字的
//  三个控件（StatusPanel 要状态与相机、TurntablePanel 要运动状态），
//  且 9.md 的 ui 结构里本就有一个 utils/ 目录（QtImageConverter 就在那里）。
//  故放在 utils/ 下，不涉及任何冻结清单的变更。
//
//  ⚠ 两张表必须与 application 层的同名函数**逐字一致**
//  （"IDLE" / "CAM25" / "STABILIZE"…）。理由：操作者看到的界面文字与
//  现场排障时读的 log.txt 是同一批字符串，两者一旦分叉，
//  "界面上显示的那个状态"就无法在日志里检索到。
//
//  ⚠ 全部为 inline，不新增编译单元（同 data/MonotonicClock.h 的做法）。
//
//  本文件不依赖 Qt，也不依赖 OpenCV —— 只有字符串与枚举。
// ============================================================================

#include <QString>

#include "data/CameraRole.h"
#include "data/MeasurementState.h"
#include "data/TurntableMotionState.h"

namespace aircraft
{
namespace ui
{

/// 测量状态名（与 application/StateMachine.cpp 的 stateName() 逐字一致）。
///
/// 兜底值 "UNKNOWN_STATE" 在当前枚举取值下不可达，但保留：
/// 界面是本系统**唯一**会把非法值直接呈现给人的地方，若把兜底写成空串，
/// 将来枚举扩容而漏改此处时，操作者看到的是一片空白而不是"有个不认识的值"。
inline QString stateText(data::MeasurementState state)
{
    switch (state)
    {
    case data::MeasurementState::IDLE:           return QStringLiteral("IDLE");
    case data::MeasurementState::SEARCH:         return QStringLiteral("SEARCH");
    case data::MeasurementState::TARGET_FOUND:   return QStringLiteral("TARGET_FOUND");
    case data::MeasurementState::ALIGN:          return QStringLiteral("ALIGN");
    case data::MeasurementState::STABILIZE:      return QStringLiteral("STABILIZE");
    case data::MeasurementState::MEASURE_SELECT: return QStringLiteral("MEASURE_SELECT");
    case data::MeasurementState::CAPTURE:        return QStringLiteral("CAPTURE");
    case data::MeasurementState::POSE_SOLVE:     return QStringLiteral("POSE_SOLVE");
    case data::MeasurementState::VALIDATE:       return QStringLiteral("VALIDATE");
    case data::MeasurementState::SAVE:           return QStringLiteral("SAVE");
    case data::MeasurementState::COMPLETE:       return QStringLiteral("COMPLETE");
    case data::MeasurementState::FAILED:         return QStringLiteral("FAILED");
    }
    return QStringLiteral("UNKNOWN_STATE");
}

/// 焦段名（与 application/MeasurementController.cpp 的 roleName() 逐字一致）。
inline QString roleText(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return QStringLiteral("CAM25");
    case data::CameraRole::CAM50:  return QStringLiteral("CAM50");
    case data::CameraRole::CAM100: return QStringLiteral("CAM100");
    }
    return QStringLiteral("UNKNOWN_ROLE");
}

/// 转台运动状态名（SYS-10 §10 的取值，见 data/TurntableMotionState.h）。
inline QString motionText(data::TurntableMotionState motion)
{
    switch (motion)
    {
    case data::TurntableMotionState::IDLE:   return QStringLiteral("IDLE");
    case data::TurntableMotionState::MOVING: return QStringLiteral("MOVING");
    case data::TurntableMotionState::STABLE: return QStringLiteral("STABLE");
    case data::TurntableMotionState::ERROR:  return QStringLiteral("ERROR");
    }
    return QStringLiteral("UNKNOWN_MOTION");
}

/// 角度显示：固定 3 位小数 + 度符号。
///
/// 3 位小数的选择依据：本系统的转台指标是 ±0.2°、验收指标是 1 角分
/// = 0.0167°，故 0.001° 的显示分辨率足以看出"是否已进指标区"，
/// 又不会引入视觉噪声。**不要**为了"看起来精确"加到 6 位：
/// 1 角分的真实含义在 0.001° 这一档已经能读出来，多余的位数只会
/// 让人误以为该位有效。
inline QString angleText(double degrees)
{
    return QString::number(degrees, 'f', 3) + QStringLiteral("°");
}

/// 角度显示：同时给出角分。
///
/// 用途只有一处：**Yaw**。1 角分是验收指标（README 首段），而界面上若
/// 只显示度，操作者需要在心里把 0.0167° 与 1 角分对应起来——这一步换算
/// 在紧张状态下极易出错（把 0.1° 当成"接近 1 角分"，实际已是 6 倍超差）。
///
/// 换算 1° = 60′ 是 ENG-09 §2.2 两条单位冻结之间的定义关系，
/// 不是新增指标、也不是新的精度声明：两个数字是**同一个量**。
inline QString angleTextWithArcmin(double degrees)
{
    return angleText(degrees) + QStringLiteral(" (")
         + QString::number(degrees * 60.0, 'f', 2) + QStringLiteral("′)");
}

}  // namespace ui
}  // namespace aircraft
