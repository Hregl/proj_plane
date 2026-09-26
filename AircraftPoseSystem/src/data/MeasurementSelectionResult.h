#pragma once

// ============================================================================
//  src/data/MeasurementSelectionResult.h
//
//  依据：ENG-09 §4.1、§5.17（类型定义冻结）
//
//  作用：MeasurementSelector 的输出 —— 选中哪台相机、得分多少。
//
//  ⚠ 本结构体**没有** valid / success 布尔量（ENG-09 §5.17 只冻结两个字段），
//  也**没有** CameraRole::UNKNOWN 可用（ENG-09 §4.1 只冻结三个角色）。
//  因此"候选集合为空、无法选择"这一情形**不能**靠返回值编码，
//  必须由调用方以布尔返回值表达：
//
//      bool MeasurementSelector::select(..., MeasurementSelectionResult& out);
//      // 返回 false = 无候选可选（MEASURE_SELECT 状态按 SYS-08 §7.3 重试，
//      //            最多 3 次；用尽后按 §7.7 进入 FAILED）
//
//  为什么不用哨兵值：若给 CameraRole 加 UNKNOWN，它会经由默认构造悄悄
//  出现在 ImageFrame.role / MeasurementCandidate.camera 等所有含该枚举的
//  结构体中，"这台相机不存在"与"这台相机是 25mm"在内存中不可区分
//  （见 CameraRole.h 的说明）。用返回值表达则编译器强制调用方处理失败分支。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "data/CameraRole.h"

namespace aircraft
{
namespace data
{

/// 测量通道选择结果（SYS-14 §8）。
struct MeasurementSelectionResult
{
    /// 被选中的相机角色。
    CameraRole selectedCamera = CameraRole::CAM25;

    /// 该候选的加权总分（MeasurementCandidate::score）。
    double score = 0.0;
};

}  // namespace data
}  // namespace aircraft
