#pragma once

// ============================================================================
//  src/data/TargetOffset.h
//
//  依据：ENG-09 §4.1、§5.11（类型定义冻结）、§2.4（像素坐标）、裁决 C-11
//
//  裁决 C-11：SYS-05 §6.3 与 SYS-10 §7 的字段一致，但**均未定义参考原点**。
//  冻结为"**相对图像中心**的带符号像素偏差"。
//
//  为什么参考原点必须冻结（这是 C-11 的全部理由）：
//  SYS-10 §7 的对准停止判据是 `abs(pixelX) <= threshold`。该判据只有在
//  pixelX 是"相对图像中心"时才有物理意义 —— 转台要做的就是让目标回到
//  视场中心。若把 pixelX 理解为"相对目标框中心"，则它恒为 0，
//  判据恒成立，**转台一次都不会动，而对准会被判为成功**。
//
//  符号约定（ENG-09 §2.4）：X 向右为正，Y 向下为正，与 OpenCV 图像坐标一致。
//  注意"Y 向下为正"是图像坐标系的约定，与数学坐标系相反；
//  AlignmentController 在把像素偏差换算为转台角度时**必须显式处理 Y 的符号**，
//  否则俯仰会朝错误方向运动，且由于闭环的存在表现为"发散"而不是"不动"。
//
//  单位：pixel，double 类型（允许亚像素）。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 目标中心相对**图像中心**的像素偏差（ENG-09 §2.4）。
struct TargetOffset
{
    /// 横向偏差，单位 pixel。右为正，相对图像中心。
    double pixelX = 0.0;

    /// 纵向偏差，单位 pixel。下为正，相对图像中心。
    double pixelY = 0.0;

    /// 是否已居中。由对齐判据计算：
    ///     abs(pixelX) <= threshold && abs(pixelY) <= threshold
    /// threshold 来自 TurntableConfig::centerThreshold，默认 50 pixel
    ///（ENG-09 §5.11 / §6.3）。本布尔量是对该判据的**记录**，
    /// 判定逻辑在 AlignmentController，不在此结构体内（data 不含业务逻辑）。
    bool centered = false;
};

}  // namespace data
}  // namespace aircraft
