#pragma once

// ============================================================================
//  src/ui/MeasurementView.h
//
//  依据：ENG-01 §13（UI 职责：只显示，不判断）、ENG-04 §14
//        ENG-03 §12.8（libui 依赖 Qt + application + preview）
//        ENG-01 §18 / ENG-04 §15（**UI 不得访问设备 / SDK**）
//        SYS-04 §4.5（"UI 只与 MeasurementController 交互"）
//
//  作用：把界面需要的测量信息压成一个**纯数据**结构，由 app 每拍填好后
//  交给 MainWindow 显示。MainWindow 不认识 MeasurementController，
//  更不认识 IMultiCameraManager / ITurntableController。
//
//  ---- 为什么需要这一层（这是 009 装配时暴露的一处真实约束）----
//
//  最直接的做法是给 MainWindow 加一个
//      void setController(application::MeasurementController*);
//  或
//      void setTurntable(device::ITurntableController*);
//  两者都不成立：
//
//  · `ITurntableController` 在 device 层。ENG-03 §12.8 冻结的 libui 依赖是
//    `Qt + application + preview`，**没有 device**。一旦 ui 的头文件里出现
//    `device/...`，编译能过（数据层的包含路径是 PUBLIC 传递的），但依赖图
//    上就多了一条 ui → device，而 ENG-01 §17 冻结的链是
//    `ui → application → device` —— 多出的这条边让"UI 不得访问设备"
//    从一条**编译期强制**的规则退化成一句口头约定。
//
//  · 即便换成 `MeasurementController*`，ui 也只是少了一层间接：它会开始
//    追问 `controller->state()`、`poseResult()`、`degraded()`…… 这些调用
//    的**组合方式**（什么时候显示什么、降级时说什么）就是测量语义本身，
//    属于 app 的判断（ENG-01 §14）。放在 ui 里，则同一个判断在离线回放
//    工具（无 GUI）里无法复用。
//
//  故此处取 POD：ui 只负责"把给定的数字画出来"，app 负责"给什么数字"。
//  这也让 MainWindow 可以在没有控制器的情况下单测（只构造一个 struct）。
//
//  ⚠ `valid == false` 是本结构的**正常初始态**，不是错误：
//    008 阶段（以及测量尚未启动时）界面就该显示占位文字。把这种场合
//    也塞进一个默认构造的 MeasurementView 会让 interface 显示
//    "IDLE / 0.000°"，看起来像一次真实测量得到了零姿态 ——
//    与"还没测"完全不是一回事，故用一个显式标志区分。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <QString>

#include "data/CameraRole.h"
#include "data/MeasurementState.h"
#include "data/TurntableState.h"

namespace aircraft
{
namespace ui
{

/// 界面所需的测量信息快照（每次刷新构造一次，成本 = 几个 double）。
struct MeasurementView
{
    /// 是否有一份有效的测量信息可显示。
    /// false 时 MainWindow 把各面板置为"无数据"占位（见文件头说明）。
    bool valid = false;

    // ---- 状态机 ----

    data::MeasurementState state = data::MeasurementState::IDLE;

    /// §7.5 的降级标记。true 时界面**必须**可见（SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 原文要求）。
    bool degraded = false;

    /// 当前可用相机数（3 / 2 / 1）。用于解释"为什么这次测量不准"。
    int availableCameras = 0;

    /// MEASURE_SELECT 选出的测量焦段。
    data::CameraRole selectedCamera = data::CameraRole::CAM25;

    // ---- 姿态 ----

    /// 是否已有姿态结果（POSE_SOLVE 之后）。
    bool hasPose = false;
    double yaw   = 0.0;   // deg（ENG-09 §2.2）
    double pitch = 0.0;   // deg
    double roll  = 0.0;   // deg

    /// 重投影误差，单位 pixel（不是角度，不能与 yaw 直接比较）。
    double reprojectionError = 0.0;

    // ---- 验证 ----

    bool hasValidation = false;
    double confidence  = 0.0;   // [0,1]
    double inlierRatio = 0.0;   // [0,1]

    // ---- 转台 ----

    bool                 hasTurntable = false;
    data::TurntableState turntable;

    // ---- 单行文字 ----

    /// 状态说明 / 降级原因 / 失败原因。空串表示无需额外提示。
    /// 由 app 生成 —— 文字的措辞属于业务判断，不在 ui 层。
    QString message;

    /// 最近一次结果落盘的位置（成功时）或失败原因。空串表示尚未落盘。
    QString record;
};

}  // namespace ui
}  // namespace aircraft
