#pragma once

// ============================================================================
//  src/data/MeasurementState.h
//
//  依据：ENG-09 §4.1、§5.8（类型定义冻结）、裁决 C-13
//        SYS-08 §4（状态定义）、§5（各状态详细设计）
//
//  裁决 C-13：SYS-05 §12 在 MeasurementTask 中使用了该类型，但定义写在
//  SYS-08 §4。冻结归属 src/data/MeasurementState.h —— 因为 MeasurementTask
//  是 data 层类型，而 data 不得依赖 application 层。
//  状态的**语义与转换规则**仍由 SYS-08 定义，本文件只管类型。
//
//  为什么枚举必须放在 data 而规则放在 application：
//  RetryManager（application 层）的接口以 MeasurementState 为键
//  （SYS-08 §7.6：beginAttempt(state, nowNs)）；而 Logger 与 Recorder
//  （infrastructure 层）要把它序列化进 log.txt / result.json。若枚举定义在
//  application，infrastructure 就必须反向依赖 application —— 违反
//  ENG-01 §18。这与 C-07 把 CameraRole 下沉 data 是同一类问题、
//  同一类解法：**跨层的类型必须落在依赖关系的最低公共层**（ENG-10 §5.1）。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 单次测量任务的状态（SYS-08 §4）。
///
/// 正常流程（SYS-08 §6）：
///     IDLE → SEARCH → TARGET_FOUND → ALIGN → STABILIZE → MEASURE_SELECT
///          → CAPTURE → POSE_SOLVE → VALIDATE → SAVE → COMPLETE
/// 异常出口：FAILED（终止态，携带 ErrorInfo）。
///
/// 终止态有两个：COMPLETE 与 FAILED。SYS-08 §11 约束 9 要求单次任务在
/// T_task（默认 60 s）内**必然**到达其中之一 —— 这一有界性由 RetryManager
/// 保证，不靠各状态自觉。
enum class MeasurementState
{
    IDLE,           ///< 空闲，等待启动
    SEARCH,         ///< 搜索目标（CAM25 大视场扫视），不设次数上限
    TARGET_FOUND,   ///< 已检出目标，计算 TargetOffset
    ALIGN,          ///< 视觉闭环对准（转台），上限 8 次
    STABILIZE,      ///< 等待转台稳定，连续稳定帧 N ≥ 3
    MEASURE_SELECT, ///< 测量通道选择（三焦段候选评分）
    CAPTURE,        ///< 采集测量帧（5~10 帧）
    POSE_SOLVE,     ///< PnP 姿态解算
    VALIDATE,       ///< 结果验证
    SAVE,           ///< 落盘测量结果包
    COMPLETE,       ///< 成功终止态
    FAILED          ///< 失败终止态（须携带 ErrorInfo，错误码取自 ENG-09 §5.27）
};

}  // namespace data
}  // namespace aircraft
