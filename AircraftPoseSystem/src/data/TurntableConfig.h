#pragma once

// ============================================================================
//  src/data/TurntableConfig.h
//
//  依据：ENG-09 §4.1、§6.3（类型定义冻结）、§2.2（角度单位）
//        裁决 C-20、C-04
//
//  ⚠ 裁决 C-20（本文件最重要的一条）：
//  原 TurntableConfig 中的 `maxIterations`（对准最大迭代次数）**已删除**。
//
//  理由：它与 SYS-08 §7.3 的 ALIGN 最大尝试次数是**同一语义**，
//  保留两个名字意味着两处数值可能不一致（SYS-10 §14 与 ENG-09 各定义一份），
//  而一旦不一致，"对准到底能试几次"就没有唯一答案 —— 实际生效的取决于
//  哪个模块先读到哪个字段，且不会有任何报错。
//
//  冻结结论：唯一数据源是 **MeasurementConfig::maxAlignAttempts**（默认 8）。
//  AlignmentController 通过 RetryManager 查询剩余次数，
//  **不得自持计数器**（SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 1）。
//  本结构体中因此没有任何 attempts/iterations 类字段 —— 这不是遗漏。
//
//  ⚠ 角度单位一律 **deg**（ENG-09 §2.2），不是弧度。
//  在这里混淆的代价：若把 deg 当 rad 传入转台 SDK，
//  指令会被解读为 0~360 度范围内的极小角度，转台几乎不动，
//  而软件层面看不出任何异常（不会报错，只是对准永远不收敛，
//  最终耗尽 8 次 ALIGN 重试后报 kErrAlignRetryExhausted 2003 ——
//  故障现象指向算法，根因却在单位）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <string>

namespace aircraft
{
namespace data
{

/// 转台配置（ENG-09 §6.3）。
struct TurntableConfig
{
    /// 通信协议："sdk" | "pekod" | "rs485" | "network"。
    /// 第一阶段仅 "sdk"（VirtualTurntable）与占位的 "pekod"
    /// （PekoTurntableController，按 ENG-08 §11 不实现具体协议）可用。
    ///
    /// 冻结为 std::string 而非枚举：协议集合随工程实施阶段扩展
    /// （SYS-10 的现场协议由供货方确定），字符串可在不改冻结表的前提下
    /// 增加取值，代价是拼写错误要到运行时才暴露 ——
    /// 因此 ConfigManager 加载时必须做取值白名单校验（ENG-10 §5.3：
    /// 取值超范围时启动失败，而非取默认值）。
    std::string protocol;

    /// 方位角下限，单位 **deg**。
    double azimuthMin = 0.0;    // deg

    /// 方位角上限，单位 **deg**。
    double azimuthMax = 0.0;    // deg

    /// 俯仰角下限，单位 **deg**。
    double elevationMin = 0.0;  // deg

    /// 俯仰角上限，单位 **deg**。
    double elevationMax = 0.0;  // deg

    /// 粗调速度，单位 **deg/s**。用于大角度快速指向（SYS-10 §6）。
    double coarseSpeed = 0.0;   // deg/s

    /// 精调速度，单位 **deg/s**。用于对准末段的小步修正。
    /// 精调速度偏高会导致越过目标后反复来回（极限环），
    /// 表现为对准永不进入 stable，最终耗尽 ALIGN 重试。
    double fineSpeed = 0.0;     // deg/s

    /// 中心判据阈值，单位 **pixel**，默认 50（ENG-09 §6.3 冻结）。
    /// 判据形式：|TargetOffset::pixelX| <= centerThreshold
    ///       且 |TargetOffset::pixelY| <= centerThreshold。
    ///
    /// ⚠ 该判据的成立**完全依赖 TargetOffset 以图像中心为原点**
    /// （裁决 C-11）。若原点被误改为检测框中心，则 pixelX/pixelY
    /// 恒为 0，判据恒真，转台永不移动，而对准会被报告为成功 ——
    /// 见 TargetOffset.h 的详细说明。
    double centerThreshold = 50.0;  // pixel
};

}  // namespace data
}  // namespace aircraft
