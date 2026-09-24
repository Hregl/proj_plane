#pragma once

// ============================================================================
//  src/application/MeasurementStrategy.h
//
//  依据：SYS-08 §3.3（职责：当前测量策略；候选相机；算法流程）、§6（正常流程）、
//        §7.2（失败三分类）、§7.3（升级规则 + 次数表）、§7.4（回退预算）、
//        §7.7（恢复路径汇总）、§11（约束 7/8）
//        ENG-02 §10.3 / ENG-04 §10.3 / SYS-03 §4.3（均为职责描述）
//        ENG-01 §9（application 冻结文件：本文件在列）
//
//  ⚠ **冻结文档没有给出本类的任何方法或成员**。ENG-01 §9 只列出文件名，
//  ENG-02 §10.3 仅"测量策略控制"，ENG-04 §10.3 仅"管理：测量流程；相机选择；
//  异常恢复"，SYS-03 §4.3 同义。7.md 给了一个 `nextState()`。
//  因此本文件的方法是**本阶段的最小可用定义**，其内容全部来自 SYS-08 §6/§7
//  的表格，不含任何自创策略。若后续需要增补，按 ENG-09 §8 走文档变更流程。
//
//  本类**不做记账、不读时钟、不持有计数**：
//    · 记账归 RetryManager（§7.6）；
//    · 时钟由调用方传入（同 RetryManager.h 的可测性理由）；
//    · 计数器一旦出现在此处，"重试上限只有一处定义"（§11 约束 6）即被破坏。
//  它只回答两类问题：**正常流程的下一步是哪个状态**，以及
//  **本次失败按 §7.2/§7.7 应该怎么走**。真正的决策执行者是 StateMachine。
//
//  ⚠ 与 MeasurementSelector 的分工（SYS-14 §6）：候选相机的**排序与评分**
//  属算法层（ENG-01 §10 `algorithm/selection/`）。本类只维护**可及集合**
//  （把已失败的相机排除掉，§7.4 / §5.9），不参与打分。因此
//  allowedCameras() 返回的是"允许参与选择的焦段"，其**顺序不代表优劣**。
// ============================================================================

#include <vector>

#include "data/CameraRole.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementState.h"

namespace aircraft
{
namespace application
{

/// 失败分类（SYS-08 §7.2，冻结）。三类的处理方式**完全不同**，
/// 把全部失败当一类处理是本项目原设计的根本缺陷（§7.2 的原文）。
enum class FailureKind
{
    TRANSIENT,   ///< 瞬态：换输入重试有成功可能（丢帧、内点不足、PnP 未收敛、暂时遮挡）
    HARDWARE,    ///< 硬件故障：重试不改变结果（相机断连、转台通信失败、触发无响应）
    CAPABILITY   ///< 能力边界：超出物理能力（超出转台行程、超出焦段覆盖、距离超范围）
};

/// 恢复动作（§7.7 的动作列）。
enum class RecoveryAction
{
    RETRY_IN_STATE,  ///< 状态不变，再试一次本状态
    ROLLBACK,        ///< 向后转换到 target（须经 RetryManager::beginRollback 批准）
    FAIL             ///< 进入 FAILED
};

/// 一次失败的处置方案。
struct Recovery
{
    RecoveryAction action = RecoveryAction::FAIL;

    /// action == ROLLBACK 时的目标状态；其余情况无意义。
    data::MeasurementState target = data::MeasurementState::FAILED;

    /// 建议的失败原因。**终止类动作（FAIL）应把它交给状态机**，
    /// 供 FAILED 状态记录（§5.12：错误状态 / 错误码 / 日志）。
    ///
    /// ⚠ `timestampNs` 由调用方补齐：本类不读时钟。若本类直接取当前时刻，
    /// 同一事件循环内的多个判断会落在不同时刻，§10 用例 7
    ///（"先到者生效"）就无从判定。
    data::ErrorInfo error;
};

/// §7.3 的升级规则：第 N 次尝试必须改变哪些输入。
///
/// 存在的理由（§7.3 原文）：**重试禁止重复完全相同的输入** ——
/// 确定性的算法用同样的输入重算必然得到同样的结果。
/// 换句话说不改变输入的重试不是重试，只是消耗预算。
struct Escalation
{
    /// 重新采集帧（新的时间点、新的帧集合）。第 1 次起恒为 true。
    bool refetchFrames = true;

    /// 换用**次优相机**（相对上一次使用的相机）。§7.3 第 2 次要求。
    bool switchCamera = false;

    /// 放宽算法内部阈值（RANSAC 迭代数、ratio 阈值）。§7.3 第 3 次要求。
    bool relaxThresholds = false;
};

/// 测量策略（SYS-08 §3.3）。
class MeasurementStrategy
{
public:
    MeasurementStrategy() = default;

    // ---- 正常流程 ---------------------------------------------------------

    /// §6 正常流程中的下一个状态。
    ///
    /// ⚠ 终止态（COMPLETE / FAILED）**返回自身**，不返回 FAILED。
    /// 7.md §七 的实现把所有 default 分支都返回 FAILED，于是
    /// `nextState(COMPLETE)` 得到 FAILED —— 一次已经成功的测量会被
    /// 状态机推成失败。终止判定应查 isTerminal()。
    data::MeasurementState nextState(data::MeasurementState current) const;

    /// 是否为终止态（COMPLETE / FAILED）。§11 约束 9 要求单次任务
    /// 在 T_task 内**必然**到达其中之一。
    bool isTerminal(data::MeasurementState state) const;

    // ---- 失败处置 ---------------------------------------------------------

    /// 一级决策：本次失败该重试、该回退、还是该立即失败（§7.2 + §7.7）。
    ///
    /// @param state       发生失败的状态。
    /// @param kind        失败分类（§7.2）。
    /// @param deviceError 设备层上报的原始错误（可为空）。**硬件故障与能力
    ///        边界的错误码来自设备层**（ENG-09 §5.27 的 1000/2000/3000 段
    ///        归设备所有），本类不重复定义这些码，只负责"不重试"这一决定。
    Recovery recoveryFor(data::MeasurementState state,
                         FailureKind kind,
                         const data::ErrorInfo& deviceError) const;

    /// 二级决策：本状态的尝试次数已用尽时怎么办（§7.7 的"超限后果"列）。
    ///
    /// 与 recoveryFor() 分开的理由：§7.3 决定"还能试几次"，§7.7 决定
    /// "试完了去哪"，两张表在文档里就是分开的，合并会把 ALIGN 的
    /// "8 次后 FAILED(2003)" 与 "瞬态失败先重试" 混成一个判断。
    Recovery onAttemptsExhausted(data::MeasurementState state) const;

    // ---- §7.3 升级规则 ----------------------------------------------------

    /// 下一次尝试（第 attempt 次，从 1 起算）必须改变的输入。
    ///
    /// 实现按 §7.3 表逐行转写，且按**累积**理解（表中第 2/3 行都以
    /// "重新采集 +" 开头）：
    ///   第 1 次：重采
    ///   第 2 次：重采 + 换次优相机
    ///   第 ≥3 次：重采 + 放宽阈值（相机沿用第 2 次的选择，不再切换）
    Escalation escalationFor(int attempt) const;

    // ---- 候选相机可及集合（§7.4 / §5.9）----------------------------------

    /// 排除一个已失败的相机。
    /// §5.9 原文："每次必须**排除上一次失败的相机**"；
    /// §7.4 给出该规则的数量依据：系统只有 3 台相机，每次回退排除一台，
    /// 第 3 次已无候选可选，故 VALIDATE 的回退上限取 2。
    void excludeCamera(data::CameraRole role);

    /// 该相机是否已被排除。
    bool isExcluded(data::CameraRole role) const;

    /// 当前允许参与测量选择的焦段。**顺序不代表优劣**（排序是
    /// MeasurementSelector 的职责，见文件头说明）。
    std::vector<data::CameraRole> allowedCameras() const;

    /// 清空排除集。**任务开始时必须调用**：排除集是任务级状态，
    /// 跨任务残留会让第二次测量在"少一台相机"的条件下开始。
    void resetExclusions();

private:
    /// 排除集合。用 vector 而非 set：只有 3 个元素，
    /// vector 的顺序确定、可复现（§11 约束 5：单次任务可完整回放），
    /// 且不必引入 <set> 的哈希/比较语义。
    std::vector<data::CameraRole> excluded_;
};

}  // namespace application
}  // namespace aircraft
