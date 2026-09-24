#pragma once

// ============================================================================
//  src/application/StateMachine.h
//
//  依据：SYS-08 §4（状态定义）、§5（各状态详细设计）、§6（正常流程）、
//        §7.4（回退预算）、§7.6（约束 1/2/4/8）、§7.7（恢复路径汇总）、
//        §11（状态机约束 1/4/7/8/9）
//        ENG-01 §9（application 冻结文件）、ENG-04 §10.4、ENG-02 §10.4
//        ENG-06 §5.2（StateMachine 测试：正常转换 / 非法转换 / 异常恢复）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 状态机**只做状态转换**，不做重试决策（ENG-04 §10.5）。               │
//  │ 每次进入某状态 → 向 RetryManager 上报一次尝试（§7.6 约束 2）；        │
//  │ 每次向后转换  → 先经 beginRollback() 批准（§7.6 约束 1 / §11 约束 8）。│
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 本类**不保存任何重试计数**（§11 约束 6）。若在此处出现
//  `int retryCount_` 之类的成员，跨状态循环（§7.4）就不再受全局预算约束，
//  而"单次任务必然终止"（§11 约束 9）随之失效。
//
//  ⚠ 与 7.md §四 的偏离（两处，均为冻结文档要求，非风格选择）：
//
//  1. `transition()` **带 nowNs 参数**。7.md 写的是 `transition(next)`，
//     但 SYS-08 §7.6 的 `beginAttempt(state, nowNs)` 与
//     `beginRollback(from, to, nowNs)` 都要求时刻，而状态机必须在每次
//     进入状态/向后转换时调用它们（§7.6 约束 1/2）。时刻由调用方传入而非
//     状态机自行读时钟，理由同 RetryManager.h 的文件头：可测性与
//     "一次事件循环内判断基于同一时刻"。
//     冻结文档中**没有任何一处**规定 StateMachine 的方法签名
//     （ENG-01 §9 / ENG-02 §10.4 / ENG-04 §10.4 只列状态），故此处由本阶段定义。
//
//  2. `reset()` **不重置重试计数**。计数属于 RetryManager（§7.6 约束 5：
//     仅在任务结束时由其 `reset()` 清零）。两个 reset() 由
//     MeasurementController::startMeasurement() 一并调用 —— 若只调用其一，
//     会出现"状态回 IDLE 但次数已用尽"或反之的不一致。
//
//  ⚠ 合法转换表是 §6 的正常流程 ∪ §7.7 的恢复路径。**任何不在表内的转换
//  都被拒绝并置 lastError**，而不是像 7.md §五 那样 `transition()` 恒返回 true。
//  ENG-06 §5.2 明确要求测试"非法转换"——恒真的实现使该用例无法成立，
//  也使 UI 或算法层的一次误调用能直接把状态机推到终态。
// ============================================================================

#include <cstdint>

#include "application/RetryManager.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementState.h"

namespace aircraft
{
namespace application
{

/// 测量流程状态机（SYS-08 §4~§7）。
class StateMachine
{
public:
    /// @param retry 重试与时限的唯一持有者，**引用而非指针**：
    ///        状态机的每次转换都必须经它批准（§7.6 约束 1/2），
    ///        不存在"没有 RetryManager 的状态机"这一合法状态，
    ///        故用引用让该前提在类型上成立。生命周期由
    ///        MeasurementController 保证（它同时持有二者）。
    explicit StateMachine(RetryManager& retry);

    /// 回到 IDLE 并清空自身错误记录。
    ///
    /// @warning **不清零重试计数**。任务开始时请走
    ///          `RetryManager::beginTask()`（它内部会 reset 计数器），
    ///          或由 MeasurementController::startMeasurement() 统一处理。
    void reset();

    /// 执行一次状态转换。
    ///
    /// 依次检查：① 是否自转换；② 是否在合法转换表内；③ 向后转换是否获
    /// 回退预算批准；④ 目标状态的尝试次数是否还有余额。任一不通过即
    /// 返回 false 且状态不变。
    ///
    /// @param next  目标状态。
    /// @param nowNs 当前时刻（data::MonotonicClock::nowNs()）。
    /// @return 转换是否发生。false 时 lastError() 给出原因；
    ///         其中 ③④ 两类失败的原因直接取自 RetryManager::lastError()。
    bool transition(data::MeasurementState next, uint64_t nowNs);

    /// 当前状态。
    data::MeasurementState state() const;

    /// 最近一次被拒绝的转换原因。成功转换后清为 0。
    data::ErrorInfo lastError() const;

    // ---- 转换表的查询（供 MeasurementController / 测试使用）--------------

    /// from → to 是否在合法转换表内（§6 ∪ §7.7）。
    static bool isLegalTransition(data::MeasurementState from,
                                  data::MeasurementState to);

    /// from → to 是否为**向后**（回退）转换，即需要 §7.4 预算批准的那一类。
    ///
    /// 判据是 §6 正常流程中的**次序**（见 forwardRank），且：
    ///   · 进入 FAILED 不是回退（是终止）；
    ///   · 由 COMPLETE / FAILED 回到 IDLE 不是回退（是新任务的开始）；
    ///   · 自转换不是回退（是 §7.3 的同状态重试，由 beginAttempt 计数）。
    static bool isRollback(data::MeasurementState from,
                           data::MeasurementState to);

    /// 状态在 §6 正常流程中的次序（IDLE=0 … COMPLETE=10）。
    /// IDLE 之前的态与 FAILED 返回 -1。
    static int forwardRank(data::MeasurementState state);

    /// 是否为终止态（COMPLETE / FAILED）。
    ///
    /// ⚠ 本方法不是命名上的便利，而是 transition() 的一条**前置判据**：
    /// 终止态的进入不受重试计数与任务时限管辖（理由见 .cpp 步骤 ④）。
    static bool isTerminal(data::MeasurementState state);

private:
    RetryManager& retry_;

    data::MeasurementState current_;

    data::ErrorInfo lastError_;
};

}  // namespace application
}  // namespace aircraft
