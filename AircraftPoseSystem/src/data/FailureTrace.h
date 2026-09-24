#pragma once

// ============================================================================
//  src/data/FailureTrace.h
//
//  依据：裁决 C-007（V2.1-C01_架构裁决变更说明.md §C-007，2026-09-23 已批准）
//        SYS-08 §7.3（重试）/ §7.4（回退预算）/ §7.7（失败三分类）
//
//  作用：失败任务的**根因 + 过程**。放进 MeasurementRecord，故**失败任务也留档**
//  （裁决 C-007：失败任务同样值得记录，且不带原图以省空间）。
//
//  ⚠ 核心区分：firstError 是根因，finalError 往往只是**症状**。
//  典型链路：POSE_SOLVE 因 5001（模型缺失）失败 → 换相机重试仍失败 → 回退预算
//  用尽 → 任务以 9002（回退预算用尽）终止。
//  此时 finalError = 9002 指向的是"重试策略"，而真正要修的是"模型没装"。
//  只报 finalError 会把现场引向错误方向 —— 这正是"**必须有 firstFailedState +
//  firstError**"的理由，而不是可选的便利字段。
//
//  ⚠ 关于"回退次数"这个曾经的字段（裁决 C-007 删除 rollbackCount）：
//  原设计在 history 之外另存一个 rollbackCount 摘要。删除的理由是实测：
//  该字段**零消费者**（全工程没有任何地方读过它），而它的值等于
//  history 中"触发失败的回退迁移"条数 —— 是一个可以导出的量。
//  保留一个无人读、且可由已有数据导出的冗余字段，只会让两个来源
//  有互相漂移的机会（改了一处忘了另一处，而两者都"看起来正常"）。
//
//  ⚠ 是否计入 history 的门槛（由生产方保证，不在本文件强制）：
//  只在**状态发生变化**时记一条。同一状态内的重试（RETRY_IN_STATE）不产生
//  迁移，否则 history 会退化成"尝试次数计数器"，丧失"路径"这一语义。
//  推论：history 里的条数**不等于**重试次数 —— 它记的是路径。
// ============================================================================

#include <vector>

#include "data/ErrorInfo.h"
#include "data/MeasurementState.h"
#include "data/StateTransition.h"

namespace aircraft
{
namespace data
{

/// 失败任务的根因与过程轨迹（裁决 C-007）。
struct FailureTrace
{
    /// **首次**失败所在的状态。
    /// 与 firstError 配对使用：只给错误码而不给所在状态，
    /// 同一个码（如 6001）在不同状态下的处置方向不同。
    MeasurementState firstFailedState = MeasurementState::IDLE;

    /// **根因**：首次失败的原因。离线排查应当先看这一个。
    ErrorInfo firstError;

    /// **终态**失败所在的状态（裁决 C-007 加入）。
    ///
    /// 与 firstFailedState 的区别是"根因 vs 终点"：典型链路里
    /// firstFailedState = POSE_SOLVE（模型缺失），
    /// finalFailedState = VALIDATE 或 MEASURE_SELECT（预算用尽处）。
    /// 只有 firstFailedState 而不知道终止在哪一步，就无法回答
    /// "这套重试策略把它耗在了哪一段"。
    MeasurementState finalFailedState = MeasurementState::IDLE;

    /// 终态错误。可能只是"预算用尽"这类**症状**（见文件头说明）。
    /// 成功任务该字段的 code == 0。
    ErrorInfo finalError;

    /// 逐状态轨迹。含全部迁移；触发失败的回退迁移其 error 非零。
    std::vector<StateTransition> history;
};

}  // namespace data
}  // namespace aircraft
