// ============================================================================
//  src/application/StateMachine.cpp
//
//  依据：SYS-08 §5~§7、§11；ENG-04 §10.4；ENG-06 §5.2
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "application/StateMachine.h"

#include <string>

namespace aircraft
{
namespace application
{

namespace
{

using data::MeasurementState;

/// 状态的日志用名称（与 RetryManager.cpp 中的同名函数一致）。
/// 两处各留一份是本阶段的有意选择：目前只有这两个 .cpp 需要它，
/// 而在 data 层新增 `measurementStateName()` 需要新增文件，
/// 属 ENG-01 §5 清单变更（同 RetryManager.cpp 的说明）。
const char* stateName(MeasurementState state)
{
    switch (state)
    {
    case MeasurementState::IDLE:           return "IDLE";
    case MeasurementState::SEARCH:         return "SEARCH";
    case MeasurementState::TARGET_FOUND:   return "TARGET_FOUND";
    case MeasurementState::ALIGN:          return "ALIGN";
    case MeasurementState::STABILIZE:      return "STABILIZE";
    case MeasurementState::MEASURE_SELECT: return "MEASURE_SELECT";
    case MeasurementState::CAPTURE:        return "CAPTURE";
    case MeasurementState::POSE_SOLVE:     return "POSE_SOLVE";
    case MeasurementState::VALIDATE:       return "VALIDATE";
    case MeasurementState::SAVE:           return "SAVE";
    case MeasurementState::COMPLETE:       return "COMPLETE";
    case MeasurementState::FAILED:         return "FAILED";
    }
    return "UNKNOWN_STATE";
}

}  // namespace

// ---------------------------------------------------------------------------

StateMachine::StateMachine(RetryManager& retry)
    : retry_(retry)
    , current_(MeasurementState::IDLE)
    , lastError_()
{
}

void StateMachine::reset()
{
    current_ = MeasurementState::IDLE;
    lastError_ = data::ErrorInfo{};
    // 刻意不动 retry_：计数与时限是任务级状态，见文件头偏离说明 2。
}

bool StateMachine::transition(data::MeasurementState next, uint64_t nowNs)
{
    // ---- ① 自转换不是转换 ----
    //
    // §7.3 的同状态重试（ALIGN 连试 8 次、CAPTURE 连试 3 次）**不是**状态
    // 转换：状态原地不动，重试由 beginAttempt() 记数。若允许自转换，
    // 每次重试都会再走一遍"进入状态"的记账，ALIGN 的重试上限会被
    // 实际可重试次数的一半耗尽（进入 1 次 + 自转换 7 次 = 8 次计数，
    // 但真正的对准动作只做了 4 次）。
    if (next == current_)
    {
        lastError_ = data::ErrorInfo{
            0,
            std::string("自转换被拒绝：当前已在 ") + stateName(next)
                + "（同状态重试应走 RetryManager::beginAttempt，不改变状态）",
            nowNs};
        return false;
    }

    // ---- ② 合法转换表（§6 的正常流程 ∪ §7.7 的恢复路径）----
    if (!isLegalTransition(current_, next))
    {
        lastError_ = data::ErrorInfo{
            0,
            std::string("非法转换被拒绝：") + stateName(current_) + " → "
                + stateName(next),
            nowNs};
        return false;
    }

    // ---- ③ 向后转换必须先获回退预算批准（§7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 1 / §11 约束 8）----
    //
    // 顺序上放在 beginAttempt 之前：回退被拒绝时不应消耗目标状态的尝试次数，
    // 否则一次不成功的回退会同时吃掉"回退预算"与"目标状态次数"两份配额，
    // 使 §7.3 与 §7.4 的算术关系（37.8 s < 60 s）失去意义。
    if (isRollback(current_, next)
        && !retry_.beginRollback(current_, next, nowNs))
    {
        lastError_ = retry_.lastError();
        return false;
    }

    // ---- ④ 进入目标状态即上报一次尝试（§7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2）----
    //
    // ⚠ 终止态（COMPLETE / FAILED）**例外，不上报**。这不是省一次记账，
    // 而是修正一处会让 §11 约束 9（"单次测量任务必然终止"）失效的缺陷：
    //
    //   RetryManager::beginAttempt() 的第一件事就是查 T_task（§7.1 的硬
    //   保证）。若终止态的进入也走这条检查，则 **T_task 到点的瞬间，
    //   恰恰是唯一能进入 FAILED 的通路被它自己关上** —— 状态机再也出不了
    //   当前状态，任务永不终止，恰好违反它本要保证的那一条。实测表现：
    //   搜索到 200 s 仍停在 SEARCH，lastError 是 9001，而状态不是 FAILED。
    //
    // 概念上也本就不该计数：§7.3 的次数上限与 §7.4〔引用无效·依据待裁决·见 Q-D2〕 的回退预算约束的是
    // **工作**（搜索、对准、采集、解算……），而"进入终止态"不做事，
    // 也没有"下一次尝试"可言。§7.3 的表中列出的上限全部属于工作状态。
    //
    // COMPLETE 同理豁免：与 T_task 同一时刻通过 VALIDATE 的任务，其结果是
    // 有效的测量结果，不应因为一次时点比较而被丢弃；"在 T_task 内终止"
    // 这一承诺由 tick() 在**每个动作之前**的检查保证（见
    // MeasurementController::tick()），与终止边的记账无关。
    if (!isTerminal(next) && !retry_.beginAttempt(next, nowNs))
    {
        lastError_ = retry_.lastError();
        return false;
    }

    current_ = next;
    lastError_ = data::ErrorInfo{};
    return true;
}

data::MeasurementState StateMachine::state() const
{
    return current_;
}

data::ErrorInfo StateMachine::lastError() const
{
    return lastError_;
}

// ---------------------------------------------------------------------------
// 转换表
// ---------------------------------------------------------------------------

bool StateMachine::isLegalTransition(data::MeasurementState from,
                                     data::MeasurementState to)
{
    switch (from)
    {
    case MeasurementState::IDLE:
        // §5.1：退出 IDLE 的唯一方式是"开始测量"→ SEARCH。
        // → FAILED 亦合法：任务启动时发现相机数不足（§7.5 ≤1 路）就直接失败，
        // 此时尚未进入 SEARCH。
        return to == MeasurementState::SEARCH
            || to == MeasurementState::FAILED;

    case MeasurementState::SEARCH:
        // §5.2：检出 → TARGET_FOUND；未检出 → 保持 SEARCH（非转换）。
        return to == MeasurementState::TARGET_FOUND
            || to == MeasurementState::FAILED;

    case MeasurementState::TARGET_FOUND:
        // §5.3：成功 → ALIGN。§5.3 未给失败行，§7.7 亦未列该状态。
        //
        // ⚠ 本阶段曾允许 TARGET_FOUND → SEARCH 的回退，现**已移除**：
        // 它是条死路。§7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2（每次进入状态都报一次尝试）与约束 5
        //（计数器不因进入新状态而清零）叠加 §7.3 给该状态的上限 1，
        // 使 TARGET_FOUND 至多能被进入一次 —— 回退到 SEARCH 之后，
        // SEARCH 唯一的前进边会被永久拒绝，任务只能空转到 T_task 并报 9001。
        // 因此 MeasurementStrategy 改为就地 FAILED（见该文件的 TARGET_FOUND 分支）。
        // 若将来有裁决允许回退后重新进入，需同时恢复本条边与 strategy 的处置。
        return to == MeasurementState::ALIGN
            || to == MeasurementState::FAILED;

    case MeasurementState::ALIGN:
        // §5.4：目标中心误差 ≤ ±50 pixel → STABILIZE；
        // 未达判据 → **继续调整**（同状态重试，非转换）。上限 8 次，超限
        // 直接 FAILED（2003），没有回退边 —— 对准失败时回退到更早的状态
        // 没有意义（目标都还没进视场中心）。
        return to == MeasurementState::STABILIZE
            || to == MeasurementState::FAILED;

    case MeasurementState::STABILIZE:
        // §5.5：稳定 → MEASURE_SELECT。
        // → ALIGN：§7.7"稳定失败 → 重做一次稳定等待（1 次）→ 超限回退至 ALIGN"。
        return to == MeasurementState::MEASURE_SELECT
            || to == MeasurementState::ALIGN
            || to == MeasurementState::FAILED;

    case MeasurementState::MEASURE_SELECT:
        // §5.6：选出相机 → CAPTURE。候选集合为空时重试（上限 3），
        // 3 次后无解可退（更早的状态同样无相机可选），故直接 FAILED。
        return to == MeasurementState::CAPTURE
            || to == MeasurementState::FAILED;

    case MeasurementState::CAPTURE:
        // §5.7：采到有效帧 → POSE_SOLVE。
        // → MEASURE_SELECT：采集持续失败意味着当前焦段不可用，
        // 回退换一个候选通道是 §7.3〔引用无效·依据待裁决·见 Q-D2〕 "第 2 次必须换用次优相机"的落地方式
        // （CAPTURE 与 MEASURE_SELECT 之间无其他状态可退）。
        return to == MeasurementState::POSE_SOLVE
            || to == MeasurementState::MEASURE_SELECT
            || to == MeasurementState::FAILED;

    case MeasurementState::POSE_SOLVE:
        // §5.8：成功 → VALIDATE；"失败：进入 FAILED 或重新 CAPTURE"。
        // §7.7 的 PnP 行则写"回退 MEASURE_SELECT"。两处并存，本阶段**同时
        // 允许两条回退边**：§7.7〔引用无效·依据待裁决·见 Q-D2〕 只声明"原 §7 的继续搜索/继续调整/重新测量
        // 表述作废"，并未否定 §5.8 的"重新 CAPTURE"；而二者的差别是
        // 重试粒度（换帧 vs 换相机），由 MeasurementStrategy 按 §7.3 的
        // 升级规则选择，两条边都消耗 §7.4 的回退预算。
        return to == MeasurementState::VALIDATE
            || to == MeasurementState::MEASURE_SELECT
            || to == MeasurementState::CAPTURE
            || to == MeasurementState::FAILED;

    case MeasurementState::VALIDATE:
        // §5.9：通过 → SAVE；不通过 → 最多 2 次回退至 MEASURE_SELECT
        //（每次必须排除上一次失败的相机），2 次后 FAILED。
        return to == MeasurementState::SAVE
            || to == MeasurementState::MEASURE_SELECT
            || to == MeasurementState::FAILED;

    case MeasurementState::SAVE:
        // §5.10：落盘完成 → COMPLETE。落盘失败重试（上限 3），
        // 3 次后 FAILED —— 没有更早的状态能解决"磁盘写不进去"。
        return to == MeasurementState::COMPLETE
            || to == MeasurementState::FAILED;

    case MeasurementState::COMPLETE:
        // §5.1：IDLE 的进入条件之一是"上次任务结束"。
        // ⚠ 不允许 COMPLETE → FAILED：任务已成功终止，此后再置失败会让
        // result.json 与状态记录互相矛盾。
        return to == MeasurementState::IDLE;

    case MeasurementState::FAILED:
        // 终止态，只能开始新任务。
        return to == MeasurementState::IDLE;
    }

    return false;
}

bool StateMachine::isRollback(data::MeasurementState from,
                              data::MeasurementState to)
{
    // 进入终止态 / 由终止态重新开始 / 回到 IDLE：都不是"向后转换"。
    if (to == MeasurementState::FAILED || to == MeasurementState::IDLE
        || from == MeasurementState::FAILED
        || from == MeasurementState::COMPLETE)
    {
        return false;
    }

    const int rf = forwardRank(from);
    const int rt = forwardRank(to);
    if (rf < 0 || rt < 0)
    {
        return false;
    }

    return rt < rf;
}

int StateMachine::forwardRank(data::MeasurementState state)
{
    switch (state)
    {
    case MeasurementState::IDLE:           return 0;
    case MeasurementState::SEARCH:         return 1;
    case MeasurementState::TARGET_FOUND:   return 2;
    case MeasurementState::ALIGN:          return 3;
    case MeasurementState::STABILIZE:      return 4;
    case MeasurementState::MEASURE_SELECT: return 5;
    case MeasurementState::CAPTURE:        return 6;
    case MeasurementState::POSE_SOLVE:     return 7;
    case MeasurementState::VALIDATE:       return 8;
    case MeasurementState::SAVE:           return 9;
    case MeasurementState::COMPLETE:       return 10;
    case MeasurementState::FAILED:         return -1;
    }
    return -1;
}

bool StateMachine::isTerminal(data::MeasurementState state)
{
    return state == MeasurementState::COMPLETE
        || state == MeasurementState::FAILED;
}

}  // namespace application
}  // namespace aircraft
