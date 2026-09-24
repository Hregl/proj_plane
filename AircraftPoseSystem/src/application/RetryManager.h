#pragma once

// ============================================================================
//  src/application/RetryManager.h
//
//  依据：SYS-08 §7.1（三级超时）、§7.3（次数与状态级超时）、§7.4（回退预算）、
//        §7.6（**接口冻结原文**）、§7.7（恢复路径汇总）、§11（状态机约束 6/8）
//        ENG-09 §6.5（MeasurementConfig 的 16 个重试/超时字段）、裁决 C-20
//        ENG-08 §9（Sprint 5 必做项：原清单遗漏本类）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类是**唯一**持有重试计数器与任务时限的类（SYS-08 §7.6）。            │
//  │ 状态机自身不得保存任何重试计数 —— 见 §7.6 约束与 §11 约束 6。          │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  为什么计数必须集中在一个类里（SYS-08 §7.4 的论证）：
//  只有"每状态次数上限"时，存在跨状态循环
//      VALIDATE（失败）→ MEASURE_SELECT → CAPTURE → POSE_SOLVE → VALIDATE
//  每转一圈，各状态的独立计数器都被重新计满，循环次数**不受 §7.3 约束**。
//  因此必须另设**回退计数器**（总预算 4、每条回退边 2）。二者的关系是
//  "取先到者"：任何一处超限即 FAILED（code 9002）。若计数分散在各状态内部，
//  "总回退次数"这一全局量就无处安放，有界性无法证明 —— 而 SYS-08 §11 约束 9
//  要求"单次任务在 T_task 内必然终止"。
//
//  ⚠ 本类**不做状态转换决策**（§7.6 约束 4）：它只回答"是否允许"，
//  转换由 StateMachine 执行。这样超时与次数策略可以独立于状态图演进。
//
//  ⚠ 时间来源：全部 nowNs 由**调用方**传入，本类不自行读时钟。
//  理由有二：
//    (1) 可测性 —— SYS-08 §10 的收敛性测试要在 2 s 的短 T_task 下遍历
//        超时路径，调用方注入时间即可，不必真的等 60 s；
//    (2) 一致性 —— 一次事件循环内所有判断必须基于**同一时刻**，
//        各处各自 clock_gettime 会得到互不相同的值，
//        使"先到者生效"（§10 用例 7）无法判定。
//  取时用 data::MonotonicClock（ENG-09 §2.5：主机 CLOCK_MONOTONIC 纳秒）。
//
//  ⚠ 单位：本类全部时间为 **ns**（uint64_t），角度/长度不出现于此。
// ============================================================================

#include <cstdint>
#include <limits>
#include <map>
#include <utility>

#include "data/ErrorInfo.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"

namespace aircraft
{
namespace application
{

/// 重试计数器与任务时限的唯一持有者（SYS-08 §7.6）。
///
/// 典型用法（状态机的事件循环内）：
///     if (retry_.deadlineExceeded(now)) { fail(9001); }
///     if (!retry_.beginAttempt(state, now)) { ... }      // 进入状态时
///     if (!retry_.beginRollback(from, to, now)) { ... }   // 向后转换前
class RetryManager
{
public:
    explicit RetryManager(const data::MeasurementConfig& config);

    // ---- 任务生命周期 -----------------------------------------------------

    /// 开始一次新任务：**清零全部计数器**并设定 T_task 截止时刻。
    ///
    /// ⚠ beginTask() 内部会先 reset()。若不先清零，第二次测量任务会继承
    /// 上一次的尝试与回退计数 —— 表现为"第二次测量一启动就 FAILED"，
    /// 而错误码指向次数用尽，与真实原因（计数未清零）相距甚远。
    ///
    /// @param nowNs 当前时刻，data::MonotonicClock::nowNs()。
    void beginTask(uint64_t nowNs);

    /// 清空全部计数与截止时刻，回到"无活动任务"状态。
    ///
    /// ⚠ 清零**只能**经由本方法与 beginTask()（§7.6 约束 5：
    /// 计数器不因进入新状态而清零，否则跨状态循环就不受 §7.4 约束）。
    void reset();

    // ---- 记账 -------------------------------------------------------------

    /// 状态进入时上报一次尝试。
    ///
    /// @return false = 该状态次数已用尽（或任务已超时），调用方应据此走
    ///         §7.7 的对应恢复路径。**返回 false 时计数器不再增加** ——
    ///         否则"已用尽"的判断会因为查询本身而被推迟。
    ///
    /// ⚠ 每次重试必须改变至少一项输入（§7.3 升级规则）：第 1 次重采帧、
    /// 第 2 次重采 + 换次优相机、第 3 次重采 + 放宽阈值。本类只计数，
    /// **不检查输入是否真的变了** —— 该约束由状态机在组织输入时保证
    /// （§10 用例 4 会验证第 2 次与第 1 次的输入不同）。
    bool beginAttempt(data::MeasurementState state, uint64_t nowNs);

    /// 向后（回退）转换审批。from/to 同时用于**同一条回退边**的计数。
    ///
    /// @return false = 回退预算已用尽（总预算或本边预算），或任务已超时。
    ///         lastError() 给出超限原因。
    ///
    /// ⚠ from == to 不是回退（应作为同状态重试走 beginAttempt）。
    /// 传入相同状态时本方法仍按一条边计数 —— 保守处理：宁可占预算，
    /// 也不让一次真正的振荡被漏计。
    bool beginRollback(data::MeasurementState from,
                       data::MeasurementState to,
                       uint64_t nowNs);

    // ---- 查询 -------------------------------------------------------------

    /// 指定状态已消耗的尝试次数。未进入过的状态返回 0。
    int attempts(data::MeasurementState state) const;

    /// 本次任务累计的回退次数（§7.4 总预算 4）。
    int rollbackCount() const;

    /// 任务时限是否已到（§7.1 的 T_task，默认 60 s）。
    ///
    /// ⚠ 无活动任务时返回 false —— 未 beginTask() 就不存在"超时"，
    /// 若返回 true 会让状态机在启动瞬间误判。
    bool deadlineExceeded(uint64_t nowNs) const;

    /// 距 T_task 截止的剩余时间，单位 ns。已超时返回 0。
    /// 无活动任务时返回极大值（语义：无期限）。
    ///
    /// 用途：SEARCH 状态不设次数上限（§7.3），其终止完全依赖 T_task，
    /// 故需要把剩余时间上报给 UI 使"还能搜多久"可见。
    uint64_t remainingNs(uint64_t nowNs) const;

    /// 最近一次拒绝的原因，供填充 ErrorInfo（§7.6）。
    ///
    /// 码的取值（ENG-09 §5.27，禁止裸字面量）：
    ///   kErrTaskTimeout(9001)         —— 任务时限到，不可重试
    ///   kErrRollbackExhausted(9002)   —— 回退预算用尽
    ///   kErrAlignRetryExhausted(2003) —— ALIGN 次数用尽
    ///   kErrPnpRetryExhausted(6001)   —— POSE_SOLVE 次数用尽
    ///   0                             —— 其余状态次数用尽
    ///
    /// ⚠ 最后一项是**有意的留白**：ENG-09 §5.27 的 9000 段已占用
    /// 9001/9002/9003，2000 段已占用 2001/2002/2003，6000 段已占用 6001。
    /// TARGET_FOUND / STABILIZE / MEASURE_SELECT / CAPTURE / SAVE 五个状态的
    /// "次数用尽"没有登记码位，因此**如实置 0 并在 message 中写明状态名**，
    /// 不新造码（新造码会让日志与冻结表的对应关系断裂，须走 ENG-09 §8
    /// 变更流程登记）。这五个状态在 §7.7 中均不作为终态：
    /// STABILIZE 用尽回退 ALIGN，其余回退 MEASURE_SELECT 或由 T_task 兜底，
    /// 因此它们的"次数用尽"本身不构成最终结论。
    data::ErrorInfo lastError() const;

private:
    /// 该状态的尝试次数上限。0 表示**不设上限**（仅 SEARCH，§7.3）。
    ///
    /// ⚠ TARGET_FOUND 与 STABILIZE 的上限恒为 1，且**不在**
    /// MeasurementConfig 中 —— 这不是遗漏：ENG-09 §6.5 冻结的 16 个
    /// 重试/超时字段是 10 个状态超时 + 6 个次数上限，其中不含这两个，
    /// 因为它们的语义是"该状态内不重试"（§7.3 原文：1（一次等待）），
    /// 对固定的 1 提供配置项只会制造"配成 5 会怎样"的歧义。
    static int maxAttemptsFor(const data::MeasurementConfig& config,
                             data::MeasurementState state);

    /// 只计一次尝试的状态所用的上限（§7.3 的 TARGET_FOUND 与 STABILIZE）。
    static constexpr int kOnceOnlyAttempts = 1;

    /// "不设上限"的表示法（§7.3 的 SEARCH）。
    ///
    /// ⚠ 取 **-1** 而不是 0，且这个选择是必须的。
    ///
    /// 0 曾被用作"无限"的哨兵值，后果是：配置里任何一处把上限误写成 0
    ///（漏写 yaml 字段、或误以为 0 表示"用默认值"），该状态就变成
    /// **无限重试**，只在 T_task 到点时以一个没有指向性的 9001 结束 ——
    /// 而它的真实原因是配置错误。这与本项目对"静默退化"的一贯反对
    ///（见 MeasurementConfig.h、sigmaPxFallback 的注释）直接冲突。
    ///
    /// 改用 -1 后，配置中的 0 恢复其字面语义：**一次都不放行**，
    /// 于是误配会在状态的第一次动作上立即暴露（FAILED + 该状态的超限码），
    /// 与 §7.4 的回退预算对 0 的处置（预算 0 → 首次回退即拒绝）一致。
    /// 0 是 int 的自然下界附近的哨兵，任何合法上限都是正数，不可能碰撞。
    static constexpr int kUnlimitedAttempts = -1;

    /// deadlineNs_ 在无活动任务时的取值。
    ///
    /// 用极大值而非 0 表达"无期限"，因为 0 会让 deadlineExceeded() 恒真
    /// （nowNs >= 0 永真）—— 那样状态机在 beginTask() 之前就会被判超时。
    /// 这与 ErrorInfo::code 用 0 表示"未设置"的方向相反，原因不同：
    /// code 的 0 是"无错误"，而这里需要的是"无穷大的剩余时间"。
    static constexpr uint64_t kNoDeadline =
        (std::numeric_limits<uint64_t>::max)();

    data::MeasurementConfig config_;

    /// T_task 截止时刻，单位 ns（CLOCK_MONOTONIC）。
    /// 无活动任务时为 kNoDeadline。
    uint64_t deadlineNs_;

    /// 各状态已消耗的尝试次数。**不因进入新状态而清零**（§7.6 约束 5）。
    std::map<data::MeasurementState, int> attempts_;

    /// 各回退边的次数，键为 (from, to)（§7.4 每边上限 2）。
    std::map<std::pair<data::MeasurementState, data::MeasurementState>, int>
        rollbackEdge_;

    /// 本次任务累计回退次数（§7.4 总预算 4）。
    int rollbackTotal_;

    /// 最近一次拒绝的原因。
    data::ErrorInfo lastError_;
};

}  // namespace application
}  // namespace aircraft
