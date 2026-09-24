// ============================================================================
//  src/application/RetryManager.cpp
//
//  依据：SYS-08 §7.1 / §7.3 / §7.4 / §7.6 / §7.7、§10（收敛性测试）、§11（约束 6/8）
//        ENG-09 §5.27（错误码）、§6.5（16 个重试/超时字段）、§2.5（时间基准）
//
//  接口按 SYS-08 §7.6 逐字实现；命名空间限定符按 ENG-09 补全（§7.6 的片段
//  写在 application 命名空间内，故其 ErrorInfo / MeasurementState 未加
//  `data::` 前缀；本工程中二者冻结在 data 层，见 data/MeasurementState.h
//  的裁决 C-13）。**除限定符外未增删任何成员与方法。**
// ============================================================================

#include "application/RetryManager.h"

#include <string>

namespace aircraft
{
namespace application
{

namespace
{

/// 状态的日志用名称。仅用于 ErrorInfo::message，**不用于机读判断**
/// （机读一律以 code 为准，见 data/ErrorInfo.h）。
///
/// 放在本 .cpp 内而非 data 层的原因：目前只有本类的错误消息需要它。
/// 009（Qt 界面 StatusPanel、infrastructure 的 Logger）落地时会同样需要，
/// 届时再评估是否在 data 层提供一个 `measurementStateName()` ——
/// 那需要新增文件，属 ENG-01 §5 清单的变更，不由本阶段顺手决定。
const char* stateName(data::MeasurementState state)
{
    switch (state)
    {
    case data::MeasurementState::IDLE:           return "IDLE";
    case data::MeasurementState::SEARCH:         return "SEARCH";
    case data::MeasurementState::TARGET_FOUND:   return "TARGET_FOUND";
    case data::MeasurementState::ALIGN:          return "ALIGN";
    case data::MeasurementState::STABILIZE:      return "STABILIZE";
    case data::MeasurementState::MEASURE_SELECT: return "MEASURE_SELECT";
    case data::MeasurementState::CAPTURE:        return "CAPTURE";
    case data::MeasurementState::POSE_SOLVE:     return "POSE_SOLVE";
    case data::MeasurementState::VALIDATE:       return "VALIDATE";
    case data::MeasurementState::SAVE:           return "SAVE";
    case data::MeasurementState::COMPLETE:       return "COMPLETE";
    case data::MeasurementState::FAILED:         return "FAILED";
    }
    return "UNKNOWN_STATE";
}

/// 次数用尽时该填哪个错误码。
///
/// 只有 SYS-08 §7.7 明确给出码位的四个状态有专属码；其余如实返回 0
/// （理由见 RetryManager.h 中 lastError() 的说明）。ENG-09 §5.27 禁止
/// 裸字面量，故这里引用 data 层的 kErr* 常量。
int exhaustionCode(data::MeasurementState state)
{
    switch (state)
    {
    case data::MeasurementState::ALIGN:
        return data::kErrAlignRetryExhausted;   // 2003（§7.7 "对准失败"）

    case data::MeasurementState::POSE_SOLVE:
        return data::kErrPnpRetryExhausted;     // 6001（§7.7 "PnP 失败"）

    default:
        // TARGET_FOUND / STABILIZE / MEASURE_SELECT / CAPTURE / SAVE：
        // 无登记码位，置 0 并在 message 中写明状态名。
        return 0;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造与任务生命周期
// ---------------------------------------------------------------------------

RetryManager::RetryManager(const data::MeasurementConfig& config)
    : config_(config)
    , deadlineNs_(kNoDeadline)
    , rollbackTotal_(0)
    , lastError_()
{
    // config_ 是**构造时快照**（值拷贝）。理由见 ENG-09 §6.8：
    // 一次任务必须由同一份配置驱动，若持有引用而外部在任务中途改配置，
    // 同一次任务的不同阶段会用到不同的次数上限，回退预算的算术随之失效。
}

void RetryManager::beginTask(uint64_t nowNs)
{
    reset();

    // 饱和加法：taskTimeoutNs 可注入（SYS-08 §10 要求可注入短值，
    // 也允许注入长值）。若注入值接近 uint64 上限，nowNs + T 会回绕成
    // 一个小数 —— 那会让 deadlineExceeded() 立刻为真，任务一启动即超时。
    // 回绕不会报错，只会表现为"刚点开始就失败"，故在此显式防住。
    const uint64_t limit = kNoDeadline - nowNs;
    deadlineNs_ = (config_.taskTimeoutNs >= limit) ? kNoDeadline
                                                   : nowNs + config_.taskTimeoutNs;
}

void RetryManager::reset()
{
    attempts_.clear();
    rollbackEdge_.clear();
    rollbackTotal_ = 0;
    deadlineNs_ = kNoDeadline;
    lastError_ = data::ErrorInfo{};
}

// ---------------------------------------------------------------------------
// 记账
// ---------------------------------------------------------------------------

bool RetryManager::beginAttempt(data::MeasurementState state, uint64_t nowNs)
{
    // ---- 先判任务时限 ----
    //
    // 顺序即 SYS-08 §10 用例 7（"时限优先级：先到者生效"）的答案：
    // 时限是任务级的**硬保证**（§7.1：T_task 到则立即 FAILED 且不可重试），
    // 而次数上限只是效率手段（§7.3 原文：次数上限负责效率，任务时限负责
    // 硬保证）。两者在同一次调用上同时成立时，硬保证优先 —— 否则会报出
    // "某状态次数用尽"，掩盖"任务已经超时"这一更根本的事实。
    if (deadlineExceeded(nowNs))
    {
        lastError_ = data::ErrorInfo{
            data::kErrTaskTimeout,
            "任务时限 T_task 已到，不再允许任何重试",
            nowNs};
        return false;
    }

    const int cap  = maxAttemptsFor(config_, state);
    const int used = attempts_[state];   // 不存在时插入 0，等价于"未尝试过"

    if (cap != kUnlimitedAttempts && used >= cap)
    {
        lastError_ = data::ErrorInfo{
            exhaustionCode(state),
            std::string(stateName(state)) + " 状态尝试次数已用尽（上限 "
                + std::to_string(cap) + " 次）",
            nowNs};
        return false;
    }

    attempts_[state] = used + 1;

    // SEARCH 不设上限（§7.3），但**仍然计数** —— 计数不为限流，
    // 而是为了让"已经搜了多少次"可上报（UI 与日志需要它判断是否在正常推进）。
    lastError_ = data::ErrorInfo{};
    return true;
}

bool RetryManager::beginRollback(data::MeasurementState from,
                                 data::MeasurementState to,
                                 uint64_t nowNs)
{
    if (deadlineExceeded(nowNs))
    {
        lastError_ = data::ErrorInfo{
            data::kErrTaskTimeout,
            "任务时限 T_task 已到，不再允许回退",
            nowNs};
        return false;
    }

    // ---- 两个预算都必须满足，取先到者（§7.4）----
    //
    // 只有总预算时，回退可在两状态间反复弹跳（A→B→A→B），每次都消耗总量
    // 且每次都像在"前进"，直到总量用尽才暴露 —— 而这期间转台已在做重复的
    // 机械运动。每边上限把这种振荡在第二次就切断。
    const auto edge = std::make_pair(from, to);

    if (rollbackTotal_ >= config_.maxRollbackTotal)
    {
        lastError_ = data::ErrorInfo{
            data::kErrRollbackExhausted,
            "单次任务总回退次数已用尽（上限 "
                + std::to_string(config_.maxRollbackTotal) + " 次）",
            nowNs};
        return false;
    }

    if (rollbackEdge_[edge] >= config_.maxRollbackPerEdge)
    {
        lastError_ = data::ErrorInfo{
            data::kErrRollbackExhausted,
            std::string("回退边 ") + stateName(from) + " → " + stateName(to)
                + " 次数已用尽（每条边上限 "
                + std::to_string(config_.maxRollbackPerEdge) + " 次）",
            nowNs};
        return false;
    }

    ++rollbackTotal_;
    ++rollbackEdge_[edge];

    lastError_ = data::ErrorInfo{};
    return true;
}

// ---------------------------------------------------------------------------
// 查询
// ---------------------------------------------------------------------------

int RetryManager::attempts(data::MeasurementState state) const
{
    const auto it = attempts_.find(state);
    return (it == attempts_.end()) ? 0 : it->second;
    // ⚠ 用 find 而非 operator[]：本方法是 const 查询，
    // 而 operator[] 在键缺失时会**插入**一个 0 —— 那会让"查询"改变
    // 对象的可观察状态（且 const 版本根本编译不过）。
    // 更要紧的是语义：未进入过的状态返回 0，与"进入过但记 0 次"不同，
    // 虽然当前二者结果相同，但插入会让 rollbackEdge_ 式的误用蔓延。
}

int RetryManager::rollbackCount() const
{
    return rollbackTotal_;
}

bool RetryManager::deadlineExceeded(uint64_t nowNs) const
{
    // 无活动任务时 deadlineNs_ == kNoDeadline，nowNs 恒小于它 → false。
    // 判定用 >=：到达截止时刻即为"已到"（§7.1 的 T_task 是时限而非宽度）。
    return nowNs >= deadlineNs_;
}

uint64_t RetryManager::remainingNs(uint64_t nowNs) const
{
    if (deadlineNs_ == kNoDeadline)
    {
        return kNoDeadline;   // 无期限（语义：无穷大，不是"没有时间"）
    }
    return (nowNs >= deadlineNs_) ? 0u : (deadlineNs_ - nowNs);
}

data::ErrorInfo RetryManager::lastError() const
{
    return lastError_;
}

// ---------------------------------------------------------------------------
// 内部
// ---------------------------------------------------------------------------

int RetryManager::maxAttemptsFor(const data::MeasurementConfig& config,
                                 data::MeasurementState state)
{
    switch (state)
    {
    // SEARCH 不设次数上限（§7.3）：目标可能确实尚未进入视场，
    // "重试次数用尽"不是有意义的结论。它由 T_task 与人工取消双重约束，
    // 且除硬件故障外不存在死循环可能。
    case data::MeasurementState::SEARCH:
        return kUnlimitedAttempts;

    // 该状态内不重试（§7.3 原文：1（一次等待））。
    // TARGET_FOUND 失败意味着这一帧/这一批检测无法给出有效 TargetOffset；
    // STABILIZE 的"重做一次稳定等待"由 §7.7 表达为回退至 ALIGN，
    // 属跨状态动作，不是状态内重试。二者上限因此恒为 1 且不进配置
    // （理由见 RetryManager.h 的 maxAttemptsFor 注释）。
    case data::MeasurementState::TARGET_FOUND:
    case data::MeasurementState::STABILIZE:
        return kOnceOnlyAttempts;

    case data::MeasurementState::ALIGN:          return config.maxAlignAttempts;
    case data::MeasurementState::MEASURE_SELECT: return config.maxSelectAttempts;
    case data::MeasurementState::CAPTURE:        return config.maxCaptureAttempts;
    case data::MeasurementState::POSE_SOLVE:     return config.maxSolveAttempts;
    case data::MeasurementState::VALIDATE:       return config.maxValidateAttempts;
    case data::MeasurementState::SAVE:           return config.maxSaveAttempts;

    // IDLE / COMPLETE / FAILED：按状态图，一次任务内各自至多进入一次。
    // 返回 1 而非 0，是为了让"重复进入终止态"这一调用方错误**立刻显形**：
    // 若返回 0（无上限），状态机在 FAILED 之后继续 beginAttempt() 会被
    // 静默放行，任务便在终态上无限空转，直到 T_task 才收场 ——
    // 那时日志里只有一条超时，看不到真正的起因。
    case data::MeasurementState::IDLE:
    case data::MeasurementState::COMPLETE:
    case data::MeasurementState::FAILED:
        return kOnceOnlyAttempts;
    }

    return kOnceOnlyAttempts;
}

}  // namespace application
}  // namespace aircraft
