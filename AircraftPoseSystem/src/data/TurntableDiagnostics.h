#pragma once

// ============================================================================
//  src/data/TurntableDiagnostics.h
//
//  依据：裁决 C-23、SYS-10 §12/§13、评审（2026-10-19）
//
//  作用：转台后端对上层公开的**诊断快照**，与冻结的 `TurntableState`
//  并列，不改变 `ITurntableController::state()` 的签名。
//
//  ⚠ 三个概念在本文件中被刻意分开，任何时候都不得合并成一个“已停止”：
//    1. stopFrameDelivered   —— 停止帧已交给传输层（**不代表设备收到**）；
//    2. stopObservedStationary —— 停止后连续新鲜样本满足静止判据
//                                （**本系统观测到的事实**）；
//    3. 协议制动/停止确认    —— 需要设备明确反馈，本批**没有证据**，
//                                故本结构体不提供该字段，也不得用
//                                stopObservedStationary 冒充它。
//
//  ⚠ WorkMode 的可信度只允许四态。回复 0x00 但与查询字节不同，只能
//    证明“报文不同”，不能证明它是独立、有效的工作模式回复 —— 记
//    Unknown。本批没有可放行运动的协议证据，WorkMode 不作为运动/静止
//    判据，运动状态只由新鲜角度样本得出。
// ============================================================================

#include <cstdint>
#include <string>

#include "data/TurntableSample.h"

namespace aircraft
{
namespace data
{

/// 工作模式回复的可信度。
enum class WorkModeTrust : std::uint8_t
{
    Unknown = 0,      ///< 有地址正确的回复，但无法证明其独立性/有效性
    EchoOnly = 1,     ///< 只收到与查询帧逐字节相同的回复（回显歧义）
    ReplyNonZero = 2, ///< 回复的 data2 != 0（明确不是常规模式）
    NoReply = 3,      ///< 没有任何地址正确的回复
};

/// 最近一条运动命令的终结结果。
enum class TurntableCommandOutcome : std::uint8_t
{
    None = 0,       ///< 尚无命令
    Delivered = 1,  ///< 两轴指令均已交付（收到回显）
    Superseded = 2, ///< 被更新的运动命令覆盖
    Cancelled = 3,  ///< 被停止屏障取消
    Failed = 4,     ///< 发送/回显失败，或被门禁拒绝
};

/// 首因诊断（完整保存，不只存码）。
struct TurntableFault
{
    /// 错误码（ENG-09 §5.27 的 kErr* 常量；0 = 无故障）。
    int code = 0;

    /// 人读描述。
    std::string message;

    /// 触发该故障的结构化上下文（例如 "axis=horizontal;attempt=3"）。
    std::string context;

    /// 发生时刻，ns，CLOCK_MONOTONIC。
    std::uint64_t timestampNs = 0;
};

/// 转台诊断快照。
struct TurntableDiagnostics
{
    /// 最近一轮样本。`sample.valid == false` 时角值不可用。
    TurntableSample sample;

    /// 样本是否未超过 `sample_stale_ms`（由读取时刻计算）。
    bool sampleFresh = false;

    /// 运动事务编号：每开始一条新的运动命令即递增（0 = 尚无运动命令）。
    /// 旧命令被覆盖/取消后，其停止事实仍属于旧事务，不能挂到新事务上。
    std::uint64_t motionTransactionId = 0;

    /// `stopFrameDelivered` / `stopObservedStationary` 所属的运动事务编号。
    /// **与 `motionTransactionId` 不同 ⇒ 这两个肯定事实属于更早的事务。**
    /// 新运动开始时二者也会被清零；本字段是并发读取时的第二道归属依据。
    std::uint64_t stopFactsTransactionId = 0;

    /// 停止帧是否已交给传输层。**不等于**设备已收到或已停止。
    bool stopFrameDelivered = false;

    /// 停止后是否在规定观测窗口内观察到静止。
    bool stopObservedStationary = false;

    /// 运动是否处于不确定状态（停止未交付 / 未观察到静止 /
    /// 看门狗到期），在显式 stop() 或成功 initialize() 之前不清除。
    bool motionUncertain = false;

    /// 错误是否已锁存。
    bool errorLatched = false;

    /// 工作模式回复可信度。
    WorkModeTrust workModeTrust = WorkModeTrust::Unknown;

    /// 首因诊断：第一次故障的完整记录，不随后续升级而改写。
    TurntableFault firstFault;

    /// 最近一条运动命令的序号（0 = 尚无命令）。
    std::uint64_t lastCommandSeq = 0;

    /// 最近一条运动命令的终结结果。
    TurntableCommandOutcome lastCommandOutcome =
        TurntableCommandOutcome::None;

    /// 终结结果的人读说明。
    std::string lastCommandMessage;

    /// 最近一次停止请求的序号与完成序号（控制器屏障用）。
    std::uint64_t stopRequestSeq = 0;
    std::uint64_t stopCompletedSeq = 0;

    /// 运动门禁当前是否在拒绝运动（只读模式或 WorkMode 证据不足）。
    bool motionInhibited = false;
};

}  // namespace data
}  // namespace aircraft
