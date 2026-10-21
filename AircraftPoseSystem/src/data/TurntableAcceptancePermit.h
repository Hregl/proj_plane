#pragma once

// ============================================================================
//  src/data/TurntableAcceptancePermit.h
//
//  依据：SYS-10 §6.5（运动门禁）、SYS-04 §21、ENG-09 §6.3、裁决 C-23
//
//  作用：**仅限实机验收**的运动许可。它不是生产配置，也不是
//  `TurntableSendPolicy`：`sendPolicy` 仍只描述“只读 / 运动使能”这一
//  结构性能力，而本许可描述“这一次验收允许哪台设备、在哪个角度范围内、
//  以多大位移、执行多少次、在什么时间段内运动”。
//
//  ⚠ 本结构体不改变以下事实：
//    · 生产装配与 `config/` 里没有任何键能装载它；
//    · 许可不改变 `WorkModeTrust`（`EchoOnly`/`Unknown` 仍是原值），
//      也不把任何回复解释成“协议确认正常”；
//    · 许可不绕过异常工作模式、通信故障、过期样本或错误锁存；
//    · 许可只约束**新的运动命令**。停止、退出与故障收口不受期限限制
//      —— 已经开始的运动在许可过期后仍必须能够停止。
// ============================================================================

#include <cstdint>
#include <string>

namespace aircraft
{
namespace data
{

/// 实机验收运动许可。
///
/// 所有字段都是**拒绝优先**：缺失、越界、非有限或过期一律拒绝，
/// 不存在“缺省即允许”的字段。
struct TurntableAcceptancePermit
{
    /// 验收范围编号（现场记录与证据引用）。必填，非空。
    std::string scopeId;

    /// 设备身份：必须与 `TurntableConfig` 的 host/port/pelcoAddress
    /// 逐项一致，否则安装即失败。
    std::string deviceHost;
    int         devicePort   = 0;
    int         pelcoAddress = 0;

    /// 允许的操作。当前协议实现只有绝对定位（两轴同帧事务）。
    /// 运动中的停止是安全动作，不设许可门禁；但**静止状态下主动发送
    /// 停止帧**是一次会改变线上流量的验收操作，必须单独授权。
    bool allowAbsolutePosition = false;
    bool allowStopFrameAtRest  = false;

    /// 许可允许的目标角范围（deg）。必须落在配置的机械行程之内，
    /// 且 min <= max。目标的**解析后**角度（含 ±360 等价角）在此范围内
    /// 才允许发送。
    double azimuthMinDeg   = 0.0;
    double azimuthMaxDeg   = 0.0;
    double elevationMinDeg = 0.0;
    double elevationMaxDeg = 0.0;

    /// 单次命令相对**当前新鲜有效实测角**的最大变化量（deg，每轴）。
    /// 用于防止一次误输入把转台推到很远的位置。
    double maxDisplacementDeg = 0.0;

    /// 本进程最多允许执行的绝对定位命令数（> 0）。
    /// 每进入发送阶段的命令消耗一次；发送失败也不退款 —— 许可约束的是
    /// “尝试次数”，不是“成功次数”。
    int maxExecutions = 0;

    /// 有效期（Unix epoch 毫秒，左闭右开）。
    /// 过期后拒绝**新的**运动命令；停止/清理路径不受影响。
    std::int64_t validFromEpochMs  = 0;
    std::int64_t validUntilEpochMs = 0;
};

}  // namespace data
}  // namespace aircraft
