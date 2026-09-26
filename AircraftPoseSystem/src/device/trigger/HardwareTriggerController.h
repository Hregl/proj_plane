#pragma once

// ============================================================================
//  src/device/trigger/HardwareTriggerController.h
//
//  依据：SYS-06 §9.2（HardwareTriggerController：真实硬件触发）
//        SYS-08 §7.5（触发故障降级）、ENG-09 §4.3、ENG-01 §6
//
//  作用：真实硬触发源（同步脉冲发生器 / 相机的硬件触发输出）。
//
//  ⚠ 本阶段的实现状态：**诚实桩**（与 ImvCameraBackend、
//  PekoTurntableController 同策略）。硬件不存在，故 initialize() 返回
//  false，上层据此走 SYS-08 §7.5 的降级路径。
//
//  ⚠ 这里必须**区分两种失败**，且两者的上层动作完全不同：
//    · initialize() 失败 → §7.5 的"触发故障"→ 降级为软触发（3001），
//      测量**继续**；
//    · 降级后连软触发也不可用 → FAILED。
//  因此本类只负责如实报告自己不可用，**不**替上层决定降级 ——
//  降级是应用层策略（SYS-08 的职责），设备层不该替它做选择。
//
//  ⚠ 硬触发的三个关键参数（极性、脉宽、输出通道）在冻结文档中
//  均无定义，且不同触发源的接口形式差异很大（相机自身输出 vs
//  独立脉冲发生器）。故本类不预设任何参数结构 —— 凭空的参数结构
//  会让读者以为该接口已定型，而实际上接入时必然要改。真正接入时
//  参数应从 TriggerConfig 扩展（按 ENG-09 §8 的变更流程）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <string>

#include "data/ErrorInfo.h"
#include "data/TriggerConfig.h"
#include "device/trigger/ITriggerController.h"

namespace aircraft
{
namespace device
{

/// 真实硬触发控制（SYS-06 §9.2）。当前为诚实桩。
class HardwareTriggerController : public ITriggerController
{
public:
    /// @param config 触发配置（ENG-09 §6.4）。
    explicit HardwareTriggerController(const data::TriggerConfig& config);

    // ---- ITriggerController ----

    bool initialize() override;
    bool enable() override;
    bool trigger() override;
    void disable() override;

    // ---- 诊断 ----

    std::string lastErrorText() const;

    data::ErrorInfo lastError() const;

private:
    data::TriggerConfig config_;
    std::string         lastErrorText_;

    /// 最近一次失败的结构化记录，由 lastError() 返回。
    data::ErrorInfo lastError_;

    /// 硬件句柄。用 void* 而非具体类型，理由同
    /// ImvCameraBackend::handle_（避免把可选 SDK 的头文件
    /// 强加给所有包含者，ENG-03 §12.3）。
    void* handle_ = nullptr;

    bool enabled_ = false;
};

}  // namespace device
}  // namespace aircraft
