#pragma once

// ============================================================================
//  src/device/trigger/VirtualTriggerController.h
//
//  依据：SYS-06 §9.3（VirtualTriggerController：用于软件测试）
//        SYS-08 §7.5（触发降级）、§10（收敛性测试）
//        ENG-09 §4.3、ENG-01 §6
//
//  作用：无硬件时的触发源替身。
//
//  ⚠ 与工作流文档 4.md 的偏离：
//  4.md 只说明它有 Trigger → Camera Exposure 的示意流程，
//  未定义任何行为。本实现记录触发次数与时刻，
//  并支持**模拟触发失效** —— 后者是必要的：
//
//  SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 的"触发故障 → 降级为软触发并记录"是一条**必须实现**的
//  路径（code=3001）。若不提供模拟失效的手段，该路径在只有虚拟设备的
//  第一阶段永远无法被执行验证，于是它是否正确只能等到现场接入真实
//  硬触发时才知道 —— 而那时它已经和其它真实硬件问题耦合在一起了。
//
//  ⚠ TriggerConfig 只在构造时用于记录配置意图（source / periodMs），
//  本类**不自行定时**。理由：触发周期若由本类定时驱动，则采集节奏
//  会变成"触发源推动相机"，而 SYS-06 §8 的架构是
//  "TriggerController → 三相机同时曝光 → MultiCameraFrame"，
//  取帧节奏由上层（CAPTURE 状态按 captureFrameCount 循环）决定。
//  让桩自行定时会引入一个与真实架构不同的额外并发源，
//  使在虚拟设备上通过的测试无法代表真机行为。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <string>

#include "data/ErrorInfo.h"
#include "data/TriggerConfig.h"
#include "device/trigger/ITriggerController.h"

namespace aircraft
{
namespace device
{

/// 虚拟触发控制器（SYS-06 §9.3）。
class VirtualTriggerController : public ITriggerController
{
public:
    /// @param config 触发配置（ENG-09 §6.4）。source 字段用于诊断：
    ///        若配置要求 "hardware" 而实际注入的是虚拟触发，
    ///        该不一致会记录在 lastErrorText() 中，使"配置与实物不符"
    ///        可见 —— 否则这类不一致只会表现为同步精度莫名偏低。
    explicit VirtualTriggerController(const data::TriggerConfig& config);

    // ---- ITriggerController ----

    bool initialize() override;
    bool enable() override;
    bool trigger() override;
    void disable() override;

    // ---- 具象类追加能力 ----

    /// 模拟触发失效。置 true 后 trigger() 返回 false，
    /// 用于验证 SYS-08 §7.5 的降级路径（code=3001）。
    void simulateFailure(bool fail);

    /// 已成功触发的次数（诊断 / 断言用）。
    uint64_t triggerCount() const;

    /// 是否处于使能状态。
    bool enabled() const;

    /// 最近一次失败原因（人读）。
    std::string lastErrorText() const;

    data::ErrorInfo lastError() const;

private:
    data::TriggerConfig config_;

    bool     initialized_ = false;
    bool     enabled_     = false;
    bool     failing_     = false;

    uint64_t triggerCount_ = 0;

    std::string     lastErrorText_;
    data::ErrorInfo lastError_;
};

}  // namespace device
}  // namespace aircraft
