// ============================================================================
//  src/device/trigger/HardwareTriggerController.cpp
//
//  依据：SYS-06 §9.2、SYS-08 §7.5、ENG-09 §5.27
//
//  诚实桩：明确报告不可用，绝不假装触发成功（理由见头文件）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "device/trigger/HardwareTriggerController.h"

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

HardwareTriggerController::HardwareTriggerController(const data::TriggerConfig& config)
    : config_(config)
{
    lastErrorText_ =
        "HardwareTriggerController 未接入真实触发硬件：第一阶段使用 "
        "VirtualTriggerController（ENG-08 §3）。接入时需按 SYS-08 §7.5 "
        "确认降级路径（硬触发失效 → 软触发 + 记录 code=3001）"
        "在实际硬件上可达。";
}

bool HardwareTriggerController::initialize()
{
    // 返回 false 的直接后果是上层按 §7.5 降级 —— 这正是当前应有的行为，
    // 不是"错误"。若此处返回 true，上层会认为硬触发可用，
    // 于是不会走降级路径，而 trigger() 又实际不发脉冲，
    // 表现为三台相机自由运行、时间戳散布很大，
    // 同步判据频繁失败，而日志中看不到任何触发相关的错误。
    lastError_.code        = data::kErrTriggerDegraded;  // 3001
    lastError_.message     = lastErrorText_;
    lastError_.timestampNs = data::monotonicNowNs();
    return false;
}

bool HardwareTriggerController::enable()
{
    // 未初始化成功即使能：拒绝（理由同 VirtualTriggerController）。
    // 保持 enabled_ 为 false，使 trigger() 能区分"未使能"与
    // "已使能但硬件无响应"这两种原因 —— 二者的排查方向不同。
    enabled_               = false;
    lastError_.code        = data::kErrTriggerDegraded;  // 3001
    lastError_.message     = "硬触发未初始化成功，无法使能";
    lastError_.timestampNs = data::monotonicNowNs();
    return false;
}

bool HardwareTriggerController::trigger()
{
    // 不产生任何副作用（不递增计数、不驱动硬件）。
    // 置 3001 表明"本次触发未生效，应降级为软触发"；
    // message 区分具体原因，供日志定位。
    lastError_.code        = data::kErrTriggerDegraded;  // 3001
    lastError_.message     = enabled_ ? "硬触发已使能但无硬件响应，未产生触发脉冲"
                                      : "硬触发不可用，未产生触发脉冲";
    lastError_.timestampNs = data::monotonicNowNs();
    return false;
}

void HardwareTriggerController::disable()
{
    // 幂等，无副作用。
    enabled_ = false;
}

std::string HardwareTriggerController::lastErrorText() const
{
    return lastErrorText_;
}

data::ErrorInfo HardwareTriggerController::lastError() const
{
    return lastError_;
}

}  // namespace device
}  // namespace aircraft
