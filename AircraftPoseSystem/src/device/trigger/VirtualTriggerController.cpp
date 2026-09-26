// ============================================================================
//  src/device/trigger/VirtualTriggerController.cpp
//
//  依据：SYS-06 §9.3、SYS-08 §7.5、ENG-09 §5.27（错误码）
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "device/trigger/VirtualTriggerController.h"

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

VirtualTriggerController::VirtualTriggerController(const data::TriggerConfig& config)
    : config_(config)
{
    // 配置声明硬触发却注入了虚拟触发：记录但不失败。
    // 第一阶段（ENG-08 §3）本就只有虚拟设备，若因此让 initialize()
    // 失败，则软件闭环无法运行；但完全不作声又会让"现场配置成硬触发、
    // 实际跑的是软触发"这一隐患无声存在。折中为：可用，但说清楚。
    if (config_.source == "hardware")
    {
        lastErrorText_ =
            "VirtualTriggerController 正在模拟硬触发（配置 source=\"hardware\"）。"
            "第一阶段无真实触发硬件（ENG-08 §3）；现场部署时须替换为 "
            "HardwareTriggerController，否则同步精度将低于 SYS-15 的误差预算要求。";
    }
}

bool VirtualTriggerController::initialize()
{
    initialized_ = true;
    return true;
}

bool VirtualTriggerController::enable()
{
    if (!initialized_)
    {
        // 未初始化即使能：拒绝。
        // 真实触发源未初始化时使能可能驱动硬件输出，
        // 而参数（极性、脉宽、输出通道）都未配置，属于危险动作。
        // 桩保持与真实实现一致的行为，否则"忘记 initialize()"这一
        // 缺陷在虚拟设备上不可见。
        lastErrorText_ = "VirtualTriggerController 未初始化即调用 enable()";
        return false;
    }

    enabled_ = true;
    return true;
}

bool VirtualTriggerController::trigger()
{
    if (!initialized_ || !enabled_)
    {
        lastError_.code        = data::kErrTriggerDegraded;  // 3001
        lastError_.message     = "触发未使能，本次触发未生效";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    if (failing_)
    {
        // 模拟失效：置 3001，语义为"触发失效，应降级为软触发"。
        // 沿用 3001 而不是另造错误码 —— ENG-09 §5.27 的错误码表是
        // 冻结的，表中 3000 段只有 3001 一项，不得自行增补
        // （增补需走 ENG-09 §8 的变更流程）。
        lastError_.code        = data::kErrTriggerDegraded;  // 3001
        lastError_.message     = "触发失效（模拟），应按 SYS-08 §7.5 降级为软触发";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    ++triggerCount_;
    return true;
}

void VirtualTriggerController::disable()
{
    // 可重复调用，无副作用（契约要求）。
    // 不重置 triggerCount_：计数是诊断信息，清零会让
    // "使能→触发若干次→关闭"这一完整周期的记录消失。
    enabled_ = false;
}

void VirtualTriggerController::simulateFailure(bool fail)
{
    failing_ = fail;
}

uint64_t VirtualTriggerController::triggerCount() const
{
    return triggerCount_;
}

bool VirtualTriggerController::enabled() const
{
    return enabled_;
}

std::string VirtualTriggerController::lastErrorText() const
{
    return lastErrorText_;
}

data::ErrorInfo VirtualTriggerController::lastError() const
{
    return lastError_;
}

}  // namespace device
}  // namespace aircraft
