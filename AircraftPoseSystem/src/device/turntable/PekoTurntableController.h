#pragma once

// ============================================================================
//  src/device/turntable/PekoTurntableController.h
//
//  依据：SYS-06 §11（PekoTurntableController）、ENG-09 §4.3、ENG-01 §6
//        ENG-08 §11（第一阶段不实现 Peko_D / RS485 / 网络控制）
//
//  作用：真实转台 SDK 适配（Peko_D 协议 / RS485 / 百兆网络）。
//
//  ⚠ 本阶段的实现状态：**诚实桩**（与 ImvCameraBackend 同策略）。
//  ENG-08 §11 明确规定第一阶段不实现该协议，故 initialize() 返回 false
//  并说明原因。详见 ImvCameraBackend.h 中"为什么不用假装成功的桩"
//  ——那里的两条理由在此处完全适用，且后果更重：
//
//  转台**无冗余**（SYS-08 §7.5）。若本类假装成功，则：
//    · 可用性判据认为转台正常，不会走 2001 失败路径；
//    · 而 move() 实际不产生任何机械运动；
//    · 结果是对准算法在永远不动的转台上反复下发命令，
//      直到耗尽 8 次 ALIGN 重试（SYS-08 §7.3），报 code=2003
//      "对准重试次数用尽"——**错误码指向算法，根因却在设备桩**。
//  返回 false 使 2001 在启动阶段就被报出，故障定位直接。
//
//  ⚠ 协议取值（TurntableConfig::protocol = "pekod" | "rs485" | "network"）
//  在此处读取并记录到诊断信息中，使"配置了哪种协议却未实现"这件事
//  在运行日志里可见，而不是靠人回忆 ENG-08 §11 的范围约定。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <string>

#include "data/ErrorInfo.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableState.h"
#include "device/turntable/ITurntableController.h"

namespace aircraft
{
namespace device
{

/// 真实转台 SDK 适配（SYS-06 §11）。当前为诚实桩。
class PekoTurntableController : public ITurntableController
{
public:
    /// @param config 转台配置（ENG-09 §6.3）。protocol 字段用于诊断输出。
    explicit PekoTurntableController(const data::TurntableConfig& config);

    // ---- ITurntableController ----

    bool initialize() override;
    bool move(const data::TurntableCommand& command) override;
    data::TurntableState state() override;
    void stop() override;

    // ---- 诊断 ----

    /// 最近一次失败原因（人读），供日志与 UI 使用。
    std::string lastErrorText() const;

    data::ErrorInfo lastError() const;

private:
    data::TurntableConfig config_;
    std::string           lastErrorText_;
    data::ErrorInfo       lastError_;

    /// 最后一次命令记录。桩不执行运动，但在 state() 中回放该命令
    /// **是不允许的** —— 那会让上层误以为转台动了。
    /// 故此处只保留用于诊断，state() 恒返回 IDLE 且角度为 0。
    data::TurntableCommand lastCommand_;
};

}  // namespace device
}  // namespace aircraft
