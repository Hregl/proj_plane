// ============================================================================
//  src/device/turntable/PekoTurntableController.cpp
//
//  依据：SYS-06 §11、ENG-08 §11、ENG-09 §5.9 / §5.10
//
//  诚实桩：每个方法明确失败并说明原因，绝不假装运动（理由见头文件）。
//
//  ⚠ 本文件**不**包含 Peko_D 协议的任何报文构造代码。
//  协议报文格式未在冻结文档中给出（SYS-10 只列了支持的协议种类，
//  未给帧结构），凭空实现一段"看起来对"的报文会让读者以为
//  该协议已接入，而实际字节序、校验方式、应答格式都无从验证。
//  真正的接入必须依据现场 SDK 文档，属于 ENG-08 第二阶段。
// ============================================================================

#include "device/turntable/PekoTurntableController.h"

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

PekoTurntableController::PekoTurntableController(const data::TurntableConfig& config)
    : config_(config)
{
    // 把配置里声明的协议记入诊断信息：使"配置了 network 协议但本类
    // 尚未实现"这一事实在日志中直接可见，而不是靠人对照 ENG-08 §11
    // 的范围表去推断。
    lastErrorText_ = "PekoTurntableController 未接入真实转台："
                     "ENG-08 §11 规定第一阶段不实现 Peko_D / RS485 / 网络控制。"
                     "配置声明的协议为 \"" + config_.protocol + "\"。"
                     "第一阶段请使用 VirtualTurntable。";
}

bool PekoTurntableController::initialize()
{
    lastError_.code        = data::kErrTurntableComm;  // 2001
    lastError_.message     = lastErrorText_;
    lastError_.timestampNs = data::monotonicNowNs();
    return false;
}

bool PekoTurntableController::move(const data::TurntableCommand& command)
{
    // 记录命令仅为诊断（见头文件），不产生任何运动。
    lastCommand_ = command;

    lastError_.code        = data::kErrTurntableComm;  // 2001
    lastError_.message     = "转台未接入：" + lastErrorText_;
    lastError_.timestampNs = data::monotonicNowNs();
    return false;
}

data::TurntableState PekoTurntableController::state()
{
    // 恒返回 IDLE 且角度为 0。
    //
    // ⚠ 绝不回放 lastCommand_ 作为"当前位置"：那会让 ALIGN 的
    //   对准判据看到"已经在目标角度上"，从而报告对准成功 ——
    //   于是一台根本没接的转台会让整个测量流程继续走下去，
    //   直到给出一个基于错误指向的姿态结果。返回 0 且 IDLE
    //   使上层立刻看出"转台不在工作"。
    data::TurntableState s;
    s.azimuth   = 0.0;    // deg
    s.elevation = 0.0;    // deg
    s.motion    = data::TurntableMotionState::IDLE;
    return s;
}

void PekoTurntableController::stop()
{
    // 桩无可停止之物。保持幂等（契约要求）。
}

std::string PekoTurntableController::lastErrorText() const
{
    return lastErrorText_;
}

data::ErrorInfo PekoTurntableController::lastError() const
{
    return lastError_;
}

}  // namespace device
}  // namespace aircraft
