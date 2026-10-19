#pragma once

// ============================================================================
//  src/device/turntable/ITurntableDiagnostics.h
//
//  依据：裁决 C-23（2026-10-19）
//
//  作用：与冻结的 `ITurntableController` **并列**的诊断接口。
//
//  ⚠ 为什么不给 `ITurntableController` 增加 `lastError()`/`diagnostics()`：
//    该接口的方法面在 SYS-06 中冻结（`initialize()/move()/state()/stop()`）。
//    诊断是“可信化”新引入的能力，用并列接口承载，生产者（真实后端）
//    实现它，消费者（装配点、测量总控）按可选指针接收 —— 虚拟后端、
//    测试桩都不必被迫实现。这样既不破坏冻结签名，也不让“没有诊断能力”
//    看起来像“没有故障”。
// ============================================================================

#include "data/TurntableDiagnostics.h"

namespace aircraft
{
namespace device
{

class ITurntableDiagnostics
{
public:
    virtual ~ITurntableDiagnostics() = default;

    /// 取一致快照。实现方**不得**在此做 I/O 或等待 I/O 锁。
    virtual data::TurntableDiagnostics diagnostics() const = 0;
};

}  // namespace device
}  // namespace aircraft
