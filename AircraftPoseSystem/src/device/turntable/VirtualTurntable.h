#pragma once

// ============================================================================
//  src/device/turntable/VirtualTurntable.h
//
//  依据：SYS-06 §12（VirtualTurntable）、§10（转台控制）
//        ENG-09 §5.9 / §5.10 / §6.3、ENG-01 §6
//
//  作用：无硬件时模拟方位角、俯仰角与运动状态。
//
//  ⚠ 与工作流文档 4.md 的偏离：
//  4.md 的 VirtualTurntable 只有一个 `TurntableState state_;` 成员，
//  move() 的语义未定义（无法判断是立即到位还是缓慢运动）。
//  本实现按**物理时间**模拟运动：
//    move() 依据 TurntableConfig 的 coarseSpeed/fineSpeed 计算行程时间，
//    记录预计到位时刻；state() 在时刻到达后从 MOVING 转为 STABLE。
//
//  为什么必须模拟时间而不能"move() 后立即 STABLE"：
//  SYS-08 §10 的收敛性测试要求验证
//    "ALIGN 恰好尝试 8 次后 FAILED，code == 2003，总耗时 ≤ 20 s"。
//  若虚拟转台零耗时到位，则 8 次 ALIGN 的总耗时接近 0，
//  该用例的"总耗时"判据失去意义，而且 STABILIZE 状态
//  （stabilizeTimeoutNs = 3.0 s，用于等振动衰减）永远无事可做 ——
//  一条在真机上必然走到的路径在测试中从未被执行过。
//
//  ⚠ 时刻来源可注入（setClock）：
//  SYS-08 §10 的测试需要把时间快进（否则每个用例都要真等 2.5 s × 8
//  次对准，测试套件会慢到没人愿意跑）。注入一个可控时钟使
//  "转台运动 1 秒"在测试中瞬间完成，而**生产代码仍用单调时钟**
//  （ENG-09 §2.5）。这与 MeasurementConfig::taskTimeoutNs 必须可注入
//  是同一个道理。
//
//  ⚠ 行程限位（TurntableConfig::azimuthMin/Max、elevationMin/Max）
//  必须真实生效：超限时返回 false 并置 ERROR 状态。
//  这是 SYS-08 §7.7 的"能力边界"类故障（code=2002）在虚拟设备上的
//  唯一复现手段 —— 而能力边界**不重试**，若虚拟设备不检查限位，
//  这条不重试的路径永远不会被测试覆盖。
// ============================================================================

#include <cstdint>
#include <functional>

#include "data/ErrorInfo.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableState.h"
#include "device/turntable/ITurntableController.h"

namespace aircraft
{
namespace device
{

/// 虚拟两轴转台（SYS-06 §12）。
class VirtualTurntable : public ITurntableController
{
public:
    /// @param config 转台配置（ENG-09 §6.3），提供行程限位与运动速度。
    explicit VirtualTurntable(const data::TurntableConfig& config);

    // ---- ITurntableController ----

    bool initialize() override;
    bool move(const data::TurntableCommand& command) override;
    data::TurntableState state() override;
    void stop() override;

    // ---- 具象类追加能力 ----

    /// 注入时钟，返回当前时刻（ns）。默认使用主机单调时钟
    /// （data::monotonicNowNs），见文件头说明。
    void setClock(std::function<uint64_t()> clock);

    /// 直接设置当前角度，**不经过运动过程**。
    /// 用途：测试准备阶段把转台摆到初始位置，避免每次都由 move() 走到位
    /// 而引入与用例无关的等待。
    /// 越限的初值会使 initialize() 之外的任何操作立即报 2002 ——
    /// 这是有意的：初始位置超限说明配置或使用方法有误。
    void setInitialAngles(double azimuth, double elevation);

    /// 最近一次失败原因（供上层填充 ErrorInfo）。
    data::ErrorInfo lastError() const;

private:
    /// 取当前时刻。封装注入的时钟，未注入时用主机单调时钟。
    uint64_t nowNs() const;

    data::TurntableConfig config_;

    /// 当前实际角度（单位 deg）。以 state() 返回的就是它 ——
    /// 不是命令值（见 ITurntableController::state() 的说明）。
    double azimuth_   = 0.0;  // deg
    double elevation_ = 0.0;  // deg

    data::TurntableMotionState motion_ = data::TurntableMotionState::IDLE;

    /// 正在执行的运动的目标角度（deg）与预计到位时刻（ns）。
    /// 二者只在 motion_ == MOVING 时有效。
    double   targetAzimuth_   = 0.0;  // deg
    double   targetElevation_ = 0.0;  // deg
    uint64_t arrivalNs_       = 0;

    /// 是否已成功 initialize()。
    bool initialized_ = false;

    data::ErrorInfo lastError_;

    std::function<uint64_t()> clock_;
};

}  // namespace device
}  // namespace aircraft
