#pragma once

// ============================================================================
//  src/device/turntable/ITurntableController.h
//
//  依据：SYS-06 §10.1（接口冻结）、SYS-06 §13（转台控制流程）
//        ENG-09 §4.3、ENG-01 §6、裁决 C-05
//
//  作用：两轴（方位 / 俯仰）转台抽象。使对准逻辑（AlignmentController）
//  不必知道转台是 SDK、Peko_D 协议还是 RS485。
//
//  ⚠ 裁决 C-05：方法名取 `state()` 而非 `getState()`。
//  SYS-06 §10.1 写 `state()`，SYS-10 §5.1 写 `getState()`，两者冲突。
//  冻结以 SYS-06 的 **`state()`** 为准。
//  这不是风格问题：本工程其余接口一律不加 `get` 前缀
//  （`MultiCameraManager::capture`、`ITriggerController::trigger`），
//  若此处破例，则同一份代码里两种命名并存，读者无法从命名推断
//  某个方法是否属于同一族接口。
//
//  ⚠ 本接口**不含**"对准是否完成"的判据。判据在
//  TargetOffset + TurntableConfig::centerThreshold，
//  由 AlignmentController（application 层）计算 ——
//  device 层只回答"转台现在在哪、在动还是在停"。
//  这样切分的理由：对准是否可接受是**任务级**决策（SYS-08 §7.3 的
//  ALIGN 重试语义），而 device 层不知道任务的上下文，
//  它无法判断"还差 3 pixel"应该是继续微调还是接受。
//
//  ⚠ 归属偏离（4.md 的 `src/interfaces/`）与命名空间偏离
//  （4.md 的 `aircraft::interfaces`）：同 ICameraBackend.h 的详细说明。
//  接口签名逐字遵循 SYS-06 §10.1。
// ============================================================================

#include "data/TurntableCommand.h"
#include "data/TurntableState.h"

namespace aircraft
{
namespace device
{

/// 两轴转台控制接口（SYS-06 §10.1）。
class ITurntableController
{
public:
    virtual ~ITurntableController() = default;

    /// 初始化：建立通信链路、读取当前角度。
    /// @return 失败返回 false。转台**无冗余**（SYS-08 §7.5），
    ///         失败即整个测量失败（ErrorInfo{code=2001}），
    ///         不存在"少一个轴也能继续"的降级路径。
    virtual bool initialize() = 0;

    /// 移动到指定角度。
    ///
    /// ⚠ 命令值为**绝对角度**，单位 deg（ENG-09 §5.10），
    ///   不是相对增量。误按增量解释的后果是逐次累加 ——
    ///   表现为"转台朝一个方向越走越远，最终报超行程 2002"，
    ///   而第一次移动看起来完全正常。
    ///
    /// ⚠ 本方法**不阻塞到到位**。返回 true 仅表示"命令已被受理"。
    ///   是否到位须由调用方轮询 state() 的 motion 字段判断
    ///   （SYS-08 的 ALIGN → STABILIZE 两状态正是为此切分：
    ///    ALIGN 负责发命令，STABILIZE 负责等静止）。
    ///   若实现改为阻塞到到位，则 ALIGN 状态的超时
    ///   （MeasurementConfig::alignTimeoutNs = 2.5 s）会与机械运动时间
    ///   耦合，而机械时间不受软件控制 —— 一次大角度调转就可能超时。
    virtual bool move(const data::TurntableCommand& command) = 0;

    /// 查询当前状态（角度 + 运动状态）。
    ///
    /// ⚠ 这是**唯一**的角度来源。Command 只是"想去哪"，
    ///   实际位置必须以本方法为准 —— 若用命令值当作实际值，
    ///   则转台因超行程或机械卡滞而未执行时，软件会认为已经到位，
    ///   并在此错误位置上完成整个测量。
    virtual data::TurntableState state() = 0;

    /// 停止运动。用于任务取消（SYS-08 §7.3 的 9003 人工取消）
    /// 与异常退出路径。必须可重复调用。
    virtual void stop() = 0;
};

}  // namespace device
}  // namespace aircraft
