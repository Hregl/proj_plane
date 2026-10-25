#pragma once

// ============================================================================
//  src/ui/TurntableControlView.h
//
//  作用：主程序“转台操作”面板的**值语义快照**。
//
//  ⚠ 本文件不包含 `ITurntableController`、`ITurntableDiagnostics` 或任何
//    device 头。ui 层拿到的只是一份已经算好的显示状态；判定“连接/受理/
//    交付/到位/故障”的逻辑全部在 app 层（ENG-01 §13：UI 不做转台控制）。
//
//  ⚠ 三个阶段必须保持分开，不得合并成一个“成功”：
//    1. 请求已受理 —— `move()` 返回 true，命令进入命令槽；
//    2. 指令已交付 —— 设备回显了两轴指令（`TurntableCommandOutcome::
//       Delivered`）；
//    3. 反馈已到位 —— 新鲜双轴样本进入到位容差并满足 STABLE。
//    停止同样分开：停止请求已发出 / 停止帧已交付 / 观测到静止 /
//    运动不确定。界面文字必须逐项显示，不能用“已停止”概括。
// ============================================================================

#include <string>

namespace aircraft
{
namespace ui
{

/// 连接/样本状态（只描述可观察事实）。
enum class TurntableConnectionState
{
    Unknown = 0,      ///< 尚未初始化或没有快照
    WaitingSample,    ///< 已初始化，尚无首轮有效样本
    Connected,        ///< 有新鲜有效双轴样本
    Stale,            ///< 样本过期（可能失联）
    Fault             ///< 错误锁存或运动不确定
};

/// 最近一条操作员定位命令的阶段。
enum class TurntableCommandStage
{
    None = 0,       ///< 本次会话尚未发过命令
    Accepted,       ///< 已受理（命令进入命令槽）
    Delivered,      ///< 两轴指令已交付（收到回显）
    Arrived,        ///< 反馈已到位（新鲜样本 + STABLE + 容差）
    Rejected,       ///< 发送前被门禁/预检拒绝
    Failed,         ///< 发送/回显失败，或转台故障
    Superseded,     ///< 被更新的命令覆盖
    Cancelled,      ///< 被停止屏障取消
    Timeout,        ///< 未在期限内到位（已按现有收口请求停止）
    LeftTolerance   ///< 曾到位，之后反馈偏离到位容差
};

/// 最近一次停止请求的可观察结果。
enum class TurntableStopStage
{
    None = 0,          ///< 本次会话尚未请求停止
    Requested,         ///< 停止请求已交给控制器（尚在处理）
    FrameDelivered,    ///< 停止帧已交付传输层
    ObservedStationary,///< 停止后连续新鲜样本满足静止判据
    Uncertain,         ///< 停止未交付或未观测到静止
    NoFrame            ///< 控制器未发送停止帧（只读/无在途运动）
};

/// 转台操作面板快照。
struct TurntableControlView
{
    bool available = false;        ///< 装配出转台并已初始化

    // ---- 连接与反馈 ----
    TurntableConnectionState connection = TurntableConnectionState::Unknown;
    std::string connectionText;    ///< 人读：已连接 / 等待首轮样本 / 样本过期 / 故障锁存
    std::string connectionDetail;  ///< 补充：故障码/原因，不替代状态

    bool hasSample = false;
    bool sampleValid = false;
    bool sampleFresh = false;
    double azimuth = 0.0;
    double elevation = 0.0;

    std::string motionText;        ///< IDLE / MOVING / STABLE / ERROR
    bool errorLatched = false;
    bool motionUncertain = false;
    std::string faultText;         ///< 首因故障（空 = 无）

    // ---- 许可与操作条件 ----
    bool motionEnabled = false;    ///< 当前是否允许新的运动命令
    bool positioningEnabled = false;
    bool stopEnabled = false;
    bool busy = false;             ///< 上一条定位或停止尚未结束
    std::string permitText;        ///< 许可/只读/虚拟转台说明
    std::string gateText;          ///< 不能定位时的原因（空 = 可定位）

    // ---- 命令阶段 ----
    TurntableCommandStage commandStage = TurntableCommandStage::None;
    std::string commandStageText;
    std::string commandDetail;
    bool hasTarget = false;
    double targetAzimuth = 0.0;
    double targetElevation = 0.0;

    // ---- 停止阶段 ----
    TurntableStopStage stopStage = TurntableStopStage::None;
    std::string stopStageText;
    std::string stopDetail;

    // ---- 操作反馈 ----
    std::string lastActionText;    ///< 最近一次按钮操作的结果说明

    // ---- 输入范围（来自配置；仅用于限制输入，不构成运动授权）----
    double azimuthMin = -180.0;
    double azimuthMax = 180.0;
    double elevationMin = -60.0;
    double elevationMax = 60.0;
};

}  // namespace ui
}  // namespace aircraft
