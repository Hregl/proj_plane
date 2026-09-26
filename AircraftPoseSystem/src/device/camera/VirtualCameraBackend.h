#pragma once

// ============================================================================
//  src/device/camera/VirtualCameraBackend.h
//
//  依据：SYS-06 §4（相机抽象）、SYS-08 §10（收敛性测试要求桩）、
//        ENG-08 §3（第一阶段用虚拟相机完成软件闭环）
//        ENG-09 V2.3 §4.3 / §6.1、ENG-01 §6、裁决 C-01 v1.7
//
//  作用：无硬件时的相机替身。它使 M1 目标（"第一次完整编译运行闭环"）
//  可以在只有一块开发机的条件下达成：VirtualCamera → Device → Preview
//  → Qt 显示全链路可见。
//
//  ⚠ 与工作流文档 4.md 的偏离：
//  4.md 的 VirtualCameraBackend 只声明 `VirtualCameraBackend(CameraRole)`
//  与五个 override 方法，并"模拟 CAM25/50/100，输出 ImageFrame"——
//  但没有说明**图像从哪来**。若 grab() 返回一张未初始化的空图，
//  则 Preview 显示黑屏、Qt 界面无法验证、算法层的特征提取必然失败，
//  M1 的"闭环"就只是"没有崩溃"而非"跑通"。
//
//  因此本实现**真正合成图像**：渐变背景 + 与角色相关的目标矩形 +
//  帧号与角色文字叠加。这样：
//    · Preview 窗口能看到内容，且能看出画面在动（背景网格 + 帧号）；
//    · 三路画面的目标大小随焦距不同（100 mm 最大），
//      与 TargetScaleEstimate 的物理含义一致，评分链有可区分输入；
//    · 叠加的角色文字能立刻暴露"frame.cam25 里装的是 CAM100 的图"
//      这类错位 —— 仅靠数据字段无法察觉的缺陷，肉眼可见。
//
//  ⚠ 构造函数带配置（4.md 为仅带 CameraRole）：
//  合成图像必须有尺寸，且 cameraId / role / exposureTime 都要写进
//  ImageFrame（ENG-09 §5.5 的 7 个字段）。这些恰好都在 CameraConfig 中
//  （ENG-09 §6.1），故直接取配置而非另设参数 —— 也与真实后端
//  ImvCameraBackend 的构造方式保持一致，使两者可互换。
//
//  ⚠ 本批（011-A1）的格式声明：虚拟相机输出 **8 位 BGR**（`BGR8`），
//  原始载荷**可选**（`RawOptional`）且**不携带** `raw` 字节。
//  这不是偷懒：8 位格式下"显示图 = 原始数据"，Recorder 按 §4.1 的
//  第二分支照常保存（标 `data_source = image`），既有行为不变；
//  而 12 位格式必须携带原始载荷（`RawRequired`），虚拟相机不产生这种帧，
//  故不会出现"虚拟帧缺载荷却无人报错"的情形。
// ============================================================================

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "data/CameraConfig.h"
#include "data/CameraTriggerMode.h"
#include "data/DeviceIdentity.h"
#include "data/DeviceState.h"
#include "data/ImageFrame.h"
#include "data/OpStatus.h"
#include "device/camera/ICameraBackend.h"

namespace aircraft
{
namespace device
{

/// 虚拟相机后端：无硬件时合成图像（SYS-06 §4）。
class VirtualCameraBackend : public ICameraBackend
{
public:
    /// @param config 相机配置（ENG-09 §6.1），提供 cameraId / role /
    ///        合成图像尺寸 / 曝光时间。
    ///        width 或 height <= 0 时回退为 1280×1024 —— 虚拟相机的
    ///        分辨率是纯软件参数，没有"硬件真实值"可与之矛盾，
    ///        故回退是诚实的；真实后端 ImvCameraBackend 的尺寸则必须
    ///        来自设备（那里不允许回退）。
    explicit VirtualCameraBackend(const data::CameraConfig& config);

    /// 析构兜底调用 `close()`（幂等）—— 见 ICameraBackend 的说明。
    ~VirtualCameraBackend() override;

    // ---- ICameraBackend ----

    data::OperationResult initialize() override;
    data::OperationResult start() override;
    data::OperationResult stop() override;
    data::OperationResult close() override;
    data::GrabResult grab(data::ImageFrame& frame, uint32_t timeoutMs) override;
    data::OperationResult setTriggerMode(data::CameraTriggerMode mode) override;
    data::OperationResult triggerSoftware() override;
    data::DeviceIdentity deviceIdentity() const override;
    data::TriggerModeState triggerModeState() const override;
    /// 本类没有设备可失败，故**只在被模拟禁用或注入失败后**给出说明；
    /// 其余时刻返回空串。文本一律写明"模拟／注入"，**不得**写成
    /// "已确认断连"（本类不接任何 SDK，没有任何设备侧确认发生过）。
    std::string lastErrorText() const override;

    // ---- 具象类追加能力（测试与 M1 闭环用，见文件头）----

    /// 模拟相机断连。置 true 后 grab() 恒返回 `Disconnected`。
    ///
    /// 这是 SYS-08 §10 两个必测用例的实现手段：
    ///   · "硬件故障不重试"：中途将 CAM100 置为断连 → 应降级继续，
    ///     且**不消耗重试计数**，result.json 中 degraded=true；
    ///   · "降级下限"：断开两台相机 → 立即 FAILED，code=1001。
    /// 因此该能力不是测试便利，而是冻结测试表的必要前提。
    ///
    /// ⚠ 断连的**文本**必须写"虚拟后端的模拟禁用"，**不得**写成
    /// "已确认断连"：本类不接任何 SDK，没有任何设备侧的确认发生过。
    void simulateFault(bool faulty);

    /// 注入一次取帧失败及其分类（覆盖分类路径的测试用）。
    ///
    /// 用途（§4.2）：验证 `MultiCameraManager` 对**不同**失败分类的
    /// 处置差异 —— 例如 `Timeout` 不得禁用通道，而 `Disconnected` 要禁用。
    /// 传 `nullopt` 恢复正常取帧。
    /// ⚠ 允许注入 `Disconnected` 以外的任何分类；注入 `Disconnected`
    /// 等价于 `simulateFault(true)`，两条路径的文本不同（一个是"模拟禁用"，
    /// 一个是"注入的失败分类"），测试应各按各的文本断言。
    void setInjectedGrabStatus(std::optional<data::OpStatus> status);

    /// 设置合成目标相对**图像中心**的像素偏移（见 TargetOffset.h：
    /// 偏移量必须相对图像中心，否则对准判据恒真）。
    /// 默认 (0,0) 即目标已在中心。
    void setTargetPixelOffset(double dx, double dy);

    /// 设置设备时间戳相对主机时间戳的滞后，单位 ns。
    /// 用于验证 ImageFrame 的两个时间戳字段语义不同
    /// （ENG-09 §2.5：timestampNs 是主机接收时刻，
    ///   deviceTimestampNs 是设备曝光时刻）。
    void setDeviceLatencyNs(uint64_t ns);

    /// 当前设备状态（SYS-06 §14）。
    data::DeviceState state() const;

    /// 已产出的帧数（诊断用）。
    uint64_t frameCount() const;

    /// 收到的软件触发命令次数（`triggerSoftware()` 的成功调用次数）。
    /// 供"单执行者"用例断言"一次 capture() 内恰好 1 次"。
    uint64_t softwareTriggerCount() const;

    /// 收到的取帧调用次数（含失败）。供"预算耗尽 ⇒ 不发令也不取帧"
    /// 这类用例断言取帧次数为 **0**。
    uint64_t grabCallCount() const;

    /// 本后端**依次**收到的生命周期调用名（"initialize"/"start"/"stop"/"close"）。
    ///
    /// ⚠ 为什么需要它（011-A1 §4.4）：本批有两条断言在既有 API 上
    ///   **不可观测**，而它们各自对应一个真实缺陷：
    ///     ① **每个后端恰好初始化一次** —— 重复 `initialize()` 在本类上
    ///        完全幂等（终态相同、计数之外没有任何痕迹），而"同一个后端
    ///        被初始化两遍"正是 011-A1 从装配层修掉的那个缺陷
    ///        （`buildDevices()` 先逐路 `initialize()`、`initializeAll()`
    ///        又各自初始化一次）。没有痕迹，该缺陷就能悄悄回来。
    ///     ② **回滚的先后**（`stop()` 必须早于 `close()`）—— 两种顺序的
    ///        **终态相同**，只有调用序列能把它们分开。
    /// ⚠ 它**不参与任何判定**，只记录；也**不**在 `close()`／再次
    ///   `initialize()` 时清空 —— "看到了第二次 initialize"正是要留下的事实。
    /// ⚠ 跨**通道**的顺序（回滚时 cam100 → cam50 → cam25）不在本记录里：
    ///   每个对象只能看到自己的调用，故那一条由 `rollbackDevices()` 的
    ///   数组字面量固定，**没有**逐对象证据（用例里如实登记，不假装覆盖）。
    const std::vector<std::string>& lifecycleLog() const;

private:
    data::CameraConfig config_;

    data::DeviceState state_ = data::DeviceState::UNKNOWN;

    /// 帧号，自 0 单调递增。**不因 stop/start 而清零** ——
    /// 上层靠它做丢帧检测（SYS-06 §8 的"丢帧检测"），
    /// 若重启后归零，则一次 start/stop 循环会被误判为"帧号回退"。
    uint64_t frameId_ = 0;

    /// **请求**的触发模式（最近一次 setTriggerMode 的实参）。
    data::CameraTriggerMode requestedMode_ = data::CameraTriggerMode::FreeRun;

    /// **实际生效**的触发模式。本类的语义是"它就是设备"：
    /// 设置成功即生效，故与 requestedMode_ 通常一致；
    /// 分成两个成员是为了让"读回"这件事有独立的事实来源 ——
    /// 若读回直接抄 requested，则 `triggerModeState()` 的
    /// "不得回报请求值"这条纪律在本类上无从检验。
    data::CameraTriggerMode reportedMode_ = data::CameraTriggerMode::FreeRun;

    bool faulty_      = false;

    /// 注入的取帧失败分类；`nullopt` = 正常取帧。
    std::optional<data::OpStatus> injectedGrabStatus_;

    uint64_t deviceLatencyNs_ = 0;

    /// 合成目标的像素偏移（相对图像中心）。
    double targetOffsetX_ = 0.0;
    double targetOffsetY_ = 0.0;

    /// 实际使用的图像尺寸（在构造函数中由 config_ 解析并回退）。
    int width_  = 0;
    int height_ = 0;

    uint64_t softwareTriggerCount_ = 0;
    uint64_t grabCallCount_        = 0;

    /// 是否已 `close()`（用于幂等：已关闭时再 `close()` 直接返回 Ok）。
    bool closed_ = false;

    /// 生命周期调用序列（见 `lifecycleLog()` 的说明）。
    std::vector<std::string> lifecycleLog_;
};

}  // namespace device
}  // namespace aircraft
