#pragma once

// ============================================================================
//  src/device/camera/VirtualCameraBackend.h
//
//  依据：SYS-06 §4（相机抽象）、SYS-08 §10（收敛性测试要求桩）、
//        ENG-08 §3（第一阶段用虚拟相机完成软件闭环）
//        ENG-09 §4.3、ENG-01 §6
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
// ============================================================================

#include <cstdint>

#include <opencv2/core.hpp>

#include "data/CameraConfig.h"
#include "data/DeviceState.h"
#include "data/ImageFrame.h"
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

    // ---- ICameraBackend ----

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool grab(data::ImageFrame& frame) override;
    bool setTriggerMode(bool enable) override;

    // ---- 具象类追加能力（测试与 M1 闭环用，见文件头）----

    /// 模拟相机断连。置 true 后 grab() 恒返回 false。
    ///
    /// 这是 SYS-08 §10 两个必测用例的实现手段：
    ///   · "硬件故障不重试"：中途将 CAM100 置为断连 → 应降级继续，
    ///     且**不消耗重试计数**，result.json 中 degraded=true；
    ///   · "降级下限"：断开两台相机 → 立即 FAILED，code=1001。
    /// 因此该能力不是测试便利，而是冻结测试表的必要前提。
    void simulateFault(bool faulty);

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

private:
    data::CameraConfig config_;

    data::DeviceState state_ = data::DeviceState::UNKNOWN;

    /// 帧号，自 0 单调递增。**不因 stop/start 而清零** ——
    /// 上层靠它做丢帧检测（SYS-06 §8 的"丢帧检测"），
    /// 若重启后归零，则一次 start/stop 循环会被误判为"帧号回退"。
    uint64_t frameId_ = 0;

    bool triggerMode_ = false;
    bool faulty_      = false;

    uint64_t deviceLatencyNs_ = 0;

    /// 合成目标的像素偏移（相对图像中心）。
    double targetOffsetX_ = 0.0;
    double targetOffsetY_ = 0.0;

    /// 实际使用的图像尺寸（在构造函数中由 config_ 解析并回退）。
    int width_  = 0;
    int height_ = 0;
};

}  // namespace device
}  // namespace aircraft
