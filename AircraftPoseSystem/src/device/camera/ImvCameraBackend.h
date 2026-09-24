#pragma once

// ============================================================================
//  src/device/camera/ImvCameraBackend.h
//
//  依据：SYS-06 §7（ImvCameraBackend）、ENG-09 §4.3、ENG-01 §6
//        ENG-08 §3 / §11（第一阶段范围）
//
//  作用：华睿 A7A20MU201 的 SDK 适配层。
//
//  ⚠ 本阶段的实现状态：**诚实桩**。
//  当前构建中 `APS_HAVE_IMVSDK` 未定义（SDK 未安装，见 Dependencies.cmake），
//  initialize() 返回 false 并给出可操作的提示。这不是未完成，
//  而是 ENG-08 §3 规定的第一阶段范围（虚拟设备完成软件闭环）。
//
//  ⚠ 为什么不用"假装成功"的桩：
//  若 initialize() 返回 true 却不真正采集，则 MultiCameraManager 会把
//  该通道计入"可用相机数"（SYS-08 §7.5），于是可用数虚高为 3、
//  降级判据失效，而 grab() 失败又会被当成"采集途中掉线"触发降级 ——
//  两条路径互相矛盾，最终表现为"明明三台都在却反复降级"。
//  返回 false 使"这台相机不可用"成为**唯一且真实**的事实。
//
//  ⚠ 三个实例，一个类（SYS-06 §7）：
//  该节写"三个实例：ImvCameraBackend25 / 50 / 100"，而 ENG-09 §4.3
//  的冻结类清单只有 `ImvCameraBackend` 一个类。二者并不冲突 ——
//  "三个实例"指的是**实例数量**而非三个类。本工程按 ENG-09 §4.3
//  实现单一类，由 CameraConfig 区分角色（与 VirtualCameraBackend 同构，
//  使二者可互换，见 VirtualCameraBackend.h 的说明）。
//  若真按三个类实现，则三段代码的 99% 是同一份 SDK 调用序列，
//  任何 SDK 调用顺序的修正都要改三处并分别验证。
// ============================================================================

#include <cstdint>
#include <string>

#include "data/CameraConfig.h"
#include "data/DeviceState.h"
#include "data/ImageFrame.h"
#include "device/camera/ICameraBackend.h"

namespace aircraft
{
namespace device
{

/// 华睿相机 SDK 适配（SYS-06 §7）。
class ImvCameraBackend : public ICameraBackend
{
public:
    /// @param config 相机配置（ENG-09 §6.1）。真实后端**不使用**
    ///        width/height 的回退值 —— 画幅由设备回报，
    ///        配置中的尺寸仅用于与该回报值比对（不一致即报错，
    ///        因为内参矩阵是按标定时的分辨率标定的，见
    ///        CameraCalibration.h 的说明）。
    explicit ImvCameraBackend(const data::CameraConfig& config);

    // ---- ICameraBackend ----

    bool initialize() override;
    bool start() override;
    void stop() override;
    bool grab(data::ImageFrame& frame) override;
    bool setTriggerMode(bool enable) override;

    // ---- 诊断 ----

    /// 最近一次失败的原因（人读）。用于日志与 UI，
    /// **不参与任何控制流判断** —— 控制流一律以返回值为准。
    std::string lastErrorText() const;

    data::DeviceState state() const;

private:
    data::CameraConfig config_;
    data::DeviceState  state_ = data::DeviceState::UNKNOWN;
    std::string        lastErrorText_;

    /// SDK 设备句柄。保持 `void*` 而不引入 SDK 类型：
    /// 本头文件会被 device 层的其它文件包含，若在此处 include SDK 头，
    /// 则所有包含者都被迫依赖 SDK 的头文件路径 —— 而 SDK 是可选依赖
    /// （ENG-03 §12.3），一旦缺失，整个 device 模块将无法编译，
    /// 连虚拟相机也用不了。`void*` 把 SDK 类型限制在 .cpp 内部。
    void* handle_ = nullptr;

    uint64_t frameId_ = 0;
    bool     triggerMode_ = false;
};

}  // namespace device
}  // namespace aircraft
