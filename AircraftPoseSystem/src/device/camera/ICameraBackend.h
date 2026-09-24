#pragma once

// ============================================================================
//  src/device/camera/ICameraBackend.h
//
//  依据：SYS-06 §6.1（接口冻结）、ENG-09 §4.3（类清单冻结）
//        ENG-01 §6（device 文件清单）
//
//  作用：屏蔽具体 SDK。MultiCameraManager 只面对本接口，
//  因此"换成另一家相机"或"用虚拟相机跑测试"都不触及上层代码。
//
//  ⚠ 归属偏离（工作流文档 4.md 的 `src/interfaces/`）：
//  4.md 把四个设备接口集中在 `src/interfaces/`，本工程**不采纳**，
//  接口与实现同目录（`device/camera/`、`device/turntable/`、`device/trigger/`）。
//
//  不采纳的理由（与 001 阶段已登记的偏离 #1 同源）：
//    1. ENG-01 §6 的文件清单里**没有** `src/interfaces/` 这一级 ——
//       而 ENG-01 是目录结构的权威文档（ENG-01 §1 的权威性规则）。
//       新增一级目录会让"目录结构以 ENG-01 为准"这一约定在第一个
//       实现阶段就失效。
//    2. 集中式接口目录会使**跨层接口**失去层级归属。本工程上游的
//       `IMatchStatsStore`（裁决 C-21）正因如此才按"依赖关系的最低公共层"
//       落在 `data/`；若另设 interfaces/ 集中层，同一问题会出现两种解法。
//    3. 命名空间与目录路径保持一致（`device/camera/ICameraBackend.h`
//       → `aircraft::device::ICameraBackend`）后，读者由 include 路径即可
//       推知所属层，而 `interfaces::ICameraBackend` 这条路径无法回答
//       "这个接口属于哪一层"。
//
//  ⚠ 命名空间偏离：4.md 用 `aircraft::interfaces`。本工程用 `aircraft::device`
//  —— 全套冻结文档均**未定义**命名空间约定（ENG-09、ENG-03 中无相关条款），
//  故此处按"命名空间 = 目录路径"的工程惯例确定。接口与其实现同命名空间，
//  使 `ICameraBackend` 与 `VirtualCameraBackend` 无需跨命名空间引用。
//
//  接口签名**逐字**遵循 SYS-06 §6.1，未增删任何方法 ——
//  偏离仅涉及**文件位置与命名空间**，不涉及接口契约。
// ============================================================================

#include "data/ImageFrame.h"

namespace aircraft
{
namespace device
{

/// 单台相机的 SDK 适配接口（SYS-06 §6.1）。
///
/// 生命周期（SYS-09 §3 的设备线程约定）：
///     initialize() → start() → grab() … → stop()
/// 本接口的实现**不负责线程安全**：每个 backend 归其所属设备线程独占，
/// 不允许多线程并发调用同一实例（SYS-09 §14 的数据所有权规则）。
class ICameraBackend
{
public:
    virtual ~ICameraBackend() = default;

    /// 初始化：加载 SDK、连接设备、配置参数。
    /// @return 失败返回 false；调用方据此将该通道标记为不可用
    ///         （SYS-08 §7.5 的降级计数以"可用相机数"为准）。
    virtual bool initialize() = 0;

    /// 开始采集（进入自由运行或等待触发）。
    virtual bool start() = 0;

    /// 停止采集。允许在未 start() 时调用（须无副作用）。
    virtual void stop() = 0;

    /// 取一帧图像，填入 frame。
    ///
    /// 实现须填满以下字段（ENG-09 §5.5 的 7 个字段）：
    ///     image / frameId / timestampNs / deviceTimestampNs /
    ///     cameraId / role / exposureTime
    ///
    /// ⚠ timestampNs 必须取**主机 CLOCK_MONOTONIC**（ENG-09 §2.5），
    ///   禁止墙钟 —— 墙钟受 NTP 校时阶跃影响，会制造虚假的同步偏差，
    ///   而三相机同步判据（TriggerConfig::syncToleranceNs）正是建立在此
    ///   字段之上。
    ///
    /// @return 失败返回 false（超时、断连、未 start）。frame 内容在
    ///         返回 false 时未定义，调用方不得使用。
    virtual bool grab(data::ImageFrame& frame) = 0;

    /// 设置触发模式：true = 硬触发，false = 软触发。
    /// 硬触发失效时上层按 SYS-08 §7.5 降级为软触发并置
    /// kErrTriggerDegraded(3001)。
    virtual bool setTriggerMode(bool enable) = 0;
};

}  // namespace device
}  // namespace aircraft
