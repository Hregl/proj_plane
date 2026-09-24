#pragma once

// ============================================================================
//  src/data/DeviceState.h
//
//  依据：ENG-09 §4.1、裁决 C-13
//        SYS-06 §14（设备状态）
//
//  裁决 C-13：原文档未定归属，冻结置于 src/data/。理由同 MeasurementState.h：
//  device 层是生产者，application / infrastructure / ui 三层都是消费者，
//  枚举留在 device 会让后两者反向依赖设备层。UI 要在 StatusPanel 显示
//  "转台状态 / 相机状态"，这条依赖若成立，UI 就被钉在设备层上，
//  ENG-01 §18 第一条（UI 不得访问 SDK）将无法在构建期强制。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 单个设备（相机 / 转台 / 触发）的生命周期状态。
enum class DeviceState
{
    UNKNOWN,  ///< 尚未探测（启动初值）
    INIT,     ///< 已创建，正在初始化（SDK 打开、参数下发）
    READY,    ///< 初始化完成、尚未开始取流/运动
    RUNNING,  ///< 正常工作
    ERROR     ///< 故障。SYS-08 §7.2 规定硬件故障**不重试**，直接按 §7.5 降级或 FAILED
};

}  // namespace data
}  // namespace aircraft
