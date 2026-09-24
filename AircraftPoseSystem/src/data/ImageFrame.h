#pragma once

// ============================================================================
//  src/data/ImageFrame.h
//
//  依据：ENG-09 §4.1、§5.5（类型定义冻结）、§2.5（时间戳）、裁决 C-01
//
//  裁决 C-01：该类型在三份文档中字段不一致：
//      SYS-05 §5.1 | image frameId timestampNs cameraId role exposureTime
//      ENG-02 §4.1 | image frameId timestampNs cameraId role
//      ENG-04 §4.1 | image frameId timestampNs role
//  冻结为上表**并集**，并新增 deviceTimestampNs。
//
//  ⚠ 两个时间戳语义不同，不得混用（ENG-09 §2.5，冻结）：
//      timestampNs       主机 CLOCK_MONOTONIC，采集线程在 grab() 返回后立即打点
//                        → 同步校验与时延统计的**唯一**基准
//      deviceTimestampNs 相机 SDK 上报的相机内部时间戳（无则 0）
//                        → **仅**用于诊断跨相机时钟偏移
//
//  为什么禁止用墙钟（CLOCK_REALTIME）做同步判断：本系统需长时间连续运行
//  且支持离线部署，墙钟可能被 NTP 或人工校时**跳变**（包括向后跳）。
//  用墙钟做三相机同步判断，一次校时就会让某一帧的时差变成 ±数十秒，
//  触发误判的丢帧告警；而单调时钟只前进，时差恒为正值且有界。
//
//  本结构体在 SYS-09 §14 中的跨线程传递方式为 shared_ptr<const ImageFrame>：
//  cv::Mat 的拷贝是引用计数（浅拷贝），若不加 const，下游修改像素会污染
//  上游仍持有的帧。固定尺寸字段（frameId 等）则随结构体一起复制，代价可忽略。
// ============================================================================

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "data/CameraRole.h"

namespace aircraft
{
namespace data
{

/// 单相机的一帧图像及其采集元数据。
struct ImageFrame
{
    /// 图像数据。通道数 1（灰度）或 3（BGR），深度 8U。
    cv::Mat image;

    /// 帧序号，由采集端单调递增。仅用于诊断"是否丢帧"，不用于同步。
    uint64_t frameId = 0;

    /// **主机单调时钟**时间戳，单位 ns（CLOCK_MONOTONIC）。
    /// 采集线程在 grab() 返回后立即打点。同步校验的唯一基准（ENG-09 §2.5）。
    uint64_t timestampNs = 0;

    /// 相机 SDK 上报的内部时间戳，单位 ns；不可用时置 0。
    /// 仅用于诊断跨相机时钟偏移，**不得**参与同步判断（ENG-09 §2.5）。
    uint64_t deviceTimestampNs = 0;

    /// 拍摄该帧的相机标识，与 CameraChannel.cameraId 对应。
    std::string cameraId;

    /// 该帧所属的焦段角色。
    CameraRole role = CameraRole::CAM25;

    /// 曝光时间，单位 **s**（不是 ms，也不是 us）。
    double exposureTime = 0.0;
};

}  // namespace data
}  // namespace aircraft
