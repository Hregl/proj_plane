#pragma once

// ============================================================================
//  src/data/PreviewFrame.h
//
//  依据：ENG-09 §4.1、§5.7（类型定义冻结）
//
//  作用：预览通路上的一帧。它是 SYS-09 §2.1 双流水线隔离的**载体** ——
//  预览流水线与测量流水线共享同一份采集结果，但预览多带一个显示时刻。
//
//  为什么预览帧要单独包一层，而不是直接传 ImageFrame：
//  显示时刻与采集时刻是两个不同的量。PreviewQueue 采用丢旧保新策略
//  （SYS-09 §13.1），若用采集时刻衡量延迟，则"队列里积压了 3 帧"这件事
//  不可见；带上 displayTimestamp 之后，PreviewWorker 可以算出
//  displayTimestamp - frame.timestampNs 即端到端延迟，这正是 SYS-07 §15
//  实时性预算中"预览不得阻塞算法"这条约束的观测量。
//  若直接传 ImageFrame，这个量只能靠再次调用单调时钟临时求差，
//  在丢帧情形下会得到错误（负的或跳变的）延迟。
// ============================================================================

#include <cstdint>

#include "data/ImageFrame.h"

namespace aircraft
{
namespace data
{

/// 预览通路上的一帧。
struct PreviewFrame
{
    /// 图像帧本体（含采集时间戳与焦段角色）。
    ImageFrame frame;

    /// 进入预览队列的主机单调时钟时刻，单位 ns（CLOCK_MONOTONIC）。
    /// 与 frame.timestampNs 求差即为该帧的预览时延。
    uint64_t displayTimestamp = 0;
};

}  // namespace data
}  // namespace aircraft
