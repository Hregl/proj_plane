#pragma once

// ============================================================================
//  src/data/TurntableSample.h
//
//  依据：SYS-10 §6.4（样本采集与到位判据）、§13（状态）、
//        评审 C2/C4（2026-10-19）、裁决 C-23
//
//  作用：一次轮询周期产出的**两轴同轮**样本。
//
//  ⚠ 为什么不能继续只发布两个角度原子量：
//    `azimuth_` 与 `elevation_` 各自更新时，一次丢包会让“新的方位角”
//    与“上一周期的俯仰角”同时出现在快照里。两个值都合法、都有限，
//    但**不属于同一时刻** —— 由它们算出的到位判据、或写入结果包的
//    转台状态，都是拼接出来的事实。本结构体用 cycleId + 采样时刻 +
//    valid 把“同一轮、同一时刻”变成可检查的不变量。
//
//  ⚠ valid=false 时 azimuth/elevation **没有意义**，不得被上层使用。
//    这不是“值为 0”，而是“本轮没有可信读数”。
// ============================================================================

#include <cstdint>

namespace aircraft
{
namespace data
{

/// 样本来源，便于结果包与日志区分“设备读数”与“本地默认值”。
enum class TurntableSampleSource : std::uint8_t
{
    None = 0,  ///< 尚无样本
    Poll = 1,  ///< 一次轮询周期内同时取得的两轴角
};

/// 同一轮询周期内同时取得的两轴角。
struct TurntableSample
{
    /// 方位角，单位 deg（ENG-09 §2.2）。
    double azimuth = 0.0;

    /// 俯仰角，单位 deg。
    double elevation = 0.0;

    /// 本轮样本的取得时刻，ns，CLOCK_MONOTONIC。
    /// 这是**主机侧完成两轴解析的时刻**，不是设备的曝光/采样时刻。
    std::uint64_t sampleTimestampNs = 0;

    /// 单调递增的轮询周期编号（从 1 开始）。两轴共享同一个编号。
    std::uint64_t cycleId = 0;

    /// 两轴是否都取得了地址正确、数值合法的回复。
    /// 任一轴缺失/非法/地址错 ⇒ false，且本轮的角值不得发布。
    bool valid = false;

    TurntableSampleSource source = TurntableSampleSource::None;
};

}  // namespace data
}  // namespace aircraft
