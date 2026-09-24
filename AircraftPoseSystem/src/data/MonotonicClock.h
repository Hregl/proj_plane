#pragma once

// ============================================================================
//  src/data/MonotonicClock.h
//
//  依据：ENG-09 §2.5（时间戳冻结规则）、§5.27（ErrorInfo::timestampNs）
//        SYS-08 §7.6（RetryManager 的 nowNs 入参）、SYS-09 §13（同步）
//
//  ⚠ 本文件是**本工程对 ENG-01 §5 文件清单的一处补充**，
//  理由如下（不是随手加的便利函数）：
//
//  ENG-09 §2.5 冻结了三条规则：
//    1. 时间戳一律为主机 CLOCK_MONOTONIC，单位 ns；
//    2. **禁止使用墙钟**做同步判断；
//    3. MultiCameraFrame::triggerTimestamp 取各相机 timestampNs 的最大值。
//
//  这三条规则要被遵守，前提是**全工程只有一个取时刻的地方**。
//  若各处自行调用 `std::chrono::system_clock::now()` 与
//  `steady_clock::now()`（甚至 `clock_gettime(CLOCK_REALTIME)`），
//  那么：
//    · 规则 2 只能靠人工审查保证，编译器与测试都无法发现违规；
//    · 更危险的是**混合使用**：一处墙钟、一处单调钟，两者的读数
//      不可比较，而比较它们的代码（三相机同步判据、
//      RetryManager::deadlineExceeded）不会报错，
//      只会给出无意义的结果 —— 且该结果随 NTP 是否校时而变化，
//      表现为"偶发的、无法复现的同步失败"。
//
//  因此把取时刻这一动作集中到本文件：审查"是否使用了墙钟"只需看这一个
//  文件（本文件不含任何墙钟调用），而不必搜索全工程。
//
//  ⚠ 放在 data 层的原因：data 是依赖链的最底层（ENG-01 §17），
//  device / optical / application / infrastructure 全部可达它。
//  若放在 infrastructure，则 device 无法使用（ENG-03 §12.3 未给 device
//  任何 infrastructure 依赖），而 device 恰恰是最需要打时间戳的层
//  （相机采集时刻）。这正是裁决 C-21"跨层接口落在最低公共层"的同一原理。
//
//  ⚠ 本文件只含内联函数，不引入新的编译单元 —— 因此不改变
//  "data 的唯一 TU 是 ErrorInfo.cpp"这一事实。
// ============================================================================

#include <cstdint>

#include <time.h>

namespace aircraft
{
namespace data
{

/// 取主机单调时钟当前时刻，单位 ns。
///
/// 使用 `CLOCK_MONOTONIC`（ENG-09 §2.5 冻结）。
///
/// 为什么不用 `std::chrono::steady_clock`：
/// 语义上二者等价（都单调，都不受校时影响），但本工程需要与
/// POSIX 接口（相机 SDK 回报的设备时间戳、`pthread` 超时）
/// 直接比较。`clock_gettime` 的读数与这些来源同源，
/// 而 `steady_clock` 的 epoch 未定义、可能带一个任意的起点偏移 ——
/// 若两者混用，就会出现一个恒定的时间偏移量，它不会被任何断言捕获，
/// 却会让"设备时间戳与主机时间戳之差"这一本该接近 0 的量变成某个常数。
///
/// 单调时钟的另外两个好处：
///   · 不受 NTP 校时阶跃影响（这正是 §2.5 禁用墙钟的原因）；
///   · 不受人工改时影响（靶场环境常见）。
inline uint64_t monotonicNowNs()
{
    struct timespec ts {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace data
}  // namespace aircraft
