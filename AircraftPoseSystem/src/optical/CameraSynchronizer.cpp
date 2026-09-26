// ============================================================================
//  src/optical/CameraSynchronizer.cpp
//
//  依据：SYS-06 §8、SYS-08 §7.5、ENG-09 §2.5 / §5.6 / §6.4
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "optical/CameraSynchronizer.h"

#include <algorithm>

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace optical
{

namespace
{
/// SYS-08 §7.5：可用相机数 <2 时无法继续测量（code=1001）。
constexpr int kMinPresentFrames = 2;
}  // namespace

CameraSynchronizer::CameraSynchronizer(const data::TriggerConfig& config)
    : config_(config)
{
}

bool CameraSynchronizer::synchronize(const data::ImageFrame& cam25,
                                     const data::ImageFrame& cam50,
                                     const data::ImageFrame& cam100,
                                     data::MultiCameraFrame& output)
{
    lastError_       = data::ErrorInfo{};
    lastSpreadNs_    = 0;
    lastPresentCount_ = 0;
    lastFrameDrop_   = false;

    const data::ImageFrame* inputs[3] = {&cam25, &cam50, &cam100};

    // ---- 1 存在性 + 帧号连续性 ----
    uint64_t minTs = 0;
    uint64_t maxTs = 0;
    bool     first = true;

    for (int i = 0; i < 3; ++i)
    {
        const data::ImageFrame& f = *inputs[i];

        // 以 image 为空判定该路不可用 —— 与 device 层
        // （MultiCameraManager::capture 不填不可用通道）的约定一致。
        if (f.image.empty())
        {
            // 不可用的通道要清掉连续性记录：否则它恢复后，
            // 帧号与断连前的旧值比较会显示一个巨大的跳跃，
            // 而这个"丢帧"其实是断连期间本就没采集。
            hasPrev_[i] = false;
            continue;
        }

        ++lastPresentCount_;

        if (first)
        {
            minTs = f.timestampNs;
            maxTs = f.timestampNs;
            first = false;
        }
        else
        {
            minTs = std::min(minTs, f.timestampNs);
            maxTs = std::max(maxTs, f.timestampNs);
        }

        // 丢帧检测（SYS-06 §8）：帧号应逐帧 +1。
        if (hasPrev_[i] && f.frameId != prevFrameId_[i] + 1)
        {
            lastFrameDrop_ = true;
        }
        prevFrameId_[i] = f.frameId;
        hasPrev_[i]     = true;
    }

    // ---- 2 路数判据（SYS-08 §7.5）----
    if (lastPresentCount_ < kMinPresentFrames)
    {
        lastError_.code        = data::kErrCameraInsufficient;  // 1001
        lastError_.message     = "可用于同步的图像不足 2 路";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    // ---- 3 同步判据 ----
    lastSpreadNs_ = maxTs - minTs;

    // ⚠ syncToleranceNs 为 0 视为"未配置"，跳过判据并记录，而不是
    //   按"容差 0"严格执行。
    //
    //   若按 0 执行，则极差必须恰为 0 才算同步 —— 现实中三台相机
    //   的时间戳不可能逐纳秒相同，于是**每一轮都失败**，
    //   表现为"系统完全无法测量"，而根因只是一处未填的配置。
    //   ENG-10 §5.3 对"字段缺失"的规定是"默认值 + 记日志"，
    //   这里遵循该规定，并把"跳过了判据"这一事实留在 message 中，
    //   使配置疏漏可见而不是静默放行。
    if (config_.syncToleranceNs == 0)
    {
        lastError_.code        = 0;  // 非错误，仅是配置疏漏的记录
        lastError_.message     =
            "TriggerConfig::syncToleranceNs 未配置（为 0），本轮跳过同步判据。"
            "该配置缺失会使三相机时间戳超差无法被发现 —— "
            "超差的帧会含有一个虚假的角速度贡献，直接进入姿态结果。";
        lastError_.timestampNs = data::monotonicNowNs();
    }
    else if (lastSpreadNs_ > config_.syncToleranceNs)
    {
        // 3002（裁决 C-006 新增）：三路时间戳超差。
        //
        // ⚠ 此前这里保持 code=0，理由是"3000 段没有这一码"。0 的冻结语义是
        // "未设置"、渲染为 "OK"，于是**一次真实的同步超差**在包里与
        // "没有错误"无法区分。C-006 为该事件登记了 3002。
        //
        // 语义上它不与 9004（兜底）同码：超差是**可重试的瞬态**，
        // 下一位姿的同一组三路帧可能就落在容差内；而 9004 是"确实无法
        // 归类"。若二者同码，"同步超差"与"不知道为什么失败"就分不开了。
        //
        // 它仍不进入 SYS-08 的 FAILED：调用方（MeasurementController）
        // 把它当瞬态处理，换一帧重采即可。码只影响"失败怎么记"。
        lastError_.code        = data::kErrSyncOutOfTolerance;
        lastError_.message     = "三相机时间戳超差：" +
                                 std::to_string(lastSpreadNs_) + " ns > 容差 " +
                                 std::to_string(config_.syncToleranceNs) + " ns";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    // ---- 4 组合输出 ----
    output = data::MultiCameraFrame{};
    output.cam25  = cam25;
    output.cam50  = cam50;
    output.cam100 = cam100;

    // ENG-09 §2.5：triggerTimestamp = 各路 timestampNs 的最大值。
    // 注意用的是**所有存在路**的极值（上面按存在性统计），
    // 而不可用路的 timestampNs 为 0，不会成为最大值。
    output.triggerTimestamp = maxTs;

    return true;
}

uint64_t CameraSynchronizer::lastSpreadNs() const
{
    return lastSpreadNs_;
}

int CameraSynchronizer::lastPresentCount() const
{
    return lastPresentCount_;
}

bool CameraSynchronizer::lastFrameDropDetected() const
{
    return lastFrameDrop_;
}

data::ErrorInfo CameraSynchronizer::lastError() const
{
    return lastError_;
}

}  // namespace optical
}  // namespace aircraft
