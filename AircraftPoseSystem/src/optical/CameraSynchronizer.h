#pragma once

// ============================================================================
//  src/optical/CameraSynchronizer.h
//
//  依据：SYS-06 §8（三相机同步采集：时间戳检查 / 丢帧检测 / 同步验证）
//        SYS-08 §7.5（硬件降级）、ENG-09 §2.5（时间戳）、§5.6、§6.4
//        ENG-01 §6、5.md §十
//
//  作用：把三路已采集的图像组合为一个 MultiCameraFrame，并验证它们
//  确实属于同一时刻。
//
//  ⚠ 与 Device 层的职责边界（5.md §十 亦强调"真正采集由 Device 负责"）：
//      device/camera/MultiCameraManager —— **取回**三路图像并填时间戳；
//      optical/CameraSynchronizer      —— **判定**这三路是否同步。
//  判据必须在本层而不是 device 层，理由：判据需要
//  TriggerConfig::syncToleranceNs，而"多大的偏差算不同步"是随任务场景
//  变化的策略；device 层不应解释配置的语义（它只面对硬件）。
//
//  ⚠ 本类**不重排、不丢弃**输入帧。若三路不同步，它报告失败，
//  由调用方（CAPTURE 状态）决定是重采还是作废 —— 重采属于应用层策略，
//  因为只有应用层知道本次测量的重试预算还剩多少（SYS-08 §7.6 的
//  RetryManager 唯一持有计数器）。
//
//  ⚠ 关于"同步失败"的错误码（裁决 C-006 更新本段）：
//  本类原先在同步超差时返回 false 且 lastError().code 保持 0，理由是
//  ENG-09 §5.27 的 3000 段当时只占用了 3001（触发失效降级），
//  没有"三相机时间戳超差"这一项，而代码中不得使用表外的裸错误码。
//
//  该缺口已由裁决 C-006 补上：**3002 = kErrSyncOutOfTolerance**
//  （登记于 ErrorInfo.h，段划分未变）。本类现在置 3002 而非 0 ——
//  0 的冻结语义是"未设置"，渲染为 "OK"，会让一次真实的同步超差
//  在结果包里与"没有错误"无法区分。
//
//  语义仍未变：同步超差是**瞬态**且**由重采解决**，它不会走到
//  SYS-08 的 FAILED。码只改变"失败怎么记"，不改变重试路径。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>

#include "data/ErrorInfo.h"
#include "data/ImageFrame.h"
#include "data/MultiCameraFrame.h"
#include "data/TriggerConfig.h"

namespace aircraft
{
namespace optical
{

/// 三相机同步验证与组合（SYS-06 §8）。
class CameraSynchronizer
{
public:
    /// @param config 触发配置（ENG-09 §6.4），提供 syncToleranceNs 判据。
    explicit CameraSynchronizer(const data::TriggerConfig& config);

    /// 把三路图像组合为一个多相机帧，并验证同步性。
    ///
    /// 允许传入**空帧**（image 为空）表示该路不可用 —— 这是 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕
    /// 降级运行（2 台相机）的正常形态，此时按 2 路判据处理。
    ///
    /// 判据：
    ///   1. 存在的路数 >= 2（否则按 §7.5 无法继续，code=1001）；
    ///   2. 存在各路 timestampNs 的极差 <= TriggerConfig::syncToleranceNs
    ///      （极端值之间的最大间隔，比逐对比较更严格且只需一次遍历）。
    ///
    /// @param output 成功时填好三路图像与 triggerTimestamp
    ///               （= 各路 timestampNs 的最大值，ENG-09 §2.5）。
    ///               失败时内容未定义。
    /// @return false 表示路数不足或同步超差。详见 lastError()。
    bool synchronize(const data::ImageFrame& cam25,
                     const data::ImageFrame& cam50,
                     const data::ImageFrame& cam100,
                     data::MultiCameraFrame& output);

    // ---- 诊断（供日志与 UI）----

    /// 上一次调用中各路时间戳的极差，单位 ns。
    uint64_t lastSpreadNs() const;

    /// 上一次调用中实际存在的路数（0~3）。
    int lastPresentCount() const;

    /// 上一次调用中是否检测到丢帧（任一路帧号相对上次不连续）。
    ///
    /// ⚠ 丢帧**不导致 synchronize() 失败**，理由是它不影响"同一时刻"这一
    /// 核心判据：三路仍然同步，只是采样数少了。CAPTURE 状态按
    /// MeasurementConfig::captureFrameCount 采 5~10 帧用于平均
    /// （SYS-15 §4 的随机误差按 1/√N 下降），丢帧只降低 N。
    /// 若把丢帧当失败，则一次偶发的采集抖动就会作废整轮 ——
    /// 而重采的代价（转台重新稳定 3.0 s）远大于少一帧的收益。
    ///
    /// 但**必须暴露**：ENG-06 的稳定性测试与现场排查都需要知道
    /// "帧率是否稳定"，而丢帧若不可见，这个问题无从回答。
    bool lastFrameDropDetected() const;

    /// 上一次失败的原因。同步超差时 code 为 0（见文件头说明）。
    data::ErrorInfo lastError() const;

private:
    data::TriggerConfig config_;

    uint64_t lastSpreadNs_       = 0;
    int      lastPresentCount_   = 0;
    bool     lastFrameDrop_      = false;

    /// 上一轮的帧号（按角色记），用于丢帧检测。
    /// 初值 0 且用 hasPrev_ 区分"首轮"—— 首轮没有前值可比，
    /// 若用 0 作为哨兵，则首轮任何帧号都会被视为"跳跃"。
    uint64_t prevFrameId_[3] = {0, 0, 0};  // 索引 0/1/2 = CAM25/50/100
    bool     hasPrev_[3]     = {false, false, false};

    data::ErrorInfo lastError_;
};

}  // namespace optical
}  // namespace aircraft
