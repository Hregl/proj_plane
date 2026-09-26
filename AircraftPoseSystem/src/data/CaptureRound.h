#pragma once

// ============================================================================
//  src/data/CaptureRound.h
//
//  依据：ENG-09 V2.3 §5.32（每轮取帧结果，冻结）、§5.29（操作结果）
//        SYS-04 V2.4 §6.1 / SYS-06 V2.3 §5.1
//        裁决 C-01 v1.7（R09 部分修复：每路结果与聚合）
//
//  作用：把**一次 `capture()` 里三路各自发生了什么**如实保留下来。
//
//  ⚠ 为什么必须新增这个载体（而不是让上层继续从帧反推）：
//  当前上层判断"哪一路可用"的依据是"该路的图非空"（见
//  MeasurementController::updateDegradation 的旧注释）。这个判据能回答
//  "拿到了没有"，但回答不了"**为什么没拿到**" —— 于是一次**超时**
//  与一次**断连**在上层看起来一模一样，而两者的处置完全相反
//  （前者重采即可，后者要禁用并降级）。R09 的失效形态正源于此：
//  "所有取帧失败被统一处置"，上层无从细分，只能一律按最重的处理。
//
//  ⚠ 三个字段的分工（不得混成一个 bool，见 ENG-09 V2.3 §5.32）：
//    `attempted`     —— 这一路**有没有被尝试**（预算耗尽/通道不可用时为 false）
//    `skippedReason` —— **没尝试的原因**（人读；只表达"未尝试"，不表达失败）
//    `result`        —— 尝试后的结果（含分类与 SDK 原码）
//  ⚠ `attempted == false` **不等于**"这一路没问题"：预算耗尽时
//  `result.status == Timeout`（本地超时，未调用 SDK）。
//  聚合必须把这种"未尝试的失败"算进去 —— 只挑"尝试过且失败"会把它整条漏掉。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <array>
#include <cstdint>
#include <string>

#include "data/CameraRole.h"
#include "data/OpStatus.h"

namespace aircraft
{
namespace data
{

/// 单通道一次取帧的全部事实。
struct ChannelGrabRecord
{
    /// 本记录对应的焦段角色。
    CameraRole role = CameraRole::CAM25;

    /// 本轮**是否真的调用过**该路的取帧（含先发软件触发令）。
    bool attempted = false;

    /// 本路**没有产出有效帧**时的原因（人读）。该路正常采到时为空串。
    ///
    /// ⚠ 它表达的不只是"未尝试"：`attempted == true` 却仍未产出有效帧的
    ///    情形同样要在这里说清（通道已被禁用、取帧成功但交付的图为空、
    ///    返回未赋值结果、软件触发令失败而不去等帧……）。
    ///    故**不得**把 `attempted == true` 当作"本字段必为空"的判据。
    /// ⚠ 必须写明具体原因（"预算耗尽"／"通道不可用"／"入口可用数不足"／
    ///    "该通道已禁用：…"），不得只写"未尝试"—— 那等于把"为什么"
    ///    推回给读日志的人。
    std::string skippedReason;

    /// 取帧结果：分类 + SDK 原码（`sdkError`） + 清理失败（`cleanupError`）。
    GrabResult result;

    /// 本路画面的主机单调时间戳（ns）；未取到帧时为 0。
    uint64_t timestampNs = 0;

    /// 软件触发令的结果（§2.4 单一执行者）。
    /// 该路模式读回不是 `Software` 时为空（`status == Unset` 表示"未发令"，
    /// 这是 `Unset` 唯一被允许出现的地方 —— 它不在聚合范围内）。
    OperationResult trigger;
};

/// 一次 `capture()` 的完整结果。
struct CaptureRound
{
    /// 三路记录，下标 0/1/2 = CAM25/50/100（与 `MultiCameraFrame` 同序）。
    std::array<ChannelGrabRecord, 3> channels{};

    /// **本轮成功交付的有效帧数**：该路 `result.status == Ok`
    /// **且**交付的图合法（非空）。
    ///
    /// ⚠ 判据与上层既有的"图非空"同源（`MeasurementController` 用的就是它），
    /// 本批**不新增第五个帧计数口径** —— 本仓此前已有四个互不相同的计数
    /// （管理器内部的 `captured` 只数 `grab()` 返回真、上层数 `!image.empty()`、
    /// Recorder 按非空逐路写 `.raw`、`availableCameraCount()` 数的是**锁存的
    /// 可用标志**）。本字段以"成功且图非空"为权威。
    /// ⚠ **不承诺**它等于最终成功写盘的文件数 —— 落盘还受 Recorder 的
    /// 分支与失败影响，两者不一致是**正常**的，不得据此断言"计数错了"。
    int capturedCount = 0;

    /// 本轮是否整体成功（`capturedCount >= 2`，即 §7.5 的降级下限）。
    bool succeeded = false;

    /// 本轮聚合状态：三路 `result.status` 中**严重度最大**者
    /// （严重度序见 `aggregationSeverity()`）。
    /// ⚠ 全部取值都在严重度表内，不含"表外状态"。
    /// 三路全 `Ok` 时为 `Ok`。
    OpStatus aggregate = OpStatus::Ok;

    /// 本轮 `capture()` 的起止时刻（注入钟，ns）。
    ///
    /// ⚠ 存在的理由：删除"组预算＝GUI 线程硬上限"这一承诺之后
    /// （该承诺不成立 —— 一次 GUI 回调会连采多组，复制/转换/发令
    /// 也都不受取帧等待参数约束），"回调实际占用了多久"必须变成
    /// **可测事实**，而不是一句无法验证的保证。
    uint64_t startedNs  = 0;
    uint64_t finishedNs = 0;

    /// 按角色取该路记录（有效角色恒定返回非空引用）。
    const ChannelGrabRecord& channel(CameraRole role) const
    {
        switch (role)
        {
        case CameraRole::CAM25:
            return channels[0];
        case CameraRole::CAM50:
            return channels[1];
        case CameraRole::CAM100:
            return channels[2];
        }
        return channels[0];
    }

    ChannelGrabRecord& channel(CameraRole role)
    {
        return const_cast<ChannelGrabRecord&>(
            static_cast<const CaptureRound*>(this)->channel(role));
    }
};

}  // namespace data
}  // namespace aircraft
