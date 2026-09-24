#pragma once

// ============================================================================
//  src/data/ImageQuality.h
//
//  依据：ENG-09 §4.1、§5.15（类型定义冻结）、裁决 C-12
//
//  裁决 C-12：SYS-05 §9.1 只有 sharpness，但 SYS-07 §6.2 与 SYS-14 §7 的
//  评分模型把"图像质量 Q"定义为 **清晰度 + 曝光 + 对比度** 三项。
//  字段缺失导致 Q 无法计算。冻结补入 exposure 与 contrast。
//
//  ⚠ 与工作流文档的偏离（3.md §十一 的 ImageQuality 只有
//      sharpness / featureCount / matchCount / matchRatio）：
//  漏掉 exposure 与 contrast 会让 Q 项退化为只看清晰度。这不只是"少一个
//  权重项"——SYS-14 §7 的评分式 Score = w1·Q + w2·F + w3·M − w4·E 中，
//  Q 是**唯一**对成像条件敏感的分项；缺了曝光与对比度，逆光/欠曝的候选
//  相机若清晰度尚可就会被选中，而这正是 M_hist 分桶要按光照区分的原因。
//
//  两个归一化字段的取值范围（ENG-09 §5.15）：
//      exposure  [0,1]，1 = 最佳曝光（不是"曝光时间"）
//      contrast  [0,1]
//  注意 exposure 不是曝光时间——曝光时间在 ImageFrame.exposureTime
//  （单位 s）。本字段是"曝光合理性"的评价量，供 ENG-10 §3.2 的
//  σ_px_est = a + b·(1/S) + c·(1/C) 使用。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 图像质量评价（SYS-07 §6.2 / SYS-14 §7）。
struct ImageQuality
{
    /// 清晰度：Laplacian 方差或梯度能量。无上界，越大越清晰。
    /// ENG-10 §3.2 的 σ_px_est 预测式中 S 即本字段。
    double sharpness = 0.0;

    /// 曝光合理性，[0,1]。1 = 最佳。ENG-10 §3.2 中不直接使用，
    /// 但 ENG-10 §4.2 的 illum_band 分桶依据就是它的分位区间。
    double exposure = 0.0;

    /// 对比度，[0,1]。ENG-10 §3.2 的 σ_px_est 预测式中 C 即本字段。
    double contrast = 0.0;

    /// 本帧提取到的关键点数（A 类 + B 类）。
    /// 对应 ENG-09 §5.16 的 nDetect，也是 F 分项的原始量（前归一化）。
    int featureCount = 0;

    /// 匹配成功的内点数。
    int matchCount = 0;

    /// 匹配内点比例 = 内点/总数，[0,1]。
    double matchRatio = 0.0;
};

}  // namespace data
}  // namespace aircraft
