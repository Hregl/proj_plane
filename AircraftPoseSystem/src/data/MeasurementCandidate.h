#pragma once

// ============================================================================
//  src/data/MeasurementCandidate.h
//
//  依据：ENG-09 §4.1、§5.16（类型定义冻结）、裁决 C-14、C-21
//        ENG-10 §3（评分 E 项的数据来源）
//
//  裁决 C-14：SYS-07 §6.2 的评分式为 Score = w1·Q + w2·F + w3·M − w4·E，
//  但 SYS-05 §9.2 的 MeasurementCandidate **没有 E 的载体字段**。
//  冻结补入 predictedError。
//
//  单位修正（ENG-09 §5.16，原注释误写为 pixel）：
//  E 是**角度**量（最终要与之比较的是 Yaw 误差 1 角分），其计算式
//      σ_θ = σ_px·√12 / (W·√N)
//  的输出就是角度。冻结为**角分（arcminute）**。
//  以 pixel 为单位的量是它的输入 sigmaPxEst，两者不可混用 ——
//  混用的后果是 E 的量级差 2000 倍以上（300 m 处 1 角分 ≈ 0.087 m，
//  对应像素随焦距变化），权重 w4 无论如何整定都无法补偿。
//
//  四分项必须分别保留（ENG-09 §5.16 新增要求）：
//  原定义只留 score，导致 SYS-14 §20 约束 3（"选择逻辑可解释"）无法兑现
//  —— **score 是加权和，不可逆推回分项**。故四个归一化分项与 E 的三个
//  输入都必须记录，供 result.json 追溯（ENG-10 §7 约束 14）。
//
//  ⚠ 与工作流文档的偏离（3.md §十一 的 MeasurementCandidate 只有
//      camera / distanceEstimate / quality / score 四个字段）：
//  缺少的 9 个字段是 ENG-10 §3 冻结的评分链，不是可选装饰。
//  例如 qNorm/fNorm/mNorm/eNorm 缺失 → 四分量归一化（ENG-10 §3.6）无从
//  实现，而不归一化时量级最大的分项实际决定结果，权重失去意义；
//  predictedErrorCalibrated 缺失 → 无法区分"E 可信"与"E 用的是保守默认
//  值 0.5 pixel"，而 ENG-10 §3.4 要求该标记必须写入 result.json。
// ============================================================================

#include "data/CameraRole.h"
#include "data/ImageQuality.h"
#include "data/TargetScaleEstimate.h"

namespace aircraft
{
namespace data
{

/// 一个测量通道候选（SYS-14 §7）。三焦段各产出一个，由 MeasurementSelector 评分。
struct MeasurementCandidate
{
    /// 候选对应的相机角色。
    CameraRole camera = CameraRole::CAM25;

    /// 距离/尺度估计（仅用于通道选择，见 TargetScaleEstimate.h）。
    TargetScaleEstimate scale;

    /// 图像质量评价。
    ImageQuality quality;

    // ---- 评分四个分项（均为归一化后的值，ENG-10 §3.6）----
    // 归一化是 E 项标定的**前置条件**：不归一化时拟合出的 a,b,c
    // 会被权重体系的量级问题掩盖。

    /// Q：图像质量分项，[0,1]。由 sharpness/exposure/contrast 按
    /// MeasurementConfig 的参考值与下限线性映射并 clamp（ENG-10 §3.6）。
    double qNorm = 0.0;

    /// F：特征数量分项，[0,1]。= min(1, nDetect / MeasurementConfig::nRef)，
    /// nRef 默认 100（SYS-15 §4.5 校核值：满足 0.20 角分所需点数）。
    double fNorm = 0.0;

    /// M：历史匹配成功率分项，[0,1]。= M_hist。本身已在 [0,1]。
    double mNorm = 0.0;

    /// E：预测误差分项，[0,1]。= 1 − min(1, predictedError / E_ref)，
    /// E_ref 取 ValidationConfig 允许的最大误差（ENG-10 §3.6）。
    double eNorm = 0.0;

    // ---- E 的三个输入（ENG-10 §3.2）----

    /// W：特征点横向展布宽度，单位 pixel。
    /// 取 B 类结构点分布 ∪ A 类关键点分布的两端之差；退化时用检测框长边。
    /// 由 ENG-10 §2.2 的特征分层保证 —— 这是"B 类保展布"的量化体现。
    double featureSpreadPx = 0.0;

    /// 本帧在该候选相机上提取到的关键点数。
    int nDetect = 0;

    /// N_est = M_hist × nDetect（ENG-10 §3.3）。
    /// M_hist 的唯一用途就是作为 N_est 的折扣系数，经 N_est 进入 E。
    double nEst = 0.0;

    /// 预测的单点定位标准差，单位 pixel。**不是像素坐标偏差**。
    /// σ_px_est = a + b·(1/S) + c·(1/C)，系数见 MeasurementConfig；
    /// 未标定时取保守值 MeasurementConfig::sigmaPxFallback（默认 0.5 pixel）。
    double sigmaPxEst = 0.0;

    /// E：预测的 Yaw 误差，单位 **角分（arcminute）**。
    ///
    ///     E = σ_px_est · √12 / (W · √N_est)
    ///
    /// 该式每一项都可在 PnP 之前获得，因此不依赖 PnP 输出 ——
    /// 这解决了"SYS-14 的选择在 PnP 之前、而 E 需要 PnP 残差"的循环依赖
    /// （ENG-10 §3.1/§3.2）。判断实现是否正确的最简标准即：
    /// **E 的计算路径上不得出现 PnP 的任何输出**（ENG-10 §7 约束 4）。
    double predictedError = 0.0;  // arcmin

    /// a,b,c 是否已由 Golden 数据标定（ENG-10 §3.4/§3.5）。
    /// false 时 predictedError 取保守值，且必须在 result.json 中记录。
    bool predictedErrorCalibrated = false;

    /// 加权总分 = w1·qNorm + w2·fNorm + w3·mNorm **+** w4·eNorm。
    ///
    /// ⚠ 符号说明（002 阶段本行曾写作 `− w4·eNorm`，007 阶段改正）：
    /// 四项都是"越大越好"的归一化优度，故**全部取正号**。
    /// SYS-14 §8.2 与 ENG-10 §3.6 把 E 归一化为 `1 − min(1, E/E_ref)`
    /// （"预测误差已达上限的候选……映射为 0 分"）—— 归一化之后 eNorm 是
    /// 优度而非误差，若仍按 §6.2 的字面写负号，"误差最小的最好候选"
    /// 反而得分最低，排序完全颠倒。完整论证见
    /// algorithm/selection/MeasurementSelector.cpp 与 README §6。
    ///
    /// **不可从中逆推分项**，故上方四项与下方三个输入必须同时保留。
    double score = 0.0;
};

}  // namespace data
}  // namespace aircraft
