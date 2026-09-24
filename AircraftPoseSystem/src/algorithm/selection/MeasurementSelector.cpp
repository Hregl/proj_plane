// ============================================================================
//  src/algorithm/selection/MeasurementSelector.cpp
//
//  依据：SYS-14 §6 / §7 / §8、ENG-10 §3.2 / §3.6 / §4.2 / §4.3、
//        ENG-09 §5.16 / §6.5 / §6.6
// ============================================================================

#include "algorithm/selection/MeasurementSelector.h"

#include <algorithm>
#include <cmath>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// SYS-15 §4 / SYS-02 的 Yaw 指标：**1 角分**。这是 E 归一化的参考值。
///
/// ⚠ ENG-10 §3.6 写"`E_ref` 取 `ValidationConfig` 允许的最大误差"，
/// 但 `ValidationConfig`（ENG-09 §6.6 冻结）只有 `maxReprojectionError`
/// （**pixel**）、`minInlierRatio`、`minConfidence`、`yawMin`/`yawMax`
/// （**deg**，且是有效范围而非误差容许值）—— **没有任何以角分表达、
/// 或可折算为角分的误差容许字段**。
///
/// 这个缺口 ENG-09 自己也承认：§9 的待办表把"`ValidationConfig` 阈值取值 /
/// `E_ref` 依赖该取值"列为未结项，并标注"算法专题"。
///
/// 本实现取**系统自身的 Yaw 指标**（1 角分）作为 E_ref，理由是：
///   · 它是冻结数值（SYS-15 §4 的合成结果是 0.97 角分，指标 1 角分），
///     不是本实现新引入的可调参数；
///   · 语义正好符合 §3.6 的意图 —— "预测误差已达容许上限的候选不应被选中，
///     故映射为 0 分"：若某候选的预测 Yaw 误差已达 1 角分，它已耗尽全部
///     精度预算；
///   · 用 `maxReprojectionError` 顶替会引入**单位混用**（pixel 当作角分），
///     而这类错误在本工程已被反复标记为"不报错但结果错"（见 ENG-09 §2.3）。
///
/// 建议裁决：在 `ValidationConfig` 中补 `yawMaxErrorArcmin`，届时本常量
/// 改为读该字段。已记入 README §6 的待裁决清单。
constexpr double kYawErrorRefArcmin = 1.0;

/// 弧度 → 角分：180/π × 60 = 3437.7467707849396。
///
/// ⚠ **R07（2026-09-24 审查报告）：漏了本因子。**
///   `σ_px·√12/(W·√N)` 的两项 σ_px 与 W **同为 pixel**，故该式是**比值**，
///   量纲是**弧度**，不是角分。要到角分必须再乘本因子。
///
///   冻结文档 ENG-10 §2.1 的**表格**（σ=0.3px、W=1930px、N=100 → 0.19 角分）
///   只有在乘上本因子后才成立；而它上面的**公式**当时漏写了因子 ——
///   同页自相矛盾正是本缺陷的形态。ENG-10 V2.2 / ENG-09 V2.2 已把公式
///   对齐到表格（本次是改公式，不是改表格）。
///
///   漏乘的后果不是"数值偏一点"：E 低估 3437.75 倍 ⇒ `eRatio` 几乎恒为 0、
///   `eNorm` 恒接近 1，E 分项**退化为常数**，四个分项的相对权重与通道排序
///   都可能被改变。实测 W=1930、σ=0.3、N=100：漏乘给 5.38461e-05，
///   正确值 0.185109 角分。
///
///   单位一致性的内部证据：本文件另两处写 `predictedError` 的地方
///   （`!(w > 0)` 分支的赋值、`isfinite` 兜底的赋值）都直接写
///   `kYawErrorRefArcmin`，紧随其后的 `eRatio` 比较也是除以它 ——
///   这三处**本来就以角分为单位**，补上因子后算式才落回与它们同一量纲。
constexpr double kRadToArcmin = 3437.7467707849396;

/// σ_px_est 的下限。预测值非正会让 E ≤ 0、eNorm > 1，破坏"四分项均在 [0,1]"
/// 的归一化前提（ENG-10 §3.6）。取 1e-3 pixel 而非 0：
/// σ=0 意味着"定位误差为零"，这在物理上不可能，且会让 E 恒为 0、eNorm 恒为 1。
constexpr double kSigmaFloorPx = 1e-3;

}  // namespace

// ---------------------------------------------------------------------------

MeasurementSelector::MeasurementSelector(
    const data::MeasurementConfig& measurement,
    const data::ValidationConfig& validation,
    const data::IMatchStatsStore* store)
    : measurement_(measurement)
    , validation_(validation)
    , store_(store)
{
}

// ---------------------------------------------------------------------------

double MeasurementSelector::sigmaPxEstimate(const data::ImageQuality& quality,
                                            bool& calibrated) const
{
    const double a = measurement_.sigmaA;
    const double b = measurement_.sigmaB;
    const double c = measurement_.sigmaC;

    // 是否已标定：三者必须有限，且**不能全为 0**。
    // 全 0 是 MeasurementConfig 的默认值，表示"未标定"（ENG-10 §3.4）。
    calibrated = std::isfinite(a) && std::isfinite(b) && std::isfinite(c)
              && (a != 0.0 || b != 0.0 || c != 0.0);

    if (!calibrated)
    {
        // 未标定 → 保守值（ENG-10 §3.4：取 0.5 而非乐观的 0.3）。
        // 理由：E 是减分项，σ 偏大只会让候选被系统性低估（放弃一个更好的
        // 候选），而 σ 偏小会让"实际定位很差"的候选看起来很好而被选中。
        // 两种错误的代价不对称。
        return measurement_.sigmaPxFallback;
    }

    // ⚠ 本式的符号含义存在一处**冻结文档冲突**，取值以 ENG-10 §3.2 为准：
    //     ENG-10 §3.2：`σ_px_est = a + b·(1/S) + c·(1/C)`，**S 为清晰度、
    //                  C 为对比度**，并说明该式"由图像质量预测"。
    //     本工程 002 阶段在 data/MeasurementConfig.h 的注释里把 S 写成了
    //     `TargetScaleEstimate::targetPixelSize`（目标像素尺寸）。
    //   后者与 §3.2 的"由**图像质量**预测"不符（目标像素尺寸不在
    //   ImageQuality 里），且 §3.2 的两项都取自 ImageQuality 才自洽。
    //   ENG-09 §6.5 只给出式子、未定义 S 与 C，故 §3.2 是唯一的定义处。
    //  → 此处按 §3.2 实现，并已修正 data/MeasurementConfig.h 的注释。
    const double s = quality.sharpness;
    const double cc = quality.contrast;

    if (!(s > 0.0) || !(cc > 0.0) || !std::isfinite(s) || !std::isfinite(cc))
    {
        // 输入退化（清晰度为 0、对比度为 0 或非有限）：本式的两项都会发散。
        // 此时**不用**发散值，而退回保守值 —— 与未标定时同一处置，
        // 使"这一帧无法预测"表现为"E 较大、候选被低估"，而非 E=inf。
        return measurement_.sigmaPxFallback;
    }

    const double sigma = a + b / s + c / cc;
    if (!std::isfinite(sigma) || sigma < kSigmaFloorPx)
    {
        return kSigmaFloorPx;
    }
    return sigma;
}

// ---------------------------------------------------------------------------

double MeasurementSelector::qualityNorm(const data::ImageQuality& quality) const
{
    // ENG-10 §3.6：Q = 清晰度 / 曝光 / 对比度三项的组合，各自归一化后
    // "按 MeasurementConfig 的参考值与下限线性映射，并 clamp 到 [0,1]"。
    //
    // ⚠ 冻结配置**只给了清晰度的下限**（`minSharpness`），没有给清晰度的
    // "参考值"，也没有给曝光/对比度的下限或参考值。因此：
    //   · exposure / contrast：ENG-09 §5.15 已冻结它们本身就 ∈ [0,1]
    //     且"1 = 最佳"，无需再映射，直接 clamp 即可（clamp 是防越界输入，
    //     不是映射）；
    //   · sharpness：无上界、无参考值。本实现取
    //         qSharp = clamp(sharpness / minSharpness, 0, 1)
    //     即把**门槛值当作饱和点**：达到门槛即满分。这在只有下限可用时
    //     是最不任意的选择（不引入新常数）。
    //     代价是**存活候选的清晰度分项恒为 1**，Q 的区分度完全来自曝光与
    //     对比度 —— 这是本实现的已知局限，已记入 README §6 的待裁决清单，
    //     建议在 MeasurementConfig 中补 `sharpnessRef`（ENG-09 §6.5 变更）。
    //
    // minSharpness ≤ 0 表示门槛未启用（ENG-09 §6.5 默认值 0）：
    // 此时无法定义饱和点，退化为 clamp(sharpness, 0, 1)。
    const double gate = measurement_.minSharpness;
    double qSharp = 0.0;
    if (gate > 0.0 && std::isfinite(gate))
    {
        qSharp = quality.sharpness / gate;
    }
    else
    {
        qSharp = quality.sharpness;
    }
    qSharp = std::min(1.0, std::max(0.0, qSharp));

    const double qExposure = std::min(1.0, std::max(0.0, quality.exposure));
    const double qContrast = std::min(1.0, std::max(0.0, quality.contrast));

    return (qSharp + qExposure + qContrast) / 3.0;
}

// ---------------------------------------------------------------------------

int MeasurementSelector::distanceBandOf(double distanceM,
                                        const data::MeasurementConfig& config)
{
    // ENG-10 §4.2：0:<80 / 1:80~150 / 2:150~220 / 3:220~300+
    // 边界**冻结于此**，代码中不得另设 —— 故只读 config 的三个边界。
    const double* e = config.distanceBandEdges;
    if (distanceM < e[0]) { return 0; }
    if (distanceM < e[1]) { return 1; }
    if (distanceM < e[2]) { return 2; }
    return 3;
}

int MeasurementSelector::illumBandOf(double exposure,
                                     const data::MeasurementConfig& config)
{
    // ENG-10 §4.2：0:低照度 / 1:正常 / 2:强光逆光，判据为 exposure 落入的
    // 分位区间。两条边界的数值须与实际曝光设置一同标定（ENG-09 §6.5）。
    //
    // ⚠ `illumBandEdges` 的默认值是 {0, 0}（未标定）。此时两个边界相等，
    // 任何 exposure ≥ 0 都会落到 band 2（强光逆光）—— 一个**看起来正常
    // 但含义错误**的分桶。故此处对"边界未配置"做显式处理：返回 band 1
    // （正常），并在注释中说明理由：未标定时无法区分光照，
    // 取"正常"使 M_hist 落在中间桶，而不会把全部观测灌进一个偏斜的桶里。
    const double* e = config.illumBandEdges;
    if (!(e[0] < e[1]))
    {
        return 1;
    }
    if (exposure < e[0]) { return 0; }
    if (exposure < e[1]) { return 1; }
    return 2;
}

// ---------------------------------------------------------------------------

bool MeasurementSelector::scoreAll(
    std::vector<data::MeasurementCandidate>& candidates,
    const std::string& targetModelId) const
{
    lastUsage_.clear();
    lastColdStart_ = false;

    // ---- ① 硬门槛（SYS-14 §6）----
    //
    // ⚠ `minMatchRatio` 的处置是本方法最容易做错的一处：
    //   `ImageQuality::matchRatio` 是**匹配之后**才有的量，而本方法运行在
    //   PnP（乃至匹配）**之前** —— 这正是 E 项要去掉 PnP 依赖的原因
    //   （ENG-10 §3.1）。若无条件套用 `minMatchRatio` 门槛，则首次选择时
    //   所有候选的 matchRatio 都是 0（结构体默认值），**全部候选被删除**，
    //   表现为"永远没有可用通道"，MEASURE_SELECT 重试 3 次后任务失败，
    //   而报出的原因与真实原因（门槛套错了阶段）毫无关系。
    //
    //   故：matchRatio 门槛**只在调用方已测得该值时**生效
    //   （matchRatio > 0）。重试路径上，上一次匹配的统计会被填回候选，
    //   此时门槛是有意义的；首次选择时它不参与。
    auto gated = [this](const data::MeasurementCandidate& c) -> bool
    {
        if (c.quality.sharpness < measurement_.minSharpness)
        {
            return true;
        }
        if (c.scale.targetPixelSize < measurement_.minTargetPixelSize)
        {
            return true;
        }
        if (c.nDetect < measurement_.minFeatureCount)
        {
            return true;
        }
        if (c.quality.matchRatio > 0.0
            && c.quality.matchRatio < measurement_.minMatchRatio)
        {
            return true;
        }
        return false;
    };

    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(), gated),
        candidates.end());

    if (candidates.empty())
    {
        return false;
    }

    // ---- ② 逐候选计算四分项 ----
    for (data::MeasurementCandidate& c : candidates)
    {
        // ---- M：历史匹配成功率 ----
        //
        // ⚠ 距离不可用时**不查表**：`TargetScaleEstimate` 的默认值是
        //   distance = 0（`confidence` 也是 0），若照字面分桶，它会落进
        //   "0~80 m"这一最近带 —— 于是"我们不知道距离"被静默地变成了
        //   "目标在 80 m 以内"，查出来的历史匹配率来自最不相干的距离带，
        //   而 M 是加权项，这会直接把选择推向错误的一侧。
        //   处理：distance/confidence 不可用时按冷启动先验处理，
        //   并把 `distanceBand` 记成 -1（result.json 里可见"距离未知"，
        //   而不是一个看似正常的 0）。
        const bool distanceKnown = std::isfinite(c.scale.distance)
                                && c.scale.distance > 0.0
                                && std::isfinite(c.scale.confidence)
                                && c.scale.confidence > 0.0;

        const int distanceBand = distanceKnown
            ? distanceBandOf(c.scale.distance, measurement_)
            : -1;
        const int illumBand =
            illumBandOf(c.quality.exposure, measurement_);

        MatchStatsUsage usage;
        usage.camera = c.camera;
        usage.distanceBand = distanceBand;
        usage.illumBand = illumBand;

        if (store_ != nullptr && distanceKnown)
        {
            usage.mHist = store_->successRate(
                targetModelId,
                static_cast<int>(c.camera),
                distanceBand,
                illumBand);
            usage.fromStore = true;
            if (!std::isfinite(usage.mHist))
            {
                // 表损坏或实现越界时不要静默用 NaN 参与评分 ——
                // NaN 会让比较恒为 false，选择结果退化为"第一个候选"。
                usage.mHist = measurement_.matchStatsColdStartPrior;
                usage.fromStore = false;
            }
        }
        else
        {
            // ENG-10 §4.3：无表 = 冷启动，取先验 0.5，使 M 对所有候选等价。
            // 距离未知（`!distanceKnown`）也走这一支，理由见上。
            usage.mHist = measurement_.matchStatsColdStartPrior;
            usage.fromStore = false;
            lastColdStart_ = true;
        }
        usage.mHist = std::min(1.0, std::max(0.0, usage.mHist));
        lastUsage_.push_back(usage);

        c.mNorm = usage.mHist;

        // ---- F：特征数量（ENG-10 §3.6：F = min(1, N_detect / N_ref)）----
        const double nRef = measurement_.nRef;
        c.fNorm = (nRef > 0.0)
                    ? std::min(1.0, static_cast<double>(c.nDetect) / nRef)
                    : 0.0;

        // ---- Q ----
        c.qNorm = qualityNorm(c.quality);

        // ---- E（ENG-10 §3.2）----
        bool calibrated = false;
        c.sigmaPxEst = sigmaPxEstimate(c.quality, calibrated);
        c.predictedErrorCalibrated = calibrated;

        // W：展布宽度。§3.2 规定"退化时用检测框长边"（ENG-10 §2.4 的
        // W ≥ 0.6 × 检测框长边亦以同一基准）。
        double w = c.featureSpreadPx;
        if (!(w > 0.0) || !std::isfinite(w))
        {
            w = c.scale.targetPixelSize;
        }

        // N_est = M_hist × N_detect（ENG-10 §3.3：M_hist 的唯一用途）。
        c.nEst = c.mNorm * static_cast<double>(c.nDetect);

        if (!(w > 0.0) || !(c.nEst > 0.0))
        {
            // 展布为零或预计有效点数为零：E 发散。
            // 取 E = E_ref 使 eNorm = 0 —— 语义正好是 §3.6 给 E_ref 的理由：
            // "预测误差已达容许上限的候选不应被选中"。比写入 inf 更适合
            // 落盘（JSON 无法表达 inf），也避免了 NaN 参与比较。
            c.predictedError = kYawErrorRefArcmin;
        }
        else
        {
            // E = σ_px_est · √12 / (W · √N_est) × (180/π×60)，单位**角分**。
            //
            // ⚠ R07：算式原本**漏了最后那个换算因子**，而注释与冻结文档都写
            //   "单位角分" —— 于是 `σ_px/W` 这个**比值**（弧度）被直接当作
            //   角分使用，低估 3437.75 倍。详见 kRadToArcmin 的说明。
            //   现在算式与本注释首次一致。
            //
            // sqrt(12) 来自均匀分布量化误差的方差（原引 SYS-15 §4.5 的模型）。
            // ⚠ 该节号是**已登记的悬空引用**：SYS-15 没有 §4.5，其 §4 是纯框图无子节，
            //   且该模型在 SYS-15 里**整份不存在**（无 √12 / σ_θ / σ_px 任何命中）。
            //   与 IF-SW-02 同类，已登记（README §6 / 待裁决问题汇总 §5），**本处不修**。
            c.predictedError = c.sigmaPxEst * std::sqrt(12.0)
                               / (w * std::sqrt(c.nEst)) * kRadToArcmin;
            if (!std::isfinite(c.predictedError) || c.predictedError < 0.0)
            {
                c.predictedError = kYawErrorRefArcmin;
            }
        }

        // ---- E 的归一化（ENG-10 §3.6：eNorm = 1 − min(1, E/E_ref)）----
        const double eRatio = std::min(1.0, c.predictedError / kYawErrorRefArcmin);
        c.eNorm = 1.0 - eRatio;

        // ---- 总分 ----
        //
        // ⚠⚠ 这里是本工程一处**冻结文档自相矛盾**的落点，符号取 "**+** w4·eNorm"，
        //   而不是 SYS-14 §6.2 / §8 字面上的 `− w4·E`。理由如下（必须记录，
        //   因为改成 "+" 后与 §8 的字面写法不符，看代码的人会以为写错了）：
        //
        //   · SYS-14 §8.2 与 ENG-10 §3.6（两处措辞一致）规定：四个分项
        //     **先各自归一化到 [0,1] 再加权**，其中 E 的归一化是
        //         eNorm = 1 − min(1, E / E_ref)
        //     并给出理由："预测误差已达验证阈值上限的候选不应被选中，
        //     故**映射为 0 分**"。
        //   · 这个映射使 eNorm 成为**优度**（越大越好），而不是误差本身。
        //     若仍按字面写成 `− w4·eNorm`：误差达上限的候选贡献恰好 0 分
        //     （表面正确），但误差为 0 的**最好候选会贡献 −w4 分**，
        //     即"越准越吃亏"，排序被完全颠倒 —— 与 §8.2 的整段意图
        //     （"不应被选中"）直接冲突。
        //   · 按本工程的冲突规则（README §6：两份冻结文档冲突时取**更具体**
        //     的一节），§8.2 / ENG-10 §3.6 是专门为"归一化后的加权形式"写的
        //     一节，比 §6.2 的原始式更具体，故以它为准：归一化之后
        //     四项都是"越大越好"的优度，**统一取正号**。
        //
        // 后果核对：w4 = 0（未配置）时两种写法等价；w4 > 0 时本写法使
        // "预测误差更小"严格地提高得分。已记入 README §6 的待裁决清单。
        //
        // ⚠ 一个容易说错的点，写在这里以免后来者据此"改回去"：
        //   `+ w4·eNorm` 与 §6.2 的 `− w4·E`（**原始**误差）**排序方向相同**
        //  （1 − E/E_ref 与 −E 都是 E 的单调减函数），差别只在归一化与常数项。
        //   真正会把排序颠倒的是**混合写法** `− w4·eNorm`（把 §6.2 的减号
        //   照搬到已归一化的优度上）。也就是说 §6.2 本身并不"错在符号方向"，
        //   它错在没跟上 §8.2 的归一化。
        c.score = measurement_.w1 * c.qNorm
                + measurement_.w2 * c.fNorm
                + measurement_.w3 * c.mNorm
                + measurement_.w4 * c.eNorm;
    }

    return true;
}

// ---------------------------------------------------------------------------

bool MeasurementSelector::select(
    const std::vector<data::MeasurementCandidate>& candidates,
    data::MeasurementSelectionResult& out) const
{
    out = data::MeasurementSelectionResult{};

    if (candidates.empty())
    {
        return false;
    }

    // 取总分最高者（8.md §七 的"取 score 最大者"）。
    //
    // ⚠ 比较用 `>` 而非 `>=`：并列时保留**先出现**的候选，而候选顺序由
    // 调用方按 allowed 列表（即优先级顺序）构造，故并列时优先级高的胜出。
    // 这使结果与输入顺序确定相关 —— SYS-04 §6.4 要求结果可离线复现，
    // 而"并列时选哪个"若无确定规则，重放会得到不同结果。
    const data::MeasurementCandidate* best = &candidates.front();
    for (const data::MeasurementCandidate& c : candidates)
    {
        if (c.score > best->score)
        {
            best = &c;
        }
    }

    out.selectedCamera = best->camera;
    out.score = best->score;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
