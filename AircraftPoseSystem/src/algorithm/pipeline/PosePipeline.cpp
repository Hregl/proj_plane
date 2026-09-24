// ============================================================================
//  src/algorithm/pipeline/PosePipeline.cpp
//
//  依据：ENG-02 §12、ENG-10 §2~§5、SYS-07、SYS-14 §7 / §10、ENG-09 §2.1
//
//  本文件是算法层的组合根。各阶段的算法细节在各自子模块内，
//  这里只负责：装配、按序调用、坐标合成、把过程量缓存给 VALIDATE 用。
// ============================================================================

#include "algorithm/pipeline/PosePipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "data/CameraPose.h"
#include "data/DetectionResult.h"
#include "data/IMatchStatsStore.h"
#include "data/ImageFrame.h"
#include "data/MeasurementCandidate.h"
#include "data/MeasurementSelectionResult.h"

namespace aircraft
{
namespace algorithm
{

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// 对比度归一化的饱和点（灰度标准差）。
///
/// ⚠ 该值**不在任何冻结文档中**：`ImageQuality::contrast` 只规定 ∈ [0,1]
/// （ENG-09 §5.15），未规定如何从图像算出。此处取 8 bit 灰度下
/// "已经算高对比"的标准差 64 作为饱和点。
/// 该口径与 `MeasurementConfig::minSharpness` 一样，是**同一实现内部自洽、
/// 跨实现不可比**的量 —— 换一套质量评价口径，minSharpness 的标定值必须
/// 重新给。已记入 README §6 的待裁决清单。
constexpr double kContrastSaturation = 64.0;

/// 把输入图像统一成 CV_8UC1，**不修改入参**（图像可能与预览线程共享）。
cv::Mat toGray(const cv::Mat& src)
{
    if (src.empty() || src.depth() != CV_8U)
    {
        return cv::Mat();
    }
    if (src.channels() == 1)
    {
        return src;
    }
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

double clamp01(double v)
{
    if (!std::isfinite(v))
    {
        return 0.0;
    }
    return std::min(1.0, std::max(0.0, v));
}

}  // namespace

// ---------------------------------------------------------------------------
//  构造 / 析构
// ---------------------------------------------------------------------------

PosePipeline::PosePipeline(const data::MeasurementConfig& measurement,
                           const data::ValidationConfig& validation,
                           const TargetModelManager& models,
                           const data::OpticalRigCalibration& rig,
                           const data::IMatchStatsStore* statsStore)
    : PosePipeline(measurement, validation, models, rig, Stages{}, statsStore)
{
}

PosePipeline::PosePipeline(const data::MeasurementConfig& measurement,
                           const data::ValidationConfig& validation,
                           const TargetModelManager& models,
                           const data::OpticalRigCalibration& rig,
                           const Stages& stages,
                           const data::IMatchStatsStore* statsStore)
    : measurement_(measurement)
    , validation_(validation)
    , models_(models)
    , rig_(rig)
    , selector_(measurement, validation, statsStore)
    , validator_(validation)
{
    // 未注入的阶段自建为真实实现。逐项判断，因此可以只替换其中一部分
    // （例如只注入 MockFeatureMatcher，其余走真实链）。
    if (stages.detector == nullptr)
    {
        // ⚠ 007 **没有**真实的 YOLO 检测器（SYS-07 §4）—— 它需要
        //    DNN 权重文件与推理后端，属于后续阶段的交付物。
        //    此处用阈值 + 最大轮廓的 `MockTargetDetector` 作为默认实现，
        //    使 M1 的合成闭环（11.md）可运行。它**不得用于真实影像**：
        //    对云、海杂波、甲板高光同样会给出"目标"。
        //    真实 `YoloDetector` 实现后即通过 Stages 注入替换，
        //    Pipeline 本身无需改动 —— 这是 Stages 存在的直接理由。
        ownedDetector_ = std::make_unique<MockTargetDetector>(measurement);
        detector_ = ownedDetector_.get();
    }
    else
    {
        detector_ = stages.detector;
    }

    if (stages.scale == nullptr)
    {
        ownedScale_ = std::make_unique<PinholeScaleEstimator>();
        scale_ = ownedScale_.get();
    }
    else
    {
        scale_ = stages.scale;
    }

    if (stages.extractor == nullptr)
    {
        ownedExtractor_ =
            std::make_unique<SiftFeatureExtractor>(measurement.minFeatureCount);
        extractor_ = ownedExtractor_.get();
    }
    else
    {
        extractor_ = stages.extractor;
    }

    if (stages.matcher == nullptr)
    {
        ownedMatcher_ = std::make_unique<DescriptorFeatureMatcher>(measurement);
        matcher_ = ownedMatcher_.get();
    }
    else
    {
        matcher_ = stages.matcher;
    }

    if (stages.pnp == nullptr)
    {
        ownedPnp_ = std::make_unique<CvPnPPoseEstimator>(measurement);
        pnp_ = ownedPnp_.get();
    }
    else
    {
        pnp_ = stages.pnp;
    }

    // `stages.cad` 不被消费：`CadStructureLocator` 无配置、无状态、无多态
    // 子类（ENG-10 §5.1 未给它任何注入项），故直接内嵌。Stages 里保留该
    // 字段会让"注入了一个 cad 却不起作用"成为可能，故本文件**不读它** ——
    // 该字段仅用于让调用方显式表达"我不替换 CAD 定位器"。
    (void)stages.cad;
}

PosePipeline::~PosePipeline() = default;

// ---------------------------------------------------------------------------
//  静态辅助
// ---------------------------------------------------------------------------

const data::ImageFrame& PosePipeline::frameOf(const data::MultiCameraFrame& frame,
                                              data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return frame.cam25;
    case data::CameraRole::CAM50:  return frame.cam50;
    case data::CameraRole::CAM100: return frame.cam100;
    }
    // 不可达：ENG-09 §4.1 只冻结三个角色，且 switch 已穷尽。
    // 返回 cam25 的引用而不是抛异常：算法层不负责错误分类
    // （IF-SW-02 的约定：失败原因由 MeasurementStrategy 判定）。
    return frame.cam25;
}

const data::CameraCalibration& PosePipeline::calibOf(
    const data::OpticalRigCalibration& rig, data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return rig.cam25;
    case data::CameraRole::CAM50:  return rig.cam50;
    case data::CameraRole::CAM100: return rig.cam100;
    }
    return rig.cam25;
}

data::ImageQuality PosePipeline::assessQuality(const cv::Mat& image)
{
    data::ImageQuality q;
    const cv::Mat gray = toGray(image);
    if (gray.empty())
    {
        return q;   // 全 0：sharpness/exposure/contrast 均为 0 → 必被门槛淘汰
    }

    // 清晰度：Laplacian 方差（`ImageQuality::sharpness` 的注释允许
    // "Laplacian 方差或梯度能量"，见 ENG-09 §5.15）。无上界。
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar lapMean;
    cv::Scalar lapStd;
    cv::meanStdDev(lap, lapMean, lapStd);
    q.sharpness = lapStd[0] * lapStd[0];

    // 曝光：与中灰的接近程度。1 = 恰好中灰，0 = 全黑或全白。
    // ⚠ "什么算最佳曝光"没有冻结定义（`ImageQuality::exposure` 只说
    // "1 = 最佳"）。取中灰为最佳是 8 bit 无符号图像的常规选择，已记入
    // README §6。它与 ENG-10 §4.2 的 illum_band 分桶边界
    // （`MeasurementConfig::illumBandEdges`）必须一同标定。
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    q.exposure = clamp01(1.0 - std::fabs(mean[0] - 128.0) / 128.0);

    // 对比度：灰度标准差归一化（饱和点见 kContrastSaturation）。
    q.contrast = clamp01(stddev[0] / kContrastSaturation);

    return q;
}

int PosePipeline::countDetectableFeatures(const cv::Mat& gray,
                                          const cv::Rect& roi,
                                          int cap)
{
    // ⚠ 这里**不跑 SIFT**。F 分项（ENG-10 §3.6）需要 N_detect 的近似值，
    //    而 MEASURE_SELECT 的整个意义是在**昂贵计算之前**挑出通道
    //    （SYS-14 §6.2：选择必须在 PnP 之前；§15 的实时预算 < 100 ms）。
    //    对三个候选各跑一次全图 SIFT 会把这个阶段变成整链最慢的一步，
    //    且理由只是"想知道有多少点"。
    //
    // 因此用 Shi-Tomasi 角点数作为代理，且**只在检测框内**统计：
    //   ① 检测框内统计使该量与焦距无关（同一物理特征在不同焦段的框内
    //      点数接近），这正是 N_detect 应有的性质 —— 它要表达的是
    //      "目标身上有多少可用的特征"，不是"图像有多少像素"；
    //   ② minDistance 取框长边的 1%，使不同焦段下的判定尺度一致，
    //      否则 100 mm 的框内会因像素间距大而数出更多点。
    // 该代理与真实 SIFT 点数的关系未标定，已记入 README §6 的待裁决清单。
    if (gray.empty() || cap <= 0)
    {
        return 0;
    }

    cv::Rect box = roi & cv::Rect(0, 0, gray.cols, gray.rows);
    if (box.width < 3 || box.height < 3)
    {
        return 0;
    }

    const double longSide = static_cast<double>(std::max(box.width, box.height));
    const double minDistance = std::max(1.0, longSide * 0.01);

    const cv::Mat patch = gray(box);
    std::vector<cv::Point2f> corners;
    try
    {
        cv::goodFeaturesToTrack(patch, corners, cap, 0.01, minDistance);
    }
    catch (const cv::Exception&)
    {
        return 0;
    }
    return static_cast<int>(corners.size());
}

// ---------------------------------------------------------------------------
//  粗姿态
// ---------------------------------------------------------------------------

void PosePipeline::setCoarseAttitude(double azimuthDeg,
                                     double elevationDeg,
                                     double distanceM)
{
    coarseAzimuthDeg_ = azimuthDeg;
    coarseElevationDeg_ = elevationDeg;
    coarseDistanceM_ = distanceM;
    hasCoarseAttitude_ = std::isfinite(azimuthDeg)
                      && std::isfinite(elevationDeg)
                      && std::isfinite(distanceM)
                      && distanceM > 0.0;
}

// ---------------------------------------------------------------------------
//  统计量推送（裁决 C-002）
// ---------------------------------------------------------------------------

void PosePipeline::setStatisticsObserver(IPipelineObserver* observer)
{
    // 只存指针，不做任何事 —— 包括**不清空** `lastMatchResult_`。
    // 换观察者不该影响算法状态：那组"内部状态查询"访问器虽然已在
    // 2026-09-23 的清理中删除，但 `lastMatchResult_` 仍是
    // `publishStatistics()` 的数据源；若在这里清缓存，一个刚挂上来的
    // 观察者会让它收到的第一次推送退化为全 0（`MatchResult` 的默认值），
    // 而那是观察者的接入动作造成的，不是测量事实。
    statisticsObserver_ = observer;
}

void PosePipeline::publishStatistics()
{
    if (statisticsObserver_ == nullptr)
    {
        return;   // 未挂观察者：算法照常工作（见 IPipelineObserver.h）
    }

    // ⚠ 本处是 `MatchResult`（算法层）→ `MeasurementStatistics`（data 层）
    // 的**唯一**转换点，映射必须**逐字段恒等**，不做裁剪也不做换算
    // （MeasurementStatistics.h 明文要求）。任何"顺手归一化""取个百分比"
    // 都会让 data 层出现一个算法语义，且量纲与算法侧悄悄不一致。
    data::MeasurementStatistics statistics;
    statistics.featureCount      = lastMatchResult_.totalCandidates;
    statistics.matchCount        = lastMatchResult_.matchCount;
    statistics.droppedByConflict = lastMatchResult_.droppedByConflict;
    statistics.cadCount          = lastMatchResult_.cadCount;
    statistics.textureCount      = lastMatchResult_.textureCount;
    statistics.spreadPx          = lastMatchResult_.spreadPx;

    statisticsObserver_->onStatistics(statistics);
}

// ---------------------------------------------------------------------------
//  SEARCH
// ---------------------------------------------------------------------------

bool PosePipeline::detect(const data::ImageFrame& frame,
                          data::DetectionResult& out)
{
    out = data::DetectionResult{};
    if (detector_ == nullptr || frame.image.empty())
    {
        return false;
    }
    const bool found = detector_->detect(frame, out);
    if (!found)
    {
        out = data::DetectionResult{};
        return false;
    }
    // IF-SW-02（IPosePipeline.h）约定：返回 true 时 `found` 必为 true。
    // 检测器若返回 true 却把 found 留成 false，那是"检出但未命中"这种
    // 无法与"没跑检测"区分的状态，此处统一成 false（并把 out 清空）。
    if (!out.found)
    {
        out = data::DetectionResult{};
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
//  TARGET_FOUND
// ---------------------------------------------------------------------------

bool PosePipeline::estimateScale(const data::ImageFrame& frame,
                                 const data::DetectionResult& detection,
                                 data::TargetScaleEstimate& out)
{
    out = data::TargetScaleEstimate{};
    if (scale_ == nullptr || frame.image.empty() || !detection.found)
    {
        return false;
    }

    // ⚠ 标定从**构造注入的 `rig_`** 取，而不是从参数取：
    //    `estimateScale` 的冻结签名（IF-SW-02）没有标定入参，而
    //    SYS-14 §10 的 Z = f·L/(l·s) 必须有 fx。见文件头说明。
    const data::CameraCalibration& calib = calibOf(rig_, frame.role);

    // ⚠ 此处**不再**预检 `realSizeM > 0`（2026-09-23 M1 实测后移除）。
    //
    //    原先的写法是：`if (!(models_.targetRealSizeM() > 0.0)) return false;`
    //    理由（"尺寸未知则距离无从估计，且错的距离会污染 M_hist 的距离分桶
    //    ——ENG-10 §4.2"）本身仍然成立，但**这条判据放错了层**：
    //
    //    1. 对真实估计器它是**重复**的。`PinholeScaleEstimator::estimate`
    //       自己第一件事就是 `if (!(targetRealSizeM > 0.0) ||
    //       !std::isfinite(targetRealSizeM)) return false;`
    //       （TargetScaleEstimator.cpp:57）——它必须自己判，因为
    //       `realSizeM` 是它的入参，它不能假定调用方替它校验过。
    //    2. 对可替换阶段它是**致命**的。`Stages`（IF-SW-02 的注入点）
    //       的存在意义就是"只替换其中一个阶段"；而这道预检把
    //       `scale_` 阶段与 `models_` **硬绑**在一起，于是任何不依赖
    //       目标真实尺寸的估计器（如 M1 的 `MockTargetScaleEstimator`，
    //       它直接给距离、不查尺寸）都永远不会被调用到。
    //       实测后果：注入 mock 后状态机仍在 TARGET_FOUND 就地失败，
    //       报"目标尺度估计失败"，而 mock 一次都没被执行 ——
    //       一个"注入了但不生效"的桩，比没有桩更难排查。
    //    3. 这道预检在本文件里**有两份**（另一份在 `scaleEstimateOf()`，
    //       见下方注释）。两份都要去掉，只改一处会得到最坏的结果：
    //       TARGET_FOUND 能过而 MEASURE_SELECT 不能过 —— 状态机前进了
    //       三个状态，然后在一个与真实原因无关的地方失败。
    //       实测就是这样发现的：只改本处后，`px` 打印为 0.0
    //       （scaleEstimateOf 把 out 重置为默认值），
    //       MEASURE_SELECT 报"未能产出可用测量通道"。
    //
    //    移除后真实链**行为逐位不变**：尺寸为 0 时 PinholeScaleEstimator
    //    仍在同一点返回 false，`out` 仍是默认值（distance=0、
    //    confidence=0），调用方对 `confidence <= 0` 的冷启动先验路径
    //    （见 selectCamera 的说明）也一字未动。变的只是"谁来做这个判断"。
    //
    //    判据的正确归属是**估计器**：需要 L 的那个（针孔）自己拒绝，
    //    不需要 L 的那个（mock）自己通过。这也是 `Stages` 能成立的前提。
    return scale_->estimate(detection, calib, models_.targetRealSizeM(), out);
}

// ---------------------------------------------------------------------------
//  MEASURE_SELECT
// ---------------------------------------------------------------------------

bool PosePipeline::selectCamera(const data::MultiCameraFrame& frame,
                                const std::vector<data::CameraRole>& allowed,
                                data::MeasurementSelectionResult& out)
{
    out = data::MeasurementSelectionResult{};
    if (allowed.empty())
    {
        return false;
    }

    std::vector<data::MeasurementCandidate> candidates;
    candidates.reserve(allowed.size());

    const int featureCap = std::max(1, static_cast<int>(measurement_.nRef) * 4);

    for (data::CameraRole role : allowed)
    {
        const data::ImageFrame& f = frameOf(frame, role);
        if (f.image.empty())
        {
            continue;
        }

        // 每个候选都要做一次检测：检测框既是尺度估计的输入，也是
        // `minTargetPixelSize` 门槛与特征计数窗口的来源。
        data::DetectionResult det;
        if (!detect(f, det))
        {
            continue;
        }

        // 尺度估计失败（目标真实尺寸未知 / 内参未标定）**不淘汰**该候选：
        // ENG-10 §3.6 的四个分项里只有 M 需要距离分桶，Q/F/E 与距离无关。
        // 失败时 `scaleEst` 保持默认值（distance = 0、confidence = 0），
        // 而 `MeasurementSelector` 对 `confidence <= 0` 走冷启动先验 ——
        // 这样"距离未知"不会被误当成"距离 0 m"落进最近的距离带。
        data::TargetScaleEstimate scaleEst;
        scaleEstimateOf(role, det, models_.targetRealSizeM(), scaleEst);

        data::MeasurementCandidate c;
        c.camera = role;
        c.scale = scaleEst;

        const cv::Mat gray = toGray(f.image);
        c.quality = assessQuality(f.image);
        c.quality.featureCount =
            countDetectableFeatures(gray, det.bbox, featureCap);
        // 选择阶段尚未匹配（这正是设计意图：§6.2 要求选择在 PnP 之前），
        // 故 matchCount/matchRatio 保持 0，匹配率门槛在 scoreAll 中因此
        // 不生效 —— 见 MeasurementSelector.cpp 的说明。
        c.quality.matchCount = 0;
        c.quality.matchRatio = 0.0;

        c.nDetect = c.quality.featureCount;
        // W 在匹配完成后才知道真值（ENG-10 §2.4：B 类结构点 ∪ A 类关键点
        // 的展布）。选择阶段用检测框长边作为**上界代理**：
        // 实际展布不可能超过目标在图像中的尺寸，故这样得到的 E 是
        // **乐观估计**（偏小）→ eNorm 偏大 → 该候选得分偏高。
        // 对"选错相机"的代价是对称的（选错了要重试，选偏乐观会真的选错），
        // 故此处是该阶段能力边界内的一处已知偏差，已记入 README §6。
        // 好在它不改变排序方向：焦段越长、目标像素尺寸越大，
        // 代理 W 越大、E 越小、得分越高，与真实展布的趋势一致。
        c.featureSpreadPx =
            static_cast<double>(std::max(det.bbox.width, det.bbox.height));

        candidates.push_back(c);
    }

    if (candidates.empty())
    {
        return false;
    }

    // M_hist 按 target_model_id 分组（ENG-10 §4.2）。
    const std::string modelId = models_.modelId();
    if (!selector_.scoreAll(candidates, modelId))
    {
        return false;
    }
    return selector_.select(candidates, out);
}

bool PosePipeline::scaleEstimateOf(data::CameraRole role,
                                   const data::DetectionResult& detection,
                                   double realSizeM,
                                   data::TargetScaleEstimate& out) const
{
    // ⚠ 原先此处还有 `|| !(realSizeM > 0.0)`，与 estimateScale() 的那份
    //    是同一个判据的第二份副本，已一并移除（理由见 estimateScale 的
    //    长注释）。留在这里的后果不只是"多余"：`realSizeM == 0` 时它把
    //    `out` **重置为默认值**并返回 false，于是 `selectCamera` 里
    //    `c.scale.targetPixelSize` 恒为 0.0，被 `min_target_pixel_size`
    //    门槛全部淘汰 —— 现场看到的是"三路候选一个都不合格"，
    //    而三路的清晰度（222/240/270）与特征数（14）其实都远超门槛。
    //    失败原因指向门槛，真实原因却是这里把值抹成了 0。
    if (scale_ == nullptr)
    {
        out = data::TargetScaleEstimate{};
        return false;
    }
    return scale_->estimate(detection, calibOf(rig_, role), realSizeM, out);
}

// ---------------------------------------------------------------------------
//  CAPTURE
// ---------------------------------------------------------------------------

bool PosePipeline::selectBestFrame(const std::vector<data::ImageFrame>& frames,
                                   int& bestIndex,
                                   data::ImageQuality& quality)
{
    bestIndex = -1;
    quality = data::ImageQuality{};

    int best = -1;
    double bestSharpness = -1.0;

    for (size_t i = 0; i < frames.size(); ++i)
    {
        if (frames[i].image.empty())
        {
            continue;
        }
        const data::ImageQuality q = assessQuality(frames[i].image);

        // 过曝/欠曝的退化情形：整帧压到 0 或 255 时 exposure 为 0。
        // ⚠ 只有**完全退化**才在此处剔除：一张"偏暗但可用"的帧不应被丢，
        //    而"多亮算过曝"没有冻结阈值（`MeasurementConfig` 的三个门槛
        //    里没有曝光项）。已记入 README §6 的待裁决清单。
        if (q.exposure <= 0.0)
        {
            continue;
        }

        // 门槛复用 `minSharpness`（SYS-14 §6 的候选质量下限）。
        // 同一个值在两处使用是同一条配置的两个应用（与
        // FeatureExtractor 复用 `minFeatureCount` 同理），不是两套口径。
        if (measurement_.minSharpness > 0.0
            && q.sharpness < measurement_.minSharpness)
        {
            continue;
        }

        // 比较用**清晰度**：同一相机的连续帧之间，唯一显著变化的成像是
        // 模糊（转台残余振动、运动模糊），曝光与对比度在数帧内几乎不变。
        // 严格大于比较 → 并列时保留先到的那一帧（确定性，SYS-04 §6.4）。
        if (q.sharpness > bestSharpness)
        {
            bestSharpness = q.sharpness;
            best = static_cast<int>(i);
            quality = q;
        }
    }

    if (best < 0)
    {
        return false;   // 全部空帧/过曝/模糊（SYS-08 §5.7）
    }
    bestIndex = best;
    return true;
}

// ---------------------------------------------------------------------------
//  POSE_SOLVE
// ---------------------------------------------------------------------------

bool PosePipeline::solvePose(const data::MultiCameraFrame& frame,
                             data::CameraRole camera,
                             const data::CameraCalibration& calib,
                             data::ShipPoseResult& out)
{
    out = data::ShipPoseResult{};

    // 每次进入都必须清缓存：上一次任务的统计量若残留，失败任务的
    // result.json 会记录到上一次的"内点比例很高"，而这次一条对应都没有。
    lastPnpStats_ = PnpStats{};
    lastCadResult_ = CadStructureResult{};
    lastMatchResult_ = MatchResult{};
    lastCadAssisted_ = false;

    if (!models_.loaded())
    {
        return false;   // 无模型 → 无三维点 → 无从解算
    }

    const data::ImageFrame& f = frameOf(frame, camera);
    if (f.image.empty() || matcher_ == nullptr || extractor_ == nullptr
        || pnp_ == nullptr)
    {
        return false;
    }

    const data::TargetModel model = models_.get(camera);
    if (model.points3d.empty())
    {
        return false;
    }

    // ---- ① A 类特征（SIFT）----
    // 提取失败不是致命错误：ENG-10 §2.2 的整个设计意图就是"无纹理目标
    // 由 B 类支撑"，故此处不能直接 return false。
    data::FeatureSet features;
    const bool hasFeatures = extractor_->extract(f, features);
    if (!hasFeatures)
    {
        features = data::FeatureSet{};
    }

    // ---- ② B 类（CAD 结构点）----
    CadStructureResult cadResult;
    bool cadUsable = false;
    if (hasCoarseAttitude_)
    {
        data::CameraPose coarse;
        // 粗姿态只用于"把模型点投到大致位置"，§2.3 Step 1。
        const double e = coarseElevationDeg_ * kDegToRad;

        // R = Rx(-elevation) · Ry(90°)
        //
        // Ry(90°) 的含义：飞机机头（+X_aircraft）指向 **-Z_camera**，
        // 即"迎头/进近"（目标朝相机飞来）。这是本实现**唯一的机头朝向
        // 假设**，见 PosePipeline.h 文件头的 180° 二义性说明。
        // 相机系坐标轴按 ENG-09 §2.4：X 右、Y 下、Z 沿光轴向前。
        const cv::Matx33d ry90(0.0, 0.0, 1.0,
                               0.0, 1.0, 0.0,
                               -1.0, 0.0, 0.0);
        // Rx(φ) = [1,0,0; 0,cosφ,-sinφ; 0,sinφ,cosφ]，此处 φ = -e。
        const double c = std::cos(e);
        const double s = std::sin(e);
        const cv::Matx33d rx(1.0, 0.0, 0.0,
                             0.0, c, s,
                             0.0, -s, c);
        coarse.aircraftToCamera.rotation = rx * ry90;
        // 转台已对准目标 → 目标在光轴上。转台**方位角**在此不产生额外的
        // 旋转：光轴本身已被转台指向目标方位，故机头朝向与视线方向的
        // 夹角余量为 0（这正是 §2.3 允许粗姿态"数度以内"的原因）。
        // 方位角仍被记录（`coarseAzimuthDeg_`），供后续实现"光机未对准"
        // 这一般化情形时使用。
        coarse.aircraftToCamera.translation =
            cv::Vec3d(0.0, 0.0, coarseDistanceM_);

        // 检测框长边用于 ENG-10 §2.4 的"W ≥ 0.6 × 检测框长边"判据。
        data::DetectionResult det;
        double longSide = 0.0;
        if (detect(f, det))
        {
            longSide = static_cast<double>(
                std::max(det.bbox.width, det.bbox.height));
        }

        cadUsable = cad_.locate(f, model, calib, coarse, longSide, cadResult);
    }
    lastCadResult_ = cadResult;

    // ---- ③ 匹配（含 ENG-10 §2.5 冲突规则）----
    std::vector<data::FeatureCorrespondence> correspondences;
    const bool matched = matcher_->match(features, model,
                                         cadResult.correspondences,
                                         correspondences, lastMatchResult_);
    if (matched)
    {
        // ENG-10 §2.4 的 `cad_assisted`：B 类可用（≥4 对应）**且**展布达标。
        // 注意它与 matched 无关：即使最终走的是 A 类为主的对应集，
        // 只要 B 类参与进来了就必须如实记录（§7 约束 3）。
        lastCadAssisted_ = cadUsable && cadResult.spreadSufficient;
    }
    else if (cadUsable)
    {
        // 退化路径：A 类不足（而 `minFeatureCount` 门槛正是按 A 类的产能
        // 定的），但 B 类自身已提供 ≥4 个对应。§2.4 明确允许"少量 B 类
        // 结构点即可支撑 PnP"，故此处**不放弃**，直接用 B 类对应集。
        // 没有这条路径，B 类在它最该起作用的场合（无纹理目标）反而
        // 永远用不上。
        correspondences = cadResult.correspondences;
        // 统计量按"实际参与 PnP 的集合"重算：`lastMatchResult_` 对外表达的
        // 是本次解算用了什么，不是匹配函数内部试过什么。A 类候选的失败
        // 情况由 `features.keypoints.size()` 与检测结果反映，不在此处重复。
        lastMatchResult_ = MatchResult{};
        lastMatchResult_.totalCandidates = static_cast<int>(correspondences.size());
        lastMatchResult_.matchCount = lastMatchResult_.totalCandidates;
        lastMatchResult_.cadCount = lastMatchResult_.totalCandidates;
        lastMatchResult_.spreadPx = cadResult.spreadPx;
        lastCadAssisted_ = cadResult.spreadSufficient;
    }

    // 统计量到此**已经成立**：无论上面走的是"融合匹配"还是"纯 B 类退化"
    // 分支，`lastMatchResult_` 都已填好，且它表达的正是"本次解算用了什么"。
    // 推送点放在这里而不是成功返回之后：下面还有 PnP 与方法链两处可能早退，
    // 而"匹配成功但未收敛"这一情形事后最需要统计量来解释
    // （有对应点却没解出位姿，只能从对应点的构成去找原因）。
    publishStatistics();

    if (correspondences.empty())
    {
        return false;
    }

    // ---- ④ PnP + RANSAC ----
    data::CameraPose cameraPose;
    if (!pnp_->estimate(correspondences, calib, cameraPose, lastPnpStats_))
    {
        return false;
    }

    // ---- ⑤ 坐标合成：Camera → Rig → Ship ----
    //
    // ENG-09 §2.1 的链方向：aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
    // 平移项必须按"先转到目标系再加平移"展开：
    //   p_ship = R_rs·(R_cr·(R_ac·p_air + t_ac) + t_cr) + t_rs
    const cv::Matx33d rAc = cameraPose.aircraftToCamera.rotation;
    const cv::Vec3d  tAc = cameraPose.aircraftToCamera.translation;
    const cv::Matx33d rCr = calib.cameraToRig.rotation;
    const cv::Vec3d  tCr = calib.cameraToRig.translation;
    const cv::Matx33d rRs = rig_.rigToShip.rotation;
    const cv::Vec3d  tRs = rig_.rigToShip.translation;

    out.aircraftToShip.rotation = rRs * rCr * rAc;
    out.aircraftToShip.translation = rRs * (rCr * tAc + tCr) + tRs;

    decomposeZyxDeg(out.aircraftToShip.rotation,
                    out.yaw, out.pitch, out.roll);
    out.reprojectionError = cameraPose.reprojectionError;
    out.success = true;
    return true;
}

// ---------------------------------------------------------------------------
//  VALIDATE
// ---------------------------------------------------------------------------

bool PosePipeline::validate(const data::ShipPoseResult& result,
                            data::PoseValidationResult& out)
{
    // 内点比例来自 `solvePose` 缓存的 `PnpStats` —— 冻结的
    // `ShipPoseResult`（ENG-09 §5.24）里没有这个量，见 validation/
    // PoseValidator.h 的文件头。
    //
    // ⚠ 若 `solvePose` 未被调用（或调用失败），缓存是空的 →
    //    inlierRatio = 0 → 置信度 0 → 除非 `minInlierRatio`/`minConfidence`
    //    都是 0，否则判不合格。这是**有意的**：没有解算就没有可验证的对象，
    //    "验证一个没算出来的姿态"在语义上不成立。
    //
    // R05：本层**原样透传** PoseValidator 的返回值，不做任何加工，
    // 故本层的 bool 与 `out.valid` 同样恒等（契约见 IPosePipeline::validate）。
    return validator_.validate(result, lastPnpStats_.inlierRatio,
                               lastPnpStats_.inlierCount, out);
}

// ---------------------------------------------------------------------------
//  欧拉角分解
// ---------------------------------------------------------------------------

void PosePipeline::decomposeZyxDeg(const cv::Matx33d& rotation,
                                   double& yawDeg,
                                   double& pitchDeg,
                                   double& rollDeg)
{
    // ENG-09 §5.24：yaw/pitch/roll 是 ZYX 欧拉角（Yaw → Pitch → Roll），
    // 即 R = Rz(yaw)·Ry(pitch)·Rx(roll)，单位 deg（§2.2）。
    //
    //   R = [ cψcθ,  cψsθsφ - sψcφ,  cψsθcφ + sψsφ ]
    //       [ sψcθ,  sψsθsφ + cψcφ,  sψsθcφ - cψsφ ]
    //       [ -sθ,   cθsφ,            cθcφ           ]
    //
    // 由此：θ = asin(-R(2,0))，φ = atan2(R(2,1), R(2,2))，
    //       ψ = atan2(R(1,0), R(0,0))。
    const double sTheta = -rotation(2, 0);
    const double clamped = std::min(1.0, std::max(-1.0, sTheta));

    pitchDeg = std::asin(clamped) * kRadToDeg;

    if (std::fabs(clamped) >= 1.0 - 1e-12)
    {
        // 万向锁（pitch = ±90°）：此时 yaw 与 roll 只以 (ψ ∓ φ) 的组合
        // 出现，无法分别确定。按惯例把 roll 取 0，把全部转角记在 yaw 上。
        // 必须显式处理：不处理时 R(2,1)=R(2,2)=0，atan2(0,0) 返回 0，
        // 于是 roll 静默归零、而 yaw 由一个退化的 atan2 算出 ——
        // 结果既不报错也不自洽（重投影时矩阵与欧拉角对不上）。
        //
        // ⚠ 这里容易写错，推导写在下面（曾按对称的形式写成
        //    atan2(sign·R(0,1), sign·R(0,2))，两个分支都差一个负号：
        //    万向锁时算出的 yaw 与真值关于 0 镜像，而 pitch 仍是对的，
        //    于是姿态"看起来正常但转过头"，且只在 pitch 接近 ±90° 时出现）。
        //
        //   θ = +90°（cθ=0, sθ=1）：
        //     R(0,1) = cψsθsφ − sψcφ = −sψcφ + cψsφ = sin(φ − ψ)
        //     R(0,2) = cψsθcφ + sψsφ =  sψsφ + cψcφ = cos(φ − ψ)
        //     ⇒ φ − ψ = atan2(R(0,1), R(0,2))，取 φ=0 得 ψ = atan2(−R(0,1), R(0,2))
        //   θ = −90°（cθ=0, sθ=−1）：
        //     R(0,1) = −cψsφ − sψcφ = −sin(φ + ψ)
        //     R(0,2) = −cψcφ + sψsφ = −cos(φ + ψ)
        //     ⇒ φ + ψ = atan2(−R(0,1), −R(0,2))，取 φ=0 得 ψ = atan2(−R(0,1), −R(0,2))
        //
        //   两个分支只差 R(0,2) 的符号，而 R(0,1) 前的负号是**共同的** ——
        //   这正是上面写错的地方。sign 只作用在第二个参数上。
        rollDeg = 0.0;
        const double sign = clamped > 0.0 ? 1.0 : -1.0;
        yawDeg = std::atan2(-rotation(0, 1), sign * rotation(0, 2))
               * kRadToDeg;
    }
    else
    {
        // atan2 对正的公共因子不敏感，故 cosθ 不必显式除出来
        // （除以一个可能很小的量还会放大数值噪声）。
        rollDeg = std::atan2(rotation(2, 1), rotation(2, 2)) * kRadToDeg;
        yawDeg = std::atan2(rotation(1, 0), rotation(0, 0)) * kRadToDeg;
    }

    if (!std::isfinite(yawDeg) || !std::isfinite(pitchDeg)
        || !std::isfinite(rollDeg))
    {
        yawDeg = 0.0;
        pitchDeg = 0.0;
        rollDeg = 0.0;
    }
}

}  // namespace algorithm
}  // namespace aircraft
