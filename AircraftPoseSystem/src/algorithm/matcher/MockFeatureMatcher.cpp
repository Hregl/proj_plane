// ============================================================================
//  src/algorithm/matcher/MockFeatureMatcher.cpp
//
//  依据：SYS-07 §9、ENG-10 §2.5（融合与冲突规则）、8.md §十
//
//  本文件含两个实现：DescriptorFeatureMatcher（真实链）与
//  MockFeatureMatcher（测试夹具）。二者放在同一编译单元，使
//  ENG-10 §2.5 的冲突规则只有一处实现，且夹具不会与真实实现产生口径差异。
// ============================================================================

#include "algorithm/matcher/FeatureMatcher.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 冲突规则的"同一区域"半径，单位 pixel。
///
/// ENG-10 §2.5 只给了一个阈值："反投影后位置差 > 2 pixel 时以 B 类为准"。
/// 本实现把这个 2 pixel **同时用作区域半径**，而不是另取一个"邻近"阈值 ——
/// 理由：§2.5 的判据本质是"同一处出现了两个互相矛盾的对应"，而"同一处"
/// 与"位置差可容忍多少"在图像上本是同一个尺度。另取一个阈值会引入一个
/// 无冻结依据的可调参数（SYS-14 §20 约束 3 反对这种参数）。
/// 该选择已记入 README §6 的待裁决清单。
constexpr double kConflictRadiusPx = 2.0;

/// 两个三维点是否"同一个点"（用于判断冲突是真冲突还是同一个点的重复对应）。
/// 阈值取 1e-6 m = 1 µm：远小于任何可制造的结构点间距，只用来吸收
/// float→double 与 yaml 读入的舍入差。
constexpr float kSamePointEps = 1e-6f;

bool sameObjectPoint(const cv::Point3f& a, const cv::Point3f& b)
{
    return std::fabs(a.x - b.x) <= kSamePointEps
        && std::fabs(a.y - b.y) <= kSamePointEps
        && std::fabs(a.z - b.z) <= kSamePointEps;
}

/// 横向展布宽度 W：图像 X 方向两端之差（与 CadStructureLocator 口径一致）。
double spreadWidth(const std::vector<data::FeatureCorrespondence>& corr)
{
    if (corr.size() < 2)
    {
        return 0.0;
    }
    float lo = corr.front().imagePoint.x;
    float hi = lo;
    for (const data::FeatureCorrespondence& c : corr)
    {
        lo = std::min(lo, c.imagePoint.x);
        hi = std::max(hi, c.imagePoint.x);
    }
    return static_cast<double>(hi - lo);
}

}  // namespace

// ---------------------------------------------------------------------------
//  DescriptorFeatureMatcher
// ---------------------------------------------------------------------------

DescriptorFeatureMatcher::DescriptorFeatureMatcher(
    const data::MeasurementConfig& config)
    : config_(config)
{
}

void DescriptorFeatureMatcher::setRatioThreshold(double ratio)
{
    // 越界的比值没有意义：0 会拒绝全部匹配，≥1 等于不做 Ratio Test。
    // 夹取而不是忽略，使误用表现为"匹配率异常"（可见），而不是静默失效。
    ratioThreshold_ = std::min(1.0, std::max(0.0, ratio));
}

bool DescriptorFeatureMatcher::match(
    const data::FeatureSet& imageFeatures,
    const data::TargetModel& model,
    const std::vector<data::FeatureCorrespondence>& cadCorrespondences,
    std::vector<data::FeatureCorrespondence>& out,
    MatchResult& stats)
{
    out.clear();
    stats = MatchResult{};

    std::vector<data::FeatureCorrespondence> merged;

    // ---- ① A 类：描述子匹配（Descriptor Matching + Ratio Test）----
    //
    // B 类先入 merged（ENG-10 §2.5："B 类全部参与"），冲突规则再按
    // "以 B 类为准"剔除 A 类 —— 顺序反过来的话，被剔除的就会是 B 类。
    merged.insert(merged.end(), cadCorrespondences.begin(),
                  cadCorrespondences.end());

    if (!imageFeatures.descriptors.empty() && !model.features.empty())
    {
        // 模型侧的描述子矩阵（N×D）。
        cv::Mat modelDescriptors;
        std::vector<int> modelPointIndex;   // 行 i → model.points3d 的下标
        for (const data::FeatureDescriptor& f : model.features)
        {
            if (f.descriptor.empty())
            {
                continue;
            }
            if (modelDescriptors.empty())
            {
                modelDescriptors = f.descriptor.clone();
                // 保证每行一行：单条描述子可能是 1×D 的行视图。
                modelDescriptors = modelDescriptors.reshape(1, 1);
            }
            else
            {
                if (f.descriptor.cols != modelDescriptors.cols)
                {
                    // 维度不一致的描述子无法同矩阵存放。跳过而不是补齐：
                    // 补齐会产生一个"看起来合法"的行，其匹配结果无意义。
                    continue;
                }
                cv::Mat row = f.descriptor.reshape(1, 1);
                modelDescriptors.push_back(row);
            }
            modelPointIndex.push_back(f.point3dIndex);
        }

        if (!modelDescriptors.empty()
            && modelDescriptors.type() == imageFeatures.descriptors.type())
        {
            cv::BFMatcher matcher(cv::NORM_L2);
            std::vector<std::vector<cv::DMatch>> knn;
            // k=2：Ratio Test 需要最近与次近两个候选。
            matcher.knnMatch(imageFeatures.descriptors, modelDescriptors,
                             knn, 2);

            for (const std::vector<cv::DMatch>& pair : knn)
            {
                if (pair.size() < 2)
                {
                    continue;
                }
                const cv::DMatch& best = pair[0];
                const cv::DMatch& second = pair[1];
                if (best.distance >= ratioThreshold_ * second.distance)
                {
                    continue;   // Ratio Test 未通过
                }
                if (best.queryIdx < 0
                    || best.queryIdx >= static_cast<int>(
                           imageFeatures.keypoints.size()))
                {
                    continue;
                }
                if (best.trainIdx < 0
                    || best.trainIdx >= static_cast<int>(modelPointIndex.size()))
                {
                    continue;
                }
                const int pointIdx = modelPointIndex[best.trainIdx];
                if (pointIdx < 0
                    || pointIdx >= static_cast<int>(model.points3d.size()))
                {
                    continue;
                }

                data::FeatureCorrespondence c;
                // keypoints 的 pt 是亚像素坐标（SIFT 输出即为亚像素），
                // 且已按 ENG-09 §2.4 的"左上原点、X 右 Y 下"约定。
                c.imagePoint = imageFeatures.keypoints[best.queryIdx].pt;
                c.objectPoint = model.points3d[pointIdx].position;
                merged.push_back(c);
            }
        }
    }

    stats.totalCandidates = static_cast<int>(merged.size());

    // ---- ② 冲突规则（ENG-10 §2.5 冻结：以 B 类为准）----
    //
    // 判据：B 类点与 A 类点在图像上落在同一区域（≤ 2 pixel），却指向
    // **不同**的三维点 —— 这表示其中一条对应是错的。以 B 类为准，
    // 因为 B 类的三维坐标是精确已知的 CAD 结构点，而 A 类点是通过描述子
    // 匹配**间接**关联到三维模型的，多一层不确定性。
    //
    // ⚠ 判据在**图像**上而不是三维上："反投影后位置差"需要先有位姿，
    // 而位姿在本阶段还没有（RANSAC/PnP 在下一步）。图像距离是同一事实的
    // 无位姿表达，且 ENG-10 §2.5 给的阈值本身就是像素量。
    std::vector<bool> isCad(merged.size(), false);
    for (size_t i = 0; i < merged.size(); ++i)
    {
        isCad[i] = i < cadCorrespondences.size();
    }

    for (size_t i = 0; i < merged.size(); ++i)
    {
        if (isCad[i])
        {
            continue;   // B 类全部参与
        }
        for (size_t j = 0; j < merged.size(); ++j)
        {
            if (!isCad[j])
            {
                continue;
            }
            if (sameObjectPoint(merged[i].objectPoint, merged[j].objectPoint))
            {
                continue;   // 同一个三维点的重复对应，不是冲突
            }
            const double d = cv::norm(merged[i].imagePoint
                                      - merged[j].imagePoint);
            if (d <= kConflictRadiusPx)
            {
                out.push_back(merged[j]);        // 保留 B 类
                ++stats.droppedByConflict;
                break;                           // 该 A 类点已被剔除
            }
        }
    }

    // ---- ③ 组装结果 ----
    // 先放 B 类（全部），再放冲突规则后幸存的 A 类。
    for (size_t i = 0; i < merged.size(); ++i)
    {
        if (isCad[i])
        {
            out.push_back(merged[i]);
            ++stats.cadCount;
        }
    }
    for (size_t i = 0; i < merged.size(); ++i)
    {
        if (isCad[i])
        {
            continue;
        }
        bool dropped = false;
        for (size_t j = 0; j < merged.size() && !dropped; ++j)
        {
            if (!isCad[j]
                || sameObjectPoint(merged[i].objectPoint, merged[j].objectPoint))
            {
                continue;
            }
            if (cv::norm(merged[i].imagePoint - merged[j].imagePoint)
                <= kConflictRadiusPx)
            {
                dropped = true;
            }
        }
        if (!dropped)
        {
            out.push_back(merged[i]);
            ++stats.textureCount;
        }
    }

    stats.matchCount = static_cast<int>(out.size());
    stats.spreadPx = spreadWidth(out);

    // ---- ④ 门槛 ----
    if (stats.matchCount < config_.minFeatureCount)
    {
        return false;
    }
    if (stats.totalCandidates > 0 && config_.minMatchRatio > 0.0)
    {
        const double ratio = static_cast<double>(stats.matchCount)
                           / static_cast<double>(stats.totalCandidates);
        if (ratio < config_.minMatchRatio)
        {
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
//  MockFeatureMatcher
// ---------------------------------------------------------------------------

void MockFeatureMatcher::setPreset(
    const std::vector<data::FeatureCorrespondence>& preset,
    int cadCount, double spreadPx)
{
    preset_ = preset;
    // 夹取到 [0, size]：一个"B 类比总数还多"的统计量与对应集自相矛盾，
    // 而下游（ENG-10 §2.4 的 cad_assisted 判据）正是靠这个数做判断。
    cadCount_ = std::min(std::max(0, cadCount),
                         static_cast<int>(preset.size()));
    spreadPx_ = spreadPx;
}

bool MockFeatureMatcher::match(
    const data::FeatureSet&, const data::TargetModel&,
    const std::vector<data::FeatureCorrespondence>&,
    std::vector<data::FeatureCorrespondence>& out, MatchResult& stats)
{
    out.clear();
    stats = MatchResult{};

    if (forceFailure_)
    {
        return false;
    }
    if (preset_.empty())
    {
        return false;
    }

    out = preset_;
    stats.totalCandidates = static_cast<int>(preset_.size());
    stats.matchCount = static_cast<int>(preset_.size());
    stats.cadCount = cadCount_;
    stats.textureCount = stats.matchCount - cadCount_;
    stats.spreadPx = spreadPx_ > 0.0 ? spreadPx_ : spreadWidth(preset_);
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
