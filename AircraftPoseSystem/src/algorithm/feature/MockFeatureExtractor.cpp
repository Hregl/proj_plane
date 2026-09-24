// ============================================================================
//  src/algorithm/feature/MockFeatureExtractor.cpp
//
//  依据：8.md §九（MockFeatureExtractor）、ENG-09 §5.21（FeatureSet 契约）
//
//  本文件包含两个实现：SiftFeatureExtractor（真实，SYS-07 §8.2）与
//  MockFeatureExtractor（8.md §九 的测试夹具）。二者同属"特征提取"这一
//  子模块，放在同一编译单元里使 ENG-09 §5.21 的
//  "第 i 行描述子 ↔ keypoints[i]" 契约只有一处被实现、一处被校验。
// ============================================================================

#include "algorithm/feature/FeatureExtractor.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 把输入图像统一成 CV_8UC1，**不修改入参**。
///
/// `ImageFrame::image` 可能与预览线程共享（SYS-04 §4.4：预览与测量共享
/// shared_ptr<const ImageFrame>，不复制图像），就地转换会污染正在显示的图。
cv::Mat toGray(const cv::Mat& src)
{
    if (src.empty())
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

/// ENG-09 §5.21 的契约：第 i 行描述子对应 keypoints[i]。
/// 破坏它的后果是匹配结果整体错位，且**不会报错**。
bool consistent(const data::FeatureSet& set)
{
    if (set.keypoints.size() != static_cast<size_t>(set.descriptors.rows))
    {
        return false;
    }
    if (set.descriptors.empty())
    {
        return set.keypoints.empty();
    }
    return set.descriptors.type() == CV_32F && set.descriptors.cols > 0;
}

}  // namespace

// ---------------------------------------------------------------------------
//  SiftFeatureExtractor
// ---------------------------------------------------------------------------

SiftFeatureExtractor::SiftFeatureExtractor(int minFeatureCount)
    : minFeatureCount_(minFeatureCount)
{
}

bool SiftFeatureExtractor::extract(const data::ImageFrame& frame,
                                   data::FeatureSet& out)
{
    out = data::FeatureSet{};

    const cv::Mat gray = toGray(frame.image);
    if (gray.empty() || gray.type() != CV_8UC1)
    {
        return false;
    }

    // 每帧新建 SIFT 对象：`cv::SIFT` 内部持有尺度空间金字塔的临时缓冲，
    // 复用同一个对象在多线程下不安全（SYS-09 §15 要求算法可被判为独立线程），
    // 而构造开销相对一次全图 SIFT 提取可忽略。
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    sift->detectAndCompute(gray, cv::noArray(), out.keypoints, out.descriptors);

    if (!consistent(out))
    {
        out = data::FeatureSet{};
        return false;
    }

    // 点数门槛：不足即视为"本帧不可用"。
    // 这里的门槛与 MeasurementSelector 的 `minFeatureCount` 门槛是**同一个
    // 配置项的两处应用**（ENG-10 §5.1 的注入矩阵把 minFeatureCount 给了
    // FeatureMatcher，此处是它的上游）：两处都用同一个值不会冲突，
    // 只是让失败在更早的阶段暴露。
    if (static_cast<int>(out.keypoints.size()) < minFeatureCount_)
    {
        out = data::FeatureSet{};
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
//  MockFeatureExtractor
// ---------------------------------------------------------------------------

MockFeatureExtractor::MockFeatureExtractor(const data::FeatureSet& preset)
{
    if (!setPreset(preset))
    {
        hasPreset_ = false;
        preset_ = data::FeatureSet{};
    }
}

bool MockFeatureExtractor::setPreset(const data::FeatureSet& preset)
{
    if (!consistent(preset))
    {
        // 不一致的预设**不被采用**（而不是"采用了但下游出错"）：
        // 一个错位的特征集在匹配阶段会给出看似合理的低匹配率，
        // 使测试断言失败在"匹配率不够"而不是"夹具写错了"上。
        return false;
    }
    preset_ = preset;
    hasPreset_ = true;
    return true;
}

bool MockFeatureExtractor::extract(const data::ImageFrame&,
                                   data::FeatureSet& out)
{
    if (!hasPreset_ || preset_.keypoints.empty())
    {
        out = data::FeatureSet{};
        return false;
    }
    out = preset_;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
