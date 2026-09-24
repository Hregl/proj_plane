// ============================================================================
//  src/algorithm/detection/MockTargetDetector.cpp
//
//  依据：SYS-07 §4（TargetDetection）、8.md §五、ENG-10 §5.1（注入矩阵）
// ============================================================================

#include "algorithm/detection/TargetDetector.h"

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 把输入图像统一成 CV_8UC1。
///
/// 相机可能给 BGR（VirtualCamera 合成场景给的是 8UC1，真实相机按
/// CameraConfig 可能是彩色）。三通道时就地转灰度，**不改动入参**——
/// `ImageFrame::image` 与预览线程共享（SYS-04 §4.4：预览与测量共享
/// shared_ptr，不复制图像），就地修改会污染另一线程正在显示的图。
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

}  // namespace

// ---------------------------------------------------------------------------

MockTargetDetector::MockTargetDetector(const data::MeasurementConfig& config)
    : config_(config)
{
}

void MockTargetDetector::forceDetection(const data::DetectionResult& result)
{
    forced_ = true;
    forcedFound_ = true;
    forcedResult_ = result;
}

void MockTargetDetector::forceMiss()
{
    forced_ = true;
    forcedFound_ = false;
}

bool MockTargetDetector::detect(const data::ImageFrame& frame,
                                data::DetectionResult& out)
{
    // ---- 强制模式（测试夹具）----
    if (forced_)
    {
        if (!forcedFound_)
        {
            out = data::DetectionResult{};
            return false;
        }
        out = forcedResult_;
        if (out.sourceCamera != frame.role)
        {
            // 强制结果里的 camera 字段常被测试写错。这里以**实际入参**为准：
            // sourceCamera 的语义是"该结果来自哪台相机的图像"，
            // 而这是 frame.role 的事实，不应该由调用者声明。
            out.sourceCamera = frame.role;
        }
        return true;
    }

    out = data::DetectionResult{};

    const cv::Mat gray = toGray(frame.image);
    if (gray.empty() || gray.type() != CV_8UC1)
    {
        return false;
    }

    // ---- 阈值分割 ----
    double minVal = 0.0;
    double maxVal = 0.0;
    cv::minMaxLoc(gray, &minVal, &maxVal);
    if (!(maxVal > minVal))
    {
        // 全画幅同灰度：没有目标，也没有背景差异。
        return false;
    }

    const double threshold = (maxVal + minVal) / 2.0;

    cv::Mat mask;
    cv::threshold(gray, mask, threshold, 255.0, cv::THRESH_BINARY);

    // ---- 取最大连通域 ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return false;
    }

    const auto largest = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b)
        { return cv::contourArea(a) < cv::contourArea(b); });

    const cv::Rect bbox = cv::boundingRect(*largest);

    // ---- 最小尺寸门槛（ENG-10 §5.1 的 minTargetPixelSize 注入行）----
    //
    // ⚠ 这个门槛的物理含义是"该焦段图像中的目标小到无法用于 PnP"
    // （SYS-14 §7.2 的尺度评分），因此它的取值必须对照**最短焦段**
    // （CAM25，SEARCH 唯一使用的通道）来定：
    //     l ≈ f_x · L / Z
    //     f_x ≈ 25mm / 3.45µm ≈ 7247 px，L = 10 m（飞机全长），Z = 300 m
    //     → l ≈ 242 px
    // 若把门槛设到数百像素，SEARCH 会在远距时直接检不出目标，
    // 而状态机会停在 SEARCH 直到 T_task 并以 9001 收场 —— 报错方向完全
    // 误导（真实原因是配置值取得过大）。故门槛值必须远小于上式结果。
    const double longSide =
        static_cast<double>(std::max(bbox.width, bbox.height));
    if (longSide < config_.minTargetPixelSize)
    {
        return false;
    }

    // ---- 填充度作为置信度 ----
    // 合成场景中矩形目标填充度接近 1；真实场景不适用（见头文件说明）。
    const double area = cv::contourArea(*largest);
    const double boxArea =
        static_cast<double>(std::max(1, bbox.width * bbox.height));
    const double fill = std::min(1.0, std::max(0.0, area / boxArea));

    out.found = true;
    out.bbox = bbox;
    out.confidence = fill;
    out.sourceCamera = frame.role;
    return true;
}

}  // namespace algorithm
}  // namespace aircraft
