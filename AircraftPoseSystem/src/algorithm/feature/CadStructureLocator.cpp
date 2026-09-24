// ============================================================================
//  src/algorithm/feature/CadStructureLocator.cpp
//
//  依据：ENG-10 §2.3（投影—匹配—拟合三步法）/§2.4（可用性判据）、
//        SYS-07 §8.2（裁决 G-1）、SYS-15 §4.5（展布宽度 W）
// ============================================================================

#include "algorithm/feature/CadStructureLocator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace aircraft
{
namespace algorithm
{

namespace
{

/// 亚像素拟合残差上限，单位 pixel（ENG-10 §2.4 **冻结**：超过则该结构点
/// 剔除）。不是可调参数，故不做成配置项 —— 改它等于改判据。
constexpr double kMaxResidualPx = 0.3;

/// 展布判据比例（ENG-10 §2.4 冻结：W ≥ 0.6 × 检测框长边）。
constexpr double kSpreadRatio = 0.6;

/// B 类可用所需的最少对应数（ENG-10 §2.4 冻结：少于 4 个无法构成几何约束）。
constexpr int kMinCadCorrespondences = 4;

/// 两条棱线的最小夹角（度）。低于它则认为窗口内只有一条棱线，
/// 交点不是结构点（而是同一条直线的伪交）。
///
/// ⚠ 这个 20° 是本实现引入的判据，冻结文档未给。取 20° 的理由：
/// 直线拟合的方向噪声在低对比度边缘上约 1~2°，20° 相当于 10 倍余量，
/// 而真实飞机结构棱线之间的夹角（翼/身交线、尾翼边缘与机身）通常远大于
/// 30°。已记入 README §6 的待裁决清单。
constexpr double kMinCrossingAngleDeg = 20.0;

/// 窗口半径的下限与上限，单位 pixel。
///
/// 半径本身是**数据驱动**的：取投影点最近邻间距中位数的一半
/// （见 windowRadius()）。这两个数是**护栏**，不是整定参数：
///   · 小于 4 px 时拟合一条直线只有几个像素参与，亚像素精度无从谈起；
///   · 大于 60 px 时窗口会跨越多个结构（机身、机翼、尾翼），
///     提取到的两组"主方向边缘"很可能来自不同部件，交点便无意义。
constexpr double kWindowMinPx = 4.0;
constexpr double kWindowMaxPx = 60.0;

/// 窗口内至少需要多少个边缘像素才尝试拟合（一条直线最少 2 点，
/// 但 2 点的拟合残差恒为 0，没有判别力；取 8 使残差判据有效）。
constexpr int kMinEdgePixelsPerLine = 8;

/// 由投影点间距估计的窗口半径（pixel）。见 kWindowMinPx 的说明。
double windowRadius(const std::vector<cv::Point2f>& projected)
{
    if (projected.size() < 2)
    {
        return kWindowMinPx;
    }

    // 每个点到最近邻的距离，取中位数。
    // 用中位数而非均值：结构点在图像上的分布本就不均（机头处密、机身处疏），
    // 均值会被最密的一簇拉小，使窗口在稀疏处过小而取不到完整棱线。
    std::vector<double> nearest;
    nearest.reserve(projected.size());
    for (size_t i = 0; i < projected.size(); ++i)
    {
        double best = -1.0;
        for (size_t j = 0; j < projected.size(); ++j)
        {
            if (i == j)
            {
                continue;
            }
            const double d = cv::norm(projected[i] - projected[j]);
            if (best < 0.0 || d < best)
            {
                best = d;
            }
        }
        if (best > 0.0)
        {
            nearest.push_back(best);
        }
    }
    if (nearest.empty())
    {
        return kWindowMinPx;
    }
    std::nth_element(nearest.begin(),
                     nearest.begin() + static_cast<long>(nearest.size() / 2),
                     nearest.end());
    const double median = nearest[nearest.size() / 2];

    return std::min(kWindowMaxPx,
                    std::max(kWindowMinPx, median / 2.0));
}

/// 一条由 cv::fitLine 得到的 2D 直线：方向向量 + 过定点。
struct Line2D
{
    cv::Point2f dir{1.0f, 0.0f};
    cv::Point2f point{0.0f, 0.0f};
    bool valid = false;
};

/// 两直线求交。返回 false 表示平行（或退化）。
bool intersect(const Line2D& a, const Line2D& b, cv::Point2f& out)
{
    if (!a.valid || !b.valid)
    {
        return false;
    }
    const double cross = static_cast<double>(a.dir.x) * b.dir.y
                       - static_cast<double>(a.dir.y) * b.dir.x;
    if (std::fabs(cross) < 1e-9)
    {
        return false;
    }
    const double dx = static_cast<double>(b.point.x - a.point.x);
    const double dy = static_cast<double>(b.point.y - a.point.y);
    const double t = (dx * b.dir.y - dy * b.dir.x) / cross;
    out = cv::Point2f(static_cast<float>(a.point.x + t * a.dir.x),
                      static_cast<float>(a.point.y + t * a.dir.y));
    return std::isfinite(out.x) && std::isfinite(out.y);
}

/// 一组像素到拟合直线的**中位**绝对距离（pixel）。
///
/// ⚠ 与 ENG-10 §2.4 的措辞差异：§2.4 只说"亚像素拟合残差 ≤ 0.3 pixel"，
///    没有规定用哪个统计量。本实现取**中位数**而不是 RMS，理由如下
///    （这是一处实现选择，已在 README §6 登记）：
///
///    1. 窗口内的像素集不可能只有这一条棱线。分组依据是**梯度方向**
///       （见 Step 2 的倍角聚类），而结构端头的横向棱、纹理边缘、压缩
///       伪影都可能带上同方向的梯度而落进同一组。实测：一个 164 点的组里
///       混进 8 个距拟合直线约 41 pixel 的端头像素，RMS 就从 0.5 pixel
///       抬到 9.476 pixel —— 也就是说 0.3 pixel 判据会退化成
///       "只要有一个杂散像素就丢弃该结构点"。其后果恰是 ENG-10 §2.2
///       要避免的：结构点全被丢弃 → B 类判为不可用 → 退回纯 A 类。
///    2. 中位数对少数离群像素免疫，但**判别力没有损失**：若两组把"相距
///       约 1.5 pixel 的两条平行棱"误并成一组，中位数会给出 ≈0.75 pixel
///       （> 0.3）而照常拒绝 —— 这正是这个判据真正要拦住的情形。
///
///    fitLine 用 `cv::DIST_HUBER`（同样的理由：它对离群点降权，而 DIST_L2
///    会把直线的位置往离群点上拽；直线位置一旦被拽偏，交点随之偏移，
///    而**交点才是输出量**）。
inline double medianAbsDistance(const std::vector<cv::Point2f>& pts,
                                const Line2D& line)
{
    if (!line.valid || pts.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    std::vector<double> dist;
    dist.reserve(pts.size());
    for (const cv::Point2f& p : pts)
    {
        const double vx = static_cast<double>(p.x - line.point.x);
        const double vy = static_cast<double>(p.y - line.point.y);
        // dir 由 fitLine 给出，已是单位向量，故叉积即点到直线的距离。
        dist.push_back(std::fabs(vx * line.dir.y - vy * line.dir.x));
    }
    const size_t mid = dist.size() / 2;
    std::nth_element(dist.begin(), dist.begin() + static_cast<long>(mid),
                     dist.end());
    return dist[mid];
}

}  // namespace

// ---------------------------------------------------------------------------

double CadStructureLocator::spreadWidth(
    const std::vector<data::FeatureCorrespondence>& correspondences)
{
    if (correspondences.size() < 2)
    {
        return 0.0;
    }
    float minX = correspondences.front().imagePoint.x;
    float maxX = minX;
    for (const data::FeatureCorrespondence& c : correspondences)
    {
        minX = std::min(minX, c.imagePoint.x);
        maxX = std::max(maxX, c.imagePoint.x);
    }
    return static_cast<double>(maxX - minX);
}

// ---------------------------------------------------------------------------

bool CadStructureLocator::locate(const data::ImageFrame& frame,
                                 const data::TargetModel& model,
                                 const data::CameraCalibration& calibration,
                                 const data::CameraPose& coarsePose,
                                 double detectionLongSidePx,
                                 CadStructureResult& out) const
{
    out = CadStructureResult{};

    if (frame.image.empty())
    {
        return false;
    }
    if (calibration.cameraMatrix.empty()
        || calibration.cameraMatrix.type() != CV_64F
        || calibration.imageWidth <= 0 || calibration.imageHeight <= 0)
    {
        return false;
    }

    cv::Mat gray;
    if (frame.image.channels() == 1)
    {
        gray = frame.image;
    }
    else
    {
        cv::cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
    }
    if (gray.type() != CV_8UC1)
    {
        return false;
    }

    // ---- Step 1：按粗姿态投影 CAD 结构点 ----
    //
    // 只取 featureType == "cad" 的点（ENG-10 §2.2 的 B 类）。
    // `Transform::rotation` 是 3x3 CV_64F，`cv::projectPoints` 的
    // rvec/tvec 需从它转换 —— Rodrigues 在此处是唯一被允许的"旋转表示
    // 转换"，因为它不改变旋转本身。
    std::vector<cv::Point3f> objectPoints;
    std::vector<size_t> modelIndices;
    for (size_t i = 0; i < model.points3d.size(); ++i)
    {
        if (model.points3d[i].featureType == "cad")
        {
            objectPoints.push_back(model.points3d[i].position);
            modelIndices.push_back(i);
        }
    }
    if (static_cast<int>(objectPoints.size()) < kMinCadCorrespondences)
    {
        // 模型的 CAD 结构点本身就不足 4 个 —— 这是模型问题，不是图像问题。
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    {
        cv::Matx33d r = coarsePose.aircraftToCamera.rotation;
        cv::Mat rmat(r);
        if (!cv::checkRange(rmat) || std::fabs(cv::determinant(rmat)) < 1e-12)
        {
            return false;
        }
        cv::Rodrigues(rmat, rvec);
        const cv::Vec3d t = coarsePose.aircraftToCamera.translation;
        tvec = (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, calibration.cameraMatrix,
                      calibration.distortion, projected);

    // 只保留落在图像内的投影点（含窗口外沿会越界的部分稍后另判）。
    std::vector<size_t> visible;      // 索引进 objectPoints / projected
    std::vector<cv::Point2f> visiblePts;
    for (size_t i = 0; i < projected.size(); ++i)
    {
        if (!std::isfinite(projected[i].x) || !std::isfinite(projected[i].y))
        {
            continue;
        }
        if (projected[i].x < 0.0f || projected[i].y < 0.0f
            || projected[i].x >= static_cast<float>(calibration.imageWidth)
            || projected[i].y >= static_cast<float>(calibration.imageHeight))
        {
            continue;
        }
        visible.push_back(i);
        visiblePts.push_back(projected[i]);
    }
    out.projectedCount = static_cast<int>(visible.size());
    if (out.projectedCount < kMinCadCorrespondences)
    {
        return false;
    }

    const double radius = windowRadius(visiblePts);

    // ---- Step 2 + 3：窗口内提取两条主方向棱线，求交 ----
    for (size_t k = 0; k < visible.size(); ++k)
    {
        const cv::Point2f p = visiblePts[k];

        const int x0 = std::max(0, static_cast<int>(std::floor(p.x - radius)));
        const int y0 = std::max(0, static_cast<int>(std::floor(p.y - radius)));
        const int x1 = std::min(calibration.imageWidth - 1,
                                static_cast<int>(std::ceil(p.x + radius)));
        const int y1 = std::min(calibration.imageHeight - 1,
                                static_cast<int>(std::ceil(p.y + radius)));
        if (x1 - x0 < 4 || y1 - y0 < 4)
        {
            continue;
        }

        const cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
        const cv::Mat window = gray(roi);

        // Canny 的两个阈值按窗口内的灰度中位数取：
        // 固定阈值在不同曝光下表现差异极大，而本方法的输入恰恰是
        // 三个不同焦段、不同曝光的相机。
        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(window, mean, stddev);
        const double lo = std::max(10.0, mean[0] * 0.66 - stddev[0]);
        const double hi = std::max(lo * 2.0, mean[0] * 1.33 + stddev[0]);

        cv::Mat edges;
        cv::Canny(window, edges, lo, hi);

        std::vector<cv::Point2f> edgePts;
        edgePts.reserve(static_cast<size_t>(cv::countNonZero(edges)));
        for (int y = 0; y < edges.rows; ++y)
        {
            const uchar* row = edges.ptr<uchar>(y);
            for (int x = 0; x < edges.cols; ++x)
            {
                if (row[x] != 0)
                {
                    // ⚠ **转回全图坐标**：拟合与求交都在全图坐标系里进行。
                    // 窗口偏移 x0/y0 一旦漏加，结果只是"整体偏了半个窗口"，
                    // 残差判据照样通过、不会报错 —— 本类最隐蔽的一处错。
                    edgePts.emplace_back(static_cast<float>(x0 + x),
                                         static_cast<float>(y0 + y));
                }
            }
        }

        if (static_cast<int>(edgePts.size()) < 2 * kMinEdgePixelsPerLine)
        {
            continue;
        }

        // 按梯度方向把边缘像素分成两组（两条棱线）。
        //
        // 用方向角的**倍角**做一维聚类，而不是直接对角度聚类：相差 180° 的
        // 两个梯度方向属于**同一条**直线（一条棱线的两侧各产生一条边缘，
        // 梯度方向相反）。直接聚类会把它们分成两组，于是"两条棱线"
        // 退化成"一条棱线的两侧"，求交得到的点毫无意义。
        cv::Mat gx;
        cv::Mat gy;
        cv::Sobel(window, gx, CV_32F, 1, 0, 3);
        cv::Sobel(window, gy, CV_32F, 0, 1, 3);

        std::vector<double> angles;      // 倍角表示，范围 [0, 2π)
        angles.reserve(edgePts.size());
        for (const cv::Point2f& q : edgePts)
        {
            const int wx = std::min(std::max(static_cast<int>(q.x) - x0, 0),
                                    gx.cols - 1);
            const int wy = std::min(std::max(static_cast<int>(q.y) - y0, 0),
                                    gx.rows - 1);
            const double dx = gx.at<float>(wy, wx);
            const double dy = gy.at<float>(wy, wx);
            const double a = std::atan2(dy, dx);
            angles.push_back(std::fmod(a * 2.0 + 2.0 * CV_PI, 2.0 * CV_PI));
        }

        // 环形距离（倍角空间内 2π 与 0 是同一个方向）。
        const auto ringDist = [](double a, double b)
        {
            const double d = std::fabs(a - b);
            return std::min(d, 2.0 * CV_PI - d);
        };

        // 一维 k-means（k=2），初值取最小与最大角。对"两条棱线"这一双峰
        // 分布，极值初始化已足够，**不引入随机初值** —— 随机初值会让同一帧
        // 的两次调用给出不同结果，破坏 SYS-04 §6.4 的可复现要求。
        const auto minmax = std::minmax_element(angles.begin(), angles.end());
        double c0 = *minmax.first;
        double c1 = *minmax.second;
        if (ringDist(c0, c1) < 1e-6)
        {
            continue;
        }

        std::vector<int> label(angles.size(), 0);
        for (int iter = 0; iter < 20; ++iter)
        {
            bool changed = false;
            for (size_t i = 0; i < angles.size(); ++i)
            {
                const int lab = (ringDist(angles[i], c0)
                                 <= ringDist(angles[i], c1)) ? 0 : 1;
                if (lab != label[i])
                {
                    label[i] = lab;
                    changed = true;
                }
            }

            // 环形均值更新两个中心。
            double sumCos0 = 0.0;
            double sumSin0 = 0.0;
            double sumCos1 = 0.0;
            double sumSin1 = 0.0;
            for (size_t i = 0; i < angles.size(); ++i)
            {
                if (label[i] == 0)
                {
                    sumCos0 += std::cos(angles[i]);
                    sumSin0 += std::sin(angles[i]);
                }
                else
                {
                    sumCos1 += std::cos(angles[i]);
                    sumSin1 += std::sin(angles[i]);
                }
            }
            if (sumCos0 != 0.0 || sumSin0 != 0.0)
            {
                c0 = std::fmod(std::atan2(sumSin0, sumCos0) + 2.0 * CV_PI,
                               2.0 * CV_PI);
            }
            if (sumCos1 != 0.0 || sumSin1 != 0.0)
            {
                c1 = std::fmod(std::atan2(sumSin1, sumCos1) + 2.0 * CV_PI,
                               2.0 * CV_PI);
            }

            if (!changed)
            {
                break;
            }
        }

        std::vector<cv::Point2f> group0;
        std::vector<cv::Point2f> group1;
        for (size_t i = 0; i < edgePts.size(); ++i)
        {
            (label[i] == 0 ? group0 : group1).push_back(edgePts[i]);
        }
        if (static_cast<int>(group0.size()) < kMinEdgePixelsPerLine
            || static_cast<int>(group1.size()) < kMinEdgePixelsPerLine)
        {
            continue;
        }

        // Step 3：亚像素直线拟合（cv::fitLine 给出亚像素的参数）。
        // ⚠ 用 DIST_HUBER 而非 DIST_L2：组内混入的端头/纹理像素会把 L2
        //    拟合的直线拽偏，而**交点**是输出量，直线的位置偏差会直接
        //    变成结构点位置偏差。Huber 对这些少数离群点降权。
        cv::Vec4f l0;
        cv::Vec4f l1;
        cv::fitLine(group0, l0, cv::DIST_HUBER, 0, 0.01, 0.01);
        cv::fitLine(group1, l1, cv::DIST_HUBER, 0, 0.01, 0.01);

        Line2D a;
        a.dir = cv::Point2f(l0[0], l0[1]);
        a.point = cv::Point2f(l0[2], l0[3]);
        a.valid = true;
        Line2D b;
        b.dir = cv::Point2f(l1[0], l1[1]);
        b.point = cv::Point2f(l1[2], l1[3]);
        b.valid = true;

        // 夹角判据：两条棱线的方向差（取锐角）。
        const double dot = std::fabs(static_cast<double>(a.dir.x) * b.dir.x
                                     + static_cast<double>(a.dir.y) * b.dir.y);
        const double n0 = cv::norm(a.dir);
        const double n1 = cv::norm(b.dir);
        if (!(n0 > 0.0) || !(n1 > 0.0))
        {
            continue;
        }
        const double cosTheta = std::min(1.0, dot / (n0 * n1));
        const double angleDeg =
            std::acos(cosTheta) * 180.0 / CV_PI;
        if (angleDeg < kMinCrossingAngleDeg)
        {
            continue;
        }

        cv::Point2f corner;
        if (!intersect(a, b, corner))
        {
            continue;
        }
        // 交点必须落在窗口内：否则两条棱线是在窗口里"擦过"的两个不同结构，
        // 其外推交点没有物理意义。
        if (cv::norm(corner - p) > radius)
        {
            continue;
        }

        // 残差：两组像素到各自直线距离的中位数中的较大者（保守）。
        // 取中位数而非 RMS 的理由见 medianAbsDistance 的说明。
        const double residual =
            std::max(medianAbsDistance(group0, a), medianAbsDistance(group1, b));

        ++out.matchedCount;
        out.maxResidualPx = std::max(out.maxResidualPx, residual);

        if (residual > kMaxResidualPx)
        {
            ++out.rejectedByResidual;
            continue;
        }

        data::FeatureCorrespondence corr;
        corr.objectPoint = objectPoints[visible[k]];
        corr.imagePoint = corner;
        out.correspondences.push_back(corr);
    }

    // ---- ENG-10 §2.4 的三个判据 ----
    out.spreadPx = spreadWidth(out.correspondences);
    out.available =
        static_cast<int>(out.correspondences.size()) >= kMinCadCorrespondences;

    if (detectionLongSidePx > 0.0)
    {
        // 判据是"W ≥ 0.6 × 检测框长边"。展布不足时**不剔除对应**，
        // 只标记：ENG-10 §2.4 对这一条的要求是"记录实际 W；不足时在结果中
        // 给出'展布不足'警告"，而不是丢弃结构点。
        out.spreadSufficient =
            out.spreadPx >= kSpreadRatio * detectionLongSidePx;
    }

    return out.available;
}

}  // namespace algorithm
}  // namespace aircraft
