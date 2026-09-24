#pragma once

// ============================================================================
//  tests/algorithm/TestSupport.h
//
//  依据：ENG-06 §8（algorithm 层测试 = 算法链各阶段的判据与数值）、
//        ENG-10 §5.2 约束 4（测试必须走与生产相同的注入路径）
//
//  为什么把夹具集中到一处：algorithm 层的两个测试文件（AlgorithmStageTest
//  校验**各阶段**、PipelineTest 校验**阶段之间的装配**）必须对"已配置"这个
//  前提持同一份定义。若各写一份，门槛值一旦不同，两个文件就会对同一份实现
//  给出互相矛盾的结论，而**两边都自洽**，看不出哪个是错的。
//
//  ⚠ 本文件里的量分三类，改动前必须分清属于哪一类：
//    1. **物理常量**（kFx25mmPx）：由像素尺寸与焦距导出，不可"调"。
//       例如 0.025 / 3.45e-6 ≈ 7246.4，SYS-14 §10 的换算要求 f 取像素。
//    2. **手算期望值的前提**（configuredConfig 的 1/1/1/1 权重、
//       minTargetPixelSize = 50 等）：这些值本身不是建议的整定结果，
//       只是"让用例里的期望值可以手算"的一组取值。
//    3. **同一份冻结判据的复述**（configuredValidation 的 ±30° 等）。
//       它们的来源是文档；改了它们等于改了被测的判据，用例会随之失效。
// ============================================================================

#include <cmath>
#include <cstdio>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "data/CameraCalibration.h"
#include "data/CameraRole.h"
#include "data/DetectionResult.h"
#include "data/FeatureCorrespondence.h"
#include "data/ImageFrame.h"
#include "data/MeasurementConfig.h"
#include "data/ValidationConfig.h"

namespace aps_test
{

using aircraft::data::CameraCalibration;
using aircraft::data::CameraRole;
using aircraft::data::DetectionResult;
using aircraft::data::FeatureCorrespondence;
using aircraft::data::ImageFrame;
using aircraft::data::MeasurementConfig;
using aircraft::data::ValidationConfig;

/// 25 mm 焦段的内参水平分量：像素尺寸 3.45 µm（华睿 A7A20MU201）。
///   fx = 0.025 / 3.45e-6 ≈ 7246.4 pixel
/// SYS-14 §10 的 Z = f·L/(l·s) 要求 f **必须以像素为单位**；
/// 用 25（毫米）代入会让距离差 290 倍且不报错（见 AlgorithmStageTest 的
/// "距离差 290 倍"用例）。
inline constexpr double kFx25mmPx = 0.025 / 3.45e-6;

/// 构造一份**已填写**的标定：内参 + 图像尺寸（缺尺寸会在算法层被拒，
/// 见 PinholeScaleEstimator / PnPPoseEstimator 的 imageWidth 判据）。
inline CameraCalibration calibrationWithFx(double fx, double fy,
                                           int width = 2448, int height = 2048)
{
    CameraCalibration c;
    c.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    c.cameraMatrix.at<double>(0, 0) = fx;
    c.cameraMatrix.at<double>(1, 1) = fy;
    c.cameraMatrix.at<double>(0, 2) = width / 2.0;
    c.cameraMatrix.at<double>(1, 2) = height / 2.0;
    c.imageWidth = width;
    c.imageHeight = height;
    return c;
}

/// 一个"已配置"的测量配置：门槛与权重都显式给值。
///
/// ⚠ 权重在 ENG-09 §6.5 中**没有默认值**（默认 0 = 未配置），故测试必须
///   自己给。这里取 1/1/1/1 只是一组便于手算的值，不是建议的整定结果。
inline MeasurementConfig configuredConfig()
{
    MeasurementConfig c;
    c.w1 = 1.0;
    c.w2 = 1.0;
    c.w3 = 1.0;
    c.w4 = 1.0;
    c.minSharpness = 10.0;
    c.minTargetPixelSize = 50.0;
    c.minFeatureCount = 50;
    c.minMatchRatio = 0.5;
    c.nRef = 100.0;
    return c;
}

inline ValidationConfig configuredValidation()
{
    ValidationConfig v;
    v.maxReprojectionError = 2.0;   // pixel
    v.minInlierRatio = 0.6;
    v.minConfidence = 0.5;
    v.yawMin = -30.0;              // deg
    v.yawMax = 30.0;
    return v;
}

/// 一架飞机的**非共面**三维标志点（单位 m，ENG-09 §2.3），
/// 覆盖机头/机尾/两翼尖/垂尾顶 —— 即 ENG-10 §2.2 要求的展布。
/// 全长 10 m（机头 +5 到尾 −5），故包围盒最长边 = 10 m = 目标真实尺寸 L。
///
/// ⚠ 点必须**非共面**：共面点集会让 PnP 解二义（ENG-10 §6 对此有要求），
/// 于是"PnP 恢复不出真值"会被误诊为实现问题。
inline std::vector<cv::Point3f> aircraftShapePoints()
{
    std::vector<cv::Point3f> pts;
    pts.emplace_back(5.0f, 0.0f, 0.0f);       // 机头
    pts.emplace_back(-5.0f, 0.0f, 0.0f);      // 机尾
    pts.emplace_back(0.0f, 6.0f, 0.0f);       // 右翼尖
    pts.emplace_back(0.0f, -6.0f, 0.0f);      // 左翼尖
    pts.emplace_back(-4.5f, 0.0f, 2.0f);      // 垂尾顶
    pts.emplace_back(1.0f, 1.0f, 0.5f);
    pts.emplace_back(2.0f, -1.5f, 0.4f);
    pts.emplace_back(-2.0f, 2.5f, 0.3f);
    pts.emplace_back(-3.0f, -2.5f, 0.6f);
    pts.emplace_back(3.5f, 2.0f, -0.4f);
    pts.emplace_back(0.5f, -3.0f, -0.6f);
    pts.emplace_back(-1.0f, 3.5f, 0.8f);
    return pts;
}

/// 用给定外参把三维点投到图像，作为 2D-3D 对应（即 PnP 的输入）。
inline std::vector<FeatureCorrespondence> project(
    const std::vector<cv::Point3f>& points,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const CameraCalibration& calib)
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(points, rvec, tvec, calib.cameraMatrix, calib.distortion,
                      projected);

    std::vector<FeatureCorrespondence> out;
    out.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        FeatureCorrespondence c;
        c.objectPoint = points[i];
        c.imagePoint = projected[i];
        out.push_back(c);
    }
    return out;
}

/// 两个旋转之间的夹角（弧度）：angle(R_a · R_bᵀ)。
inline double rotationAngleBetween(const cv::Matx33d& a, const cv::Matx33d& b)
{
    const cv::Matx33d rel = a * b.t();
    const double trace = rel(0, 0) + rel(1, 1) + rel(2, 2);
    const double c = std::min(1.0, std::max(-1.0, (trace - 1.0) / 2.0));
    return std::acos(c);
}

/// 转成弧度（用例里手算角度时避免散落 pi 的字面量）。
inline constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
inline constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// 绕 X / Y / Z 的单轴旋转矩阵（右手系，与 cv::Rodrigues 一致）。
inline cv::Matx33d rotX(double deg)
{
    const double a = deg * kDegToRad;
    const double c = std::cos(a);
    const double s = std::sin(a);
    return cv::Matx33d(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
}

inline cv::Matx33d rotY(double deg)
{
    const double a = deg * kDegToRad;
    const double c = std::cos(a);
    const double s = std::sin(a);
    return cv::Matx33d(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
}

inline cv::Matx33d rotZ(double deg)
{
    const double a = deg * kDegToRad;
    const double c = std::cos(a);
    const double s = std::sin(a);
    return cv::Matx33d(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
}

/// 构造一帧图像（CV_8UC1）。`role` 一并写入，因为 ImageFrame::role 是
/// DetectionResult::sourceCamera 的来源（算法层不允许自行改写）。
inline ImageFrame makeFrame(const cv::Mat& image, CameraRole role,
                            long long frameId = 1)
{
    ImageFrame f;
    f.image = image;
    f.frameId = frameId;
    f.role = role;
    return f;
}

/// 暗背景上的亮矩形 —— `MockTargetDetector` 的合成场景形态
/// （阈值 + 最大连通域），长宽与位置可控。
inline cv::Mat rectangleImage(int width, int height, const cv::Rect& box,
                              unsigned char background = 20,
                              unsigned char target = 220)
{
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(background));
    const cv::Rect clipped = box & cv::Rect(0, 0, width, height);
    if (clipped.width > 0 && clipped.height > 0)
    {
        img(clipped).setTo(cv::Scalar(target));
    }
    return img;
}

/// 同样被检测为**一个**矩形连通域，但矩形**内部有纹理**的合成目标。
///
/// 为什么要它：`MockTargetDetector` 用全局阈值 (max+min)/2 取最大连通域，
/// 于是一个**填满常量**的矩形虽然能被检测到，却让 `countDetectableFeatures`
/// （goodFeaturesToTrack，窗口取检测框）返回 0 —— 常量区域上一个可用角点
/// 都没有。而 `MeasurementSelector` 的候选门槛里有 `nDetect ≥
/// minFeatureCount` 一条，于是"目标清晰可见却一个候选都没有"，
/// 表现为通道选择直接失败。用一块**平铺矩形**（棋盘格）代替常量填充即可：
/// 角点数量随格数上升，而所有目标像素仍在亮带内（> 阈值），
/// 连通域仍是一个矩形。
///
/// @param cellPx 棋盘格边长（pixel）。越小角点越多。
inline cv::Mat texturedTargetImage(int width, int height, const cv::Rect& box,
                                   int cellPx = 16,
                                   unsigned char background = 10,
                                   unsigned char dark = 180,
                                   unsigned char bright = 255)
{
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(background));
    const cv::Rect clipped = box & cv::Rect(0, 0, width, height);
    if (clipped.width > 0 && clipped.height > 0 && cellPx > 0)
    {
        for (int y = 0; y < clipped.height; ++y)
        {
            for (int x = 0; x < clipped.width; ++x)
            {
                const bool odd = ((x / cellPx) + (y / cellPx)) % 2 != 0;
                img.at<unsigned char>(clipped.y + y, clipped.x + x) =
                    odd ? bright : dark;
            }
        }
    }
    return img;
}

/// 手工构造的合成图能提供的特征数由**合成纹理的密度**决定，与整机量级
/// 无关；而 `configuredConfig()` 的 `minFeatureCount = 50` 是按整机
/// （真实飞机表面的可检测特征数）取的量级。凡是要经过
/// `minFeatureCount` 门槛的用例 —— 坐标链用例（注入固定的 12 组对应）
/// 与通道选择用例（合成图上的 nDetect）—— 都必须用本函数把该门槛压到
/// 合成图给得出的量级，否则用例测的就成了"配置值大于夹具能提供的数量"，
/// 与它真正要校验的几何/选择逻辑无关。
///
/// ⚠ 只改 `minFeatureCount` 一项，其余门槛仍是 `configuredConfig()` 的口径：
///   若连别的门槛也一起放松，用例会失去对阈值接线的判别力。
inline MeasurementConfig syntheticFixtureConfig(int minPoints = 10)
{
    MeasurementConfig c = configuredConfig();
    c.minFeatureCount = minPoints;
    return c;
}

// ---------------------------------------------------------------------------
//  临时目录（模型库的落盘 / 读回）
// ---------------------------------------------------------------------------

/// 建立一个本次进程专属的临时目录，形如
/// `/tmp/aps_test_<tag>_<pid>_<n>`。目录已存在时直接复用。
///
/// 用途：`TargetModelManager` 的完整加载路径（points3d.yaml + model.yaml +
/// 三个 featureNN.bin）**必须**在文件系统上验证一次 —— 内存里构造的
/// `TargetModel` 绕过了路径拼接、角色标签、跨文件一致性这三处最容易错的
/// 环节（ENG-06 §10 的 golden 目录只放数据，不放生成器，故夹具在此生成）。
inline std::string makeTempDir(const std::string& tag)
{
    static int counter = 0;
    ++counter;
    const std::string path = "/tmp/aps_test_" + tag + "_"
                             + std::to_string(static_cast<long>(getpid()))
                             + "_" + std::to_string(counter);
    ::mkdir(path.c_str(), 0755);
    return path;
}

/// 写文本文件。失败时返回 false（调用方必须断言，否则用例会在
/// "文件根本没写出来"的情况下继续跑，得到一个与被测代码无关的失败）。
inline bool writeTextFile(const std::string& path, const std::string& text)
{
    std::FILE* fp = std::fopen(path.c_str(), "wb");
    if (fp == nullptr)
    {
        return false;
    }
    const size_t written = std::fwrite(text.data(), 1, text.size(), fp);
    std::fclose(fp);
    return written == text.size();
}

}  // namespace aps_test
