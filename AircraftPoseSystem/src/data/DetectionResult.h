#pragma once

// ============================================================================
//  src/data/DetectionResult.h
//
//  依据：ENG-09 §4.1、§5.12（类型定义冻结）、§2.4（像素坐标）
//
//  ⚠ 与工作流文档的偏离（8.md §四 定义的 DetectionResult 用的是
//      int x/y/width/height + double confidence）：
//  ENG-09 §5.12 冻结为 `cv::Rect bbox` + `CameraRole sourceCamera`，
//  且**没有**独立的 x/y/width/height 字段。理由：
//  1. cv::Rect 直接可用于 OpenCV 的 ROI、距离变换、mask 构造等操作，
//     而四个散装 int 每次使用都要重新组装成 Rect；
//  2. bbox 保持 OpenCV 约定（左上角原点，X 右 Y 下，单位像素，§2.4）；
//  3. sourceCamera 是必需的：三焦段候选打分时，每个候选的检测结果必须
//     自证来自哪台相机，否则 MeasurementSelector 的候选集合无法建立
//     （SYS-14 §7 的三焦段并行检测正是如此）。
//
//  ⚠ bbox 在 det 失败时无意义：以 `found` 为准，不要读 bbox。
//  ENG-09 §7.1 已删除 TrackingResult，并明确"DetectionResult 已完整覆盖
//  对准所需信息（目标框中心 → 像素偏差）"—— V2.1 是单次测量架构，
//  不存在跨帧跟踪目标。
// ============================================================================

#include <opencv2/core.hpp>

#include "data/CameraRole.h"

namespace aircraft
{
namespace data
{

/// 单帧目标检测结果（SYS-07 §4）。
struct DetectionResult
{
    /// 是否检出目标。false 时其余字段均无意义。
    bool found = false;

    /// 目标包围框。左上角原点，X 右 Y 下，单位 pixel（ENG-09 §2.4）。
    /// 后续用途：TargetOffset 由 bbox 中心与图像中心求差得到；
    /// ENG-10 §2.4 的展布宽度判据 W ≥ 0.6 × 检测框长边也以本框为参照。
    cv::Rect bbox;

    /// 检测置信度，[0,1]。
    double confidence = 0.0;

    /// 产出该结果的相机角色。三焦段候选打分必需（SYS-14 §7）。
    CameraRole sourceCamera = CameraRole::CAM25;
};

}  // namespace data
}  // namespace aircraft
