#pragma once

// ============================================================================
//  src/data/FeatureSet.h
//
//  依据：ENG-09 §4.1、§5.21（类型定义冻结）、裁决 C-09
//
//  裁决 C-09：SYS-07 §8 声明 FeatureExtraction 输出 FeatureSet，
//  但**全套文档从未定义该类型**。本文件即冻结补定义。
//  （同批被删除的有 TrackingResult / FeatureDatabase / PoseResult；
//    被补定义的还有 ErrorInfo。）
//
//  ⚠ 本类型在 8.md §九 中的版本是 `struct FeatureSet { int count; };` ——
//  那是一个占位骨架，只有计数器没有数据，无法承载任何匹配。
//  ENG-09 §5.21 冻结为 keypoints + descriptors，这才是 SIFT 的真实输出，
//  也是 ENG-10 §2.2 中 A 类特征"保数量 N"的载体。
//
//  组合方式的选择理由（cv::Mat 而非 vector<cv::Mat>）：
//  描述子矩阵按 N × D 扁平存放，与 OpenCV 的
//  BFMatcher / FlannBasedMatcher 入参格式一致，避免每次匹配前重新组装。
//  单条描述子仍是 1 × D 的 cv::Mat 行视图（见 FeatureDescriptor）。
// ============================================================================

#include <vector>

#include <opencv2/core.hpp>

namespace aircraft
{
namespace data
{

/// 单帧图像的特征提取结果（SYS-07 §8）。
struct FeatureSet
{
    /// 关键点表，含亚像素坐标、尺度、方向。
    std::vector<cv::KeyPoint> keypoints;

    /// 描述子矩阵，N × D，CV_32F，第 i 行对应 keypoints[i]。
    cv::Mat descriptors;
};

}  // namespace data
}  // namespace aircraft
