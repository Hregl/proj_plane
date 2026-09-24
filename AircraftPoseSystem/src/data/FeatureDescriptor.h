#pragma once

// ============================================================================
//  src/data/FeatureDescriptor.h
//
//  依据：ENG-09 §4.1、§5.19（类型定义冻结）、裁决 C-08
//
//  裁决 C-08：SYS-05 §10.1 与 SYS-12 §5.2/§9 对同一概念给出两套互斥建模：
//      SYS-05 §10.1 | points3d: vector<cv::Point3f> + descriptors: cv::Mat（扁平，无关联）
//      SYS-12 §5.2  | ModelPoint3D 结构 + FeatureDescriptor 结构（显式关联）
//  **以 SYS-12 为准。**
//
//  理由（不采用扁平方案的原因）：SYS-05 的扁平方案丢失了"哪个描述子对应
//  哪个三维点"的关联，而**该关联正是 PnP 2D-3D 对应的唯一依据**。
//  扁平 descriptors 矩阵的第 i 行没有指明其三维坐标，PnP 无从建立方程，
//  整个算法链在此断裂。
//
//  冻结为 FeatureDescriptor.point3dIndex + TargetModel.points3d 的**索引**
//  关联，同时消除"FeatureDescriptor 内嵌 Point3D 副本"与
//  "ModelPoint3D.position"的重复存储 —— 同一三维点存两份时，若只更新了
//  其中一份（例如结构点坐标经标定修正），PnP 会用到过期的那一份，
//  且不会报错。
// ============================================================================

#include <opencv2/core.hpp>

namespace aircraft
{
namespace data
{

/// 一个模型特征描述子，通过索引关联到 TargetModel::points3d。
struct FeatureDescriptor
{
    /// 描述子标识，在同一 TargetModel 内唯一。
    int featureId = 0;

    /// 索引 TargetModel::points3d 中对应的三维点。
    /// **这是 PnP 建立 2D-3D 对应的唯一依据**（裁决 C-08）。
    int point3dIndex = 0;

    /// 描述子向量，1 × D，CV_32F。
    cv::Mat descriptor;
};

}  // namespace data
}  // namespace aircraft
