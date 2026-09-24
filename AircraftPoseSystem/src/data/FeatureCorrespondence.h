#pragma once

// ============================================================================
//  src/data/FeatureCorrespondence.h
//
//  依据：ENG-09 §4.1、§5.22（类型定义冻结）、§2.3 / §2.4（单位）
//        ENG-10 §2.5（A/B 两类的融合规则）
//
//  作用：一组 2D-3D 对应，即 PnP 的输入。它同时承载 A 类与 B 类来源的点 ——
//  ENG-10 §2.5 冻结的融合规则在此处的体现是：
//      B 类（CAD 结构点）：全部参与
//      A 类（纹理点）    ：按 RANSAC 内点筛选后参与
//      冲突规则：同一区域两类反投影后位置差 > 2 pixel 时**以 B 类为准**
//
//  ⚠ 为什么冲突时以 B 类为准（ENG-10 §2.5 的理由，实现时必须理解）：
//  B 类的三维坐标是**精确已知的 CAD 结构点**；A 类点是通过描述子匹配
//  **间接**关联到三维模型的，多一层不确定性。当两者矛盾时，前者的可信度
//  来自几何定义，后者的可信度来自一次可能出错的匹配。
//
//  ⚠ 首版不设权重差异（ENG-10 §2.5）：仅靠 RANSAC 与冲突规则处理，
//  避免引入无依据的可调参数。若实测发现 A 类外点率过高，再按 ENG-09 §8
//  的变更流程引入权重。
//
//  单位：objectPoint 为 **m**（Aircraft Frame），imagePoint 为 **pixel**。
// ============================================================================

#include <opencv2/core.hpp>

namespace aircraft
{
namespace data
{

/// 一组 2D-3D 对应（PnP 输入）。
struct FeatureCorrespondence
{
    /// 三维点，单位 **m**，Aircraft Frame。
    cv::Point3f objectPoint;

    /// 二维点，单位 pixel，图像坐标系（左上原点，X 右 Y 下）。
    cv::Point2f imagePoint;
};

}  // namespace data
}  // namespace aircraft
