#pragma once

// ============================================================================
//  src/data/ModelPoint3D.h
//
//  依据：ENG-09 §4.1、§5.18（类型定义冻结）、§2.3（长度单位）
//        ENG-10 §2.2（B 类：CAD 结构点）、§6（CAD 模型输入要求）
//
//  作用：目标模型上的一个三维点。它就是 ENG-10 §2.2 所说的 **B 类特征**：
//  几何稳定、分布可控、正是杠杆臂端点（翼尖、机头、尾翼、进气口唇口）。
//
//  为什么 featureType 是 std::string 而不是枚举：
//  ENG-09 §5.18 冻结为字符串，取值为 "cad" | "texture"。这不是疏忽 ——
//  特征来源的类别集合在标定阶段（ENG-10 §2.3）与离线建模阶段
//  （SYS-12 §12）由不同工具产出，且未来可能增加第三类（如"人工标注点"）。
//  若设为枚举，每增加一类都要走 ENG-09 §8 的变更流程改冻结表；
//  字符串在此处的可扩展性收益大于类型安全的收益。
//  代价是取值必须靠约定约束 —— 故本注释即为该约定的唯一记录处。
//
//  单位：position 单位为 **m**（ENG-09 §2.3），坐标系为 Aircraft Frame
//  （SYS-13）。两者都必须与 CAD 模型一致，否则 ENG-10 §6 的"坐标系已统一"
//  要求不成立，三维点与图像点的对应会整体错误。
// ============================================================================

#include <string>

#include <opencv2/core.hpp>

namespace aircraft
{
namespace data
{

/// 目标模型上的一个三维点（ENG-09 §5.18）。
struct ModelPoint3D
{
    /// 点标识，在同一 TargetModel 内唯一。
    int id = 0;

    /// 位置，单位 **m**，Aircraft Frame。
    cv::Point3f position;

    /// 特征来源类型（约定取值，见文件头说明）："cad" | "texture"。
    /// "cad" 类点由 ENG-10 §2.3 的投影—匹配—拟合三步法在图像中定位，
    /// 精度可达 0.1~0.2 pixel；"texture" 类点由 SIFT 匹配间接关联。
    std::string featureType;
};

}  // namespace data
}  // namespace aircraft
