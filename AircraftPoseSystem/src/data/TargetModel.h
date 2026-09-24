#pragma once

// ============================================================================
//  src/data/TargetModel.h
//
//  依据：ENG-09 §4.1、§5.20（类型定义冻结）、裁决 C-08、C-09
//        SYS-12（目标模型与特征库）、ENG-10 §6（CAD 模型输入要求）
//
//  作用：一台确定外形状态的飞机的三维模型 + 特征库。
//
//  ⚠ 按 model_id 区分**外形状态**（ENG-10 §6）：
//  同一机型的不同挂载/起落架状态必须使用不同的 model_id。若换了状态却沿用
//  旧 model_id，结构点坐标与实物不符，B 类特征（几何稳定的杠杆臂端点）
//  会引入**系统偏差**，直接进入 Yaw —— 而系统偏差在 SYS-15 §4 的误差合成
//  规则中**线性相加**（不像随机误差那样 RSS 相消），因此代价比看起来大。
//
//  ⚠ 三焦段特征库物理分离（ENG-09 §7.2）：
//  离线产物为 feature25.bin / feature50.bin / feature100.bin（ENG-01 §3.3），
//  TargetModelManager 按 CameraRole 直接索引，**无需** FeatureDatabase
//  这类额外容器类型 —— 该类型已被裁决 C-09 删除。
// ============================================================================

#include <string>
#include <vector>

#include "data/FeatureDescriptor.h"
#include "data/ModelPoint3D.h"

namespace aircraft
{
namespace data
{

/// 目标模型（SYS-12 §5.2）。
struct TargetModel
{
    /// 模型标识。对应 ENG-09 §6.1 的机型号 + 外形状态，也是
    /// runtime/match_stats.yaml 的分组键（ENG-10 §4.1：换机型必须换表）。
    std::string modelId;

    /// 三维点表。B 类（CAD 结构点，featureType="cad"）与纹理辅助点
    /// （featureType="texture"）共存于此表。
    /// 索引由 FeatureDescriptor::point3dIndex 引用（裁决 C-08）。
    std::vector<ModelPoint3D> points3d;

    /// 特征描述子表，仅服务 A 类（自然纹理）匹配。
    /// B 类的离线产物是 points3d 中的坐标表，无需描述子（ENG-10 §2.6）。
    std::vector<FeatureDescriptor> features;
};

}  // namespace data
}  // namespace aircraft
