#pragma once

// ============================================================================
//  src/algorithm/feature/FeatureExtractor.h
//
//  依据：SYS-07 §8（FeatureExtraction：初期用 SIFT）、
//        ENG-01 §10（feature 子模块：`FeatureExtractor.h` = **A 类**：
//                    SIFT 关键点 + 描述子；`CadStructureLocator.h` = B 类）、
//        ENG-10 §2.2（A/B 两类特征的分工）、§2.6（SYS-07 §8.2 的"初期 SIFT"
//                    冻结为"A 类用 SIFT，B 类用边缘拟合法"）、
//        ENG-02 §11.5、ENG-09 §5.21（`data::FeatureSet` 冻结定义）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与 8.md §九 的偏离：8.md 的 `struct FeatureSet { int count = 0; };`   │
//  │ 是**只有计数器、没有数据**的占位结构体，无法承载任何匹配 ——           │
//  │ 一个 int 无法参与描述子匹配。ENG-09 §5.21 已冻结为                  │
//  │ `data::FeatureSet { std::vector<cv::KeyPoint> keypoints;             │
//  │                     cv::Mat descriptors; }`，本类使用冻结类型。       │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ **本类只负责 A 类（自然纹理），B 类（CAD 结构点）在 CadStructureLocator。**
//  两者不是同一件事的实现细节差异：
//    · FeatureExtractor 找**纹理不连续**（尺度空间极值）；
//    · CadStructureLocator 找**几何不连续**（棱线交点）。
//  ENG-10 §2.2 的分工正是为此：A 类保证点数 N，B 类保证展布 W。
//  用 SIFT 代替 B 类会失去展布保证 —— SIFT 关键点与真实几何角点之间存在
//  随视角变化的偏移，不满足 0.3 pixel 的定位要求（SYS-07 §8.2 的裁决 G-1）。
// ============================================================================

#include "data/FeatureSet.h"
#include "data/ImageFrame.h"

namespace aircraft
{
namespace algorithm
{

/// 特征提取抽象（SYS-07 §8）。实现方向：本类 → `SiftFeatureExtractor`。
class FeatureExtractor
{
public:
    virtual ~FeatureExtractor() = default;

    /// @param frame 待提取图像（MEASURE_SELECT / POSE_SOLVE 阶段为被选通道的图）。
    /// @param out   `data::FeatureSet`（keypoints + descriptors，N×D CV_32F）。
    /// @return 是否提取到**可用于匹配**的特征（点数 > 0 且描述子非空）。
    ///         图像为空、类型不符、点数不足门槛时返回 false。
    virtual bool extract(const data::ImageFrame& frame,
                         data::FeatureSet& out) = 0;
};

/// SIFT 特征提取（SYS-07 §8.2 的"初期：SIFT"）。
///
/// 选 SIFT 的冻结理由（SYS-07 §8.2）：尺度鲁棒、旋转鲁棒、适合多焦段。
/// 多焦段这一点在本系统里是硬需求 —— 同一个目标在 25/50/100 mm 三个通道
/// 中的图像尺度差异可达 4 倍（100/25），尺度不变的描述子是同一套匹配
/// 逻辑能跨通道复用的前提。
///
/// ⚠ OpenCV 4.4 起 SIFT 已从 `opencv_contrib` 移入 `features2d` 主模块
/// （本工程使用 4.6.0，已验证 `cv::SIFT::create()` 可用），
/// 因此**不需要**额外的 contrib 依赖 —— 这一点必须在离线交付时与
/// 部署文档对齐，否则会误装整个 contrib。
///
/// 提取参数（`nFeatures` / `contrastThreshold` / `edgeThreshold`）
/// **不在冻结配置内**：ENG-09 §6.5 的 `MeasurementConfig` 只有
/// `minFeatureCount`（点数门槛）与 `nRef`（归一化参考），没有 SIFT 自身的
/// 参数。故本类使用 OpenCV 默认值，并把 `minFeatureCount` 作为输出门槛。
/// 已记入 README §6 的待裁决清单（是否需要把 SIFT 参数纳入配置）。
class SiftFeatureExtractor : public FeatureExtractor
{
public:
    /// @param minFeatureCount 点数门槛，来自 `MeasurementConfig`。
    ///        不足即返回 false —— 让"特征太少、无法解算"在提取阶段就暴露，
    ///        而不是等到匹配/PnP 阶段退化成一个更含糊的失败。
    explicit SiftFeatureExtractor(int minFeatureCount);

    bool extract(const data::ImageFrame& frame,
                 data::FeatureSet& out) override;

private:
    int minFeatureCount_;
};

/// 固定特征集提取（8.md §九 的 `MockFeatureExtractor`）。
///
/// 返回调用方预设的特征集，用于"匹配/PnP 之外"的逻辑单测：
/// 例如验证 `MeasurementSelector` 的 F 分项随 nDetect 变化，
/// 或验证 `FeatureMatcher` 对固定输入的融合规则。
///
/// ⚠ 预设的 `keypoints` 与 `descriptors` 行数必须一致 —— 不一致时
/// `extract` 返回 false，而不是把不一致的数据交给下游
/// （第 i 行描述子对应 keypoints[i] 是 ENG-09 §5.21 的隐含契约，
/// 破坏它会让匹配结果整体错位且无法察觉）。
class MockFeatureExtractor : public FeatureExtractor
{
public:
    /// 默认构造：空特征集（`extract` 返回 false）。
    MockFeatureExtractor() = default;

    explicit MockFeatureExtractor(const data::FeatureSet& preset);

    /// 设置预设特征集，并校验 keypoints / descriptors 的一致性。
    /// @return 是否通过校验（不一致时返回 false 且预设**不被采用**）。
    bool setPreset(const data::FeatureSet& preset);

    bool extract(const data::ImageFrame& frame,
                 data::FeatureSet& out) override;

private:
    data::FeatureSet preset_;
    bool hasPreset_ = false;
};

}  // namespace algorithm
}  // namespace aircraft
