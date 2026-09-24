#pragma once

// ============================================================================
//  src/algorithm/pose/PnPPoseEstimator.h
//
//  依据：ENG-01 §10（pose 子模块，类名 `PnPPoseEstimator` 被冻结）、
//        SYS-07 §10（PnP 解算，三个焦段均须支持）、§12.2（三项验证指标）、
//        SYS-14 §17（不得同时运行三个完整 PnP）、
//        ENG-02 §11.7、ENG-10 §5.1（配置注入矩阵）、
//        ENG-09 §5.23（CameraPose）、§2.1（变换方向）、裁决 C-06
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ RANSAC 在本类，而不是在 FeatureMatcher（与 SYS-07 §9 的流程图不同）：  │
//  │ PnP 的几何模型本身就是"拟合模型"，残差 = 重投影误差，只有算出位姿才    │
//  │ 有意义。放在匹配阶段等于在匹配里跑一次 PnP，既违反 §6.2"选择必须在   │
//  │ PnP 之前"，也违反 SYS-14 §17。故：                                    │
//  │   匹配 → 候选对应集（含外点）；本类 → RANSAC 求位姿 + 内点集。         │
//  │ SYS-07 §12.2 的第二项验证指标（内点比例）因此由本类产出。             │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与冻结类型的偏离（内点统计量的落点）：                                │
//  │ ENG-09 §5.23 冻结的 `CameraPose` 只有 `aircraftToCamera` 与           │
//  │ `reprojectionError` 两个字段，**没有内点数/内点比例**。而              │
//  │ `PoseValidationResult`（§5.25）要求 `inlierRatio`，SYS-07 §12.2 也把 │
//  │ 它列为必检指标 —— 于是这两个量在冻结类型里无处安放。                  │
//  │ 处理方式：不改冻结类型，而是在返回值之外增加一个**并列的输出结构**     │
//  │ `PnpStats`。这样 CameraPose 仍是"单次 PnP 的姿态结果"（其原意），     │
//  │ 而统计量作为"本次解算的过程信息"单独传递，由 PosePipeline 缓存并转交  │
//  │ PoseValidator。已记入 README §6 的偏离登记。                          │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 变换方向（ENG-09 §5.23 文件头详解，此处再强调一次因为写反不会报错）：
//    `cv::solvePnPRansac` 的输入是"飞机坐标系的三维点"与"其像素坐标"，
//    输出 (rvec, tvec) 满足  p_camera = R·p_aircraft + t，
//    因此它**天然就是 aircraftToCamera**（C-06 的结论）。取逆会把 Yaw 的
//    符号与量值同时算错，且没有任何错误提示。
// ============================================================================

#include <vector>

#include "data/CameraCalibration.h"
#include "data/CameraPose.h"
#include "data/FeatureCorrespondence.h"
#include "data/MeasurementConfig.h"

namespace aircraft
{
namespace algorithm
{

/// 一次 PnP 解算的过程统计量。
///
/// 存在的理由：`CameraPose`（ENG-09 §5.23）与 `ShipPoseResult`（§5.24）都是
/// 冻结类型且都不含内点信息，而 SYS-07 §12.2 与 ENG-09 §5.25 都需要它。
struct PnpStats
{
    /// 参与解算的对应数（RANSAC 的输入规模）。
    int inputCount = 0;

    /// RANSAC 判定的内点数。
    int inlierCount = 0;

    /// 内点比例 = inlierCount / inputCount，[0,1]。
    /// 判据：≥ `ValidationConfig::minInlierRatio`（SYS-07 §12.2）。
    double inlierRatio = 0.0;

    /// 只用内点重解后的重投影误差，单位 pixel。
    /// 与 `CameraPose::reprojectionError` 是同一个数，此处重复是为了让
    /// "只看统计量"的调用方（测试、日志）不必再拆一个类型。
    double reprojectionError = 0.0;

    /// 是否收敛出位姿。
    bool converged = false;
};

/// PnP 姿态解算抽象（SYS-07 §10）。实现方向：`cv::solvePnPRansac`。
class PnPPoseEstimator
{
public:
    virtual ~PnPPoseEstimator() = default;

    /// @param correspondences 2D-3D 对应（`FeatureMatcher` 的输出，含外点）。
    /// @param calibration     该通道标定（内参 + 畸变）。**由调用方注入**。
    /// @param out             解算结果（`aircraftToCamera` + 重投影误差）。
    /// @param stats           过程统计量（内点数/内点比例）。
    /// @return 是否收敛出位姿。点不足、退化构型或 RANSAC 未能找到一致集时
    ///         返回 false，且 out/stats 保持"未收敛"状态。
    virtual bool estimate(
        const std::vector<data::FeatureCorrespondence>& correspondences,
        const data::CameraCalibration& calibration,
        data::CameraPose& out,
        PnpStats& stats) = 0;
};

/// 真实解算器：`cv::solvePnPRansac` + 内点集重解 refine。
///
/// 配置注入（ENG-10 §5.1 冻结的注入行）：
///   `MeasurementConfig → PnPPoseEstimator`
/// ⚠ §5.1 列了这一行但**未指明用哪个字段**。本实现用 `minFeatureCount`
///   作为"最少对应数"门槛 —— 它是配置里唯一描述"特征数量下限"的字段。
///   该值不可能低于 4（PnP 的几何最小点数），故实现取
///   `max(minFeatureCount, 4)`：配置若被填成 0（002 阶段的"未配置"哨兵），
///   不会让 PnP 去解一个欠定的方程组。
class CvPnPPoseEstimator : public PnPPoseEstimator
{
public:
    explicit CvPnPPoseEstimator(const data::MeasurementConfig& config);

    bool estimate(
        const std::vector<data::FeatureCorrespondence>& correspondences,
        const data::CameraCalibration& calibration,
        data::CameraPose& out,
        PnpStats& stats) override;

    /// RANSAC 重投影误差阈值，单位 pixel，默认 3.0。
    /// ⚠ 该阈值不在冻结文档中（冻结的是 `ValidationConfig::
    ///   maxReprojectionError`，那是**判定**门槛，不是 RANSAC 的**分类**
    ///   门槛，两者取同一个值会让"刚好在门槛上的点"既算内点又判不通过）。
    ///   已记入 README §6 的待裁决清单。
    void setRansacReprojectionThreshold(double px);

    /// RANSAC 最大迭代次数，默认 1000。
    void setRansacIterations(int iterations);

    /// RANSAC 置信度，默认 0.99。
    void setRansacConfidence(double confidence);

private:
    /// ⚠ 按值持有（入参仍是 `const&`），理由同 MeasurementSelector.h：
    /// 存引用时传临时量的构造可编译但悬空，此处表现为 `minFeatureCount`
    /// 门槛随机生效/失效。见 ENG-10 §5.2 约束 2。
    data::MeasurementConfig config_;
    double ransacReprojectionThreshold_ = 3.0;   // pixel
    int ransacIterations_ = 1000;
    double ransacConfidence_ = 0.99;
};

/// 固定输出（8.md §十一 的 Mock）。用于状态机与验证逻辑的单测：
/// 使"PnP 失败应重试 / 结果超出 Yaw 范围应判不合格"之类的断言
/// 不依赖真实数据与求解器数值行为。
class MockPnPPoseEstimator : public PnPPoseEstimator
{
public:
    MockPnPPoseEstimator() = default;

    /// 预设一次成功的解算。
    /// @param pose      输出姿态（`aircraftToCamera`）。
    /// @param inlierRatio 内点比例，将按 `inlierCount = round(ratio·n)`
    ///        与调用时的对应数一起换算，使统计量之间保持自洽。
    void setPreset(const data::CameraPose& pose, double inlierRatio);

    /// 强制 `estimate()` 返回 false。
    void forceFailure(bool fail) { forceFailure_ = fail; }

    bool estimate(
        const std::vector<data::FeatureCorrespondence>& correspondences,
        const data::CameraCalibration& calibration,
        data::CameraPose& out,
        PnpStats& stats) override;

private:
    data::CameraPose preset_;
    double inlierRatio_ = 0.9;
    bool forceFailure_ = false;
    bool hasPreset_ = false;
};

}  // namespace algorithm
}  // namespace aircraft
