#pragma once

// ============================================================================
//  src/algorithm/pipeline/PosePipeline.h
//
//  依据：ENG-02 §12（算法流水线与 007 的文件清单）、ENG-10 §2~§5
//        （A/B 两类特征、评分与配置注入）、SYS-04 §4.2（IF-SW-02）、
//        SYS-07 §15（实时预算 < 100 ms）、SYS-08 §5（各状态的输出）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类是 IF-SW-02 的**唯一实现方**，也是算法层的组合根。               │
//  │                                                                      │
//  │ 组合方式：构造时把各阶段对象与配置注入（ENG-10 §5.1 的注入矩阵），     │
//  │ 之后**不可变**（§5.2 约束 2）。Pipeline 自身不读配置管理器 ——         │
//  │ §5.2 约束 1：算法层不得访问 ConfigManager。                           │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 坐标链的落点（ENG-01 §17 与 IF-SW-02 的一处结构性冲突）：             │
//  │                                                                      │
//  │ IF-SW-02 冻结的 `solvePose` 返回 `ShipPoseResult`（含               │
//  │ `aircraftToShip`），但冻结的入参里只有 `CameraCalibration`            │
//  │ （带 `cameraToRig`），**不带 `rigToShip`** —— 后者在                 │
//  │ `OpticalRigCalibration` 里。即"入参凑不出返回类型"。                  │
//  │                                                                      │
//  │ 而 ENG-01 §17 把 optical 放在 algorithm **之上**                   │
//  │ （app → ui → application → preview/optical → device/algorithm），     │
//  │ 所以算法层**不能**调 `optical::CoordinateTransformer` 来补这一步。     │
//  │                                                                      │
//  │ 处理：把整份 `OpticalRigCalibration`（含三相机内参与 `rigToShip`）      │
//  │ 作为**构造注入**的一部分（应用层持有标定，由它提供），Pipeline 内部    │
//  │ 按 ENG-09 §2.1 的链方向自行合成：                                    │
//  │     aircraftToShip = rigToShip · cameraToRig · aircraftToCamera        │
//  │ 注入整份标定而不是只注入 `rigToShip`，是因为 `estimateScale`           │
//  │ 与 `selectCamera` 同样需要内参（SYS-14 §10 的 Z = f·L/l 里 f 就是      │
//  │ `cameraMatrix` 的 fx），而这两个方法的冻结签名里也没有标定入参。       │
//  │ 代价是欧拉角分解（ZYX）在本文件内有一份实现，与                     │
//  │ `optical::CoordinateTransformer` 重复约 30 行。重复是因为分层禁止     │
//  │ 复用，不是疏忽。已记入 README §6 的待裁决清单：                       │
//  │ 是应把标定并入冻结入参，还是允许算法层依赖 optical。                  │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 粗姿态的 180° 二义性（本类已知的、无法在算法层消除的风险）：
//    §2.3 的粗姿态来自转台角度 + 距离，而"飞机机头朝向"**不能**由这两个
//    量确定 —— 迎头与尾追的转台角度完全相同。本实现假定**进近（迎头）**，
//    即机头大致指向相机。若实际是尾追，粗姿态会差 180°，CAD 结构点会投影
//    到机身的错误一侧，B 类定位失败（表现为 `cad_assisted = false`，
//    退化为纯 A 类 —— 不会产生错误结果，但会丢掉 B 类带来的精度）。
//    消除途径是引入航迹方向或外部引导信息，不在本层能力范围内。
//    已记入 README §6 的待裁决清单。
// ============================================================================

#include <memory>
#include <vector>

#include "algorithm/detection/TargetDetector.h"
#include "algorithm/feature/CadStructureLocator.h"
#include "algorithm/feature/FeatureExtractor.h"
#include "algorithm/matcher/FeatureMatcher.h"
#include "algorithm/model/TargetModelManager.h"
#include "algorithm/pipeline/IPosePipeline.h"
#include "algorithm/pose/PnPPoseEstimator.h"
#include "algorithm/scale/TargetScaleEstimator.h"
#include "algorithm/selection/MeasurementSelector.h"
#include "algorithm/validation/PoseValidator.h"

#include "data/CameraRole.h"
#include "data/MeasurementConfig.h"
#include "data/OpticalRigCalibration.h"
#include "data/ValidationConfig.h"

namespace aircraft
{
namespace data
{
class IMatchStatsStore;
}

namespace algorithm
{

/// 算法流水线（IF-SW-02 的实现）。
class PosePipeline : public IPosePipeline
{
public:
    /// 各阶段对象的注入集合。
    ///
    /// 生产路径不需要它（见下面的便捷构造，Pipeline 自建真实实现）；
    /// 它存在的意义是单测能只替换**一个**阶段 —— 例如注入
    /// `MockPnPPoseEstimator` 让"PnP 失败时 solvePose 返回 false"这一断言
    /// 不依赖 SIFT 与 RANSAC 的数值行为（ENG-10 §5.2 约束 4：
    /// 测试必须通过与生产相同的注入路径）。
    ///
    /// 指针**非拥有**：对象由调用方保证在 Pipeline 生命周期内有效。
    struct Stages
    {
        TargetDetector*       detector  = nullptr;
        TargetScaleEstimator* scale     = nullptr;
        FeatureExtractor*     extractor = nullptr;
        CadStructureLocator*  cad       = nullptr;
        FeatureMatcher*       matcher   = nullptr;
        PnPPoseEstimator*     pnp       = nullptr;
    };

    /// 便捷构造：自建全真实链（检测 → 尺度 → SIFT/CAD → 匹配 → PnP → 验证）。
    ///
    /// @param measurement 测量配置（含评分权重 w1~w4、各门槛、σ_px 系数）。
    /// @param validation  验证阈值。
    /// @param models      目标模型管理器（**已 loadModel**；未加载时
    ///        `solvePose` 返回 false，因为没有任何三维点可匹配）。
    /// @param rig         光机刚体标定（ENG-09 §5.3）：三相机内参 +
    ///        cameraToRig + rigToShip。见文件头说明。
    /// @param statsStore  历史匹配率存储（ENG-10 §4）。可为 nullptr ——
    ///        此时 M_hist 全部取冷启动先验（§4.3），选择仍可工作，
    ///        但 result.json 的 `match_stats_version` 无来源。
    PosePipeline(const data::MeasurementConfig& measurement,
                 const data::ValidationConfig& validation,
                 const TargetModelManager& models,
                 const data::OpticalRigCalibration& rig,
                 const data::IMatchStatsStore* statsStore = nullptr);

    /// 全注入构造（单测用）。
    PosePipeline(const data::MeasurementConfig& measurement,
                 const data::ValidationConfig& validation,
                 const TargetModelManager& models,
                 const data::OpticalRigCalibration& rig,
                 const Stages& stages,
                 const data::IMatchStatsStore* statsStore = nullptr);

    ~PosePipeline() override;

    PosePipeline(const PosePipeline&) = delete;
    PosePipeline& operator=(const PosePipeline&) = delete;

    // ---- IPosePipeline ----

    void setStatisticsObserver(IPipelineObserver* observer) override;

    void setCoarseAttitude(double azimuthDeg,
                           double elevationDeg,
                           double distanceM) override;

    bool detect(const data::ImageFrame& frame,
                data::DetectionResult& out) override;

    bool estimateScale(const data::ImageFrame& frame,
                       const data::DetectionResult& detection,
                       data::TargetScaleEstimate& out) override;

    bool selectCamera(const data::MultiCameraFrame& frame,
                      const std::vector<data::CameraRole>& allowed,
                      data::MeasurementSelectionResult& out) override;

    bool selectBestFrame(const std::vector<data::ImageFrame>& frames,
                         int& bestIndex,
                         data::ImageQuality& quality) override;

    bool solvePose(const data::MultiCameraFrame& frame,
                   data::CameraRole camera,
                   const data::CameraCalibration& calib,
                   data::ShipPoseResult& out) override;

    bool validate(const data::ShipPoseResult& result,
                  data::PoseValidationResult& out) override;

    // ---- 诊断接口：**已删除**（裁决 C-008 同批的"死诊断接口清理"）----
    //
    // 本处原有 6 个"内部状态查询"访问器：
    //   lastPnpStats() / lastCadResult() / lastMatchResult() /
    //   lastCadAssisted() / lastMatchStatsUsage() / lastSelectionColdStart()
    //
    // 删除的判据是**消费者实测**，不是"看起来没人用"：
    //   · `lastCadResult()` / `lastCadAssisted()`：全工程**零消费者**
    //     （生产与测试都没有）；
    //   · 其余四个：**只有测试**在用，而它们查询的是私有成员 ——
    //     测试读被测对象的内部状态，等于把测试与实现绑在一起
    //     （本文件反复避免的那类断言）。
    //
    // ⚠ 它们**不属于 IF-SW-02**（SYS-04 §4.2 冻结的调用面只有
    //   detect / estimateScale / selectCamera / selectBestFrame /
    //   solvePose / validate），故删除不需要动任何冻结接口 ——
    //   这正是"不属 IF-SW-02"这句话的实际价值。
    //
    // ⚠ 与之相对，**禁止**给 `IPosePipeline` 增加此类访问器的裁决
    //   （C-002 Step 5，见 IPipelineObserver.h）仍然有效，且正是本次
    //   清理的依据：统计量走 `IPipelineObserver` 推送，不走内部状态查询。
    //
    // ⚠ 一处**未取消的要求**（不要读成"这条需求被删了"）：
    //   ENG-10 §2.4 / §4.4 要求 result.json 记录 `cad_assisted` 与
    //   M_hist 三元组 + 冷启动标记。这不属于"诊断接口"，而是**尚未实施
    //   的落盘字段**；本处删除的只是访问器，私有成员
    //   （`lastCadResult_` / `lastCadAssisted_` / 选择器的 usage 缓存）
    //   **全部保留**，实施该需求时经推送通道（`IPipelineObserver`）
    //   或新增推送载荷暴露，而不是重新加回查询接口。
    //   已登记为本批的开放项。

    /// ZYX 欧拉角分解（Yaw → Pitch → Roll），单位 deg（ENG-09 §2.2）。
    ///
    /// 与 `aircraftToShip.rotation` 是同一旋转的两种表达，必须自洽；
    /// 独立暴露（static）以便单测直接校验这条自洽性 —— 分解写错不会
    /// 报错，只会让 Yaw 与矩阵不一致，而验收指标正是 Yaw。
    static void decomposeZyxDeg(const cv::Matx33d& rotation,
                                double& yawDeg,
                                double& pitchDeg,
                                double& rollDeg);

private:
    /// 取 `MultiCameraFrame` 中某角色对应的图像。
    /// 角色非法时返回空帧引用（调用方据 `image.empty()` 判定）。
    static const data::ImageFrame& frameOf(const data::MultiCameraFrame& frame,
                                           data::CameraRole role);

    /// 取某角色的标定（内参 + cameraToRig）。
    static const data::CameraCalibration& calibOf(
        const data::OpticalRigCalibration& rig, data::CameraRole role);

    /// 用注入的标定与模型尺寸做尺度估计（`estimateScale` 与
    /// `selectCamera` 共用的同一条路径）。
    /// 失败时 `out` 被重置为默认值（distance = 0、confidence = 0）。
    bool scaleEstimateOf(data::CameraRole role,
                         const data::DetectionResult& detection,
                         double realSizeM,
                         data::TargetScaleEstimate& out) const;

    /// 由 2D 图像评估 `ImageQuality`（清晰度/曝光/对比度 + 特征点数）。
    /// 非公开：它的口径是"选择阶段可用"的近似，与 `solvePose` 内基于
    /// 真实 SIFT 关键点数的质量量是两个不同用途的量（后者才进 result.json）。
    static data::ImageQuality assessQuality(const cv::Mat& image);

    /// 统计检测框内"可检测特征"的近似数量（F 分项的 N_detect）。
    /// 见 .cpp 中关于为何不能在此处跑完整 SIFT 的说明。
    static int countDetectableFeatures(const cv::Mat& gray,
                                       const cv::Rect& roi,
                                       int cap);

    // ⚠ 两份配置**按值**持有，而非 ENG-10 §5.1 字面上的 `const&`
    //   （构造函数入参仍是 `const&`，注入面不变）；理由见
    //   MeasurementSelector.h 中同一处说明：存引用会让传临时量的构造通过编译，
    //   之后阈值全是栈上残留数据。本类是整条链的装配点，配置被各子模块
    //   各持一份副本后更不可能出现"半新半旧"的配置视图。
    data::MeasurementConfig measurement_;
    data::ValidationConfig  validation_;

    /// 机型库**按引用**持有（不是配置，是数据）：
    /// 由 ApplicationContext 持有整个进程生命周期，拷贝一份会随
    /// 特征点与描述子成倍放大内存（每个机型数百点的描述子是 128×N 浮点）。
    /// 代价是要求传入对象的生命周期覆盖本对象 —— 由 009 的装配顺序保证。
    const TargetModelManager&      models_;
    data::OpticalRigCalibration    rig_;

    /// 按值持有整份标定（而非只持 `rigToShip`）：`estimateScale` 与
    /// `selectCamera` 也需要内参，见文件头的坐标链说明。
    /// 标定在进程内是只读的，各测量任务共用同一份，故此处按值拷贝一次
    /// 比每次调用都从外部取更安全（外部对象若在任务期间被 CalibrationManager
    /// 重新装载，按值持有可保证本 Pipeline 的注入在构造时就已固定 ——
    /// ENG-10 §5.2 约束 2："注入后不可变"）。

    // 自建的真实链（注入构造时为空）。
    std::unique_ptr<TargetDetector>       ownedDetector_;
    std::unique_ptr<TargetScaleEstimator> ownedScale_;
    std::unique_ptr<FeatureExtractor>     ownedExtractor_;
    std::unique_ptr<FeatureMatcher>       ownedMatcher_;
    std::unique_ptr<PnPPoseEstimator>     ownedPnp_;

    // 非拥有指针：指向注入对象或 owned* 之一。
    TargetDetector*       detector_  = nullptr;
    TargetScaleEstimator* scale_     = nullptr;
    FeatureExtractor*     extractor_ = nullptr;
    FeatureMatcher*       matcher_   = nullptr;
    PnPPoseEstimator*     pnp_       = nullptr;

    // `CadStructureLocator` 无配置、无状态，直接内嵌（不入注入集合）。
    CadStructureLocator cad_;
    MeasurementSelector selector_;
    PoseValidator       validator_;

    // ---- 粗姿态（`setCoarseAttitude` 注入）----
    bool   hasCoarseAttitude_ = false;
    double coarseAzimuthDeg_   = 0.0;
    double coarseElevationDeg_ = 0.0;
    double coarseDistanceM_    = 0.0;

    // ---- 跨阶段缓存（POSE_SOLVE → VALIDATE）----
    // `validate` 的冻结签名只收 `ShipPoseResult`，而验证所需的
    // inlierRatio/matchCount 不在其中，只能由 `solvePose` 阶段留下。
    PnpStats           lastPnpStats_;
    CadStructureResult lastCadResult_;
    MatchResult        lastMatchResult_;
    bool               lastCadAssisted_ = false;

    /// 统计量推送出口（裁决 C-002）。**不持有所有权**，可为空。
    IPipelineObserver* statisticsObserver_ = nullptr;

    /// 把 `lastMatchResult_` 推送给观察者（若有）。
    ///
    /// ⚠ 调用点必须在**匹配完成之后、任何早退之前**：
    /// 统计量此时才成立。若放在 `solvePose` 成功返回之后，
    /// 那么"匹配成功但 PnP 未收敛"这一分支就没有统计量 ——
    /// 而那恰恰是事后最需要看清楚的一次（它有对应点，却没解出位姿）。
    void publishStatistics();
};

}  // namespace algorithm
}  // namespace aircraft
