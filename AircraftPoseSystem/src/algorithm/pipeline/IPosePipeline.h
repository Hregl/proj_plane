#pragma once

// ============================================================================
//  src/algorithm/pipeline/IPosePipeline.h
//
//  依据：SYS-04 §4.2（IF-SW-02：application ↔ algorithm，`MeasurementController`
//                     ↔ `PosePipeline`）、§7 第 1 条（IF-SW-01~05 的签名变更视为
//                     不兼容变更）
//        ENG-02 §2.3（`MeasurementController(IMultiCameraManager* camera,
//                     IPosePipeline* pipeline)` —— 本接口名的出处）
//        ENG-09 §3.1 R3（接口类 `I*` 与实现类置于其所属层）
//        SYS-08 §5.2 / §5.3 / §5.6 / §5.7 / §5.8 / §5.9（各状态的**输出**）
//        ENG-10（算法链内部结构：FeatureExtractor → CadStructureLocator →
//                FeatureMatcher → PnPPoseEstimator → PoseValidator）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本接口是 application 与 algorithm 之间**唯一的**调用面（IF-SW-02）。   │
//  │ 方法集合 = SYS-08 §5 各状态输出列表的逐条转写，不含任何算法内部概念   │
//  │ （特征、描述子、RANSAC、重投影……这些都在实现内部，不跨层）。          │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  为什么由 006 建立（而不是留给 007）：
//  SYS-08 §5 的 SEARCH→TARGET_FOUND→MEASURE_SELECT→CAPTURE→POSE_SOLVE→VALIDATE
//  六个状态**没有一个是纯逻辑**，它们的出口条件都取决于算法结论。
//  若本接口缺席，MeasurementController 只能伪造结论才能推进状态，
//  而 ENG-08 §9 的验收恰恰要求"注入恒失败的算法桩时任务在 T_task 内 FAILED"——
//  连可注入的桩都无处挂载。故本文件在 006 建立最小接口，007 补齐实现。
//
//  ⚠ 007 的扩展边界（ENG-09 §8）：
//    · **允许**增加纯虚方法（属接口扩展；需要同步更新全部实现方，
//      当前实现方只有 007 自己的 Mock 与真实链）；
//    · **不允许**删除或改名已有方法、改动已有签名或返回类型
//      （SYS-04 §7 第 1 条：IF-SW-01~05 的签名变更视为不兼容变更）。
//
//  ⚠ 007 实增的纯虚方法只有 `setCoarseAttitude` 一个，理由见其声明处。
//    （C-02 Step 5 追加了第二个：`setStatisticsObserver`，理由见声明处 ——
//      它同样不构成 IF-SW-02 的签名变更。）
//    **这不是 IF-SW-02 的签名变更**：IF-SW-02 冻结的是"已有方法的签名与
//    返回类型"，而 B 类（CAD 结构点）定位所需的粗姿态在冻结入参集合里
//    根本没有来源 —— 若不扩展接口，只能让算法层去读设备层（ENG-02 §16
//    禁止），或让 B 类依赖 A 类先做一次 PnP（ENG-10 §2.2 明确反对）。
//    已记入 README §6 的待裁决清单。
//
//  ⚠ 本接口**不依赖** device / optical / preview / infrastructure，
//  只依赖 data（ENG-09 §3.1 R3 + SYS-04 §4.2 约束：algorithm 不得包含
//  设备层头文件）。标定通过参数传入 `data::CameraCalibration`，
//  不由算法层自行读取（ENG-10 §5.1 配置注入：算法不依赖 ConfigManager）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <vector>

#include "algorithm/pipeline/IPipelineObserver.h"
#include "data/CameraCalibration.h"
#include "data/CameraRole.h"
#include "data/DetectionResult.h"
#include "data/ImageFrame.h"
#include "data/ImageQuality.h"
#include "data/MeasurementSelectionResult.h"
#include "data/MultiCameraFrame.h"
#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"
#include "data/TargetScaleEstimate.h"

namespace aircraft
{
namespace algorithm
{

/// 姿态解算链的对外接口（SYS-04 IF-SW-02）。
///
/// ⚠ 全部方法均为**同步阻塞**（一次调用内完成计算）。异步化由调用方决定：
/// SYS-08 §9 把状态机放在 Application 线程上，ENG-05 §3 的 AlgorithmWorker
/// 负责"算法线程独立"（§21 约束 5）。二者通过队列衔接，不在本接口内体现。
///
/// ⚠ 返回值语义统一：true = 本次计算给出了**可用结论**；false = 未给出
/// （图像为空、目标不在视场、内点不足、未收敛……）。false 时**不要求**
/// lastError() 有值，因为失败原因的分类（瞬时/硬件/能力边界）由
/// MeasurementStrategy 依据 SYS-08 §7.2〔引用无效·依据待裁决·见 Q-D2〕 判定，而非由算法层自陈。
class IPosePipeline
{
public:
    virtual ~IPosePipeline() = default;

    // ---- 统计量推送（裁决 C-002 的落地通道）-------------------------------

    /// 设置统计量观察者（见 IPipelineObserver.h 的完整理由）。
    ///
    /// ⚠ 这是**增设纯虚方法**，不是改动已有签名 —— `IPosePipeline.h`
    /// 的文件头允许前者、禁止后者（SYS-04 §7 第 1 条）。
    /// 之所以不给 `solvePose` 加输出参数，正是为了不触碰后一条：
    /// 那会让 IF-SW-02 的一个冻结方法签名变更，成为不兼容变更。
    ///
    /// @param observer 观察者指针，**不持有所有权**，生命周期必须长于
    ///        本对象的后续调用；传 nullptr 表示停止推送。
    ///        未设置时算法照常工作，只是没有统计量输出 ——
    ///        这是一个**静默**失败，故必须由测试守住（见 IPipelineObserver.h）。
    virtual void setStatisticsObserver(IPipelineObserver* observer) = 0;

    // ---- 粗姿态注入（ENG-10 §2.3 Step 1 的前置量）------------------------

    /// 注入粗姿态（转台角度 + 距离估计）。
    ///
    /// 为什么需要它：ENG-10 §2.3 的 B 类定位第一步是"CAD 模型按粗姿态投影"，
    /// 而粗姿态的原始来源是**转台角度**。但 IF-SW-02（SYS-04 §4.2）传入
    /// 算法的只有 `MultiCameraFrame` + `CameraRole` + `CameraCalibration`，
    /// 转台角度不在其中；算法层也不得访问设备层（ENG-02 §16）。
    /// 故由**应用层**（持有 TurntableController）在每次测量开始时注入。
    ///
    /// @param azimuthDeg  转台方位角，单位 deg。转台已对准目标，故它同时
    ///        近似等于目标的方位。**粗姿态的机头朝向假设与该方位一致**
    ///        （进近场景：机头大致沿"舰→机"连线），故它在相机系中不产生
    ///        额外的偏航余量。该假设的 180° 二义性见 PosePipeline.cpp。
    /// @param elevationDeg 转台俯仰角，单位 deg。它决定粗姿态的俯仰分量。
    /// @param distanceM   距离估计，单位 m（SYS-14 §10）。粗平移取
    ///        (0, 0, distance) —— 转台已对准，目标在光轴上。
    ///
    /// @note 注入是**可选的**：未注入时 B 类不可用（`cad_assisted = false`），
    ///       算法退化为纯 A 类（ENG-10 §2.4 允许的退化路径），不报错。
    ///       交付系统 M1 起由 MeasurementController 在 ALIGN 完成后注入。
    virtual void setCoarseAttitude(double azimuthDeg,
                                   double elevationDeg,
                                   double distanceM) = 0;

    // ---- SEARCH（SYS-08 §5.2：Camera25 → YOLO → DetectionResult）----------

    /// 单帧目标检测。
    ///
    /// @param frame    待检测图像（SEARCH 阶段为 CAM25）。
    /// @param out      `DetectionResult`；返回 true 时 `found` 必为 true，
    ///                 `sourceCamera` 取 frame.role。
    /// @return 是否检出目标。**未检出返回 false 而非 true+found=false**：
    ///         "检出但 found=false" 这种组合无法与"根本没跑检测"区分，
    ///         而 SEARCH 的重试判据（§7.3：YOLO 未检出目标）依赖这个区分。
    virtual bool detect(const data::ImageFrame& frame,
                        data::DetectionResult& out) = 0;

    // ---- TARGET_FOUND（SYS-08 §5.3：尺度估计）----------------------------

    /// 目标尺度估计（SYS-14 §10：Z = f·L/(l·s)）。
    ///
    /// @note `TargetOffset` **不**由本方法给出：它是
    ///       `DetectionResult.bbox` 中心与图像中心的纯几何差
    ///       （ENG-09 §2.4 冻结参考原点），由 MeasurementController 计算，
    ///       不需要算法结论。把纯几何留给上层，可让对准回路在算法不可用时
    ///       仍能工作（ALIGN 的重算命令只依赖像素偏差）。
    virtual bool estimateScale(const data::ImageFrame& frame,
                               const data::DetectionResult& detection,
                               data::TargetScaleEstimate& out) = 0;

    // ---- MEASURE_SELECT（SYS-08 §5.6：MeasurementSelector）----------------

    /// 测量通道选择。候选构造（ENG-10 §3：Q/F/M/E 四分项）在实现内部完成，
    /// 上层只给出**允许的候选集合**（已被排除的相机不在此列，§7.4〔引用无效·依据待裁决·见 Q-D2〕）。
    ///
    /// @param frame   三相机同步帧。
    /// @param allowed 允许参与选择的焦段（顺序即优先级）。
    /// @param out     `MeasurementSelectionResult`。
    /// @return 是否选出相机。候选集合为空或全部不达门槛（SYS-14 §6）时 false。
    virtual bool selectCamera(const data::MultiCameraFrame& frame,
                              const std::vector<data::CameraRole>& allowed,
                              data::MeasurementSelectionResult& out) = 0;

    // ---- CAPTURE（SYS-08 §5.7：多帧采集，选最佳帧）------------------------

    /// 从已采集的多帧中选出最佳测量帧（§5.7：5~10 帧，选择最佳帧）。
    ///
    /// @param frames   同一相机的连续帧，数量 = MeasurementConfig::captureFrameCount。
    /// @param bestIndex 输出：最佳帧在 frames 中的下标。
    /// @param quality  输出：该帧的 `ImageQuality`（供 result.json 记录，
    ///                 SYS-04 §6.4 的 `quality` 字段）。
    /// @return 是否存在可用帧（全部过曝/模糊/空时 false）。
    virtual bool selectBestFrame(const std::vector<data::ImageFrame>& frames,
                                 int& bestIndex,
                                 data::ImageQuality& quality) = 0;

    // ---- POSE_SOLVE（SYS-08 §5.8：特征→匹配→RANSAC→PnP→坐标变换）----------

    /// 姿态解算，输出 `ShipPoseResult`（含 `aircraftToShip`）。
    ///
    /// @param frame   同步帧（含被选相机与其他相机的图像）。
    /// @param camera  测量通道（§8：MEASURE 阶段用 Selected Camera）。
    /// @param calib   该通道的标定（内参 + 相机→光机外参）。**由调用方注入**，
    ///                算法层不读标定文件（ENG-10 §5.1）。
    /// @return 是否收敛出结果。false 时上层按 §7.3 升级规则重试。
    virtual bool solvePose(const data::MultiCameraFrame& frame,
                           data::CameraRole camera,
                           const data::CameraCalibration& calib,
                           data::ShipPoseResult& out) = 0;

    // ---- VALIDATE（SYS-08 §5.9：重投影误差 / 内点比例 / Yaw 范围）---------

    /// 结果可信度验证，输出 `PoseValidationResult`。
    ///
    /// @note 阈值来自 `ValidationConfig`（ENG-09 §6.6），在实现**构造时**注入
    ///       （ENG-10 §5.2 约束 2：注入后不可变，禁止运行时改配置）。
    ///
    /// @return 是否通过全部判据（**等于 `out.valid`**）。
    ///
    /// ⚠ 本返回值**不是**"验证过程是否执行成功"。本接口**没有**表达执行
    ///   失败的通道：判不合格与无法执行都只能表现为 `out.valid == false`，
    ///   两者的差异体现在 `out.reason`（裁决 C-008）与三个数值字段上。
    ///   调用方**不得**据返回值分支，一律读 `out.valid` —— 否则会把
    ///   "判据不通过"读成"验证过程失败"，从而跳过"排除该相机并回退"
    ///   这一步（这正是 R05，2026-09-24 审查报告）。
    ///
    ///   与 `solvePose` 的 @return 语义**不同**：那里 false = 没算出结果；
    ///   此处 false = 算出来了但**不合格**。同一个 bool、两种含义，
    ///   是本接口最容易误读的地方。
    ///
    /// @note 实现须保证返回值与 `out.valid` **恒等**；调用方会对账，
    ///       不一致按契约违背处理（可见地失败，不静默按任一方继续）。
    virtual bool validate(const data::ShipPoseResult& result,
                          data::PoseValidationResult& out) = 0;
};

}  // namespace algorithm
}  // namespace aircraft
