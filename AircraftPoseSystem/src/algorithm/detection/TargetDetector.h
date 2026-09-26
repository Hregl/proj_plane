#pragma once

// ============================================================================
//  src/algorithm/detection/TargetDetector.h
//
//  依据：SYS-07 §4（TargetDetection）、ENG-02 §11.1（文件路径与职责）、
//        ENG-01 §10（detection 子模块）、ENG-03 §12.5（algorithm 只依赖
//        data + OpenCV）、ENG-10 §5.1（配置注入矩阵）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 与 8.md §四 的偏离：8.md 自行定义了 `DetectionResult`（int x/y/w/h +  │
//  │ double confidence）并把它放在本头文件里。                            │
//  │ ENG-09 §5.12 已冻结 `data::DetectionResult`（cv::Rect bbox +        │
//  │ CameraRole sourceCamera），故本类**使用 data 层的冻结类型**，不再      │
//  │ 定义第二个同名结构体。理由有三：                                      │
//  │  1. 结构体是跨层契约，两份定义必然漂移；                              │
//  │  2. bbox 直接可用于 ROI / mask / 距离变换；四个散装 int 每次都要重组；  │
//  │  3. sourceCamera 是必需的 —— 三焦段候选打分时每个检测结果必须自证      │
//  │     来自哪台相机，否则候选集合无法建立（SYS-14 §7）。                  │
//  │ 该偏离已记录在 data/DetectionResult.h 的文件头与本工程 README §6。     │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 与 IPosePipeline::detect 的关系：二者的**调用契约完全一致**
//  （bool 返回值 + 出参），`PosePipeline` 的 detect 就是本接口的一次转发。
//  之所以仍保留本接口，是因为检测只是算法链 7 步中的第 1 步，链的其余 6 步
//  同样需要被独立替换、独立测试（ENG-06 §8 的算法测试按模块划分）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "data/DetectionResult.h"
#include "data/ImageFrame.h"
#include "data/MeasurementConfig.h"

namespace aircraft
{
namespace algorithm
{

/// 目标检测抽象（SYS-07 §4）。实现方向：`TargetDetector` → `YoloDetector`。
///
/// 返回 false 表示**未检出**，而非"检出但 found=false"——
/// SYS-08 §7.3 给 SEARCH 的重试判据是"YOLO 未检出目标"，
/// 而"检出了但框不可用"与"根本没跑检测"必须是可区分的两种情形。
class TargetDetector
{
public:
    virtual ~TargetDetector() = default;

    /// @param frame 待检测图像（SEARCH 阶段为 CAM25 的大视场图）。
    /// @param out   检测结果。返回 true 时 `found` 必为 true，
    ///              且 `sourceCamera` 取 `frame.role`。
    /// @return 是否检出满足最小尺寸门槛的目标。
    virtual bool detect(const data::ImageFrame& frame,
                        data::DetectionResult& out) = 0;
};

/// 合成场景检测器（8.md §五 的 `MockTargetDetector`）。
///
/// **它不是"随机返回一个框"的占位实现**，而是"阈值 + 最大连通域外接框"的
/// 真实检测器，用于两类场合：
///   1. 合成场景闭环（VirtualCamera 画出亮目标、暗背景）—— ENG-06 §3 的
///      integration 测试与 1.md Phase 10 的 M1 首跑都依赖它；
///   2. Golden 数据集的离线复核（ENG-06 §10）。
///
/// ⚠ **不得用于真实图像。** 真实场景中背景亮度分布未知、目标无固定灰度，
/// 本类会稳定地检出错误区域。真实检测由 YOLO 承担（SYS-07 §4.2），
/// 届时本类保留为测试夹具即可 —— 这正是不把它写死的理由。
///
/// 实现要点：
///   · 灰度化（已灰度则原样使用）；
///   · 固定阈值二值化 —— 阈值取 (max+min)/2 而非 Otsu：Otsu 在"目标占画面
///     比例很小"时会退化为按背景方差分割，而本类的使用场景（暗背景上一个
///     亮目标）恰恰是 Otsu 最不擅长的形态；
///   · 取最大连通域的外接框，`confidence` 由该域占框面积比给出（填充度）。
class MockTargetDetector : public TargetDetector
{
public:
    /// @param config 提供 `minTargetPixelSize`（ENG-10 §5.1 的注入行）。
    ///        注入面是 `const&`（ENG-10 §5.2 约束 2 要求注入后不可变），
    ///        但对象内**按值存一份** —— 见下方 config_ 的说明。
    explicit MockTargetDetector(const data::MeasurementConfig& config);

    bool detect(const data::ImageFrame& frame,
                data::DetectionResult& out) override;

    /// 测试用：强制返回固定的检出结果（`found=true`），跳过图像处理。
    /// 使"算法链的其余部分"可在不构造图像的情况下被单测（ENG-06 §8）。
    void forceDetection(const data::DetectionResult& result);

    /// 测试用：强制返回"未检出"。
    void forceMiss();

private:
    /// ⚠ 按值持有（入参仍是 `const&`），理由同 MeasurementSelector.h：
    /// 存引用时传临时量的构造可编译但悬空，此处表现为
    /// `minTargetPixelSize` 门槛随机生效/失效。见 ENG-10 §5.2 约束 2。
    data::MeasurementConfig config_;

    bool        forced_ = false;   ///< 是否处于强制模式。
    bool        forcedFound_ = true;
    data::DetectionResult forcedResult_;
};

}  // namespace algorithm
}  // namespace aircraft
