#pragma once

// ============================================================================
//  src/algorithm/feature/CadStructureLocator.h
//
//  依据：ENG-01 §10（feature 子模块，**文件名与职责均被冻结**：
//                    `CadStructureLocator.h` = B 类：CAD 结构点定位，
//                    "实现 ENG-10 §2.3 的投影—匹配—拟合三步法……职责还包括
//                    ENG-10 §2.4 的可用性判据计算"）、
//        ENG-10 §2.2（A/B 两类分工）/§2.3（B 类定位方法，冻结）/§2.4（可用性判据）、
//        SYS-07 §8.2（裁决 G-1：特征来源分层）、SYS-12 §6.1（CAD 特征点）、
//        SYS-15 §4.5（展布宽度 W 决定 Yaw 精度）、ENG-09 §5.18（ModelPoint3D）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 这个类存在的原因是**精度**，不是完整性。                              │
//  │                                                                      │
//  │ SYS-15 §4.5 的估算模型  σ_θ ≈ σ_px·√12/(W·√N)  里有展布宽度 W。      │
//  │ 若 100 个特征点全部集中在机头 20% 区域，W ≈ 386 px，σ_θ ≈ 0.93 角分 │
//  │ —— 单独一项就吃掉 1 角分指标的全部预算（SYS-07 §8.2 的算例）。       │
//  │ **数量达标不等于精度达标，分布必须被保证。** 所以 B 类不可省略。      │
//  │                                                                      │
//  │ 而 B 类不能用 SIFT 代替：翼尖、进气口唇口、尾翼边缘是**几何不连续**   │
//  │ （角点/棱线），不是纹理不连续。SIFT 关键点由尺度空间极值决定，        │
//  │ 与真实几何角点之间存在随视角变化的偏移，达不到 0.3 pixel 的定位要求； │
//  │ 边缘拟合面对几何量本身，可达 0.1~0.2 pixel（ENG-10 §2.4）。           │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ **实现与 ENG-10 §2.3 的一处偏离（粗姿态的来源）**：
//    §2.3 Step 1 写"CAD 模型按粗姿态投影（粗姿态来自转台角度 + 距离估计）"。
//    但 IF-SW-02（SYS-04 §4.2，已冻结）传给算法的只有
//    `MultiCameraFrame` + `CameraRole` + `CameraCalibration` ——
//    **转台角度不在其中**，算法层也不得访问设备层（ENG-02 §16）。
//    故本类把粗姿态作为**入参**（`coarsePose`），由 `PosePipeline` 提供；
//    `PosePipeline` 的粗姿态有两个来源，优先级如下：
//      1. 应用层注入的转台角度 + 距离估计（`IPosePipeline::setCoarseAttitude`）
//         —— 与 §2.3 的原意一致；
//      2. 缺失时，由 A 类（SIFT）先做一次 PnP 得到。
//    ⚠ 来源 2 有一个**结构性隐患**需登记：B 类存在的理由是"A 类保不住展布"，
//    而来源 2 让 B 类的启动依赖 A 类先给出粗姿态。对无纹理目标（A 类匹配
//    失败），粗姿态无从获得，B 类也就无法工作 —— 恰好退回到 ENG-10 §2.2
//    要避免的局面。已记入 README §6 的待裁决清单。
// ============================================================================

#include <vector>

#include "data/CameraCalibration.h"
#include "data/CameraPose.h"
#include "data/FeatureCorrespondence.h"
#include "data/ImageFrame.h"
#include "data/TargetModel.h"

namespace aircraft
{
namespace algorithm
{

/// B 类定位的一次结果，同时承载 ENG-10 §2.4 的三个可用性判据。
struct CadStructureResult
{
    /// 通过残差门槛的 2D-3D 对应（图像点 + 模型点，单位见 FeatureCorrespondence）。
    std::vector<data::FeatureCorrespondence> correspondences;

    /// Step 1：投影到图像内的结构点数（`featureType == "cad"`）。
    int projectedCount = 0;

    /// Step 2：在窗口内找到两条非平行棱线的点数。
    int matchedCount = 0;

    /// Step 3：因亚像素拟合残差超限被剔除的点数（ENG-10 §2.4）。
    int rejectedByResidual = 0;

    /// 本次定位中最大的亚像素拟合残差，单位 pixel。
    /// 判据：≤ 0.3 pixel（ENG-10 §2.4，超过则剔除该结构点）。
    double maxResidualPx = 0.0;

    /// 展布宽度 W：全部通过的结构点的**横向**（图像 X 方向）分布两端之差，
    /// 单位 pixel。它不直接进入评分式，但直接决定 Yaw 精度
    /// （SYS-15 §4.5），必须写入 result.json（ENG-10 §7 约束 3）。
    double spreadPx = 0.0;

    /// ENG-10 §2.4 的可用性判据结论（对应结构点数 ≥ 4）。
    /// false 时调用方必须退化为纯 A 类并把 `cad_assisted = false` 记为
    /// **不满足项**（本结构体不再另设 degraded 字段：`available == false`
    /// 即是该事实，多一个同义布尔量只会给"两者不一致"留出空间）。
    bool available = false;

    /// ENG-10 §2.4 的第二条判据：W ≥ 0.6 × 目标检测框长边。
    /// 仅在调用方提供了检测框长边时才有意义（`locate` 的
    /// `detectionLongSidePx > 0`）；否则为 true（不判定）。
    bool spreadSufficient = true;
};

/// B 类（CAD 结构点）定位器。实现 ENG-10 §2.3 的投影—匹配—拟合三步法。
///
/// 三步在本实现中的对应：
///   Step 1  投影：`cv::projectPoints` 用粗姿态 + 内参把模型点投到图像；
///   Step 2  匹配：在投影点的局部窗口内取 Canny 边缘，按**梯度方向**把边缘
///           像素聚成两组（一组对应一条棱线）——
///           两条夹角显著的棱线的交点即结构点；
///   Step 3  拟合：对每组边缘像素做 `cv::fitLine`（亚像素），两直线求交，
///           取两组像素到各自拟合直线的 RMS 距离作为残差。
///
/// ⚠ 与 §2.3 的措辞差异：§2.3 写"与 CAD 投影线框做最近邻匹配"。
/// "线框"意味着**棱线连通关系**（哪两点之间有边），而冻结的
/// `data::TargetModel` 只有三维点表 + 描述子，没有任何边/面信息
/// （ENG-09 §5.20）。故本实现改为"在投影点邻域内提取两条主方向边缘"，它
/// 不依赖连通关系，但需要结构点附近**确实存在两条非平行棱线**（这正是
/// ENG-10 §6 对 CAD 模型提出的"含结构棱线"要求）。已记入 README §6。
///
/// ⚠ 本类**不注入 `MeasurementConfig`**：ENG-10 §5.1 的注入矩阵未给本类
///   任何配置项，本类用到的两个常量（0.3 pixel 残差门槛、0.6 展布比）
///   都是 ENG-10 §2.4 **冻结的判据值**，不是可调参数。
class CadStructureLocator
{
public:
    CadStructureLocator() = default;

    /// @param frame     该焦段的图像。
    /// @param model     目标模型（只用 `featureType == "cad"` 的点）。
    /// @param calibration 该通道标定（内参 + 畸变）。
    /// @param coarsePose 粗姿态（Aircraft → Camera）。精度要求不高：
    ///        它只用于把模型点投到"大致位置"，真正的定位由边缘拟合完成。
    ///        窗口半径按投影点间距自适应，故粗姿态的角度误差在数度以内即可。
    /// @param detectionLongSidePx 目标检测框长边，单位 pixel；用于 ENG-10 §2.4
    ///        的"W ≥ 0.6 × 检测框长边"判据。传 0 表示不判定。
    /// @param out       定位结果（含三个可用性判据）。
    /// @return 是否得到**可用**的 B 类对应（即 out.available）。
    ///         注意：返回 false 时 out 中仍可能包含少量通过残差的对应，
    ///         但**不得**按 ENG-10 §2.4 参与 PnP —— 少于 4 个点无法构成
    ///         几何约束，参与只会给 RANSAC 增加噪声。
    bool locate(const data::ImageFrame& frame,
                const data::TargetModel& model,
                const data::CameraCalibration& calibration,
                const data::CameraPose& coarsePose,
                double detectionLongSidePx,
                CadStructureResult& out) const;

    /// 横向展布宽度 W（图像 X 方向两端之差），单位 pixel。
    /// 独立暴露以便单测直接校验 SYS-15 §4.5 的输入量。
    static double spreadWidth(
        const std::vector<data::FeatureCorrespondence>& correspondences);
};

}  // namespace algorithm
}  // namespace aircraft
