#pragma once

// ============================================================================
//  src/optical/CalibrationManager.h
//
//  依据：ENG-09 §2.3（长度单位 + 装载转换要求）、§5.2 / §5.3 / §6.2
//        ENG-01 §3.4（标定文件清单）、§6（optical 模块）
//        ENG-10 §5.3（配置装载的三种失败模式）、SYS-11（标定）
//        5.md §七
//
//  职责：相机内参 + Camera→Rig 外参 + Rig→Ship 的装载与管理。
//
//  ⚠⚠ 本类承担一项**冻结的强制职责**（ENG-09 §2.3 原文）：
//
//      "装载转换：若使用 OpenCV 工具链生成的标定文件（平移单位为毫米），
//        转换必须在 CalibrationManager::load() 内一次性完成，
//        **不得泄漏到算法层**。"
//
//  为什么必须集中在这一处转换（这是本类存在的核心理由）：
//  OpenCV 的标定工具链（cv::calibrateCamera / solvePnP 的标定流程）
//  习惯以**毫米**表达平移量（标定板方格边长通常按毫米录入）。
//  而 ENG-09 §2.3 冻结本工程所有长度为**米**。
//  若转换散落在多个消费点（PnP、坐标合成、结果输出各转一次），
//  则"漏转一处"是必然的 —— 而漏转的表现是某个平移量大了 1000 倍，
//  且**不会有任何报错**：投影计算中若分子分母同时含该量，
//  重投影误差依然很小，SYS-07 §12 的三项验证指标全部通过。
//  最后得到的是一个"看起来合格"但完全错误的姿态。
//
//  转换点唯一 ⇒ 只需在这一个地方正确。本类的 load() 因此是
//  全工程唯一的毫米→米边界。
//
//  ⚠ 关于缺省标定（loadDefaults）：见 .h 中该方法的详细说明。
//  它存在的唯一理由是 M1/M2 阶段尚无真实标定数据，
//  而 ENG-10 §5.3 要求"文件缺失 → 启动失败"，二者需要一个
//  **显式的、调用点可见的**折中，而不是让 load() 悄悄放行。
// ============================================================================

#include <string>
#include <vector>

#include "data/CameraChannel.h"
#include "data/CameraRole.h"
#include "data/OpticalRigCalibration.h"
#include "data/OpticalRigConfig.h"

namespace aircraft
{
namespace optical
{

/// 标定装载与管理（SYS-11）。
class CalibrationManager
{
public:
    CalibrationManager() = default;

    /// 从标定目录装载全部 7 个文件（ENG-01 §3.4）。
    ///
    /// 目录内容（文件名冻结，方向命名与 ENG-09 §2.1 的 `AToB` 一致）：
    ///     cam25.yaml / cam50.yaml / cam100.yaml   内参 + 畸变 + 图像尺寸
    ///     cam25_to_rig.yaml / cam50_to_rig.yaml /
    ///     cam100_to_rig.yaml                      Camera → OpticalRig 外参
    ///     rig_to_ship.yaml                        OpticalRig → Ship 外参
    ///
    /// **本方法是全工程唯一的毫米→米转换点**（见文件头）。
    ///
    /// @return false 表示至少一个文件缺失或无法解析。
    ///         ENG-10 §5.3：文件缺失属"启动失败"类，
    ///         调用方应终止启动而**不是**继续用零值标定运行 ——
    ///         零值标定会让 PnP 解出无意义的结果，而验证指标因
    ///         同样退化而可能通过。
    ///         ⚠ 与"字段缺失"不同：单个字段缺失时用默认值并记日志
    ///         （ENG-10 §5.3 第二类），故字段级缺失**不**导致本方法返回
    ///         false，但会在 lastWarnings() 中留下记录。
    bool load(const std::string& calibrationDir);

    /// 装载**合成**标定，用于尚无真实标定数据的阶段（M1/M2）。
    ///
    /// ⚠ 本方法是有意与 load() 分开的，不是 load() 的容错分支。
    ///
    /// 分开的理由：ENG-10 §5.3 冻结"文件缺失 → 启动失败"。
    /// 若让 load() 在文件缺失时自动回退到合成值，则该冻结规则被静默
    /// 破坏 —— 现场部署时标定文件忘拷，系统会照常启动，
    /// 并用一套**虚构的内参**给出姿态结果，而日志里只有一条
    /// 容易被忽略的警告。分成两个方法后，"用合成标定"成为调用点上
    /// 一个显式的、可被审查的决定。
    ///
    /// ⚠ 用合成标定算出的**任何测量结果都没有物理意义**。
    /// 合成内参的焦距是由一个标称视场角反推的占位值，
    /// 与真实镜头无关；外参是单位矩阵，即假定三台相机光轴平行且
    /// 与舰体坐标轴对齐。它的唯一用途是让预览、算法链的数据通路
    /// 可以被完整跑通与调试。
    ///
    /// @param imageWidth  合成内参对应的图像宽度，单位 pixel。
    /// @param imageHeight 合成内参对应的图像高度，单位 pixel。
    /// @return 恒定 true（合成值总能构造），保留返回值以便将来加入校验。
    bool loadDefaults(int imageWidth, int imageHeight);

    /// 按角色取单台相机的标定（内参 + Camera→Rig）。
    ///
    /// @return 未装载或角色无对应标定时返回**默认构造**的 CameraCalibration。
    ///         ⚠ 默认构造的 cameraMatrix 在 data/CameraCalibration.h 中被
    ///         初始化为**单位矩阵**（而非空矩阵），使"未装载"这件事
    ///         至少不会在矩阵运算中表现为奇异；但调用方**必须**先用
    ///         loaded() 判断，不得依赖该回退值参与测量。
    data::CameraCalibration getCalibration(data::CameraRole role) const;

    /// Rig → Ship 外参（ENG-09 §5.3）。
    /// 未装载时返回单位变换。
    ///
    /// ⚠ 该变换是 SYS-15 §4 中"舰体坐标标定"系统误差项（0.20 角分）
    /// 的物理来源 —— 它是误差预算中**不可通过多帧平均削减**的分量之一，
    /// 因为它对同一任务内的所有帧是**同一个**常量偏差。
    data::Transform getRigToShip() const;

    /// 完整的光机标定（三相机 + rigToShip），供 OpticalRig::setCalibration
    /// 一次性灌入。分开逐个灌入会造成中间态（部分通道已更新），
    /// 而该中间态若被并发读取会得到混合了新旧标定的结果。
    data::OpticalRigCalibration calibration() const;

    /// 标定版本标识，来自 OpticalRigConfig::calibrationId（ENG-09 §6.2）。
    /// 写入结果包供追溯（ENG-09 §6.8、SYS-12 §17）。
    std::string calibrationId() const;

    /// 是否已成功装载（load() 或 loadDefaults() 返回 true 后为 true）。
    bool loaded() const;

    /// 本次装载中发生的非致命问题（字段缺失等）。
    /// ENG-10 §5.3 要求这类问题"记日志"而非失败 —— 本方法即该记录。
    std::vector<std::string> lastWarnings() const;

    /// 最近一次失败原因（人读），供日志与 UI。
    std::string lastErrorText() const;

private:
    /// 由图像尺寸与标称视场角构造合成内参。
    /// 见 loadDefaults() 的说明：**仅供数据通路调试**。
    static data::CameraCalibration makeSyntheticCalibration(int imageWidth,
                                                           int imageHeight);

    data::OpticalRigCalibration calibration_;
    std::string                 calibrationId_;
    bool                        loaded_ = false;
    std::vector<std::string>    warnings_;
    std::string                 lastErrorText_;
};

}  // namespace optical
}  // namespace aircraft
