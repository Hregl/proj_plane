#pragma once

// ============================================================================
//  src/optical/OpticalRig.h
//
//  依据：ENG-01 §6（optical 模块）、ENG-09 §3.3（裁决 C-18）、§5.3 / §5.4
//        SYS-06 §4.2（接口形状）、5.md §五
//
//  作用（裁决 C-18 冻结的语义，逐字遵循）：
//      "静态描述：三相机通道注册表 + 光机标定数据 + 只读查询接口"
//      **不含任何设备控制能力。**
//
//  ⚠ 裁决 C-18 的层级纠正：
//  SYS-06 §4.2 把 OpticalRig 写在 **Device 章节**，与依存图矛盾 ——
//  ENG-01 §6 与 ENG-03 §12.2 都把 optical 放在 device **之下**
//  （device 依赖 optical）。若 OpticalRig 真在 device 层，则
//  optical 层的 CoordinateTransformer 要拿 rigToShip 就得向上依赖 device，
//  形成**环**。
//  冻结结论：OpticalRig 归 optical 层；相机控制由
//  device/camera/MultiCameraManager 负责，后者**依赖**本类取通道与标定。
//
//  ⚠⚠ 本文件最关键的一处偏离（5.md §六 的实现有真实缺陷）：
//
//  5.md 的 OpticalRig.cpp 写：
//        cameras_.emplace_back("CAM25",  CameraRole::CAM25,  25.0);
//        cameras_.emplace_back("CAM50",  CameraRole::CAM50,  50.0);
//        cameras_.emplace_back("CAM100", CameraRole::CAM100, 100.0);
//  这三个数值是**毫米**。而 ENG-09 §2.3 冻结：
//      "所有长度、平移、距离一律为「米」，
//       CameraChannel.focalLength（**注意：为米，100mm 镜头写作 0.1**）"
//
//  即 5.md 的焦距大了 **1000 倍**。本实现改用 0.025 / 0.05 / 0.1。
//
//  为什么这不是"数值风格"问题（ENG-09 §2.3 称其为"最容易出错的一处"）：
//  焦距进入 OpenCV 内参矩阵的 (0,0) 与 (1,1) 元素，而内参矩阵在
//  两种用法下都出现：
//    · 作为标定结果直接使用（此时单位自洽，错了也不报错）；
//    · 与**三维点**（单位 m，见 ModelPoint3D.position）共同参与
//      投影/反投影计算。
//  第二种用法的前提是二者单位一致。焦距若为毫米而三维点为米，
//  投影出的像点坐标会小 1000 倍 —— 表现为 PnP 解算出的平移量
//  比真实距离大 1000 倍（300 m 变成 300 km），
//  而重投影误差因二者同步缩放**依然很小**，
//  SYS-07 §12 的三项验证指标全部通过。
//  于是系统会给出一个"看起来合格"但完全错误的姿态。
//
//  这也说明为什么 ENG-09 §2.3 要求标定文件的毫米→米转换
//  "必须在 CalibrationManager::load() 内一次性完成，不得泄漏到算法层"：
//  转换点唯一，才能保证全工程只有一种长度单位。
// ============================================================================

#include <vector>

#include "data/CameraChannel.h"
#include "data/OpticalRigCalibration.h"

namespace aircraft
{
namespace optical
{

/// 固定三相机光机平台（裁决 C-18：静态描述 + 只读查询）。
class OpticalRig
{
public:
    OpticalRig();

    /// 按缺省配置建立三个通道（见 .cpp 中的焦距取值说明）。
    ///
    /// 5.md §六 把建通道的动作放在 initialize() 里。本实现保留该形状，
    /// 但**参数化**为可传入自定义通道表 —— 理由：ENG-01 §3.2 的
    /// cameras/*.yaml 是配置的权威来源，通道的 cameraId 与焦点角色
    /// 来自现场，硬编码 "CAM25" 这类标识会让配置无法生效。
    ///
    /// @param channels 通道表。为空时使用缺省三通道（M1 与无配置场景）。
    /// @return 失败条件：通道表非空但未覆盖三个角色，
    ///         或同一角色重复出现（后者会让 getCamera() 的返回值
    ///         取决于表中顺序，而三相机评分的正确性依赖角色唯一）。
    bool initialize(const std::vector<data::CameraChannel>& channels = {});

    /// 按角色取通道。
    ///
    /// @return 指向内部通道的**只读**指针；未找到时返回 nullptr。
    ///
    /// ⚠ 返回 const 指针而非 5.md 的 `CameraChannel*`：
    /// C-18 冻结本类为"只读查询接口"。返回非 const 指针后，
    /// 任何包含本头文件的代码都能改写通道的标定值 ——
    /// 而标定值一旦被中途改写，同一任务内前后帧会使用不同内参，
    /// 表现为"姿态结果缓慢漂移"，且没有任何断言能捕获。
    ///
    /// ⚠ 未找到返回 nullptr 而非哨兵角色：
    /// CameraRole 只有三个取值（ENG-09 §4.1），无 UNKNOWN 可用，
    /// 见 data/CameraRole.h 的说明。
    const data::CameraChannel* getCamera(data::CameraRole role) const;

    /// 全部通道（只读）。5.md 返回 `std::vector<CameraChannel>&`（可写），
    /// 理由同上，改为 const。
    const std::vector<data::CameraChannel>& cameras() const;

    /// 光机标定（含三相机外参与 rigToShip）。
    data::OpticalRigCalibration calibration() const;

    /// 设置光机标定。
    ///
    /// 这是本类**唯一**的写入口，且语义是"由 CalibrationManager 装载后
    /// 灌入"，不是设备控制。它同时把三个通道的 calibration 字段
    /// 统一更新 —— 若只更新 OpticalRigCalibration 而不更新通道，
    /// 则 getCamera(role)->calibration 与 calibration().camXX 会不一致，
    /// 而两处都会被不同模块读取（前者由 device，后者由
    /// CoordinateTransformer），结果是同一帧数据用了两套外参。
    ///
    /// @return false 表示通道表尚未建立（未 initialize()）。
    bool setCalibration(const data::OpticalRigCalibration& calibration);

private:
    std::vector<data::CameraChannel> channels_;
    data::OpticalRigCalibration      calibration_;
};

}  // namespace optical
}  // namespace aircraft
