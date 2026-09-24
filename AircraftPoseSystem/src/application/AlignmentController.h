#pragma once

// ============================================================================
//  src/application/AlignmentController.h
//
//  依据：SYS-10 §7（居中判据 ±50 pixel）、§8（**像素误差 → 角度误差的公式**）、
//        §6.2（工作流程）、§9（粗调/精调）、§12（与状态机关系）、§14（对准失败）
//        SYS-08 §3.2（输入 TargetOffset / 输出 TurntableCommand）、§5.4（ALIGN）
//        ENG-09 §2.2（角度 = 度）、§2.4（**像素坐标符号约定**）、§5.10（命令字段名）、
//        §5.11（TargetOffset）、§6.3（TurntableConfig，含 centerThreshold）
//        ENG-02 §10.2 / ENG-04 §10.2（输入/输出）、ENG-01 §9（本文件在冻结清单内）
//        ENG-09 §7 裁决 C-20（对准次数**不**在此处，唯一数据源是
//                          MeasurementConfig::maxAlignAttempts）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类**只做一次几何换算**：像素偏差 → 转台绝对角度命令。                │
//  │ 它不计数、不循环、不判断是否需要再来一次 ——                       │
//  │ 对准闭环的次数与超时由 RetryManager + StateMachine 掌握（C-20）。      │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  为什么是"绝对命令"而不是"角度增量"（这是 7.md §九 的核心错处）：
//  7.md 写的是 `cmd.azimuth = offset.pixelX;` —— 把**像素**直接赋给
//  **角度**字段。该赋值不会产生任何编译错误，后果是转台按"1 pixel ≈ 1 度"
//  运动，而实际比例约为 1 pixel ≈ 0.03 度（fx=2000 时）。因为对准是闭环，
//  这个错误的表现是"来回震荡、永不收敛"，最终耗尽 8 次 ALIGN 重试并报
//  2003 —— 故障现象指向算法，根因却在量纲。
//  正确关系见 SYS-10 §8：Δθ = arctan(Δpixel / f)，f 为**像素**单位的焦距参数。
//
//  ⚠ 与 7.md / SYS-08 §3.2 的签名偏离（一处）：
//  `calculate()` 除 TargetOffset 外还需**转台当前角度**与**相机标定**。
//  理由：`TurntableCommand` 的两个字段名是 `azimuthCommand` /
//  `elevationCommand`（ENG-09 §5.10），ENG-09 给它们的定义是
//  "**要求转台去哪**"（与 `TurntableState` 的"转台现在在哪"相对），
//  即**绝对角度**。绝对角度无法只由像素偏差算出，必须叠加当前角度；
//  而 arctan(Δpixel/f) 的 f 只能来自 `CameraCalibration::cameraMatrix`
//  —— 全系统中唯一以像素为单位的焦距参数（`CameraChannel::focalLength`
//  按 ENG-09 §2.3 是**米**，不能用于该式，见下方 convertToAngle 的说明）。
//
//  ⚠ SYS-10 §9 的粗调/精调在本层**无法实现**：`TurntableCommand` 只有
//  两个角度字段（ENG-09 §5.10），没有任何速度字段，而 SYS-10 §9 也未给出
//  增益、比例系数与粗精切换阈值。故速度选择只能落在转台后端内部
//  （`TurntableConfig::coarseSpeed` / `fineSpeed` 的消费方是 device 层）。
//  这属冻结文档的缺口，本阶段不新造字段（ENG-09 §8 变更流程）。
// ============================================================================

#include "data/CameraCalibration.h"
#include "data/ErrorInfo.h"
#include "data/TargetOffset.h"
#include "data/TurntableConfig.h"
#include "data/TurntableCommand.h"
#include "data/TurntableState.h"

namespace aircraft
{
namespace application
{

/// 视觉闭环转台控制（SYS-10）。
class AlignmentController
{
public:
    /// @param config 转台配置快照（行程限位 + centerThreshold）。
    ///        按 ENG-09 §6.8 在任务级冻结，故构造时取副本。
    explicit AlignmentController(const data::TurntableConfig& config);

    /// 像素偏差 → 转台绝对角度命令（SYS-10 §6.2 的"像素误差转换角度误差"
    /// 与"生成转台命令"两步）。
    ///
    /// @param offset      目标中心相对**图像中心**的像素偏差（ENG-09 §2.4）。
    /// @param current     转台当前状态（提供当前方位/俯仰角，见文件头说明）。
    /// @param calibration 当前通道的标定（提供 fx / fy，单位像素）。
    /// @return 转台命令。**若任一前提不成立（越程、标定无效），
    ///         返回的是"保持当前角度"的原地命令**（见下方注释），
    ///         调用方必须先查 commandValid() 再决定是否下发。
    data::TurntableCommand calculate(const data::TargetOffset& offset,
                                     const data::TurntableState& current,
                                     const data::CameraCalibration& calibration);

    /// §5.4 / SYS-10 §7 的居中判据：
    ///     |pixelX| <= centerThreshold && |pixelY| <= centerThreshold
    bool isCentered(const data::TargetOffset& offset) const;

    /// 按上述判据填写 `TargetOffset::centered`。
    ///
    /// 该字段的判定逻辑放在这里而不是 data 层，是 TargetOffset.h 的明文
    /// 规定（"data 不含业务逻辑"）；C-11 亦指出该判据只有在 pixelX/pixelY
    /// 以图像中心为原点时才有物理意义。
    void judgeCentered(data::TargetOffset& offset) const;

    /// 上一次 calculate() 是否给出了**可信**的命令。
    ///
    /// 与 lastError() 的关系：commandValid() == false 时 lastError() 必有
    /// 说明；但 lastError().code 仍可能为 0 —— 见 lastError() 的取值表。
    /// 故**不可**用 code != 0 代替本查询。
    bool commandValid() const;

    /// 上一次 calculate() 的失败原因。0 表示命令有效（或命令有效但有附注）。
    ///
    /// 码的取值（裁决 C-006 更新）：
    ///   kErrTurntableOverTravel(2002)  —— 命令超出转台行程（§7.7 的能力边界）
    ///   kErrCalibrationMissing(4001)   —— 标定缺失/无效，拒绝换算
    ///   kErrStateFailure(9004)         —— 输入含非有限值等兜底情形
    ///   0                              —— **命令有效**时的附注（如"行程未
    ///                                     配置，跳过越程判定"）。这一类是
    ///                                     事实记录而非失败，故不加码。
    data::ErrorInfo lastError() const;

    /// 上一次 calculate() 是否因**超出转台行程**而失败（§7.7 的能力边界，
    /// 须"停止运动 + 报警 + FAILED"，不可重试）。
    bool overTravel() const;

private:
    data::TurntableConfig config_;

    data::ErrorInfo lastError_;

    bool commandValid_;
};

}  // namespace application
}  // namespace aircraft
