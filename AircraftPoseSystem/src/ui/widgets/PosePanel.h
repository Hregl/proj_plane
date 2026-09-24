#pragma once

// ============================================================================
//  src/ui/widgets/PosePanel.h
//
//  依据：9.md §六（PosePanel：显示 Yaw / Pitch / Roll）
//        ENG-01 §13、ENG-02 §14、ENG-04 §14（均只列类名，9.md 的内容更具体）
//        ENG-09 §5.24（ShipPoseResult 的 yaw/pitch/roll，**单位 deg**）
//
//  ---- 与 9.md §六 的两处偏离 ----
//
//  (1) **增加 clear()，且未解算时显示 "—" 而不是 "0.0°"。**
//      这不是显示偏好，是本工程反复出现的同一类错误的一个实例：
//      ShipPoseResult 的 yaw/pitch/roll 默认值都是 0.0（data/ShipPoseResult.h），
//      于是"解算还没跑过"与"解算结果恰好是 0°"在界面上长得一模一样。
//      真实后果：010 首次运行、或 VALIDATE 失败后重试期间，操作者看到
//      Yaw = 0.000° 会以为系统已给出结果，而实际上一个结果都没有。
//      用 "—" 让"无结果"成为一个**看得见**的状态。
//
//  (2) **Yaw 同时给出角分。** 依据是 README 首段与 SYS-15 §4：验收指标
//      是 Yaw ≤ **1 角分** = 0.0167°，而 9.md §六 的示例只显示度。
//      只显示度时，操作者需要在心里做度→角分的换算才能判断"是否进指标"，
//      而把 0.1° 误读成"接近 1 角分"（实际已超差 6 倍）是很自然的。
//      Pitch / Roll **不给角分**：它们没有角分级指标，
//      给出换算后的数字只会让人以为那也是一个受检量。
//      两个数字是同一个量的两种单位（ENG-09 §2.2），不是两个观测量。
// ============================================================================

#include <QString>
#include <QWidget>

class QLabel;

namespace aircraft
{
namespace ui
{

/// 姿态显示面板（9.md §六）。
class PosePanel : public QWidget
{
public:
    explicit PosePanel(QWidget* parent = nullptr);

    /// 显示一次解算结果（9.md §六 的原接口，保留）。
    /// @param yaw/pitch/roll 单位 **deg**（ENG-09 §5.24），由调用方负责换算。
    void updatePose(double yaw, double pitch, double roll);

    /// 回到"尚未解算"状态（见偏离说明 (1)）。
    void clear();

private:
    QLabel* yaw_   = nullptr;
    QLabel* pitch_ = nullptr;
    QLabel* roll_  = nullptr;
};

}  // namespace ui
}  // namespace aircraft
