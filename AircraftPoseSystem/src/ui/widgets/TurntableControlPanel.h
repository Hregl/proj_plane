#pragma once

// ============================================================================
//  src/ui/widgets/TurntableControlPanel.h
//
//  作用：主程序里的**受控转台操作入口**（目标输入 / 定位 / 停止）以及
//  连接、反馈、门禁、命令阶段、停止阶段的显示。
//
//  ⚠ 本类只发信号，不调用任何控制器：
//    · `positionRequested` / `stopRequested` 由 app 层接收；
//    · 判定"能否定位 / 是否到位 / 是否停止"全部在
//      `app::TurntableControlService`，本类只显示它给出的值语义快照；
//    · 不存在第二套控制器或第二条发令路径。
//
//  ⚠ 三个阶段（受理 / 交付 / 反馈到位）与停止阶段由快照分别给出，
//    本面板**不得**把它们合并成一个“成功”标签。
// ============================================================================

#include <QWidget>

#include "ui/TurntableControlView.h"

class QLabel;
class QDoubleSpinBox;
class QPushButton;

namespace aircraft
{
namespace ui
{

class TurntableControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TurntableControlPanel(QWidget* parent = nullptr);

    /// 显示一份控制快照。纯显示，不触发任何设备动作。
    void setView(const TurntableControlView& view);

    /// 无快照（转台未装配或启动未就绪）。
    void clear();

signals:
    /// 操作者按下“定位”。参数为输入框中的目标角（deg）。
    /// 是否受理由 app 层的既有门禁决定。
    void positionRequested(double azimuthDeg, double elevationDeg);

    /// 操作者按下“停止”。
    void stopRequested();

private:
    void emitPositionRequested();

    QLabel* connection_    = nullptr;
    QLabel* connectionDetail_ = nullptr;
    QLabel* feedback_      = nullptr;
    QLabel* sample_        = nullptr;
    QLabel* motion_        = nullptr;
    QLabel* fault_         = nullptr;
    QLabel* permit_        = nullptr;
    QLabel* gate_          = nullptr;
    QLabel* commandStage_  = nullptr;
    QLabel* commandDetail_ = nullptr;
    QLabel* stopStage_     = nullptr;
    QLabel* stopDetail_    = nullptr;
    QLabel* action_        = nullptr;

    QDoubleSpinBox* azimuthInput_   = nullptr;
    QDoubleSpinBox* elevationInput_ = nullptr;
    QPushButton*    positionButton_ = nullptr;
    QPushButton*    stopButton_     = nullptr;

    bool initializedInputs_ = false;
};

}  // namespace ui
}  // namespace aircraft
