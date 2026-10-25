// ============================================================================
//  src/ui/widgets/TurntableControlPanel.cpp
//
//  见头文件说明。
// ============================================================================

#include "TurntableControlPanel.h"

#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace aircraft
{
namespace ui
{

namespace
{

const char* const kNoValue = "—";

QString angleText(double deg)
{
    return QString::number(deg, 'f', 4);
}

QLabel* makeValueLabel(QWidget* parent, bool monospace = false)
{
    QLabel* label = new QLabel(QString::fromUtf8(kNoValue), parent);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    if (monospace)
    {
        QFont font;
        font.setFamily(QStringLiteral("monospace"));
        font.setStyleHint(QFont::Monospace);
        label->setFont(font);
    }
    return label;
}

void setLabel(QLabel* label, const QString& text)
{
    label->setText(text.isEmpty() ? QString::fromUtf8(kNoValue) : text);
}

}  // namespace

// ---------------------------------------------------------------------------

TurntableControlPanel::TurntableControlPanel(QWidget* parent)
    : QWidget(parent)
{
    QLabel* title = new QLabel(QStringLiteral("转台操作（受控）"), this);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);

    connection_       = makeValueLabel(this);
    connectionDetail_ = makeValueLabel(this);
    feedback_         = makeValueLabel(this, true);
    sample_           = makeValueLabel(this);
    motion_           = makeValueLabel(this, true);
    fault_            = makeValueLabel(this);
    permit_           = makeValueLabel(this);
    gate_             = makeValueLabel(this);
    commandStage_     = makeValueLabel(this);
    commandDetail_    = makeValueLabel(this);
    stopStage_        = makeValueLabel(this);
    stopDetail_       = makeValueLabel(this);
    action_           = makeValueLabel(this);

    connection_->setObjectName(QStringLiteral("turntableConnectionLabel"));
    connectionDetail_->setObjectName(
        QStringLiteral("turntableConnectionDetailLabel"));
    feedback_->setObjectName(QStringLiteral("turntableFeedbackLabel"));
    sample_->setObjectName(QStringLiteral("turntableSampleLabel"));
    motion_->setObjectName(QStringLiteral("turntableMotionLabel"));
    fault_->setObjectName(QStringLiteral("turntableFaultLabel"));
    permit_->setObjectName(QStringLiteral("turntablePermitLabel"));
    gate_->setObjectName(QStringLiteral("turntableGateLabel"));
    commandStage_->setObjectName(QStringLiteral("turntableCommandStageLabel"));
    commandDetail_->setObjectName(QStringLiteral("turntableCommandDetailLabel"));
    stopStage_->setObjectName(QStringLiteral("turntableStopStageLabel"));
    stopDetail_->setObjectName(QStringLiteral("turntableStopDetailLabel"));
    action_->setObjectName(QStringLiteral("turntableActionLabel"));

    azimuthInput_ = new QDoubleSpinBox(this);
    elevationInput_ = new QDoubleSpinBox(this);
    azimuthInput_->setObjectName(QStringLiteral("turntableAzimuthInput"));
    elevationInput_->setObjectName(QStringLiteral("turntableElevationInput"));
    for (QDoubleSpinBox* box : {azimuthInput_, elevationInput_})
    {
        box->setDecimals(4);
        box->setSingleStep(0.01);
        box->setKeyboardTracking(false);
        box->setSuffix(QStringLiteral(" deg"));
    }

    positionButton_ = new QPushButton(QStringLiteral("定位"), this);
    stopButton_ = new QPushButton(QStringLiteral("停止"), this);
    positionButton_->setObjectName(QStringLiteral("turntablePositionButton"));
    stopButton_->setObjectName(QStringLiteral("turntableStopButton"));
    positionButton_->setEnabled(false);
    stopButton_->setEnabled(false);

    QHBoxLayout* inputRow = new QHBoxLayout;
    inputRow->setContentsMargins(0, 0, 0, 0);
    inputRow->addWidget(new QLabel(QStringLiteral("方位"), this));
    inputRow->addWidget(azimuthInput_, 1);
    inputRow->addWidget(new QLabel(QStringLiteral("俯仰"), this));
    inputRow->addWidget(elevationInput_, 1);

    QHBoxLayout* buttonRow = new QHBoxLayout;
    buttonRow->setContentsMargins(0, 0, 0, 0);
    buttonRow->addWidget(positionButton_);
    buttonRow->addWidget(stopButton_);

    QFormLayout* form = new QFormLayout;
    form->setContentsMargins(8, 4, 8, 8);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignTop);
    form->addRow(QStringLiteral("连接"), connection_);
    form->addRow(QString(), connectionDetail_);
    form->addRow(QStringLiteral("反馈"), feedback_);
    form->addRow(QStringLiteral("样本"), sample_);
    form->addRow(QStringLiteral("运动"), motion_);
    form->addRow(QStringLiteral("故障"), fault_);
    form->addRow(QStringLiteral("许可"), permit_);
    form->addRow(QStringLiteral("门禁"), gate_);
    form->addRow(QStringLiteral("命令阶段"), commandStage_);
    form->addRow(QString(), commandDetail_);
    form->addRow(QStringLiteral("停止阶段"), stopStage_);
    form->addRow(QString(), stopDetail_);
    form->addRow(QStringLiteral("最近操作"), action_);
    form->addRow(QStringLiteral("目标"), inputRow);
    form->addRow(QString(), buttonRow);

    QVBoxLayout* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(title);
    outer->addLayout(form);

    connect(positionButton_, &QPushButton::clicked, this,
            &TurntableControlPanel::emitPositionRequested);
    connect(stopButton_, &QPushButton::clicked, this,
            &TurntableControlPanel::stopRequested);

    setMinimumWidth(260);
}

void TurntableControlPanel::emitPositionRequested()
{
    // 只转发输入；不在 UI 层做“是否允许”的判断。
    emit positionRequested(azimuthInput_->value(), elevationInput_->value());
}

void TurntableControlPanel::setView(const TurntableControlView& view)
{
    if (!view.available)
    {
        clear();
        return;
    }

    setLabel(connection_, QString::fromStdString(view.connectionText));
    setLabel(connectionDetail_, QString::fromStdString(view.connectionDetail));
    connectionDetail_->setToolTip(
        QString::fromStdString(view.connectionDetail));

    if (view.hasSample && view.sampleValid)
    {
        setLabel(feedback_,
                 QStringLiteral("%1 / %2 deg")
                     .arg(angleText(view.azimuth), angleText(view.elevation)));
    }
    else
    {
        setLabel(feedback_, QStringLiteral("无有效双轴读数"));
    }

    const QString sampleText =
        !view.hasSample
            ? QStringLiteral("尚无样本")
            : (!view.sampleValid
                   ? QStringLiteral("无效（不得用于定位/判到位）")
                   : (view.sampleFresh ? QStringLiteral("有效 / 新鲜")
                                       : QStringLiteral("有效 / 已过期")));
    setLabel(sample_, sampleText);
    setLabel(motion_, QString::fromStdString(view.motionText));
    setLabel(fault_, QString::fromStdString(view.faultText));
    setLabel(permit_, QString::fromStdString(view.permitText));
    setLabel(gate_, QString::fromStdString(view.gateText));
    setLabel(commandStage_, QString::fromStdString(view.commandStageText));
    setLabel(commandDetail_, QString::fromStdString(view.commandDetail));
    setLabel(stopStage_, QString::fromStdString(view.stopStageText));
    setLabel(stopDetail_, QString::fromStdString(view.stopDetail));
    setLabel(action_, QString::fromStdString(view.lastActionText));

    // 输入范围只来自配置；它不是运动授权。
    if (view.azimuthMax > view.azimuthMin)
    {
        azimuthInput_->setRange(view.azimuthMin, view.azimuthMax);
        elevationInput_->setRange(view.elevationMin, view.elevationMax);
    }

    // 首次拿到样本时把当前反馈填入输入框，减少手工抄写错误；
    // 之后**不再覆盖**操作者正在输入的值（只由操作者修改）。
    if (!initializedInputs_ && view.hasSample && view.sampleValid)
    {
        azimuthInput_->setValue(view.azimuth);
        elevationInput_->setValue(view.elevation);
        initializedInputs_ = true;
    }

    positionButton_->setEnabled(view.positioningEnabled);
    stopButton_->setEnabled(view.stopEnabled);

    // 故障用红色，其余保持默认；避免整个过程不停变色（与 TurntablePanel 同）。
    fault_->setStyleSheet(view.errorLatched || view.motionUncertain
                              ? QStringLiteral("color: #d03030;")
                              : QString());
}

void TurntableControlPanel::clear()
{
    for (QLabel* label :
         {connection_, connectionDetail_, feedback_, sample_, motion_, fault_,
          permit_, gate_, commandStage_, commandDetail_, stopStage_,
          stopDetail_, action_})
    {
        label->setText(QString::fromUtf8(kNoValue));
    }
    fault_->setStyleSheet(QString());
    positionButton_->setEnabled(false);
    stopButton_->setEnabled(false);
}

}  // namespace ui
}  // namespace aircraft
