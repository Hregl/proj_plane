// ============================================================================
//  src/ui/widgets/StatusPanel.cpp
//
//  见头文件的两点偏离说明。
// ============================================================================

#include "StatusPanel.h"

#include <QFont>
#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>

#include "ui/utils/UiText.h"

namespace aircraft
{
namespace ui
{

namespace
{

/// 未设置状态时的占位符。与 PosePanel 同一理由：
/// MeasurementState 的默认值是 IDLE，直接显示 "IDLE" 会让
/// "还没启动"与"启动了但空闲"看起来一样。
const char* const kNoValue = "—";

}  // namespace

// ---------------------------------------------------------------------------

StatusPanel::StatusPanel(QWidget* parent)
    : QWidget(parent)
{
    status_  = new QLabel(QString::fromUtf8(kNoValue), this);
    camera_  = new QLabel(QString::fromUtf8(kNoValue), this);
    message_ = new QLabel(QString(), this);

    // 消息行：可换行、左对齐，长错误消息不撑破窗口。
    message_->setWordWrap(true);
    message_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    message_->setVisible(false);

    QFont valueFont;
    valueFont.setFamily(QStringLiteral("monospace"));
    valueFont.setStyleHint(QFont::Monospace);
    valueFont.setBold(true);
    status_->setFont(valueFont);
    camera_->setFont(valueFont);

    QLabel* title = new QLabel(QStringLiteral("系统状态"), this);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);

    QFormLayout* form = new QFormLayout;
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->addRow(QStringLiteral("状态"), status_);
    form->addRow(QStringLiteral("相机"), camera_);

    QVBoxLayout* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(title);
    outer->addLayout(form);
    outer->addWidget(message_);
    outer->addStretch(1);

    setMinimumWidth(200);
}

// ---------------------------------------------------------------------------

void StatusPanel::applyStateColor(data::MeasurementState state)
{
    // 三个终止/异常态给颜色，运行中的状态保持默认前景色：
    // 若每个状态一种颜色，颜色就不再是"需要注意"的信号。
    QString css;
    switch (state)
    {
    case data::MeasurementState::FAILED:
        css = QStringLiteral("color: #d03030;");  // 红：故障
        break;
    case data::MeasurementState::COMPLETE:
        css = QStringLiteral("color: #1a8a3a;");  // 绿：成功
        break;
    case data::MeasurementState::IDLE:
        css = QStringLiteral("color: #808080;");  // 灰：未运行
        break;
    default:
        css = QStringLiteral("color: #c8a000;");  // 琥珀：进行中
        break;
    }
    status_->setStyleSheet(css);
}

void StatusPanel::setMeasurementState(data::MeasurementState state)
{
    status_->setText(stateText(state));
    applyStateColor(state);
}

void StatusPanel::setStatus(const QString& text)
{
    // 自由文本形式（9.md 原接口）：不改变颜色。
    // 保留上一次的颜色是对的——文本形式没有携带状态语义，
    // 凭猜上色比不上色更糟（可能把一次故障提示显示成琥珀色）。
    status_->setText(text.isEmpty() ? QString::fromUtf8(kNoValue) : text);
}

void StatusPanel::setCamera(data::CameraRole role)
{
    camera_->setText(roleText(role));
}

void StatusPanel::setMessage(const QString& text)
{
    message_->setText(text);
    message_->setVisible(!text.isEmpty());
}

}  // namespace ui
}  // namespace aircraft
