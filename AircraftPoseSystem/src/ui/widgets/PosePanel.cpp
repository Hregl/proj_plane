// ============================================================================
//  src/ui/widgets/PosePanel.cpp
//
//  见头文件的两点偏离说明。
// ============================================================================

#include "PosePanel.h"

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

/// 无结果时的占位符（头文件偏离说明 (1)）。
/// 用 "—"（U+2014）而不是空串：空串会让标签看起来像"这个字段不存在"，
/// 而 "—" 明确表达"字段存在、此刻没有值"。
const char* const kNoValue = "—";

/// 生成一个数值标签：右对齐、等宽数字。
///
/// 等宽数字（QFont::setStyleHint + 特点选择）是为了避免数字变化时
/// 标签宽度抖动——本面板每收到一次解算结果就重排一次文本，
/// 若字宽不等，同一行的 "9.999°" 与 "10.000°" 会让整行左右跳动。
QLabel* makeValueLabel(QWidget* parent)
{
    QLabel* label = new QLabel(QString::fromUtf8(kNoValue), parent);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QFont font = label->font();
    // 等宽族名由 Qt 按平台解析；国产 UOS 上通常落到 "Noto Sans Mono"
    // 或 "DejaVu Sans Mono"。即便解析不到等宽字体，也只是退化为抖动，
    // 不影响数值正确性，故不在此处做字体存在性检查。
    font.setFamily(QStringLiteral("monospace"));
    font.setStyleHint(QFont::Monospace);
    label->setFont(font);

    return label;
}

}  // namespace

// ---------------------------------------------------------------------------

PosePanel::PosePanel(QWidget* parent)
    : QWidget(parent)
{
    yaw_   = makeValueLabel(this);
    pitch_ = makeValueLabel(this);
    roll_  = makeValueLabel(this);

    // 标题行不算在 QFormLayout 里：QFormLayout 的语义是"名称—值"配对，
    // 把标题塞成一行会与其余三行的对齐基准混在一起。
    QLabel* title = new QLabel(QStringLiteral("姿态"), this);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);

    QFormLayout* form = new QFormLayout;
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    // Yaw 的标签带单位提示：它是唯一有角分级指标的通道（头文件偏离 (2)）。
    form->addRow(QStringLiteral("Yaw (deg / ′)"), yaw_);
    form->addRow(QStringLiteral("Pitch (deg)"),   pitch_);
    form->addRow(QStringLiteral("Roll (deg)"),    roll_);

    QVBoxLayout* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(title);
    outer->addLayout(form);
    outer->addStretch(1);

    setMinimumWidth(220);
}

void PosePanel::updatePose(double yaw, double pitch, double roll)
{
    yaw_->setText(angleTextWithArcmin(yaw));
    pitch_->setText(angleText(pitch));
    roll_->setText(angleText(roll));
}

void PosePanel::clear()
{
    // 恢复到占位符而不是 0.000°：见头文件偏离说明 (1)。
    yaw_->setText(QString::fromUtf8(kNoValue));
    pitch_->setText(QString::fromUtf8(kNoValue));
    roll_->setText(QString::fromUtf8(kNoValue));
}

}  // namespace ui
}  // namespace aircraft
