// ============================================================================
//  src/ui/widgets/TurntablePanel.cpp
//
//  见头文件说明。
// ============================================================================

#include "TurntablePanel.h"

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

/// 未取到状态时的占位符（与 PosePanel / StatusPanel 同一约定）。
const char* const kNoValue = "—";

}  // namespace

// ---------------------------------------------------------------------------

TurntablePanel::TurntablePanel(QWidget* parent)
    : QWidget(parent)
{
    azimuth_   = new QLabel(QString::fromUtf8(kNoValue), this);
    elevation_ = new QLabel(QString::fromUtf8(kNoValue), this);
    motion_    = new QLabel(QString::fromUtf8(kNoValue), this);

    QFont valueFont;
    valueFont.setFamily(QStringLiteral("monospace"));
    valueFont.setStyleHint(QFont::Monospace);
    valueFont.setBold(true);
    azimuth_->setFont(valueFont);
    elevation_->setFont(valueFont);
    motion_->setFont(valueFont);

    azimuth_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    elevation_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    motion_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QLabel* title = new QLabel(QStringLiteral("转台"), this);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);

    QFormLayout* form = new QFormLayout;
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->addRow(QStringLiteral("方位 (deg)"), azimuth_);
    form->addRow(QStringLiteral("俯仰 (deg)"), elevation_);
    form->addRow(QStringLiteral("状态"),       motion_);

    QVBoxLayout* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(title);
    outer->addLayout(form);
    outer->addStretch(1);

    setMinimumWidth(200);
}

void TurntablePanel::setState(const data::TurntableState& state)
{
    azimuth_->setText(angleText(state.azimuth));
    elevation_->setText(angleText(state.elevation));
    motion_->setText(motionText(state.motion));

    // 只有 ERROR 着色：ALIGN / STABILIZE 两个状态是在**等待** motion 变成
    // STABLE，若给 MOVING / STABLE 也上色，界面会在整个对准过程中不停变色，
    // 真正的故障反而不显眼。见 StatusPanel::applyStateColor 的同一考虑。
    if (state.motion == data::TurntableMotionState::ERROR)
        motion_->setStyleSheet(QStringLiteral("color: #d03030;"));
    else
        motion_->setStyleSheet(QString());
}

void TurntablePanel::clear()
{
    azimuth_->setText(QString::fromUtf8(kNoValue));
    elevation_->setText(QString::fromUtf8(kNoValue));
    motion_->setText(QString::fromUtf8(kNoValue));
    motion_->setStyleSheet(QString());
}

}  // namespace ui
}  // namespace aircraft
