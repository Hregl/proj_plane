#pragma once

// ============================================================================
//  src/ui/widgets/StatusPanel.h
//
//  依据：9.md §七（StatusPanel：当前状态 / 当前相机 / 转台状态）
//        ENG-01 §13、ENG-02 §14、ENG-04 §14
//        ENG-09 §4.1（MeasurementState / CameraRole 的取值冻结）
//
//  ---- 与 9.md §七 的两处偏离 ----
//
//  (1) **"转台状态"一行迁到 TurntablePanel，本面板不再重复显示。**
//      两份冻结文档的划分是明确的：
//        · ENG-04 §14 / ENG-01 §13：TurntablePanel 显示"方位 / 俯仰 / 状态"，
//          其中"状态"只能是 TurntableState::motion
//          （data/TurntableState.h 的注释亦写明它"是 StatusPanel /
//          TurntablePanel 的显示数据源"）；
//        · 9.md §七 把"转台状态"给了 StatusPanel。
//      按 README §6 的冲突规则（更具体者优先），此处取 ENG-04 §14：
//      转台的运动状态与它的两个角度在同一处显示才有意义——操作者判断
//      "转台到位没有"看的就是"角度 + 状态"这一组，拆到两个面板会把
//      一次判断变成两次扫视。
//
//  (2) **增加 setMessage() 及其一行显示区。**
//      状态机进入 FAILED 时携带 ErrorInfo（ENG-09 §5.27，含错误码与
//      可读消息）。9.md §七 的三个显示项里没有任何一处能承载它，
//      而"FAILED 但不说明原因"是不可接受的：现场唯一能做的事就是
//      看日志文件。本行显示由 application 层格式化好的短消息，
//      本控件不解析错误码（保持"UI 只显示、不判断"的边界）。
//
//  ---- 关于 setStatus(const QString&) 与 setMeasurementState() 的分工 ----
//
//  两者写的是**同一个**"状态"行。这不是冗余，而是 9.md 的原接口
//  （自由文本）与 data 层的冻结枚举（MeasurementState）之间的适配：
//  枚举形式是首选（枚举到文字的转换集中在 ui/utils/UiText.h，
//  与日志逐字一致）；文本形式保留给 9.md 的用法与一次性提示。
//  **先设置者显示，后设置者覆盖**——若调用方同时用两者，
//  以最后一次调用为准，这一点必须知道，否则会出现"界面显示的状态
//  与日志不符"这类难查的问题。
// ============================================================================

#include <QString>
#include <QWidget>

#include "data/CameraRole.h"
#include "data/MeasurementState.h"

class QLabel;

namespace aircraft
{
namespace ui
{

/// 系统状态显示面板（9.md §七）。
class StatusPanel : public QWidget
{
public:
    explicit StatusPanel(QWidget* parent = nullptr);

    /// 按枚举显示测量状态（首选形式，文字见 ui/utils/UiText.h）。
    void setMeasurementState(data::MeasurementState state);

    /// 按自由文本显示状态行（9.md §七 的原接口）。
    void setStatus(const QString& text);

    /// 当前测量通道（SYS-14 选出的焦段）。
    void setCamera(data::CameraRole role);

    /// 附加消息行（错误码 / 提示）。空串表示隐藏该行。
    void setMessage(const QString& text);

private:
    /// 状态行文字着色。
    ///
    /// 着色是**冗余**通道而不是唯一通道：同一变化也会改文字
    /// （FAILED / COMPLETE 与运行中的状态名不同），故色觉障碍者
    /// 不依赖颜色也能读出状态。这条是刻意保持的——若把颜色做成
    /// 唯一区分手段，"红色没看见"就等于"故障没看见"。
    void applyStateColor(data::MeasurementState state);

    QLabel* status_  = nullptr;
    QLabel* camera_  = nullptr;
    QLabel* message_ = nullptr;
};

}  // namespace ui
}  // namespace aircraft
