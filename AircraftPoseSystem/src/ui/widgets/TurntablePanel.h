#pragma once

// ============================================================================
//  src/ui/widgets/TurntablePanel.h
//
//  依据：ENG-01 §13（ui 模块文件清单**含** TurntablePanel.h）
//        ENG-02 §14 / ENG-04 §14（显示：方位 / 俯仰 / 状态）
//        data/TurntableState.h 的注释："本结构体是
//        ITurntableController::state() 的返回类型，**也是 StatusPanel /
//        TurntablePanel 的显示数据源**"
//        ENG-09 §5.9（TurntableState 字段冻结：azimuth / elevation / motion）
//
//  ⚠ **本文件是 9.md §一 目录清单里缺失的一个类**，不是额外发挥：
//  ENG-01 §13 的 ui 结构图与 ENG-02 §14、ENG-04 §14 都明确列出了
//  TurntablePanel，而 9.md 的一~九节只写了 MainWindow / ImageViewer /
//  PosePanel / StatusPanel 四个，把转台状态放进了 StatusPanel。
//  按 README §6 的冲突规则（冻结文档优先，两份冻结文档冲突时取更具体的一节），
//  这里实现 TurntablePanel 并把转台一行从 StatusPanel 移出
//  （理由见 StatusPanel.h 的偏离说明 (1)）。差异记入 README §6 偏离表。
//
//  ---- 与 9.md 相关章节的关系 ----
//  TurntablePanel 在 9.md 中**没有任何代码示例**，故不存在"偏离 9.md 代码"
//  的问题；本类只实现冻结文档要求的三个显示项，API 直接吃
//  `data::TurntableState` 而不是三个 double —— 理由：
//    · TurntableState 是 ITurntableController::state() 的返回类型，
//      也是 application 层唯一持有的转台状态快照。让面板接收该结构体，
//      调用方就是"把已有对象传下去"，不存在"三个参数填错顺序"的可能
//      （azimuth 与 elevation 都是 double，顺序填反编译器完全不会报错，
//      而现象是"方位显示成俯仰值"——一个只能靠肉眼发现的错误）。
// ============================================================================

#include <QString>
#include <QWidget>

#include "data/TurntableState.h"

class QLabel;

namespace aircraft
{
namespace ui
{

/// 转台状态面板（ENG-04 §14）。
class TurntablePanel : public QWidget
{
public:
    explicit TurntablePanel(QWidget* parent = nullptr);

    /// 显示一次转台状态快照。
    void setState(const data::TurntableState& state);

    /// 回到"未知"状态（尚未取到过转台状态时）。
    void clear();

private:
    QLabel* azimuth_   = nullptr;
    QLabel* elevation_ = nullptr;
    QLabel* motion_    = nullptr;
};

}  // namespace ui
}  // namespace aircraft
