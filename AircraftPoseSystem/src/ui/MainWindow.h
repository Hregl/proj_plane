#pragma once

// ============================================================================
//  src/ui/MainWindow.h
//
//  依据：9.md §八 / §九（MainWindow）、§十三（主程序依赖关系，冻结）
//        ENG-01 §13、ENG-02 §14、ENG-04 §14（"负责用户交互"）
//        ENG-03 §12.8（libui 依赖 Qt + application + preview）
//        ENG-01 §18 与 ENG-04 §15（**UI 不得访问设备 / SDK**）
//        ENG-05 §15 / SYS-09 §15（不得继承 QThread）
//        preview/PreviewManager.h（getFrame 的单消费者与 60 Hz 约定）
//
//  职责：把四个显示控件装配成主界面，并按 60 Hz 从 PreviewManager 取帧
//  转成 QImage 交给 ImageViewer。
//
//  ---- 与 9.md §八 的两处偏离 ----
//
//  (1) **构造参数多了 `preview::PreviewManager*`（默认 nullptr）。**
//      9.md 的 MainWindow 无参，然后由 main.cpp 直接 `MainWindow window;`
//      —— 那样窗口就拿不到任何图像（9.md 的示例代码里根本没有取帧的地方，
//      §十四 展示的却是一个"能显示图像"的界面，这中间缺的正是这一处连接）。
//
//      为什么用**指针入参**而不是让 MainWindow 自己 new 一个
//      PreviewManager：ENG-02 §15/§16、ENG-04 §15 把"创建对象"这件事
//      明确禁止在 UI 层做（"UI 创建设备"是冻结的禁止项）。窗口只**使用**
//      注入进来的 manager，生命周期由 009 的 ApplicationContext 持有。
//      默认 nullptr 使 `MainWindow window;`（9.md 与 10.md §九 都这么写）
//      依然编译通过，只是没有图像可显示——这也正是 008 阶段应有的样子。
//
//  (2) **增加了四个控件访问器。** 009/010 的职责是"把控制器的输出接到
//      界面上"，而它必须能拿到具体面板才能接。若不给访问器，MainWindow
//      就得替应用层去决定"哪个信号接哪个面板"——那等于把 009 的接线工作
//      搬进 ui 层，而 ui 层一旦开始解释测量语义，就违反了
//      ENG-01 §13"UI 不负责相机控制 / 算法计算 / 转台控制"。
//      访问器只是"把零件交出去"，不含任何判断。
//
//  ---- 取帧这一处必须说清楚的两件事 ----
//
//  · **单消费者**：PreviewManager::getFrame 记录"上次已交付的序号"，
//    从两个线程调用会让先到者把帧吃掉（其头文件的原话）。故本类的
//    定时器**只在 GUI 线程**跑，且整个工程只有本类调用 getFrame。
//  · **60 Hz 而非 30 Hz**：PreviewManager 的注释写明该接口"为配合 Qt 的
//    定时重绘"而设计，界面以 60 Hz 调用、只有返回 true 才重绘。
//    16 ms 对应约 62.5 Hz，且 getFrame 在无新帧时是纯计数比较，开销可忽略。
// ============================================================================

#include <QMainWindow>
#include <QtGlobal>

#include "data/PreviewFrame.h"
#include "ui/MeasurementView.h"
#include "widgets/ImageViewer.h"
#include "widgets/PosePanel.h"
#include "widgets/StatusPanel.h"
#include "widgets/TurntablePanel.h"

class QTimer;
class QShowEvent;
class QHideEvent;
class QKeyEvent;

namespace aircraft
{

namespace preview
{
class PreviewManager;
}

namespace ui
{

/// 主界面（9.md §八、ENG-04 §14）。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /// @param preview 预览管理器，**可为 nullptr**（008 阶段尚无采集链，
    ///                009 的 SystemInitializer 创建后注入）。
    ///                本类**不持有其所有权**，只保存裸指针。
    explicit MainWindow(preview::PreviewManager* preview = nullptr,
                        QWidget* parent = nullptr);

    /// 注入 / 更换预览管理器（供 009 在装配完成后再接）。
    /// 传入 nullptr 即停止取帧。重复注入同一指针无副作用。
    void setPreviewManager(preview::PreviewManager* preview);

    preview::PreviewManager* previewManager() const { return preview_; }

    /// 显示一份测量信息快照（009 的 SystemInitializer 每拍构造并调用）。
    ///
    /// 入参是**值语义的 POD**（见 MeasurementView.h 的说明）：本类不认识
    /// MeasurementController，也不认识任何设备类型。它只负责把 struct 里的
    /// 数字转成面板上的文字，不含任何"什么时候该显示什么"的判断 ——
    /// 那是 app 的职责。
    ///
    /// 重复传入相同内容无副作用（各面板都是 setText，Qt 自身会短路重绘）。
    void setMeasurementView(const MeasurementView& view);

    // ---- 控件访问器（供 009/010 接线）----
    ImageViewer*    viewer()         { return viewer_; }
    PosePanel*      posePanel()      { return posePanel_; }
    StatusPanel*    statusPanel()    { return statusPanel_; }
    TurntablePanel* turntablePanel() { return turntablePanel_; }

signals:
    /// 操作者请求开始 / 停止一次测量。
    ///
    /// 用信号而不是回调函数指针：本类**不知道**谁在听，也不需要知道。
    /// 发出"用户按了开始"这件事属于界面，决定"开始意味着什么"属于 app
    /// （ENG-01 §13：UI 不负责相机控制 / 算法计算 / 转台控制）。
    ///
    /// 为什么现在才有：008 阶段的界面只有显示，没有任何输入；
    /// 009 装配出 MeasurementController 之后，"谁去调 startMeasurement()"
    /// 才成为一个必须回答的问题 —— 而 controller 绝不能出现在 ui 层
    /// （见 MeasurementView.h 的说明）。信号是这两者之间唯一不引入
    /// 新依赖的连接方式。
    void startRequested();
    void stopRequested();

protected:
    /// 窗口可见性变化时启停取帧（理由见 startPreviewTimer 的注释）。
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

    /// 快捷键：空格 = 开始测量，Esc = 停止。
    ///
    /// 用键盘而不是加按钮：9.md §十四 冻结了本界面的布局（图像 + 三块面板），
    /// 加一个工具栏是**改界面**，而 009 的职责是装配，不是改 UI。
    /// 快捷键是纯增量、不动布局，且足以让 M1 检查单里的"状态机运行"
    /// 可以被人工触发验证（11.md §五）。M2 的正式操作面板另议。
    void keyPressEvent(QKeyEvent* event) override;

private:
    /// 定时取帧（GUI 线程）。见头文件"取帧这一处必须说清楚的两件事"。
    void onPreviewTick();

    /// 组装覆盖层文字：焦段 / 分辨率 / 预览时延。
    ///
    /// 时延的计算依据是 PreviewFrame.h 的字段注释
    /// （displayTimestamp - frame.timestampNs）；本函数只做格式化，
    /// 不参与任何判断，也不用时延决定"是否丢帧"——
    /// 丢帧与否由 PreviewManager 的队列策略决定，界面无权改变。
    QString buildOverlayText(const data::PreviewFrame& frame) const;

    void startPreviewTimer();
    void stopPreviewTimer();

    preview::PreviewManager* preview_ = nullptr;

    QTimer*  timer_ = nullptr;
    quint64  lastGeneration_ = 0;

    ImageViewer*    viewer_         = nullptr;
    PosePanel*      posePanel_      = nullptr;
    StatusPanel*    statusPanel_    = nullptr;
    TurntablePanel* turntablePanel_ = nullptr;
};

}  // namespace ui
}  // namespace aircraft
