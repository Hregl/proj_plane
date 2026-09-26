// ============================================================================
//  src/ui/MainWindow.cpp
//
//  见头文件的两点偏离说明。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "MainWindow.h"

#include <QHideEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "preview/PreviewManager.h"
#include "ui/utils/QtImageConverter.h"
#include "ui/utils/UiText.h"

namespace aircraft
{
namespace ui
{

namespace
{

/// 取帧周期，单位 ms。
/// 16 ms ≈ 62.5 Hz。取值理由见 MainWindow.h 的"60 Hz 而非 30 Hz"一节。
const int kPreviewTickMs = 16;

/// 窗口标题（9.md §十四：界面显示 "AircraftPoseSystem V2.1"）。
/// 版本号与根 CMakeLists 的工程版本一致；不在此处再抄一遍版本号的**值**，
/// 因为一旦两处不同步，窗口标题会与交付包版本不符而无人察觉——
/// 010 接入版本宏后，此处应改为引用编译期定义的版本字符串。
const char* const kWindowTitle = "AircraftPoseSystem V2.1";

}  // namespace

// ---------------------------------------------------------------------------

MainWindow::MainWindow(preview::PreviewManager* preview, QWidget* parent)
    : QMainWindow(parent)
    , preview_(preview)
{
    setWindowTitle(QString::fromUtf8(kWindowTitle));

    // ---- 控件 ----
    viewer_         = new ImageViewer(this);
    posePanel_      = new PosePanel(this);
    statusPanel_    = new StatusPanel(this);
    turntablePanel_ = new TurntablePanel(this);

    // ---- 布局 ----
    // 左侧图像 + 右侧三块信息面板。9.md §九 的示例是把三个控件平铺进
    // 一个 QHBoxLayout（三个等宽的横条），那只在窗口很宽时勉强可用：
    // 1280 宽下每个面板分到约 427 px，而图像本身需要尽量大——
    // 预览窗口的用途是看目标，图像越小越无用。
    // 故改为"图像占据全部剩余空间，面板列固定最小宽度"。
    QWidget* side = new QWidget(this);
    QVBoxLayout* sideLayout = new QVBoxLayout(side);
    sideLayout->setContentsMargins(0, 0, 0, 0);
    sideLayout->addWidget(posePanel_);
    sideLayout->addWidget(turntablePanel_);
    sideLayout->addWidget(statusPanel_);
    sideLayout->addStretch(1);

    QWidget* central = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(central);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->addWidget(viewer_, 1);   // 伸缩因子 1：图像吃掉所有多余空间
    layout->addWidget(side, 0);
    setCentralWidget(central);

    resize(1280, 800);   // 9.md §九 的初始尺寸

    // ---- 取帧定时器 ----
    timer_ = new QTimer(this);
    timer_->setInterval(kPreviewTickMs);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onPreviewTick);

    startPreviewTimer();
}

// ---------------------------------------------------------------------------

void MainWindow::setPreviewManager(preview::PreviewManager* preview)
{
    if (preview_ == preview)
        return;

    preview_ = preview;

    // 换了显示源：按 PreviewManager::displayGeneration() 的语义，
    // 上一路的最后一帧不该再停留在画面上（见 ImageViewer.h 的偏离说明 (2)）。
    // 此处**不**去比较新旧 manager 的代次，直接清屏：
    // 换的是整个数据来源，清屏总是正确的。
    viewer_->clear();
    viewer_->setOverlayText(QString());

    startPreviewTimer();
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    // 只处理两个键，其余一律交给基类 —— 否则会把 Tab 焦点切换、
    // 方向键等在 QMainWindow 内部有含义的按键吞掉。
    switch (event->key())
    {
    case Qt::Key_Space:
        emit startRequested();
        event->accept();
        return;
    case Qt::Key_Escape:
        emit stopRequested();
        event->accept();
        return;
    default:
        break;
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::setMeasurementView(const MeasurementView& view)
{
    // 无有效信息：四个面板一律回到占位态。
    // 不保留上一次的内容 —— "上一次的结果"与"这一次还没测"在界面上
    // 必须可区分，否则操作者会把上一架的姿态当成这一架的读数。
    if (!view.valid)
    {
        posePanel_->clear();
        turntablePanel_->clear();
        statusPanel_->setMeasurementState(data::MeasurementState::IDLE);
        statusPanel_->setMessage(QString());
        return;
    }

    statusPanel_->setMeasurementState(view.state);
    statusPanel_->setCamera(view.selectedCamera);

    // 状态说明：降级优先于普通 message。
    // SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求降级"必须可见"，而它一旦被普通 message 覆盖，
    // 就成了一条被淹没的提示 —— 故降级时把它并进同一行文字的开头。
    if (view.degraded)
    {
        statusPanel_->setMessage(
            QStringLiteral("[降级] 可用相机 %1 / 3  ·  %2")
                .arg(QString::number(view.availableCameras), view.message));
    }
    else
    {
        statusPanel_->setMessage(view.message);
    }

    if (view.hasPose)
    {
        posePanel_->updatePose(view.yaw, view.pitch, view.roll);
    }
    else
    {
        posePanel_->clear();
    }

    if (view.hasTurntable)
    {
        turntablePanel_->setState(view.turntable);
    }
    else
    {
        turntablePanel_->clear();
    }
}

void MainWindow::startPreviewTimer()
{
    if (!timer_)
        return;

    // 无 manager 或窗口不可见时不取帧。
    // 窗口最小化后 Qt 仍会派发定时器事件，而此时的缩放绘制是纯浪费
    // （2448×2048 的降采样每帧数毫秒，见 ImageViewer.cpp）。
    if (preview_ == nullptr || !isVisible())
    {
        timer_->stop();
        return;
    }

    if (!timer_->isActive())
        timer_->start();
}

void MainWindow::stopPreviewTimer()
{
    if (timer_)
        timer_->stop();
}

QString MainWindow::buildOverlayText(const data::PreviewFrame& frame) const
{
    const data::ImageFrame& image = frame.frame;

    // 焦段名：优先用 PreviewManager 给出的相机标识（来自 OpticalRig，
    // 即配置文件里的真实 id，如 "cam25"），拿不到时退回枚举名。
    // 两者都缺时仍显示枚举名——界面永远不会出现空白标签。
    QString name = preview_ != nullptr
                       ? QString::fromStdString(preview_->cameraId(image.role))
                       : roleText(image.role);
    if (name.isEmpty())
        name = roleText(image.role);

    QString text = name;

    // 分辨率：操作者据此判断"是否拿到了预期的那台相机的图"。
    // 三台相机分辨率不同（SYS-06 §5），一张来自错误通道的图往往
    // 只在这一项上露出破绽。
    if (!image.image.empty())
    {
        text += QStringLiteral("  %1x%2")
                    .arg(image.image.cols)
                    .arg(image.image.rows);
    }

    // 预览时延（PreviewFrame.h 冻结的算法：displayTimestamp - timestampNs）。
    // 单调时钟保证后者不大于前者（displayTimestamp 是提交时刻，
    // 必然晚于采集时刻），但这条推理依赖"两处都调用
    // data::monotonicNowNs()"——若将来有谁用别处取的时刻填了
    // displayTimestamp，这里会算出天文数字而不报错，故显式保底。
    if (frame.displayTimestamp >= image.timestampNs)
    {
        const double latencyMs =
            static_cast<double>(frame.displayTimestamp - image.timestampNs) / 1e6;
        text += QStringLiteral("  %1 ms").arg(latencyMs, 0, 'f', 1);
    }

    return text;
}

void MainWindow::onPreviewTick()
{
    if (preview_ == nullptr)
    {
        // 没有预览源时定时器本不该在跑（startPreviewTimer 已拦截），
        // 但 window 隐藏/显示会改变 isVisible()，故此处再守一道：
        // 少了它，一旦将来有人直接 start() 就会每秒空转 60 次。
        stopPreviewTimer();
        return;
    }

    // 窗口重新可见时恢复取帧（startPreviewTimer 在 hide 之后被调用过，
    // 那时它把定时器停了；这里负责把它拉回来）。
    if (!timer_->isActive())
    {
        startPreviewTimer();
        if (!timer_->isActive())
            return;
    }

    // 显示源切换：清掉上一路的残留画面（ImageViewer.h 偏离说明 (2)）。
    const quint64 generation = static_cast<quint64>(preview_->displayGeneration());
    if (generation != lastGeneration_)
    {
        lastGeneration_ = generation;
        viewer_->clear();
    }

    data::PreviewFrame frame;

    // getFrame 仅在**有新帧**时返回 true（PreviewManager 的注释：
    // 界面 60 Hz 调用它、只在新帧到来时重绘，否则纯属浪费）。
    if (!preview_->getFrame(frame))
        return;

    // 转换失败与"还没有图像"都得到空 QImage → ImageViewer 显示占位文字。
    // 两者对操作者的含义相同：此刻没有可信的画面（见 ImageViewer.h 偏离 (4)）。
    viewer_->setImage(QtImageConverter::fromMat(frame.frame.image));
    viewer_->setOverlayText(buildOverlayText(frame));
}

// ---------------------------------------------------------------------------

void MainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    // 重新可见 → 恢复取帧。
    startPreviewTimer();
}

void MainWindow::hideEvent(QHideEvent* event)
{
    QMainWindow::hideEvent(event);
    // 不可见 → 停止取帧（理由见 startPreviewTimer）。
    stopPreviewTimer();
}

}  // namespace ui
}  // namespace aircraft
