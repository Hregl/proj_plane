#pragma once

// ============================================================================
//  src/ui/widgets/ImageViewer.h
//
//  依据：9.md §五（ImageViewer）、ENG-01 §13、ENG-02 §14、ENG-04 §14
//        （三者对该类的全部描述均为"负责图像显示"）
//
//  职责：把一帧 QImage 画出来。**不含任何图像处理、不取帧、不判新旧**——
//  取帧由 MainWindow 按 PreviewManager 的单消费者约定做（见
//  PreviewManager.h 的 getFrame 说明），格式转换由 QtImageConverter 做。
//
//  ---- 与 9.md §五 的四处偏离，逐条给出理由 ----
//
//  (1) **不用 QLabel，改为自绘 paintEvent。**
//      9.md 的示例是 `QLabel* label_` + `label_->setPixmap(...)`。改用自绘
//      的理由有两条，都是"用 QLabel 会出错"而不是风格偏好：
//        · 缩放：相机出图远大于主窗口，QLabel 显示原尺寸 pixmap 时，
//          要么把窗口撑大、要么把图像裁掉一角，现象是"界面一片灰，
//          图像跑到窗口外面去了"；而 QLabel 的缩放（setScaledContents）
//          是不等比拉伸，会把图像拉变形——对测量系统是不可接受的显示
//          失真（操作者会据此判断目标是否变形）。自绘时用
//          `painter.drawImage(targetRect, image)` 一行即得等比缩放。
//        · 线程与失效：QLabel 只接受 QPixmap（GUI 线程限定，且需在
//          缩放尺寸变化时重建），而 QImage 是线程安全且与尺寸无关的。
//          自绘只持 QImage，缩放发生在绘制那一刻，不存在"缓存的缩放图
//          与当前尺寸不一致"这一类问题。
//
//  (2) **增加 clear() 与"无图像"占位文字。** 这不只是美观问题：
//      PreviewManager::displayGeneration() 的语义是"显示源一变，上一路的
//      最后一帧就不该再停留在画面上"（其头文件原话）。界面若没有清空
//      能力，就会出现"新焦段的标签配旧焦段的图像"，而这种错配看起来
//      完全像是**测量结果不对**，会把排障方向引到算法上去。
//      占位文字同时覆盖另一种情形：010 首次运行时尚无任何相机出图，
//      一块全黑的窗口无法与"程序卡死"区分。
//
//  (3) **增加 setOverlayText()**：左上角叠一行信息（焦段 / 分辨率 /
//      预览时延）。SYS-07 §15 与 PreviewFrame.h 都把
//      `displayTimestamp - frame.timestampNs` 定为"预览不得阻塞算法"
//      这条约束的**可观测量**；不显示出来，就只能在事后翻日志。
//      文字由调用方（MainWindow）拼好，本控件不求值、也不解释其含义。
//
//  (4) 空 QImage 先经 QtImageConverter 过滤（其 .cpp 逐条说明了拒绝理由）。
//      本控件对"转换失败"与"还没有图像"给出同一句占位文字：
//      两者对操作者的含义相同——**此刻没有可信的画面**。
//      把二者分开显示会让界面多出一个只有开发者看得懂的状态。
// ============================================================================

#include <QImage>
#include <QString>
#include <QWidget>

class QPainter;

namespace aircraft
{
namespace ui
{

/// 图像显示控件（9.md §五）。
class ImageViewer : public QWidget
{
public:
    explicit ImageViewer(QWidget* parent = nullptr);

    /// 设置待显示图像（9.md §五 的原接口，保留）。
    ///
    /// 传入空 QImage 等价于 clear()：QtImageConverter 的失败返回值
    /// 正好是空 QImage，故调用方无需分情况处理。
    void setImage(const QImage& image);

    /// 左上角信息行。空串表示不显示。
    void setOverlayText(const QString& text);

    /// 清空图像，回到"无图像"占位状态。覆盖层文字**保留**——
    /// 显示源刚切换时，"切到了哪个焦段"正是操作者最需要看到的信息。
    void clear();

    /// 当前是否有可显示图像。
    bool hasImage() const { return !image_.isNull(); }

    /// 供主窗口给出合理的初始尺寸（4:3，符合三台相机的出图比例）。
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    /// 等比缩放的目标矩形（在控件内居中）。缩放比例取宽高比例的
    /// **较小**者，即"完整显示、允许留黑边"，绝不裁切：
    /// 裁掉的那部分可能正是目标所在位置，而画面上的目标通常在机头/机尾，
    /// 恰好是最先被裁掉的地方。
    QRect scaledTargetRect() const;

    void drawPlaceholder(QPainter& painter);

    QImage  image_;
    QString overlay_;
};

}  // namespace ui
}  // namespace aircraft
