// ============================================================================
//  src/ui/widgets/ImageViewer.cpp
//
//  见头文件的四点偏离说明。本文件只补充绘制相关的两处取舍：
//    · 缩放比例的"不放大"约定；
//    · 平滑缩放的代价与为何仍然开启。
// ============================================================================

#include "ImageViewer.h"

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QPaintEvent>
#include <QRect>

namespace aircraft
{
namespace ui
{

namespace
{

/// 无图像时的占位文字。选中文而非 "No image"：本系统的操作者与现场
/// 排障人员都是中文使用者，而界面其余文字（状态名、焦段名）是
/// 与日志逐字对应的英文枚举名，必须保留英文——两者的区分是有意的。
const char* const kPlaceholderText = "无图像";

/// 背景色：不纯黑。纯黑会让"控件没画出来"与"图像是黑的"无法区分，
/// 而夜间低照度下预览画面本身就接近全黑。
const QColor kBackground(28, 28, 30);

/// 覆盖层底衬透明度（0~255）。0.55 的取值：在本系统常见的
/// 天空背景（接近纯白）与暗色机身上都能保证文字可读。
const int kOverlayAlpha = 140;

}  // namespace

// ---------------------------------------------------------------------------

ImageViewer::ImageViewer(QWidget* parent)
    : QWidget(parent)
{
    // 由本控件自己擦除背景（paintEvent 中整幅填充），
    // 故告诉 Qt 不要重复绘制系统背景。
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setMinimumSize(320, 240);
}

void ImageViewer::setImage(const QImage& image)
{
    // ⚠ 不在此处做任何缩放或 QPixmap 转换：只保存原图并请求重绘。
    // 缩放发生在 paintEvent 内、针对当时的控件尺寸，因而不会出现
    // "缓存的缩放结果与当前窗口大小不一致"（开头的偏离说明 (1)）。
    image_ = image;
    update();
}

void ImageViewer::setOverlayText(const QString& text)
{
    if (overlay_ == text)
        return;
    overlay_ = text;
    update();
}

void ImageViewer::clear()
{
    if (image_.isNull())
        return;
    image_ = QImage();
    update();
}

QSize ImageViewer::sizeHint() const
{
    // 4:3 —— 三台相机的出图比例（SYS-06 §5 的分辨率表）。
    // 给 4:3 而不是让布局随便定，是为了让图像在初始窗口尺寸下
    // 恰好铺满而不留黑边，省得每次启动都要手动拉窗口。
    return QSize(640, 480);
}

// ---------------------------------------------------------------------------

QRect ImageViewer::scaledTargetRect() const
{
    if (image_.isNull() || width() <= 0 || height() <= 0)
        return QRect();

    const double sx = static_cast<double>(width())  / image_.width();
    const double sy = static_cast<double>(height()) / image_.height();

    // min 而不是 max：完整显示、允许留黑边（见头文件的说明）。
    double s = sx < sy ? sx : sy;

    // ⚠ 不放大：画面大于控件时按 s 缩小；小于控件时保持 1:1 居中。
    // 放大不会带来任何信息（原始像素就那么多），只会把插值出来的
    // 模糊当成细节——对一个要据以判断"目标是否对焦清楚"的预览窗口，
    // 这是会误导人的。操作者若要看细节，应切到长焦通道。
    if (s > 1.0)
        s = 1.0;

    const int w = static_cast<int>(image_.width()  * s + 0.5);
    const int h = static_cast<int>(image_.height() * s + 0.5);

    return QRect((width() - w) / 2, (height() - h) / 2, w, h);
}

void ImageViewer::drawPlaceholder(QPainter& painter)
{
    painter.setPen(QColor(170, 170, 175));

    QFont font = painter.font();
    font.setPointSizeF(font.pointSizeF() * 1.4);
    painter.setFont(font);

    painter.drawText(rect(), Qt::AlignCenter, QString::fromUtf8(kPlaceholderText));
}

void ImageViewer::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);

    // 整幅填充：WA_OpaquePaintEvent 声明了不留背景，此处必须真的填满，
    // 否则缩放后留出的黑边区域是未初始化内容（花屏）。
    painter.fillRect(rect(), kBackground);

    if (image_.isNull())
    {
        drawPlaceholder(painter);
    }
    else
    {
        const QRect target = scaledTargetRect();
        if (!target.isEmpty())
        {
            // 平滑缩放：2448×2048 → 约 800×600 的降采样若用最近邻，
            // 飞机蒙皮上的铆钉与刻线会变成闪烁的麻点，操作者无法据此
            // 判断是否合焦（这正是预览窗口的核心用途）。
            // 代价：降采样每帧约数毫秒，占用 GUI 线程——这是本控件
            // 唯一的可观开销，若将来 010 实测 GUI 线程吃紧，
            // 应先降预览帧率（PreviewConfig），而不是关掉平滑。
            painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
            painter.drawImage(target, image_);
        }
    }

    // ---- 覆盖层（偏离说明 (3)）----
    if (overlay_.isEmpty())
        return;

    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);

    const QFontMetrics fm(painter.font());
    const QRect textRect = fm.boundingRect(overlay_);
    const int pad = 4;
    const QRect box(8, 8,
                    textRect.width()  + pad * 2,
                    textRect.height() + pad * 2);

    QColor backing(0, 0, 0, kOverlayAlpha);
    painter.setPen(Qt::NoPen);
    painter.setBrush(backing);
    painter.drawRoundedRect(box, 3, 3);

    painter.setPen(QColor(240, 240, 240));
    painter.drawText(box, Qt::AlignCenter, overlay_);
}

}  // namespace ui
}  // namespace aircraft
