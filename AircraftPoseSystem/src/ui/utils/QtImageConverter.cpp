// ============================================================================
//  src/ui/utils/QtImageConverter.cpp
//
//  见 QtImageConverter.h 的文件头说明。本文件只补充**逐条拒绝理由**：
//  9.md §四 的示例只判断了空矩阵与通道数，本实现额外拒绝两类输入，
//  每一类都对应一个"不拒绝就会静默出错"的具体后果。
// ============================================================================

#include "QtImageConverter.h"

namespace aircraft
{
namespace ui
{

QImage QtImageConverter::fromMat(const cv::Mat& image)
{
    // ---- 1 空矩阵 ----
    // 9.md 原有判断。返回空 QImage 而不是构造一个 0×0 的 QImage：
    // 调用方用 isNull() 一次即可同时判掉这两种情形。
    if (image.empty())
        return QImage();

    // ---- 2 非 8 位深度 ----
    // ⚠ 这一条 9.md 的示例**没有**，而它是本项目最可能踩到的一处：
    // `QImage::Format_Grayscale8` / `Format_BGR888` 都假定每通道 8 位。
    // 若传入的是 CV_16U（相机 SDK 的 HDR / 高位深模式常默认 16 位）
    // 或 CV_32F（算法中间结果），QImage 不会报错，只会**按字节重新解释**
    // 同一块内存：16 位灰度图会显示成宽度不变、高度减半、内容交错的
    // 雪花状图像；而 cv::Mat 本身是完好的。这类"图像看着不对但程序不报错"
    // 的现象，排查成本极高（会先怀疑相机、再怀疑标定，最后才怀疑格式）。
    // 显式拒绝，由调用方决定是降位深还是记日志。
    if (image.depth() != CV_8U)
        return QImage();

    // ---- 3 逐行跨度不足 ----
    // QImage 的 bytesPerLine 语义是"相邻两行首字节的距离"，与 cv::Mat::step
    // 完全一致，但 QImage 内部是 int，而 step 是 size_t。此处先做一次
    // 下界检查：step 小于"一行像素的字节数"说明这个 Mat 的元数据已经损坏
    // （正常构造的 Mat 不会如此，但手工拼装 Mat 头或跨 SDK 传递时出现过），
    // 此时 QImage 会越界读。
    const int bytesPerLine = static_cast<int>(image.step);
    const int64_t minBytesPerLine =
        static_cast<int64_t>(image.cols) * static_cast<int64_t>(image.elemSize());
    if (bytesPerLine <= 0 || minBytesPerLine <= 0
        || static_cast<int64_t>(bytesPerLine) < minBytesPerLine)
        return QImage();

    // ---- 4 3 通道：BGR ----
    // 通道顺序按 ENG-09 §5.5 冻结为 **BGR**（ImageFrame::image 的注释）。
    // ⚠ 不能用 Format_RGB888：两者的红蓝分量互换，画面上的表现是
    // "颜色不对"——而本系统的主要被测目标是灰色调的飞机，操作者很可能
    // 根本看不出来，于是这个错误会一直留在交付版本里。
    // 若将来某台相机 SDK 只出 RGB，必须在**采集侧**转换（cv::cvtColor），
    // 而不是在这里加一个布尔开关：否则"同一个 ImageFrame 的通道顺序"
    // 就不再唯一，所有下游（含算法层的灰度转换）都要跟着怀疑。
    if (image.channels() == 3)
    {
        // .copy() 是必需的：见头文件"QImage 构造不拷贝数据"一节。
        return QImage(image.data,
                      image.cols,
                      image.rows,
                      bytesPerLine,
                      QImage::Format_BGR888)
            .copy();
    }

    // ---- 5 1 通道：灰度 ----
    if (image.channels() == 1)
    {
        return QImage(image.data,
                      image.cols,
                      image.rows,
                      bytesPerLine,
                      QImage::Format_Grayscale8)
            .copy();
    }

    // ---- 6 其余通道数（含 4 通道 BGRA）----
    // 4 通道**不**映射到 Format_ARGB32：cv::Mat 的第 4 通道是 A 还是
    // 其它（某些 SDK 用它存深度或置信度）取决于具体后端，猜错会让
    // 整幅图带上一个错误的透明通道。宁可不显示。
    return QImage();
}

}  // namespace ui
}  // namespace aircraft
