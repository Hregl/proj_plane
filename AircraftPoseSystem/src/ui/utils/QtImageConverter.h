#pragma once

// ============================================================================
//  src/ui/utils/QtImageConverter.h
//
//  依据：9.md §四（QtImageConverter）
//        ENG-01 §13（ui 模块）、ENG-03 §12.8（libui 依赖 Qt + application +
//        preview）
//        ENG-09 §4.1 / §5.5（ImageFrame::image 的冻结格式：1 或 3 通道、8U）
//
//  职责：把 cv::Mat 转成 QImage。**只做格式转换，不持有任何状态**
//  （故全部是静态函数），也**不解释图像内容**——不知道什么是目标、
//  什么是候选通道。
//
//  ⚠ 这是 ui 层唯一允许触碰 cv::Mat 的角落（ENG-01 §18 禁止 ui 访问 SDK，
//    但 OpenCV 不是 SDK：libui 的依赖表里有 Qt 与 OpenCV 两者）。
//    把转换收进一个文件，是为了让"界面拿到的像素是否被正确解释"
//    这个问题只有一个答案来源。
//
//  ---- 一个必须记住的事实：QImage 的构造**不拷贝数据** ----
//
//  `QImage(uchar*, w, h, bytesPerLine, format)` 是**浅包装**：它只保存
//  指针。若直接用这种方式返回，调用方拿到的 QImage 指向的是 cv::Mat 的
//  像素缓冲区；一旦该 Mat 析构（引用计数归零）或被原地改写，QImage 就
//  指向已释放/已变样的内存——表现为**偶发的花屏或崩溃**，且崩溃点
//  通常在 Qt 的绘制代码里，与本文件毫无关系。
//
//  这正是 9.md §四 的示例在构造之后立刻调用 .copy() 的原因，也是本实现
//  唯一保留 9.md 原样的一处。下面的实现改为"先构造后 copy"以外的另一条
//  等价路径（QImage::copy() 同样显式深拷贝），并在 .cpp 中逐条说明
//  拒绝 9.md 哪几种输入。
// ============================================================================

#include <QImage>

#include <opencv2/core.hpp>

namespace aircraft
{
namespace ui
{

/// cv::Mat → QImage 的转换（9.md §四）。
///
/// 支持：CV_8UC1（Format_Grayscale8）、CV_8UC3（Format_BGR888）。
/// 其余（含 4 通道、非 8 位深度、空矩阵）返回**空 QImage**（`isNull()`），
/// 具体拒绝理由见 .cpp 的逐条说明。
///
/// 返回的 QImage **拥有自己的像素**，与入参 cv::Mat 无任何共享，
/// 可在 Mat 被改写或析构后继续安全使用与绘制。
class QtImageConverter
{
public:
    /// @param image 待转换的图像。
    /// @return 深拷贝的 QImage；入参不受支持时返回空 QImage。
    static QImage fromMat(const cv::Mat& image);
};

}  // namespace ui
}  // namespace aircraft
