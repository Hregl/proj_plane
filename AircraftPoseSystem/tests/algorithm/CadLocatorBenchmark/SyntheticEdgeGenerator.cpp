// ============================================================================
//  tests/algorithm/CadLocatorBenchmark/SyntheticEdgeGenerator.cpp
//  见同名 .h 的文件头（依据 / 为什么不能沿用 checkerCornerImage / 生成三步）。
// ============================================================================

#include "SyntheticEdgeGenerator.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace cadbench
{
namespace
{

/// 棋盘格在**旋转后坐标系**里的周期，单位 pixel。
///
/// ⚠ **这个值必须远大于定位器的窗口半径**，否则整个基准测不出任何东西。
///   实测证据（本文件写就时跑过）：取 40 pixel 时四个结构点全部
///   `rejectedByResidual = 4`、`maxResidualPx = 39.999` —— 恰好等于周期本身。
///   原因是分组依据是**梯度方向**（倍角聚类），而同一族的相邻两条边界
///   （u = 0 与 u = P）**梯度方向相同**，必然被并成一组；一组里混着相距
///   一个周期的两条平行边缘，到拟合直线的距离的中位数就是 P/2 量级，
///   冻结的 0.3 pixel 残差判据永远不可能满足。**失败与定位器实现无关。**
///
///   同样的机理在现行夹具的注释里被单独记录过一次（"一条亮线有两条边界…
///   必然被并成一组"）—— 这里只是换了个触发方式：不是亮线的两条边，
///   而是棋盘格的**相邻两条同族边界**。
///
/// 取 1000：远大于本基准使用的任何窗口（窗口按投影点间距自适应，
/// 而角点间距 ≤ 200 pixel）。代价是接缝（"最近角点胜"的中垂线）离角点
/// 更远 —— 那正是我们要的，接缝必须落在窗口之外。
constexpr double kCheckerPitchPx = 1000.0;

/// 向下取整的整数除法（对负数也给出数学意义上的 floor）。
/// 棋盘格的象限判定必须用它：直接写 `u / P` 时，u<0 会按 C++ 的截断除法
/// 得到 0 而不是 −1，于是角点**左侧**的两个象限与右侧同相位 ——
/// 四个象限退化成两个，**角点消失**。
long floorDiv(double a, double b)
{
    return static_cast<long>(std::floor(a / b));
}

/// 设计说明 §1.1 的硬性前提检查。
///
/// 不在生成器里"默认帮调用方补上"超采样：那样调用方会以为 `ss = 1` 也能用，
/// 而 `ss = 1` 下**真值不可表示**（见 generateDegenerateSs1Image 的说明），
/// 基准会稳定输出看起来正常却无意义的数。宁可当场抛。
void assertSupersampling(int ss)
{
    if (ss < 8)
    {
        throw std::invalid_argument(
            "CadLocatorBenchmark: 超采样倍数必须 >= 8，实得 "
            + std::to_string(ss)
            + "。ss = 1 时硬边的亚像素真值不可表示（同一张图对应一整段 t），"
              "基准将输出无意义的数字。见设计说明 §1.1。");
    }
}

/// 某个角点在其自锚定棋盘格下的灰度。
///
/// 坐标系：以角点真值 (cx, cy) 为原点、按 θ 旋转。
///   u = (X−cx)·cosθ + (Y−cy)·sinθ      （棱线 1 的法向）
///   v = −(X−cx)·sinθ + (Y−cy)·cosθ     （棱线 2 的法向）
/// 象限判定 `(floor(u/P) + floor(v/P)) % 2`，于是 u=0 与 v=0 两条**边界**
/// 的之交恰好是角点 —— 这就是"结构点 = 两条非平行棱线的交点"。
///
/// ⚠ 相位从哪来：`cx` 的小数部分。输出像素中心在整数+0.5 处，故 `cx = 100.37`
///    表示边界落在像素栅格之间 —— 这是**唯一**的相位来源，不需要额外参数。
inline int checkerValueAt(double X, double Y, double cx, double cy,
                          double cosT, double sinT)
{
    const double dx = X - cx;
    const double dy = Y - cy;
    const double u = dx * cosT + dy * sinT;
    const double v = -dx * sinT + dy * cosT;
    const long cu = floorDiv(u, kCheckerPitchPx);
    const long cv = floorDiv(v, kCheckerPitchPx);
    // 正确处理负数取模：cu 可为负，(cu+cv) 亦可为负。
    const long parity = ((cu + cv) % 2 + 2) % 2;
    return parity == 0 ? 255 : 0;
}

/// 施加设计说明 §1.5 的退化链。
///
/// ⚠ 顺序固定为 **模糊 → 压缩 → 噪声**，且必须在降采样**之后**调用：
///   在超采样图上加噪再盒式平均等于自动降噪 ss² 倍（ss=8 时降到 1/64），
///   噪声强度会被糊掉 —— 那测的不是真实 CMOS 链路。
cv::Mat applyDegradations(const cv::Mat& clean, const GenOptions& opt)
{
    cv::Mat out = clean;

    if (opt.blurSigma > 0.0)
    {
        // 核大小取 0 → 由 sigma 自动推算（OpenCV 的约定）。
        cv::GaussianBlur(out, out, cv::Size(0, 0), opt.blurSigma);
    }

    if (opt.jpegQuality < 100)
    {
        std::vector<uchar> buf;
        const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,
                                         opt.jpegQuality};
        cv::imencode(".jpg", out, buf, params);
        out = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
    }

    if (opt.noiseSigma > 0.0)
    {
        // 种子来自 GenOptions，故可重放（设计说明 §1.8）。
        cv::RNG rng(opt.seed);
        cv::Mat noise(out.size(), CV_16SC1);
        rng.fill(noise, cv::RNG::NORMAL, 0.0, opt.noiseSigma);
        cv::Mat asFloat;
        out.convertTo(asFloat, CV_16S);
        asFloat += noise;
        asFloat.convertTo(out, CV_8U);
    }

    return out;
}

}  // namespace

GeneratedImage generateCheckerCornerImage(const std::vector<CornerSpec>& corners,
                                          const GenOptions& options)
{
    assertSupersampling(options.supersample);

    if (corners.empty())
    {
        throw std::invalid_argument(
            "CadLocatorBenchmark: 至少需要一个结构点。"
            "（另注：定位器要求 CAD 模型点 >= 4，见设计说明 §1.3 P-1 —— "
            "那是模型侧的要求，与本处的图侧数量是两件事。）");
    }

    const int ss = options.supersample;
    const int w = options.width;
    const int h = options.height;
    const double thetaRad = options.thetaDeg * CV_PI / 180.0;
    const double cosT = std::cos(thetaRad);
    const double sinT = std::sin(thetaRad);

    // ---- Step 1：在 ss×ss 细网格上按解析几何画 ----
    cv::Mat big(h * ss, w * ss, CV_8UC1);

    for (int by = 0; by < h * ss; ++by)
    {
        uchar* row = big.ptr<uchar>(by);
        // 细网格采样点取子像素中心（+0.5），与 INTER_AREA 的盒式平均口径一致。
        const double Y = (by + 0.5) / ss;

        for (int bx = 0; bx < w * ss; ++bx)
        {
            const double X = (bx + 0.5) / ss;

            // 取**最近**角点的棋盘格。
            //
            // 为什么不把多个角点的半平面叠加：一个角点的象限图案是四个
            // 半平面的交，多个角点叠加后整个平面被切成互相冲突的区域，
            // 边界不再只是"棱线"，角点附近会出现额外阶跃。
            //
            // 为什么"最近者胜"不会引入假边缘：接缝出现在两角点的**中垂线**
            // 上，而定位器只在投影点的小窗口内取边缘。只要窗口半径 < 角点
            // 间距之半，窗口内永远只看到**一个**角点的干净棋盘格。
            // ⚠ 调用方须自行保证角点间距 ≫ 窗口半径（接缝两侧的象限图案
            //   不同相位，接缝本身是一条阶跃 —— 它必须是窗口之外的东西）。
            double bestD2 = -1.0;
            double bestX = 0.0;
            double bestY = 0.0;
            for (const CornerSpec& c : corners)
            {
                const double d2 = (X - c.x) * (X - c.x) + (Y - c.y) * (Y - c.y);
                if (bestD2 < 0.0 || d2 < bestD2)
                {
                    bestD2 = d2;
                    bestX = c.x;
                    bestY = c.y;
                }
            }

            row[bx] = static_cast<uchar>(
                checkerValueAt(X, Y, bestX, bestY, cosT, sinT));
        }
    }

    // ---- Step 2：INTER_AREA 降采样 ----
    //
    // 盒式平均。⚠ **但灰度并不"恰好等于亮面积占比"** —— 这一点本节写就时
    // 实测纠正过，原文的措辞是错的，必须记在此处以免后来者据错的前提选 ss：
    //
    //   细网格上每个子样本是**硬判**亮/暗，盒式平均于是把该像素的灰度
    //   量化到 `n/ss`（n = 亮子样本数），实测值为 0, 32, 64, 96, 128, 159,
    //   191, 223 —— 即 255 的 1/8 步长。**渲染量化 = 1/ss pixel。**
    //
    // 这带来一条必须按被测目标选择的量级判据：
    //
    //   · 测 **Baseline**（误差 ~1 pixel）：1/ss = 0.125 pixel ≪ 1 pixel，
    //     完全够用 —— 而且实测 ss=8/16/32/64 给出**逐位相同**的读数，
    //     证明该量级的测量不受渲染限制（见设计说明 §1.1 的 ss 敏感性检查）。
    //   · 测 **亚像素方法**（C-005 路径 2 的目标是 0.1~0.2 pixel）：
    //     0.125 pixel 的渲染量化**与被测量同量级**，此时基准会开始量自己
    //     的夹具而不是量实现。**必须把 ss 提到 ≥ 64（1/64 = 0.016 pixel）**，
    //     或改用解析面积渲染。
    //
    // 本文件把这条写成注释而不是自动选 ss：自动选会让"测的是什么"变得
    // 不可见，而本基准的全部价值就在于口径可见。
    cv::Mat image;
    cv::resize(big, image, cv::Size(w, h), 0.0, 0.0, cv::INTER_AREA);

    // ---- Step 3：退化 ----
    image = applyDegradations(image, options);

    GeneratedImage out;
    out.image = image;
    out.corners = corners;
    out.supersample = ss;
    out.options = options;
    return out;
}

cv::Mat generateDegenerateSs1Image(int width, int height, double subPixelOffset)
{
    // 这条边判定为 `x >= boundary`。
    //
    // ⚠ V-3 的正确形式（本节写就时实测纠正过）：**必须比较落在同一个整数格
    //   内的两个偏移**，即 `width/2 + t` 与 `width/2 + t'` 同属一个 (k, k+1]。
    //   实测：t=0.1 与 t=0.5 → 逐位相同 ✅；t=0.9 与 t=1.1 → 不同 ✅（量化步长恰好 1 px）。
    //
    //   为什么不是"任意两个不同的 t"：`t = 0.0` 使 boundary 恰好等于整数
    //   `width/2`，而判定式是 `x >= boundary` —— 该整数列**算作亮**，
    //   与 t ∈ (0,1] 的全部取值都相反。于是 t=0.0 vs t=0.5 会不同，
    //   而那是**边界取整的端点效应**，不是量化本身。
    //   按"任意两个 t 必须相同"去写 V-3 会**误判超采样未生效**。
    const double boundary = width / 2.0 + subPixelOffset;

    cv::Mat img(height, width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < height; ++y)
    {
        uchar* row = img.ptr<uchar>(y);
        for (int x = 0; x < width; ++x)
        {
            row[x] = (static_cast<double>(x) >= boundary) ? 255 : 0;
        }
    }
    return img;
}

}  // namespace cadbench
