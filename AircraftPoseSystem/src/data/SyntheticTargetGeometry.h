#pragma once

// ============================================================================
//  src/data/SyntheticTargetGeometry.h
//
//  依据：批次 B（最小模拟成功流程）的验收要求：
//        "图像、模型、标定来自同一已知几何"
//
//  作用：**唯一一份**合成目标的几何定义。以下两处必须共用本文件：
//    1. `models/` 中的 `points3d.yaml` 生成器（测试夹具）；
//    2. `SyntheticTargetScene` 的投影渲染。
//
//  ⚠ 为什么几何只写一份（而不是"模型一套、画面一套"）：
//    若两处各写一份点表，它们可以**同时**通过编译与运行，只在数值上
//    悄悄分叉。那时 PnP 会解出一个"重投影很小、但与图像内容无关"的姿态，
//    而 RANSAC 的内点比例照样正常 —— 这正是本工程反复出现的
//    "字段存在、数值合法、语义不成立"。
//
//  坐标系：Aircraft Frame（SYS-13），单位 **m**（ENG-09 §2.3）。
//  点的顺序即多边形边的顺序（相邻点相连，末点与首点相连）。
//  本夹具是**具有深度变化的六点合成靶标**，不是“平板”：六个点在
//  Aircraft Frame 中的三维分布经 SVD 检验确为非共面（判据与实测值
//  见 `outline()` 下方注释）。投影后为直角 L 形，理由见
//  `outline()` 上方注释。
//
//  ⚠ 场景适用范围：本几何专为“轴对齐、无背景网格”的基础通路正例
//  而构造。斜边光栅化与背景平行线干扰会使 `CadStructureLocator`
//  按冻结残差判据拒绝，那是已登记的算法适用性问题，**不**由本夹具
//  掩盖，也不在本批解决。
// ============================================================================

#include <vector>

#include <opencv2/core.hpp>

#include "data/ModelPoint3D.h"

namespace aircraft
{
namespace data
{
namespace synthetic
{

/// 合成机型库的标识（写入 model.yaml / result.json）。
inline const char* modelId()
{
    return "synthetic_aircraft_6pt";
}

/// 机型类型（C-003 的两种取值之一）。
inline const char* modelType()
{
    return "synthetic";
}

/// 真值来源标记：写入结果包，表明真值来自本文件的几何 + 场景构造，
/// **不是**从算法输出反推的。
inline const char* truthSource()
{
    return "synthetic_geometry";
}

/// 组件真实度声明：结果包中的 `component_realism`。
/// 逐项列出本次运行**不是**真实部件的环节，避免"COMPLETE"被误读为
/// "真实测量链已验收"。
inline const char* componentRealism()
{
    return "camera=virtual;turntable=virtual;trigger=virtual;"
           "scene=synthetic_projection;detector=mock;scale=pinhole;"
           "features=sift;matcher=descriptor;pose=iterative_pnp";
}

/// 六个结构点在 Aircraft Frame 中的位置，单位 m。
///
/// 这是**具有深度变化的六点合成靶标**（投影为直角 L 形，三条边长短
/// 不一），不是“平板”。形状本身不是随手取的凸多边形。
/// 取舍（全部可检验）：
///  · 在上述已知位姿与内参下，六个点投影到图像上的设计位置为
///    (480,400) (800,400) (800,624) (620,624) (620,500) (480,500)，
///    即每条边都**严格轴对齐**（水平或竖直）。原因不是好看：
///    `CadStructureLocator` 对每条拟合棱线使用冻结判据
///    “中位残差 ≤ 0.3 px”（ENG-10 §2.4）。硬光栅化下，斜边的 Canny
///    迹线通常是 2 px 宽，中位残差约 0.35 px，会**整批**被拒；只有
///    轴对齐边的迹线稳定为 1 px。这是夹具的几何选择，**不是**放宽
///    或改动生产判据。
///  · 该性质依赖两件事：本表按 `SyntheticTargetScene::
///    defaultTruthAircraftToShip()` 的位姿与
///    `CalibrationManager::loadDefaults(1280,1024)` 的内参反投影得到。
///    任一处改动都必须重新用定位器核对“六个角点仍全部通过”，
///    集成用例中的 `correspondences >= 4` 断言是最后防线。
///  · 相邻顶点投影间距 100~320 px，定位器窗口半径为最近邻中位数的一半
///    （上限 60 px），每个角点的窗口内两条棱线各可提供 ≥8 个边缘像素。
///
/// **非共面性检验（夹具事实，不是从深度极差推断）**：
///  对下表六个坐标先减去质心 (−0.010367, −0.095183, −0.174317) m，
///  再对 6×3 中心化矩阵做 SVD（`/tmp/svd_check.cpp`，2026-10-20 实测）：
///    奇异值 σ1=8.546060674, σ2=5.468042345, σ3=0.644843763 m
///    σ3/σ1 = 7.545509e-02
///    最佳拟合平面法向 (0.989987445, 0.139938914, −0.018492144)
///    六点到该平面的有符号距离分别为
///      −0.117626, +0.231364, −0.338474, +0.442892, −0.192578, −0.025577 m
///    max|d| = 0.442892 m。
///  判据：σ3/σ1 与 max|d| 同时不可忽略（此处置信地判为非共面）。
///  仅“深度 39.5~40.5 m 不同”**不能**证明非共面 —— 倾斜平板上的点
///  同样有不同深度；上表坐标与 SVD 才是依据。
///  ⚠ 本几何为已冻结的夹具事实，不得为“看起来更非共面”再改坐标；
///  若将来改坐标，必须重跑上述检验并更新本注释。
inline std::vector<cv::Point3f> outline()
{
    return {
        cv::Point3f( 0.1922f, -2.8931f, -4.1421f),   // 1 图像设计位 (480,400)
        cv::Point3f( 0.6887f, -2.8238f,  4.0904f),   // 2 (800,400)
        cv::Point3f(-0.6935f,  2.8961f,  4.1939f),   // 3 (800,624)
        cv::Point3f( 0.0084f,  2.8919f, -0.5152f),   // 4 (620,624)
        cv::Point3f(-0.1789f, -0.3248f, -0.5204f),   // 5 (620,500) 凹角
        cv::Point3f(-0.0791f, -0.3174f, -4.1525f),   // 6 (480,500)
    };
}

/// 与 `outline()` 一一对应的三维点表（B 类 CAD 结构点）。
inline std::vector<ModelPoint3D> modelPoints()
{
    std::vector<ModelPoint3D> points;
    const std::vector<cv::Point3f> pts = outline();
    points.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        ModelPoint3D p;
        p.id = static_cast<int>(i) + 1;
        p.position = pts[i];
        p.featureType = "cad";
        points.push_back(p);
    }
    return points;
}

}  // namespace synthetic
}  // namespace data
}  // namespace aircraft
