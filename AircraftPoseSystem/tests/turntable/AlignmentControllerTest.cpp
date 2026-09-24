// ============================================================================
//  tests/turntable/AlignmentControllerTest.cpp
//
//  覆盖：ENG-06 §9.2 的三类用例 —— 大偏差 / 小偏差 / 居中停止 ——
//        以及 SYS-10 §8 的换算律、§14 的失败出口、ENG-09 §2.4 的符号约定。
//
//  ⚠ 本文件承担一个超出普通单测的职责：**把冻洁文档里没有写明的两件事
//  钉死**，使它们不再靠注释约定而靠测试保证：
//    1. arctan 换算是**非线性**的（大偏差不能当线性比例处理）；
//    2. 像素 Y 向下为正，俯仰命令取负号（ENG-09 §2.4 明文要求显式处理）。
//  这两条若取反，闭环会**发散**而不是静止，现象是耗尽 8 次对准后报 2003
//  ——故障现象指向算法，根因在符号。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include "application/AlignmentController.h"
#include "data/CameraCalibration.h"
#include "data/ErrorInfo.h"
#include "data/TargetOffset.h"
#include "data/TurntableConfig.h"
#include "data/TurntableCommand.h"
#include "data/TurntableState.h"

using aircraft::application::AlignmentController;
using aircraft::data::CameraCalibration;
using aircraft::data::ErrorInfo;
using aircraft::data::TargetOffset;
using aircraft::data::TurntableCommand;
using aircraft::data::TurntableConfig;
using aircraft::data::TurntableState;

namespace
{

constexpr double kDegToRad = 0.01745329251994329576923690768489;

/// 一个"已配置"的转台：行程 ±180° / −60°~+60°，居中阈值 50 pixel（§6.3 默认）。
TurntableConfig configuredTurntable()
{
    TurntableConfig c;
    c.azimuthMin = -180.0;
    c.azimuthMax = 180.0;
    c.elevationMin = -60.0;
    c.elevationMax = 60.0;
    c.centerThreshold = 50.0;
    return c;
}

/// 相机标定：fx = fy = 2000 pixel（≈ 1280×1024 靶面 + 中焦镜头）。
CameraCalibration calibrationWith(double fx, double fy)
{
    CameraCalibration c;
    c.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    c.cameraMatrix.at<double>(0, 0) = fx;
    c.cameraMatrix.at<double>(1, 1) = fy;
    c.imageWidth = 1280;
    c.imageHeight = 1024;
    return c;
}

TargetOffset offsetOf(double x, double y)
{
    TargetOffset o;
    o.pixelX = x;
    o.pixelY = y;
    return o;
}

TurntableState turntableAt(double az, double el)
{
    TurntableState s;
    s.azimuth = az;
    s.elevation = el;
    return s;
}

}  // namespace

// ===========================================================================
//  1 居中判据（§5.4 / SYS-10 §7）
// ===========================================================================

TEST(AlignmentControllerTest, 居中停止_阈值内判为居中)
{
    AlignmentController ac(configuredTurntable());

    EXPECT_TRUE(ac.isCentered(offsetOf(0.0, 0.0)));
    EXPECT_TRUE(ac.isCentered(offsetOf(50.0, -50.0)));   // 边界含等于
    EXPECT_TRUE(ac.isCentered(offsetOf(-49.9, 49.9)));
}

TEST(AlignmentControllerTest, 居中停止_阈值外不判居中)
{
    AlignmentController ac(configuredTurntable());

    EXPECT_FALSE(ac.isCentered(offsetOf(50.1, 0.0)));
    EXPECT_FALSE(ac.isCentered(offsetOf(0.0, -50.1)));
    EXPECT_FALSE(ac.isCentered(offsetOf(1000.0, 0.0)));
}

TEST(AlignmentControllerTest, 居中停止_judgeCentered填写TargetOffset字段)
{
    // TargetOffset::centered 的判定逻辑按 TargetOffset.h 的明文规定留在
    // 本层（data 不含业务逻辑），故必须验证它真的被填上了 ——
    // 若始终为 false，ALIGN 会永远无法满足停止条件，最终报 2003。
    AlignmentController ac(configuredTurntable());

    TargetOffset inside = offsetOf(10.0, -10.0);
    EXPECT_FALSE(inside.centered);          // 默认值
    ac.judgeCentered(inside);
    EXPECT_TRUE(inside.centered);

    TargetOffset outside = offsetOf(200.0, 0.0);
    ac.judgeCentered(outside);
    EXPECT_FALSE(outside.centered);
}

TEST(AlignmentControllerTest, 阈值取自配置而非硬编码50)
{
    TurntableConfig c = configuredTurntable();
    c.centerThreshold = 5.0;
    AlignmentController ac(c);

    EXPECT_FALSE(ac.isCentered(offsetOf(20.0, 0.0)))
        << "阈值改为 5 pixel 后，20 pixel 不应判居中（不得硬编码 50）";
    EXPECT_TRUE(ac.isCentered(offsetOf(4.9, 0.0)));
}

// ===========================================================================
//  2 小偏差：换算律 Δθ = arctan(Δpixel / f)（SYS-10 §8）
// ===========================================================================

TEST(AlignmentControllerTest, 小偏差_命令等于当前角加arctan换算量)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    // 目标在图像中心右侧 200 pixel → 方位应增大 arctan(200/2000) = 5.7106°。
    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(10.0, 0.0), cal);

    EXPECT_TRUE(ac.commandValid());
    EXPECT_EQ(ac.lastError().code, 0);
    EXPECT_NEAR(cmd.azimuthCommand, 10.0 + std::atan(200.0 / 2000.0) / kDegToRad, 1e-9);
    EXPECT_NEAR(cmd.elevationCommand, 0.0, 1e-9);
}

TEST(AlignmentControllerTest, 小偏差_命令是绝对角而不是增量)
{
    // TurntableCommand 的两个字段语义是"要求转台去哪"（ENG-09 §5.10），
    // 不是"转多少"。若实现写成增量，同样输入在 az=0 与 az=100 时会得到
    // 相同的命令 —— 而闭环会因此永远差一个固定偏置。
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand a =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), cal);
    const TurntableCommand b =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(100.0, 0.0), cal);

    EXPECT_GT(b.azimuthCommand - a.azimuthCommand, 99.0)
        << "当前角不同时命令必须不同（绝对角语义）";
}

// ===========================================================================
//  3 大偏差：arctan 的非线性（SYS-10 §8）
// ===========================================================================

TEST(AlignmentControllerTest, 大偏差_arctan非线性而非线性比例)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    // Δpixel = f 时 Δθ 恰为 45°（arctan(1)）。
    const TurntableCommand cmd45 =
        ac.calculate(offsetOf(2000.0, 0.0), turntableAt(0.0, 0.0), cal);
    EXPECT_NEAR(cmd45.azimuthCommand, 45.0, 1e-9);

    // 偏差加倍不会让角度加倍：arctan(2) = 63.43°，而线性外推会说 90°。
    const TurntableCommand cmd90 =
        ac.calculate(offsetOf(4000.0, 0.0), turntableAt(0.0, 0.0), cal);
    EXPECT_NEAR(cmd90.azimuthCommand, std::atan(2.0) / kDegToRad, 1e-9);
    EXPECT_LT(cmd90.azimuthCommand, 90.0)
        << "线性比例处理会给出 90°（甚至更大），必然过冲";

    // 大偏差下仍不得越界（2000 pixel → 45°，而非 arctan 的近似值）。
    EXPECT_TRUE(ac.commandValid());
}

TEST(AlignmentControllerTest, 大偏差_fx与fy分别用于方位与俯仰)
{
    // 非方形像素（fx != fy）时混用会引入固定比例误差。
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(1600.0, 2400.0);

    const TurntableCommand cmd =
        ac.calculate(offsetOf(1600.0, -2400.0), turntableAt(0.0, 0.0), cal);

    EXPECT_NEAR(cmd.azimuthCommand, 45.0, 1e-9);       // arctan(1600/1600)
    // 俯仰：pixelY = -2400（目标在中心**上方**）→ 须向上转 → 俯仰角增大。
    EXPECT_NEAR(cmd.elevationCommand, std::atan(2400.0 / 2400.0) / kDegToRad, 1e-9);
    EXPECT_GT(cmd.elevationCommand, 0.0);
}

// ===========================================================================
//  4 符号约定（ENG-09 §2.4：X 向右为正，Y 向下为正）
// ===========================================================================

TEST(AlignmentControllerTest, 符号_目标在中心下方时俯仰角减小)
{
    // 图像 Y 向下为正：pixelY > 0 表示目标在画面**下方**，相机须向下转。
    // SYS-10 §3.2 的口径是"俯仰向上为正、水平为零位"，故向下转 = 俯仰减小。
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand below =
        ac.calculate(offsetOf(0.0, 100.0), turntableAt(0.0, 0.0), cal);
    EXPECT_LT(below.elevationCommand, 0.0)
        << "目标在中心下方时俯仰命令必须为负（Y 向下为正）";

    const TurntableCommand above =
        ac.calculate(offsetOf(0.0, -100.0), turntableAt(0.0, 0.0), cal);
    EXPECT_GT(above.elevationCommand, 0.0);
}

TEST(AlignmentControllerTest, 符号_目标在中心右侧时方位角增大)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand right =
        ac.calculate(offsetOf(100.0, 0.0), turntableAt(0.0, 0.0), cal);
    EXPECT_GT(right.azimuthCommand, 0.0);

    const TurntableCommand left =
        ac.calculate(offsetOf(-100.0, 0.0), turntableAt(0.0, 0.0), cal);
    EXPECT_LT(left.azimuthCommand, 0.0);
}

TEST(AlignmentControllerTest, 符号_两轴都已在位时命令等于当前角)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand cmd =
        ac.calculate(offsetOf(0.0, 0.0), turntableAt(12.5, -3.5), cal);

    EXPECT_NEAR(cmd.azimuthCommand, 12.5, 1e-9);
    EXPECT_NEAR(cmd.elevationCommand, -3.5, 1e-9);
    EXPECT_TRUE(ac.commandValid());
}

// ===========================================================================
//  5 量纲陷阱：focalLength（米）不得用于 arctan（ENG-09 §2.3 / §2.4）
// ===========================================================================

TEST(AlignmentControllerTest, 量纲_必须用像素单位的焦距而不是米)
{
    // CameraChannel::focalLength 的单位是**米**（100 mm 镜头写作 0.1）。
    // 若把它代进 arctan(Δpixel / f)，200 pixel 会得到 arctan(2000) ≈ 89.97°
    // ——转台直接飞向行程端点。
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), cal);

    EXPECT_LT(cmd.azimuthCommand, 10.0)
        << "结果为数十度说明用米当成了像素";
    EXPECT_NEAR(cmd.azimuthCommand, 5.710593137, 1e-6);
}

// ===========================================================================
//  6 越程（§14 / §7.7 的能力边界：停止运动 + 报警 + FAILED）
// ===========================================================================

TEST(AlignmentControllerTest, 越程_方位越界时拒绝且返回原地命令)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    // 当前 az = 179.5°，目标偏右 200 pixel → 命令 185.2° > 180° 上限。
    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(179.5, 0.0), cal);

    EXPECT_FALSE(ac.commandValid());
    EXPECT_TRUE(ac.overTravel());
    EXPECT_EQ(ac.lastError().code, aircraft::data::kErrTurntableOverTravel);

    // §7.7 要求"**停止运动**"：返回的命令必须等于当前位置，
    // 这样即便调用方漏查 commandValid() 就下发，转台也不会移动。
    // 若实现改为"夹取到限位"，命令看起来合法（180.0°）却会让机构真的走过去。
    EXPECT_DOUBLE_EQ(cmd.azimuthCommand, 179.5);
    EXPECT_DOUBLE_EQ(cmd.elevationCommand, 0.0);
}

TEST(AlignmentControllerTest, 越程_俯仰越界时拒绝)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    // 当前 el = 0°，目标在中心下方 2000 pixel → 命令 −45°，下限 −60° 之内。
    EXPECT_TRUE(ac.calculate(offsetOf(0.0, 2000.0), turntableAt(0.0, 0.0), cal)
                    .elevationCommand < 0.0);
    EXPECT_TRUE(ac.commandValid());

    // 当前 el = −30°，目标在下方 4000 pixel → 命令 −93.4°，超出 −60° 下限。
    const TurntableCommand cmd =
        ac.calculate(offsetOf(0.0, 4000.0), turntableAt(0.0, -30.0), cal);
    EXPECT_FALSE(ac.commandValid());
    EXPECT_EQ(ac.lastError().code, aircraft::data::kErrTurntableOverTravel);
    EXPECT_DOUBLE_EQ(cmd.elevationCommand, -30.0);
}

TEST(AlignmentControllerTest, 越程_恰好落在限位上不算越程)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    // 当前 el = -60°，就在下限上且无偏差 → 命令 = -60.0°，应放行。
    const TurntableCommand cmd =
        ac.calculate(offsetOf(0.0, 0.0), turntableAt(0.0, -60.0), cal);
    EXPECT_TRUE(ac.commandValid());
    EXPECT_DOUBLE_EQ(cmd.elevationCommand, -60.0);
}

TEST(AlignmentControllerTest, 越程_行程未配置时跳过判定并如实记录)
{
    // TurntableConfig 的四个限位字段默认都是 0。把"min == max == 0"当成
    // 真实行程，会让任何非零命令都被判越程 —— 一次没配 yaml 的启动
    // 会表现为"转台一动就超程报警"。故 min < max 才启用判定，
    // 不启用时必须留下可见的 notice（不静默）。
    TurntableConfig unconfigured;   // 全部保持默认 0
    AlignmentController ac(unconfigured);
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), cal);

    EXPECT_TRUE(ac.commandValid()) << "行程未配置不应把命令判成越程";
    EXPECT_FALSE(ac.overTravel());
    EXPECT_NEAR(cmd.azimuthCommand, 5.710593137, 1e-6);
    EXPECT_FALSE(ac.lastError().message.empty())
        << "必须记录「限位未生效」这一事实，不得静默";
    EXPECT_EQ(ac.lastError().code, 0);
}

// ===========================================================================
//  7 标定无效（不能静默给出一个看似合理的角度）
// ===========================================================================

TEST(AlignmentControllerTest, 标定_空内参时拒绝而不是当成零焦距)
{
    // data::CameraCalibration 的默认构造给的是 3x3 单位阵 + 靶面尺寸 0，
    // 即 fx = fy = **1**。fx = 1 会让 arctan(200/1) ≈ 89.68° ——
    // 一个"算得出来"、看上去完全合法、而转台会真的照做的大角度。
    // 这正是未标定相机最危险的失败方式，故必须显式拒绝。
    AlignmentController ac(configuredTurntable());
    CameraCalibration empty;

    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), empty);

    EXPECT_FALSE(ac.commandValid());
    // 裁决 C-006 已为"标定缺失/无效"登记 4001。此前这里断言的是 0，
    // 理由是"标定段无登记码位" —— 而 0 的冻结语义是"未设置"、渲染为
    // "OK"，于是一台**因缺标定而失败**的测量，其终态错误码看起来像
    // 一切正常。4001 是该段的第一个码。
    EXPECT_EQ(ac.lastError().code, aircraft::data::kErrCalibrationMissing);
    EXPECT_FALSE(ac.lastError().message.empty());
    EXPECT_DOUBLE_EQ(cmd.azimuthCommand, 0.0) << "拒绝时返回原地命令";
}

TEST(AlignmentControllerTest, 标定_只填了靶面尺寸但焦距仍为占位值时被拒绝)
{
    // 半成品标定：靶面尺寸填了，fx/fy 还是默认的 1。
    // 它是 ①b（靶面尺寸）拦不住的 —— 只能靠 ①c（视场 < 90°
    // 即 fx > 半幅宽）拦下。缺了 ①c，这条输入会产出 89.68° 的方位命令。
    AlignmentController ac(configuredTurntable());
    CameraCalibration half;
    half.imageWidth = 1280;
    half.imageHeight = 1024;   // cameraMatrix 仍为单位阵：fx = fy = 1

    const TurntableCommand cmd =
        ac.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), half);

    EXPECT_FALSE(ac.commandValid());
    EXPECT_FALSE(ac.lastError().message.empty());
    EXPECT_DOUBLE_EQ(cmd.azimuthCommand, 0.0) << "拒绝时返回原地命令";

    // 边界另一侧：fx 恰好等于半幅宽（视场正好 90°）同样被拒，
    // 因为判据是**严格**大于。
    AlignmentController boundary(configuredTurntable());
    CameraCalibration exact = calibrationWith(640.0, 512.0);
    boundary.calculate(offsetOf(200.0, 0.0), turntableAt(0.0, 0.0), exact);
    EXPECT_FALSE(boundary.commandValid()) << "fx == imageWidth/2 即视场 90°，应拒";
}

TEST(AlignmentControllerTest, 标定_非正或非有限的焦距被拒绝)
{
    AlignmentController ac(configuredTurntable());

    CameraCalibration zero = calibrationWith(2000.0, 2000.0);
    zero.cameraMatrix.at<double>(0, 0) = 0.0;
    const TurntableCommand cmdZero =
        ac.calculate(offsetOf(10.0, 0.0), turntableAt(7.0, 0.0), zero);
    EXPECT_FALSE(ac.commandValid()) << "fx = 0 无法给出有限角度，必须拒绝";
    EXPECT_DOUBLE_EQ(cmdZero.azimuthCommand, 7.0) << "拒绝时返回原地命令";

    CameraCalibration nan = calibrationWith(2000.0, 2000.0);
    nan.cameraMatrix.at<double>(1, 1) = std::nan("");
    const TurntableCommand cmdNan =
        ac.calculate(offsetOf(0.0, 10.0), turntableAt(0.0, 4.0), nan);
    EXPECT_FALSE(ac.commandValid()) << "fy = NaN 必须拒绝";
    EXPECT_DOUBLE_EQ(cmdNan.elevationCommand, 4.0);
}

TEST(AlignmentControllerTest, 输入含非有限值时拒绝)
{
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand cmd =
        ac.calculate(offsetOf(std::nan(""), 0.0), turntableAt(2.0, 1.0), cal);
    EXPECT_FALSE(ac.commandValid());
    EXPECT_DOUBLE_EQ(cmd.azimuthCommand, 2.0) << "拒绝时返回原地命令，不得发出 NaN";
    EXPECT_DOUBLE_EQ(cmd.elevationCommand, 1.0);

    // 当前角本身就是 NaN 时同样必须拒绝。此时"原地命令"也等于 NaN ——
    // 它是**不可下发**的，契约因此只有一条：调用方必须先查 commandValid()。
    // 这比"悄悄改发 0°"安全：0° 是一个合法角度，会被如实执行成一次大幅运动。
    ac.calculate(offsetOf(10.0, 0.0), turntableAt(std::nan(""), 0.0), cal);
    EXPECT_FALSE(ac.commandValid());
}

// ===========================================================================
//  8 状态无关性：本类不计数、不循环、不读时钟（裁决 C-20）
// ===========================================================================

TEST(AlignmentControllerTest, 连续调用结果一致且不累积)
{
    // 若本类内部藏了计数器或累加器，同一次输入在第 N 次调用后会给出
    // 不同的命令 —— 而"对准次数"的唯一数据源必须是
    // MeasurementConfig::maxAlignAttempts（裁决 C-20）。
    AlignmentController ac(configuredTurntable());
    const CameraCalibration cal = calibrationWith(2000.0, 2000.0);

    const TurntableCommand first =
        ac.calculate(offsetOf(120.0, -80.0), turntableAt(3.0, -1.0), cal);

    for (int i = 0; i < 20; ++i)
    {
        const TurntableCommand again =
            ac.calculate(offsetOf(120.0, -80.0), turntableAt(3.0, -1.0), cal);
        EXPECT_DOUBLE_EQ(again.azimuthCommand, first.azimuthCommand);
        EXPECT_DOUBLE_EQ(again.elevationCommand, first.elevationCommand);
        EXPECT_TRUE(ac.commandValid());
    }
}
