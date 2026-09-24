// ============================================================================
//  tests/optical/OpticalLayerTest.cpp
//
//  依据：ENG-06（测试工程设计）、ENG-08 §13（第一阶段测试任务）
//        ENG-09 §2.1（坐标链方向）、§2.3（毫米→米）、§2.5（时间戳）
//        ENG-10 §5.3（配置装载的三种失败模式）
//        SYS-08 §7.5（硬件降级）、SYS-13 §5（坐标合成）、SYS-06 §8（同步）
//
//  覆盖 004 Optical 的四个类：
//      OpticalRig           通道注册表与只读标定
//      CalibrationManager   7 个标定文件的装载与毫米→米换算
//      CoordinateTransformer 坐标链合成与欧拉角分解
//      CameraSynchronizer    三相机同步验证与组合
//
//  ⚠ 本文件**自造标定文件**（用 cv::FileStorage 在临时目录写出），
//  不依赖 tests/golden/ 的资产。两个理由：
//    1. 走一遍 OpenCV 的写→读往返，才能证明本工程的 YAML schema 与
//       OpenCV 工具链（SYS-11 标定程序将会用的同一套 API）相容；
//       手写的固定文本文件只能证明"本实现能读它自己写的东西"。
//    2. 标定目录里每个数值都直接决定毫米→米的判据，测试需要能
//       逐项控制（例如把 200 mm 写成别的值），而 golden 资产是只读的。
//
//  ⚠ 不在本文件覆盖的内容：
//    · CameraSynchronizer 与真实 VirtualCameraBackend 的联调
//      → tests/integration/（010），那里三者都有实例；
//    · 三相机同步精度对最终姿态误差的贡献
//      → tests/stability/ 与 SYS-15 §4 的误差合成验证。
// ============================================================================

#include <gtest/gtest.h>

#include <unistd.h>  // getpid()

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <type_traits>
#include <vector>

#include <opencv2/core.hpp>

#include "data/CameraChannel.h"
#include "data/CameraPose.h"
#include "data/CameraRole.h"
#include "data/ErrorInfo.h"
#include "data/ImageFrame.h"
#include "data/MultiCameraFrame.h"
#include "data/TriggerConfig.h"
#include "optical/CalibrationManager.h"
#include "optical/CameraSynchronizer.h"
#include "optical/CoordinateTransformer.h"
#include "optical/OpticalRig.h"

namespace
{

using aircraft::data::CameraRole;
using aircraft::optical::CalibrationManager;
using aircraft::optical::CameraSynchronizer;
using aircraft::optical::CoordinateTransformer;
using aircraft::optical::OpticalRig;

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol = 1e-9)
{
    return std::fabs(a - b) <= tol;
}

/// 绕 Z 轴旋转 deg 度的旋转矩阵。
cv::Matx33d rotZ(double deg)
{
    const double r = deg * kPi / 180.0;
    return cv::Matx33d(std::cos(r), -std::sin(r), 0.0,
                       std::sin(r),  std::cos(r), 0.0,
                       0.0,        0.0,        1.0);
}

// ---------------------------------------------------------------------------
// 标定目录夹具
//
// 用 OpenCV 自己的 FileStorage（WRITE）写出 7 个文件，再交给
// CalibrationManager 读 —— 见文件头第 1 条理由。
// ---------------------------------------------------------------------------
class CalibrationDirFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dir_ = std::filesystem::temp_directory_path() /
               ("aps_optical_test_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);

        writeIntrinsic("cam25.yaml",  1280, 1024, 1000.0);
        writeIntrinsic("cam50.yaml",  1280, 1024, 2000.0);
        writeIntrinsic("cam100.yaml", 1280, 1024, 4000.0);

        // 基线放在 x 轴上，用**毫米**书写（字段名 translation_mm）。
        writeExtrinsic("cam25_to_rig.yaml",   0.0,   0.0, 0.0, 0.0);
        writeExtrinsic("cam50_to_rig.yaml",   0.0, 200.0, 0.0, 0.0);
        writeExtrinsic("cam100_to_rig.yaml",  0.0, 400.0, 0.0, 0.0);

        // rig → ship：绕 Z 转 90°，平移 (1,2,3) mm，并带标定版本号。
        writeExtrinsic("rig_to_ship.yaml", 90.0, 1.0, 2.0, 3.0, "CALIB-TEST-0001");
    }

    void TearDown() override
    {
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec);
    }

    std::string path(const std::string& name) const
    {
        return (dir_ / name).string();
    }
    std::string dir() const { return dir_.string(); }

    /// 删掉一个标定文件。
    void removeFile(const std::string& name)
    {
        std::error_code ec;
        std::filesystem::remove(dir_ / name, ec);
    }

    /// 删掉 4 个外参文件，只留内参 —— 用于制造"文件缺失"。
    ///
    /// ⚠ 必须真的**删文件**：SetUp 已经写过它们，仅"不再写"是删不掉的，
    /// 于是 load() 仍会成功，测试会以"预期失败却成功"的形式失败。
    void removeExtrinsics()
    {
        removeFile("cam25_to_rig.yaml");
        removeFile("cam50_to_rig.yaml");
        removeFile("cam100_to_rig.yaml");
        removeFile("rig_to_ship.yaml");
    }

private:
    static void writeIntrinsic(const std::string& name,
                               int w, int h, double fx)
    {
        const std::string p =
            (std::filesystem::temp_directory_path() /
             ("aps_optical_test_" + std::to_string(::getpid())) / name).string();

        cv::FileStorage fs(p, cv::FileStorage::WRITE);
        cv::Mat k = (cv::Mat_<double>(3, 3) << fx, 0.0, w / 2.0,
                                              0.0, fx, h / 2.0,
                                              0.0, 0.0, 1.0);
        cv::Mat d = cv::Mat::zeros(1, 5, CV_64F);
        fs << "image_width"  << w;
        fs << "image_height" << h;
        fs << "camera_matrix" << k;
        fs << "distortion"    << d;
    }

    static void writeExtrinsic(const std::string& name,
                               double rotZDeg,
                               double txMm, double tyMm, double tzMm,
                               const std::string& calibId = std::string())
    {
        const std::string p =
            (std::filesystem::temp_directory_path() /
             ("aps_optical_test_" + std::to_string(::getpid())) / name).string();

        cv::FileStorage fs(p, cv::FileStorage::WRITE);

        if (!calibId.empty())
        {
            fs << "calibration_id" << calibId;
        }

        // 注意字段名是 rotation（矩阵）而不是欧拉角 ——
        // CoordinateTransformer 会拒绝非正交矩阵，故夹具必须写矩阵。
        const double r = rotZDeg * kPi / 180.0;
        cv::Mat R = (cv::Mat_<double>(3, 3) << std::cos(r), -std::sin(r), 0.0,
                                               std::sin(r),  std::cos(r), 0.0,
                                               0.0,        0.0,        1.0);
        cv::Mat t = (cv::Mat_<double>(3, 1) << txMm, tyMm, tzMm);

        fs << "rotation"       << R;
        fs << "translation_mm" << t;  // ⚠ 单位毫米（ENG-09 §2.3）
    }

    std::filesystem::path dir_;
};

// ===========================================================================
//  OpticalRig
// ===========================================================================

TEST(OpticalRigTest, DefaultChannelsUseMetresNotMillimetres)
{
    OpticalRig rig;
    ASSERT_TRUE(rig.initialize());

    // ⚠ 本工程最容易犯的单位错误（ENG-09 §2.3）：100mm 镜头写作 0.1。
    // 若这里出现 25/50/100，则焦距与以米表达的 3D 点相差 1000 倍，
    // 而重投影误差会同比例缩小，SYS-07 §12 的验证判据全部照常通过。
    const auto* c25  = rig.getCamera(CameraRole::CAM25);
    const auto* c50  = rig.getCamera(CameraRole::CAM50);
    const auto* c100 = rig.getCamera(CameraRole::CAM100);

    ASSERT_NE(c25, nullptr);
    ASSERT_NE(c50, nullptr);
    ASSERT_NE(c100, nullptr);

    EXPECT_TRUE(near(c25->focalLength,  0.025, 1e-12));
    EXPECT_TRUE(near(c50->focalLength,  0.05,  1e-12));
    EXPECT_TRUE(near(c100->focalLength, 0.1,   1e-12));
}

TEST(OpticalRigTest, EitherExtremeOfTheMillimetreBugIsRejected)
{
    OpticalRig rig;
    ASSERT_TRUE(rig.initialize());

    // 反向确认上面的判据真的有鉴别力：毫米值必须**不**满足。
    const auto* c100 = rig.getCamera(CameraRole::CAM100);
    ASSERT_NE(c100, nullptr);
    EXPECT_FALSE(near(c100->focalLength, 100.0, 1.0));
}

TEST(OpticalRigTest, DefaultTableRegistersEachRoleExactlyOnce)
{
    OpticalRig rig;
    ASSERT_TRUE(rig.initialize());

    EXPECT_EQ(rig.cameras().size(), 3u);

    int n25 = 0, n50 = 0, n100 = 0;
    for (const auto& ch : rig.cameras())
    {
        if (ch.role == CameraRole::CAM25)  { ++n25;  }
        if (ch.role == CameraRole::CAM50)  { ++n50;  }
        if (ch.role == CameraRole::CAM100) { ++n100; }
    }
    EXPECT_EQ(n25,  1);
    EXPECT_EQ(n50,  1);
    EXPECT_EQ(n100, 1);
}

TEST(OpticalRigTest, DuplicateRoleIsRejected)
{
    OpticalRig rig;

    std::vector<aircraft::data::CameraChannel> channels(2);
    channels[0].role = CameraRole::CAM25;
    channels[1].role = CameraRole::CAM25;  // 重复

    // 重复角色会让 getCamera() 的结果依赖表内顺序 ——
    // 即"哪一台相机的标定被用到"变成一个隐式事实，故在 initialize 处拒绝。
    EXPECT_FALSE(rig.initialize(channels));
}

TEST(OpticalRigTest, SetCalibrationAlsoUpdatesChannel)
{
    OpticalRig rig;
    ASSERT_TRUE(rig.initialize());

    aircraft::data::OpticalRigCalibration cal;
    cal.cam50.imageWidth = 640;

    ASSERT_TRUE(rig.setCalibration(cal));

    // 若只更新 calibration_ 而不推入通道，则 getCamera(role)->calibration
    // 与 calibration().camXX 会不一致 —— 而 device 读其中一个、
    // CoordinateTransformer 读另一个。
    const auto* c50 = rig.getCamera(CameraRole::CAM50);
    ASSERT_NE(c50, nullptr);
    EXPECT_EQ(c50->calibration.imageWidth, 640);
    EXPECT_EQ(rig.calibration().cam50.imageWidth, 640);
}

TEST(OpticalRigTest, QueryReturnsConstPointerNotWritableReference)
{
    OpticalRig rig;
    ASSERT_TRUE(rig.initialize());

    // 编译期约束的可读形式：若 getCamera 返回非 const 指针，
    // 任意包含者都能在中途改写标定，产生一种无法被断言捕获的姿态缓慢漂移。
    const aircraft::data::CameraChannel* ch = rig.getCamera(CameraRole::CAM25);
    static_assert(
        std::is_same<decltype(ch),
                     const aircraft::data::CameraChannel*>::value,
        "getCamera 必须返回 const 指针（ENG-09 §5.2 / C-18 只读语义）");
    SUCCEED();
}

// ===========================================================================
//  CalibrationManager
// ===========================================================================

TEST_F(CalibrationDirFixture, LoadsAllSevenFiles)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir())) << cm.lastErrorText();

    EXPECT_TRUE(cm.loaded());
    EXPECT_TRUE(cm.lastErrorText().empty());
}

TEST_F(CalibrationDirFixture, ConvertsMillimetresToMetres)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));

    // ⚠ 核心判据：文件里写的是毫米（1/2/3 与 200/400），
    // 进入内存必须是米。若漏掉换算，值会大 1000 倍。
    const auto rigToShip = cm.getRigToShip();
    EXPECT_TRUE(near(rigToShip.translation[0], 0.001, 1e-12));
    EXPECT_TRUE(near(rigToShip.translation[1], 0.002, 1e-12));
    EXPECT_TRUE(near(rigToShip.translation[2], 0.003, 1e-12));

    const auto c50 = cm.getCalibration(CameraRole::CAM50);
    EXPECT_TRUE(near(c50.cameraToRig.translation[0], 0.2, 1e-12));

    const auto c100 = cm.getCalibration(CameraRole::CAM100);
    EXPECT_TRUE(near(c100.cameraToRig.translation[0], 0.4, 1e-12));
}

TEST_F(CalibrationDirFixture, DoesNotConvertIntrinsics)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));

    // 内参矩阵的单位是**像素**，ENG-09 §2.3 的换算规则不适用于它。
    // 若把同一条规则误用到内参上，fx 会从 2000 变成 2，
    // 所有投影退化到主点附近（表现为"看不到目标"而非数值错误）。
    const auto k50 = cm.getCalibration(CameraRole::CAM50);
    ASSERT_EQ(k50.cameraMatrix.rows, 3);
    ASSERT_EQ(k50.cameraMatrix.cols, 3);
    EXPECT_TRUE(near(k50.cameraMatrix.at<double>(0, 0), 2000.0, 1e-9));
    EXPECT_TRUE(near(k50.cameraMatrix.at<double>(1, 1), 2000.0, 1e-9));
}

TEST_F(CalibrationDirFixture, ReadsResolutionAndRotation)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));

    const auto k25 = cm.getCalibration(CameraRole::CAM25);
    EXPECT_EQ(k25.imageWidth,  1280);
    EXPECT_EQ(k25.imageHeight, 1024);

    const auto r = cm.getRigToShip().rotation;
    EXPECT_TRUE(near(r(0, 0),  0.0, 1e-12));
    EXPECT_TRUE(near(r(0, 1), -1.0, 1e-12));
    EXPECT_TRUE(near(r(1, 0),  1.0, 1e-12));
    EXPECT_TRUE(near(r(1, 1),  0.0, 1e-12));
}

TEST_F(CalibrationDirFixture, CalibrationIdComesFromTheFile)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));

    // 版本号取自标定文件而非配置：若只信配置，则标定目录被整包替换
    // 而配置未更新时，结果包会记录一个与实际数据不符的版本号。
    EXPECT_EQ(cm.calibrationId(), "CALIB-TEST-0001");
}

TEST_F(CalibrationDirFixture, MissingExtrinsicFileFailsStartup)
{
    removeExtrinsics();  // 缺 3 个外参与 rig_to_ship

    CalibrationManager cm;
    // ENG-10 §5.3 第一类：文件缺失 → 启动失败（不是"用默认值继续"）。
    EXPECT_FALSE(cm.load(dir()));
    EXPECT_FALSE(cm.loaded());
    EXPECT_FALSE(cm.lastErrorText().empty());
}

TEST_F(CalibrationDirFixture, MissingRigToShipFileFailsStartup)
{
    removeFile("rig_to_ship.yaml");

    CalibrationManager cm;
    // rig_to_ship 是坐标链的最后一环。缺它而继续运行会让所有姿态
    // 都停在 rig 系（相机机架）而非舰体系 —— 数值上看起来仍然"合理"。
    EXPECT_FALSE(cm.load(dir()));
    EXPECT_FALSE(cm.loaded());
}

TEST(CalibrationManagerTest, EmptyDirFailsStartup)
{
    CalibrationManager cm;
    EXPECT_FALSE(cm.load(""));
    EXPECT_FALSE(cm.loaded());
}

TEST(CalibrationManagerTest, NonExistentDirFailsStartup)
{
    CalibrationManager cm;
    EXPECT_FALSE(cm.load("/nonexistent/aps/calibration/dir"));
    EXPECT_FALSE(cm.loaded());
}

TEST(CalibrationManagerTest, SyntheticCalibrationIsLabelledAndWarned)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.loadDefaults(1280, 1024));

    // 合成标定必须自报家门：一个看起来正常但物理上无意义的标定
    // 若不带标识，其姿态结果会被当成真实测量结果使用。
    EXPECT_EQ(cm.calibrationId(), "SYNTHETIC-NO-CALIBRATION");
    EXPECT_FALSE(cm.lastWarnings().empty());

    const auto k = cm.getCalibration(CameraRole::CAM25);
    EXPECT_TRUE(k.cameraMatrix.at<double>(0, 0) > 0.0);
    EXPECT_EQ(k.imageWidth, 1280);
}

TEST(CalibrationManagerTest, SyntheticFocalLengthFollowsFromFieldOfView)
{
    // ⚠ 只经 loadDefaults() 这条**唯一**的合成通路取值：
    // makeSyntheticCalibration 被刻意设为 private，以免生产路径绕过
    // ENG-10 §5.3 的"文件缺失 → 启动失败"而静默使用合成标定。
    // 测试不应为了取一个数而放宽该约束。
    CalibrationManager cm;
    ASSERT_TRUE(cm.loadDefaults(1280, 1024));

    const auto c = cm.getCalibration(CameraRole::CAM25);

    // fx = (W/2) / tan(HFOV/2)，HFOV = 45° → fx = 640 / tan(22.5°)
    const double expected = 640.0 / std::tan(22.5 * kPi / 180.0);
    EXPECT_TRUE(near(c.cameraMatrix.at<double>(0, 0), expected, 1e-9));

    // 主点在画幅中心
    EXPECT_TRUE(near(c.cameraMatrix.at<double>(0, 2), 640.0, 1e-9));
    EXPECT_TRUE(near(c.cameraMatrix.at<double>(1, 2), 512.0, 1e-9));
}

// ===========================================================================
//  CoordinateTransformer
// ===========================================================================

TEST_F(CalibrationDirFixture, ComposesRotationChainInTheFrozenOrder)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));
    CoordinateTransformer ct(cm);

    // cam25 外参为单位变换，故合成的旋转 = rigToShip(Z90) · I · pose(Z10)
    //                                     = Z100
    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation          = rotZ(10.0);
    pose.aircraftToCamera.translation       = cv::Vec3d(0.0, 0.0, 50.0);
    pose.reprojectionError                  = 0.37;

    const auto r = ct.transform(CameraRole::CAM25, pose);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(near(r.yaw,   100.0, 1e-9));
    EXPECT_TRUE(near(r.pitch,   0.0, 1e-9));
    EXPECT_TRUE(near(r.roll,    0.0, 1e-9));
}

TEST_F(CalibrationDirFixture, ComposesTranslationLevelByLevel)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));
    CoordinateTransformer ct(cm);

    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation    = rotZ(10.0);
    pose.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 50.0);

    // cam50 的外参含 0.2 m 基线，且被 Rz(90) 从 x 轴转到 y 轴：
    //   t_afterRig = (0.2,0,0) + (0,0,50)            = (0.2, 0, 50)
    //   t_ship     = (0.001,0.002,0.003) + Rz(90)·…  = (0.001, 0.202, 50.003)
    // 直接把旋转的乘法顺序照抄给平移（不做逐级变换）会得到 (0.201, 0.002, 50.003)。
    const auto r = ct.transform(CameraRole::CAM50, pose);

    ASSERT_TRUE(r.success);
    EXPECT_TRUE(near(r.aircraftToShip.translation[0], 0.001, 1e-9));
    EXPECT_TRUE(near(r.aircraftToShip.translation[1], 0.202, 1e-9));
    EXPECT_TRUE(near(r.aircraftToShip.translation[2], 50.003, 1e-9));
}

TEST_F(CalibrationDirFixture, UsesPerCameraExtrinsics)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));
    CoordinateTransformer ct(cm);

    // 三台相机的 cameraToRig 是三个独立文件。同一个输入姿态经不同相机
    // 变换后，平移必须不同 —— 若本类固定用某一台的外参，
    // 三者的结果会完全相同，且错误量级恰为基线视差（看起来像"小偏差"）。
    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation    = rotZ(10.0);
    pose.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 50.0);

    const auto a = ct.transform(CameraRole::CAM25,  pose);
    const auto b = ct.transform(CameraRole::CAM50,  pose);
    const auto c = ct.transform(CameraRole::CAM100, pose);

    ASSERT_TRUE(a.success && b.success && c.success);
    EXPECT_FALSE(near(a.aircraftToShip.translation[1],
                      b.aircraftToShip.translation[1], 1e-9));
    EXPECT_FALSE(near(b.aircraftToShip.translation[1],
                      c.aircraftToShip.translation[1], 1e-9));

    EXPECT_TRUE(near(ct.cameraToRig(CameraRole::CAM50).translation[0],  0.2, 1e-12));
    EXPECT_TRUE(near(ct.cameraToRig(CameraRole::CAM100).translation[0], 0.4, 1e-12));
}

TEST_F(CalibrationDirFixture, CarriesReprojectionErrorForward)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.load(dir()));
    CoordinateTransformer ct(cm);

    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation    = rotZ(5.0);
    pose.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 100.0);
    pose.reprojectionError            = 0.37;

    // VALIDATE 状态要用它（SYS-07 §12.2）；此处丢弃会迫使验证阶段重算。
    EXPECT_TRUE(near(ct.transform(CameraRole::CAM25, pose).reprojectionError,
                     0.37, 1e-12));
}

TEST(CoordinateTransformerTest, RejectsNonRotationMatrix)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.loadDefaults(1280, 1024));
    CoordinateTransformer ct(cm);

    // 把外参文件的 rotation 填成欧拉角（格式误会）：矩阵仍为 3x3、
    // 元素有限，只有正交性检查能拦住它。
    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation = cv::Matx33d(0.1, 0.2, 0.3,
                                                 0.4, 0.5, 0.6,
                                                 0.7, 0.8, 0.9);

    const auto r = ct.transform(CameraRole::CAM25, pose);
    EXPECT_FALSE(r.success);
}

TEST(CoordinateTransformerTest, RejectsNonFiniteInput)
{
    CalibrationManager cm;
    ASSERT_TRUE(cm.loadDefaults(1280, 1024));
    CoordinateTransformer ct(cm);

    // NaN 参与比较恒为 false，于是所有阈值判据都"不通过"，
    // 表现为验证失败但看不出根因；且序列化会得到 "nan"。
    aircraft::data::CameraPose pose;
    pose.aircraftToCamera.rotation = cv::Matx33d::eye();
    pose.aircraftToCamera.rotation(0, 0) = std::nan("");
    pose.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 50.0);

    EXPECT_FALSE(ct.transform(CameraRole::CAM25, pose).success);
}

TEST(CoordinateTransformerTest, DecomposesYawPitchRollInZyxOrder)
{
    double y = 0.0, p = 0.0, r = 0.0;

    ASSERT_TRUE(CoordinateTransformer::decomposeZyx(rotZ(30.0), y, p, r));
    EXPECT_TRUE(near(y, 30.0, 1e-9));
    EXPECT_TRUE(near(p,  0.0, 1e-9));
    EXPECT_TRUE(near(r,  0.0, 1e-9));

    // 绕 Y 转 20°：pitch = +20，yaw = roll = 0
    const double a = 20.0 * kPi / 180.0;
    const cv::Matx33d ry(std::cos(a), 0.0, std::sin(a),
                         0.0, 1.0, 0.0,
                        -std::sin(a), 0.0, std::cos(a));
    ASSERT_TRUE(CoordinateTransformer::decomposeZyx(ry, y, p, r));
    EXPECT_TRUE(near(p, 20.0, 1e-9));
    EXPECT_TRUE(near(y,  0.0, 1e-9));
}

TEST(CoordinateTransformerTest, ReportsAnglesInDegrees)
{
    double y = 0.0, p = 0.0, r = 0.0;
    ASSERT_TRUE(CoordinateTransformer::decomposeZyx(rotZ(1.0), y, p, r));

    // ENG-09 §2.2：角度一律为度。1° 若被当成弧度输出会得到 0.017453。
    EXPECT_TRUE(near(y, 1.0, 1e-9));
    // 1 角分 = 1/60 度（本系统的指标量级，用于确认量纲不失真）
    double ym = 0.0, pm = 0.0, rm = 0.0;
    ASSERT_TRUE(CoordinateTransformer::decomposeZyx(rotZ(1.0 / 60.0), ym, pm, rm));
    EXPECT_TRUE(near(ym, 0.016666666666666666, 1e-12));
}

TEST(CoordinateTransformerTest, RejectsGimbalLockInsteadOfInventingASolution)
{
    double y = 0.0, p = 0.0, r = 0.0;

    // pitch = +90°：yaw 与 roll 退化为只由二者之差决定，无法分别确定。
    // 编造一个 yaw=0 的解会在数值上自洽但与真实姿态不同。
    const double h = kPi / 2.0;
    const cv::Matx33d ry90(std::cos(h), 0.0, std::sin(h),
                           0.0, 1.0, 0.0,
                          -std::sin(h), 0.0, std::cos(h));
    EXPECT_FALSE(CoordinateTransformer::decomposeZyx(ry90, y, p, r));

    // 接近但不等于：应正常分解
    const double h2 = 89.0 * kPi / 180.0;
    const cv::Matx33d ry89(std::cos(h2), 0.0, std::sin(h2),
                           0.0, 1.0, 0.0,
                          -std::sin(h2), 0.0, std::cos(h2));
    EXPECT_TRUE(CoordinateTransformer::decomposeZyx(ry89, y, p, r));
    EXPECT_TRUE(near(p, 89.0, 1e-9));
}

// ===========================================================================
//  CameraSynchronizer
// ===========================================================================

namespace
{

aircraft::data::ImageFrame makeFrame(uint64_t frameId,
                                     uint64_t timestampNs,
                                     int gray = 128)
{
    aircraft::data::ImageFrame f;
    f.image         = cv::Mat(8, 8, CV_8UC1, cv::Scalar(gray));
    f.frameId       = frameId;
    f.timestampNs   = timestampNs;
    f.deviceTimestampNs = timestampNs;
    return f;
}

aircraft::data::ImageFrame emptyFrame()
{
    return aircraft::data::ImageFrame{};
}

aircraft::data::TriggerConfig makeTriggerConfig(uint64_t toleranceNs)
{
    aircraft::data::TriggerConfig c;
    c.source          = "virtual";
    c.periodMs        = 100.0;
    c.syncToleranceNs = toleranceNs;
    return c;
}

}  // namespace

TEST(CameraSynchronizerTest, CombinesThreeSynchronisedFrames)
{
    CameraSynchronizer sync(makeTriggerConfig(500));

    aircraft::data::MultiCameraFrame out;
    ASSERT_TRUE(sync.synchronize(makeFrame(1, 1000, 10),
                                 makeFrame(1, 1100, 20),
                                 makeFrame(1, 1200, 30), out));

    EXPECT_EQ(sync.lastPresentCount(), 3);
    EXPECT_EQ(sync.lastSpreadNs(), 200u);

    // ENG-09 §2.5：triggerTimestamp = 各路 timestampNs 的最大值。
    EXPECT_EQ(out.triggerTimestamp, 1200u);

    // 三路图像都被搬进输出
    EXPECT_FALSE(out.cam25.image.empty());
    EXPECT_FALSE(out.cam50.image.empty());
    EXPECT_FALSE(out.cam100.image.empty());
    EXPECT_EQ(out.cam25.image.at<unsigned char>(0, 0), 10);
    EXPECT_EQ(out.cam100.image.at<unsigned char>(0, 0), 30);
}

TEST(CameraSynchronizerTest, AcceptsSpreadExactlyAtTolerance)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    // 边界：极差 == 容差应通过（判据是 "> 容差" 才失败）。
    EXPECT_TRUE(sync.synchronize(makeFrame(1, 0),
                                 makeFrame(1, 250),
                                 makeFrame(1, 500), out));
}

TEST(CameraSynchronizerTest, RejectsSpreadBeyondTolerance)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    EXPECT_FALSE(sync.synchronize(makeFrame(1, 0),
                                  makeFrame(1, 250),
                                  makeFrame(1, 501), out));

    // 裁决 C-006 已为"三相机时间戳超差"登记 3002（ENG-09 §5.27 的 3000 段）。
    // 此前这里断言的是 0，理由是"3000 段没有这一码" —— 而 0 的冻结语义是
    // "未设置"、在 result.json 中渲染为 "OK"，于是一次**真实的同步超差**
    // 在包里与"没有错误"无法区分。这正是 C-006 要消除的模糊。
    EXPECT_EQ(sync.lastError().code, aircraft::data::kErrSyncOutOfTolerance);
    EXPECT_FALSE(sync.lastError().message.empty());
}

TEST(CameraSynchronizerTest, SpreadIsMeasuredAcrossPresentChannelsOnly)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    // cam100 不可用时它的 timestampNs 为 0。若把它算进极差，
    // 极差会变成 1200 - 0 = 1200 > 500，于是两相机降级下**每一轮都不同步**，
    // 表现为"拔掉一台相机系统就彻底不工作"。
    ASSERT_TRUE(sync.synchronize(makeFrame(1, 1000),
                                 emptyFrame(),
                                 makeFrame(1, 1200), out));

    EXPECT_EQ(sync.lastPresentCount(), 2);
    EXPECT_EQ(sync.lastSpreadNs(), 200u);   // 1000 → 1200，不是 0 → 1200
    EXPECT_EQ(out.triggerTimestamp, 1200u);
}

TEST(CameraSynchronizerTest, TwoCamerasDegradeAndContinue)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    // SYS-08 §7.5：2 台 → 降级并继续测量。
    EXPECT_TRUE(sync.synchronize(makeFrame(1, 1000),
                                 makeFrame(1, 1050),
                                 emptyFrame(), out));
    EXPECT_EQ(sync.lastPresentCount(), 2);

    // 另两路的组合同样成立（确认判据与路的排列无关）
    EXPECT_TRUE(sync.synchronize(makeFrame(2, 2000),
                                 emptyFrame(),
                                 makeFrame(2, 2050), out));
    EXPECT_EQ(sync.lastPresentCount(), 2);
}

TEST(CameraSynchronizerTest, SingleCameraFailsWithInsufficientCode)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    // SYS-08 §7.5：<=1 台 → 直接 FAILED，code = 1001。
    EXPECT_FALSE(sync.synchronize(makeFrame(1, 1000),
                                  emptyFrame(),
                                  emptyFrame(), out));
    EXPECT_EQ(sync.lastError().code, aircraft::data::kErrCameraInsufficient);
    EXPECT_STREQ(aircraft::data::errorCodeName(sync.lastError().code),
                 "ErrCameraInsufficient");
}

TEST(CameraSynchronizerTest, ZeroCamerasFailsWithInsufficientCode)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    EXPECT_FALSE(sync.synchronize(emptyFrame(), emptyFrame(), emptyFrame(), out));
    EXPECT_EQ(sync.lastError().code, aircraft::data::kErrCameraInsufficient);
}

TEST(CameraSynchronizerTest, UnconfiguredToleranceIsReportedNotSilentlyPassed)
{
    CameraSynchronizer sync(makeTriggerConfig(0));
    aircraft::data::MultiCameraFrame out;

    // syncToleranceNs = 0 视为"未配置"：按 0 严格执行会让三路时间戳
    // 必须逐纳秒相同 —— 即**每一轮都失败**，系统完全无法测量，
    // 而根因只是一处未填的配置。故跳过判据，但必须留下记录。
    EXPECT_TRUE(sync.synchronize(makeFrame(1, 0),
                                 makeFrame(1, 999999),
                                 makeFrame(1, 0), out));
    EXPECT_FALSE(sync.lastError().message.empty());
}

TEST(CameraSynchronizerTest, DetectsFrameDropButDoesNotFail)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    ASSERT_TRUE(sync.synchronize(makeFrame(1, 1000),
                                 makeFrame(1, 1000),
                                 makeFrame(1, 1000), out));
    EXPECT_FALSE(sync.lastFrameDropDetected()) << "首轮无前值可比，不应报丢帧";

    ASSERT_TRUE(sync.synchronize(makeFrame(2, 2000),
                                 makeFrame(2, 2000),
                                 makeFrame(2, 2000), out));
    EXPECT_FALSE(sync.lastFrameDropDetected()) << "逐帧 +1 应视为连续";

    // CAM100 跳号：丢帧应被检出，但**不**导致失败 ——
    // 三路仍然同步，只是 CAPTURE 的可平均帧数少了；重采的代价
    // （转台重新稳定 3.0 s）远大于少一帧的收益。
    ASSERT_TRUE(sync.synchronize(makeFrame(3, 3000),
                                 makeFrame(3, 3000),
                                 makeFrame(5, 3000), out));
    EXPECT_TRUE(sync.lastFrameDropDetected());
}

TEST(CameraSynchronizerTest, ResetOfContinuityAfterDisconnect)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    ASSERT_TRUE(sync.synchronize(makeFrame(10, 1000),
                                 makeFrame(10, 1000),
                                 makeFrame(10, 1000), out));

    // cam100 断连一轮（帧号不再推进）
    ASSERT_TRUE(sync.synchronize(makeFrame(11, 2000),
                                 makeFrame(11, 2000),
                                 emptyFrame(), out));

    // 恢复后帧号从 100 继续。断连期间本就没采集，故不应报"丢帧 85 帧"。
    ASSERT_TRUE(sync.synchronize(makeFrame(12, 3000),
                                 makeFrame(12, 3000),
                                 makeFrame(100, 3000), out));
    EXPECT_FALSE(sync.lastFrameDropDetected())
        << "断连期间的缺帧不应计为丢帧（该通道当时本不可用）";
}

TEST(CameraSynchronizerTest, FailureLeavesErrorClearedOnNextSuccess)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    ASSERT_FALSE(sync.synchronize(makeFrame(1, 0), emptyFrame(), emptyFrame(), out));
    ASSERT_EQ(sync.lastError().code, aircraft::data::kErrCameraInsufficient);

    // 成功一次后，上一次的错误码不得残留 —— 否则 UI 与 result.json
    // 会把一个已经恢复的故障一直报下去。
    ASSERT_TRUE(sync.synchronize(makeFrame(2, 0), makeFrame(2, 0), emptyFrame(), out));
    EXPECT_EQ(sync.lastError().code, 0);
}

TEST(CameraSynchronizerTest, OutputIsOverwrittenNotAccumulated)
{
    CameraSynchronizer sync(makeTriggerConfig(500));
    aircraft::data::MultiCameraFrame out;

    ASSERT_TRUE(sync.synchronize(makeFrame(1, 1000),
                                 makeFrame(1, 1000),
                                 makeFrame(1, 1000), out));
    ASSERT_FALSE(out.cam100.image.empty());

    ASSERT_TRUE(sync.synchronize(makeFrame(2, 2000),
                                 makeFrame(2, 2000),
                                 emptyFrame(), out));

    // 第二轮 cam100 不可用，输出里不得残留第一轮的图像与帧号 ——
    // 残留会让下游用上一帧的图像配这一帧的时间戳。
    EXPECT_TRUE(out.cam100.image.empty());
    EXPECT_EQ(out.cam100.frameId, 0u);
}

}  // namespace
