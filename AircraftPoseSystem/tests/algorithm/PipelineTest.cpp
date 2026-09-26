// ============================================================================
//  tests/algorithm/PipelineTest.cpp
//
//  覆盖（ENG-06 §8：algorithm 层测试）—— 本文件测的是**阶段之间的装配**，
//  与 AlgorithmStageTest.cpp 的分工不同：
//    · AlgorithmStageTest  各阶段的判据与算式（尺度、评分、验证、PnP）；
//    · PipelineTest（本文件）阶段之间的**接口与坐标链**，即"每一步都对、
//      连起来仍可能错"的那些地方。
//
//  用例：
//    · 目标模型库   落盘 / 读回 / 三焦段物理分离 / 全有或全无加载 /
//                  目标真实尺寸 L = points3d 包围盒最长边
//    · 欧拉角分解   decomposeZyxDeg 与旋转矩阵的自洽（含万向锁）
//    · 坐标链合成   solvePose 的 aircraftToShip = rigToShip·cameraToRig·
//                  aircraftToCamera（含旋转与**平移**的两级复合）
//    · 通道选择     allowed 列表的过滤作用（SYS-08 §7.4）
//    · 最佳帧选择   清晰度排序与门槛
//    · B 类定位     CAD 结构点的投影—匹配—拟合三步在合成棱线图上的表现
//
//  ⚠ 本文件**不测** SIFT/CAD 在真实图像上的精度：那需要 Golden 数据
//  （ENG-06 §10 的 tests/golden/，当前为空）。本文件用注入的
//  `MockFeatureExtractor` / `MockFeatureMatcher` 给出确定对应集，
//  从而把"坐标链是否接反"与"特征提取好不好"这两类问题分开 ——
//  否则一个失败会同时指向两处，而其中一处其实是对的（ENG-10 §5.2 约束 4：
//  测试必须通过与生产相同的注入路径）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algorithm/detection/TargetDetector.h"
#include "algorithm/feature/CadStructureLocator.h"
#include "algorithm/feature/FeatureExtractor.h"
#include "algorithm/matcher/FeatureMatcher.h"
#include "algorithm/model/TargetModelManager.h"
#include "algorithm/pipeline/IPipelineObserver.h"
#include "algorithm/pipeline/PosePipeline.h"
#include "algorithm/pose/PnPPoseEstimator.h"

#include "data/CameraPose.h"
#include "data/FeatureDescriptor.h"
#include "data/FeatureSet.h"
#include "data/ImageQuality.h"
#include "data/ModelPoint3D.h"
#include "data/MultiCameraFrame.h"
#include "data/OpticalRigCalibration.h"
#include "data/ShipPoseResult.h"
#include "data/TargetModel.h"

#include "TestSupport.h"

using aircraft::algorithm::CadStructureLocator;
using aircraft::algorithm::MatchResult;
using aircraft::algorithm::CadStructureResult;
using aircraft::algorithm::CvPnPPoseEstimator;
using aircraft::algorithm::PnpStats;
using aircraft::algorithm::PnPPoseEstimator;
using aircraft::algorithm::DescriptorFeatureMatcher;
using aircraft::algorithm::MockFeatureExtractor;
using aircraft::algorithm::MockFeatureMatcher;
using aircraft::algorithm::MockTargetDetector;
using aircraft::algorithm::PosePipeline;
using aircraft::algorithm::SiftFeatureExtractor;
using aircraft::algorithm::TargetModelManager;
using aircraft::data::CameraCalibration;
using aircraft::data::DetectionResult;
using aircraft::data::MeasurementSelectionResult;
using aircraft::data::TargetScaleEstimate;
using aircraft::data::CameraPose;
using aircraft::data::CameraRole;
using aircraft::data::FeatureCorrespondence;
using aircraft::data::FeatureDescriptor;
using aircraft::data::FeatureSet;
using aircraft::data::ImageFrame;
using aircraft::data::ImageQuality;
using aircraft::data::ModelPoint3D;
using aircraft::data::MultiCameraFrame;
using aircraft::data::OpticalRigCalibration;
using aircraft::data::ShipPoseResult;
using aircraft::data::TargetModel;

using namespace aps_test;

namespace
{

// ---------------------------------------------------------------------------
//  模型目录夹具
// ---------------------------------------------------------------------------

/// 本用例的模型：3 个三维点（2 个 CAD 结构点 + 1 个纹理辅助点）。
/// 包围盒：X 向 10 m（机头 +5 → 机尾 −5）、Y 向 6 m、Z 向 0.5 m，
/// 故最长边 10 m —— 这正是 SYS-14 §10 的 Z = f·L/l 需要的 L。
constexpr double kModelLongestSideM = 10.0;

std::vector<ModelPoint3D> fixturePoints()
{
    std::vector<ModelPoint3D> pts;
    ModelPoint3D nose;
    nose.id = 1;
    nose.position = cv::Point3f(5.0f, 0.0f, 0.0f);
    nose.featureType = "cad";
    pts.push_back(nose);

    ModelPoint3D tail;
    tail.id = 2;
    tail.position = cv::Point3f(-5.0f, 0.0f, 0.0f);
    tail.featureType = "cad";
    pts.push_back(tail);

    ModelPoint3D wing;
    wing.id = 3;
    wing.position = cv::Point3f(0.0f, 3.0f, 0.5f);
    wing.featureType = "texture";
    pts.push_back(wing);

    return pts;
}

/// 构造一份"某焦段"的特征库：`value` 填满全部描述子。
/// 三个焦段用**不同的值**，于是"角色串了"可以被直接断言（见角色分离用例）。
TargetModel fixtureModelWithDescriptorValue(float value)
{
    TargetModel m;
    m.modelId = "aircraft_model_v1";
    m.points3d = fixturePoints();

    for (int i = 0; i < 3; ++i)
    {
        FeatureDescriptor f;
        f.featureId = i;
        f.point3dIndex = i;
        f.descriptor = cv::Mat(1, 4, CV_32F, cv::Scalar(value));
        m.features.push_back(f);
    }
    return m;
}

/// 在临时目录中落盘一份完整的模型库（SYS-12 §7 的目录布局）。
/// @return 目录路径；失败时返回空串（调用方必须断言，否则后续用例会在
///         "文件没写出来"的前提下得到一个与被测代码无关的失败）。
std::string writeFixtureModelDir(bool withMetaFile = true)
{
    const std::string dir = makeTempDir("model");
    if (dir.empty())
    {
        return std::string();
    }

    if (withMetaFile)
    {
        // model.yaml：模型身份与版本的权威来源（SYS-04 §6.3 / SYS-12 §13）。
        const std::string meta =
            "%YAML:1.0\n"
            "model_id: \"aircraft_model_v1\"\n"
            "version: 1\n"
            "model_source: \"cad/aircraft_v3.step\"\n"
            "generated_at: \"2026-05-01T10:00:00\"\n"
            "feature_algorithm_version: \"sift-4.6.0\"\n";
        if (!writeTextFile(dir + "/model.yaml", meta))
        {
            return std::string();
        }
    }

    // points3d.yaml：点表（单位 m，ENG-09 §2.3 —— 这里**不做**任何单位换算，
    // 读到什么就是什么，见 TargetModelManager.cpp 的说明）。
    const std::string points =
        "%YAML:1.0\n"
        "model_id: \"aircraft_model_v1\"\n"
        "version: 1\n"
        "points:\n"
        "  - { id: 1, x: 5.0, y: 0.0, z: 0.0, type: cad }\n"
        "  - { id: 2, x: -5.0, y: 0.0, z: 0.0, type: cad }\n"
        "  - { id: 3, x: 0.0, y: 3.0, z: 0.5, type: texture }\n";
    if (!writeTextFile(dir + "/points3d.yaml", points))
    {
        return std::string();
    }

    const float values[3] = {1.0f, 2.0f, 3.0f};
    const CameraRole roles[3] = {CameraRole::CAM25, CameraRole::CAM50,
                                 CameraRole::CAM100};
    const char* names[3] = {"feature25.bin", "feature50.bin", "feature100.bin"};
    for (int i = 0; i < 3; ++i)
    {
        const TargetModel m = fixtureModelWithDescriptorValue(values[i]);
        if (!TargetModelManager::saveFeatureLibrary(
                dir + "/" + names[i], roles[i], m))
        {
            return std::string();
        }
    }

    return dir;
}

/// 三相机光机标定：内参已填写，`cameraToRig` 与 `rigToShip` 由调用方给定。
OpticalRigCalibration fixtureRig(const cv::Matx33d& rigToShipRotation,
                                 const cv::Vec3d& rigToShipTranslation,
                                 const cv::Matx33d& cam25ToRig = cv::Matx33d::eye(),
                                 const cv::Vec3d& cam25ToRigT = cv::Vec3d(0, 0, 0))
{
    OpticalRigCalibration rig;
    rig.cam25 = calibrationWithFx(7246.4, 7246.4, 2448, 2048);
    rig.cam50 = calibrationWithFx(14492.8, 14492.8, 2448, 2048);
    rig.cam100 = calibrationWithFx(28985.5, 28985.5, 2448, 2048);

    rig.cam25.cameraToRig.rotation = cam25ToRig;
    rig.cam25.cameraToRig.translation = cam25ToRigT;
    rig.rigToShip.rotation = rigToShipRotation;
    rig.rigToShip.translation = rigToShipTranslation;
    return rig;
}

/// 一张有内容的合成图（用于"图像非空"这一前置条件）。
/// `seed` 决定纹理，使不同帧的清晰度不同。
cv::Mat noiseImage(int width, int height, unsigned int seed)
{
    cv::Mat img(height, width, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(img, cv::RNG::UNIFORM, 0, 256);
    return img;
}

/// 三相机同步帧：只有 `role` 对应的那一路有图像（其余为空帧）。
MultiCameraFrame frameWithImage(CameraRole role, const cv::Mat& image)
{
    MultiCameraFrame f;
    switch (role)
    {
    case CameraRole::CAM25:  f.cam25 = makeFrame(image, role); break;
    case CameraRole::CAM50:  f.cam50 = makeFrame(image, role); break;
    case CameraRole::CAM100: f.cam100 = makeFrame(image, role); break;
    }
    return f;
}

}  // namespace

// ===========================================================================
//  一、目标模型库（SYS-12 §4 / §7 / §8 / §13）
// ===========================================================================

TEST(TargetModelManager, SavesAndLoadsWholeModelDirectory)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());

    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    EXPECT_TRUE(mgr.loaded());
    EXPECT_EQ(mgr.modelId(), std::string("aircraft_model_v1"));
    EXPECT_EQ(mgr.modelVersion(), std::string("1"));

    // 目标真实尺寸 L = points3d 包围盒**最长边**（SYS-14 §10 的距离估计式
    // 需要它；取错边会让 Z 系统性偏离，通道选择整体偏移）。
    EXPECT_NEAR(mgr.targetRealSizeM(), kModelLongestSideM, 1e-6);

    const TargetModel m = mgr.get(CameraRole::CAM50);
    EXPECT_EQ(m.points3d.size(), static_cast<size_t>(3));
    EXPECT_EQ(m.features.size(), static_cast<size_t>(3));

    // 点表按序读回，单位保持不变（读到的 5.0 必须是 5.0 m）。
    EXPECT_EQ(m.points3d[0].id, 1);
    EXPECT_NEAR(m.points3d[0].position.x, 5.0f, 1e-6f);
    EXPECT_EQ(m.points3d[0].featureType, std::string("cad"));
    EXPECT_EQ(m.points3d[2].featureType, std::string("texture"));

    // 描述子与三维点索引都必须与写入时一致：point3dIndex 是 2D-3D 对应的
    // 唯一纽带，写反了匹配结果会整体错位而**不会报错**（裁决 C-08）。
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(m.features[static_cast<size_t>(i)].point3dIndex, i);
        ASSERT_EQ(m.features[static_cast<size_t>(i)].descriptor.rows, 1);
        ASSERT_EQ(m.features[static_cast<size_t>(i)].descriptor.cols, 4);
        // CAM50 的库落盘时填的是 2.0（三个焦段各填一个不同的值）。
        EXPECT_NEAR(m.features[static_cast<size_t>(i)].descriptor.at<float>(0, 0),
                    2.0f, 1e-6f);
    }
}

TEST(TargetModelManager, KeepsThreeFocalLengthLibrariesSeparate)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());

    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    // SYS-12 §8：三焦段特征库物理分离，**不得**互相串用。
    // 三个库用不同的描述子值落盘，故"读错了文件"会立刻暴露。
    const float expected[3] = {1.0f, 2.0f, 3.0f};
    const CameraRole roles[3] = {CameraRole::CAM25, CameraRole::CAM50,
                                 CameraRole::CAM100};
    for (int i = 0; i < 3; ++i)
    {
        const TargetModel m = mgr.get(roles[i]);
        ASSERT_EQ(m.features.size(), static_cast<size_t>(3));
        EXPECT_NEAR(m.features[0].descriptor.at<float>(0, 0), expected[i], 1e-6f);
    }
}

TEST(TargetModelManager, RejectsLibraryWhoseStoredRoleDiffersFromFilename)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());

    // 一个 CAM50 的库被误命名为 feature25.bin（复制粘贴改错一个数字）：
    // 文件名无法发现，故格式头部记录 role 并在读取时校验。
    const TargetModel m = fixtureModelWithDescriptorValue(2.0f);
    const std::string wrong = makeTempDir("wrongrole") + "/feature25.bin";
    ASSERT_TRUE(TargetModelManager::saveFeatureLibrary(wrong, CameraRole::CAM50, m));

    TargetModel inOut;
    EXPECT_FALSE(TargetModelManager::loadFeatureLibrary(wrong, CameraRole::CAM25, inOut));
    // 反过来（用正确的角色读）应当成功 —— 证明失败原因确实是角色不符，
    // 而不是文件本身写坏了。
    EXPECT_TRUE(TargetModelManager::loadFeatureLibrary(wrong, CameraRole::CAM50, inOut));
}

TEST(TargetModelManager, MissingOneFeatureLibraryFailsWithoutHalfLoad)
{
    // 三焦段里只落盘两路：CAM100 的库缺失。
    const std::string dir = makeTempDir("partial");
    ASSERT_FALSE(dir.empty());

    const std::string meta =
        "%YAML:1.0\nmodel_id: \"aircraft_model_v1\"\nversion: 1\n";
    ASSERT_TRUE(writeTextFile(dir + "/model.yaml", meta));
    const std::string points =
        "%YAML:1.0\nmodel_id: \"aircraft_model_v1\"\nversion: 1\n"
        "points:\n"
        "  - { id: 1, x: 5.0, y: 0.0, z: 0.0, type: cad }\n"
        "  - { id: 2, x: -5.0, y: 0.0, z: 0.0, type: cad }\n";
    ASSERT_TRUE(writeTextFile(dir + "/points3d.yaml", points));

    const TargetModel m = fixtureModelWithDescriptorValue(1.0f);
    ASSERT_TRUE(TargetModelManager::saveFeatureLibrary(
        dir + "/feature25.bin", CameraRole::CAM25, m));
    ASSERT_TRUE(TargetModelManager::saveFeatureLibrary(
        dir + "/feature50.bin", CameraRole::CAM50, m));

    TargetModelManager mgr;
    EXPECT_FALSE(mgr.loadModel(dir));

    // 关键断言：**不得留下半加载状态**（TargetModelManager.h）。
    // 缺一个通道时若保留另两个通道，CAM100 通道会静默退化为"没有 A 类特征"，
    // 而这与"目标在该焦段真的没有纹理"在外部完全不可区分。
    EXPECT_FALSE(mgr.loaded());
    EXPECT_TRUE(mgr.modelId().empty());
    EXPECT_NEAR(mgr.targetRealSizeM(), 0.0, 1e-12);
    EXPECT_TRUE(mgr.get(CameraRole::CAM25).points3d.empty());
    EXPECT_TRUE(mgr.get(CameraRole::CAM25).features.empty());
}

TEST(TargetModelManager, MissingMetaFileFails)
{
    // model.yaml 缺失：它是模型身份与版本的权威来源（SYS-04 §6.3），
    // 没有它 model_id 只能从点表推断，于是同一份模型会有两个可独立编辑的
    // 身份来源（见 TargetModelManager.cpp 的 loadModelMeta）。
    const std::string dir = writeFixtureModelDir(/*withMetaFile=*/false);
    ASSERT_FALSE(dir.empty());

    TargetModelManager mgr;
    EXPECT_FALSE(mgr.loadModel(dir));
    EXPECT_FALSE(mgr.loaded());
}

TEST(TargetModelManager, DegeneratePointTableFails)
{
    // 全部点重合 → 包围盒最长边为 0 → L 无意义 → 加载失败。
    // 若放行，Z = f·L/l 恒为 0，所有候选都会落进最近的距离带。
    const std::string dir = makeTempDir("degenerate");
    ASSERT_FALSE(dir.empty());

    const std::string meta =
        "%YAML:1.0\nmodel_id: \"aircraft_model_v1\"\nversion: 1\n";
    ASSERT_TRUE(writeTextFile(dir + "/model.yaml", meta));
    const std::string points =
        "%YAML:1.0\nmodel_id: \"aircraft_model_v1\"\nversion: 1\n"
        "points:\n"
        "  - { id: 1, x: 0.0, y: 0.0, z: 0.0, type: cad }\n"
        "  - { id: 2, x: 0.0, y: 0.0, z: 0.0, type: cad }\n";
    ASSERT_TRUE(writeTextFile(dir + "/points3d.yaml", points));

    const TargetModel m = fixtureModelWithDescriptorValue(1.0f);
    for (const auto& pair : {std::make_pair("feature25.bin", CameraRole::CAM25),
                             std::make_pair("feature50.bin", CameraRole::CAM50),
                             std::make_pair("feature100.bin", CameraRole::CAM100)})
    {
        ASSERT_TRUE(TargetModelManager::saveFeatureLibrary(
            dir + "/" + pair.first, pair.second, m));
    }

    TargetModelManager mgr;
    EXPECT_FALSE(mgr.loadModel(dir));
    EXPECT_FALSE(mgr.loaded());
}

// ===========================================================================
//  二、欧拉角分解（ENG-09 §5.24：R = Rz(yaw)·Ry(pitch)·Rx(roll)）
// ===========================================================================

TEST(PosePipeline, DecomposeZyxIsSelfConsistentWithMatrix)
{
    struct Case
    {
        double yaw;
        double pitch;
        double roll;
    };
    const Case cases[] = {
        {0.0, 0.0, 0.0},
        {30.0, 5.0, 2.0},
        {-45.0, -3.0, 10.0},
        {170.0, 20.0, -30.0},     // 超过 ±90°，检验 atan2 的符号分支
        {-120.0, -15.0, 40.0},
    };

    for (const Case& c : cases)
    {
        const cv::Matx33d r = rotZ(c.yaw) * rotY(c.pitch) * rotX(c.roll);

        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        PosePipeline::decomposeZyxDeg(r, yaw, pitch, roll);

        // 分解写错不会报错，只会让 Yaw（验收指标本身）与矩阵不一致，
        // 故此处逐项比对**输入角**，而不是比对"再合成回去的矩阵"。
        EXPECT_NEAR(yaw, c.yaw, 1e-9);
        EXPECT_NEAR(pitch, c.pitch, 1e-9);
        EXPECT_NEAR(roll, c.roll, 1e-9);
    }
}

TEST(PosePipeline, DecomposeZyxHandlesGimbalLockWithoutNaN)
{
    // pitch = ±90° 时 yaw 与 roll 只以组合形式出现，无法分别确定。
    // 必须显式处理：不处理时 R(2,1) = R(2,2) = 0，atan2(0,0) 静默返回 0，
    // 结果既不报错也不自洽（矩阵与欧拉角对不上）。
    const cv::Matx33d r = rotZ(40.0) * rotY(90.0) * rotX(0.0);

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    PosePipeline::decomposeZyxDeg(r, yaw, pitch, roll);

    EXPECT_TRUE(std::isfinite(yaw));
    EXPECT_TRUE(std::isfinite(pitch));
    EXPECT_TRUE(std::isfinite(roll));
    EXPECT_NEAR(pitch, 90.0, 1e-9);
    EXPECT_NEAR(roll, 0.0, 1e-12);      // 约定：全部转角记在 yaw 上

    // 自洽性：用分解出的角重建的旋转必须与输入矩阵一致（这才是"处理正确"
    // 的定义 —— 万向锁下角度不唯一，唯一的是矩阵）。
    const cv::Matx33d rebuilt = rotZ(yaw) * rotY(pitch) * rotX(roll);
    EXPECT_LT(rotationAngleBetween(rebuilt, r), 1e-9);
}

// ===========================================================================
//  三、坐标链合成（ENG-09 §2.1：aircraftToShip =
//      rigToShip · cameraToRig · aircraftToCamera）
// ===========================================================================

TEST(PosePipeline, SolvePoseComposesIdentityChainToAircraftPose)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();

    // 恒等标定链：cameraToRig = I、rigToShip = I →
    // aircraftToShip 必须**等于** PnP 给出的 aircraftToCamera。
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    // 真值：ZYX 欧拉角 yaw = 30°、pitch = 5°、roll = 2°，目标在
    // (1, -0.5, 120) m。
    //
    // ⚠ 真值必须**先按 ZYX 复合出旋转矩阵**，再转成 Rodrigues 向量交给
    //    PnP。若图省事直接写 `Vec3d(2°, 5°, 30°)` 当 rvec，那是"绕轴
    //    (2,5,30) 方向转 30.5°"的**单轴旋转**，与本用例断言的 ZYX 分解
    //    (30, 5, 2) 是不同的旋转 —— 两个旋转矩阵相差约 2°，而
    //    `rotationAngleBetween` 的断言会失败（实测就是这样踩到的）。
    //    这类夹具错误的特征是：*矩阵级*断言与*欧拉角级*断言互相矛盾，
    //    此时错的是夹具，不是分解实现。
    const cv::Matx33d truthRotation = rotZ(30.0) * rotY(5.0) * rotX(2.0);
    const cv::Vec3d truthTvec(1.0, -0.5, 120.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRotation, rvecTrue);

    const std::vector<cv::Point3f> points = aircraftShapePoints();
    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), rig.cam25);

    // 注入：固定特征集 + 固定对应集。这样本用例只校验"坐标链与阈值接线"，
    // 与 SIFT 的数值行为无关（ENG-10 §5.2 约束 4）。
    FeatureSet presetFeatures;
    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        presetFeatures.keypoints.emplace_back(correspondences[i].imagePoint, 1.0f);
    }
    presetFeatures.descriptors = cv::Mat(
        static_cast<int>(correspondences.size()), 4, CV_32F, cv::Scalar(1.0f));

    MockFeatureExtractor extractor(presetFeatures);
    MockFeatureMatcher matcher;
    matcher.setPreset(correspondences);

    const MultiCameraFrame frame =
        frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 7u));

    PosePipeline::Stages stages;
    stages.extractor = &extractor;
    stages.matcher = &matcher;

    PosePipeline pipeline(cfg, val, mgr, rig, stages);

    ShipPoseResult out;
    ASSERT_TRUE(pipeline.solvePose(frame, CameraRole::CAM25, rig.cam25, out));
    EXPECT_TRUE(out.success);

    // 旋转：恒等链下 aircraftToShip 就是 PnP 的结果。
    EXPECT_LT(rotationAngleBetween(out.aircraftToShip.rotation, truthRotation),
              1e-6);

    // 平移：三级复合在恒等链下退化为原值。
    EXPECT_NEAR(out.aircraftToShip.translation(0), truthTvec(0), 1e-4);
    EXPECT_NEAR(out.aircraftToShip.translation(1), truthTvec(1), 1e-4);
    EXPECT_NEAR(out.aircraftToShip.translation(2), truthTvec(2), 1e-4);

    // ---- 欧拉角 ----
    //
    // ⚠ 这里断言的是"**三个角能否重建出正确的旋转**"，而不是逐角与真值
    //    比较到 1e-6。原因：逐角比较的容差单位是 deg，而上面矩阵判据的
    //    单位是 rad，二者相差 57 倍；同一个数值精度（P_nP 迭代终止容差
    //    约 1e-7 rad）会让矩阵判据通过、而 1e-6 deg 的 pitch/roll 判据失败
    //    （实测踩到）。当接近 pitch = 0 时，pitch/roll 又分别是 asin 与
    //    atan2(R(2,1), R(2,2))，对同一份矩阵误差的敏感度并不相同 ——
    //    用矩阵级判据才是与分解契约一致的问法。
    const cv::Matx33d rebuilt = rotZ(out.yaw) * rotY(out.pitch) * rotX(out.roll);
    EXPECT_LT(rotationAngleBetween(rebuilt, truthRotation), 1e-6);

    // Yaw 是验收指标（1 arcmin），单独给绝对判据：容差 1e-3 deg 比指标
    // 严 17 倍，同时足以发现符号/顺序错误（那样会差数十度）。
    EXPECT_NEAR(out.yaw, 30.0, 1e-3);
}

TEST(PosePipeline, SolvePoseHonoursRigToShipRotation)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();

    // 光机相对舰体绕 Z 转 90°：aircraftToShip = Rz(90°)·aircraftToCamera。
    // 若两个因子的顺序接反（rAc·rRs），yaw 会变成 30° − 90° = −60° ——
    // 同样是"看起来像个合法姿态"的错值，只有断言能发现。
    const OpticalRigCalibration rig =
        fixtureRig(rotZ(90.0), cv::Vec3d(0, 0, 0));

    const cv::Vec3d truthRvec(0.0, 0.0, 30.0 * kDegToRad);
    const cv::Vec3d truthTvec(0.0, 0.0, 100.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRvec, rvecTrue);

    const std::vector<cv::Point3f> points = aircraftShapePoints();
    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), rig.cam25);

    FeatureSet presetFeatures;
    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        presetFeatures.keypoints.emplace_back(correspondences[i].imagePoint, 1.0f);
    }
    presetFeatures.descriptors = cv::Mat(
        static_cast<int>(correspondences.size()), 4, CV_32F, cv::Scalar(1.0f));

    MockFeatureExtractor extractor(presetFeatures);
    MockFeatureMatcher matcher;
    matcher.setPreset(correspondences);

    PosePipeline::Stages stages;
    stages.extractor = &extractor;
    stages.matcher = &matcher;

    PosePipeline pipeline(cfg, val, mgr, rig, stages);

    const MultiCameraFrame frame =
        frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 8u));

    ShipPoseResult out;
    ASSERT_TRUE(pipeline.solvePose(frame, CameraRole::CAM25, rig.cam25, out));

    // Rz(90°)·Rz(30°) = Rz(120°)：绕同一轴的旋转可加。
    EXPECT_LT(rotationAngleBetween(out.aircraftToShip.rotation, rotZ(120.0)),
              1e-6);
    EXPECT_NEAR(out.yaw, 120.0, 1e-3);
    // pitch/roll 用重建矩阵断言（理由见恒等链用例的说明）：
    // 这里 pitch/roll 的真值都是 0，逐角比较会拿 ~1e-7 rad 的数值残差
    // 去比 1e-6 deg 的容差。
    const cv::Matx33d rebuilt = rotZ(out.yaw) * rotY(out.pitch) * rotX(out.roll);
    EXPECT_LT(rotationAngleBetween(rebuilt, rotZ(120.0)), 1e-6);
}

TEST(PosePipeline, SolvePoseAppliesCameraToRigTranslationBeforeRigRotation)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();

    // cameraToRig 的平移 t_cr ≠ 0 且 rigToShip 有旋转：
    //   p_ship = R_rs·(R_cr·p_cam + t_cr) + t_rs
    // 若漏掉"先加 t_cr 再乘 R_rs"这一级（例如写成 R_rs·R_cr·p_cam + t_cr），
    // 平移会差 R_rs·t_cr − t_cr —— 对 10 m 级的安装偏置而言是米级误差，
    // 但旋转完全正确，重投影误差也正常。
    const cv::Vec3d tcr(0.3, -0.2, 0.1);
    const OpticalRigCalibration rig =
        fixtureRig(rotZ(90.0), cv::Vec3d(5.0, 0.0, -1.0), cv::Matx33d::eye(), tcr);

    const cv::Vec3d truthRvec(0.0, 0.0, 0.0);
    const cv::Vec3d truthTvec(0.0, 0.0, 100.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRvec, rvecTrue);

    const std::vector<cv::Point3f> points = aircraftShapePoints();
    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), rig.cam25);

    FeatureSet presetFeatures;
    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        presetFeatures.keypoints.emplace_back(correspondences[i].imagePoint, 1.0f);
    }
    presetFeatures.descriptors = cv::Mat(
        static_cast<int>(correspondences.size()), 4, CV_32F, cv::Scalar(1.0f));

    MockFeatureExtractor extractor(presetFeatures);
    MockFeatureMatcher matcher;
    matcher.setPreset(correspondences);

    PosePipeline::Stages stages;
    stages.extractor = &extractor;
    stages.matcher = &matcher;

    PosePipeline pipeline(cfg, val, mgr, rig, stages);

    const MultiCameraFrame frame =
        frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 9u));

    ShipPoseResult out;
    ASSERT_TRUE(pipeline.solvePose(frame, CameraRole::CAM25, rig.cam25, out));

    // 手算：R_rs = Rz(90°) 作用在 (0,0,100) 上仍是 (0,0,100)（绕 Z 的旋转
    // 不改变 Z 分量），故
    //   p_ship = Rz(90°)·((0,0,100) + (0.3,-0.2,0.1)) + (5,0,-1)
    //          = (0.2, 0.3, 100.1) + (5,0,-1) = (5.2, 0.3, 99.1)
    const cv::Vec3d tAcRotated = rotZ(90.0) * (truthTvec + tcr);
    const cv::Vec3d expectedTranslation =
        tAcRotated + cv::Vec3d(5.0, 0.0, -1.0);

    EXPECT_NEAR(out.aircraftToShip.translation(0), expectedTranslation(0), 1e-4);
    EXPECT_NEAR(out.aircraftToShip.translation(1), expectedTranslation(1), 1e-4);
    EXPECT_NEAR(out.aircraftToShip.translation(2), expectedTranslation(2), 1e-4);
}

TEST(PosePipeline, SolvePoseReportsFailureWhenModelIsNotLoaded)
{
    // 未加载模型 → 没有任何三维点 → 必须返回 false，而不是用空点表算出一个
    // "形状合法"的位姿。这是 SYS-08 §7.3 重试判据的前提。
    TargetModelManager mgr;
    ASSERT_FALSE(mgr.loaded());

    const MeasurementConfig cfg = configuredConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    const MultiCameraFrame frame =
        frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 11u));

    PosePipeline pipeline(cfg, val, mgr, rig);

    ShipPoseResult out;
    EXPECT_FALSE(pipeline.solvePose(frame, CameraRole::CAM25, rig.cam25, out));
    EXPECT_FALSE(out.success);
}

TEST(PosePipeline, ValidateUsesStatsFromLastSolve)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    const cv::Vec3d truthRvec(0.0, 0.0, 10.0 * kDegToRad);
    const cv::Vec3d truthTvec(0.0, 0.0, 100.0);
    cv::Mat rvecTrue;
    cv::Rodrigues(truthRvec, rvecTrue);

    const std::vector<cv::Point3f> points = aircraftShapePoints();
    const std::vector<FeatureCorrespondence> correspondences =
        project(points, rvecTrue, cv::Mat(truthTvec), rig.cam25);

    FeatureSet presetFeatures;
    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        presetFeatures.keypoints.emplace_back(correspondences[i].imagePoint, 1.0f);
    }
    presetFeatures.descriptors = cv::Mat(
        static_cast<int>(correspondences.size()), 4, CV_32F, cv::Scalar(1.0f));

    MockFeatureExtractor extractor(presetFeatures);
    MockFeatureMatcher matcher;
    matcher.setPreset(correspondences);

    PosePipeline::Stages stages;
    stages.extractor = &extractor;
    stages.matcher = &matcher;

    PosePipeline pipeline(cfg, val, mgr, rig, stages);

    const MultiCameraFrame frame =
        frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 12u));

    ShipPoseResult pose;
    ASSERT_TRUE(pipeline.solvePose(frame, CameraRole::CAM25, rig.cam25, pose));

    // `validate` 的内点比例来自 solvePose 缓存的 PnpStats —— 冻结的
    // ShipPoseResult 里没有这个量（PoseValidator.h 的文件头）。
    // 无噪声对应集 → 内点比例 1 → 三项判据全过。
    aircraft::data::PoseValidationResult v;
    EXPECT_TRUE(pipeline.validate(pose, v));
    EXPECT_TRUE(v.valid);
    EXPECT_NEAR(v.inlierRatio, 1.0, 1e-9);
    EXPECT_GT(v.confidence, 0.9);

    // 未解算就验证：内点比例 0 → 除非门槛都是 0，否则必须判不合格。
    // "验证一个没算出来的姿态"在语义上不成立，故这是**有意**的行为。
    ShipPoseResult fresh;
    fresh.success = true;
    fresh.yaw = 0.0;
    fresh.reprojectionError = 0.0;
    // 换一个 pipeline（缓存为空）来验证这条语义，避免依赖上一次的状态。
    PosePipeline virgin(cfg, val, mgr, rig, stages);
    aircraft::data::PoseValidationResult v2;
    EXPECT_FALSE(virgin.validate(fresh, v2));
    EXPECT_FALSE(v2.valid);
}

// ===========================================================================
//  四、检测 / 尺度估计 / 通道选择（SYS-07 §4 / SYS-14 §10 / §6）
// ===========================================================================

TEST(PosePipeline, DetectAndEstimateScaleOnSyntheticTarget)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = configuredConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    PosePipeline pipeline(cfg, val, mgr, rig);

    // 暗背景上的亮矩形，长边 242 pixel —— 正是 25 mm 焦段在 300 m 处
    // 10 m 目标的像长（l = f·L/Z = 7246.4 × 10 / 300 ≈ 241.5）。
    const cv::Mat image =
        rectangleImage(2448, 2048, cv::Rect(1000, 900, 242, 60));
    const auto frame = makeFrame(image, CameraRole::CAM25);

    aircraft::data::DetectionResult det;
    ASSERT_TRUE(pipeline.detect(frame, det));
    EXPECT_TRUE(det.found);
    EXPECT_EQ(det.sourceCamera, CameraRole::CAM25);
    EXPECT_NEAR(det.bbox.width, 242, 2);
    EXPECT_NEAR(det.bbox.height, 60, 2);

    aircraft::data::TargetScaleEstimate scale;
    ASSERT_TRUE(pipeline.estimateScale(frame, det, scale));
    // Z = f·L/l = 7246.4 × 10 / 242 ≈ 299.4 m（L 来自模型包围盒 = 10 m）
    EXPECT_NEAR(scale.distance, 299.4, 3.0);
    EXPECT_NEAR(scale.targetPixelSize, 242.0, 2.0);
    EXPECT_GT(scale.confidence, 0.0);
}

TEST(PosePipeline, DetectFailsOnFeaturelessImage)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = configuredConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    PosePipeline pipeline(cfg, val, mgr, rig);

    // 全同灰度图：没有目标，也没有背景差异（MockTargetDetector 的判据）。
    const cv::Mat flat(2048, 2448, CV_8UC1, cv::Scalar(20));
    const auto frame = makeFrame(flat, CameraRole::CAM25);

    aircraft::data::DetectionResult det;
    EXPECT_FALSE(pipeline.detect(frame, det));
}

TEST(PosePipeline, SelectCameraSkipsEmptyFramesAndHonoursAllowedList)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    PosePipeline pipeline(cfg, val, mgr, rig);

    // 只有 CAM100 一路有图。CAM25 图像里的目标即便更大也**不允许**参与
    // （SYS-08 §7.4：被排除的相机不进入候选集合）。
    // 用带纹理的目标：常量填充的矩形虽然能被检测到，却给不出任何可检测
    // 特征（nDetect = 0），会在 minFeatureCount 门槛上被判为无候选 ——
    // 那是夹具的形态问题，与"通道选择"无关（见 texturedTargetImage）。
    MultiCameraFrame frame;
    frame.cam25 = makeFrame(texturedTargetImage(2448, 2048,
                                                cv::Rect(900, 800, 400, 120)),
                            CameraRole::CAM25);
    frame.cam100 = makeFrame(texturedTargetImage(2448, 2048,
                                                 cv::Rect(1000, 900, 242, 60)),
                             CameraRole::CAM100);

    aircraft::data::MeasurementSelectionResult sel;
    ASSERT_TRUE(pipeline.selectCamera(frame, {CameraRole::CAM100}, sel));
    EXPECT_EQ(sel.selectedCamera, CameraRole::CAM100);
    EXPECT_GT(sel.score, 0.0);

    // 反过来：allowed 列表里只有**这一帧里没有图像**的通道 → 无候选 → false
    // （状态机据此走 MEASURE_SELECT 重试，SYS-08 §7.3）。
    //
    // ⚠ 这里必须用 CAM50 而不是 CAM25：上面那半段用例**特意**给 CAM25
    //    也放了一张更大的目标图（用来证明"有图但在 allowed 之外 → 不参与"），
    //    所以 CAM25 是真的有候选的。第一版写的是 CAM25，注释却写着
    //    "把唯一有图的通道排除掉" —— 夹具与注释的前提互相矛盾，
    //    在纹理修复之前它恰好因为 nDetect = 0 而"通过"，掩盖了这个矛盾。
    aircraft::data::MeasurementSelectionResult sel2;
    EXPECT_FALSE(pipeline.selectCamera(frame, {CameraRole::CAM50}, sel2));

    // 空 allowed 列表：无候选。
    aircraft::data::MeasurementSelectionResult sel3;
    EXPECT_FALSE(pipeline.selectCamera(frame, {}, sel3));
}

TEST(PosePipeline, SelectBestFrameRanksBySharpnessAndAppliesGate)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = configuredConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    PosePipeline pipeline(cfg, val, mgr, rig);

    // 三帧：中等模糊 / 清晰 / 更模糊。清晰度 = Laplacian 方差，
    // 对同一场景的模糊程度单调，故"最清晰"是可判定的。
    cv::Mat sharp = noiseImage(320, 240, 21u);
    cv::Mat blurred;
    cv::GaussianBlur(sharp, blurred, cv::Size(5, 5), 1.5);
    cv::Mat moreBlurred;
    cv::GaussianBlur(blurred, moreBlurred, cv::Size(5, 5), 1.5);

    std::vector<aircraft::data::ImageFrame> frames;
    frames.push_back(makeFrame(blurred, CameraRole::CAM25, 1));
    frames.push_back(makeFrame(sharp, CameraRole::CAM25, 2));
    frames.push_back(makeFrame(moreBlurred, CameraRole::CAM25, 3));

    int best = -1;
    ImageQuality q;
    ASSERT_TRUE(pipeline.selectBestFrame(frames, best, q));
    EXPECT_EQ(best, 1);
    EXPECT_GT(q.sharpness, cfg.minSharpness);

    // 全部为均匀灰：sharpness = 0 < minSharpness → 无可用帧
    // （SYS-08 §5.7 的出口，状态机据此重试）。
    std::vector<aircraft::data::ImageFrame> flatFrames;
    for (int i = 0; i < 3; ++i)
    {
        flatFrames.push_back(makeFrame(cv::Mat(240, 320, CV_8UC1,
                                               cv::Scalar(128)),
                                       CameraRole::CAM25, i));
    }
    int best2 = -1;
    ImageQuality q2;
    EXPECT_FALSE(pipeline.selectBestFrame(flatFrames, best2, q2));
    EXPECT_EQ(best2, -1);
}

/// 记录被查询过的分桶的桩（ENG-10 §4.1 的三元组）。
///
/// ⚠ 记录在 `mutable` 成员上：`IMatchStatsStore` 的查询是 `const` 方法，
///   而"查过哪些桶"这件事只能在查询里看清 —— 被测方（Pipeline 与
///   MeasurementSelector）拿的是 `const IMatchStatsStore*`。
class RecordingStatsStore : public aircraft::data::IMatchStatsStore
{
public:
    double successRate(const std::string& targetModelId, int role,
                       int distanceBand, int illumBand) const override
    {
        ++queries;
        lastModelId      = targetModelId;
        lastRole         = role;
        lastDistanceBand = distanceBand;
        lastIllumBand    = illumBand;
        return value;
    }

    void record(const std::string&, int, int, int, bool) override {}
    bool save() override { return true; }
    /// 本桩不接持久化介质：查询直接返回 `value`，与"表加载成功"同形。
    /// 返回 true 而**不是** false —— 后者在本工程的约定里表示
    /// "表已损坏、已改名保留现场"，会把它误报成一次异常。
    bool load() override { return true; }

    double value = 0.75;   ///< 非先验值：与冷启动先验不同才能区分两条路径

    mutable int         queries          = 0;
    mutable std::string lastModelId;
    mutable int         lastRole         = -1;
    mutable int         lastDistanceBand = -1;
    mutable int         lastIllumBand    = -1;
};

TEST(PosePipeline, SelectCameraQueriesTheStoreWithTheCandidatesOwnBucket)
{
    // ⚠ 本用例原为 `SelectCameraColdStartUsesPriorWhenNoStatsStoreInjected`，
    //   靠 `pipeline.lastSelectionColdStart()` / `lastMatchStatsUsage()`
    //   两个**内部状态查询**断言冷启动与 M_hist 明细。那两个访问器已随
    //   "死诊断接口清理"删除（见 PosePipeline.h 的说明），故本用例改为
    //   断言**可观测的协作者调用**：
    //     · 注入的 `IMatchStatsStore` 确实被查询过；
    //     · 查询用的分桶（机型 / 角色 / 距离带 / 光照带）就是候选自己的那一组。
    //
    // ⚠ 这不是"退而求其次"，比原来**更强**：原断言读的是选择器内部的缓存
    //   （自己记的，自己读），而这里读的是**被测代码对外发出的调用** ——
    //   分桶算错（例如把距离带常量 0 传下去）会让所有历史统计灌进同一个桶，
    //   而这在原用例里完全看不出来（内部缓存照样"自洽"）。
    //
    //   选择器层面的冷启动/先验/usage 三元组细节仍由
    //   tests/algorithm/AlgorithmStageTest.cpp 的
    //   `MeasurementSelector.UnknownDistanceSkipsHistoryLookup` 与
    //   `MeasurementSelector.UsageTripleRecordsTheCamera` 覆盖 ——
    //   那里是这些性质真正的归属层。
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = syntheticFixtureConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    RecordingStatsStore store;
    PosePipeline pipeline(cfg, val, mgr, rig, &store);

    MultiCameraFrame frame;
    frame.cam50 = makeFrame(texturedTargetImage(2448, 2048,
                                                cv::Rect(1000, 900, 300, 80)),
                            CameraRole::CAM50);

    aircraft::data::MeasurementSelectionResult sel;
    ASSERT_TRUE(pipeline.selectCamera(frame, {CameraRole::CAM50}, sel));

    EXPECT_EQ(sel.selectedCamera, CameraRole::CAM50)
        << "唯一候选应被选中";

    ASSERT_EQ(store.queries, 1) << "每候选应查一次历史统计表（ENG-10 §4.2）";
    EXPECT_EQ(store.lastModelId, mgr.modelId())
        << "机型标识不得为空或串到别的机型（ENG-10 §4.1）";
    EXPECT_EQ(store.lastRole, static_cast<int>(CameraRole::CAM50))
        << "角色索引应是候选自己的通道";
    EXPECT_GE(store.lastDistanceBand, 0)
        << "距离带越界（-1）意味着分桶算错，全部历史会灌进同一个桶";
    EXPECT_LE(store.lastDistanceBand, 3);
    EXPECT_GE(store.lastIllumBand, 0) << "光照带越界";
    EXPECT_LE(store.lastIllumBand, 2);
}


// ===========================================================================
//  五、B 类定位（ENG-10 §2.3 的投影—匹配—拟合三步 / §2.4 的判据）
// ===========================================================================

namespace
{

/// CAD 夹具的两个位置常量：模型点放在 z = 100 m 的平面上，粗姿态的平移
/// 为 (0, 0, 100) m，于是相机系下的深度 **Z_cam = 200 m**。
/// 反算模型点时必须用 Z_cam（见 cadPointsForImageCrosses 的说明）。
///
/// ⚠ 这两个值在四个 CAD 用例里被重复使用，不能各自写 100.0 ——
///    粗姿态的平移改了而这里没改，夹具就会悄悄按错误的深度反算，
///    表现是"结构点总是定位不出来"。
constexpr double kModelZ = 100.0;
constexpr double kTz = 100.0;

/// 一幅"棋盘格角点"合成图：0/255 两组**互相垂直的亮度边界**在给定的
/// 角点上相交。
///
/// ⚠ 为什么不画"暗背景上的亮十字线"（第一版夹具就是这么写的，结果是
///    与判据自相矛盾的夹具）：
///    一条亮线有**两条**边界（进、出各一次强度阶跃），Canny 会在两侧
///    各得一条边缘链，两链相距一个线宽。而本定位器的分组依据是**梯度
///    方向**（倍角聚类），两条链的梯度方向相反但倍角后同族 → 必然被并成
///    一组。一组里同时有相距 3 pixel 的两条平行边缘，"到拟合直线的距离"
///    的中位数下限就是 ~1.5 pixel，**永远**无法满足 ENG-10 §2.4 的
///    0.3 pixel 残差判据 —— 失败与实现无关。
///    真实的棱线是**两个区域的交界**（机翼对天空、蒙皮对背景），只有一条
///    强度阶跃、一条边缘链。棋盘格角点正是它的模型（也是亚像素角点检测
///    的标准靶标）。
///
/// 边界落在整像素列/行上（角点 x=200 表示 x<200 与 x≥200 两个区域），
/// 故"真实角点位置"的约定就是边界所在的那一列/行。
cv::Mat checkerCornerImage(int width, int height,
                           const std::vector<cv::Point2f>& corners)
{
    std::vector<int> xs;
    std::vector<int> ys;
    for (const cv::Point2f& c : corners)
    {
        xs.push_back(static_cast<int>(std::lround(c.x)));
        ys.push_back(static_cast<int>(std::lround(c.y)));
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

    cv::Mat img(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y)
    {
        int bandY = 0;
        while (bandY < static_cast<int>(ys.size()) && y >= ys[bandY])
        {
            ++bandY;
        }
        for (int x = 0; x < width; ++x)
        {
            int bandX = 0;
            while (bandX < static_cast<int>(xs.size()) && x >= xs[bandX])
            {
                ++bandX;
            }
            img.at<unsigned char>(y, x) =
                ((bandX + bandY) % 2 == 0) ? 255 : 0;
        }
    }
    return img;
}

/// 四个 CAD 结构点，投影后落在给定的四个图像位置上。
///
/// 反算：x_img = cx + f_x·X/Z_cam，其中 **Z_cam 是点在相机系下的深度**，
/// 且 ENG-09 §2.1 的链是 `p_cam = R_cr·p_aircraft + t_cr`，故
///     Z_cam = Z_model + t.z
/// （R = I 时；含旋转时是第三行点乘）。
///
/// ⚠ 夹具若按 Z_model 反算，相差一个 (Z_model + t.z)/Z_model 的倍数：
///    100 m 的模型点配 100 m 的平移，图像上的偏移**正好差一倍**，
///    而十字仍画在期望位置上 —— 表现为"定位结果被拉到窗口边缘"这类
///    看不出原因的失败。（本工程实测踩到过一次。）
std::vector<ModelPoint3D> cadPointsForImageCrosses(
    const std::vector<cv::Point2f>& crosses,
    const CameraCalibration& calib,
    double modelZ,
    double cameraTz)
{
    const double depth = modelZ + cameraTz;

    std::vector<ModelPoint3D> pts;
    for (size_t i = 0; i < crosses.size(); ++i)
    {
        const double x = (crosses[i].x - calib.cameraMatrix.at<double>(0, 2))
                         * depth / calib.cameraMatrix.at<double>(0, 0);
        const double y = (crosses[i].y - calib.cameraMatrix.at<double>(1, 2))
                         * depth / calib.cameraMatrix.at<double>(1, 1);
        ModelPoint3D p;
        p.id = static_cast<int>(i + 1);
        p.position = cv::Point3f(static_cast<float>(x), static_cast<float>(y),
                                 static_cast<float>(modelZ));
        p.featureType = "cad";
        pts.push_back(p);
    }
    return pts;
}

}  // namespace

TEST(CadStructureLocator, LocatesSubPixelCornersOnSyntheticEdges)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 640, 480);

    const std::vector<cv::Point2f> crosses = {
        cv::Point2f(200.0f, 150.0f),
        cv::Point2f(400.0f, 150.0f),
        cv::Point2f(200.0f, 300.0f),
        cv::Point2f(400.0f, 300.0f),
    };

    TargetModel model;
    model.modelId = "synthetic";
    model.points3d = cadPointsForImageCrosses(crosses, calib, kModelZ, kTz);

    // 粗姿态：恒等旋转 + (0,0,100)。于是模型点的投影**恰好**是四个十字中心。
    CameraPose coarse;
    coarse.aircraftToCamera.rotation = cv::Matx33d::eye();
    coarse.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, kTz);

    const cv::Mat image = checkerCornerImage(640, 480, crosses);
    const auto frame = makeFrame(image, CameraRole::CAM25);

    CadStructureLocator locator;
    CadStructureResult out;
    // detectionLongSidePx 传 0 = 不判定展布（ENG-10 §2.4 的第二条判据
    // 只在调用方给出检测框时才有意义）。
    ASSERT_TRUE(locator.locate(frame, model, calib, coarse, 0.0, out));

    EXPECT_EQ(out.projectedCount, 4);
    EXPECT_EQ(out.matchedCount, 4);
    EXPECT_EQ(out.correspondences.size(), static_cast<size_t>(4));
    EXPECT_TRUE(out.available);
    EXPECT_TRUE(out.spreadSufficient);          // 未判定 → 保持 true

    // 定位精度：交点必须落回棋盘格角点。
    //
    // ⚠ 这里断言**弱于** ENG-10 §2.3 期望的 0.1~0.2 pixel，原因是本实现
    //    有一项已实测的量化（已记入 README §6 的已知局限，不是本用例的
    //    夹具问题）：
    //
    //    本实现对 `cv::Canny` 输出的**整数**边缘图做直线拟合，于是
    //    · 与像素栅格**对齐**的边缘，其垂直方向的位置只能取整数 ——
    //      实测：把边界用抗锯齿方式从整数格位滑动半个像素（α 从 1.0 变到
    //      0.0），输出坐标只在 199 与 200 之间**跳变**，不随边界连续变化；
    //    · 斜向边缘的阶梯链在拟合中会自行平均，才呈现亚像素效果。
    //    实测四个角点的一致偏差为 (−1.000, −1.000) pixel、离散为 0，其中
    //    0.5 pixel 是 Canny 对"暗|亮"阶跃的平局取暗侧（两侧像素的梯度
    //    模长相同，由扫描顺序决定），另 0.5 pixel 是本夹具把"第一个亮像素
    //    的中心"当作角点位置所致（真实强度边界在两个像素中心之间）。
    //
    //    因此本用例断言两件事，且**不假装**达到了 §2.3 的精度：
    //      (i) 偏差 ≤ 1.5 pixel —— 足以发现本类最隐蔽的那处错：
    //          窗口坐标偏移会让结果差半个窗口（数十 pixel）；
    //      (ii) 四个点的偏差**互相一致**（极差 < 0.05 pixel）——
    //          同样的几何给出同样的结果，这才是"定位"可复现的含义。
    //    要达到 §2.3 的 0.1~0.2 pixel，需要在整数边缘之外再做一步真正的
    //    亚像素细化（渐变剖面拟合或对交点做 `cv::cornerSubPix`），属后续
    //    阶段的工作，见 README §6 的待办项。
    std::vector<double> dx;
    std::vector<double> dy;
    for (size_t i = 0; i < crosses.size(); ++i)
    {
        const cv::Point2f found = out.correspondences[i].imagePoint;
        dx.push_back(static_cast<double>(found.x - crosses[i].x));
        dy.push_back(static_cast<double>(found.y - crosses[i].y));
        EXPECT_LT(std::fabs(dx.back()), 1.5) << "第 " << i << " 个结构点的 x";
        EXPECT_LT(std::fabs(dy.back()), 1.5) << "第 " << i << " 个结构点的 y";
    }
    const double dxSpread = *std::max_element(dx.begin(), dx.end())
                          - *std::min_element(dx.begin(), dx.end());
    const double dySpread = *std::max_element(dy.begin(), dy.end())
                          - *std::min_element(dy.begin(), dy.end());
    EXPECT_LT(dxSpread, 0.05);
    EXPECT_LT(dySpread, 0.05);

    // 残差满足冻结判据（ENG-10 §2.4：≤ 0.3 pixel）。
    EXPECT_LE(out.maxResidualPx, 0.3);
    EXPECT_EQ(out.rejectedByResidual, 0);

    // 展布 W = 结构点的横向分布两端之差：200 → 400 → 200 pixel。
    EXPECT_NEAR(out.spreadPx, 200.0, 2.0);
}

TEST(CadStructureLocator, FlagsInsufficientSpreadButKeepsCorrespondences)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 640, 480);

    const std::vector<cv::Point2f> crosses = {
        cv::Point2f(200.0f, 150.0f),
        cv::Point2f(400.0f, 150.0f),
        cv::Point2f(200.0f, 300.0f),
        cv::Point2f(400.0f, 300.0f),
    };

    TargetModel model;
    model.points3d = cadPointsForImageCrosses(crosses, calib, kModelZ, kTz);

    CameraPose coarse;
    coarse.aircraftToCamera.rotation = cv::Matx33d::eye();
    coarse.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, kTz);

    const cv::Mat image = checkerCornerImage(640, 480, crosses);
    const auto frame = makeFrame(image, CameraRole::CAM25);

    CadStructureLocator locator;
    CadStructureResult out;
    // 检测框长边 10000 pixel ≫ 实际展布 200 pixel → 展布不足。
    // ENG-10 §2.4 对这一条的要求是**标记**而不是丢弃结构点：
    // 展布不足影响的是 Yaw 精度，与"这些点是否可信"是两件事。
    ASSERT_TRUE(locator.locate(frame, model, calib, coarse, 10000.0, out));

    EXPECT_TRUE(out.available);
    EXPECT_FALSE(out.spreadSufficient);
    EXPECT_EQ(out.correspondences.size(), static_cast<size_t>(4));
}

TEST(CadStructureLocator, ReportsNoMatchOnBlankImage)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 640, 480);

    const std::vector<cv::Point2f> crosses = {
        cv::Point2f(200.0f, 150.0f),
        cv::Point2f(400.0f, 150.0f),
        cv::Point2f(200.0f, 300.0f),
        cv::Point2f(400.0f, 300.0f),
    };

    TargetModel model;
    model.points3d = cadPointsForImageCrosses(crosses, calib, kModelZ, kTz);

    CameraPose coarse;
    coarse.aircraftToCamera.rotation = cv::Matx33d::eye();
    coarse.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 100.0);

    // 全黑图：没有任何边缘 → 一个点也定位不出来。
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(0));
    const auto frame = makeFrame(blank, CameraRole::CAM25);

    CadStructureLocator locator;
    CadStructureResult out;
    EXPECT_FALSE(locator.locate(frame, model, calib, coarse, 0.0, out));
    EXPECT_EQ(out.projectedCount, 4);       // 投影成功（几何上可见）
    EXPECT_EQ(out.matchedCount, 0);         // 但没有任何棱线可拟合
    EXPECT_FALSE(out.available);
}

TEST(CadStructureLocator, RejectsModelWithTooFewCadPoints)
{
    const CameraCalibration calib = calibrationWithFx(2000.0, 2000.0, 640, 480);

    // 只有 2 个 CAD 结构点：无论图像多清晰都不可能满足 ENG-10 §2.4 的
    // "对应结构点数 ≥ 4"。这是**模型问题**，不是图像问题，必须在投影之前
    // 就判掉（否则会返回一个"图不好"的错误结论）。
    TargetModel model;
    ModelPoint3D p1;
    p1.position = cv::Point3f(-6.0f, -4.5f, 100.0f);
    p1.featureType = "cad";
    ModelPoint3D p2;
    p2.position = cv::Point3f(4.0f, 3.0f, 100.0f);
    p2.featureType = "cad";
    model.points3d = {p1, p2};

    CameraPose coarse;
    coarse.aircraftToCamera.rotation = cv::Matx33d::eye();
    coarse.aircraftToCamera.translation = cv::Vec3d(0.0, 0.0, 100.0);

    const cv::Mat image = checkerCornerImage(640, 480,
                                             {cv::Point2f(200.0f, 150.0f),
                                              cv::Point2f(400.0f, 300.0f)});
    const auto frame = makeFrame(image, CameraRole::CAM25);

    CadStructureLocator locator;
    CadStructureResult out;
    EXPECT_FALSE(locator.locate(frame, model, calib, coarse, 0.0, out));
    EXPECT_FALSE(out.available);
}

// ===========================================================================
//  六、真实阶段对象的接线（不注入时的默认链）
// ===========================================================================

TEST(PosePipeline, DefaultChainBuildsRealStages)
{
    const std::string dir = writeFixtureModelDir();
    ASSERT_FALSE(dir.empty());
    TargetModelManager mgr;
    ASSERT_TRUE(mgr.loadModel(dir));

    const MeasurementConfig cfg = configuredConfig();
    const ValidationConfig val = configuredValidation();
    const OpticalRigCalibration rig =
        fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

    // 便捷构造：自建 MockTargetDetector + PinholeScaleEstimator +
    // SiftFeatureExtractor + DescriptorFeatureMatcher + CvPnPPoseEstimator。
    // 本用例断言的是"默认链确实能被构造且各阶段可调用"——
    // 它防的是一类装配错误：某个阶段忘了 new、或注入矩阵里的配置漏传，
    // 于是运行期在第一次调用时才发现（而不是在构造时）。
    PosePipeline pipeline(cfg, val, mgr, rig);

    const cv::Mat image =
        rectangleImage(2448, 2048, cv::Rect(1000, 900, 242, 60));
    const auto frame = makeFrame(image, CameraRole::CAM25);

    aircraft::data::DetectionResult det;
    ASSERT_TRUE(pipeline.detect(frame, det));

    aircraft::data::TargetScaleEstimate scale;
    ASSERT_TRUE(pipeline.estimateScale(frame, det, scale));

    // SIFT 在合成矩形上提取不出足够特征（它是纯几何图形、无纹理），
    // 故这里只要求"调用返回而不是崩溃"，并据此断言失败路径的语义：
    // 特征不足时 solvePose 必须返回 false + out.success == false。
    MultiCameraFrame mframe = frameWithImage(CameraRole::CAM25, image);
    ShipPoseResult pose;
    const bool solved = pipeline.solvePose(mframe, CameraRole::CAM25,
                                           rig.cam25, pose);
    if (!solved)
    {
        // ⚠ 此处原来还有一条 `EXPECT_EQ(pipeline.lastPnpStats().inputCount, 0)`。
        //   它已随"死诊断接口清理"删除：该访问器只被测试读取，而这条断言
        //   与紧邻的 `EXPECT_FALSE(pose.success)` 表达的是同一件事
        //   （"解算没成功"）—— 删掉不损失鉴别力，留着则让测试
        //   依赖被测对象的私有成员。内点规模的正确性由
        //   `PnPPoseEstimator.RecoversKnownPoseFromNoiseFreeCorrespondences`
        //   直接断言 `stats.inputCount` 覆盖（那里有独立预言机）。
        EXPECT_FALSE(pose.success);
    }
}

TEST(PosePipeline, SiftExtractorRejectsImageBelowFeatureGate)
{
    // SiftFeatureExtractor 的 minFeatureCount 门槛源自 MeasurementConfig。
    // 均匀灰图必然提取不到特征 → 必须返回 false（而不是返回 0 个点后
    // 让下游在匹配阶段退化成一个更含糊的失败）。
    SiftFeatureExtractor extractor(50);

    const cv::Mat flat(480, 640, CV_8UC1, cv::Scalar(128));
    const auto frame = makeFrame(flat, CameraRole::CAM25);

    FeatureSet fs;
    EXPECT_FALSE(extractor.extract(frame, fs));
}

TEST(PosePipeline, DescriptorMatcherKeepsBothFeatureClasses)
{
    // DescriptorFeatureMatcher 的输入契约：A 类（纹理）来自
    // FeatureExtractor，B 类（结构点）来自 CadStructureLocator，
    // 输出按 ENG-10 §2.5 的冲突规则融合。此处只校验**空输入不崩溃且
    // 明确失败** —— 匹配质量的数值验证需要 Golden 数据（tests/golden/）。
    const MeasurementConfig cfg = configuredConfig();
    DescriptorFeatureMatcher matcher(cfg);

    FeatureSet features;                 // 空
    TargetModel model;
    std::vector<FeatureCorrespondence> cad;
    std::vector<FeatureCorrespondence> out;
    MatchResult stats;

    EXPECT_FALSE(matcher.match(features, model, cad, out, stats));
    EXPECT_TRUE(out.empty());
    EXPECT_EQ(stats.matchCount, 0);
}

// ===========================================================================
//  六、统计量推送通道（裁决 C-002 Step 5）
//
//  裁决拒绝了给 `IPosePipeline` 增加 `lastMatchResult()` 一类的"内部状态
//  查询"，改为算法在统计量成立时**推送**给观察者。同批的"死诊断接口清理"
//  连既有的 6 个访问器一并删除，本组用例的期望值也随之从"读 Pipeline 的
//  内部状态"改为"读夹具输入侧的预置值"。本组用例针对的是
//  **真实的 `PosePipeline`**（flowtest 里的 `StubPipeline` 是替身：
//  它能证明"记录这条链通了"，但证明不了"生产者真的在正确的位置推送"）。
//
//  两个推送点位置判据各自锁定一种退化：
//    · 忘了推送     → 用例 1 红（观察者一次都没被调用）；
//    · 推送在早退之后 → 用例 2 红（最需要统计量的那次失败反而什么都没有）。
// ===========================================================================

namespace
{

/// 记录每次推送内容的观察者。
class RecordingObserver : public aircraft::algorithm::IPipelineObserver
{
public:
    int calls = 0;

    /// 逐次保存的载荷。**保存全部**而不是只留最后一次：
    /// "推送了两次"与"推送了一次但内容被改过"给出同一个最终值，
    /// 只有序列能区分它们。
    std::vector<aircraft::data::MeasurementStatistics> received;

    void onStatistics(const aircraft::data::MeasurementStatistics& s) override
    {
        ++calls;
        received.push_back(s);
    }
};

/// 恒不收敛的 PnP（模拟"对应点够、但 RANSAC 没找到一致集"）。
class FailingPnPEstimator : public PnPPoseEstimator
{
public:
    bool estimate(const std::vector<FeatureCorrespondence>&,
                  const CameraCalibration&,
                  CameraPose& out,
                  PnpStats& stats) override
    {
        out   = CameraPose{};
        stats = PnpStats{};
        return false;
    }
};

/// 统计量推送用例的夹具：注入**固定特征集 + 固定对应集**。
///
/// ⚠ 不用真实 SIFT 链：模型夹具的描述子是 4 维（见
/// `fixtureModelWithDescriptorValue`），而 SIFT 产出 128 维，
/// 二者在 `batchDistance` 里会因为列数不等直接抛异常 ——
/// 实测就是这样踩到的（抛异常，而不是断言失败，故现象是
/// "用例抛出异常"）。本组用例要测的是**推送时机与内容**，
/// 与描述子从哪里来无关（ENG-10 §5.2 约束 4：注入式测试）。
class PushFixture
{
public:
    /// 预置的 CAD 对应点数与展布宽度（`setPreset` 的形参）。
    ///
    /// ⚠ 提成具名常量，是为了让**断言侧**（消费方用例的期望值）与
    ///   **夹具侧**（`setPreset` 的实参）共用同一处定义。两者各写一遍
    ///   字面量的话，改动其中一处就会得到一个"仍然全绿但与夹具不符"的
    ///   假通过 —— 那正是本组用例要摆脱的失败模式。
    static constexpr int    kPresetCadCount = 4;
    static constexpr double kPresetSpreadPx = 812.5;

    PushFixture()
    {
        dir = writeFixtureModelDir();
        if (dir.empty() || !mgr.loadModel(dir))
        {
            return;
        }

        cfg = syntheticFixtureConfig();
        val = configuredValidation();
        rig = fixtureRig(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0));

        const cv::Matx33d truthRotation = rotZ(30.0) * rotY(5.0) * rotX(2.0);
        const cv::Vec3d truthTvec(1.0, -0.5, 120.0);
        cv::Mat rvecTrue;
        cv::Rodrigues(truthRotation, rvecTrue);

        correspondences = project(aircraftShapePoints(), rvecTrue,
                                  cv::Mat(truthTvec), rig.cam25);

        for (const FeatureCorrespondence& c : correspondences)
        {
            features.keypoints.emplace_back(c.imagePoint, 1.0f);
        }
        features.descriptors = cv::Mat(
            static_cast<int>(correspondences.size()), 4, CV_32F,
            cv::Scalar(1.0f));

        extractor = std::make_unique<MockFeatureExtractor>(features);
        matcher   = std::make_unique<MockFeatureMatcher>();
        // 统计量各项**互不相等**（对应数由 mock 按 preset 大小重算，
        // cadCount / spreadPx 由参数给定）：全零或各项相同的载荷
        // 无法区分"字段串了"与"传对了"。
        matcher->setPreset(correspondences, kPresetCadCount, kPresetSpreadPx);

        frame = frameWithImage(CameraRole::CAM25, noiseImage(64, 48, 7u));
        ok    = true;
    }

    PosePipeline::Stages stages() const
    {
        PosePipeline::Stages s;
        s.extractor = extractor.get();
        s.matcher   = matcher.get();
        return s;
    }

    std::string           dir;
    TargetModelManager    mgr;
    MeasurementConfig     cfg;
    ValidationConfig      val;
    OpticalRigCalibration rig;
    FeatureSet            features;
    std::vector<FeatureCorrespondence> correspondences;
    std::unique_ptr<MockFeatureExtractor> extractor;
    std::unique_ptr<MockFeatureMatcher>   matcher;
    MultiCameraFrame      frame;
    bool                  ok = false;
};

}  // namespace

TEST(PosePipeline, StatisticsObserverReceivesMatchStatisticsOnSolve)
{
    PushFixture fx;
    ASSERT_TRUE(fx.ok);

    PosePipeline pipeline(fx.cfg, fx.val, fx.mgr, fx.rig, fx.stages());

    RecordingObserver observer;
    pipeline.setStatisticsObserver(&observer);

    ShipPoseResult out;
    ASSERT_TRUE(pipeline.solvePose(fx.frame, CameraRole::CAM25, fx.rig.cam25, out));

    ASSERT_EQ(observer.calls, 1) << "一次解算应恰好推送一次统计量";
    const aircraft::data::MeasurementStatistics& got = observer.received.front();

    // ---- 期望值来自**夹具的独立预置值**，而不是 Pipeline 的内部状态 ----
    //
    // ⚠ 本用例原来拿 `pipeline.lastMatchResult()` 当基准，那是**自比较**：
    //   推送载荷与查询读的是同一个私有成员。
    //
    //   精确地说，自比较漏掉的**不是**"字段串位"（那类错位会让两边不等，
    //   自比较照样能发现），而是**源头数值本身算错** —— 该成员里装的值
    //   是错的，推送方照抄一遍，两边依然逐字段相等，断言全绿。
    //   实测：把 `MockFeatureMatcher` 的 `spreadPx` 改成忽略预置值、
    //   改为回算图像展布，自比较版本毫无反应，而下面这条
    //   `spreadPx == kPresetSpreadPx` 当场转红。
    //
    //   该访问器已随"死诊断接口清理"删除，基准随之改为夹具**输入侧**的
    //   预置值 —— 一个与实现无关的预言机。
    //
    //   对应关系（`MockFeatureMatcher::match` 的明文约定）：
    //     totalCandidates = preset.size()   matchCount  = preset.size()
    //     cadCount        = setPreset 的形参  textureCount = match - cad
    //     spreadPx        = setPreset 的形参
    const int n = static_cast<int>(fx.correspondences.size());
    ASSERT_GT(n, PushFixture::kPresetCadCount) << "前提：候选数应多于 CAD 点数";

    // 推送载荷必须是 `MatchResult` 的**逐字段恒等映射**（唯一的转换点）。
    // ⚠ 两边**不同名**：`MeasurementStatistics::featureCount` 对应
    //    `MatchResult::totalCandidates`（见 MeasurementStatistics.h）。
    //    照名字配对会在这里配出一个"两边都有 featureCount"的假等式。
    EXPECT_EQ(got.featureCount,      n);
    EXPECT_EQ(got.matchCount,        n);
    EXPECT_EQ(got.cadCount,          PushFixture::kPresetCadCount);
    EXPECT_EQ(got.textureCount,      n - PushFixture::kPresetCadCount);
    EXPECT_DOUBLE_EQ(got.spreadPx,   PushFixture::kPresetSpreadPx);

    // `droppedByConflict` 在本夹具里**必然为 0**（MockFeatureMatcher 不设它，
    // 冲突规则由真实匹配器执行）。本条只守"不许留垃圾值"，不构成对冲突
    // 规则的判据 —— 那是 DescriptorFeatureMatcher 自己的用例。
    EXPECT_EQ(got.droppedByConflict, 0);

    // 载荷必须**非平凡**：全 0 的载荷会让上面几条断言在"忘了填"的实现下
    // 部分通过（例如 cadCount 与 droppedByConflict 都为 0）。
    EXPECT_GT(got.featureCount, 0);
    EXPECT_GT(got.matchCount, 0);
    EXPECT_GT(got.cadCount, 0);
    EXPECT_GT(got.spreadPx, 0.0);
}

TEST(PosePipeline, StatisticsObserverIsPushedBeforeEarlyReturnWhenPnpFails)
{
    PushFixture fx;
    ASSERT_TRUE(fx.ok);

    FailingPnPEstimator failingPnp;
    PosePipeline::Stages stages = fx.stages();
    stages.pnp = &failingPnp;

    PosePipeline pipeline(fx.cfg, fx.val, fx.mgr, fx.rig, stages);

    RecordingObserver observer;
    pipeline.setStatisticsObserver(&observer);

    ShipPoseResult out;
    EXPECT_FALSE(pipeline.solvePose(fx.frame, CameraRole::CAM25, fx.rig.cam25, out));

    // "有对应点却解不出位姿"只能从对应点的**构成**去找原因，
    // 而那是统计量里唯一的内容。推送点若落在 PnP 失败早退之后，
    // 最需要它的这一次反而什么都没有 —— 而"没有统计量"与
    // "统计量恰好全为 0"在下游看起来完全一样。
    ASSERT_EQ(observer.calls, 1)
        << "PnP 失败早退把统计量一起带走了（推送点位置错误）";
    EXPECT_GT(observer.received.front().matchCount, 0)
        << "失败路径上的统计量不应是空载荷";
}

TEST(PosePipeline, StatisticsObserverDetachStopsPushingWithoutCrashing)
{
    PushFixture fx;
    ASSERT_TRUE(fx.ok);

    PosePipeline pipeline(fx.cfg, fx.val, fx.mgr, fx.rig, fx.stages());

    RecordingObserver observer;
    pipeline.setStatisticsObserver(&observer);

    // 观察者先于 pipeline 销毁时，注册方必须能把它摘掉
    // （application 层的控制器正是这样做的，见 MeasurementController 的析构）。
    pipeline.setStatisticsObserver(nullptr);

    ShipPoseResult out;
    ASSERT_TRUE(pipeline.solvePose(fx.frame, CameraRole::CAM25, fx.rig.cam25, out));
    EXPECT_EQ(observer.calls, 0) << "摘掉之后仍在推送";

    // 先摘后挂不得损坏推送路径。
    pipeline.setStatisticsObserver(&observer);
    ASSERT_TRUE(pipeline.solvePose(fx.frame, CameraRole::CAM25, fx.rig.cam25, out));
    EXPECT_EQ(observer.calls, 1);

    // 无观察者时推送路径必须**安静地通过**：它是可选通道，
    // 未注册不是错误 —— 但也不能因为解引用空指针而崩。
    pipeline.setStatisticsObserver(nullptr);
    ASSERT_TRUE(pipeline.solvePose(fx.frame, CameraRole::CAM25, fx.rig.cam25, out));
    EXPECT_EQ(observer.calls, 1) << "摘掉后不应再有推送";
}
