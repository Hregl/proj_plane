// ============================================================================
//  tests/integration/RecorderAdapterTest.cpp
//
//  依据：裁决 C-009（V2.1-C01_架构裁决变更说明.md §C-009，2026-09-23 批准）
//        SYS-04 §6.4（IF-FILE-04 测量结果包的字段清单）
//        ENG-01 §14（app 是全工程唯一装配点）
//        ENG-01 §17/§18（依赖方向 application → infrastructure，反向禁止）
//        H-002（标定版本 / 模型标识 / 软件版本必须进入记录）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本文件补的是 **`RecorderSinkAdapter` 这一层**的自动化判据。            │
//  │                                                                      │
//  │ 该层是"控制器产出的记录"与"落盘者接受的记录"之间唯一的缝，且它做的是   │
//  │ **改写**而不是转发（把控制器**没有来源**的三个外部事实补齐）。        │
//  │ 改写点一旦出错，结果包里的值仍然合法、仍然能解析、仍然非空 ——         │
//  │ 只是**是错的**。这正是本工程反复出现的那一类缺陷：字段存在、类型合法、│
//  │ 数值看起来正常，但语义不成立。                                        │
//  │                                                                      │
//  │ 缺口此前已被两处**如实登记**（不是遗漏，是当时没有判据）：            │
//  │  · `RecorderPackageTest.cpp` 文件头："`calibrationId` / `modelId` /    │
//  │    `softwareVersion` 由 app 层的 `RecorderSinkAdapter` 补齐 … 即       │
//  │    '适配器真的填了' 这件事目前**没有自动化判据**"；                   │
//  │  · README §8.5 / C-02 §12.5 的同一句结论。                            │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 为什么本文件可以 `#include "app/ApplicationContext.h"`（前一版的结论是"不行"）：
//    `RecorderSinkAdapter` 是**头文件里的内联类**，不需要链接 app 的库。
//    `src/app/` 确实只产可执行目标（无 libapp），但本文件要的不是目标，
//    是那个类 —— 包含头即可。实测：仅需 `-DAPS_VERSION_*`（编译期版本宏，
//    见 cmake/BuildOptions.cmake），**不需要 Qt**（ApplicationContext.h
//    自身不含任何 Qt 头）。
//
//  ⚠ 与 `RecorderPackageTest` 的分工（两个文件都写 result.json，但问的不是同一件事）：
//    · RecorderPackageTest：**结果包本身**对不对（字段串位、raw 与 json 的尺寸
//      不符、NaN 写出非法 json）。它自己填那三个外部事实，故**绕过了适配器**。
//    · 本文件：**适配器这一层**对不对（它补了没有、补对没有、有没有顺手把
//      别的东西改坏、有没有把失败吞掉）。落盘细节交给上面的文件，本文件
//      只在需要"读回来看适配器改了什么"时才解析 json。
//
//  ⚠ 本文件不重复 `Recorder` 内部的逐字节 raw 比对 —— 那是 RecorderPackageTest
//    的判据。这里只断言"三路 raw 在不在"，因为对适配器而言，丢了 bestFrame
//    与写错 bestFrame 的像素是两回事：前者是适配器的缺陷，后者不是。
// ============================================================================

#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "app/ApplicationContext.h"
#include "data/CameraRole.h"
#include "data/ImageQuality.h"
#include "data/MeasurementRecord.h"
#include "data/MeasurementState.h"
#include "data/MeasurementStatistics.h"
#include "data/MultiCameraFrame.h"
#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"
#include "data/SystemConfig.h"
#include "data/TurntableState.h"
#include "infrastructure/recorder/Recorder.h"

using aircraft::app::RecorderSinkAdapter;
using aircraft::data::CameraRole;
using aircraft::infrastructure::Recorder;

namespace
{

// ---------------------------------------------------------------------------
//  临时目录（与 RecorderPackageTest 同形：置 APS_REC_KEEP=1 保留现场）
// ---------------------------------------------------------------------------

class TempDir
{
public:
    explicit TempDir(const std::string& tag)
    {
        static int counter = 0;
        dir_ = "/tmp/aps_adapter_" + tag + "_" + std::to_string(::getpid())
             + "_" + std::to_string(++counter);
        ::system(("rm -rf '" + dir_ + "'").c_str());
        ::system(("mkdir -p '" + dir_ + "'").c_str());
    }

    ~TempDir()
    {
        if (::getenv("APS_REC_KEEP") == nullptr)
        {
            ::system(("rm -rf '" + dir_ + "'").c_str());
        }
    }

    TempDir(const TempDir&)            = delete;
    TempDir& operator=(const TempDir&) = delete;

    const std::string& path() const { return dir_; }

private:
    std::string dir_;
};

std::string joinPath(const std::string& a, const std::string& b)
{
    return a + "/" + b;
}

bool pathExists(const std::string& path)
{
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

long fileSize(const std::string& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    return in.is_open() ? static_cast<long>(in.tellg()) : -1;
}

/// 目录下的条目名（不递归）。用于"快照里有几个文件、分别叫什么"。
std::vector<std::string> listDir(const std::string& dir)
{
    std::vector<std::string> names;
    const std::string cmd = "ls -A '" + dir + "' 2>/dev/null";
    FILE* p = ::popen(cmd.c_str(), "r");
    if (p == nullptr)
    {
        return names;
    }
    char buf[512];
    while (std::fgets(buf, sizeof(buf), p) != nullptr)
    {
        std::string s(buf);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
        {
            s.pop_back();
        }
        if (!s.empty())
        {
            names.push_back(s);
        }
    }
    ::pclose(p);
    std::sort(names.begin(), names.end());
    return names;
}

/// `Recorder` 在 `writeConfigSnapshot()` 里逐文件拷贝的那 7 个名字。
///
/// ⚠ 这份清单在本文件里**再写一遍是有意的**：下面
///   `RepoConfigMatchesTheNamesTheRecorderExpects` 用它去比"部署侧给了什么"，
///   若直接从 Recorder.cpp 抄同一个常量（例如把它导出成公开数组），
///   比较的两边就同源了 —— 那正是本工程反复出现的"自比较"，
///   改名后两边一起改，断言永远绿。这里必须是**第二份独立陈述**。
const char* const kExpectedSnapshotFiles[] = {
    "system.yaml", "camera.yaml", "optical_rig.yaml", "turntable.yaml",
    "trigger.yaml", "measurement.yaml", "validation.yaml"};

constexpr int kExpectedSnapshotCount = 7;

/// 写一个只含名字的占位文件（config_snapshot 只做**逐文件拷贝**，
/// 不改内容，故快照用例不关心 yaml 内容）。
void writePlaceholder(const std::string& path, const std::string& body)
{
    std::ofstream out(path, std::ios::binary);
    out << body;
}

// ---------------------------------------------------------------------------
//  夹具
// ---------------------------------------------------------------------------

/// 每像素值唯一的小图：raw 写错通道 / 行序颠倒时逐字节可比对。
cv::Mat patternImage(int width, int height, unsigned char seed)
{
    cv::Mat m(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            m.at<unsigned char>(y, x) =
                static_cast<unsigned char>(seed + y * width + x);
        }
    }
    return m;
}

/// 控制器**能**产出的那种记录：测量事实齐全，而三个"外部事实"带**哨兵值**。
///
/// ⚠ 三个外部事实刻意填成 `SENTINEL-*` 而不是留空：
///   · 留空时，"适配器没补"与"适配器补成了空串"不可区分；
///   · 哨兵值还能抓住**方向搞反**的缺陷（适配器把记录里的值当成结果，
///     而不是用自己那一份去覆盖）—— 留空时这个缺陷看起来完全正常。
aircraft::data::MeasurementRecord recordWithSentinels()
{
    aircraft::data::MeasurementRecord r;

    r.task.taskId = "measurement_c009";
    r.task.state  = aircraft::data::MeasurementState::COMPLETE;

    r.task.result = aircraft::data::ShipPoseResult{};
    r.task.result.success           = true;
    r.task.result.yaw               = 12.5;
    r.task.result.pitch             = -3.25;
    r.task.result.roll              = 0.75;
    r.task.result.reprojectionError = 0.42;
    r.task.result.aircraftToShip.rotation    = cv::Matx33d::eye();
    r.task.result.aircraftToShip.translation = cv::Vec3d(1.5, -2.5, 123.0);

    r.task.validation = aircraft::data::PoseValidationResult{};
    r.task.validation.valid       = true;
    r.task.validation.inlierRatio = 0.91;
    r.task.validation.confidence  = 0.88;

    // 三路尺寸互不相同（配置里真实的三种），故"raw 串了通道"同样可比对。
    r.bestFrame.cam25.role         = CameraRole::CAM25;
    r.bestFrame.cam25.cameraId     = "cam25";
    r.bestFrame.cam25.frameId      = 77;
    r.bestFrame.cam25.image        = patternImage(64, 48, 1);
    r.bestFrame.cam25.exposureTime = 0.005;

    r.bestFrame.cam50.role         = CameraRole::CAM50;
    r.bestFrame.cam50.cameraId     = "cam50";
    r.bestFrame.cam50.frameId      = 78;
    r.bestFrame.cam50.image        = patternImage(80, 60, 2);
    r.bestFrame.cam50.exposureTime = 0.005;

    r.bestFrame.cam100.role         = CameraRole::CAM100;
    r.bestFrame.cam100.cameraId     = "cam100";
    r.bestFrame.cam100.frameId      = 79;
    r.bestFrame.cam100.image        = patternImage(96, 72, 3);
    r.bestFrame.cam100.exposureTime = 0.005;

    r.bestFrame.triggerTimestamp = 1234567890123ULL;
    r.bestFrame.exposureIndex    = 41;

    // ⚠ 这两个 match_count **含义不同**（融合后保留数 vs 过 RANSAC 的内点数），
    //   故取值必须不同 —— 否则"适配器把 statistics 与 quality 写反"
    //   在结果包里看不出来。
    r.statistics.featureCount      = 140;
    r.statistics.matchCount        = 77;
    r.statistics.droppedByConflict = 9;
    r.statistics.cadCount          = 21;
    r.statistics.textureCount      = 56;
    r.statistics.spreadPx          = 812.5;

    r.turntable.azimuth   = 20.125;
    r.turntable.elevation = -5.5;
    r.turntable.motion    = aircraft::data::TurntableMotionState::STABLE;

    r.selectedCamera  = CameraRole::CAM50;
    r.selectedScore   = 0.9375;
    r.selectedQuality = aircraft::data::ImageQuality{};
    r.selectedQuality.matchCount = 100;   // ⚠ 与 statistics.matchCount 不同义

    r.degraded         = true;
    r.camerasAvailable = 2;

    // ---- 适配器的改写对象：哨兵值 ----
    r.calibrationId   = "SENTINEL-CALIB";
    r.modelId         = "SENTINEL-MODEL";
    r.softwareVersion = "SENTINEL-VERSION";

    // ⚠ `modelType` **不给哨兵值**，保持默认的空串 —— 因为全仓库
    //   **没有任何生产代码**给它赋过值（`MeasurementController:1059`
    //   只在注释里提到它）。这里若填哨兵值，造出来的就不是"控制器能产出的
    //   记录"，而是在考察一个不存在的场景。
    //
    //   ⚠ 更要紧的是：适配器对它**是透传，不是强制清空**
    //     （`ApplicationContext.h:122` 的"留空"描述的是**当前结果**，
    //     不是一条被执行的约束）。所以"结果包里 model_type 为空"这件事
    //     成立的前提是"还没有人产出类型"，而不是"适配器会拦住它"。
    //     下一条用例用一个正对照把这个区别钉住。
    r.modelType = "";

    return r;
}

/// 装配点侧的那一份事实（`ApplicationContext` 从这里取，见 ENG-01 §14）。
constexpr const char* kAdapterCalibrationId = "calib_v7";
constexpr const char* kAdapterModelId       = "aircraft_v3";

/// 建一个输出目录 + 一个**含全部 7 个 yaml** 的配置目录
/// （真实 `configDir` 的形态；空配置目录另有用例）。
struct Fixture
{
    explicit Fixture(const std::string& tag)
        : out(tag + "_out"), cfg(tag + "_cfg")
    {
        for (int i = 0; i < kExpectedSnapshotCount; ++i)
        {
            writePlaceholder(joinPath(cfg.path(), kExpectedSnapshotFiles[i]),
                             std::string("placeholder:") + kExpectedSnapshotFiles[i]);
        }
    }

    /// 落盘一次，返回包目录（失败时为空串）。
    std::string save(const aircraft::data::MeasurementRecord& r,
                     std::string* errText = nullptr)
    {
        aircraft::data::SystemConfig sys;
        sys.outputDir = out.path();

        Recorder rec(sys, cfg.path(), kAdapterCalibrationId, kAdapterModelId,
                     "feat_v1");
        RecorderSinkAdapter adapter(rec);
        const bool ok = adapter.save(r);
        if (errText != nullptr)
        {
            *errText = adapter.lastErrorText();
        }
        return ok ? adapter.lastPackageDir() : std::string();
    }

    /// 同上，但配置目录可以另指（空目录用例用）。
    std::string saveWithConfigDir(const aircraft::data::MeasurementRecord& r,
                                  const std::string& configDir,
                                  std::string& errText)
    {
        aircraft::data::SystemConfig sys;
        sys.outputDir = out.path();

        Recorder rec(sys, configDir, kAdapterCalibrationId, kAdapterModelId,
                     "feat_v1");
        RecorderSinkAdapter adapter(rec);
        const bool ok = adapter.save(r);
        errText = adapter.lastErrorText();
        return ok ? adapter.lastPackageDir() : std::string();
    }

    TempDir out;
    TempDir cfg;
};

}  // namespace

// ===========================================================================
//  A. 组装验证：适配器改了**该改的**，没改**不该改的**
// ===========================================================================

/// 适配器必须用装配点那一份事实**覆盖**记录里的值。
///
/// ⚠ 这是本文件最要紧的一条。适配器存在的全部理由就是"补齐控制器没有来源的
///   三个字段"，而补齐是一个**写**动作。若某次改动把它退化成
///   `return recorder_.save(record);`（形如"透传"），或者只改了其中一个，
///   结果包里的 `calibration_id` 会是控制器留下的空串/占位 ——
///   **仍然合法、仍然能解析、字段仍然存在**，而"这份结果依据哪版标定"
///   这个问题从此永远答错。
TEST(RecorderAdapter, OverwritesTheExternalFactsTheControllerCannotKnow)
{
    Fixture fx("facts");
    const std::string dir = fx.save(recordWithSentinels());
    ASSERT_FALSE(dir.empty()) << "落盘应成功";

    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened()) << "result.json 不是合法 json";

    std::string calib, model, sw;
    fs["calibration_id"]   >> calib;
    fs["model_id"]         >> model;
    fs["software_version"] >> sw;

    EXPECT_EQ(calib, kAdapterCalibrationId)
        << "适配器没把标定版本补齐（或补成了记录里的哨兵值）";
    EXPECT_EQ(model, kAdapterModelId) << "适配器没把模型标识补齐";
    EXPECT_NE(calib, "SENTINEL-CALIB")
        << "记录里的哨兵值漏进了结果包 —— 说明适配器是透传而不是覆盖";
    EXPECT_NE(model, "SENTINEL-MODEL");

    // 软件版本来自编译期宏（cmake/BuildOptions.cmake），非空即证明读到了。
    EXPECT_NE(sw, "SENTINEL-VERSION")
        << "编译期版本宏没有被写进记录（APS_VERSION_STRING）";
    EXPECT_FALSE(sw.empty());
    EXPECT_NE(sw.find("2.1"), std::string::npos)
        << "软件版本里应含版本号，实际：" << sw;
}

/// `modelType` 保持为空、且因此进 `missing_required_fields`；
/// **同时**用一个正对照证明这个"空"不是因为写入侧恒写空串。
///
/// ⚠ 这不是"漏了"，是 C-003 未实施时的**如实表达**（见
///   ApplicationContext.h 的注释）。用一条用例钉住它，是为了让将来 C-003
///   落地时**必须先看这里** —— 那三行补齐点就在适配器里，而不是在控制器。
///   若有人图省事在适配器里填一个 `"unknown"` 兜底，本用例转红。
///
/// ⚠ **正对照为什么必须有**：只断言"为空"的话，这条用例在
///   "`Recorder` 恒写空串"或"适配器把 modelType 清空了"这两种实现下
///   **同样通过** —— 那它证明的就不是"事实尚不存在"，而是"没人写过"。
///   补一次 `modelType = "production"` 的落盘：值必须活着穿过适配器，
///   且缺项表必须随之变空。两半合起来才是完整的那句话。
TEST(RecorderAdapter, LeavesModelTypeEmptyUntilC003Lands)
{
    Fixture fx("modeltype");
    const std::string dir = fx.save(recordWithSentinels());
    ASSERT_FALSE(dir.empty());

    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());

    std::string modelType;
    fs["model_type"] >> modelType;
    EXPECT_TRUE(modelType.empty())
        << "model_type 应保持为空（C-003 未实施），实际：" << modelType;

    // 同一事实的另一面：它必须出现在 missing_required_fields 里 ——
    // "字段为空"与"字段为空**且被如实登记**"是两件事。
    {
        const cv::FileNode missing = fs["missing_required_fields"];
        ASSERT_FALSE(missing.empty())
            << "model_type 为空却没有进 missing_required_fields";
        bool found = false;
        for (const auto& n : missing)
        {
            std::string name;
            n >> name;
            if (name == "model_type")
            {
                found = true;
            }
        }
        EXPECT_TRUE(found) << "missing_required_fields 里没有 model_type";
    }

    // ---- 正对照：适配器对 modelType 没有主张（透传以外的行为都没有） ----
    //
    // ⚠ 必须换一个 taskId：结果包目录名**就是** taskId
    //   （`Recorder.cpp` 的 `joinPath(outputDir_, task.taskId)`），
    //   沿用同一个 taskId 会**覆盖**上一个包，于是"两次落盘"实际上是
    //   "同一个目录被写了两次" —— 那既读不到第一个包，也让本用例的
    //   两半变成同一份数据。
    aircraft::data::MeasurementRecord typed = recordWithSentinels();
    typed.task.taskId = "measurement_c009_typed";
    typed.modelType   = "production";
    const std::string dir2 = fx.save(typed);
    ASSERT_FALSE(dir2.empty());
    ASSERT_NE(dir2, dir) << "两个 taskId 应落到两个包目录";

    cv::FileStorage fs2(joinPath(dir2, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs2.isOpened());

    std::string passedThrough;
    fs2["model_type"] >> passedThrough;
    EXPECT_EQ(passedThrough, "production")
        << "适配器把 modelType 清掉/改掉了 —— 它应当是透传";

    EXPECT_EQ(fs2["missing_required_fields"].size(), 0u)
        << "modelType 已填上，缺项表却不是空 —— 上一半的'空'因此不可信";
}

/// 适配器**不得**动测量事实。
///
/// ⚠ 与上一条互补：上一条防"该改的没改"，这一条防"不该改的改了"。
///   适配器为了补齐三个字段而**按值拷贝**整个记录（见
///   ApplicationContext.h 的注释），一旦有人图性能改成
///   `const auto&` 后又顺手清理"用不上的字段"，测量事实就会被静默清空。
TEST(RecorderAdapter, PassesMeasurementFactsThroughUntouched)
{
    Fixture fx("passthru");
    const std::string dir = fx.save(recordWithSentinels());
    ASSERT_FALSE(dir.empty());

    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());

    std::string taskId;
    fs["task_id"] >> taskId;
    EXPECT_EQ(taskId, "measurement_c009");

    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    fs["yaw"] >> yaw;
    fs["pitch"] >> pitch;
    fs["roll"] >> roll;
    EXPECT_NEAR(yaw, 12.5, 1e-6);
    EXPECT_NEAR(pitch, -3.25, 1e-6);
    EXPECT_NEAR(roll, 0.75, 1e-6);

    // 三路 raw 在不在 —— 丢了 bestFrame 是**适配器**的缺陷（区别于
    // "raw 内容写错"，那是 Recorder 的判据，见 RecorderPackageTest）。
    for (const char* raw : {"cam25.raw", "cam50.raw", "cam100.raw"})
    {
        EXPECT_TRUE(pathExists(joinPath(dir, raw)))
            << raw << " 不见了 —— 适配器把 bestFrame 丢了";
    }
}

// ===========================================================================
//  B. 结果包验证：适配器交出去的那一份记录，字段确实都落了盘
// ===========================================================================

/// 六个"必须有"的字段同时在场，且**取值正确**。
///
/// ⚠ 断言取值而不是断言键存在：`fs["degraded"]` 在字段缺失时也能构造出
///   一个空节点，"键在不在"这个判据在 FileStorage 上不成立。
TEST(RecorderAdapter, ResultJsonCarriesTheRequiredFieldsWithTheirValues)
{
    Fixture fx("fields");
    const std::string dir = fx.save(recordWithSentinels());
    ASSERT_FALSE(dir.empty());

    const std::string jsonPath = joinPath(dir, "result.json");
    ASSERT_GT(fileSize(jsonPath), 0) << "result.json 未写出";

    cv::FileStorage fs(jsonPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());

    std::string taskId;
    fs["task_id"] >> taskId;
    EXPECT_EQ(taskId, "measurement_c009");

    std::string cam;
    fs["selected_camera"] >> cam;
    EXPECT_EQ(cam, "CAM50") << "通道选择没有进包";

    std::string camUsed;
    fs["camera_used"] >> camUsed;
    EXPECT_EQ(camUsed, "CAM50");

    std::string modelType;
    fs["model_type"] >> modelType;   // 值由上一条用例负责，这里只确认键存在

    int degraded = 0;
    fs["degraded"] >> degraded;
    EXPECT_EQ(degraded, 1) << "degraded 是**本次测量的事实**，必须进包";

    int cameras = 0;
    fs["cameras_available"] >> cameras;
    EXPECT_EQ(cameras, 2);

    const cv::FileNode stats = fs["statistics"];
    ASSERT_FALSE(stats.empty()) << "统计量未进包";
    int featureCount = 0, matchCount = 0, cadCount = 0;
    double spreadPx = 0.0;
    stats["feature_count"] >> featureCount;
    stats["match_count"]   >> matchCount;
    stats["cad_count"]     >> cadCount;
    stats["spread_px"]     >> spreadPx;
    EXPECT_EQ(featureCount, 140);
    EXPECT_EQ(matchCount, 77);
    EXPECT_EQ(cadCount, 21);
    EXPECT_NEAR(spreadPx, 812.5, 1e-6);
}

// ===========================================================================
//  C. 快照验证：config_snapshot/ 里到底有几个文件
//      —— 关闭 §5.1 N-6
// ===========================================================================

/// 真实配置目录 → 快照里必须有**全部 7 个**文件。
///
/// ⚠ 这条用例的存在理由是既有断言**验错了对象**：
///   `RecorderPackageTest.cpp:639` 只断言 `pathExists(config_snapshot)`，
///   而它传的 `configDir`（`"config"` 临时目录）**不含任何 yaml** ——
///   于是该断言在一个**空目录**上照样通过，而它的失败消息写的是
///   "复盘时要用**当时**的阈值"。**断言没验它自称要验的东西。**
///   （这就是 N-6：生产逻辑正确，缺的是覆盖。）
///
/// ⚠ 本用例断言"文件数 == 7 且逐个可在、非空"，而不是"目录存在"。
///   一个空目录与一个装齐 7 份配置的目录，对"能不能复盘"是完全相反的答案。
TEST(RecorderAdapter, SnapshotsAllSevenConfigFilesFromARealConfigDir)
{
    Fixture fx("snap7");
    std::string errText;
    const std::string dir = fx.save(recordWithSentinels(), &errText);
    ASSERT_FALSE(dir.empty());

    EXPECT_TRUE(errText.empty())
        << "配置齐全时不应有错误文本，实际：" << errText;

    const std::string snap = joinPath(dir, "config_snapshot");
    ASSERT_TRUE(pathExists(snap)) << "config_snapshot 未创建";

    const std::vector<std::string> names = listDir(snap);
    EXPECT_EQ(static_cast<int>(names.size()), kExpectedSnapshotCount)
        << "快照文件数应为 7，实际 " << names.size()
        << " —— 空快照意味着这份结果包**无法复盘**";

    for (int i = 0; i < kExpectedSnapshotCount; ++i)
    {
        const std::string p = joinPath(snap, kExpectedSnapshotFiles[i]);
        EXPECT_TRUE(pathExists(p)) << "快照缺少 " << kExpectedSnapshotFiles[i];
        EXPECT_GT(fileSize(p), 0) << kExpectedSnapshotFiles[i] << " 是空文件";
    }
}

/// 空配置目录 → `save()` 仍返回 true（**设计如此，非缺陷**），
/// 但 `lastErrorText()` **必须非空且点名**。
///
/// ⚠ 这条钉的是一个"非致命但不许静默"的分支：`writeConfigSnapshot()`
///   对缺失文件的处置是"返回 true、把缺失清单交给调用方写日志"
///   （见 Recorder.cpp 的注释：结果本身仍然有效，只是复现性受损）。
///   该设计成立的前提是**调用方真的读了 error**。
///   若有人把 `error` 吞掉（例如把 lastErrorText 的重置挪到写快照之后），
///   结果包会在"没有配置快照"的情况下**毫无提示地**产出 ——
///   本用例转红。
TEST(RecorderAdapter, EmptyConfigDirStillSavesButMustReportWhy)
{
    Fixture fx("snap0");
    TempDir emptyCfg("snap0_empty");   // 建了但一个 yaml 也不放

    std::string errText;
    const std::string dir =
        fx.saveWithConfigDir(recordWithSentinels(), emptyCfg.path(), errText);

    EXPECT_FALSE(dir.empty())
        << "配置缺失**不应**让落盘失败 —— 那是既有设计（结果有效、复现性受损）";
    ASSERT_TRUE(pathExists(joinPath(dir, "result.json")))
        << "配置缺失时 result.json 仍应写出";

    EXPECT_FALSE(errText.empty())
        << "配置全缺却毫无提示 —— 结果包会静默地失去可复现性";
    EXPECT_NE(errText.find("config_snapshot"), std::string::npos)
        << "错误文本没有指出是 config_snapshot 的问题，实际：" << errText;
    // 点名到具体文件，才可能据此去补。
    EXPECT_NE(errText.find("camera.yaml"), std::string::npos)
        << "错误文本没有点名缺失的文件，实际：" << errText;
}

/// 仓库 `config/` 里提供的 yaml 名字，与 `Recorder` 期望拷贝的名字**一致**。
///
/// ⚠ 这是**跨文件的一致性**判据，不是重复断言：`Recorder::writeConfigSnapshot`
///   硬编码了 7 个名字。将来加第 8 个配置文件而忘了改那份清单时，
///   所有"快照 7 个文件"的用例都会**继续通过**（因为它们用的是自己造的
///   配置目录），而真实部署下第 8 个文件会一直缺失 —— 且只在复盘时才发现。
///   本用例把"部署侧给了什么"与"落盘侧要什么"对起来。
///
/// ⚠ 依赖 cwd 为仓库根（`build_tests.sh` 会 `cd "$ROOT"`）。找不到配置目录
///   时**报失败而不是跳过**：静默跳过正是本批要消灭的失败模式。
TEST(RecorderAdapter, RepoConfigMatchesTheNamesTheRecorderExpects)
{
    const std::string repoCfg = "config";
    ASSERT_TRUE(pathExists(repoCfg))
        << "未找到 " << repoCfg
        << " —— 本用例需在仓库根运行（build_tests.sh 已 cd \"$ROOT\"）";

    std::vector<std::string> onDisk;
    for (const std::string& name : listDir(repoCfg))
    {
        if (name.size() > 5 && name.compare(name.size() - 5, 5, ".yaml") == 0)
        {
            onDisk.push_back(name);
        }
    }
    std::sort(onDisk.begin(), onDisk.end());

    std::vector<std::string> expected;
    for (int i = 0; i < kExpectedSnapshotCount; ++i)
    {
        expected.push_back(kExpectedSnapshotFiles[i]);
    }
    std::sort(expected.begin(), expected.end());

    EXPECT_EQ(onDisk.size(), expected.size())
        << "config/ 里的 yaml 数量与 Recorder 期望的不一致 —— "
        << "多出来的那份永远不会进快照（且复盘前无人察觉）";

    for (const std::string& name : expected)
    {
        EXPECT_NE(std::find(onDisk.begin(), onDisk.end(), name), onDisk.end())
            << "config/ 缺少 Recorder 要拷贝的 " << name;
    }
}
