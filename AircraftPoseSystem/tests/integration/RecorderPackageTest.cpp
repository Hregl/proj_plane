// ============================================================================
//  tests/integration/RecorderPackageTest.cpp
//
//  依据：SYS-04 §6.4（IF-FILE-04 测量结果包的字段清单）
//        SYS-09 §12（RecorderWorker 的产出：result.json / cam*.raw /
//                    turntable.json / config_snapshot/）
//        ENG-03 §12.7（Recorder 归属 infrastructure）
//        裁决 C-002（入参 MeasurementTask → MeasurementRecord）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本文件证明的是 **结果包本身**：它能不能被读、读出来的值对不对。        │
//  │ 缺了它，"落盘成功"只是一次 return true —— 而一个字段串位、raw 与       │
//  │ result.json 的尺寸不符、或含 NaN 而**不是合法 json** 的结果包，        │
//  │ 在 save() 眼里与一份完好的结果包完全相同。                            │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 判据用 `cv::FileStorage` **解析** result.json，而不是在里面找子串。
//    `Recorder` 的 json 是手写的 ostringstream（见 Recorder.cpp 的 num()/
//    escapeJson()），少一个逗号、多一个 NaN 都会产出**语法错误**的文件 ——
//    而 substring 检查对语法错误完全无感（"能找到 yaw 这几个字符"）。
//
//  ⚠ 本文件覆盖不到的一处（如实登记，不假装覆盖）：
//    `calibrationId` / `modelId` / `softwareVersion` 三个"外部事实"字段由
//    app 层的 `RecorderSinkAdapter` 补齐（见 ApplicationContext.h）。故本文件
//    只能自己填这三个字段 —— 即"适配器真的填了"这件事**本文件不判**。
//
//    ⚠ **该缺口已由 `tests/integration/RecorderAdapterTest.cpp` 补上**
//      （裁决 C-009，2026-09-23）。此前的判断"app 目录只有可执行目标、
//      测试链接不到它"**结论对但推理错**：`RecorderSinkAdapter` 是
//      ApplicationContext.h 里的**内联类**，包含头即可用，不需要链接
//      app 的库（实测仅需编译期版本宏 `APS_VERSION_*`，无需 Qt）。
//      两个文件的分工：本文件问"结果包本身对不对"，那边问"适配器这一层
//      对不对"——**不要**把适配器的判据挪回这里（那需要在本文件里再写一遍
//      适配器逻辑，等于自比较）。
// ============================================================================

#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "data/CameraRole.h"
#include "data/ErrorInfo.h"
#include "data/FailureTrace.h"
#include "data/ImageFrame.h"
#include "data/ImageQuality.h"
#include "data/MeasurementRecord.h"
#include "data/MeasurementState.h"
#include "data/MeasurementStatistics.h"
#include "data/MeasurementTask.h"
#include "data/MultiCameraFrame.h"
#include "data/PoseValidationResult.h"
#include "data/RawImagePayload.h"
#include "data/ShipPoseResult.h"
#include "data/StateTransition.h"
#include "data/SystemConfig.h"
#include "data/TurntableState.h"
#include "infrastructure/recorder/Recorder.h"

using aircraft::data::CameraRole;
using aircraft::infrastructure::Recorder;

namespace
{

// ---------------------------------------------------------------------------
//  临时目录
// ---------------------------------------------------------------------------

/// 进程内唯一的临时目录，析构时整棵删除。
///
/// ⚠ 结果包是**目录树**（result.json + 3 个 raw + turntable.json +
///    config_snapshot/），用一次性临时文件代替不了；且测试跑在开发机上，
///    不能往工程目录里写（那会让"结果包"与"仓库内容"混在一起）。
class TempDir
{
public:
    explicit TempDir(const std::string& tag)
    {
        static int counter = 0;
        dir_ = "/tmp/aps_rec_" + tag + "_" + std::to_string(::getpid()) + "_"
             + std::to_string(++counter);
        ::system(("rm -rf '" + dir_ + "'").c_str());
        ::system(("mkdir -p '" + dir_ + "'").c_str());
    }

    /// ⚠ 置 `APS_REC_KEEP=1` 时**保留**目录 —— 排查"结果包内容不对"时，
    /// 删掉现场是最不该做的事（本文件正是靠它看到 `"yaw": nan` 那一行的）。
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

long fileSize(const std::string& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    return in.is_open() ? static_cast<long>(in.tellg()) : -1;
}

/// 路径是否存在（文件或目录皆可）。
///
/// ⚠ `fileSize() < 0` 只能回答"这个**文件**打不开"，对目录也返回 -1。
///   "失败包里有没有 cam25.raw"这个判据必须落在"**不存在**"上：
///   一个 0 字节的 cam25.raw 同样是打不开的（fileSize == 0，可开但为空），
///   而它在本工程的语义里是"写失败被截断"—— 与"本来就不该写"
///   完全不同的两件事。
bool pathExists(const std::string& path)
{
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

std::vector<unsigned char> readBytes(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(in)),
                                      std::istreambuf_iterator<char>());
}

// ---------------------------------------------------------------------------
//  记录夹具
// ---------------------------------------------------------------------------

/// 一幅**每像素值唯一**的小图：raw 文件写错通道、写错尺寸或行序颠倒时，
/// 逐字节比对必然失败。
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

/// 十二个 C-002 冻结字段全部非空、且**同名字段取不同值**的记录。
///
/// ⚠ 值必须互不相同：若 `statistics.matchCount` 与
///    `selectedQuality.matchCount`（**同名不同义**，前者是融合后保留的
///    对应数、后者是过 RANSAC 的内点数）取同一个值，那么"把两者写反"
///    这个缺陷在结果包里完全看不出来。
aircraft::data::MeasurementRecord fixtureRecord()
{
    aircraft::data::MeasurementRecord r;

    r.task.taskId = "measurement_123456789";
    r.task.state  = aircraft::data::MeasurementState::COMPLETE;

    r.task.result = aircraft::data::ShipPoseResult{};
    r.task.result.success          = true;
    r.task.result.yaw              = 12.5;
    r.task.result.pitch            = -3.25;
    r.task.result.roll             = 0.75;
    r.task.result.reprojectionError = 0.42;
    r.task.result.aircraftToShip.rotation =
        (cv::Matx33d::eye() * 1.0);   // 单位阵：便于断言
    r.task.result.aircraftToShip.translation = cv::Vec3d(1.5, -2.5, 123.0);

    r.task.validation = aircraft::data::PoseValidationResult{};
    r.task.validation.valid             = true;
    r.task.validation.reprojectionError = 0.42;
    r.task.validation.inlierRatio       = 0.91;
    r.task.validation.confidence        = 0.88;

    // 三个通道各写一幅不同种子的图，尺寸也不同（640×480 / 800×600 /
    // 1024×768 是配置里真实的三种），故"raw 串了通道"同样会被逐字节比对抓住。
    r.bestFrame.cam25.role       = CameraRole::CAM25;
    r.bestFrame.cam25.cameraId   = "cam25";
    r.bestFrame.cam25.frameId    = 77;
    r.bestFrame.cam25.image      = patternImage(64, 48, 1);
    r.bestFrame.cam25.exposureTime = 0.005;

    r.bestFrame.cam50.role       = CameraRole::CAM50;
    r.bestFrame.cam50.cameraId   = "cam50";
    r.bestFrame.cam50.frameId    = 78;
    r.bestFrame.cam50.image      = patternImage(80, 60, 2);
    r.bestFrame.cam50.exposureTime = 0.005;

    r.bestFrame.cam100.role      = CameraRole::CAM100;
    r.bestFrame.cam100.cameraId  = "cam100";
    r.bestFrame.cam100.frameId   = 79;
    r.bestFrame.cam100.image     = patternImage(96, 72, 3);
    r.bestFrame.cam100.exposureTime = 0.005;

    r.bestFrame.triggerTimestamp = 1234567890123ULL;
    r.bestFrame.exposureIndex    = 41;   // 裁决 D-C02-6

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
    r.selectedQuality.sharpness    = 1.0;
    r.selectedQuality.exposure     = 0.8;
    r.selectedQuality.contrast     = 0.9;
    r.selectedQuality.featureCount = 120;
    r.selectedQuality.matchCount   = 100;   // ⚠ 与 statistics.matchCount 不同义
    r.selectedQuality.matchRatio   = 0.8333;

    r.degraded         = true;
    r.camerasAvailable = 2;

    // 外部分事实（app 侧适配器补齐；本文件自行填入，见文件头）。
    r.calibrationId   = "calib_v7";
    r.modelId         = "aircraft_v3";
    r.modelType       = "";   // C-003 未实施 → 必然进 missing_required_fields
    r.softwareVersion = "2.1.0-v2.1-framework";

    return r;
}

/// 一份**失败任务**的记录（裁决 C-007）。
///
/// 与 `fixtureRecord()` 的差别正是"失败包"与"成功包"的全部差别：
///   · `task.state == FAILED`、无姿态、无验证结果；
///   · `bestFrame` 为空 → `writeRawFrames` 跳过全部三路（**失败包不带原图**，
///     这是省空间的唯一手段，且不需要改 `IRecorderSink::save` 的签名）；
///   · `failure` 有内容：根因（5001 机型库缺失）与终点（9002 回退预算用尽）
///     **不是同一个码**，轨迹非空。
///
/// ⚠ 根因与终点必须**取不同的码**，否则"首/末两个字段"在用例里可以互相
///    顶替，测试也就无法证明它们分别被写对。这个 5001 → 9002 的组合是
///    FailureTrace.h 文件头举的那条真实链路。
aircraft::data::MeasurementRecord fixtureFailureRecord()
{
    aircraft::data::MeasurementRecord r = fixtureRecord();

    r.task.state      = aircraft::data::MeasurementState::FAILED;
    r.task.result     = aircraft::data::ShipPoseResult{};
    r.task.validation = aircraft::data::PoseValidationResult{};
    r.bestFrame       = aircraft::data::MultiCameraFrame{};

    using aircraft::data::MeasurementState;
    using aircraft::data::StateTransition;

    r.failure.firstFailedState = MeasurementState::POSE_SOLVE;
    r.failure.firstError = aircraft::data::ErrorInfo{
        aircraft::data::kErrModelMissing,
        "PnP 姿态解算失败（机型库未加载，无法给出点表/尺度依据）",
        1000ULL};

    r.failure.finalFailedState = MeasurementState::MEASURE_SELECT;
    r.failure.finalError = aircraft::data::ErrorInfo{
        aircraft::data::kErrRollbackExhausted, "回退预算用尽", 9000ULL};

    // 三次迁移：正常前进两次 + 一次由失败触发的回退（error 非零）。
    // ⚠ 中间那条**带错误**，正是 history 存在的理由（"哪一次迁移是失败
    //   引起的"不必靠相邻元素推断，见 StateTransition.h）。
    r.failure.history.push_back(
        StateTransition{MeasurementState::SEARCH, MeasurementState::TARGET_FOUND,
                        100ULL, aircraft::data::ErrorInfo{}});
    r.failure.history.push_back(
        StateTransition{MeasurementState::TARGET_FOUND, MeasurementState::POSE_SOLVE,
                        200ULL, aircraft::data::ErrorInfo{}});
    r.failure.history.push_back(
        StateTransition{MeasurementState::POSE_SOLVE, MeasurementState::MEASURE_SELECT,
                        300ULL, r.failure.firstError});

    return r;
}

/// 记录 → 落盘 → 返回结果目录。失败时返回空串。
std::string saveRecord(const TempDir& out, const std::string& configDir,
                       const aircraft::data::MeasurementRecord& record)
{
    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();

    Recorder rec(sys, configDir, record.calibrationId, record.modelId,
                 "feat_v1");
    if (!rec.save(record))
    {
        std::fprintf(stderr, "[RecorderPackageTest] save 失败：%s\n",
                     rec.lastErrorText().c_str());
        return std::string();
    }
    return rec.lastPackageDir();
}

}  // namespace

// ===========================================================================
//  1 结果包可被解析，且值就是记录里的值
// ===========================================================================

TEST(RecorderPackageTest, ResultJsonParsesAndCarriesTheRecordedFacts)
{
    TempDir out("json");
    const std::string dir = saveRecord(out, "config", fixtureRecord());
    ASSERT_FALSE(dir.empty()) << "save 应成功";

    const std::string jsonPath = joinPath(dir, "result.json");
    ASSERT_GT(fileSize(jsonPath), 0) << "result.json 未写出";

    // 解析（不是找子串）：手写 json 的语法错误只有这一步能发现。
    cv::FileStorage fs(jsonPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened()) << "result.json 不是合法 json：" << jsonPath;

    std::string taskId;
    fs["task_id"] >> taskId;
    EXPECT_EQ(taskId, "measurement_123456789");

    std::string state;
    fs["state"] >> state;
    EXPECT_EQ(state, "COMPLETE") << "落盘的记录必为成功任务（见 stepSave）";

    std::string camera;
    fs["selected_camera"] >> camera;
    EXPECT_EQ(camera, "CAM50");

    std::string calib;
    fs["calibration_id"] >> calib;
    EXPECT_EQ(calib, "calib_v7");

    std::string sw;
    fs["software_version"] >> sw;
    EXPECT_EQ(sw, "2.1.0-v2.1-framework");

    int degraded = 0;
    fs["degraded"] >> degraded;
    EXPECT_EQ(degraded, 1) << "degraded 是**本次测量的事实**，必须进包";

    int cameras = 0;
    fs["cameras_available"] >> cameras;
    EXPECT_EQ(cameras, 2);

    // ---- 姿态 ----
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    fs["yaw"] >> yaw;
    fs["pitch"] >> pitch;
    fs["roll"] >> roll;
    EXPECT_NEAR(yaw, 12.5, 1e-6);
    EXPECT_NEAR(pitch, -3.25, 1e-6);
    EXPECT_NEAR(roll, 0.75, 1e-6);

    // ---- 统计量与质量 ----
    //
    // ⚠ 两者都有 match_count 字段而**含义不同**（见 MeasurementStatistics.h）：
    //    取值 77 vs 100，故"写反"会被下面的断言抓住；若夹具让两者相等，
    //    本用例就只能证明"有这么两个数"。
    cv::FileStorage fs2(jsonPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs2.isOpened());
    const cv::FileNode stats = fs2["statistics"];
    const cv::FileNode qual  = fs2["quality"];
    ASSERT_FALSE(stats.empty()) << "统计量未进包（Step 5 的推送通道断了）";
    ASSERT_FALSE(qual.empty());

    int statsMatch = 0;
    int qualMatch  = 0;
    int statsCad   = 0;
    stats["match_count"] >> statsMatch;
    qual["match_count"] >> qualMatch;
    stats["cad_count"] >> statsCad;
    EXPECT_EQ(statsMatch, 77);
    EXPECT_EQ(qualMatch, 100);
    EXPECT_EQ(statsCad, 21);
    EXPECT_NE(statsMatch, qualMatch) << "前提：两个同名不同义的字段取值必须不同";

    // ---- 最佳帧与曝光序号 ----
    const cv::FileNode best = fs2["best_frame"];
    ASSERT_FALSE(best.empty());
    int exposureIndex = 0;   // ⚠ OpenCV 的 read 只到 int（无 uint64 重载）
    best["exposure_index"] >> exposureIndex;
    EXPECT_EQ(exposureIndex, 41) << "曝光序号（D-C02-6）必须进包";

    // ---- 转台 ----
    const cv::FileNode turntable = fs2["turntable"];
    ASSERT_FALSE(turntable.empty());
    double azimuth = 0.0;
    turntable["azimuth"] >> azimuth;
    EXPECT_NEAR(azimuth, 20.125, 1e-6);

    // ---- 验证原因分类（裁决 C-008）----
    //
    // ⚠ 落盘用**名字**而不是枚举的整数值（同 motionName 的理由）：
    //   在枚举中间插入一个成员，历史结果包里的 "1" 就会悄悄改含义。
    const cv::FileNode validation = fs2["validation"];
    ASSERT_FALSE(validation.empty());
    std::string reason;
    validation["reason"] >> reason;
    EXPECT_EQ(reason, "OK")
        << "通过验证时原因应为 OK（不是空串、不是整数值、不是 UNKNOWN）";

    // ---- 必含字段的缺项：只有 model_type（C-003 未实施）----
    const cv::FileNode missing = fs2["missing_required_fields"];
    ASSERT_FALSE(missing.empty()) << "缺项表本身必须存在（空表也要写出来）";
    EXPECT_EQ(missing.size(), 1u) << "RANSAC 之后只剩 model_type 可能缺";
    std::string missingName;
    missing[0] >> missingName;
    EXPECT_EQ(missingName, "model_type");
}

// ===========================================================================
//  2 raw 是"无头裸缓冲"，且与 result.json 里声明的尺寸一致
// ===========================================================================

TEST(RecorderPackageTest, RawFramesAreHeaderlessBuffersMatchingDeclaredGeometry)
{
    TempDir out("raw");
    const aircraft::data::MeasurementRecord record = fixtureRecord();
    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty());

    // 逐通道比对：尺寸 + **逐字节**内容。
    //
    // ⚠ 只比尺寸是不够的（写错通道时尺寸都可能恰好相同）；
    //    只比内容也是不够的（尺寸错的文件前 N 字节可能正好相同）。
    const struct
    {
        const char*       file;
        const cv::Mat&    image;
    } channels[3] = {
        {"cam25.raw",  record.bestFrame.cam25.image},
        {"cam50.raw",  record.bestFrame.cam50.image},
        {"cam100.raw", record.bestFrame.cam100.image},
    };

    for (const auto& ch : channels)
    {
        const std::string path = joinPath(dir, ch.file);
        const long expected = static_cast<long>(ch.image.total())
                            * ch.image.elemSize();
        ASSERT_EQ(fileSize(path), expected)
            << ch.file << " 应是**无头裸缓冲**（宽×高×通道字节），"
               "尺寸由 result.json 的 best_frame.frames[] 声明";

        const std::vector<unsigned char> onDisk = readBytes(path);
        ASSERT_EQ(onDisk.size(), static_cast<std::size_t>(expected));

        // cv::Mat 的行是连续的（patternImage 新建即连续），故可整体比对。
        EXPECT_EQ(std::memcmp(onDisk.data(), ch.image.data,
                              static_cast<std::size_t>(expected)), 0)
            << ch.file << " 的像素内容与记录里的图像不一致";
    }

    // 三个通道尺寸互不相同 → 上面若把 `&ch.image` 写成了同一个通道，
    // 尺寸断言立刻失败（而不是三个通道"都通过"）。
    EXPECT_NE(record.bestFrame.cam25.image.size(),
              record.bestFrame.cam50.image.size());
    EXPECT_NE(record.bestFrame.cam50.image.size(),
              record.bestFrame.cam100.image.size());
}

// ===========================================================================
//  2b 12 位帧的原始载荷缺失 ⇒ **拒绝回落**（011-A1 §2.1 第 7 条 / [缺口 1]）
// ===========================================================================
//
//  ⚠ 这一对用例是**同一份帧的两个方向**，缺任何一个都不成立：
//    ① 只清空 `raw`、其余信息原样保留 ⇒ 保存必须**失败**；
//    ② 同一份帧把 `raw` 装回去 ⇒ 保存成功且写出的字节就是 `raw`。
//    只有 ① 时，"保存失败"也可能是因为帧根本是坏的（尺寸不对、格式非法）；
//    只有 ② 时，"保存成功"也说明不了旧判据（`!image.empty()`）有问题。
//    两者合起来才证明**判据恰好是 `raw` 本身**。

namespace
{

/// 一帧 Mono12 的两个侧面：16 位容器（**原始载荷的本体**）与由它
/// 右移 4 位得到的 8U 显示图（§2.1 第 6 条，不缩放、不拉伸）。
///
/// ⚠ 显示图**由容器派生**而不是另一幅随手画的图：这样 ① 用例才真的在问
///   "这份 12 位帧的载荷丢了会怎样"，而不是"一幅无关的 8U 图会怎样"。
struct Mono12Fixture
{
    uint32_t             width  = 0;
    uint32_t             height = 0;
    std::vector<uint8_t> container;   ///< 每像素 2 字节的小端 16 位容器
    cv::Mat              display;     ///< 8U，= 容器里的值 >> (12−8)
};

Mono12Fixture makeMono12(uint32_t width, uint32_t height)
{
    Mono12Fixture fx;
    fx.width  = width;
    fx.height = height;
    fx.display.create(static_cast<int>(height), static_cast<int>(width), CV_8UC1);

    fx.container.resize(static_cast<std::size_t>(width) * height * 2u);
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            // 取值覆盖 0 / 满量程 / 中间值，且**低位非零** ——
            // 低位非零才能让"右移 4 位"与"截断/取高字节"区分开。
            const uint32_t index = y * width + x;
            const uint16_t value =
                static_cast<uint16_t>((index * 37u + 0x0801u) & 0x0FFFu);
            const std::size_t off = static_cast<std::size_t>(index) * 2u;
            fx.container[off]     = static_cast<uint8_t>(value & 0xFFu);   // 小端
            fx.container[off + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
            fx.display.at<unsigned char>(static_cast<int>(y), static_cast<int>(x)) =
                static_cast<unsigned char>(value >> 4);
        }
    }
    return fx;
}

/// 把夹具装成一帧 Mono12。`withRaw == false` 即**只清空原始载荷**、
/// 采集格式与策略（`captureFormat` / `rawPolicy`）原样保留。
aircraft::data::ImageFrame mono12Frame(const Mono12Fixture& fx, bool withRaw)
{
    aircraft::data::ImageFrame f;
    f.role          = CameraRole::CAM25;
    f.cameraId      = "cam25";
    f.frameId       = 101;
    f.timestampNs   = 4242;
    f.captureFormat = aircraft::data::PixelFormat::Mono12;
    f.rawPolicy     = aircraft::data::RawDataPolicy::RawRequired;
    f.image         = fx.display;

    if (withRaw)
    {
        aircraft::data::RawImagePayload p;
        p.bytes = std::make_shared<const std::vector<uint8_t>>(fx.container);
        p.format      = aircraft::data::PixelFormat::Mono12;
        p.validBits   = 12;
        p.packing     = aircraft::data::Packing::Unpacked;
        p.bitAlignment = aircraft::data::BitAlignment::LsbZeroPadded;
        p.width  = fx.width;
        p.height = fx.height;
        p.sdkPayloadBytes = fx.container.size();
        // ⚠ 期望长度**算**出来，不写死 ±2 字节
        //   （写死的话，将来紧凑契约改了这里不会红）。
        aircraft::data::computeExpectedCompactBytes(fx.width, fx.height,
                                                    p.format,
                                                    p.expectedCompactBytes);
        p.compactSizeMatches = (p.sdkPayloadBytes == p.expectedCompactBytes);
        p.declaredByteOrder  = aircraft::data::ByteOrder::LittleEndian;
        f.raw = p;
    }
    return f;
}

}  // namespace

TEST(RecorderPackageTest, Mono12FrameWithoutRawPayloadIsRefusedNotSilentlyDowngraded)
{
    TempDir out("mono12noraw");

    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/false);

    // ---- 前提：这不是"缺图"场景 ----
    // 显示图在、尺寸对、深度 8U —— 旧判据 `!image.empty()` 在这里为真，
    // 于是旧版会**照写**一份 8U 的 cam25.raw，而元数据（若按同一判据生成）
    // 说什么都不影响那一份文件已经被按显示图解释。
    ASSERT_FALSE(record.bestFrame.cam25.image.empty());
    ASSERT_EQ(record.bestFrame.cam25.image.depth(), CV_8U);
    ASSERT_EQ(record.bestFrame.cam25.captureFormat,
              aircraft::data::PixelFormat::Mono12);
    ASSERT_TRUE(record.bestFrame.cam25.raw.bytes == nullptr);

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record))
        << "12 位帧丢了原始载荷却保存成功 —— 这份包里的 cam25.raw 其实是一张"
           "8U 显示图，而元数据按 Mono12 描述它（事后**无法**分辨）";

    const std::string dir = rec.lastPackageDir();
    ASSERT_FALSE(dir.empty());

    // 错误必须**点名是哪一路、为什么** —— 只说"保存失败"就无法排查。
    EXPECT_TRUE(rec.lastErrorText().find("cam25.raw") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_TRUE(rec.lastErrorText().find("未携带原始载荷") != std::string::npos)
        << rec.lastErrorText();

    // ---- 一个文件都不该有 ----
    // 契约错误在**写 result.json 的那一步**就被拦下（它是第一步），
    // 故不存在"元数据已落盘、raw 却没写"那种**看起来完整**的包 ——
    // 它比整包失败更危险：目录里有 result.json，读的人会认为结果齐全。
    EXPECT_FALSE(pathExists(joinPath(dir, "cam25.raw")));
    // 另外两路（8U、RawOptional、无 raw）本来**可以**正常写出 ——
    // 断言它们也没有，是为了证明"一路违约 ⇒ 整包失败"，而不是
    // "跳过坏的那一路、把其余两路写出去"。
    EXPECT_FALSE(pathExists(joinPath(dir, "cam50.raw")));
    EXPECT_FALSE(pathExists(joinPath(dir, "result.json")));
}

TEST(RecorderPackageTest, Mono12FrameWithRawPayloadIsSavedByteForByte)
{
    TempDir out("mono12raw");

    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);

    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty());

    // ① 写出的字节**逐字节等于原始载荷**（不是显示图、不是重新打包的容器）。
    const std::string path = joinPath(dir, "cam25.raw");
    ASSERT_EQ(fileSize(path), static_cast<long>(fx.container.size()))
        << "Mono12 的载荷是 2 字节/像素的 16 位容器（OCCUPY16BIT），"
           "按 1 字节/像素写出说明它被当成 8 位读了";
    const std::vector<unsigned char> onDisk = readBytes(path);
    ASSERT_EQ(onDisk.size(), fx.container.size());
    EXPECT_EQ(std::memcmp(onDisk.data(), fx.container.data(), fx.container.size()), 0);

    // ② 元数据**描述的就是这份文件**：写 "raw" 且格式三件套一致。
    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());
    cv::FileNode frames = fs["best_frame"]["frames"];
    ASSERT_TRUE(frames.isSeq());
    ASSERT_EQ(frames.size(), 3u);

    // 下标 0 = CAM25（三路与 role 一一对应，见 Recorder.cpp 的说明）。
    cv::FileNode f0 = frames[0];
    std::string  role, source, format, packing, order;
    int          validBits = 0;
    f0["role"] >> role;
    f0["data_source"] >> source;
    f0["pixel_format"] >> format;
    f0["packing"] >> packing;
    f0["declared_byte_order"] >> order;
    f0["valid_bits"] >> validBits;

    EXPECT_EQ(role, "CAM25");
    EXPECT_EQ(source, "raw") << "有原始载荷时必须写真身，不得落成 image";
    EXPECT_EQ(format, "Mono12");
    EXPECT_EQ(packing, "Unpacked");
    EXPECT_EQ(order, "LittleEndian");
    EXPECT_EQ(validBits, 12);

    // ③ 显示图**不是**这份文件的解码依据：它的尺寸与容器尺寸不同，
    //    若谁按 display_image 去解释这份 raw，长度立刻对不上。
    cv::FileNode disp = f0["display_image"];
    int          dw = 0, dh = 0;
    disp["width"] >> dw;
    disp["height"] >> dh;
    EXPECT_EQ(dw, static_cast<int>(fx.width));
    EXPECT_EQ(dh, static_cast<int>(fx.height));
    EXPECT_EQ(static_cast<std::size_t>(dw) * static_cast<std::size_t>(dh),
              fx.container.size() / 2u)
        << "显示图是 1 字节/像素、载荷是 2 字节/像素 —— 两者长度天然不同，"
           "这正是'type/width/height 不能当裸缓冲解码依据'的直接体现";
}

TEST(RecorderPackageTest, EightBitFrameWithoutRawPayloadIsSavedAndLabelledAsImage)
{
    TempDir out("mono8noraw");

    // 分支②的**正**用例：8 位帧本来就没有（也不需要）原始载荷 ——
    // `fixtureRecord()` 的三路正是这个形态（虚拟后端 = BGR8/8U、
    // `RawOptional`）。这里要证明的不是"能写出去"，而是**标签如实**：
    // 文件内容与 `data_source = "image"` 这一条必须同时成立。
    const aircraft::data::MeasurementRecord record = fixtureRecord();
    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty());

    const std::string path = joinPath(dir, "cam25.raw");
    const long        expected =
        static_cast<long>(record.bestFrame.cam25.image.total()) *
        static_cast<long>(record.bestFrame.cam25.image.elemSize());
    ASSERT_EQ(fileSize(path), expected);

    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());
    cv::FileNode f0 = fs["best_frame"]["frames"][0];

    std::string source, format;
    int         validBits = 0;
    bool        frameValid = false;
    f0["data_source"] >> source;
    f0["pixel_format"] >> format;
    f0["valid_bits"] >> validBits;
    f0["frame_valid"] >> frameValid;

    EXPECT_EQ(source, "image")
        << "没有原始载荷、回落保存显示图时，必须**如实标注** image —— "
           "标成 raw 会让离线解码按 16 位容器去读一张 8 位图";
    EXPECT_EQ(format, "Mono8");
    EXPECT_EQ(validBits, 8);
    EXPECT_TRUE(frameValid) << "8 位帧没有 raw 是**正常**形态，不是缺帧";
}

// ===========================================================================
//  2c 原始载荷的**几何**必须能从结果包本身恢复（011-A1 九项缺口 §4）
// ===========================================================================
//
//  ⚠ 这一节要证明的是"**结果包自足**"：拿一个包、一台没装过本工程的机器，
//    只按包里写的元数据就能把原始二维图像解出来。
//    上一版做不到 —— 包内唯一的宽高是 `display_image.width/height`，
//    而它取自 `image.cols/rows`（**加工产物**，12 位时是右移 4 位的结果），
//    于是"有原始载荷、没有显示图"的那种包里宽高是 0：载荷完好，
//    却**没有任何字段说明它是几乘几**。

namespace
{

/// 位置编码的 Mono12 夹具：`value(x, y) = x * 16 + y`。
///
/// ⚠ 为什么不用既有 `makeMono12` 的 `index * 37` 序列：那是一条**一维**
///   序列。宽高互换后按新几何去扫，得到的序列与原序列不同 —— 但只有在
///   "某两个位置的值恰好互换"时才会露馅，而且逐位置比对时**期望值要
///   从原容器里取**，等于拿输入当答案。
///   位置编码让每个 `(x, y)` 的期望值都能**独立写出来**（`x * 16 + y`
///   这一条公式即可），与输入缓冲区无关；`width != height`（4×2）则让
///   "宽高互换"这件事必然在某些位置上取到不同的值。
Mono12Fixture makeMono12PositionEncoded(uint32_t width, uint32_t height)
{
    Mono12Fixture fx;
    fx.width  = width;
    fx.height = height;
    fx.display.create(static_cast<int>(height), static_cast<int>(width), CV_8UC1);

    fx.container.resize(static_cast<std::size_t>(width) * height * 2u);
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            const uint16_t value = static_cast<uint16_t>(x * 16u + y);
            const std::size_t off =
                (static_cast<std::size_t>(y) * width + x) * 2u;
            fx.container[off]     = static_cast<uint8_t>(value & 0xFFu);   // 小端
            fx.container[off + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
            fx.display.at<unsigned char>(static_cast<int>(y), static_cast<int>(x)) =
                static_cast<unsigned char>(value >> 4);
        }
    }
    return fx;
}

}  // namespace

TEST(RecorderPackageTest, Mono12FrameWithRawButNoDisplayImageIsStillSaved)
{
    TempDir out("mono12nodisp");

    // 12 位帧的另一种真实形态：**有原始载荷、没有显示图**（例如显示转换
    // 尚未做、或调用方只要原始数据）。它必须是"可用帧"——
    // ⚠ 这正是旧判据（`valid = !image.empty()`）会判错的那一格：
    //   按旧判据这份包会说 CAM25 缺帧，而它的原始数据其实完好。
    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12PositionEncoded(4, 2);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);
    record.bestFrame.cam25.image           = cv::Mat();

    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty());

    // ---- 从这一行起**只看结果包**，不再碰 `fx` ----
    //   ⚠ 重建所需的每一个数字都必须来自包里。若这里还去读
    //     `fx.width` / `fx.container.size()`，那么"包内缺宽高"这件事
    //     在测试里**永远看不见** —— 正是本节能提供旧版给不出的证据的原因。
    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());
    cv::FileNode f0 = fs["best_frame"]["frames"][0];

    std::string source, format;
    bool        frameValid = false;
    int         dw = -1, dh = -1;
    f0["data_source"] >> source;
    f0["frame_valid"] >> frameValid;
    f0["display_image"]["width"] >> dw;
    f0["display_image"]["height"] >> dh;
    f0["pixel_format"] >> format;

    EXPECT_EQ(source, "raw");
    EXPECT_TRUE(frameValid)
        << "有原始载荷就是可用帧 —— 判据不得再等同于 `!image.empty()`";
    EXPECT_EQ(dw, 0);
    EXPECT_EQ(dh, 0);

    // ---- ① 原始几何得由包里给出（§4 的核心断言）----
    //   少了这一条，下面那条 `pkgBytes == pkgW * pkgH * 2` 会在
    //   "宽高都是 0、长度也是 0"时自洽地成立 —— 而文件明明有 16 字节。
    EXPECT_FALSE(f0["raw_image"].empty())
        << "结果包里没有 raw_image —— 有原始载荷却没有它的宽高，"
           "这份载荷的二维几何**无法**从包内恢复";

    int pkgW = -1, pkgH = -1, pkgBytes = -1;
    f0["raw_image"]["width"] >> pkgW;
    f0["raw_image"]["height"] >> pkgH;
    f0["sdk_payload_bytes"] >> pkgBytes;

    // 已知设计（来自本用例构造的那一帧，不是从包里抄的）：
    //   `x * 16 + y` 的编码在 4×2 上给出 0,16,32,48 / 1,17,33,49。
    EXPECT_EQ(pkgW, 4) << "raw_image.width 不是原始载荷的宽度";
    EXPECT_EQ(pkgH, 2) << "raw_image.height 不是原始载荷的高度";
    EXPECT_NE(pkgW, pkgH) << "本用例必须用非方形几何：方形的宽高互换看不出来";

    // ---- ② 包里声明的长度就是文件的长度 ----
    const std::string path = joinPath(dir, "cam25.raw");
    const std::vector<unsigned char> onDisk = readBytes(path);
    ASSERT_EQ(onDisk.size(), static_cast<std::size_t>(pkgBytes))
        << "文件长度与包内声明的 sdk_payload_bytes 不符";
    ASSERT_EQ(static_cast<std::size_t>(pkgW) * static_cast<std::size_t>(pkgH) * 2u,
              onDisk.size())
        << "Mono12 是 2 字节/像素的 16 位容器";

    // ---- ③ 按包内声明的几何与格式逐位置解码 ----
    //   依据（全部来自包与冻结契约，没有一条来自测试输入）：
    //     `pixel_format = Mono12` → 2 字节容器、有效位 12；
    //     `valid_bit_alignment = LsbZeroPadded` → 值在低 12 位；
    //     `declared_byte_order = LittleEndian` → 低字节在前。
    EXPECT_EQ(format, "Mono12");
    std::string alignment, order;
    f0["valid_bit_alignment"] >> alignment;
    f0["declared_byte_order"] >> order;
    ASSERT_EQ(alignment, "LsbZeroPadded");
    ASSERT_EQ(order, "LittleEndian");

    for (int y = 0; y < pkgH; ++y)
    {
        for (int x = 0; x < pkgW; ++x)
        {
            const std::size_t off =
                (static_cast<std::size_t>(y) * static_cast<std::size_t>(pkgW) +
                 static_cast<std::size_t>(x)) * 2u;
            const uint16_t value =
                static_cast<uint16_t>(onDisk[off] |
                                      (static_cast<uint16_t>(onDisk[off + 1]) << 8));
            // ⚠ 逐位置断言，而不是"总长度对"：只比长度的话，
            //   宽高互换（2×4 与 4×2 的字节数相同）照样通过。
            EXPECT_EQ(static_cast<int>(value & 0x0FFFu), x * 16 + y)
                << "(" << x << ", " << y << ") 处的解码值不对 —— "
                   "按包内几何解码得到的不是原始图像";
        }
    }
}

TEST(RecorderPackageTest, RawCarrierShorterThanItsOwnDeclaredLengthIsRefused)
{
    TempDir out("mono12short");

    // 元数据写的是 `sdk_payload_bytes`，而写出去的字节数取自载体本身。
    // 两者不等时那份 raw 的**解码依据就是错的**（文件比元数据短），
    // 而它仍然能被读成一张"尺寸不匹配"的图 —— 又一条静默失效。
    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);

    std::vector<uint8_t> short_ = fx.container;
    short_.resize(short_.size() / 2);   // 载体被截短，元数据不动
    record.bestFrame.cam25.raw.bytes =
        std::make_shared<const std::vector<uint8_t>>(short_);

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record));
    EXPECT_TRUE(rec.lastErrorText().find("载体") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_FALSE(pathExists(joinPath(rec.lastPackageDir(), "cam25.raw")));
}

// ===========================================================================
//  2d 落盘的"复检"必须只依据**帧自身的字段**（011-A1 九项缺口 §6）
// ===========================================================================
//
//  ⚠ 上一版的 `decideRawSource` 标称"不信任上游"，实际只读了上游算好的
//    两个字段（`compactSizeMatches` 与 `expectedCompactBytes`），**从不重算**
//    —— 于是"上游把期望长度算错"这件事在这里被原样放行，错误的元数据
//    直接进了结果包。下面五条各自封住一个"转录别人的结论"的入口。

TEST(RecorderPackageTest, CompactSizeMatchesTrueButLengthActuallyWrongIsStillRefused)
{
    TempDir out("mono12booltrue");

    // 情形①：上游**转录过来的三个事实全部自洽**（期望＝自报＝载体，
    // 布尔量为真），而**用它自己的几何重算**得到的是另一个数。
    // 旧版只看布尔量与上游算好的期望值 ⇒ 放行；本版重算 ⇒ 必须拒绝。
    //
    // ⚠ 本用例的**鉴别力全在夹具上**：若改成"把 `expectedCompactBytes`
    //    ±1"，那个不一致**不需要重算**就能被"期望≠自报"这一条抓住 ⇒
    //    用例在"删掉重算"的变异下**照样通过**（实测：该变异曾在此夹具上
    //    保持全绿）。故这里改的是**几何**（`raw.width`），让四个数字里
    //    只有"重算"这一个能发现问题。
    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);

    // 改动只有这一处：裸缓冲自报的宽比载体实际承载的少 1 像素
    //（32×24×2 = 1536 ⇒ 重算 31×24×2 = 1488）。
    record.bestFrame.cam25.raw.width -= 1;

    // ---- 前提：另外三个数字**完全一致**，布尔量也为真 ----
    ASSERT_TRUE(record.bestFrame.cam25.raw.compactSizeMatches)
        << "本用例的前提是**上游布尔量为真**";
    ASSERT_EQ(record.bestFrame.cam25.raw.expectedCompactBytes,
              record.bestFrame.cam25.raw.sdkPayloadBytes)
        << "前提：期望与自报相等（否则不必重算就能发现不一致）";
    ASSERT_EQ(record.bestFrame.cam25.raw.sdkPayloadBytes,
              record.bestFrame.cam25.raw.bytes->size())
        << "前提：自报与载体相等";
    uint64_t recomputedNow = 0;
    ASSERT_TRUE(aircraft::data::computeExpectedCompactBytes(
        31, 24, aircraft::data::PixelFormat::Mono12, recomputedNow));
    ASSERT_NE(recomputedNow,
              record.bestFrame.cam25.raw.expectedCompactBytes)
        << "前提：按几何重算的结果与那三个数字**不等**（这才需要重算）";

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record))
        << "上游布尔量为真、重算却不等 —— 拒绝的判据必须是**重算**，"
           "而不是转录上游的结论";
    EXPECT_TRUE(rec.lastErrorText().find("四方不一致") != std::string::npos)
        << rec.lastErrorText();
    // 文本要带上**重算值与实到值**，否则离线无法判断差在哪一边。
    EXPECT_TRUE(rec.lastErrorText().find("重算=") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_TRUE(rec.lastErrorText().find("expected_compact_bytes=") !=
                std::string::npos)
        << rec.lastErrorText();
    EXPECT_FALSE(pathExists(joinPath(rec.lastPackageDir(), "cam25.raw")));
}

TEST(RecorderPackageTest, CompactSizeMatchesFalseWhileFourFactsAgreeIsStillRefused)
{
    TempDir out("mono12boolfalse");

    // 情形②：四个长度事实**完全一致**（帧本身没问题），而布尔量是假。
    // 它必为假 ⇒ 这个组合不可能来自正常上游 ⇒ 是**元数据自相矛盾**。
    // ⚠ "四方一致就放行、把布尔量放着不管"是另一种错法：它会把这个
    //   矛盾原样写进结果包，让读包的人拿一个假的字段去做判断。
    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);

    record.bestFrame.cam25.raw.compactSizeMatches = false;
    ASSERT_EQ(record.bestFrame.cam25.raw.sdkPayloadBytes,
              record.bestFrame.cam25.raw.expectedCompactBytes)
        << "本用例的前提是**四个长度事实一致**";

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record)) << "自相矛盾的元数据不得被原样保存";
    EXPECT_TRUE(rec.lastErrorText().find("compact_size_matches=false") !=
                std::string::npos)
        << rec.lastErrorText();
    EXPECT_FALSE(pathExists(joinPath(rec.lastPackageDir(), "cam25.raw")));
}

TEST(RecorderPackageTest, BigEndianDeclaredRawPayloadIsRefused)
{
    TempDir out("mono12be");

    // 情形③：声明大端。本批适配层**只声明小端**（依据在 ENG-09 §5.28）；
    // 按小端去解一份大端载荷，得到的是"逐样本字节颠倒"的值 ——
    // 而它仍是一张**看起来合法的图**，没有任何一处会报错。
    //
    // ⚠ `ByteOrder::BigEndian` 在 `PixelFormat.h` 里一直存在，而全仓此前
    //   **没有任何一处读过它**：这个取值此前完全不参与判断。
    aircraft::data::MeasurementRecord record = fixtureRecord();
    const Mono12Fixture             fx     = makeMono12(32, 24);
    record.bestFrame.cam25                 = mono12Frame(fx, /*withRaw=*/true);
    record.bestFrame.cam25.raw.declaredByteOrder =
        aircraft::data::ByteOrder::BigEndian;

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record));
    EXPECT_TRUE(rec.lastErrorText().find("BigEndian") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_FALSE(pathExists(joinPath(rec.lastPackageDir(), "cam25.raw")));
}

TEST(RecorderPackageTest, Mono8DeclaredWithThreeChannelImageIsRefused)
{
    TempDir out("mono8bgr");

    // 情形④：格式说单通道、图是三通道。写出去的是 `rows × cols × 3` 字节，
    // 元数据却写 `Mono8` ⇒ 离线按元数据解码得到一张"宽度对、每行只取
    // 前 1/3"的错位图 —— 而它看起来完全像一张正常的图。
    //
    // ⚠ 上一版的回落分支只查 `depth() == CV_8U`，**通道数完全不参与判断**。
    aircraft::data::MeasurementRecord record = fixtureRecord();

    aircraft::data::ImageFrame f;
    f.role          = CameraRole::CAM25;
    f.cameraId      = "cam25";
    f.frameId       = 102;
    f.captureFormat = aircraft::data::PixelFormat::Mono8;
    f.rawPolicy     = aircraft::data::RawDataPolicy::RawOptional;
    f.image         = cv::Mat(3, 4, CV_8UC3, cv::Scalar(1, 2, 3));
    record.bestFrame.cam25 = f;

    ASSERT_EQ(record.bestFrame.cam25.image.depth(), CV_8U);   // 深度这一关是过的
    ASSERT_EQ(record.bestFrame.cam25.image.channels(), 3);

    aircraft::data::SystemConfig sys;
    sys.outputDir = out.path();
    Recorder rec(sys, "config", record.calibrationId, record.modelId, "feat_v1");

    EXPECT_FALSE(rec.save(record));
    EXPECT_TRUE(rec.lastErrorText().find("通道数") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_TRUE(rec.lastErrorText().find("Mono8") != std::string::npos)
        << rec.lastErrorText();
    EXPECT_FALSE(pathExists(joinPath(rec.lastPackageDir(), "cam25.raw")));
}

TEST(RecorderPackageTest, FallbackMetadataDescribesTheImageActuallyWritten)
{
    TempDir out("fallbackmeta");

    // 情形⑤（**正**用例）：一份合法回落所写出的元数据，必须**逐键**描述
    // 它实际写出的那个文件。
    //
    // ⚠ 上一版从 `raw` 取格式四件套 —— 而回落路径**根本没有 raw**，
    //   取到的是 `RawImagePayload` 的**默认值**（恰好是 `Mono8/8/Unpacked`）。
    //   于是 `captureFormat = BGR8` 的 3 通道图会被描述成 Mono8 单通道文件。
    //   本用例断的就是这个：`pixel_format` 必须是 `BGR8`。
    aircraft::data::MeasurementRecord record = fixtureRecord();

    aircraft::data::ImageFrame f;
    f.role          = CameraRole::CAM25;
    f.cameraId      = "cam25";
    f.frameId       = 103;
    f.captureFormat = aircraft::data::PixelFormat::BGR8;
    f.rawPolicy     = aircraft::data::RawDataPolicy::RawOptional;
    f.image         = cv::Mat(4, 6, CV_8UC3, cv::Scalar(7, 8, 9));
    record.bestFrame.cam25 = f;

    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty());

    cv::FileStorage fs(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());
    cv::FileNode f0 = fs["best_frame"]["frames"][0];

    std::string source, format, packing, alignment, order;
    int         validBits = -1, dw = -1, dh = -1;
    f0["data_source"] >> source;
    f0["pixel_format"] >> format;
    f0["packing"] >> packing;
    f0["valid_bit_alignment"] >> alignment;
    f0["declared_byte_order"] >> order;
    f0["valid_bits"] >> validBits;
    f0["display_image"]["width"] >> dw;
    f0["display_image"]["height"] >> dh;

    EXPECT_EQ(source, "image") << "没有原始载荷 ⇒ 如实标注 image";
    EXPECT_EQ(format, "BGR8")
        << "元数据的格式取自 `raw` 的默认值（Mono8）而不是实际写出的图";
    EXPECT_EQ(validBits, 8);
    EXPECT_EQ(packing, "Unpacked");
    EXPECT_EQ(alignment, "LsbZeroPadded");
    EXPECT_EQ(order, "LittleEndian");
    EXPECT_EQ(dw, 6);
    EXPECT_EQ(dh, 4);

    // ⚠ 回落路径**不得**写 `raw_image`：那份文件就是显示图，
    //   写出这个键会让人以为存在一份原始载荷（本包没有）。
    EXPECT_TRUE(f0["raw_image"].empty())
        << "回落路径写出了 raw_image —— 本包没有原始载荷";

    // 元数据与文件**同源**的最后一道：按元数据算出的长度就是文件长度。
    const long expected = 6L * 4L * 3L;   // 宽 × 高 × 3 通道（BGR8 的布局）
    EXPECT_EQ(fileSize(joinPath(dir, "cam25.raw")), expected)
        << "按元数据（BGR8、6×4）算出的长度与文件不符 —— 那是元数据在描述"
           "另一份文件";
}

// ===========================================================================
//  3 缺项表反映**记录**，而不是一份写死的清单
// ===========================================================================

TEST(RecorderPackageTest, MissingFieldTableFollowsTheRecordNotAHardcodedList)
{
    TempDir out("missing");

    aircraft::data::MeasurementRecord record = fixtureRecord();
    ASSERT_TRUE(record.modelType.empty());
    const std::string dirA = saveRecord(out, "config", record);
    ASSERT_FALSE(dirA.empty());

    cv::FileStorage fsA(joinPath(dirA, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fsA.isOpened());
    ASSERT_EQ(fsA["missing_required_fields"].size(), 1u);

    // 同一份记录，只把 modelType 填上 → 缺项表必须变空。
    // 这是与"写死的 6 项清单"的关键区别：旧实现无论记录里有没有值，
    // 都报同样一份清单，于是"这次真的缺"与"本来就没有承载者"无法区分。
    record.modelType = "production";
    const std::string dirB = saveRecord(out, "config", record);
    ASSERT_FALSE(dirB.empty());

    cv::FileStorage fsB(joinPath(dirB, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fsB.isOpened());
    EXPECT_EQ(fsB["missing_required_fields"].size(), 0u)
        << "记录里已经补齐的字段不应仍出现在缺项表里";

    std::string modelType;
    fsB["model_type"] >> modelType;
    EXPECT_EQ(modelType, "production");
}

// ===========================================================================
//  4 非有限的姿态数值不得产出一个**读不了**的结果包
// ===========================================================================

TEST(RecorderPackageTest, NonFinitePoseStillProducesParsableJson)
{
    TempDir out("nan");

    aircraft::data::MeasurementRecord record = fixtureRecord();
    // PnP 退化构型下 yaw 可能是 NaN（例如旋转矩阵里出现 0/0）。这种值一旦
    // 原样写进 json，得到的是 `"yaw": nan` —— **不是合法 json**，
    // 于是**整份**结果包在事后分析时完全读不出来（不是少一个字段，
    // 是整份都解析不了），而 save() 返回 true。
    //
    // ⚠ 这条路径**可达**，不是假想的：`PoseValidator` 的各项判据都是
    //    `x > 上限` / `x < 下限` 形式的比较，而 NaN 与任何数比较都为 false，
    //    故一个 yaw = NaN 的姿态不会被判不合格，会一路走到 SAVE 并落盘。
    //    （判定该由哪一层拦住这个姿态，是待裁决项 —— 见 README §6。）
    record.task.result.yaw = std::nan("");
    // C-008 起，这样的姿态在 VALIDATE 阶段就被判不合格（reason =
    // NON_FINITE_VALUE），根本走不到 SAVE。本用例仍按"如果它到了落盘侧"
    // 构造，验的是**最后一道防线**（num() 把非有限值写成 0 + 点名表）——
    // 闸门与落盘侧的兜底是两道独立的防线，前者不取消后者。
    record.task.validation.valid  = false;
    record.task.validation.reason =
        aircraft::data::PoseValidationReason::NON_FINITE_VALUE;

    const std::string dir = saveRecord(out, "config", record);
    ASSERT_FALSE(dir.empty()) << "写不进去是一回事，写出一个坏文件是另一回事";

    const std::string jsonPath = joinPath(dir, "result.json");

    // ⚠ `cv::FileStorage` 在解析失败时**抛异常**（不是返回未打开的对象），
    //    故这里必须自己接住 —— 否则"文件是坏的"会表现为"用例抛出异常"，
    //    现象与"文件能被打开"完全不同，但报告里看不出区别。
    bool parsed = false;
    try
    {
        cv::FileStorage fs(jsonPath, cv::FileStorage::READ);
        parsed = fs.isOpened();
    }
    catch (const cv::Exception&)
    {
        parsed = false;
    }
    EXPECT_TRUE(parsed)
        << "含 NaN 的位姿产出了语法错误的结果包：整份记录无法回放";

    if (!parsed)
    {
        return;   // 已是坏文件，后面的断言没有意义
    }

    // 非有限值写成 0 + 点名：文件可解析，且"这个 0 其实不是数"是
    // **机器可读**的事实。二者缺一不可 ——
    //   · 只写 0 不点名 → 一个 NaN 姿态在结果包里是一个看起来完全正常的
    //     0.000000，读者无从发现（本项目的典型缺陷形态）；
    //   · 写 null 或 "nan" → OpenCV 4.6 分别报 `Value 'null' is not
    //     supported` 与把它读成 1.79e308（均实测），前者让整份文件读不出来。
    cv::FileStorage fs(jsonPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened());

    double yaw = -1.0;
    fs["yaw"] >> yaw;
    EXPECT_DOUBLE_EQ(yaw, 0.0);

    // 原因分类随之落盘：离线可以按 reason 直接筛出"数值不是数"这一类，
    // 不必逐个比对阈值。
    std::string reason;
    fs["validation"]["reason"] >> reason;
    EXPECT_EQ(reason, "NON_FINITE_VALUE");

    const cv::FileNode nonFinite = fs["non_finite_fields"];
    ASSERT_FALSE(nonFinite.empty()) << "点名表本身必须存在（空表也要写出来）";
    ASSERT_EQ(nonFinite.size(), 1u);
    std::string flagged;
    nonFinite[0] >> flagged;
    EXPECT_EQ(flagged, "yaw");

    // 有限值不得被误报（否则点名表会退化成"永远有几项"的噪声）。
    // ⚠ 不遍历 FileNode：本文件的判据是"表里恰好有哪几项"，
    //    而把迭代器接到 std::find 上会把断言写成一条与语义无关的编译题。
    for (std::size_t i = 0; i < nonFinite.size(); ++i)
    {
        std::string name;
        nonFinite[static_cast<int>(i)] >> name;
        EXPECT_NE(name, "pitch") << "有限值被误报为非有限";
        EXPECT_NE(name, "roll") << "有限值被误报为非有限";
    }
}

// ===========================================================================
//  5 失败包：文件齐备，唯独没有原图（裁决 C-007）
// ===========================================================================

TEST(RecorderPackageTest, FailurePackageKeepsEveryFileExceptTheRawFrames)
{
    // 裁决 C-007 的两条要求在此合起来验：
    //   ① 失败任务**也落盘**（此前失败任务不产生结果包，现场只留一行日志，
    //      "过程走到哪一步"永远丢失）；
    //   ② 失败包**不带原图**（省空间）。
    //
    // ⚠ ①与②必须同时成立才叫"失败包"，只验其一会走向两个极端：
    //    只验①→ 失败包与成功包一样大，连续失败的现场会把磁盘写满；
    //    只验②→ 把"整个目录没写"当成"没有原图"，用例在实现完全没落盘时
    //    照样全绿。
    TempDir out("failpkg");
    const std::string dir = saveRecord(out, "config", fixtureFailureRecord());
    ASSERT_FALSE(dir.empty()) << "失败任务必须落盘（C-007）";

    // ② 三路原图一律**不存在**（不是"存在但为 0 字节"，见 pathExists 说明）。
    for (const char* raw : {"cam25.raw", "cam50.raw", "cam100.raw"})
    {
        EXPECT_FALSE(pathExists(joinPath(dir, raw)))
            << raw << " 出现在失败包里 —— 失败包不带原图";
    }

    // ① 其余文件齐全。
    EXPECT_GT(fileSize(joinPath(dir, "result.json")), 0)
        << "失败包缺 result.json";
    EXPECT_GT(fileSize(joinPath(dir, "failure.json")), 0)
        << "失败包缺 failure.json —— 那正是失败包存在的理由";
    EXPECT_GT(fileSize(joinPath(dir, "turntable.json")), 0);
    EXPECT_GT(fileSize(joinPath(dir, "calibration_id.txt")), 0);
    // ⚠ 本断言只验**目录被创建**（失败包与成功包在这一点上不应有差别）。
    //   它**不**验快照里有几个文件 —— 本用例传的 configDir 是个不含 yaml 的
    //   临时目录，快照必然是空的，故"复盘时要用**当时**的阈值"那句话
    //   在这里**验不了**（此前该断言的失败消息正是这么写的，属于
    //   "断言没验它自称要验的东西"，见 §5.1 N-6）。
    //   快照**内容**的判据在 `RecorderAdapterTest`：
    //   真 configDir → 7 个文件齐；空 configDir → 必须报错点名。
    EXPECT_TRUE(pathExists(joinPath(dir, "config_snapshot")))
        << "失败包同样需要 config_snapshot（目录须创建）";

    // 与成功包对照：同一夹具的 bestFrame 非空时三路 raw 都在。
    // 若两边的断言一起绿，说明"是否写 raw"确实由 bestFrame 决定，
    // 而不是由某个与帧无关的分支（如"状态是否为 FAILED"）决定。
    TempDir out2("okpkg");
    const std::string dir2 = saveRecord(out2, "config", fixtureRecord());
    ASSERT_FALSE(dir2.empty());
    for (const char* raw : {"cam25.raw", "cam50.raw", "cam100.raw"})
    {
        EXPECT_TRUE(pathExists(joinPath(dir2, raw)))
            << raw << " 在成功包里也不见了 —— 对照组失效";
    }
}

// ===========================================================================
//  6 失败包回答两个问题：根因是什么、走到了哪一步
// ===========================================================================

TEST(RecorderPackageTest, FailureJsonAnswersRootCauseAndReconstructsThePath)
{
    TempDir out("failjson");
    const std::string dir = saveRecord(out, "config", fixtureFailureRecord());
    ASSERT_FALSE(dir.empty());

    const std::string jsonPath = joinPath(dir, "failure.json");
    ASSERT_GT(fileSize(jsonPath), 0);

    // 与上面几条一样：**解析**而不是找子串。手写 json 的语法错误
    // （少一个逗号、多一个 NaN）只有解析器能发现。
    cv::FileStorage fs(jsonPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened()) << "failure.json 不是合法 json";

    const aircraft::data::MeasurementRecord want = fixtureFailureRecord();

    // ---- 第一问：根因 ----
    //
    // ⚠ 判据落在 **code 数值**上（而不是名字）：离线按码筛选
    //  （"这批失败里有多少是机型库缺失"）是 C-006/C-007 的实际用法，
    //   而按名字筛选在改名时会静默失效。
    const cv::FileNode firstErr = fs["first_error"];
    ASSERT_FALSE(firstErr.empty()) << "failure.json 缺 first_error";
    int firstCode = 0;
    firstErr["code"] >> firstCode;
    EXPECT_EQ(firstCode, aircraft::data::kErrModelMissing);

    // 名字与码必须**一致**：只写名字或只写码都会让上面那类查询失效，
    // 而两者不一致（码改了名没改）比缺一个更糟 —— 它看起来是对的。
    std::string firstName;
    firstErr["name"] >> firstName;
    EXPECT_EQ(firstName, "ErrModelMissing");
    EXPECT_EQ(std::string(aircraft::data::errorCodeName(firstCode)), firstName);

    std::string firstState;
    fs["first_failed_state"] >> firstState;
    EXPECT_EQ(firstState, "POSE_SOLVE")
        << "根因不给所在状态时，同一个码在不同状态下的处置方向分不开";

    // ---- 第二问：终点，且它**不是**根因 ----
    const cv::FileNode finalErr = fs["final_error"];
    ASSERT_FALSE(finalErr.empty());
    int finalCode = 0;
    finalErr["code"] >> finalCode;
    EXPECT_EQ(finalCode, aircraft::data::kErrRollbackExhausted);
    EXPECT_NE(finalCode, firstCode)
        << "首末同码——这条链路里就没有'根因 vs 症状'的区分，"
           "C-007 要消除的正是'只报终点，把现场引向错误方向'";
    EXPECT_NE(finalCode, 0) << "失败任务的终态码不得是 0（C-006 验收判据）";

    std::string finalState;
    fs["final_failed_state"] >> finalState;
    EXPECT_EQ(finalState, "MEASURE_SELECT")
        << "终点状态回答'这套重试策略把它耗在了哪一段'";

    // ---- 第三个问题：过程 ----
    //
    // ⚠ 期望值写**字面名字**，而不是从 `want.failure.history` 里的枚举
    //   反算出来（那需要一个与 Recorder.cpp 同源的 stateName() 副本 ——
    //   用被测实现自身的映射去验被测输出，是自比较，改名时两边一起变，
    //   用例照样全绿）。字面量是**独立预言机**：写盘时的状态名一旦漂移
    //   （例如把 TARGET_FOUND 写成 TARGET），这里立刻转红。
    const char* const kExpectedFrom[3] = {"SEARCH", "TARGET_FOUND", "POSE_SOLVE"};
    const char* const kExpectedTo[3]   = {"TARGET_FOUND", "POSE_SOLVE",
                                          "MEASURE_SELECT"};

    const cv::FileNode history = fs["history"];
    ASSERT_FALSE(history.empty()) << "history 序列本身必须存在";
    ASSERT_EQ(history.size(), want.failure.history.size())
        << "轨迹条数与记录不一致（漏记或多记）";
    ASSERT_EQ(history.size(), 3u) << "前提：夹具给了三次迁移";

    for (std::size_t i = 0; i < 3; ++i)
    {
        const cv::FileNode e = history[static_cast<int>(i)];
        std::string from, to;
        e["from"] >> from;
        e["to"] >> to;
        EXPECT_EQ(from, kExpectedFrom[i]) << "第 " << i << " 条的 from 不对";
        EXPECT_EQ(to, kExpectedTo[i]) << "第 " << i << " 条的 to 不对";

        // 顺序即信息：第 i 条的 from 必须等于第 i-1 条的 to。
        // ⚠ 这条**连续性**判据与上面的字面比对是两件事：把三条**打乱重排**
        //   时，"每条都在文件里出现过"仍可能成立（只要改成集合比较），
        //   而顺序错乱的轨迹会指向完全相反的处置方向
        //   （见 StateTransition.h 的两个例子）。
        if (i > 0)
        {
            std::string prevTo;
            history[static_cast<int>(i - 1)]["to"] >> prevTo;
            EXPECT_EQ(from, prevTo)
                << "第 " << i << " 条的 from 与上一条的 to 不衔接，轨迹顺序错乱";
        }

        // `error` 字段必须**逐条**如实：正常前进的两条无码（0），
        // 第三条由失败触发故带根因码。若实现给每条都塞同一个码
        // （或都不塞），"哪一次迁移是失败引起的"就无法从 history 读出 ——
        // 那正是 StateTransition::error 存在的理由。
        int errCode = -1;
        e["error"]["code"] >> errCode;
        const int expected = (i == 2) ? aircraft::data::kErrModelMissing : 0;
        EXPECT_EQ(errCode, expected) << "第 " << i << " 条的 error 不对";
    }

    // ---- 自足性：result.json 的 failure 摘要与本文件同源 ----
    //
    // ⚠ 两份文件都由同一次 record.failure 生成（见 Recorder.cpp 的
    //   writeFailureJson 说明）。这条断言是"摘要与轨迹不会漂移"的判据 ——
    //   若将来有人在其中一处改了取值来源，这里立刻转红。
    cv::FileStorage fsr(joinPath(dir, "result.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fsr.isOpened());
    const cv::FileNode rfail = fsr["failure"];
    ASSERT_FALSE(rfail.empty()) << "result.json 缺 failure 块";
    int summaryFirst = 0;
    int summaryFinal = 0;
    rfail["first_error"]["code"] >> summaryFirst;
    rfail["final_error"]["code"] >> summaryFinal;
    EXPECT_EQ(summaryFirst, firstCode);
    EXPECT_EQ(summaryFinal, finalCode);

    // 摘要里**不该**有全量轨迹（那是 failure.json 的职责）——
    // 若 result.json 也长了 history，两个文件就有一份冗余，
    // 而冗余的两份迟早不一致。
    EXPECT_TRUE(rfail["history"].empty())
        << "result.json 不应携带全量轨迹（C-007：摘要与全量分两处）";
}

// ===========================================================================
//  7 成功任务的失败块：有路径、无根因
// ===========================================================================

TEST(RecorderPackageTest, SuccessfulTaskCarriesAPathButNoRootCause)
{
    // C-007 的验收里有一句"不得用空轨迹冒充成功"：
    // 成功任务一定有轨迹（它必然走过了若干状态），只是**没有错误**。
    // 若实现图省事对成功任务整块不写、或对失败任务写空轨迹，
    // 结果包里这两种情形就长得一样了。
    TempDir out("okjson");
    const std::string dir = saveRecord(out, "config", fixtureRecord());
    ASSERT_FALSE(dir.empty());

    cv::FileStorage fs(joinPath(dir, "failure.json"), cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened()) << "成功任务同样要有 failure.json";

    // 夹具的成功记录未填 failure → 首/末错误的 code 为 0
    //（= 未设置，渲染为 "OK"），首/末状态为默认的 IDLE。
    int firstCode = -1;
    int finalCode = -1;
    fs["first_error"]["code"] >> firstCode;
    fs["final_error"]["code"] >> finalCode;
    EXPECT_EQ(firstCode, 0) << "成功任务不该有一个'根因'";
    EXPECT_EQ(finalCode, 0);

    std::string firstName;
    fs["first_error"]["name"] >> firstName;
    EXPECT_EQ(firstName, "OK") << "0 的冻结名是 OK，而不是空串或 UNREGISTERED";
}
