// ============================================================================
//  tests/integration/QtSimulationUiTest.cpp
//
//  依据：Qt 仿真交付（2026-10-20）——
//        “可复现启动入口 → 界面连续完成两次测量 → 两个独立结果包”
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本文件证明的是**界面入口**，不是新的算法能力：                        │
//  │   · 用交付脚本同一份 `simulation/config/` 装配（7 个 yaml 原样拷贝）； │
//  │   · 通过 `MainWindow::keyPressEvent` 的空格键触发开始 ——              │
//  │     不是直接调 `SystemInitializer::startMeasurement()`；              │
//  │   · 第一次 COMPLETE 后再按一次空格，第二次同样到 COMPLETE；           │
//  │   · 两个任务号不同、两个结果包目录不同、两份 result.json 均为         │
//  │     `state=COMPLETE`、`run_mode=simulation`；                         │
//  │   · 第二次启动前对第一包做逐文件字节快照，第二次结束后重新读回，      │
//  │     逐字节比较，证明第一包没有被第二次改写；                          │
//  │   · 两包各自检查 3 路 RAW、标定标识与 7 个配置快照是否完整。          │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ---- 证据边界（必须与结论一起写）----
//
//  · 本用例在 `QT_QPA_PLATFORM=offscreen` 下运行：键盘事件经 Qt 事件系统
//    送入 MainWindow，但**没有**人工观察真实窗口。它证明的是“界面输入
//    路径与状态机/落盘连通”，不是人机界面视觉验收。
//  · 图像由 `SyntheticTargetScene` 渲染，检测器仍是 `MockTargetDetector`，
//    纹理匹配数为 0；结果是**指定合成场景**的链路证据，不是通用场景
//    适用性、不是测量精度、不是实机链路。
//  · 两次测量之间没有重启进程；这证明同一进程可连续测量，不证明长时
//    稳定性（长跑另列）。
//
//  ---- 为什么把工作目录做成可注入 ----
//
//  一般测试用 mkdtemp；证据复跑设置 APS_QT_SIM_WORKDIR 指向仓库根
//  `验收证据/` 下的持久目录（该目录被 .gitignore 排除，但文件必须留存）。
//  `/tmp` 会被清理，**不能**作为追溯依据。
// ============================================================================

#include <gtest/gtest.h>

#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QKeyEvent>
#include <QTimer>
#include <QWidget>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <opencv2/core.hpp>

#include "algorithm/model/TargetModelManager.h"
#include "app/ApplicationContext.h"
#include "app/SystemInitializer.h"
#include "data/CameraRole.h"
#include "data/MeasurementState.h"
#include "data/MonotonicClock.h"
#include "data/SyntheticTargetGeometry.h"
#include "data/TargetModel.h"
#include "ui/MainWindow.h"

#ifndef APS_SOURCE_DIR
#error "APS_SOURCE_DIR 未定义：本套件需要它定位 simulation/config/ 与模型几何。"
#endif

using aircraft::app::ApplicationContext;
using aircraft::app::SystemInitializer;
using aircraft::data::MeasurementState;
using S = aircraft::data::MeasurementState;

namespace
{

bool writeText(const std::filesystem::path& path, const std::string& text)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out)
    {
        return false;
    }
    out << text;
    return out.good();
}

/// 与 `tools/gen_synthetic_model` **同一来源**的机型库：
/// 点表逐点取自 `data::synthetic::modelPoints()`，feature*.bin 用
/// `TargetModelManager::saveFeatureLibrary` 写出（空纹理库 N=0、D=0）。
bool writeSyntheticModelDir(const std::filesystem::path& dir, std::string& err)
{
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec)
    {
        err = "创建模型目录失败：" + dir.string() + "：" + ec.message();
        return false;
    }

    const std::vector<aircraft::data::ModelPoint3D> points =
        aircraft::data::synthetic::modelPoints();

    std::ostringstream meta;
    meta << "%YAML:1.0\n"
         << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
         << "version: 1\n"
         << "model_source: \"data/SyntheticTargetGeometry.h\"\n"
         << "feature_algorithm_version: \"none\"\n";
    if (!writeText(dir / "model.yaml", meta.str()))
    {
        err = "写 model.yaml 失败";
        return false;
    }

    std::ostringstream pts;
    pts << "%YAML:1.0\n"
        << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
        << "version: 1\n"
        << "points:\n";
    pts.setf(std::ios::fixed);
    pts.precision(6);
    for (const aircraft::data::ModelPoint3D& p : points)
    {
        pts << "  - { id: " << p.id << ", x: " << p.position.x
            << ", y: " << p.position.y << ", z: " << p.position.z
            << ", type: " << p.featureType << " }\n";
    }
    if (!writeText(dir / "points3d.yaml", pts.str()))
    {
        err = "写 points3d.yaml 失败";
        return false;
    }

    aircraft::data::TargetModel model;
    model.modelId = aircraft::data::synthetic::modelId();
    model.points3d = points;

    const aircraft::data::CameraRole roles[3] = {
        aircraft::data::CameraRole::CAM25,
        aircraft::data::CameraRole::CAM50,
        aircraft::data::CameraRole::CAM100};
    const char* const names[3] = {
        "feature25.bin", "feature50.bin", "feature100.bin"};
    for (int i = 0; i < 3; ++i)
    {
        if (!aircraft::algorithm::TargetModelManager::saveFeatureLibrary(
                (dir / names[i]).string(), roles[i], model))
        {
            err = std::string("写特征库失败：") + names[i];
            return false;
        }
    }
    return true;
}

std::string timestampTag()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    ::localtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string(buf);
}

/// 建工作目录并返回。`APS_QT_SIM_WORKDIR` 非空时在其下建 session 子目录，
/// 用于持久化证据；否则 mkdtemp。
bool makeWorkDir(std::filesystem::path& out, std::string& err)
{
    const char* fixed = std::getenv("APS_QT_SIM_WORKDIR");
    std::error_code ec;
    if (fixed != nullptr && fixed[0] != '\0')
    {
        const std::filesystem::path base(fixed);
        std::filesystem::create_directories(base, ec);
        if (ec)
        {
            err = "创建 APS_QT_SIM_WORKDIR 失败：" + base.string() + "：" +
                  ec.message();
            return false;
        }
        out = base / ("session_" + timestampTag());
        std::filesystem::create_directories(out, ec);
        if (ec)
        {
            err = "创建 session 目录失败：" + out.string() + "：" + ec.message();
            return false;
        }
        return true;
    }

    char tmpl[] = "/tmp/aps_qt_sim_XXXXXX";
    char* const dir = ::mkdtemp(tmpl);
    if (dir == nullptr)
    {
        err = "mkdtemp 失败";
        return false;
    }
    out = dir;
    return true;
}

/// 把交付用的 `simulation/config/` 原样拷到工作目录的同名相对路径下。
/// 不重写任何取值：本用例要验的就是交付入口本身。
bool copySimulationConfig(const std::filesystem::path& work,
                          std::string& err)
{
    const std::filesystem::path src =
        std::filesystem::path(APS_SOURCE_DIR) / "simulation" / "config";
    const std::filesystem::path dst = work / "simulation" / "config";
    std::error_code ec;
    std::filesystem::create_directories(dst, ec);
    if (ec)
    {
        err = "创建配置目录失败：" + dst.string() + "：" + ec.message();
        return false;
    }

    const char* const files[7] = {
        "system.yaml", "camera.yaml", "measurement.yaml", "validation.yaml",
        "optical_rig.yaml", "turntable.yaml", "trigger.yaml"};
    for (const char* f : files)
    {
        std::filesystem::copy_file(src / f, dst / f,
                                   std::filesystem::copy_options::overwrite_existing,
                                   ec);
        if (ec)
        {
            err = std::string("拷贝 ") + f + " 失败：" + ec.message();
            return false;
        }
    }
    return true;
}

/// 推进 Qt 事件循环直到 predicate 为真或超时。返回是否满足。
bool pumpUntil(const std::function<bool()>& predicate, int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs)
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        if (predicate())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    return predicate();
}

/// 把空格键送到窗口本身。走 `MainWindow::keyPressEvent` 的生产路径，
/// 不直接调 controller。
void pressSpace(aircraft::ui::MainWindow& window)
{
    QKeyEvent event(QEvent::KeyPress, Qt::Key_Space, Qt::NoModifier);
    QCoreApplication::sendEvent(&window, &event);
}

struct PackageFacts
{
    std::string dir;
    std::string resultPath;
    std::string taskId;
    std::string state;
    std::string runMode;
    bool        success = false;
};

bool readPackage(const std::string& resultPath, PackageFacts& facts,
                 std::string& err)
{
    cv::FileStorage fs(resultPath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        err = "result.json 打不开：" + resultPath;
        return false;
    }
    fs["task_id"] >> facts.taskId;
    fs["state"] >> facts.state;
    fs["run_mode"] >> facts.runMode;
    int ok = 0;
    fs["success"] >> ok;
    facts.success = (ok != 0);
    return true;
}

using FileContents = std::map<std::string, std::string>;

bool readFileBytes(const std::filesystem::path& path, std::string& out,
                   std::string& err)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        err = "打不开文件：" + path.string();
        return false;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    if (in.bad())
    {
        err = "读取文件失败：" + path.string();
        return false;
    }
    out = buf.str();
    return true;
}

/// 结果包必须自带的文件。RAW 不是“有就行”：三路都必须等于
/// 1280×1024×3 字节（BGR8 未压缩），否则半截文件也能骗过 exists()。
bool expectPackageComplete(const std::filesystem::path& dir,
                           std::string& err)
{
    const char* const topLevel[] = {
        "result.json", "failure.json", "turntable.json",
        "calibration_id.txt", "cam25.raw", "cam50.raw", "cam100.raw"};
    for (const char* name : topLevel)
    {
        const std::filesystem::path p = dir / name;
        std::error_code ec;
        if (!std::filesystem::is_regular_file(p, ec) || ec)
        {
            err = std::string("结果包缺少文件：") + p.string();
            return false;
        }
        if (std::filesystem::file_size(p, ec) == 0 || ec)
        {
            err = std::string("结果包文件为空：") + p.string();
            return false;
        }
    }

    const char* const snapshots[] = {
        "camera.yaml", "measurement.yaml", "optical_rig.yaml", "system.yaml",
        "trigger.yaml", "turntable.yaml", "validation.yaml"};
    for (const char* name : snapshots)
    {
        const std::filesystem::path p = dir / "config_snapshot" / name;
        std::error_code ec;
        if (!std::filesystem::is_regular_file(p, ec) || ec ||
            std::filesystem::file_size(p, ec) == 0 || ec)
        {
            err = std::string("结果包缺少非空配置快照：") + p.string();
            return false;
        }
    }

    constexpr std::uintmax_t kRawBytes = 1280u * 1024u * 3u;
    for (const char* name : {"cam25.raw", "cam50.raw", "cam100.raw"})
    {
        std::error_code ec;
        const std::uintmax_t size = std::filesystem::file_size(dir / name, ec);
        if (ec || size != kRawBytes)
        {
            err = std::string("RAW 尺寸不是 1280x1024x3：") + name + " = " +
                  std::to_string(static_cast<unsigned long long>(size));
            return false;
        }
    }

    std::string calibration;
    if (!readFileBytes(dir / "calibration_id.txt", calibration, err))
    {
        return false;
    }
    if (calibration.find("SYNTHETIC") == std::string::npos)
    {
        err = "calibration_id.txt 不是本次合成标定标识：" + calibration;
        return false;
    }
    return true;
}

/// 对整个结果包做逐文件快照（相对路径 → 原始字节）。
bool snapshotPackage(const std::filesystem::path& dir, FileContents& out,
                     std::string& err)
{
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec) || ec)
    {
        err = "结果包目录不存在：" + dir.string();
        return false;
    }
    for (std::filesystem::recursive_directory_iterator it(dir, ec), end;
         it != end; it.increment(ec))
    {
        if (ec)
        {
            err = "遍历结果包失败：" + dir.string() + "：" + ec.message();
            return false;
        }
        if (!it->is_regular_file(ec) || ec)
        {
            continue;
        }
        const std::filesystem::path rel = it->path().lexically_relative(dir);
        if (rel.empty())
        {
            err = "无法计算相对路径：" + it->path().string();
            return false;
        }
        std::string bytes;
        if (!readFileBytes(it->path(), bytes, err))
        {
            return false;
        }
        out[rel.generic_string()] = std::move(bytes);
    }
    if (out.empty())
    {
        err = "结果包目录中没有文件：" + dir.string();
        return false;
    }
    return true;
}

/// 第一次测量完成后与第二次测量完成后，第一包必须逐文件、逐字节相同。
void expectSnapshotsEqual(const FileContents& before, const FileContents& after,
                          const std::string& label)
{
    EXPECT_EQ(before.size(), after.size())
        << label << " 的文件数发生变化";
    for (const auto& kv : before)
    {
        const auto it = after.find(kv.first);
        EXPECT_NE(it, after.end()) << label << " 丢失文件：" << kv.first;
        if (it == after.end())
        {
            continue;
        }
        EXPECT_EQ(kv.second.size(), it->second.size())
            << label << " 文件长度变化：" << kv.first;
        EXPECT_TRUE(kv.second == it->second)
            << label << " 文件内容被改写：" << kv.first;
    }
    for (const auto& kv : after)
    {
        EXPECT_TRUE(before.count(kv.first) != 0)
            << label << " 多出文件：" << kv.first;
    }
}

}  // namespace

// ============================================================================
//  场景 Q：主界面空格键连续两次测量
// ============================================================================
TEST(QtSimulationUiTest, 界面空格连续两次测量生成两个独立结果包)
{
    std::string err;
    std::filesystem::path work;
    ASSERT_TRUE(makeWorkDir(work, err)) << err;
    ASSERT_TRUE(copySimulationConfig(work, err)) << err;
    ASSERT_TRUE(writeSyntheticModelDir(work / "simulation" / "models", err))
        << err;

    ASSERT_EQ(::chdir(work.c_str()), 0) << "chdir 失败：" << work;

    std::fprintf(stderr,
                 "\n[QtSim] 工作目录：%s\n"
                 "[QtSim] 配置：工作目录下 simulation/config（交付目录原样拷贝）\n",
                 work.c_str());

    // Qt 平台插件必须在 QApplication 构造**之前**选定。offscreen 不要求
    // 显示服务器；本用例只经事件系统驱动窗口，不做视觉验收。
    qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));

    int argc = 1;
    char name[] = "QtSimulationUiTest";
    char* argv[] = {name, nullptr};
    QApplication app(argc, argv);

    ApplicationContext ctx;
    SystemInitializer init(ctx);
    ASSERT_TRUE(init.initialize("simulation/config"))
        << "仿真配置启动失败：" << init.errorText();
    ASSERT_TRUE(init.ready());

    aircraft::ui::MainWindow window(ctx.preview.get());
    window.show();

    // 与 src/app/main.cpp 的接线逐字相同：界面只发信号，由 app 决定含义。
    QObject::connect(&window, &aircraft::ui::MainWindow::startRequested,
                     [&init, &window]() {
                         if (!init.startMeasurement())
                         {
                             window.setMeasurementView(init.view());
                         }
                     });
    QObject::connect(&window, &aircraft::ui::MainWindow::stopRequested,
                     [&init]() { init.stopMeasurement(); });

    QTimer ticker;
    QObject::connect(&ticker, &QTimer::timeout,
                     [&init, &window]() {
                         const uint64_t nowNs =
                             aircraft::data::monotonicNowNs();
                         if (init.tick(nowNs))
                         {
                             window.setMeasurementView(init.view());
                         }
                     });
    ticker.start(16);

    // 让 show 事件与首拍先走完（窗口在事件循环里才会真正展开）。
    ASSERT_TRUE(pumpUntil([] { return true; }, 100));

    auto state = [&init]() { return init.view().state; };

    // ---- 第一次：空格 → 终态 ----
    ASSERT_EQ(state(), S::IDLE) << "启动后应处于 IDLE";
    pressSpace(window);
    ASSERT_TRUE(pumpUntil([&]() { return state() != S::IDLE; }, 1000))
        << "按空格后状态机没有启动（界面信号未接到 controller？）";

    ASSERT_TRUE(pumpUntil([&]() {
        const S s = state();
        return s == S::COMPLETE || s == S::FAILED;
    }, 30000)) << "第一次测量未在 30 s 内终止，当前状态 "
                << static_cast<int>(state());

    const std::string firstDir = ctx.recorder->lastPackageDir();
    EXPECT_EQ(state(), S::COMPLETE)
        << "第一次测量未成功；错误："
        << ctx.controller->lastError().message;
    ASSERT_FALSE(firstDir.empty()) << "第一次 COMPLETE 但没有结果包";
    ASSERT_TRUE(std::filesystem::exists(
        std::filesystem::path(firstDir) / "result.json"))
        << "第一次结果包缺少 result.json：" << firstDir;

    PackageFacts first;
    first.dir = firstDir;
    first.resultPath =
        (std::filesystem::path(firstDir) / "result.json").string();
    ASSERT_TRUE(readPackage(first.resultPath, first, err)) << err;
    EXPECT_EQ(first.state, "COMPLETE");
    EXPECT_EQ(first.runMode, "simulation");
    EXPECT_TRUE(first.success);
    ASSERT_TRUE(expectPackageComplete(firstDir, err)) << err;

    // 第二次启动前把第一包整体读入内存。第二次测量结束后再读一遍，
    // 逐文件、逐字节比较 —— 仅检查“文件还在、非空”抓不到覆盖写。
    FileContents firstBefore;
    ASSERT_TRUE(snapshotPackage(firstDir, firstBefore, err)) << err;
    EXPECT_EQ(firstBefore.size(), 14u)
        << "第一包文件数异常（应为 result/failure/turntable/calibration/"
           "3 RAW + 7 配置快照 = 14），实际快照到 "
        << firstBefore.size();
    std::fprintf(stderr,
                 "[QtSim] 第一次结果包：%s（task_id=%s，已快照 %zu 个文件）\n",
                 firstDir.c_str(), first.taskId.c_str(), firstBefore.size());

    // ---- 第二次：同一窗口、同一进程，再按一次空格 ----
    pressSpace(window);
    ASSERT_TRUE(pumpUntil([&]() {
        const S s = state();
        return s != S::COMPLETE && s != S::FAILED && s != S::IDLE;
    }, 1000)) << "第二次按空格后没有开始新测量（COMPLETE 后启动被拒？）";

    ASSERT_TRUE(pumpUntil([&]() {
        const S s = state();
        return s == S::COMPLETE || s == S::FAILED;
    }, 30000)) << "第二次测量未在 30 s 内终止，当前状态 "
                << static_cast<int>(state());

    const std::string secondDir = ctx.recorder->lastPackageDir();
    EXPECT_EQ(state(), S::COMPLETE)
        << "第二次测量未成功；错误："
        << ctx.controller->lastError().message;
    ASSERT_FALSE(secondDir.empty()) << "第二次 COMPLETE 但没有结果包";
    ASSERT_TRUE(std::filesystem::exists(
        std::filesystem::path(secondDir) / "result.json"))
        << "第二次结果包缺少 result.json：" << secondDir;

    PackageFacts second;
    second.dir = secondDir;
    second.resultPath =
        (std::filesystem::path(secondDir) / "result.json").string();
    ASSERT_TRUE(readPackage(second.resultPath, second, err)) << err;
    EXPECT_EQ(second.state, "COMPLETE");
    EXPECT_EQ(second.runMode, "simulation");
    EXPECT_TRUE(second.success);
    std::fprintf(stderr, "[QtSim] 第二次结果包：%s（task_id=%s）\n",
                 secondDir.c_str(), second.taskId.c_str());

    // ---- 独立性：任务号、目录都不同；内容各自完整 ----
    EXPECT_NE(first.taskId, second.taskId)
        << "两次测量的任务号相同：结果包会互相覆盖";
    EXPECT_NE(std::filesystem::path(firstDir).filename(),
              std::filesystem::path(secondDir).filename())
        << "两次测量的结果目录同名";
    EXPECT_NE(firstDir, secondDir);
    EXPECT_NE(first.taskId, std::string());
    EXPECT_NE(second.taskId, std::string());

    // 两次都是独立的完整包，而不是第二次复用第一次的文件。
    ASSERT_TRUE(expectPackageComplete(secondDir, err)) << err;

    // 第一包在第二次测量后必须**逐字节**保持原样。第二包也做完整快照，
    // 证明两包各自独立，而不是共享/替换了部分文件。
    FileContents firstAfter;
    ASSERT_TRUE(snapshotPackage(firstDir, firstAfter, err)) << err;
    FileContents secondFiles;
    ASSERT_TRUE(snapshotPackage(secondDir, secondFiles, err)) << err;
    expectSnapshotsEqual(firstBefore, firstAfter, "第一结果包");
    EXPECT_EQ(secondFiles.size(), 14u)
        << "第二包文件数异常，实际快照到 " << secondFiles.size();
    EXPECT_TRUE(firstAfter != secondFiles)
        << "两个结果包内容完全相同：第二次没有生成独立结果";

    ticker.stop();
    window.hide();
    init.shutdown();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
}
