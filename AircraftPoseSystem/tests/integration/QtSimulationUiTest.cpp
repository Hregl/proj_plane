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
//  │     `state=COMPLETE`、`run_mode=simulation`。                         │
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
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
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
    std::fprintf(stderr, "[QtSim] 第一次结果包：%s（task_id=%s）\n",
                 firstDir.c_str(), first.taskId.c_str());

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
    for (const PackageFacts& f : {first, second})
    {
        const std::filesystem::path dir(f.dir);
        EXPECT_TRUE(std::filesystem::exists(dir / "failure.json"))
            << "结果包缺少 failure.json：" << dir;
        EXPECT_TRUE(std::filesystem::exists(dir / "turntable.json"))
            << "结果包缺少 turntable.json：" << dir;
        EXPECT_GT(std::filesystem::file_size(dir / "result.json"), 0u);
    }

    ticker.stop();
    window.hide();
    init.shutdown();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
}
