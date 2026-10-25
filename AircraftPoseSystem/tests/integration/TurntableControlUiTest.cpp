// ============================================================================
//  tests/integration/TurntableControlUiTest.cpp
//
//  主程序“受控转台操作”面板的操作演示（离线，VirtualTurntable）：
//    MainWindow 面板 → MainWindow 信号 → app::TurntableControlService
//    → 既有 ITurntableController；
//  并验证面板确实把“已连接 / 受理 / 交付 / 反馈到位 / 停止结果”分开展示。
//
//  ⚠ 本用例是**界面输入路径与显示语义**的证据，不是实机证据：
//    虚拟转台不会产生任何协议帧，停止结果明确显示“无协议帧”。
//  ⚠ 面板信号与 controller 之间仍隔着 `TurntableControlService`；
//    UI 层不判断“能否定位”，与 src/app/main.cpp 的接线原则一致。
// ============================================================================

#include <gtest/gtest.h>

#include <QApplication>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QtGlobal>

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "app/TurntableControlService.h"
#include "data/TurntableConfig.h"
#include "data/TurntableMotionState.h"
#include "device/turntable/VirtualTurntable.h"
#include "ui/MainWindow.h"
#include "ui/TurntableControlView.h"

namespace aircraft
{
namespace app
{
namespace
{

using Stage = ui::TurntableCommandStage;
using StopStage = ui::TurntableStopStage;
using S = data::TurntableMotionState;

data::TurntableConfig makeSimulationConfig()
{
    data::TurntableConfig cfg;
    cfg.protocol        = "sdk";       // VirtualTurntable
    cfg.azimuthMin      = -180.0;
    cfg.azimuthMax      = 180.0;
    cfg.elevationMin    = -60.0;
    cfg.elevationMax    = 60.0;
    cfg.coarseSpeed     = 5.0;
    cfg.fineSpeed       = 1.2;
    cfg.centerThreshold = 50.0;
    cfg.arrivalToleranceDeg = 0.2;
    return cfg;
}

QString labelText(QWidget* parent, const char* name)
{
    QLabel* label = parent->findChild<QLabel*>(QString::fromLatin1(name));
    return label == nullptr ? QStringLiteral("<missing>") : label->text();
}

TEST(TurntableControlUiTest, 主界面定位停止与连续操作演示)
{
    qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));

    int argc = 1;
    char name[] = "TurntableControlUiTest";
    char* argv[] = {name, nullptr};
    QApplication app(argc, argv);

    const data::TurntableConfig cfg = makeSimulationConfig();
    device::VirtualTurntable turntable(cfg);
    ASSERT_TRUE(turntable.initialize());

    // 与生产装配一致：诊断接口为 nullptr（虚拟后端没有并列诊断）。
    TurntableControlService service(turntable, cfg, nullptr);

    ui::MainWindow window(nullptr);
    window.show();

    // 与 src/app/main.cpp 相同的接线：界面只发意图，app 层决定含义。
    std::string lastError;
    QObject::connect(
        &window, &ui::MainWindow::turntablePositionRequested,
        [&service, &lastError](double az, double el) {
            std::string error;
            if (!service.requestMove(az, el, error))
            {
                lastError = error;
            }
            else
            {
                lastError.clear();
            }
        });
    QObject::connect(&window, &ui::MainWindow::turntableStopRequested,
                     [&service]() { service.requestStop(); });

    auto refresh = [&]() {
        service.poll();
        window.setTurntableControlView(service.view());
    };
    auto pumpUntil = [&](const std::function<bool()>& predicate,
                         int timeoutMs) {
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < timeoutMs)
        {
            refresh();
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            if (predicate())
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        refresh();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        return predicate();
    };

    ASSERT_TRUE(pumpUntil([] { return true; }, 100));

    QDoubleSpinBox* azimuthInput =
        window.findChild<QDoubleSpinBox*>(QStringLiteral("turntableAzimuthInput"));
    QDoubleSpinBox* elevationInput =
        window.findChild<QDoubleSpinBox*>(QStringLiteral("turntableElevationInput"));
    QPushButton* positionButton =
        window.findChild<QPushButton*>(QStringLiteral("turntablePositionButton"));
    QPushButton* stopButton =
        window.findChild<QPushButton*>(QStringLiteral("turntableStopButton"));
    ASSERT_NE(azimuthInput, nullptr);
    ASSERT_NE(elevationInput, nullptr);
    ASSERT_NE(positionButton, nullptr);
    ASSERT_NE(stopButton, nullptr);

    // 首屏：连接/样本/许可必须可见，且明确标注虚拟后端。
    ASSERT_TRUE(pumpUntil([&] { return positionButton->isEnabled(); }, 1000))
        << "面板没有在拿到有效样本后放行定位按钮";
    EXPECT_EQ(labelText(&window, "turntableConnectionLabel"),
              QString::fromUtf8("已连接（虚拟转台）"));
    EXPECT_EQ(labelText(&window, "turntableSampleLabel"),
              QString::fromUtf8("有效 / 新鲜"));
    EXPECT_NE(labelText(&window, "turntablePermitLabel")
                  .contains(QString::fromUtf8("不适用验收许可")),
              false);

    // ---- 定位：受理 → 交付 → 反馈到位 ----
    azimuthInput->setValue(2.0);
    elevationInput->setValue(0.5);
    positionButton->click();
    EXPECT_TRUE(lastError.empty()) << lastError;

    // 运动中定位按钮应禁用，停止按钮仍可用。
    EXPECT_TRUE(pumpUntil(
        [&] {
            return labelText(&window, "turntableCommandStageLabel") ==
                       QString::fromUtf8("指令已交付") &&
                   !positionButton->isEnabled();
        },
        2000)) << labelText(&window, "turntableCommandDetailLabel").toStdString();
    EXPECT_TRUE(stopButton->isEnabled());

    ASSERT_TRUE(pumpUntil(
        [&] {
            return labelText(&window, "turntableCommandStageLabel") ==
                   QString::fromUtf8("反馈已到位");
        },
        5000)) << labelText(&window, "turntableCommandDetailLabel").toStdString();
    EXPECT_EQ(labelText(&window, "turntableMotionLabel"),
              QStringLiteral("STABLE"));
    EXPECT_TRUE(labelText(&window, "turntableFeedbackLabel")
                    .contains(QStringLiteral("2.0000 / 0.5000")));
    EXPECT_TRUE(positionButton->isEnabled())
        << "到位后应允许下一条命令";

    // ---- 测量任务进行中：手动定位必须禁用并给出原因 ----
    window.setTurntableControlView(service.view(/*measurementActive=*/true));
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    EXPECT_FALSE(positionButton->isEnabled());
    EXPECT_TRUE(labelText(&window, "turntableGateLabel")
                    .contains(QString::fromUtf8("测量任务进行中")));

    // ---- 运动中停止：结果是“无协议帧/就地停止”，不是设备 ACK ----
    // 先恢复一次快照：上一段注入的是“测量进行中”，定位按钮此时
    // 必须是禁用的；恢复后才能继续操作。
    ASSERT_TRUE(pumpUntil([&] { return positionButton->isEnabled(); }, 1000));
    azimuthInput->setValue(-1.0);
    elevationInput->setValue(-0.5);
    positionButton->click();
    ASSERT_TRUE(pumpUntil(
        [&] {
            return labelText(&window, "turntableCommandStageLabel") ==
                   QString::fromUtf8("指令已交付");
        },
        2000));

    stopButton->click();
    ASSERT_TRUE(pumpUntil(
        [&] {
            return labelText(&window, "turntableCommandStageLabel") ==
                       QString::fromUtf8("被停止取消") &&
                   labelText(&window, "turntableStopStageLabel") ==
                       QString::fromUtf8("未发送停止帧");
        },
        2000)) << labelText(&window, "turntableStopDetailLabel").toStdString();
    EXPECT_TRUE(labelText(&window, "turntableStopDetailLabel")
                    .contains(QString::fromUtf8("不是设备 ACK")));
    EXPECT_EQ(turntable.state().motion, S::IDLE);

    // ---- 停止后继续操作，最终再次到位 ----
    azimuthInput->setValue(0.0);
    elevationInput->setValue(0.0);
    positionButton->click();
    ASSERT_TRUE(pumpUntil(
        [&] {
            return labelText(&window, "turntableCommandStageLabel") ==
                   QString::fromUtf8("反馈已到位");
        },
        5000)) << labelText(&window, "turntableCommandDetailLabel").toStdString();
    EXPECT_EQ(turntable.state().motion, S::STABLE);
    EXPECT_NEAR(turntable.state().azimuth, 0.0, 1e-6);
    EXPECT_NEAR(turntable.state().elevation, 0.0, 1e-6);

    // 新定位开始后，上一次停止结果不得继续显示为当前状态。
    EXPECT_EQ(labelText(&window, "turntableStopStageLabel"),
              QString::fromUtf8("未请求停止"));

    // 整轮操作不产生“故障”文字。
    EXPECT_TRUE(labelText(&window, "turntableFaultLabel") ==
                QString::fromUtf8("—"))
        << labelText(&window, "turntableFaultLabel").toStdString();
}

}  // namespace
}  // namespace app
}  // namespace aircraft
