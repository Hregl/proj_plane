// ============================================================================
//  src/app/main.cpp
//
//  依据：9.md §十（程序入口）、§十三（主程序依赖关系，冻结）
//        10.md §九（009 的启动序列）、11.md §四（010 的首次闭环验证）
//        ENG-01 §14（app 模块：QApplication / 依赖创建 / 系统启动）
//        ENG-02 §15（对象创建顺序：… → MeasurementController → MainWindow）
//        ENG-03 §12.9（可执行文件 AircraftPoseSystem，链接 8 个模块）
//        SYS-08 §9（状态机运行于 Application 线程）
//
//  本文件是**装配的最后一环**：建 QApplication → SystemInitializer::initialize()
//  → 建 MainWindow → 把两者接上定时器。全部装配逻辑在 SystemInitializer，
//  此处只负责 Qt 的部分（QApplication 必须先于任何 QWidget 存在）。
//
//  ---- 为什么 QApplication 必须在 initialize() 之前 ----
//
//  ENG-02 §15 的顺序是"… → MeasurementController → MainWindow"，MainWindow
//  排最后一项。而 QApplication 必须先于**任何** QWidget 构造，故它是
//  main() 的第一件事。这中间有个容易忽略的后果：如果 QApplication 的构造
//  与 initialize() 之间有耗时操作（读 7 个 yaml、开三个相机、起预览线程），
//  Qt 已经装好了事件循环但还没跑 exec()，此时不会派发任何事件 ——
//  这是正常的，不是卡死。故 initialize() 的进度用 stderr/日志体现，
//  而不是指望界面先画出来再显示"正在初始化"。
//
//  ---- 定时器：为什么是 16 ms ----
//
//  它同时驱动三件事（见 SystemInitializer::tick 的说明）：
//    1. MeasurementController::tick() —— SYS-08 §7.6 约束 3 要求状态机
//       "在每个事件循环中检查 deadlineExceeded"，故节拍必须**远小于**
//       最短的状态超时（validate 的 1.0e8 ns = 100 ms）。16 ms 对 100 ms
//       有 6 倍余量，超时判定不会因为节拍粗而迟到一整个周期。
//    2. 空闲态的预览采集（相机以自由运行模式出图）。
//    3. 界面刷新（与 MainWindow 自己的 16 ms 取帧定时器独立 ——
//       那个只管从 PreviewManager 取已发布的帧，两者不冲突）。
//
//  ---- 退出时的顺序是刻意的 ----
//
//  窗口关闭 → event loop 退出 → （栈回退）window 析构 → init 析构 →
//  ctx 析构。ctx 的成员按声明的逆序销毁：controller → sink → recorder →
//  pipeline → stats → models → previewWorker → preview → … → config。
//  每一对相邻的"持有者/被持有者"都满足"持有者先死"（见
//  ApplicationContext.h 的说明）。若把 window 建成 main 的局部对象而 ctx
//  建成堆对象，就会反过来：ctx 先于 window 销毁，而 window 的定时器还在
//  跑并访问已销毁的 preview —— 表现为关闭程序时偶发段错误。
//  故这里全部用**栈对象**，让作用域与析构顺序由语言保证。
// ============================================================================

#include <QApplication>
#include <QTimer>

#include <cstdio>
#include <string>

#include "app/ApplicationContext.h"
#include "app/SystemInitializer.h"
#include "data/MonotonicClock.h"
#include "ui/MainWindow.h"
#include "ui/MeasurementView.h"

namespace
{

/// 界面刷新节拍，ms。取值理由见文件头。
constexpr int kTickIntervalMs = 16;

}  // namespace

int main(int argc, char* argv[])
{
    // ---- 高 DPI ----
    // 必须在 QApplication 构造**之前**设置（Qt 5 的文档明确要求），
    // 之后设置会静默无效。现场一体机常用 4K 屏，不开启时整个界面
    // 会以 1 倍逻辑像素渲染，文字小到无法判读——
    // 而这是交付到靶场后才会暴露的问题（开发机通常是 1080p）。
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);

    QApplication::setOrganizationName(QStringLiteral("APS"));
    QApplication::setApplicationName(QStringLiteral("AircraftPoseSystem"));
    QApplication::setApplicationVersion(QStringLiteral("2.1"));

    // ---- 配置目录 ----
    // 允许 `AircraftPoseSystem /path/to/config` 指定，默认相对当前目录的
    // "config"。现场部署时用绝对路径（见 config/system.yaml 的书写约定：
    // 从不同目录启动会让日志落到不同地方，排障前先要花时间找文件）。
    std::string configDir = "config";
    if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0')
    {
        configDir = argv[1];
    }

    // ---- 装配 ----
    // ctx 在 init **之前**声明，故它比 init 后销毁：init 析构时不会去动
    // 已销毁的 ctx 成员（本类析构是 = default，事实上不访问，但顺序仍
    // 按依赖方向安排好，免得将来加了清理逻辑才发现反了）。
    aircraft::app::ApplicationContext ctx;
    aircraft::app::SystemInitializer  init(ctx);

    if (!init.initialize(configDir))
    {
        // 启动失败按 ENG-10 §5.3 的第一档处理：报错并退出。
        // **不做半启动**：没有配置就没有可信的相机数、标定与阈值，
        // 带着默认值跑起来比直接不启动更危险 —— 操作者会看到界面，
        // 以为系统是好的。
        std::fprintf(stderr,
                     "\n启动失败：%s\n"
                     "  配置目录：%s\n"
                     "  提示：确认该目录下存在 system.yaml 等 7 个配置文件，"
                     "且 %s 指向的路径可解析。\n\n",
                     init.errorText().c_str(), configDir.c_str(),
                     "system.yaml");
        return 1;
    }

    // ---- 主界面 ----
    // 注入顺序：先构造（窗口会立刻启动自己的取帧定时器），再 setPreviewManager
    // 也不迟 —— 但 PreviewManager 在 initialize() 里就已就绪，故直接走构造参数。
    aircraft::ui::MainWindow window(ctx.preview.get());
    window.show();

    // ---- 把状态机、预览与界面接上 ----
    // 界面只发"用户按了开始/停止"，由 app 决定这意味着什么
    // （见 MainWindow.h 的 startRequested 说明）。控制器不出现在 ui 层。
    QObject::connect(&window, &aircraft::ui::MainWindow::startRequested,
                     [&init, &window]() {
                         if (!init.startMeasurement())
                         {
                             // 启动被拒（例如可用相机数 ≤ 1 → 1001）。
                             // 立刻刷一次界面，让拒绝的原因可见 ——
                             // 否则操作者的感受是"按了空格没反应"。
                             window.setMeasurementView(init.view());
                         }
                     });
    QObject::connect(&window, &aircraft::ui::MainWindow::stopRequested,
                     [&init]() { init.stopMeasurement(); });

    QTimer ticker;

    // 首拍标记（R02 收尾，2026-09-24 审查报告）。
    //
    // 一次性，写在**真实定时器回调**的最后，而不是 `SystemInitializer::tick()`
    // 的入口。理由：入口只能证明"进入了这个函数"，排除不了卡在首拍内部
    // （预览线程没起来、采集阻塞、界面刷新里死锁）；只有写在回调**正常
    // 走完**之后，它才等价于"初始化完成，且事件循环至少完成了一次应用
    // 定时回调" —— 这正是安装自检 install_launch_check 要断言的事情
    // （只判退出码 124 是不够的：初始化里卡死同样满足 124，见
    //  cmake/InstallRules.cmake 的说明）。
    bool firstTickLogged = false;

    QObject::connect(&ticker, &QTimer::timeout,
                     [&init, &window, &ctx, &firstTickLogged]() {
        const uint64_t nowNs = aircraft::data::monotonicNowNs();

        const bool changed = init.tick(nowNs);

        // 界面刷新条件：
        //   · 状态机有变化（含进入终止态），或
        //   · 空闲态刚提交了一帧（让"就绪"时的转台/状态文字也能跟上），或
        //   · 有告警文字需要显示。
        // 不做"每拍都刷"：setMeasurementView 虽然内部是 setText，
        // 但每 16 ms 走一遍四个面板的布局，在国产 CPU 上是可观测的开销。
        if (changed)
        {
            window.setMeasurementView(init.view());
        }

        if (!firstTickLogged)
        {
            firstTickLogged = true;

            // 走日志通道（终端与日志文件都留下痕迹）。logger 未就绪时
            // 退化为 stderr —— 与 SystemInitializer::log() 的回落写法同构，
            // 使"该有的痕迹一行都不能少"这件事不依赖日志子系统是否起来。
            if (ctx.logger)
            {
                ctx.logger->info("app", "APP_FIRST_TICK_COMPLETED");
            }
            else
            {
                std::fprintf(stderr, "APP_FIRST_TICK_COMPLETED\n");
            }
        }
    });
    ticker.start(kTickIntervalMs);

    // 首屏：立刻把"就绪 / 特征库缺失 / 降级"这些启动结论显示出来，
    // 而不是等第一拍（16 ms 后）—— 启动告警越早可见越好。
    window.setMeasurementView(init.view());

    // ---- 启动结论（终端侧）----
    // 与日志文件双写：现场可能没有终端，开发期可能不看日志文件。
    const std::vector<std::string>& warnings = init.warnings();
    if (!warnings.empty())
    {
        std::fprintf(stderr, "\n启动告警 %zu 条：\n", warnings.size());
        for (const std::string& w : warnings)
        {
            std::fprintf(stderr, "  [WARN] %s\n", w.c_str());
        }
        std::fprintf(stderr, "（以上同样写入了日志文件）\n\n");
    }

    return app.exec();
}
