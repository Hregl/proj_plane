# ============================================================================
#  tests/integration/CMakeLists.txt
#
#  依据：ENG-06 §3（测试分层：integration = Application 流程闭环
#                    + SYS-08 §10 的 7 个收敛性用例）、
#        SYS-08 §10（收敛性测试：新增，必测）、§11 约束 9
#        ENG-01 §15、ENG-08 §9（第一阶段必做项）
#
#  依赖：data + optical + preview + device + algorithm + application
#
#  ⚠ 本目录是全工程**唯一**能证明"单次测量任务必然终止"（§11 约束 9）的
#  测试位置。它需要四类注入式桩：VirtualCameraBackend（此处用
#  VirtualCameras：直接在内存里合成图像与目标框）、VirtualTurntable、
#  算法桩（StubPipeline）、以及可注入的短 T_task。
#
#  ⚠ 不依赖 Qt：application 与 preview 都不链接 Qt，故整个闭环可以在
#  没有 QApplication 与事件循环的条件下被逐 tick 推导。这正是把状态机
#  放在 application 层的收益（ENG-03 §20 第 6 条）。
#
#  ⚠ 例外是文件末尾的两个目标，它们都编译 src/app 的源文件并链接 ui：
#  · test_system_initializer —— `SystemInitializer.h` 含
#    `ui/MeasurementView.h`（`<QString>`），故链接 ui / Qt5::Widgets；
#    它**不起事件循环**，仍是逐 tick 调用的。
#  · test_qt_simulation_ui —— 需要 `MainWindow` 与 Qt 事件循环，验证
#    空格键启动路径（见该目标处的说明）。
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明。
# ============================================================================

# ============================================================================
#  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
# ============================================================================

aps_add_test(test_measurement_flow
    SOURCES
        MeasurementFlowTest.cpp
    DEPENDS
        data
        optical
        preview
        algorithm
        device
        application
)

# ----------------------------------------------------------------------------
#  结果包落盘（C-02 Step 5）
#
#  ⚠ 本目录下**唯一**依赖 infrastructure 的测试：Recorder 住在 infrastructure
#    （ENG-01 §11：该层是**唯一**允许做文件 I/O 的层），而 C-02 的统计量/新增
#    事实字段最终都要落进它产出的 result.json，故必须在这里验一次"文件真的
#    写得出来、且事后解析得回去"。
#
#  ⚠ 判据是**解析级**的，不是子串匹配：`nan` 与"少一个逗号"都能让
#    `find("yaw")` 蒙混过去，却让整份包事后完全读不出来
#    （见 V2.1-C02_实施设计说明.md §12.3.2 / §12.4 的 M8、M10）。
#
#  ⚠ 覆盖缺口：`RecorderSinkAdapter`（src/app）**测不到** —— src/app 只有
#    可执行文件目标，没有库目标，故 adapter 本身链不进任何测试。
#    本套件用 `Recorder` 直连，验的是它下游那一段。
#
#    ⚠ 本条原写作"src/app 只构建可执行文件，测试链不进去"。该前提
#      **已不再成立**：下方 test_system_initializer 以**单个源文件**为单位
#      把 `SystemInitializer.cpp` 编进了独立测试目标（见该处的说明）。
#      app 层仍无库目标，仍未覆盖的是 `main.cpp` 与 `RecorderSinkAdapter`。
# ----------------------------------------------------------------------------
aps_add_test(test_recorder_package
    SOURCES
        RecorderPackageTest.cpp
    DEPENDS
        data
        infrastructure
)

# ----------------------------------------------------------------------------
#  app 层落盘适配器（C-009 / H-002）
#
#  ⚠ 覆盖缺口修正：此前这里写着"`RecorderSinkAdapter` 测不到"，理由是
#    "src/app 只有可执行目标，adapter 链不进测试"。该理由**不成立** ——
#    `RecorderSinkAdapter` 是 `app/ApplicationContext.h` 里的**内联类**，
#    包含头即可编译，不需要 libapp。本目标即为它补上判据（7 个用例）。
#
#  ⚠ 依赖只需 data + infrastructure：适配器内联调用 `Recorder::save()`，
#    其余头文件仅用于类型声明。Qt 亦不需要（ApplicationContext.h 不含 Qt 头）。
# ----------------------------------------------------------------------------
aps_add_test(test_recorder_adapter
    SOURCES
        RecorderAdapterTest.cpp
    DEPENDS
        data
        infrastructure
    WORKING_DIRECTORY
        "${PROJECT_SOURCE_DIR}"
)

# 该用例按绝对路径检查仓库 config/（见 RecorderAdapterTest.cpp 的说明）。
# WORKING_DIRECTORY 之外再传入编译期宏：换 cwd 也不会退化成“找不到配置”。
target_compile_definitions(test_recorder_adapter PRIVATE
    APS_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

# ----------------------------------------------------------------------------
#  app 层装配与空闲补帧（R04 收尾，2026-09-24 评审裁定）
#
#  依据：ENG-01 §14（app = 唯一装配点）、ENG-02 §15（冻结创建顺序）
#        SYS-08 §7.7（能力边界不重试 → 2002）、§8（AUTO 显示源映射）
#
#  ⚠ 本目标**不新增 app 库、不重构生产架构**：把 `src/app/SystemInitializer.cpp`
#    直接列为 SOURCES（`add_executable` 原样接受测试目录之外的相对路径）。
#    动机是那个文件里的空闲补帧规则（`!stateAdvanced`）此前**没有任何测试**，
#    而它挡的是一条可达边界（ALIGN 越程 → 当拍 FAILED）。
#
#  ⚠ 依赖 `ui` 只为 `ui/MeasurementView.h` 的 `<QString>`：SystemInitializer::view()
#    返回它，而 ui 目标 PUBLIC 链到 Qt5::Widgets。测试本身不起 QApplication、
#    不开窗口（`view()` 是纯读）。
#
#  ⚠ 测试启用状态（2026-10-20 更新）：本机已通过本地解包的 GoogleTest
#    1.12.1 启用原生 CTest，本目标 26 条在 ON/OFF 两棵构建树均实际执行
#    并通过（证据见 evidence/正式测试入口_2026-10-20/）。此前“本机未装
#    GoogleTest、只跑垫片”的记录仍按垫片路径保留，不追改为原生结果。
#    缺少 GoogleTest 时 tests/CMakeLists.txt 仍会在配置期 return()。
# ----------------------------------------------------------------------------
aps_add_test(test_system_initializer
    SOURCES
        SystemInitializerTest.cpp
        ../../src/app/SystemInitializer.cpp
    DEPENDS
        data
        optical
        preview
        algorithm
        device
        application
        infrastructure
        ui
)

# ⚠ 仓库路径由编译期宏传入（该测试需要把仓库 config/ 的 7 个 yaml 拷进
#   临时工作目录，见用例内的说明）。**必须已展开**为绝对路径 ——
#   `${PROJECT_SOURCE_DIR}` 在配置期展开，正是这一点。
target_compile_definitions(test_system_initializer PRIVATE
    APS_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

# ----------------------------------------------------------------------------
#  Qt 仿真交付：界面空格键连续两次测量（2026-10-20）
#
#  依据：交付要求“可复现启动入口 → 界面连续完成两次测量 → 两个独立结果包”。
#
#  ⚠ 本目标验的是**界面入口**，不是新的算法能力：
#    · 配置直接拷贝 `simulation/config/`（交付脚本使用的同一份）；
#    · 开始动作由 `QKeyEvent(Qt::Key_Space)` 送入 MainWindow 触发，
#      不直接调 `SystemInitializer::startMeasurement()`；
#    · 两次 COMPLETE 的任务号/结果目录必须不同。
#
#  ⚠ 运行在 `QT_QPA_PLATFORM=offscreen`（用例内设置）：这是输入路径与
#    落盘连通的证据，**不是**人工观察窗口的视觉验收。
#
#  ⚠ 与 test_system_initializer 一样，把 `SystemInitializer.cpp` 列为源文件；
#    app 层仍无库目标。本目标需要 ui（MainWindow）与 Qt 事件循环。
# ----------------------------------------------------------------------------
aps_add_test(test_qt_simulation_ui
    SOURCES
        QtSimulationUiTest.cpp
        ../../src/app/SystemInitializer.cpp
    DEPENDS
        data
        optical
        preview
        algorithm
        device
        application
        infrastructure
        ui
)

target_compile_definitions(test_qt_simulation_ui PRIVATE
    APS_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
