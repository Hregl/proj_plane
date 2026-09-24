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
#  ⚠ 唯一例外是 test_system_initializer（文件末尾）：它编译 src/app 的一个
#  源文件，而 `SystemInitializer.h` 含 `ui/MeasurementView.h`（`<QString>`），
#  故它链接 ui / Qt5::Widgets。它同样**不起事件循环**，仍是逐 tick 调用的。
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明。
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
#  ⚠ 本目标今日**无法在 CMake 侧实测**：本机未装 GoogleTest，tests/CMakeLists.txt
#    在配置期就 return()，全部测试目标（含既有的 10 个）都不生成。
#    实际执行走 `/tmp/gtshim/build_tests.sh`（见 README §6），那里同样把
#    `src/app/SystemInitializer.cpp` 编进该目标。两条路径中已跑过的是 shim。
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
