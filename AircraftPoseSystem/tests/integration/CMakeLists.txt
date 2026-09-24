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
#  ⚠ 覆盖缺口：`RecorderSinkAdapter`（src/app）**测不到** —— src/app 只构建
#    可执行文件，测试链不进去。本套件用 `Recorder` 直连，验的是它下游那一段。
# ----------------------------------------------------------------------------
aps_add_test(test_recorder_package
    SOURCES
        RecorderPackageTest.cpp
    DEPENDS
        data
        infrastructure
)
