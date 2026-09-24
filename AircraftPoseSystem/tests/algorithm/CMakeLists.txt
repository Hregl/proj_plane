# ============================================================================
#  tests/algorithm/CMakeLists.txt
#
#  依据：ENG-03 §15（测试工程）、ENG-06 §8（algorithm 层测试 = 算法链各阶段
#        的判据与数值）、ENG-10 §5.2 约束 4（测试必须走与生产相同的注入路径）
#        ENG-08 §13（第一阶段测试任务）
#
#  依赖：data + algorithm（ENG-03 §12.5 冻结的 algorithm 依赖集，
#        不含 optical / device —— 算法层不得反向依赖采集层）
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 的约定说明。
#
#  ⚠ 本目录的测试当前**未运行**：本机未安装 GoogleTest，
#    tests/CMakeLists.txt 会提前 return() 跳过全部测试目标。
#    安装 libgtest-dev 后重新 cmake 即自动启用，无需改动本文件。
# ============================================================================

# ----------------------------------------------------------------------------
#  为什么拆成两个目标，而不是把两个 .cpp 合成一个测试可执行文件
#
#  两个文件测的是两件不同的事：
#    · AlgorithmStageTest —— **各阶段自身**的契约（尺度估计、PnP、
#      校验、检测、匹配的输入输出与判据），桩掉彼此；
#    · PipelineTest      —— **阶段之间的装配**（投影→匹配→PnP 的接线、
#      三级坐标复合、阈值注入、B 类定位）。
#  合成一个目标后，一次报错无法从可执行文件名判断是"某个阶段坏了"还是
#  "接错了"；拆开后 `ctest -R pipeline` 可以只跑装配类用例。
#
#  两者共享 TestSupport.h：它集中定义"已配置"（各门槛值）与合成夹具。
#  这正是拆目标而非拆头文件的原因 —— 若两个文件各自定义一套门槛，
#  它们会对同一份实现给出互相矛盾却各自自洽的结论。
# ----------------------------------------------------------------------------

aps_add_test(test_algorithm_stages
    SOURCES
        AlgorithmStageTest.cpp
    DEPENDS
        data
        algorithm
)

aps_add_test(test_algorithm_pipeline
    SOURCES
        PipelineTest.cpp
    DEPENDS
        data
        algorithm
)
