# ============================================================================
#  tests/optical/CMakeLists.txt
#
#  依据：ENG-03 §15（测试工程）、ENG-01 §15、ENG-06（测试工程设计）
#        ENG-08 §13（第一阶段测试任务）、ENG-09 §2.1 / §2.3（坐标链与单位）
#
#  依赖：data + optical
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明
#  （测试被误删时 GLOB 会让它静默消失，而测试数量减少最难被发现）。
#
#  ⚠ 本目录的测试当前**未运行**：本机未安装 GoogleTest，
#  tests/CMakeLists.txt 会提前 return() 跳过全部测试目标。
#  安装 libgtest-dev 后重新 cmake 即自动启用，无需改动本文件。
#
#  测试自造标定文件（临时目录 + cv::FileStorage 写→读往返），
#  故不依赖 tests/golden/ 的资产，也无须在此登记数据路径。理由见
#  OpticalLayerTest.cpp 文件头。
# ============================================================================

aps_add_test(test_optical_layer
    SOURCES
        OpticalLayerTest.cpp
    DEPENDS
        data
        optical
)
