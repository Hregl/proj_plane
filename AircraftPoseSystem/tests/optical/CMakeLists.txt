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
#  ⚠ 测试启用状态（2026-10-20 更新）：本机已通过本地解包的
#  GoogleTest 1.12.1 启用原生 CTest，本目录 48 条在 ON/OFF 两棵
#  构建树均实际执行并通过（证据见 evidence/正式测试入口_2026-10-20/）。
#  此前“本机未安装 GoogleTest、测试未运行”的记录仍按垫片路径保留，
#  不追改为原生结果。缺少 GoogleTest 时 tests/CMakeLists.txt 仍会
#  提前 return()；安装后重新 cmake 即自动启用，无需改动本文件。
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
