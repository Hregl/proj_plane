# ============================================================================
#  tests/device/CMakeLists.txt
#
#  依据：ENG-03 §15（测试工程）、ENG-01 §15、ENG-06（测试工程设计）
#        ENG-08 §13（第一阶段测试任务）、SYS-06（设备抽象层）
#
#  依赖：data + device（聚合目标，含 camera / turntable / trigger 三个子库）
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明
#  （测试被误删时 GLOB 会让它静默消失，而测试数量减少最难被发现）。
#
#  ⚠ 测试启用状态（2026-10-20 更新）：本机已通过本地解包的
#  GoogleTest 1.12.1 启用原生 CTest，本目录 83 + 87 条在 ON/OFF
#  两棵构建树均实际执行并通过（证据见 evidence/正式测试入口_2026-10-20/）。
#  此前“本机未安装 GoogleTest、测试未运行”的记录仍按垫片路径保留，
#  不追改为原生结果。缺少 GoogleTest 时 tests/CMakeLists.txt 仍会
#  提前 return()；安装后重新 cmake 即自动启用，无需改动本文件。
# ============================================================================

aps_add_test(test_device_layer
    SOURCES
        DeviceLayerTest.cpp
    DEPENDS
        data
        device
)

# Pelco-D 编解码/传输/会话/后端（C-22）。源文件逐步加入，显式列举。
aps_add_test(test_pelco_d
    SOURCES
        PelcoDCodecTest.cpp
        PelcoDTransportTest.cpp
        PelcoDSessionTest.cpp
        PelcoDTurntableControllerTest.cpp
    DEPENDS
        data
        device
)

# PelcoDTurntableControllerTest.cpp 含真实 Qt 事件循环用例
# （QtEventLoopDispatchesStopWhileMoveInFlight）：需要 QCoreApplication/QTimer。
if(TARGET Qt5::Core)
    target_link_libraries(test_pelco_d PRIVATE Qt5::Core)
else()
    message(STATUS
        "[APS] test_pelco_d：Qt5::Core 未生成，Qt 事件循环用例将无法编译")
endif()
