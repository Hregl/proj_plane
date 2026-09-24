# ============================================================================
#  tests/turntable/CMakeLists.txt
#
#  依据：ENG-06 §9.2（AlignmentController 测试：大偏差 / 小偏差 / 居中停止）、
#        ENG-06 §3（测试分层：turntable = VirtualTurntable 与对准收敛）、
#        ENG-01 §15、ENG-08 §13、SYS-10 §7/§8/§14
#
#  依赖：data + optical + application
#
#  ⚠ 本阶段（006）本目录只放 AlignmentController 的**纯几何**用例：
#  像素偏差 → 转台角度命令的换算、居中判据、越程与标定无效的拒绝路径。
#  VirtualTurntable 与"闭环收敛"用例需要设备桩，按 ENG-06 §3 的分层
#  落在 tests/integration/（与 SYS-08 §10 的收敛性用例同处）。
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明。
# ============================================================================

aps_add_test(test_alignment_controller
    SOURCES
        AlignmentControllerTest.cpp
    DEPENDS
        data
        optical
        application
)
