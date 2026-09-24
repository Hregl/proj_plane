# ============================================================================
#  tests/preview/CMakeLists.txt
#
#  依据：ENG-06 §10（PreviewManager / PreviewQueue 的测试要求）、
#        ENG-06 §6（预览测试目录）、ENG-01 §15（tests 目录清单）、
#        ENG-08 §13、SYS-09 §2.1 / §13.1
#
#  依赖：data + optical + preview
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明
#  （测试被误删时 GLOB 会让它静默消失，而测试数量减少最难被发现）。
#
#  ⚠ 本目录的测试**不需要 Qt**：preview 模块不链接 Qt（ENG-03 §12.4），
#  PreviewWorker 是 std::thread 而非 QThread，故无需 QCoreApplication、
#  事件循环或 xvfb 即可运行完整链路。这也是当初取舍 SYS-09 §15 的
#  直接收益（理由见 PreviewManager.h / PreviewWorker.h 文件头）。
#
#  ⚠ 本目录的测试当前**未运行**：本机未安装 GoogleTest，
#  tests/CMakeLists.txt 会提前 return() 跳过全部测试目标。
#  安装 libgtest-dev 后重新 cmake 即自动启用，无需改动本文件。
# ============================================================================

aps_add_test(test_preview_layer
    SOURCES
        PreviewLayerTest.cpp
    DEPENDS
        data
        optical
        preview
)
