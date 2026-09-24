# ============================================================================
#  src/preview/CMakeLists.txt  →  libpreview
#
#  依据：ENG-03 §12.4（生成 libpreview；含 PreviewManager / PreviewWorker；
#                      依赖 data / optical）
#        ENG-01 §7（preview 模块，冻结 4 个文件）
#        SYS-09 §2.1 / §13.1、ENG-05 §2.1 / §13.1
#
#  依赖：data（数据结构与 PreviewFrame） + optical（OpticalRig，供
#  焦段可用性判断与相机标识/焦距查询）
#
#  ⚠ 本模块**不链接 Qt**。ENG-03 §12.4 把 preview 的依赖冻结为
#  data + optical，ENG-01 §17 的依赖链把 Qt 放在 ui 层，6.md §12 的
#  CMakeLists 也只链接 AircraftData。因此 PreviewWorker 用 std::thread
#  而非 QThread —— 与 SYS-09 §15 / ENG-05 §15 的偏离及其取舍理由写在
#  PreviewManager.h 与 PreviewWorker.h 的文件头（须登记进 README §6）。
#
#  这样做的直接收益：tests/preview 不需要 QCoreApplication、不需要事件
#  循环、不需要 xvfb 就能跑完整的"提交→排队→发布→取帧"链路，
#  而这正是 ENG-06 §10 要求预览可测的前提。
# ============================================================================

aps_add_module_library(preview
    DEPENDS
        data
        optical
)
