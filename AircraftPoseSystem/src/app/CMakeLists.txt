# ============================================================================
#  src/app/CMakeLists.txt  →  可执行文件 AircraftPoseSystem
#
#  依据：ENG-03 §12.9（生成 AircraftPoseSystem；链接 ui application algorithm
#                       device preview optical infrastructure data）
#        ENG-01 §14（app 模块：QApplication / 依赖创建 / 系统启动）
#        1.md Phase 8 与《第一版代码里程碑》
#
#  职责（ENG-01 §14）：**全工程唯一的依赖装配点**。
#  app 负责按依赖顺序创建并连接所有对象：Logger → ConfigManager → 设备后端
#  → 算法链 → MeasurementController → MainWindow。其他任何模块都不允许
#  自行构造跨层对象。
#
#  这条约束的实际价值：ENG-10 §5.1 的配置注入矩阵、C-21 的 IMatchStatsStore
#  注入、以及"虚拟设备 vs 真实设备"的切换（ENG-08 §3：CAM25 真实 +
#  CAM50/100 虚拟）三件事，全部通过在此处选择具体实现类来完成。
#  若各模块自行 new 实现，替换虚拟/真实后端就需要改多处代码。
# ============================================================================

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ----------------------------------------------------------------------------
# 1 源文件
#
# 第一阶段（1.md Phase 1~9）本目录只有 main.cpp，由 Phase 8 生成。
# 在此之前为空——此时不生成可执行文件目标，见下方分支。
# ----------------------------------------------------------------------------
file(GLOB _app_sources CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

if(NOT _app_sources)
    message(STATUS
        "[APS] 可执行文件 AircraftPoseSystem：src/app/ 暂无源文件，跳过生成。\n"
        "      （main.cpp 于 1.md Phase 8 / ENG-08 Sprint 5 生成；\n"
        "        生成后无需修改任何 CMake 文件，重新 cmake 即自动生效）")
    return()
endif()

# ----------------------------------------------------------------------------
# 2 可执行文件
# ----------------------------------------------------------------------------
add_executable(AircraftPoseSystem ${_app_sources})

set_target_properties(AircraftPoseSystem PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(AircraftPoseSystem PRIVATE
    "${PROJECT_SOURCE_DIR}/src")

# ----------------------------------------------------------------------------
# 3 链接（ENG-03 §12.9 的冻结链接表）
#
# 逐个 if(TARGET) 过滤：本阶段模块库大多尚未生成，硬写会让链接期报
# "cannot find -ldata" 这类与真实原因无关的错误。
#
# ⚠ 冻结表未含 runtime：刻意保持。libruntime 只在 ComputeBackend 被实际
#   调用后才应链入，届时须先更新 ENG-03 §12.9 再改此处（见 src/runtime/
#   CMakeLists.txt 的说明）。
#
# ⚠ 冻结表未含 Qt。此处补 aps::qt5：main.cpp 要创建 QApplication 与
#   MainWindow（ENG-01 §14 > QApplication）。libui 虽已 PUBLIC 传递
#   Qt5::Widgets，但显式列出可让"app 依赖 Qt"这件事在依赖表中可见，
#   不依赖传递性。
# ----------------------------------------------------------------------------
set(_aps_link_targets "")

foreach(_module IN ITEMS
        ui
        application
        algorithm
        device
        preview
        optical
        infrastructure
        data)
    if(TARGET ${_module})
        list(APPEND _aps_link_targets ${_module})
    else()
        message(STATUS
            "[APS] 可执行文件：模块 ${_module} 尚未生成，暂不链接")
    endif()
endforeach()

target_link_libraries(AircraftPoseSystem PRIVATE
    ${_aps_link_targets}
    aps::qt5
)

message(STATUS "[APS] 可执行文件 AircraftPoseSystem 已生成（链接 ${_aps_link_targets}）")
