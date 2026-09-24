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

# ----------------------------------------------------------------------------
# 4 安装（R02，2026-09-24 审查报告）
#
# 模块库的 install(TARGETS ...) 由 src/CMakeLists.txt 的 aps_add_module_library()
# 统一给出（理由见该处注释）；可执行文件不经过那个函数，故在此单独安装。
#
# ⚠ 只写 RUNTIME：可执行文件没有 ARCHIVE/LIBRARY 产物。写成三种全给在这里
#   不算错，但会让人以为本 target 也可能是库。库与可执行文件的安装规则
#   分开写、各自只列真正会产生的产物类型，正是为了让"谁装了什么"可读。
#
# ⚠ 本行**不可删**：紧随其后的 install(CODE ...) 断言
#   bin/AircraftPoseSystem 必须存在，删掉这里会让 `cmake --install` 直接失败。
# ----------------------------------------------------------------------------
install(TARGETS AircraftPoseSystem
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

# ----------------------------------------------------------------------------
# 5 安装自检断言（在 `cmake --install` **内部**执行）
#
# R02（2026-09-24 审查报告）暴露的缺陷：全工程 `install(TARGETS ...)` 实体 0 处，
# 于是 `deploy/` 只有空的 config/models/calibration，**没有 bin/、没有 lib/**——
# 一个没有主程序的部署目录。而原 install_check 只跑 `cmake --install`、
# 不校验任何产物，所以这个缺陷不被任何检查覆盖。
#
# 本断言让"装了个没有主程序的目录"**从此不可能成功**，无论谁手工敲安装命令。
#
# ⚠ 断言**两件事**：bin/ 里有可执行文件，且 lib/ 里**每一个**已建立的模块库
#   都在。只断言 bin/ 会漏掉 R02 缺陷的一半 —— 报告原文就是"没有 bin/、没有 lib/"。
#   这也不与"全静态构建下 lib/ 运行期不需要"矛盾：这里断言的不是"运行需要它"，
#   而是"我们 created 的每个模块库真的被装了出去"。
#
# ⚠ 为什么写在**这里**而不是 cmake/InstallRules.cmake：见该文件 §5 的说明 ——
#   CMake 生成的 install 脚本中，父目录自己的规则全部早于子目录的 include()，
#   写在 InstallRules 里的断言会在 bin/ 被拷贝之前执行，必然失败。
#   凡断言"某个 target 已装好"的 CODE，都要与那条 install(TARGETS ...)
#   写在同一个文件里、且排在其后。
#
# 转义要点（这一步最容易写错）：
#   ${CMAKE_INSTALL_BINDIR}   —— **配置期**展开（GNUInstallDirs 已定义），不转义；
#   \${CMAKE_INSTALL_PREFIX}  —— **安装期**读取，必须 `\$` 转义。两者写反了
#                                会得到一个"配置期的前缀 + 安装期的 bin"的怪路径；
#   \$ENV{DESTDIR}            —— 安装期读取；DESTDIR 只影响实际落盘位置，
#                                **不**并入 CMAKE_INSTALL_PREFIX 变量，故须显式前置。
#
# 期望的库文件名在**配置期**算好再烘进 CODE：APS_INSTALLED_LIBRARY_TARGETS 是
# 全局属性，本文件（src/app）在 src/CMakeLists.txt 中最后 add_subdirectory，
# 故此刻已收齐全部模块 —— 以后新增模块会自动进入这个断言，不必回来改这里。
# 库前缀/后缀按 APS_LIBRARY_TYPE 取 GNUInstallDirs 的变量，故 STATIC/SHARED 都对。
# ----------------------------------------------------------------------------
get_property(_aps_lib_targets GLOBAL PROPERTY APS_INSTALLED_LIBRARY_TARGETS)

if(APS_LIBRARY_TYPE STREQUAL "SHARED")
    set(_aps_lib_prefix "${CMAKE_SHARED_LIBRARY_PREFIX}")
    set(_aps_lib_suffix "${CMAKE_SHARED_LIBRARY_SUFFIX}")
else()
    set(_aps_lib_prefix "${CMAKE_STATIC_LIBRARY_PREFIX}")
    set(_aps_lib_suffix "${CMAKE_STATIC_LIBRARY_SUFFIX}")
endif()

set(_aps_expected_libs "")
foreach(_aps_t IN LISTS _aps_lib_targets)
    list(APPEND _aps_expected_libs "${_aps_lib_prefix}${_aps_t}${_aps_lib_suffix}")
endforeach()
list(LENGTH _aps_expected_libs _aps_lib_count)

install(CODE "
    set(_aps_exe \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/AircraftPoseSystem\")

    if(NOT EXISTS \"\${_aps_exe}\")
        message(FATAL_ERROR
            \"[APS] 安装自检失败：安装前缀下没有可执行文件 \${_aps_exe}\\n\"
            \"  → 检查本文件是否仍有 install(TARGETS AircraftPoseSystem ...)，\\n\"
            \"     以及构建时是否真的生成了 AircraftPoseSystem 目标。\")
    endif()

    set(_aps_libdir \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\")
    set(_aps_missing \"\")
    foreach(_aps_lib IN ITEMS ${_aps_expected_libs})
        if(NOT EXISTS \"\${_aps_libdir}/\${_aps_lib}\")
            list(APPEND _aps_missing \"\${_aps_lib}\")
        endif()
    endforeach()
    if(_aps_missing)
        message(FATAL_ERROR
            \"[APS] 安装自检失败：模块库未安装到 \${_aps_libdir}\：\${_aps_missing}\\n\"
            \"  → 检查 src/CMakeLists.txt 的 aps_add_module_library() 里是否仍有\\n\"
            \"     install(TARGETS \\\${target} ...)。\")
    endif()
    message(STATUS \"[APS] 安装自检：\${_aps_libdir} 下 ${_aps_lib_count} 个模块库齐备\")

    # 可执行位。用 test(1) 而非 CMake 内置判定——CMake 没有 IS_EXECUTABLE。
    # 若本机没有 test 命令，只提示、不失败：检查器缺失不应让安装失败。
    execute_process(COMMAND test -x \"\${_aps_exe}\" RESULT_VARIABLE _aps_test_rc)
    if(_aps_test_rc MATCHES \"^[0-9]+\")
        if(NOT _aps_test_rc EQUAL 0)
            message(FATAL_ERROR
                \"[APS] 安装自检失败：\${_aps_exe} 存在但**没有可执行权限**。\")
        endif()
    else()
        message(STATUS \"[APS] 安装自检：本机无 test 命令，跳过可执行位检查\")
    endif()

    message(STATUS \"[APS] 安装自检通过：\${_aps_exe}\")
")

message(STATUS "[APS] 可执行文件 AircraftPoseSystem 已生成（链接 ${_aps_link_targets}）")
