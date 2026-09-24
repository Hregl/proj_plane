# ============================================================================
#  cmake/Dependencies.cmake
#
#  依据：ENG-03 §7（第三方依赖统一管理）、§8（Qt）、§9（OpenCV）、
#        §10（ImvSDK）、§11（Turntable SDK 预留）、§15（测试工程）
#
#  职责：全工程**唯一**的第三方依赖查找点。模块 CMakeLists.txt 不得自行
#        find_package，只能链接本文件导出的 aps::* 目标。
#
#  为什么必须集中：
#   ENG-03 §2.2 要求依赖单向。若 data/ 与 algorithm/ 各自 find_package(OpenCV)，
#   两处可能拿到不同版本/不同编译选项的 OpenCV，导致同一份 Transform 类型在
#   两个模块中 ABI 不一致——这类问题在链接期不报错、运行期随机崩溃。
#   集中一次查找 + INTERFACE 目标传递，是唯一的根治办法。
# ============================================================================

# ----------------------------------------------------------------------------
# 0 ⚠ 登记本工程的 Find 模块目录（**2026-09-23 修复一处长期失效**）
#
# `find_package(<Pkg>)` 的 MODULE 模式只查两处：`CMAKE_MODULE_PATH` 与
# CMake 自带的 `<prefix>/share/cmake-*/Modules/`。**不含**调用者所在目录。
#
# 本工程原先**从未设置** `CMAKE_MODULE_PATH`（实测缓存中该变量不存在，
# `--debug-find-pkg=ImvSdk` 显示只搜索了
# `/usr/share/cmake-3.31/Modules/FindImvSdk.cmake`）。后果：
# `cmake/FindImvSdk.cmake` 与 `cmake/FindTurntableSdk.cmake`
# **从写下来那天起就没有被加载过一次**，`-DIMV_SDK_ROOT=<真实路径>` 也不会生效。
#
# ⚠ 这个失效之所以长期没被发现，是因为它的**表现与预期状态完全一致**：
#   打印的是"未找到 ImvSdk（非致命）… 拿到 SDK 后：cmake .. -DIMV_SDK_ROOT=…"，
#   而"Sprint 2 之前本来就没装 SDK"正是当时的真实情况 —— 失败信息看不出
#   是"没装"还是"查找器压根没跑"。这正是本项目反复出现的
#   **"断言没验它自称要验的东西"** 同族缺陷。
#
# 放在本文件（而不是顶层 CMakeLists）的原因：`find_package` 的调用点在这里，
# 契约应当随调用点走 —— 任何将来 include 本文件的地方都自动获得正确行为。
# ----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)

# ----------------------------------------------------------------------------
# 1 OpenCV（ENG-03 §9：图像处理 / SIFT / PnP / 坐标计算）
#
# REQUIRED：ENG-03 §9 明确要求，且 OpenCV 是 data（cv::Mat / cv::Vec3d）
# 的硬依赖，缺失则整个工程无法编译，早失败优于晚失败。
# ----------------------------------------------------------------------------
find_package(OpenCV REQUIRED)

add_library(aps_opencv INTERFACE)
add_library(aps::opencv ALIAS aps_opencv)

target_include_directories(aps_opencv SYSTEM INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aps_opencv INTERFACE ${OpenCV_LIBS})

# 部分发行版的 OpenCVConfig 不把库目录并进 OpenCV_LIBS 的解析路径，
# 此处补上；为空时不产生任何效果。
if(OpenCV_LIBRARY_DIRS)
    target_link_directories(aps_opencv INTERFACE ${OpenCV_LIBRARY_DIRS})
endif()

message(STATUS "[APS] 找到 OpenCV ${OpenCV_VERSION}：${OpenCV_INCLUDE_DIRS}")

# ----------------------------------------------------------------------------
# 2 Qt5（ENG-03 §8：MainWindow / ImageViewer / Panel）
#
# REQUIRED：ENG-08 §16 的验收标准含"Qt 启动"，且 ui 与 app 是最后两层，
# 缺失会让 Phase 8 才发现问题，故此处即失败。
#
# 只查 Widgets：ENG-01 §13 的 ui 模块只有 MainWindow/widgets/ImageViewer/
# PosePanel/StatusPanel/TurntablePanel，均为 Widgets 组件。Core/Gui 作为
# Widgets 的传递依赖自动带入。将来若用到 QtNetwork（转台网络控制）或
# QtConcurrent，在 ENG-03 §8 补充后再在此追加组件。
# ----------------------------------------------------------------------------
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_library(aps_qt5 INTERFACE)
add_library(aps::qt5 ALIAS aps_qt5)
target_link_libraries(aps_qt5 INTERFACE Qt5::Widgets)

message(STATUS "[APS] 找到 Qt5 Widgets ${Qt5Widgets_VERSION}")

# ----------------------------------------------------------------------------
# 3 GoogleTest（ENG-03 §15：推荐 GoogleTest；ENG-06 §6 测试工程）
#
# **刻意用 QUIET 而非 REQUIRED。**
#
# 原因：ENG-08 §5 的 Sprint 1 验收是"cmake / make 通过"，而 GoogleTest 在
# UOS/Debian 上不随开发机预装，且本项目**离线部署**——不能用 FetchContent
# 联网拉取。若写成 REQUIRED，第一次 clone 后 cmake 会直接失败，把"依赖没装"
# 伪装成"工程配置错误"，排查方向完全跑偏。
#
# 处理策略：
#   找到  → APS_HAVE_GTEST=TRUE，tests/ 正常生成测试目标；
#   找不到 → APS_HAVE_GTEST=FALSE，打印显式警告 + 安装命令，tests/ 短路。
#
# 警告足够醒目，不会让"测试框架"这个交付项静默消失（1.md 第一阶段范围
# 明确列有 ✅ GoogleTest框架）。
# ----------------------------------------------------------------------------
set(APS_HAVE_GTEST FALSE)

find_package(GTest QUIET)

if(GTest_FOUND)
    set(APS_HAVE_GTEST TRUE)

    add_library(aps_gtest INTERFACE)
    add_library(aps::gtest ALIAS aps_gtest)

    # GoogleTest 是第三方头，标 SYSTEM 以隔离其自身警告
    target_include_directories(aps_gtest SYSTEM INTERFACE
        ${GTEST_INCLUDE_DIRS})

    # 优先用上游导出的 imported 目标（Debian 的 libgtest-dev / googletest
    # 均提供 GTest::gtest 与 GTest::gtest_main）；老式 FIND_PACKAGE 只给
    # 变量，故保留变量分支作为回退。
    if(TARGET GTest::gtest_main)
        target_link_libraries(aps_gtest INTERFACE GTest::gtest_main)
    elseif(TARGET GTest::GTest)
        target_link_libraries(aps_gtest INTERFACE GTest::GTest)
    else()
        target_link_libraries(aps_gtest INTERFACE ${GTEST_LIBRARIES})
        if(GTEST_MAIN_LIBRARIES)
            target_link_libraries(aps_gtest INTERFACE ${GTEST_MAIN_LIBRARIES})
        endif()
    endif()

    # gtest_main 已提供 main()；显式链接 pthread 以支持 GTest 的线程断言
    find_package(Threads QUIET)
    if(Threads_FOUND)
        target_link_libraries(aps_gtest INTERFACE Threads::Threads)
    endif()

    message(STATUS "[APS] 找到 GoogleTest，测试目标将正常生成")
else()
    message(WARNING
        "[APS] 未找到 GoogleTest：测试目标将被跳过，BUILD_TESTING=ON 实际不生效。\n"
        "      ENG-08 §5 Sprint 1 的交付项含\"测试框架\"，1.md 第一阶段范围亦列出\n"
        "      ✅ GoogleTest框架，因此请先安装后再配置：\n"
        "          sudo apt install libgtest-dev        # UOS / Debian / Ubuntu\n"
        "          sudo apt install googletest          # 若上者不存在\n"
        "      安装后无需改动任何 CMake 文件，重新执行 cmake .. 即可自动启用。\n"
        "      （本项目为离线部署，不采用 FetchContent 联网拉取。）")
endif()

# ----------------------------------------------------------------------------
# 4 ImvSdk（ENG-03 §10：include / library / runtime 路径）
#
# 华睿 A7A20MU201 相机 SDK。ENG-03 §10 的示例写的是 find_package(ImvSdk
# REQUIRED)，但照抄会让**尚未拿到 SDK 的任何一台机器**无法配置工程。
# ENG-08 §3 明确第一阶段"一台真实相机 + 虚拟设备补全"，即软件闭环可以在
# 无 SDK 时先行完成——这与 REQUIRED 直接矛盾，矛盾处按 ENG-08 §3 的
# 开发策略取"可选"。
#
# 需要强制时用 -DAPS_REQUIRE_DEVICE_SDK=ON（见 BuildOptions.cmake）。
#
# 查找细节在 FindImvSdk.cmake（模块化，遵循 CMake 的 Find<Package>.cmake
# 约定，使 IDE 与 cmake --help-module 可用）。
# ----------------------------------------------------------------------------
set(APS_HAVE_IMVSDK FALSE)

find_package(ImvSdk QUIET)

if(ImvSdk_FOUND)
    set(APS_HAVE_IMVSDK TRUE)

    add_library(aps_imvsdk INTERFACE)
    add_library(aps::imvsdk ALIAS aps_imvsdk)

    target_include_directories(aps_imvsdk SYSTEM INTERFACE ${ImvSdk_INCLUDE_DIRS})
    target_link_libraries(aps_imvsdk INTERFACE ${ImvSdk_LIBRARIES})

    # ------------------------------------------------------------------------
    # ⚠ 构建树的运行期搜索路径（2026-09-23 实测新增，011-A0）
    #
    # 只给 `-L<runtime> -lMVSDK`，二进制**跑不起来**。实测（011-A0.1 §6.2）：
    #   默认 `-Wl,-rpath` 在现代 binutils 下生成的是 **RUNPATH**，而 RUNPATH
    #   **不传递** —— 它只覆盖可执行文件**直接**链接的库。于是可执行文件能找到
    #   libMVSDK.so，但 libMVSDK.so 找不到自己的依赖：
    #       libGCBase_gcc421_v3_0.so: cannot open shared object file（退出码 127）
    #   这属于本项目最警惕的失效形态：**配置能过、构建通过、运行才失败**。
    #
    # 两条修法（实测均通过，二者取其一即可）：
    #   · LD_LIBRARY_PATH=<runtime>            —— 需要每个运行入口都记得设；
    #   · `--disable-new-dtags` 生成 DT_RPATH  —— **可传递**，一次链接长期有效。
    # 此处取后者，让构建树里的任何可执行文件（含测试）**开箱即跑**，
    # 不依赖调用者记得设环境变量 —— 依赖人记住的东西迟早会被忘掉。
    #
    # 用 $<BUILD_INTERFACE:...> 限定：安装后的 deploy/ 不继承本路径。
    # 这既符合 CompilerOptions.cmake §4 `CMAKE_SKIP_INSTALL_RPATH ON`
    # 的既定策略（deploy/bin 不得指向开发机构建目录），也把"安装后如何找
    # deploy/lib"这个独立问题留给 011-B（见核验报告 §7.3，当前无机制）。
    # ------------------------------------------------------------------------
    if(ImvSdk_RUNTIME_DIRS)
        target_link_options(aps_imvsdk INTERFACE
            $<BUILD_INTERFACE:-Wl,--disable-new-dtags>
            $<BUILD_INTERFACE:-Wl,-rpath,${ImvSdk_RUNTIME_DIRS}>)
    endif()

    # ENG-03 §10 要求同时管理 "runtime 路径"：SDK 的动态库需随部署包
    # 一起下发到 deploy/lib，否则现场无法启动。
    if(ImvSdk_RUNTIME_DIRS)
        set(APS_IMVSDK_RUNTIME_DIRS "${ImvSdk_RUNTIME_DIRS}"
            CACHE INTERNAL "ImvSdk 运行期动态库目录，由 InstallRules 使用")
    endif()

    message(STATUS "[APS] 找到 ImvSdk：${ImvSdk_LIBRARIES}")
elseif(APS_REQUIRE_DEVICE_SDK)
    message(FATAL_ERROR
        "[APS] APS_REQUIRE_DEVICE_SDK=ON 但未找到 ImvSdk。\n"
        "      请指定 SDK 根目录：cmake .. -DIMV_SDK_ROOT=/path/to/ImvSdk")
else()
    message(STATUS
        "[APS] 未找到 ImvSdk（非致命）：真实相机后端将在 Phase 2/Sprint 2 接入。\n"
        "      当前使用 VirtualCameraBackend 完成软件闭环（ENG-08 §3）。\n"
        "      拿到 SDK 后：cmake .. -DIMV_SDK_ROOT=/path/to/ImvSdk")
endif()

# ----------------------------------------------------------------------------
# 5 TurntableSdk（ENG-03 §11：Peko SDK / 网络协议库 / RS485 —— 可选依赖）
#
# ENG-08 §11 明确第一阶段"不实现 Peko协议 / RS485 / 网络控制"，因此此项
# 在第一阶段**预期就是找不到的**，不产生警告，只报 STATUS。
# ----------------------------------------------------------------------------
set(APS_HAVE_TURNTABLESDK FALSE)

find_package(TurntableSdk QUIET)

if(TurntableSdk_FOUND)
    set(APS_HAVE_TURNTABLESDK TRUE)

    add_library(aps_turntablesdk INTERFACE)
    add_library(aps::turntablesdk ALIAS aps_turntablesdk)

    target_include_directories(aps_turntablesdk SYSTEM INTERFACE ${TurntableSdk_INCLUDE_DIRS})
    target_link_libraries(aps_turntablesdk INTERFACE ${TurntableSdk_LIBRARIES})

    message(STATUS "[APS] 找到 TurntableSdk：${TurntableSdk_LIBRARIES}")
elseif(APS_REQUIRE_DEVICE_SDK)
    message(FATAL_ERROR
        "[APS] APS_REQUIRE_DEVICE_SDK=ON 但未找到 TurntableSdk。\n"
        "      请指定：cmake .. -DTURNTABLE_SDK_ROOT=/path/to/PekoSdk")
else()
    message(STATUS
        "[APS] 未找到 TurntableSdk（预期行为）：第一阶段使用 VirtualTurntable\n"
        "      （ENG-08 §11），Peko_D 协议在 Phase 2 接入。")
endif()
