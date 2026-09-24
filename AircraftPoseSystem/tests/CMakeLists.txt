# ============================================================================
#  tests/CMakeLists.txt
#
#  依据：ENG-03 §15（测试工程：目录清单 / enable_testing / 推荐 GoogleTest）
#        ENG-01 §15（tests 目录，含 preview / hardware / golden）
#        ENG-06（测试工程设计）、ENG-08 §13（第一阶段测试任务）
#        1.md Phase 9（tests/ 生成）
#
#  目录（ENG-03 §15 与 ENG-01 §15 已对齐）：
#      unit  optical  device  turntable  trigger  preview
#      algorithm  integration  hardware  stability  golden
#
#  测试分层与归属（ENG-06 / ENG-08 §13）：
#      unit         Data / StateMachine / Queue 等纯逻辑单测
#      optical      OpticalRig / CoordinateTransformer / 标定
#      device       ImvCameraBackend / VirtualCameraBackend
#      turntable    VirtualTurntable / 对准收敛
#      trigger      触发时序与同步校验
#      preview      预览显示与通道切换（ENG-06 §6 明确要求）
#      algorithm    Golden 数据驱动的算法链测试
#      integration  Application 流程闭环 + SYS-08 §10 的 7 个收敛性用例
#      hardware     需要真实设备的测试（CI 中标记为不可用）
#      stability    长时间运行 / 内存增长 / 队列积压
#      golden       测试数据资产目录（**非测试代码**，无 CMakeLists.txt）
# ============================================================================

# ----------------------------------------------------------------------------
# 1 短路条件
#
# 两层短路，均打印明确原因，不允许静默失效：
#
#   BUILD_TESTING=OFF  → 使用者主动关闭（ENG-03 §6 定义的开关）
#   GoogleTest 缺失    → 依赖未安装（Dependencies.cmake 已打印安装指引）
#
# 第二层之所以必要：若在此直接 find_package(GTest REQUIRED)，工程会因
# "测试依赖没装"而整体无法配置，连不带测试的构建都做不了。而这恰恰是
# 本机当前的状态（UOS 未预装 GoogleTest）。
# ----------------------------------------------------------------------------
if(NOT BUILD_TESTING)
    message(STATUS "[APS] tests/：BUILD_TESTING=OFF，跳过全部测试目标")
    return()
endif()

if(NOT APS_HAVE_GTEST)
    message(STATUS
        "[APS] tests/：未找到 GoogleTest，跳过全部测试目标。\n"
        "      安装方法见上方 Dependencies.cmake 的警告；\n"
        "      安装后重新执行 cmake .. 即自动启用，无需改动任何文件。")
    return()
endif()

include(GoogleTest)

# ----------------------------------------------------------------------------
# 2 测试装配函数
#
# 用法：
#     aps_add_test(<测试名>
#         SOURCES <a.cpp> <b.cpp>
#         DEPENDS <模块目标> ...
#     )
#
# 约定：
#   · 测试名与被测模块同名缀（如 test_data / test_state_machine）；
#   · DEPENDS 只写直接依赖的模块目标，不写 aps::gtest（已内置）；
#   · 源文件必须显式列出，**不用 GLOB**。
#
# 为什么测试源文件不用 GLOB（与本工程其它 CMakeLists 的做法相反）：
#   模块库用 GLOB 是为了让"逐阶段生成文件"的过程免于反复改 CMake；
#   而测试文件被误删或改名时，GLOB 会让它**静默消失**——测试数量从
#   37 变成 36 不会有人发现，这正是 ENG-06 最不能接受的失效模式。
#   显式列举换来的是：文件缺失时配置期直接报错。
# ----------------------------------------------------------------------------
function(aps_add_test name)
    cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDS" ${ARGN})

    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "[APS] aps_add_test(${name})：未指定 SOURCES")
    endif()

    add_executable(${name} ${ARG_SOURCES})

    set_target_properties(${name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_include_directories(${name} PRIVATE
        "${PROJECT_SOURCE_DIR}/src")

    # 依赖过滤：模块可能尚未生成（逐阶段开发），同模块库的处理方式
    set(_deps "")
    foreach(_dep IN LISTS ARG_DEPENDS)
        if(TARGET ${_dep})
            list(APPEND _deps ${_dep})
        else()
            message(STATUS
                "[APS] 测试 ${name}：依赖 ${_dep} 尚未生成，暂不链接")
        endif()
    endforeach()

    target_link_libraries(${name} PRIVATE
        ${_deps}
        aps::gtest
    )

    # gtest_discover_tests：从测试可执行文件自动枚举 TEST() 用例，
    # 免去逐条 add_test（ENG-06 的用例数会持续增长，手工登记必然遗漏）。
    #
    # DISCOVERY_MODE PRE_TEST 把枚举从**构建期**移到 **ctest 期**：
    #   构建期枚举需要在 make 过程中执行刚生成的二进制，在
    #   · 目标机架构不同、（本项目虽同架构，但 hardware 组测试将来可能
    #     在另一台机器上跑）
    #   · 测试需要显示环境或真实设备
    #   这两种情形下会让"编译通过"这一步莫名失败。
    # 该选项 3.18 才引入，故加版本判断以兼容 min 3.16。
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
        gtest_discover_tests(${name} DISCOVERY_MODE PRE_TEST)
    else()
        gtest_discover_tests(${name})
    endif()
endfunction()

# ----------------------------------------------------------------------------
# 3 自动发现测试子目录
#
# 只加载**含有 CMakeLists.txt** 的子目录。tests/golden/ 保存的是数据资产
# （ENG-06 §10），没有构建脚本，会被自然跳过——不需要在此写排除名单。
#
# 这种"目录 + 自带 CMakeLists"的方式，使 1.md 的逐阶段生成无需回头修改
# 本文件：Phase 9 生成哪个目录，哪个目录就自动生效。
# ----------------------------------------------------------------------------
set(APS_TEST_SUBDIRS_LOADED "")

file(GLOB _entries RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
     "${CMAKE_CURRENT_SOURCE_DIR}/*")

foreach(_entry IN LISTS _entries)
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${_entry}"
       AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_entry}/CMakeLists.txt")
        add_subdirectory("${_entry}")
        list(APPEND APS_TEST_SUBDIRS_LOADED ${_entry})
    endif()
endforeach()

if(APS_TEST_SUBDIRS_LOADED)
    list(LENGTH APS_TEST_SUBDIRS_LOADED _n)
    message(STATUS "[APS] 测试：已加载 ${_n} 个测试子目录（${APS_TEST_SUBDIRS_LOADED}）")
else()
    message(STATUS
        "[APS] 测试：GoogleTest 可用，但尚无测试子目录。\n"
        "      1.md Phase 9 生成 tests/ 下的 CMakeLists.txt 与 test_*.cpp 后\n"
        "      自动加载；当前 ctest 只包含 InstallRules.cmake 注册的安装自检。")
endif()
