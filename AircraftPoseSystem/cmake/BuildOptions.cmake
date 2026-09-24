# ============================================================================
#  cmake/BuildOptions.cmake
#
#  依据：ENG-03 §6（编译选项）、§16（Debug/Release）、§19（版本基线）
#
#  职责：定义全部编译期开关与全局输出路径。**只定义变量，不查找依赖、
#        不生成目标**——依赖在 Dependencies.cmake，目标在 src/ 各模块。
#
#  本文件必须最先被 include（见根 CMakeLists.txt 的加载顺序说明）。
# ============================================================================

# ----------------------------------------------------------------------------
# 1 编译开关（ENG-03 §6，四个 option 的名称与默认值均已冻结）
# ----------------------------------------------------------------------------

# 测试开关：ENG-03 §6 冻结默认 ON。ENG-08 §5 Sprint 1 要求"测试框架"随
# 工程骨架一起交付，默认关闭会让验收项悄悄消失，故保持 ON。
option(
    BUILD_TESTING
    "Build tests"
    ON
)

# GPU 加速：冻结默认 OFF。V2.1 第一阶段全程虚拟设备 + CPU 算法链，
# 无 CUDA 依赖；离线部署目标机也未确认有 GPU。
option(
    ENABLE_GPU
    "Enable GPU acceleration"
    OFF
)

# 内存检查：冻结默认 OFF。Debug/测试流程按 ENG-03 §18 用 -DENABLE_ASAN=ON 显式开启。
option(
    ENABLE_ASAN
    "Enable Address Sanitizer"
    OFF
)

# 线程检查：冻结默认 OFF。SYS-09 的双流水线（Preview / Measurement）是
# 数据竞争高发区，集成测试阶段用 -DENABLE_TSAN=ON 显式开启。
option(
    ENABLE_TSAN
    "Enable Thread Sanitizer"
    OFF
)

# 警告视为错误：ENG-03 未规定，默认关闭。留作 CI/发布前的收紧手段，
# 不给日常开发制造阻塞。
option(
    ENABLE_WERROR
    "Treat compiler warnings as errors"
    OFF
)

# ImvSDK / 转台 SDK 是否可用时"必须"构建真实设备后端。
# 默认 OFF：SDK 缺失时跳过真实后端而非中断整个工程（ENG-03 §11 明确
# 转台 SDK 为可选依赖；ENG-08 §3 第一阶段只有一台真实相机）。
# 置 ON 后若 SDK 未找到则直接 FATAL_ERROR，用于发布构建。
option(
    APS_REQUIRE_DEVICE_SDK
    "Fail configuration when ImvSdk / TurntableSdk is missing"
    OFF
)

# ----------------------------------------------------------------------------
# 2 编译类型默认值（ENG-03 §16）
#
# 单配置生成器（Unix Makefiles）下 CMAKE_BUILD_TYPE 默认为空，会退化成
# 无优化且无调试符号的尴尬状态。此处兜底为 Debug：ENG-08 §5 的 Sprint 1
# 验收是"程序可在 UOS 编译运行"，调试友好优先于性能。
# ----------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
        "Build type: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
    message(STATUS "[APS] CMAKE_BUILD_TYPE 未指定，默认取 Debug")
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)

# ----------------------------------------------------------------------------
# 3 输出路径
#
# 约定：
#   <build>/          → 可执行文件直接落在构建根目录
#   <build>/lib/      → 全部模块静态库
#
# 可执行文件放构建根目录是**刻意**的：1.md《第一版代码里程碑》与
# ENG-08 §5 的验收命令均为 `cd build && ./AircraftPoseSystem`。
# 若改到 bin/ 子目录，文档里的验收步骤会失败，后续每个阶段的验证都要
# 跟着改写，代价高于收益。
# ----------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# 多配置生成器（Ninja Multi-Config 等）下需要逐配置设置，否则上面三项被忽略
foreach(_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${_cfg}" _CFG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_CFG} "${CMAKE_BINARY_DIR}/lib")
endforeach()

# ----------------------------------------------------------------------------
# 4 版本基线（ENG-03 §19）
#
#   v2.1-framework → 工程骨架     （本阶段，1.md 第一阶段）
#   v2.1-device    → 硬件抽象
#   v2.1-algorithm → 算法链
#   v2.1-release   → 交付版本
#
# 编译期注入为宏，供 UI 标题栏与日志记录基线（ENG-08 §16 要求 Qt 启动可见）。
# ----------------------------------------------------------------------------
set(APS_VERSION_STAGE "v2.1-framework"
    CACHE STRING "版本基线阶段：v2.1-framework | v2.1-device | v2.1-algorithm | v2.1-release")

set_property(CACHE APS_VERSION_STAGE PROPERTY STRINGS
    v2.1-framework v2.1-device v2.1-algorithm v2.1-release)

add_compile_definitions(
    APS_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    APS_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    APS_VERSION_STRING="${PROJECT_VERSION}"
    APS_VERSION_STAGE="${APS_VERSION_STAGE}"
)

# ----------------------------------------------------------------------------
# 5 开关合法性校验
#
# ASAN 与 TSAN 由同一编译/链接参数位切换，同时开启会产生互相冲突的运行期
# 并且是**静默**的错误诊断。此处直接阻断配置，不留到运行期排查。
# ----------------------------------------------------------------------------
if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR
        "[APS] ENABLE_ASAN 与 ENABLE_TSAN 不能同时开启：\n"
        "      两者互斥，且同时注入会导致链接期/运行期行为未定义。\n"
        "      请分别构建：-DENABLE_ASAN=ON 或 -DENABLE_TSAN=ON。")
endif()

if((ENABLE_ASAN OR ENABLE_TSAN) AND CMAKE_BUILD_TYPE STREQUAL "Release")
    # 不算错误——Release+ASAN 是合法的性能/内存联合排查手段——但必须让
    # 使用者意识到优化会改变变量生命周期，ASAN 报告的解释难度显著上升。
    message(WARNING
        "[APS] 在 Release 构建上开启了 Sanitizer：优化会改变变量生命周期，\n"
        "      误报/漏报的解释难度上升。建议用 RelWithDebInfo 或 Debug。")
endif()

if(ENABLE_GPU)
    # ENG-03 §6 只定义了开关，未定义后端。V2.1 第一阶段无 GPU 算法实现，
    # 若此处不报错，使用者会以为加速已生效。
    message(WARNING
        "[APS] ENABLE_GPU=ON，但 V2.1 第一阶段尚未实现任何 GPU 计算后端\n"
        "      （src/runtime/GpuRuntime 为空）。该开关当前不产生实际加速效果。")
endif()
