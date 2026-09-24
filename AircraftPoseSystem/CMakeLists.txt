# ============================================================================
#  AircraftPoseSystem V2.1（多相机闭环测量版）—— 根 CMakeLists.txt
#
#  依据：ENG-03 §4（根目录 CMake）
#       ENG-01 §2（工程总体目录）
#       ENG-08 §5（Sprint 1：UOS 工程环境初始化）
#
#  本文件是整个代码工程的入口，只负责三件事：
#    1. 项目定义与语言标准；
#    2. 加载 cmake/ 下的工程配置；
#    3. 按依赖顺序加载子目录。
#
#  验收（ENG-03 §20 / ENG-08 §5）：
#      mkdir build && cd build && cmake .. && make
# ============================================================================

cmake_minimum_required(VERSION 3.16)

project(
    AircraftPoseSystem
    VERSION 2.1
    LANGUAGES CXX
)

# ----------------------------------------------------------------------------
# 语言标准（ENG-03 §4）
#
# 冻结为 C++17。CMAKE_CXX_STANDARD_REQUIRED 防止编译器在标准不支持时静默
# 回退（Qt5.15 / OpenCV4.6 / UOS gcc 12.3 均完整支持 C++17）。
# CMAKE_CXX_EXTENSIONS=OFF 关闭 gnu++17 扩展，保证跨编译器一致性——
# 本项目最终要离线部署到 UOS x86_64，不用 GNU 方言换取可移植性。
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 导出 compile_commands.json，供 clangd / IDE 索引（不参与构建产物）
#
# 必须写成 CACHE 变量并 FORCE，不能写 set(CMAKE_EXPORT_COMPILE_COMMANDS ON)：
# 该变量是 CMake 在 project() 阶段预先建立的 **cache** 项（默认空值、带
# ADVANCED 标记），而 Makefile/Ninja 生成器读的是 cache 而非普通变量，
# 因此普通 set() 会被静默忽略——表现为"设置了但 compile_commands.json
# 始终不生成"，且没有任何警告。（已在 CMake 3.31 + Unix Makefiles 复现。）
#
# 仅在值为空（即用户未显式指定）时改为 ON，保留 -D...=OFF 的否决权。
if(CMAKE_EXPORT_COMPILE_COMMANDS STREQUAL "")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL
        "导出 compile_commands.json 供 clangd / IDE 索引" FORCE)
endif()

# ----------------------------------------------------------------------------
# 工程配置（ENG-03 §5 规定的 cmake/ 文件全表）
#
# 加载顺序有依赖，不可调换：
#   BuildOptions  → 定义 option()，CompilerOptions 与 InstallRules 都要读它；
#   CompilerOptions → 依赖 BuildOptions 的开关决定编译/链接参数；
#   Dependencies  → 依赖 BuildOptions 的 ENABLE_GPU 决定是否查找 CUDA；
#   InstallRules  → 依赖 BuildOptions 的 CMAKE_BUILD_TYPE 决定安装断言。
# ----------------------------------------------------------------------------
include(cmake/BuildOptions.cmake)

# enable_testing() 必须在**顶层目录**调用，否则 ctest 在构建根目录找不到
# 测试清单（CMake 只在调用了 enable_testing() 的目录生成 CTestTestfile.cmake）。
# 位置必须在 InstallRules.cmake 之前——后者用 add_test() 注册"安装自检"，
# 而 add_test() 要求测试机制已启用。
if(BUILD_TESTING)
    enable_testing()
endif()

include(cmake/CompilerOptions.cmake)
include(cmake/Dependencies.cmake)
include(cmake/InstallRules.cmake)

# ----------------------------------------------------------------------------
# 子目录加载（ENG-03 §4）
#
# src/  → 10 个模块库 + 1 个可执行文件
# tests/ → GoogleTest 单元/集成测试（内部按 BUILD_TESTING 自行短路）
# ----------------------------------------------------------------------------
add_subdirectory(src)
add_subdirectory(tests)

# ----------------------------------------------------------------------------
# 配置期摘要
#
# 集中打印一次，避免开发者翻 build/CMakeCache.txt 才知道依赖有没有找到。
# 缺失项一律在此显式暴露，不允许静默降级。
# ----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "==================== AircraftPoseSystem V${PROJECT_VERSION} 配置摘要 ====================")
message(STATUS "  构建类型        : ${CMAKE_BUILD_TYPE}")
message(STATUS "  编译器          : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  C++ 标准        : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  构建测试        : ${BUILD_TESTING}")
message(STATUS "  GPU 加速        : ${ENABLE_GPU}")
message(STATUS "  AddressSanitizer: ${ENABLE_ASAN}")
message(STATUS "  ThreadSanitizer : ${ENABLE_TSAN}")
message(STATUS "  Qt5 Widgets     : ${Qt5Widgets_VERSION}")
message(STATUS "  OpenCV          : ${OpenCV_VERSION}")
message(STATUS "  GoogleTest      : ${APS_HAVE_GTEST}")
message(STATUS "  ImvSdk          : ${APS_HAVE_IMVSDK}")
message(STATUS "  TurntableSdk    : ${APS_HAVE_TURNTABLESDK}")
message(STATUS "  安装前缀        : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  可执行文件输出  : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "===================================================================================")
message(STATUS "")
