# ============================================================================
#  cmake/CompilerOptions.cmake
#
#  依据：ENG-03 §5（cmake/ 文件清单）、§16（Debug/Release）、§2.1（模块独立编译）
#
#  职责：把 §16 的 Debug/Release 要求与 BuildOptions 的开关，翻译成具体的
#        编译/链接参数。
#
#  实现方式：在根目录作用域调用 add_compile_options() / add_link_options()，
#  它们对**当前目录及其全部子目录**生效，因此 src/ 与 tests/ 下的所有目标
#  自动继承，无需逐模块 target_compile_options（ENG-03 §2.1 要求各模块
#  独立编译，参数一致才谈得上"独立"）。
# ============================================================================

# ----------------------------------------------------------------------------
# 1 基础警告集
#
# 取 -Wall -Wextra -Wpedantic 三档：
#   -Wall    捕获未初始化、隐式转换等常规缺陷；
#   -Wextra  捕获未使用参数、符号比较等；
#   -Wpedantic 阻止 GNU 方言渗入——本项目要求 CMAKE_CXX_EXTENSIONS=OFF，
#              编译器若不警告就无从发现越界用法。
#
# 不加 -Werror：见 BuildOptions 的 ENABLE_WERROR 开关。开发期警告应当是
# 提示而非阻塞；发布前再用 -DENABLE_WERROR=ON 收紧。
# ----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
    )

    if(ENABLE_WERROR)
        add_compile_options(-Werror)
        message(STATUS "[APS] 警告视为错误：已开启（ENABLE_WERROR=ON）")
    endif()
endif()

# ----------------------------------------------------------------------------
# 2 Debug / Release 参数（ENG-03 §16）
#
# §16 只写"Debug 符号 + 低优化"和"编译优化 + 安装打包"，此处落到具体参数：
#   Debug   -g -O0                    → 断点、单步、变量观察全部可用
#   Release -O2 -DNDEBUG              → 与 NDEBUG 对齐关闭 assert
#
# 刻意不用 -O3：本项目误差预算余量仅 3%（SYS-15 §4），逐位可复现比峰值
# 性能重要。PnP/特征提取的浮点结果在 -O3 下可能因向量化而改变末位，
# 使 Golden 数据比对（ENG-06 §10）出现难解释的差异。
# ----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0"
        CACHE STRING "Debug 编译参数" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG"
        CACHE STRING "Release 编译参数" FORCE)
endif()

# ----------------------------------------------------------------------------
# 3 Sanitizer（ENG-03 §18）
#
# ASAN 与 TSAN 的互斥校验已在 BuildOptions.cmake §5 完成，此处只注入参数。
#
# 注意 ASAN 需要 -fno-omit-frame-pointer 才能给出有意义的调用栈，
# 且必须 compile/link 成对出现（故同时写 add_compile_options 与 add_link_options）。
# ----------------------------------------------------------------------------
if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
    message(STATUS "[APS] AddressSanitizer 已启用")
endif()

if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
    message(STATUS "[APS] ThreadSanitizer 已启用")
endif()

# ----------------------------------------------------------------------------
# 4 离线部署相关
#
# 目标机为 UOS x86_64，与开发机同架构，不做交叉编译，因此不需要
# sysroot / toolchain file。此处只关闭一个隐患：
# RPATH 在安装时必须清空，否则 deploy/bin 里的可执行文件会指向开发机的
# 构建目录，拷贝到现场后无法启动（ENG-03 §20 要求"支持离线部署"）。
# ----------------------------------------------------------------------------
set(CMAKE_SKIP_INSTALL_RPATH ON)

# ----------------------------------------------------------------------------
# 5 未纳入本次处理的事项（留待对应阶段，此处显式登记避免遗忘）
#
#   · Qt 的 moc 生成文件在 -Wextra 下可能报 unused-parameter：
#     若 Phase 8（Qt 主窗口）出现该噪声，在 src/ui 与 src/app 的
#     CMakeLists.txt 中对该目标单独 target_compile_options(-Wno-unused-parameter)，
#     不要在根目录全局关闭——它会掩盖算法模块里真正的未使用参数缺陷。
#
#   · OpenCV 头文件在 -Wextra 下的第三方警告：
#     同理，若出现则在链接 OpenCV 的模块上把 OpenCV 的 include 目录标为
#     SYSTEM（target_include_directories(... SYSTEM ...)），不全局降级警告。
# ----------------------------------------------------------------------------
