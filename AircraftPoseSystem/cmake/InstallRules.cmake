# ============================================================================
#  cmake/InstallRules.cmake
#
#  依据：ENG-03 §17（install 部署：deploy/{bin,lib,config,models,calibration}）
#        ENG-01 §16（deploy 目录）
#        ENG-03 §20 第 6 条（支持离线部署）
#
#  职责：定义安装布局，使 `cmake --install build` 产出一个**可直接拷到现场**
#        的目录树。
#
#  分工（重要，2026-09-24 按 R02 修正为**如实**的三条）：
#
#    本文件                → 安装前缀、目录结构、纯数据资产
#                            （config / models / calibration）、SDK 运行期库，
#                            以及 §6 的两条安装自检测试。
#
#    src/CMakeLists.txt    → **10 个模块库**的 install(TARGETS ...)，写在
#      的 aps_add_module_library()  那个函数体里。之所以不写在本文件、也不各自
#                            写在各模块的 CMakeLists.txt 里：该函数是**唯一
#                            知道 target 是否存在**的地方（它对空模块提前
#                            return()）。写在各模块文件里就要各自再判一次
#                            if(TARGET ...)，漏一个就是配置期直接报
#                            "install TARGETS given target ... does not exist"
#                            —— 把"这个模块还没写"变成"整个工程配置不过"。
#
#    src/app/CMakeLists.txt → **可执行文件**的 install(TARGETS ...)，以及紧随
#                            其后的 install(CODE ...) 安装断言（见该文件 §5）。
#
#  ⚠ 修正说明：本段原文写"各模块 → 各自的 install(TARGETS ...)"，描述的是一个
#    **从未实现的约定** —— R02 审查发现全工程 install(TARGETS ...) 实体 0 处，
#    于是 deploy/ 只有空的 config/models/calibration。约定写了不等于做了，
#    故本段现在只描述**代码里实际存在**的分工。
# ============================================================================

include(GNUInstallDirs)

# ----------------------------------------------------------------------------
# 1 安装前缀
#
# 默认安装到工程内的 deploy/，而不是 /usr/local。
#
# 理由：本系统是**离线交付**的整套设备软件（ENG-03 §20 第 6 条），交付物是
# 一个自包含的目录包，现场解压即用。安装到 /usr/local 会把可执行文件、模型、
# 标定数据散落到系统各处的只读目录，既不便打包，也会让"五元组基线"中的
# 标定版本（ENG-09 §6.8）难以与具体交付物对应。
#
# 仍可用 -DCMAKE_INSTALL_PREFIX=/opt/AircraftPoseSystem 覆盖。
# CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT 保证只覆盖"用户没显式指定"的情形。
# ----------------------------------------------------------------------------
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/deploy"
        CACHE PATH "安装前缀" FORCE)
    message(STATUS "[APS] 安装前缀未指定，默认取工程内 deploy/（离线交付包）")
endif()

# ----------------------------------------------------------------------------
# 2 目录结构（ENG-03 §17）
#
#   deploy/
#   ├── bin          ← 可执行文件        （CMAKE_INSTALL_BINDIR  = bin）
#   ├── lib          ← 模块库 + 第三方 SDK 动态库（CMAKE_INSTALL_LIBDIR = lib）
#   ├── config       ← 运行配置 yaml（ENG-01 §3.2）
#   ├── models       ← 目标模型与算法模型（ENG-01 §3.3）
#   └── calibration  ← 相机与光机标定数据（ENG-01 §3.4）
#
# bin / lib 用 GNUInstallDirs 的变量，保证与各模块 install(TARGETS ...) 的
# 目标位置一致；config/models/calibration 是工程专有目录，直接写名。
# ----------------------------------------------------------------------------
set(APS_INSTALL_CONFIG_DIR      "config")
set(APS_INSTALL_MODELS_DIR      "models")
set(APS_INSTALL_CALIBRATION_DIR "calibration")

# ----------------------------------------------------------------------------
# 3 纯数据资产
#
# 这三个目录的内容由后续阶段填充：
#   config/      → ENG-09 §6.5 的 measurement.yaml（16 个重试/超时字段）
#   models/      → SYS-12 目标模型与特征库；ENG-10 §6 的 CAD 模型输入要求
#   calibration/ → SYS-11 光机刚体标定产出
#
# 现在（001）这些目录都是空的，install(DIRECTORY ... OPTIONAL) 保证：
#   目录为空或不存在时**不报错**，一旦后续阶段放入文件，无需修改本文件
#   即自动纳入安装包。
#
# OPTIONAL 关键字在此不是偷懒：ENG-08 §5 Sprint 1 的验收就是"cmake/make
# 通过"，若因标定目录尚不存在而让 install 失败，会把正常的阶段推进
# 误判为构建错误。
# ----------------------------------------------------------------------------
# 排除项说明：这三个目录用 .gitkeep 占位，版本控制需要它，但**交付包不需要**
# 一个 0 字节的 VCS 占位文件——现场看到 deploy/config/.gitkeep 会以为配置
# 漏发了。除 .gitkeep 外一并排除其它点文件（.gitignore 等），它们同样只
# 服务于仓库而非运行。
#
# ⚠ 参数顺序是硬性的：OPTIONAL / MESSAGE_NEVER 等关键字必须写在
#   [PATTERN|REGEX] 之前。写成 DESTINATION → PATTERN → OPTIONAL 会直接
#   中断配置：
#       install DIRECTORY does not allow "OPTIONAL" after PATTERN or REGEX.
#   故三个 install(DIRECTORY ...) 的顺序统一为：
#       DESTINATION → OPTIONAL → PATTERN...
#   （已在 CMake 3.31 复现并修正。）
set(APS_INSTALL_EXCLUDES
    PATTERN ".gitkeep" EXCLUDE
    PATTERN ".gitignore" EXCLUDE
    PATTERN ".gitattributes" EXCLUDE
)

# 显式创建三个目标目录本身。
#
# 必要性：当源目录**只有** .gitkeep（即当前阶段，三个目录均为空）时，
# 上面的 PATTERN ... EXCLUDE 会把全部内容排除掉，此时 CMake 的
# install(DIRECTORY) 连目标目录**也不创建**——交付包里就没有 config/。
# 现场拿到一个没有 config/ 的包，会先怀疑配置漏发，而不是"这一步还没填
# 内容"，排查方向从一开始就错了。显式建目录消除这个歧义。
install(DIRECTORY DESTINATION "${APS_INSTALL_CONFIG_DIR}")
install(DIRECTORY DESTINATION "${APS_INSTALL_MODELS_DIR}")
install(DIRECTORY DESTINATION "${APS_INSTALL_CALIBRATION_DIR}")

install(DIRECTORY "${CMAKE_SOURCE_DIR}/config/"
        DESTINATION "${APS_INSTALL_CONFIG_DIR}"
        OPTIONAL
        ${APS_INSTALL_EXCLUDES})

install(DIRECTORY "${CMAKE_SOURCE_DIR}/models/"
        DESTINATION "${APS_INSTALL_MODELS_DIR}"
        OPTIONAL
        ${APS_INSTALL_EXCLUDES})

install(DIRECTORY "${CMAKE_SOURCE_DIR}/calibration/"
        DESTINATION "${APS_INSTALL_CALIBRATION_DIR}"
        OPTIONAL
        ${APS_INSTALL_EXCLUDES})

# ----------------------------------------------------------------------------
# 4 第三方 SDK 运行期动态库
#
# Dependencies.cmake 在找到 ImvSdk / TurntableSdk 时会把其运行期目录写入
# APS_IMVSDK_RUNTIME_DIRS。相机打不开的现场故障里，绝大多数是"可执行文件
# 到位了但 SDK 的 .so 没跟过去"，故此处主动随包下发。
#
# 使用 install(DIRECTORY) 而非 install(FILES)：SDK 的 runtime 目录常含
# 插桩/传输层的多级子目录，逐文件列举既易漏也难以维护。
# ----------------------------------------------------------------------------
if(DEFINED APS_IMVSDK_RUNTIME_DIRS AND APS_IMVSDK_RUNTIME_DIRS)
    foreach(_dir IN LISTS APS_IMVSDK_RUNTIME_DIRS)
        if(IS_DIRECTORY "${_dir}")
            install(DIRECTORY "${_dir}/"
                    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
                    OPTIONAL)
            message(STATUS "[APS] ImvSdk 运行期库将随包安装：${_dir}")
        endif()
    endforeach()
endif()

# ----------------------------------------------------------------------------
# 5 安装断言（在 `cmake --install` **内部**执行）
#
# ⚠ 断言**不在本文件**，在 `src/app/CMakeLists.txt` 里紧随 install(TARGETS)
#   之后。这是个必须记住的顺序陷阱，不是笔误：
#
#   CMake 生成的 cmake_install.cmake 中，**本目录自己的 install 规则全部排在
#   各子目录的 include() 之前** —— 与 add_subdirectory() 在 CMakeLists.txt 中
#   出现的位置无关。所以在这里写 install(CODE) 会在 `src/` 的规则之前执行，
#   那时 bin/AircraftPoseSystem 还没被拷贝过去，断言**必然失败**。
#   （已在 3.31 实测：本文件的 CODE 位于 install 脚本第 75~96 行，
#     而 `include(.../build/src/cmake_install.cmake)` 在第 102 行。）
#
#   凡"断言某个 target 已装好"的 CODE，都必须与那条 install(TARGETS ...)
#   写在同一个 CMakeLists.txt 里、且排在其后。
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# 6 安装自检
#
# 把安装动作本身登记成两条 CTest 测试，使 ENG-03 §20 第 6 条"支持离线部署"
# 在 CI 中可自动验证，而不是靠人工记得敲一次 cmake --install。
#
# 分工：
#   install_check        —— 跑得通 `cmake --install`，且产物校验通过
#                           （产物校验即 §5 所指 src/app 里的 install(CODE ...)）
#   install_launch_check —— 从**安装目录**真的启动一次，确认它是可运行的
#                           （R02，2026-09-24 审查报告）
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
    # ⚠ 先**清空自检前缀**再安装（R02 收尾，2026-09-24 审查报告）。
    #
    # 这一步不是洁癖，是必需的：本前缀在多次运行之间**留存**，而上一版自检
    # 只跑 `cmake --install`，于是"本次安装什么都没装出来"完全可以被**上一次
    # 的陈旧产物**掩盖。已实测复现：把 src/app 的整段 R02 安装规则停用
    # （install(TARGETS) 与 install(CODE) 断言一起停用），前缀里留着上一轮
    # 的 bin/AircraftPoseSystem 与 logs/，两条自检**依旧全绿**（install_check
    # 0.01 s、install_launch_check 3.00 s 通过）—— 而一次全新安装的产物里
    # 根本没有主程序。判据必须建立在"本次运行装出来的东西"上。
    #
    # 清空只针对自检前缀（${CMAKE_BINARY_DIR}/install_check），不碰 deploy/：
    # 后者是真实交付树，自检指向它会让一次 ctest 覆盖交付内容。
    # （.gitignore 已忽略 install_check，故它不会污染工作区状态。）
    #
    # 不写 COMMENT：CMake 的 add_test() 会把它并入命令参数，导致
    # `cmake --install` 报 "Unknown argument COMMENT"（已在 3.31 复现）。
    # 测试名本身已说明用途，无需额外注释字段。
    add_test(NAME install_check
        COMMAND sh -c
            "${CMAKE_COMMAND} -E rm -rf '${CMAKE_BINARY_DIR}/install_check' && ${CMAKE_COMMAND} --install '${CMAKE_BINARY_DIR}' --prefix '${CMAKE_BINARY_DIR}/install_check'")

    # 自检前缀固定为 ${CMAKE_BINARY_DIR}/install_check，**不是 deploy/**：
    # 后者是真实交付树，若自检指向它，一次 ctest 就会覆盖交付内容。
    # （.gitignore 已忽略 install_check，故它不会污染工作区状态。）

    # 从安装目录真的启动一次，并核对**四条**判据（R02 收尾）：
    #   ① 退出码 124；② 日志里有"启动完成"；
    #   ③ 日志里有首拍标记 APP_FIRST_TICK_COMPLETED；④ 日志里**没有**"启动失败"。
    #
    # ⚠ 为什么 ① 单独一条不够：`timeout 3` 把进程杀掉只能说明"跑满 3 秒仍存活"，
    #   而**初始化过程里卡死**同样满足 124 —— 那种情况下窗口从未显示、状态机从未
    #   推进一拍，而这恰恰是最需要被发现的一类缺陷（它表现为"界面一直不出来"，
    #   却能让自检通过）。②③ 把判据从"进程活着"推进到"启动走完了、且事件循环
    #   至少完成了一次应用定时回调"；④ 挡的是另一侧：程序打印了"启动失败"却
    #   因为没退出而活满 3 秒（例如失败后仍进入了事件循环）。
    #
    # ⚠ 输出必须落到**本次运行**的文件：旧版把输出扔进 /dev/null，出问题时
    #   只有一行"退出码不对"，既看不到程序打印了什么，也无法区分"缺动态库"
    #   与"初始化失败"。日志写在前缀内（本次运行刚装出来的那棵树里），
    #   失败时 cat 出来再退出非零。
    #
    # ⚠ **WORKING_DIRECTORY 必须显式设为安装前缀**，不能用 ctest 的默认值。
    #    本程序**默认按当前工作目录**解析配置目录（`main.cpp`：configDir 初值
    #    为 "config"），而 ctest 的默认工作目录是 ${CMAKE_BINARY_DIR}，那里没有
    #    config/，于是程序以**退出码 1** 打印"找不到配置文件"后退出 ——
    #    这条测试就会恒红，而失败原因与安装规则毫无关系。
    #    （实测：cwd=安装根 → 124；cwd=bin/ 或 build/ → 1。）
    #
    #    取"cwd=安装根 + 默认配置路径"而不是"用 argv[1] 显式指定配置目录"：
    #    前者正是现场解包后的动作（`cd deploy && ./bin/AircraftPoseSystem`），
    #    要测的就是这条默认路径能跑通。
    #    ⚠ 注意 argv[1] **确实**可以指定配置目录（故程序并非只能从包根启动），
    #      但 `models/`、`calibration/`、`logs/`、`output/` 仍按 CWD 解析
    #      （已登记为 Q-B10）。本测试不依赖这些路径 —— 它们缺失只打 WARN，
    #      不影响事件循环起来。
    #
    # ⚠ `\$?` 写成转义形式，但**这不是必需的** —— CMake 只把 `${…}` / `$ENV{…}` /
    #    `$CACHE{…}` / `$<…>` 当变量引用，裸 `$?` 它会原样输出。
    #    （已实测：去掉 `\` 后生成的命令逐字节相同。）转义写法只是把
    #      "这是给 sh 的、不是给 CMake 的"写明白而已。
    #    ⚠ 本条注释原先断言"不转义会被展开成空串、测试退化为无脑通过"，**那是错的**，
    #      已改正。真正会咬人的是另一侧：`${CMAKE_INSTALL_BINDIR}` 这类**确实**在
    #      配置期展开，写错位置就会得到配置期与安装期混拼的怪路径。
    #
    # ⚠ 不进 `deploy/`、也不重新安装 —— 依赖 install_check 已经装好。
    #    用 FIXTURES 而不是 DEPENDS：fixture 的 setup 测试即使做了
    #    `ctest -R install_launch_check` 也会被自动带上，DEPENDS 不会。
    add_test(NAME install_launch_check
        COMMAND sh -c
            "cd '${CMAKE_BINARY_DIR}/install_check' || exit 2; LOG='${CMAKE_BINARY_DIR}/install_check/launch_check.log'; QT_QPA_PLATFORM=offscreen timeout 3 './${CMAKE_INSTALL_BINDIR}/AircraftPoseSystem' >\"\$LOG\" 2>&1; RC=\$?; if [ \$RC -ne 124 ]; then echo \"[APS] 启动自检失败：退出码 \$RC（期望 124=被 timeout 杀掉，即事件循环已起来）\"; cat \"\$LOG\"; exit 1; fi; if ! grep -q '启动完成' \"\$LOG\"; then echo '[APS] 启动自检失败：日志中无\"启动完成\"'; cat \"\$LOG\"; exit 1; fi; if ! grep -q 'APP_FIRST_TICK_COMPLETED' \"\$LOG\"; then echo '[APS] 启动自检失败：无首拍标记 APP_FIRST_TICK_COMPLETED（初始化完成但事件循环未完成一次应用定时回调）'; cat \"\$LOG\"; exit 1; fi; if grep -q '启动失败' \"\$LOG\"; then echo '[APS] 启动自检失败：日志中出现\"启动失败\"'; cat \"\$LOG\"; exit 1; fi; echo '[APS] 启动自检通过：退出码 124 + 启动完成 + 首拍标记，且无启动失败'")

    set_tests_properties(install_check PROPERTIES
        FIXTURES_SETUP aps_install)
    set_tests_properties(install_launch_check PROPERTIES
        FIXTURES_REQUIRED aps_install
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/install_check"
        TIMEOUT 30)
endif()
