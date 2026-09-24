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
#  分工（重要）：
#    本文件    → 安装前缀、目录结构、纯数据资产（config/models/calibration）。
#    各模块    → 各自的 install(TARGETS ...)。**不在此集中写 install(TARGETS)**，
#                因为目标定义在 src/ 各子目录中，集中书写会让本文件依赖
#                加载顺序，且每新增一个模块都要回来改这里，正是 ENG-03 §2.1
#                "模块独立编译"要避免的耦合。
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
# 5 安装自检
#
# 把安装动作本身登记成一条 CTest 测试，使 ENG-03 §20 第 6 条"支持离线部署"
# 在 CI 中可自动验证，而不是靠人工记得敲一次 cmake --install。
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
    # 不写 COMMENT：CMake 的 add_test() 会把它并入命令参数，导致
    # `cmake --install` 报 "Unknown argument COMMENT"（已在 3.31 复现）。
    # 测试名本身已说明用途，无需额外注释字段。
    add_test(NAME install_check
        COMMAND ${CMAKE_COMMAND} --install "${CMAKE_BINARY_DIR}"
                --prefix "${CMAKE_BINARY_DIR}/install_check")
endif()
