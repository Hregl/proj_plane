# ============================================================================
#  cmake/FindImvSdk.cmake
#
#  依据：ENG-03 §10（ImvSDK 配置：include 路径 / library 路径 / runtime 路径）
#        ENG-08 §6（Sprint 2：真实单相机 SDK 接入，A7A20MU201）
#
#  查找对象：华睿（Huaray）A7A20MU201 工业相机 SDK（MVViewer 客户端包内的 SDK 部分）
#
#  产出变量：
#     ImvSdk_FOUND         - TRUE 表示头文件与库均已找到
#     ImvSdk_INCLUDE_DIRS  - 头文件目录
#     ImvSdk_LIBRARIES     - 链接参数（`-L<runtime>;-lMVSDK`，见 §3 的说明）
#     ImvSdk_LIBRARY_FILE  - libMVSDK 的**实际文件**（带版本号，供台账/校验用）
#     ImvSdk_RUNTIME_DIRS  - 运行期动态库目录（**由本文件生成**，见 §4）
#
#  产出目标（由 Dependencies.cmake 建立）：aps::imvsdk
#
#  用法：
#     cmake .. -DIMV_SDK_ROOT=<SDK 根目录>
#   或设置环境变量：export IMV_SDK_ROOT=<SDK 根目录>
#
#  ============================================================================
#  ⚠ 本文件于 2026-09-23 在**真实 SDK 包上重写**（A-1 解包 + 核验）。
#    初版是按"厂商 SDK 通用布局"猜的，实测**三处不符、且按原样必然失败**：
#
#    ① 平台子目录缺 `m64x86`。原始包布局是 `lib/m64x86/`（32 位为 `lib/m32x86/`），
#       初版的 PATH_SUFFIXES 只有 `lib` / `linux/lib` / `lib/x86_64-linux-gnu`。
#    ② **库只有带版本号的文件，没有 `.so` 软链**：实测归档 831 个条目中
#       `libMVSDK*` 只有两处 —— `libMVSDK.so.2.7.0.1.422704`（64 位）与
#       `libMVSDKGuiQt.so`。**且 `libMVSDK.so.2.7.0.1.422704` 自身没有 SONAME**。
#       故 `find_library(NAMES MVSDK)` 永远匹配不到（它只认 `libMVSDK.so`/`.a`）。
#       → 本文件改用 `find_file` 精确定位，见 §3。
#    ③ 包内 `lib/m64x86/Qt/`（Qt **5.6.3**）与 `lib/m32x86/Qt/`（Qt **5.5.1**）
#       与本工程的系统 Qt 5.15.8 冲突，**必须排除**。实测 `libMVSDK` 本身
#       `ldd` 无任何 Qt 依赖（GUI 才有，走 `libMVSDKGuiQt.so`），故排除是安全的。
#
#    ⚠ 厂商自带的例程 Makefile 写的是 `-L<SDK>/lib -lMVSDK`，在本包上**连不上**
#      （缺 `libMVSDK.so`）。原因是它按 `install.sh` **安装后**的布局写：
#      `install.sh` 会把 `lib/m64x86/*` 拍平复制到 `<安装根>/lib/`，
#      但**不会**补软链。本工程按 O-19 裁决"解包而不安装"，故必须自己补，
#      这正是 §4 生成 runtime/ 的原因。
# ============================================================================

# ----------------------------------------------------------------------------
# 1 SDK 根目录
#
# 优先级（评审 2026-09-23 裁定）：
#     命令行 -DIMV_SDK_ROOT  >  环境变量 IMV_SDK_ROOT  >  默认路径  >  失败提示
#
# ⚠ 默认路径**不含**任何"在系统里乱搜"的项：只认两个有依据的位置 ——
#   本工程的解包位置（third_party/imvsdk，O-19 裁决）与厂商安装器的落点
#   （/opt/HuarayTech/MVviewer，由包内 install.sh 的 INSTALL_ROOT=/opt/$1/$2
#    与 scriptargs "HuarayTech MVviewer" 决定，见 A-1 核验）。
# ----------------------------------------------------------------------------

# 先取命令行/CACHE 值；未给则回落到环境变量。用 set(... CACHE ...) 保证在
# ccmake/GUI 里可见可改，同时 `if(NOT IMV_SDK_ROOT)` 使环境变量只在"没显式给"时生效。
set(IMV_SDK_ROOT "${IMV_SDK_ROOT}" CACHE PATH "华睿 ImvSdk 安装根目录")
if(NOT IMV_SDK_ROOT AND DEFINED ENV{IMV_SDK_ROOT})
    set(IMV_SDK_ROOT "$ENV{IMV_SDK_ROOT}")
endif()

set(_IMVSDK_DEFAULT_ROOTS
    "${CMAKE_CURRENT_LIST_DIR}/../third_party/imvsdk"   # O-19：本工程解包位置
    "/opt/HuarayTech/MVviewer"                          # 厂商 install.sh 的落点
)

if(NOT IMV_SDK_ROOT)
    foreach(_r IN LISTS _IMVSDK_DEFAULT_ROOTS)
        if(EXISTS "${_r}/include/IMVApi.h")
            get_filename_component(IMV_SDK_ROOT "${_r}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

# ----------------------------------------------------------------------------
# 2 头文件
#
# 实测（A-1）：原始包与安装布局下头文件都在 `<根>/include/`，主头为 `IMVApi.h`，
# 另有 `IMVDefines.h` / `IMVFGApi.h` / `IMVFGDefines.h`。
# 不再罗列 `ImvApi.h` / `imv_api.h` / `MVSDK.h` 等臆测名 —— 保留它们只会在
# 同名头文件出现在别处时匹配到**错误的 SDK**。
# ----------------------------------------------------------------------------
find_path(ImvSdk_INCLUDE_DIR
    NAMES IMVApi.h
    HINTS "${IMV_SDK_ROOT}"
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
    DOC "ImvSdk 头文件目录（含 IMVApi.h）"
)

# ----------------------------------------------------------------------------
# 3 库文件（**精确定位，不盲搜**）
#
# 实测两种布局都要支持：
#   · 原始包布局：`<根>/lib/m64x86/libMVSDK.so.2.7.0.1.422704`
#   · 安装后布局：`<根>/lib/libMVSDK.so.2.7.0.1.422704`（install.sh 拍平的结果）
#
# 因为该文件**没有 SONAME 也没有 .so 软链**，只能用 find_file 按名字找。
# 名字取"带版本号的实测名"在前、"通用名"在后：前者可命中真实包，后者兼容
# 将来厂商补了软链的版本。
# ----------------------------------------------------------------------------
set(_IMVSDK_LIB_SUBDIRS
    lib/m64x86     # 原始包（64 位 x86）
    lib            # install.sh 安装后（拍平）
    lib64
)

find_file(ImvSdk_LIBRARY_FILE
    NAMES libMVSDK.so.2.7.0.1.422704 libMVSDK.so
    HINTS "${IMV_SDK_ROOT}"
    PATH_SUFFIXES ${_IMVSDK_LIB_SUBDIRS}
    NO_DEFAULT_PATH
    DOC "ImvSdk 主库文件（libMVSDK）"
)

set(ImvSdk_GENICAM_LIB_DIR "")
if(ImvSdk_LIBRARY_FILE)
    get_filename_component(_imvsdk_lib_dir "${ImvSdk_LIBRARY_FILE}" DIRECTORY)

    # GenICam 运行时在 `GenICam/bin/Linux64_x64/`，是 libMVSDK 的硬依赖
    # （实测 ldd 有 7 个 lib*_gcc421_v3_0.so）。找不到也不致命：下面的
    # runtime 生成会把已存在的都链进去，最终由 §5 的存在性检查说话。
    foreach(_g Linux64_x64 Linux64_x86 linux64-x64)
        if(EXISTS "${_imvsdk_lib_dir}/GenICam/bin/${_g}")
            set(ImvSdk_GENICAM_LIB_DIR "${_imvsdk_lib_dir}/GenICam/bin/${_g}")
            break()
        endif()
    endforeach()
endif()

# ----------------------------------------------------------------------------
# 4 运行期目录 runtime/（ENG-03 §10 明确要求管理 runtime 路径）
#
# ⚠ 这一段不是"尽力探测"，而是**构造**。原因见文件头 ② 与 ③：
#   · libMVSDK 要的 11 个库里，有的文件名与 SONAME 不一致
#     （`libcompress_decode.so` 的 SONAME 是 `libcompress_decode.so.1`），
#      加载器按 SONAME 找不到 → 即便 LD_LIBRARY_PATH 指对，**打开相机时才失败**；
#   · 链接期还需要一个包内**不存在**的 `libMVSDK.so`。
#   故本文件在**构建树**里生成一个 `imvsdk_runtime/`，把所需动态库按加载器
#   要的**名字**软链齐，并把它作为唯一的运行期目录。
#
# 放在构建树而不是 SDK 树里：可复现（每次 configure 重建）、不改动解包产物、
# 且与 third_party/imvsdk/ 被 .gitignore 忽略无关。
#
# ⚠ 明确**排除**任何路径含 `/Qt` 的目录 —— 评审 O-19 与 011-A0 计划 §2.3 第 8 项。
# ----------------------------------------------------------------------------
set(ImvSdk_RUNTIME_DIRS "")
set(_imvsdk_runtime "${CMAKE_BINARY_DIR}/imvsdk_runtime")

if(ImvSdk_LIBRARY_FILE)
    file(REMOVE_RECURSE "${_imvsdk_runtime}")   # 幂等：重复 configure 不残留旧链
    file(MAKE_DIRECTORY "${_imvsdk_runtime}")

    # 4.1 SDK 自带库。目录**非递归**遍历：`Qt/` 是子目录，天然不会被取到；
    #     但仍逐项判一次路径，避免将来有人把 Qt 库挪到同级。
    file(GLOB _imvsdk_own_libs
        "${_imvsdk_lib_dir}/*.so"
        "${_imvsdk_lib_dir}/*.so.*")
    foreach(_lib IN LISTS _imvsdk_own_libs)
        if(_lib MATCHES "/Qt/")
            continue()
        endif()
        # GUI 用库：依赖 Qt5，本工程（无 GUI 依赖的采集进程）不需要，链进去
        # 反而会把 Qt 5.6.3 拖进运行期目录。实测 libMVSDK 自身不依赖它。
        if(_lib MATCHES "libMVSDKGuiQt")
            continue()
        endif()
        get_filename_component(_n "${_lib}" NAME)
        file(CREATE_LINK "${_lib}" "${_imvsdk_runtime}/${_n}" SYMBOLIC)
    endforeach()

    # 4.2 GenICam 运行时（libMVSDK 的硬依赖，实测 7 个）
    if(ImvSdk_GENICAM_LIB_DIR)
        file(GLOB _imvsdk_genicam_libs
            "${ImvSdk_GENICAM_LIB_DIR}/*.so"
            "${ImvSdk_GENICAM_LIB_DIR}/*.so.*")
        foreach(_lib IN LISTS _imvsdk_genicam_libs)
            if(_lib MATCHES "/Qt/")
                continue()
            endif()
            get_filename_component(_n "${_lib}" NAME)
            file(CREATE_LINK "${_lib}" "${_imvsdk_runtime}/${_n}" SYMBOLIC)
        endforeach()
    endif()

    # 4.3 ⚠ 名字修正（**实测所得，不是猜测**）。
    #     左列是加载器要的名字（来自 ldd，即被依赖方记录的 SONAME 或文件名），
    #     右列是包内实际存在的文件。两者不一致时必须补链，否则**编译能过、
    #     运行到打开相机才失败** —— 那正是本项目最警惕的失效形态。
    #
    #     libcompress_decode.so.1  ← 文件 libcompress_decode.so（SONAME 就是 .so.1）
    #     liblog4cpp.so.5          ← 文件 liblog4cpp.so（SONAME 就是 .so.5）
    #     libusb-1.0.so.0          ← 文件 libusb-1.0.so（SONAME 就是 .so.0）
    #                                （系统也有同名库，此处优先用 SDK 自带的配版）
    #     libMVSDK.so              ← **链接期**需要，包内根本没有（见文件头 ②）
    set(_imvsdk_alias_pairs
        "libcompress_decode.so.1" "libcompress_decode.so"
        "liblog4cpp.so.5"         "liblog4cpp.so"
        "libusb-1.0.so.0"         "libusb-1.0.so"
        "libMVSDK.so"             "libMVSDK.so.2.7.0.1.422704"
    )
    set(_imvsdk_aliases_ok TRUE)
    while(_imvsdk_alias_pairs)
        list(POP_FRONT _imvsdk_alias_pairs _alias _target)
        if(EXISTS "${_imvsdk_runtime}/${_target}")
            file(CREATE_LINK "${_target}" "${_imvsdk_runtime}/${_alias}" SYMBOLIC)
        else()
            # 目标不存在有两种情形：换了 SDK 版本（名字变了）或解包不全。
            # 不在这里 FATAL —— 由 §5 的完整性与解析检查统一报，消息更集中。
            set(_imvsdk_aliases_ok FALSE)
        endif()
    endwhile()

    set(ImvSdk_RUNTIME_DIRS "${_imvsdk_runtime}")
endif()

# ----------------------------------------------------------------------------
# 5 收尾
# ----------------------------------------------------------------------------
include(FindPackageHandleStandardArgs)

set(_IMVSDK_FAIL_MESSAGE
    "未找到 ImvSdk。请用 -DIMV_SDK_ROOT=<SDK根目录> 指定，或设置环境变量 IMV_SDK_ROOT。
       本工程约定的解包位置：<工程根>/third_party/imvsdk（O-19 裁决）。
       解包方法（**不执行**厂商 install.sh）见 third_party/README.md §1.3。
       第一阶段（ENG-08 §3）不依赖该 SDK，软件闭环用 VirtualCameraBackend；
       Sprint 2 接入真实相机时才需要。")

find_package_handle_standard_args(ImvSdk
    REQUIRED_VARS
        ImvSdk_INCLUDE_DIR
        ImvSdk_LIBRARY_FILE
    FAIL_MESSAGE "${_IMVSDK_FAIL_MESSAGE}"
)

if(ImvSdk_FOUND)
    set(ImvSdk_INCLUDE_DIRS "${ImvSdk_INCLUDE_DIR}")

    # ⚠ 链接参数用 `-L<runtime> -lMVSDK`，**不是**直接链那个带版本号的文件。
    #   理由：`libMVSDK.so.2.7.0.1.422704` **没有 SONAME**，直接以绝对路径链接时
    #   链接器会把该**绝对路径**写进 DT_NEEDED —— 构建树搬走后二进制就失效。
    #   走 `-l` 时，无 SONAME 的库会把**文件名**（`libMVSDK.so`）写进 DT_NEEDED，
    #   运行期由 RPATH/LD_LIBRARY_PATH 解析，可重定位。
    #
    #   ✅ 上面最后一句已于 2026-09-23 由**真实调用 SDK 符号的探针实测确认**
    #      （011-A0.1 核验报告 §6.2）：`readelf -d` 得到
    #          NEEDED  共享库：[libMVSDK.so]
    #      即写进 DT_NEEDED 的是**文件名**，不是绝对路径。
    #
    #   ⚠⚠ 但同一次实测还发现一个**光有 runtime/ 目录解决不了**的问题：
    #      现代 binutils 默认 `--enable-new-dtags`，`-Wl,-rpath` 生成的是
    #      **RUNPATH**，而 RUNPATH **不传递** —— 它只作用于可执行文件**直接**
    #      链接的库，不作用于那些库自己再加载的依赖。于是出现：
    #          可执行文件能找到 libMVSDK.so，
    #          但 libMVSDK.so 找不到 libGCBase_gcc421_v3_0.so → 启动即失败：
    #          "libGCBase_gcc421_v3_0.so: cannot open shared object file"
    #      （实测：RUNPATH 版本退出码 127；同一二进制加 LD_LIBRARY_PATH 后
    #        `IMV_GetVersion() = 2.7.0.1.422704`、`IMV_EnumDevices` 返回 IMV_OK。）
    #
    #      ⇒ 故仅提供 `ImvSdk_RUNTIME_DIRS` 是**必要但不充分**的。构建树里由
    #        Dependencies.cmake 用 `--disable-new-dtags` 生成**DT_RPATH**（可传递）
    #        解决；安装后（deploy/）由 CMAKE_SKIP_INSTALL_RPATH 清空 RPATH，
    #        需靠 LD_LIBRARY_PATH —— 该缺口登记在核验报告 §7.3，由 011-B 处置。
    set(ImvSdk_LIBRARIES "-L${ImvSdk_RUNTIME_DIRS};-lMVSDK")

    if(NOT _imvsdk_aliases_ok)
        message(WARNING
            "[APS] ImvSdk 的名称修正链接未全部生成：runtime/ 里缺少预期的文件。\n"
            "      这通常意味着 SDK 版本与 cmake/FindImvSdk.cmake §4.3 记录的实测名不符。\n"
            "      后果是**运行到打开相机时才失败**，请先跑核验：\n"
            "        LD_LIBRARY_PATH=${ImvSdk_RUNTIME_DIRS} ldd ${ImvSdk_LIBRARY_FILE} | grep 'not found'")
    endif()
endif()

mark_as_advanced(
    IMV_SDK_ROOT
    ImvSdk_INCLUDE_DIR
    ImvSdk_LIBRARY_FILE
)
