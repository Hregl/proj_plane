# ============================================================================
#  cmake/FindTurntableSdk.cmake
#
#  依据：ENG-03 §11（Turntable SDK 预留：Peko SDK / 网络协议库 / RS485）
#        ENG-08 §11（第一阶段不实现 Peko协议 / RS485 / 网络控制）
#
#  **本文件在第一阶段是纯预留。** ENG-08 §11 明确第一阶段只交付
#  ITurntableController + VirtualTurntable，因此本文件当前**预期返回
#  NOTFOUND**，且这不构成任何问题——Dependencies.cmake 对它是 QUIET 查找，
#  找不到只打印一条 STATUS。
#
#  产出变量：
#     TurntableSdk_FOUND        - TRUE 表示找到
#     TurntableSdk_INCLUDE_DIRS - 头文件目录
#     TurntableSdk_LIBRARIES    - 需要链接的库
#     TurntableSdk_RUNTIME_DIRS - 运行期动态库目录
#
#  产出目标（由 Dependencies.cmake 建立）：aps::turntablesdk
#
#  用法（Phase 2 接入转台时）：
#     cmake .. -DTURNTABLE_SDK_ROOT=/opt/PekoSdk
#
#  ⚠ 待确认项（Phase 2）：
#    库名候选基于"转台厂商 SDK 常见命名"推测。ENG-03 §11 列出的三类适配
#    对象（Peko SDK / 网络协议库 / RS485 通信库）可能并非同一个包，届时
#    需按实际交付的 SDK 拆分本文件或增加多个查找分支。
# ============================================================================

set(TURNTABLE_SDK_ROOT "$ENV{TURNTABLE_SDK_ROOT}"
    CACHE PATH "转台 SDK 安装根目录")

set(_TT_HINTS
    ${TURNTABLE_SDK_ROOT}
    /opt/PekoSdk
    /opt/TurntableSdk
    /usr/local/PekoSdk
)

# ----------------------------------------------------------------------------
# 1 头文件
# ----------------------------------------------------------------------------
set(_TT_HEADER_NAMES
    PekoApi.h
    PekoD.h
    TurntableApi.h
    turntable_api.h
)

find_path(TurntableSdk_INCLUDE_DIR
    NAMES ${_TT_HEADER_NAMES}
    HINTS ${_TT_HINTS}
    PATH_SUFFIXES include Include inc linux/include
    DOC "转台 SDK 头文件目录"
)

# ----------------------------------------------------------------------------
# 2 库文件
# ----------------------------------------------------------------------------
set(_TT_LIBRARY_NAMES
    PekoApi
    PekoD
    peko
    TurntableSdk
    turntable
)

find_library(TurntableSdk_LIBRARY
    NAMES ${_TT_LIBRARY_NAMES}
    HINTS ${_TT_HINTS}
    PATH_SUFFIXES lib Lib library lib/x86_64-linux-gnu linux/lib
    DOC "转台 SDK 库文件"
)

# ----------------------------------------------------------------------------
# 3 运行期目录
# ----------------------------------------------------------------------------
set(TurntableSdk_RUNTIME_DIRS "")

if(TurntableSdk_LIBRARY)
    get_filename_component(_tt_lib_dir "${TurntableSdk_LIBRARY}" DIRECTORY)

    foreach(_cand
            "${_tt_lib_dir}"
            "${_tt_lib_dir}/../runtime"
            "${TURNTABLE_SDK_ROOT}/runtime")
        if(EXISTS "${_cand}")
            get_filename_component(_abs "${_cand}" ABSOLUTE)
            list(APPEND TurntableSdk_RUNTIME_DIRS "${_abs}")
        endif()
    endforeach()

    if(TurntableSdk_RUNTIME_DIRS)
        list(REMOVE_DUPLICATES TurntableSdk_RUNTIME_DIRS)
    endif()
endif()

# ----------------------------------------------------------------------------
# 4 标准 find_package 收尾
#
# 注意：FAIL_MESSAGE 里刻意说明"未找到是正常的"，因为这是第一阶段最常见的
# 一条输出——若措辞像错误，会误导后续维护者去"修"一个不该修的状态。
# ----------------------------------------------------------------------------
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(TurntableSdk
    REQUIRED_VARS
        TurntableSdk_INCLUDE_DIR
        TurntableSdk_LIBRARY
    FAIL_MESSAGE
        "未找到 TurntableSdk —— 这是第一阶段的预期状态，无需处理。
         按 ENG-08 §11，第一阶段只交付 ITurntableController + VirtualTurntable，
         不实现 Peko_D 协议 / RS485 / 网络控制；Phase 2 拿到转台 SDK 后再用
         -DTURNTABLE_SDK_ROOT=<SDK根目录> 指定。"
)

if(TurntableSdk_FOUND)
    set(TurntableSdk_INCLUDE_DIRS "${TurntableSdk_INCLUDE_DIR}")
    set(TurntableSdk_LIBRARIES    "${TurntableSdk_LIBRARY}")
endif()

mark_as_advanced(
    TURNTABLE_SDK_ROOT
    TurntableSdk_INCLUDE_DIR
    TurntableSdk_LIBRARY
)
