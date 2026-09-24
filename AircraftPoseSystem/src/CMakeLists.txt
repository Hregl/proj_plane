# ============================================================================
#  src/CMakeLists.txt
#
#  依据：ENG-03 §12（src 模块设计）、§2.1（模块独立编译）、§2.2（单向依赖）
#        ENG-01 §4（src 源码目录）、§17（模块依赖关系——冻结）
#
#  ┌────────────────────────────────────────────────────────────────────────┐
#  │ 本文件定义全工程唯一的模块装配函数 aps_add_module_library()，并按下述    │
#  │ 依赖顺序加载 10 个模块。                                                │
#  │                                                                        │
#  │ 加载顺序不是风格问题：target_link_libraries() 要求被链接的目标已存在。   │
#  │ 顺序 = ENG-01 §17 冻结依赖链的自底向上展开。                            │
#  └────────────────────────────────────────────────────────────────────────┘
#
#  冻结依赖链（ENG-01 §17 / ENG-03 §12，方向与 C-18 裁决一致）：
#
#      app
#       ↓
#      ui
#       ↓
#      application
#       ↓
#      preview  optical
#       ↓
#      device   algorithm
#       ↓
#      infrastructure
#       ↓
#      data
#
#  ⚠ ENG-01 §17 的方框图存在一处歧义，此处按 ENG-03 §12 的逐模块声明执行：
#    §17 把 `preview / optical` 画在 `device / algorithm` **之上**，若按"箭头
#    即依赖"读，会得到 optical 依赖 device。但 ENG-03 §12.3 明确 device 依赖
#    optical，且 C-18 裁决亦冻结"optical 位于 device 之下，device 依赖
#    optical"。**§12 的显式声明优先**，§17 应理解为粗略分层示意。
#
#  ⚠ ENG-03 §3 的目录树缺少 runtime/，但 ENG-01 §4/§12 与 1.md 目录树均含
#    runtime/。ENG-01 是目录结构的权威（其 §1 即声明本文件为工程基线），故
#    **保留 src/runtime/**。它不在 §17 的依赖链与 §12.9 的链接表上——当前
#    无人消费 ComputeBackend，故 libruntime 不链入 app，仅独立编译。
# ============================================================================

# ----------------------------------------------------------------------------
# 1 模块库类型
#
# 默认 STATIC。理由：本系统离线交付整套设备软件，静态库让最终产物是一个
# 自包含的可执行文件，现场只需处理 Qt / OpenCV / 相机 SDK 三类外部 .so，
# 不必再关心本项目 10 个模块库的加载路径与版本匹配。
#
# 保留可切换能力：-DAPS_LIBRARY_TYPE=SHARED，供"模块热替换"调试场景使用。
# ----------------------------------------------------------------------------
set(APS_LIBRARY_TYPE "STATIC"
    CACHE STRING "APS 模块库类型：STATIC | SHARED")
set_property(CACHE APS_LIBRARY_TYPE PROPERTY STRINGS STATIC SHARED)

# ----------------------------------------------------------------------------
# 2 模块装配函数 —— 全工程唯一的库定义入口
#
# 用法：
#     aps_add_module_library(<target> [RECURSE] [DEPENDS <t1> <t2> ...])
#
# 设计要点（三条，逐条对应 ENG-03 的硬要求）：
#
#   (1) 空模块自动跳过 —— 这是"逐阶段生成"工程能持续保持绿灯的关键。
#       1.md 规定按 Phase 1~9 逐步生成文件，任何时刻都有大量模块尚无源文件。
#       若直接 add_library(空)，CMake 会以
#           "No SOURCES given to target: data"
#       中断配置，使每一步的中间状态都无法构建与验证。
#       跳过时打印 STATUS 说明，**不静默**——静默会让"模块漏加源文件"
#       与"模块本就没写"无法区分。
#
#   (2) 只链接已存在的依赖 —— 与 (1) 配套。若 application 已有源文件而
#       device 尚无，未加判断的 target_link_libraries(application PUBLIC device)
#       会被 CMake 当成裸库名 `-ldevice`，在**链接期**才失败，报错信息
#       （cannot find -ldevice）与真实原因（模块还没写）相距甚远。
#       此处用 if(TARGET) 过滤，并打印提示说明是"尚未生成"而非"配错了"。
#
#   (3) 每个目录只收集自己的 *.cpp（非递归）—— 保证源文件归属唯一。
#       若父模块递归收集，则子目录将来一旦建立自己的库，同一份 .cpp 会被
#       编译两次，链接期出现 duplicate symbol；且"哪个模块拥有哪个文件"
#       变得不可回答。需要递归时显式传 RECURSE。
#
# CONFIGURE_DEPENDS：让 CMake 在构建时检查目录内容变化。没有它，新增 .cpp
# 后必须手动重新 cmake，否则新文件不参与编译——正是"加了个文件却没生效"
# 这类最难排查的问题的来源。
# ----------------------------------------------------------------------------
function(aps_add_module_library target)
    cmake_parse_arguments(ARG "RECURSE" "" "DEPENDS" ${ARGN})

    if(ARG_RECURSE)
        file(GLOB_RECURSE _sources CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
    else()
        file(GLOB _sources CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
    endif()

    # (1) 空模块跳过
    if(NOT _sources)
        message(STATUS
            "[APS] 模块 ${target}：暂无源文件，跳过生成（加入 .cpp 后自动启用）")
        return()
    endif()

    add_library(${target} ${APS_LIBRARY_TYPE} ${_sources})

    # 统一头文件根：使全工程使用 `data/ImageFrame.h`、`optical/OpticalRig.h`
    # 这类**带模块前缀**的包含形式（与 SYS-06 §4.2、ENG-04 的文件标注一致）。
    # PUBLIC 传播，消费者无须各自添加。
    target_include_directories(${target} PUBLIC
        "${PROJECT_SOURCE_DIR}/src")

    # 位置无关代码：静态库在被链入可执行文件时需要；切 SHARED 时同样需要
    set_target_properties(${target} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF)

    # (2) 只链接已存在的依赖
    foreach(_dep IN LISTS ARG_DEPENDS)
        if(TARGET ${_dep})
            target_link_libraries(${target} PUBLIC ${_dep})
        else()
            message(STATUS
                "[APS] 模块 ${target}：依赖 ${_dep} 尚未生成，暂不链接")
        endif()
    endforeach()

    # 登记，供 app/CMakeLists.txt 汇总与后续审计使用
    set_property(GLOBAL APPEND PROPERTY APS_CREATED_MODULES ${target})

    # (3) 安装规则（R02，2026-09-24 审查报告）
    #
    # 为什么写在这里而不是各模块自己的 CMakeLists.txt：
    # **本函数是唯一知道"这个 target 是否存在"的地方** —— 上面第 (1) 步对空模块
    # 直接 return()，那时 add_library 根本没执行。若写在各模块文件里，每个都要
    # 自己再判一次 if(TARGET ...)，漏一个就是配置期直接报
    #   install TARGETS given target "data" which does not exist
    # 把"这个模块还没写"变成"整个工程配置不过" —— 正是第 (1) 步要避免的事。
    #
    # 也不违反 cmake/InstallRules.cmake 文件头那句"不集中写 install(TARGETS)"的
    # **理由**（那里担心的是"本文件依赖加载顺序"）：include(InstallRules) 在顶
    # CMakeLists 中早于 add_subdirectory(src)，那时无任何 target；
    # 而本函数恰好在 target 刚建立之后执行。
    #
    # 三种 DESTINATION 全给：对 STATIC/SHARED 用同一份规则，避免"某一种库型
    # 静默不安装"。不写 INCLUDES DESTINATION —— 它只在 EXPORT 时起作用，
    # 而 ENG-03 §17 的冻结布局里没有 include/。
    install(TARGETS ${target}
            ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

    # 登记"会产生库产物、且已安装"的目标，供 src/app 的安装断言逐个核对。
    #
    # ⚠ 与上面的 APS_CREATED_MODULES **刻意分开**：那个属性还含聚合模块
    #   （aps_add_aggregate_module 也往里 APPEND），而聚合模块是 INTERFACE 库、
    #   **没有产物**，混在一起会让断言去找不存在的 lib<聚合名>.a。
    #   本属性只在真正执行过 add_library 的路径上追加，故恰好是"应当存在文件"的集合。
    set_property(GLOBAL APPEND PROPERTY APS_INSTALLED_LIBRARY_TARGETS ${target})

    list(LENGTH _sources _n)
    message(STATUS "[APS] 模块 ${target}：${_n} 个源文件")
endfunction()

# ----------------------------------------------------------------------------
# 3 聚合模块装配函数
#
# 用法：
#     aps_add_aggregate_module(<target> DEPENDS <t1> <t2> ...)
#
# 存在理由（这是 001 阶段遗留的一个真实缺口，003 阶段才暴露）：
#
#   ENG-03 §14 要求 device 按外部依赖拆分为 device_camera /
#   device_turntable / device_trigger 三个静态库，再由 libdevice 聚合。
#   但"聚合"这一动作若靠 aps_add_module_library(device ...) 完成，
#   会因为 **src/device/ 目录下本就没有 .cpp**（源文件都在子目录里）
#   而被 (1) 规则跳过 —— 于是 `device` 这个目标**根本不存在**。
#
#   此时上层写 target_link_libraries(application PUBLIC device) 时，
#   会被 (2) 规则的 if(TARGET) 过滤掉，并打印一条"依赖 device 尚未生成"
#   的 STATUS 提示 —— 而 device **已经写好了**。
#   最终结果是 application 静默地没有链接任何设备代码，
#   错误推迟到链接期以 undefined reference 的形式出现，
#   且提示信息把人引向"device 还没写"这一错误方向。
#
#   "跳过空的中间层"对真正的空模块是对的（001 的三条设计要点），
#   对**只有子库的聚合层**则是错的：聚合层非空，它的内容就是子库。
#   本函数把这两种情形区分开。
#
# 为什么用 INTERFACE 而不是再造一个空源文件的 STATIC：
#   加一个空 .cpp 能让 add_library 成立，但那个文件没有任何语义，
#   纯粹为绕过构建规则而存在；且它会被 (1) 规则当作有效源文件，
#   使"本模块有哪些源文件"这一问题的答案变成一个谎言。
#   INTERFACE 库如实表达"本目标只做链接聚合，不产出目标文件"。
#
# 与 ENG-03 §12.3"生成 libdevice"的关系：
#   实际归档为 libdevice_camera.a / libdevice_turntable.a /
#   libdevice_trigger.a 三个（§14 的拆分要求）。§12.3 的 "libdevice"
#   在此理解为"device 模块这一可链接的装配单位"，由本 INTERFACE 目标承担。
#   二者冲突时以更具体的 §14 为准，差异记入 README §6 偏离表。
# ----------------------------------------------------------------------------
function(aps_add_aggregate_module target)
    cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})

    # 同名实体库已存在（本目录真有 .cpp）时不重复创建：
    # 否则 add_library 会因目标重名直接报错。
    if(TARGET ${target})
        message(STATUS
            "[APS] 聚合模块 ${target}：实体库已存在，无需聚合")
        return()
    endif()

    add_library(${target} INTERFACE)

    # 与实体库一致的头文件根（见 aps_add_module_library 的同名说明）
    target_include_directories(${target} INTERFACE
        "${PROJECT_SOURCE_DIR}/src")

    foreach(_dep IN LISTS ARG_DEPENDS)
        if(TARGET ${_dep})
            target_link_libraries(${target} INTERFACE ${_dep})
        else()
            message(STATUS
                "[APS] 聚合模块 ${target}：依赖 ${_dep} 尚未生成，暂不链接")
        endif()
    endforeach()

    set_property(GLOBAL APPEND PROPERTY APS_CREATED_MODULES ${target})

    message(STATUS
        "[APS] 聚合模块 ${target}：INTERFACE 库（无自身源文件，仅聚合子库）")
endfunction()

# ----------------------------------------------------------------------------
# 4 模块加载（顺序即 ENG-01 §17 冻结依赖链的自底向上展开）
#
# 第 1 层 data             —— 无依赖（ENG-03 §12.1）
# 第 2 层 infrastructure    —— 依赖 data（ENG-03 §12.7 未列依赖；按 §17 依赖链
#                              补 data：Logger/Recorder 要写 ImageFrame 与
#                              ShipPoseResult 字段，无 data 无法编译）
# 第 3 层 runtime           —— 依赖 data（ENG-01 §12；见文件头 ⚠ 说明）
# 第 4 层 optical           —— 依赖 data（ENG-03 §12.2）
# 第 5 层 algorithm         —— 依赖 data + OpenCV（ENG-03 §12.5）
# 第 6 层 device            —— 依赖 data + optical + SDK(optional)（ENG-03 §12.3）
# 第 7 层 preview           —— 依赖 data + optical（ENG-03 §12.4）
# 第 8 层 application       —— 依赖 device + preview + optical + algorithm（§12.6）
# 第 9 层 ui                —— 依赖 Qt + application + preview（ENG-03 §12.8）
# 第 10 层 app              —— 可执行文件（ENG-03 §12.9）
# ----------------------------------------------------------------------------
add_subdirectory(data)
add_subdirectory(infrastructure)
add_subdirectory(runtime)
add_subdirectory(optical)
add_subdirectory(algorithm)
add_subdirectory(device)
add_subdirectory(preview)
add_subdirectory(application)
add_subdirectory(ui)
add_subdirectory(app)
