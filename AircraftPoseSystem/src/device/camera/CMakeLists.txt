# ============================================================================
#  src/device/camera/CMakeLists.txt  →  device_camera
#
#  依据：ENG-03 §14（Device 子模块 CMake：camera 管理 Camera SDK）
#        ENG-01 §8、SYS-06 §5~§8
#
#  依赖：data + aps::imvsdk
#
#  本子模块包含：
#    ICameraBackend.h          —— 接口（SYS-06 §6.1）
#    MultiCameraManager.h/.cpp —— 三相机管理（SYS-06 §5）
#    VirtualCameraBackend.h/.cpp —— 虚拟相机（1.md Phase 4）
#    ImvCameraBackend.h/.cpp   —— 真实 SDK 适配（ENG-08 §6 / Sprint 2）
#
#  ⚠ 接口位置（回应 1.md 第三阶段的 src/interfaces/ 规划）：
#  ICameraBackend.h **放在本目录**，不放 src/interfaces/。
#  依据 ENG-01 §4：src 模块表里没有 interfaces 模块；ENG-02 §5 与
#  ENG-03 §12.3 亦将接口置于使用方旁边。ENG-01 §11 对此有明确的原则说明：
#      "跨层的接口必须落在依赖关系的最低公共层；同层的接口可与实现并置。"
#  ICameraBackend 及其全部实现（ImvCameraBackend / VirtualCameraBackend）
#  **同处本目录一层**，不存在反向引用，故并置合法——这与 C-07
#  （CameraRole 必须下沉 data/）是同一原则的两种结果，不矛盾。
#
#  1.md 并额外列出了 IPoseEstimator.h，该名称在 22 份设计文档中**从未出现**，
#  属 1.md 自创，未进入冻结基线，本阶段不实现；若确需，须走 ENG-09 §8 的
#  变更流程先登记裁决。
#
#  ⚠ SDK 链接方式：aps::imvsdk 为可选依赖。VirtualCameraBackend 不需要它，
#  只有 ImvCameraBackend.cpp 需要。当前用 PUBLIC 链接是简化处理——待
#  Sprint 2 接入真实 SDK 时，若发现其头文件污染了消费者的编译单元，改为
#  PRIVATE（届时在 aps_add_module_library 中增加 PRIVATE 依赖支持）。
# ============================================================================

aps_add_module_library(device_camera
    DEPENDS
        data
        aps::imvsdk
)

# ----------------------------------------------------------------------------
#  APS_HAVE_IMVSDK：把 CMake 侧的查找结果导出为**编译期常量**
#
#  为什么需要它：ImvCameraBackend 在 SDK 缺失时必须走"诚实桩"分支
#  （每个方法明确失败），而在 SDK 存在时必须编译真实适配代码。
#  这两种代码路径无法靠链接期解决 —— 没有 SDK 时，任何对 SDK 符号的
#  引用都会链接失败，而条件编译需要一个编译期已知的宏。
#
#  为什么**总是**定义（缺省为 0）而不是"有则定义、无则不定义"：
#  若采用"有则定义"的写法，C++ 侧只能写成 `#ifdef APS_HAVE_IMVSDK`，
#  而 `#ifdef` 无法区分"SDK 没有"与"SOMEONE 拼错了宏名"——
#  后者会让真实适配代码被静默排除，构建成功但接入失败，
#  且没有任何提示。显式定义 0/1 使 `#if APS_HAVE_IMVSDK` 的两种取值
#  都有确定含义（配合 -Wundef 还能捕获拼写错误）。
#
#  为什么是 PUBLIC：本宏描述的是"本模块能否提供真实相机能力"这一事实，
#  上层（application 的装配代码）需要据此决定装配真实还是虚拟后端，
#  从而在启动日志中如实说明当前运行形态。
#
#  与 Dependencies.cmake 的 APS_HAVE_IMVSDK 变量**同源**：
#  此处只是把同一个事实转成编译期可见的形式，不存在第二个数据源。
# ----------------------------------------------------------------------------
#  if(TARGET) 守卫：aps_add_module_library 在本目录暂无 .cpp 时会
#  return() 而不创建目标（001 阶段确立的"空模块自动跳过"约定）。
#  无守卫时那种情况会让 configure 直接报错 —— 而"删掉源文件后
#  cmake 反而配置失败"是极难定位的：错误信息指向一个不存在的目标，
#  看起来像拼写错误。
if(TARGET device_camera)
    if(APS_HAVE_IMVSDK)
        target_compile_definitions(device_camera PUBLIC APS_HAVE_IMVSDK=1)
    else()
        target_compile_definitions(device_camera PUBLIC APS_HAVE_IMVSDK=0)
    endif()
endif()
