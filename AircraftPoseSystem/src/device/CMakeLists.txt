# ============================================================================
#  src/device/CMakeLists.txt  →  libdevice
#
#  依据：ENG-03 §12.3（生成 libdevice；包含 camera/turntable/trigger；
#                      依赖 data + optical + ImvSDK(optional) + TurntableSDK(optional)）
#        ENG-03 §14（Device 子模块 CMake：分别管理三类 SDK）
#        ENG-01 §8（device 模块）、SYS-06（设备抽象层设计）
#
#  依赖：data + optical + 三个子模块目标 + 两个可选 SDK
#
#  本模块是 ENG-03 §13/§14 中**唯一真正需要子模块独立 CMakeLists** 的地方：
#    camera    → 需要 ImvSDK（真实相机 A7A20MU201）
#    turntable → 需要 TurntableSdk（Peko_D 协议，Phase 2）
#    trigger   → 两者都不需要（硬触发走相机 SDK 的触发接口本身）
#  这三者的外部依赖确实不同，因此按 §14 拆分为 device_camera /
#  device_turntable / device_trigger 三个静态库，再由 libdevice 聚合。
#
#  这样做的实际收益（不是形式上的整齐）：
#    · 无 SDK 的机器上，device_turntable / device_trigger 照常构建，
#      只有 device_camera 的真实后端被条件编译排除——工程整体仍可编译；
#    · 链接期能精确看出"是哪个设备的 SDK 没接上"，而不是一个笼统的
#      libdevice 链接失败。
#
#  SYS-06 §4.2 / C-18 提醒：OpticalRig 不在本模块。本模块只**消费**
#  optical 层提供的标定数据，不定义它。
# ============================================================================

# ----------------------------------------------------------------------------
# 1 子模块（顺序无关，彼此不依赖）
# ----------------------------------------------------------------------------
add_subdirectory(camera)
add_subdirectory(turntable)
add_subdirectory(trigger)

# ----------------------------------------------------------------------------
# 2 聚合库
#
# 子模块目标全部通过 if(TARGET) 由 aps_add_module_library 过滤：
# 001 阶段三者都还没有源文件，会全部跳过，libdevice 也随即被跳过——
# 这是预期行为，不是配置错误。
#
# aps::imvsdk / aps::turntablesdk 仅在 Dependencies.cmake 实际找到时才存在。
# 当前开发机两者都没有，故此处静默不链接，由 Dependencies.cmake 统一
# 打印"未找到（非致命）"的说明，避免同一件事报两遍。
# ----------------------------------------------------------------------------
aps_add_module_library(device
    DEPENDS
        data
        optical
        device_camera
        device_turntable
        device_trigger
        aps::imvsdk
        aps::turntablesdk
)

# ----------------------------------------------------------------------------
# 3 聚合目标
#
# 本目录**没有** .cpp（源文件全在 camera/ turntable/ trigger/ 三个子目录，
# 由 ENG-03 §14 要求拆分为独立库），因此上面的 aps_add_module_library
# 会按"空模块自动跳过"规则 return()，`device` 目标不会存在。
#
# 那会导致一个静默缺陷：application 写 `DEPENDS device` 时被 if(TARGET)
# 过滤掉，链接期才以 undefined reference 暴露，而提示信息说
# "依赖 device 尚未生成"——把排查引向"device 还没写"这一错误方向。
#
# 故此处补建 INTERFACE 聚合目标。若将来本目录直接加入 .cpp，
# aps_add_module_library 会创建实体库，本调用随即自动跳过（见函数内守卫）。
# ----------------------------------------------------------------------------
aps_add_aggregate_module(device
    DEPENDS
        data
        optical
        device_camera
        device_turntable
        device_trigger
)
