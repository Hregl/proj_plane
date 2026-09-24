# ============================================================================
#  src/device/trigger/CMakeLists.txt  →  device_trigger
#
#  依据：ENG-03 §14（Device 子模块 CMake：trigger 管理触发接口）
#        ENG-01 §8、SYS-06 §8~§9、ENG-08 §12
#
#  依赖：data（**无外部 SDK**）
#
#  本子模块包含：
#    ITriggerController.h            —— 接口（SYS-06 §9.1）
#    VirtualTriggerController.h/.cpp —— 软件触发（1.md Phase 4 / ENG-08 §12）
#    HardwareTriggerController.*     —— 硬触发（Phase 2）
#
#  ⚠ 与另两个子模块的关键差异：本目录**不链接任何厂商 SDK**。
#
#  硬触发在 A7A20MU201 上是通过相机自身的触发输入引脚（Line0 类）实现的，
#  控制入口就在相机 SDK 的触发模式设置里（SYS-06 §6.1 的
#  ICameraBackend::setTriggerMode）。因此：
#    · HardwareTriggerController 将来若要直接操作触发板卡，才需要引入新 SDK；
#    · 当前按 ENG-03 §14 的定位，本模块只负责"触发的时序与语义"
#      （enable / trigger / disable + 同步校验），物理输出交给相机 SDK。
#
#  这条分工决定了三相机同步的正确性归谁：
#    device_camera   保证"每台相机在收到硬触发后正确曝光并带出时间戳"；
#    device_trigger  保证"三台相机收到的是同一个触发脉冲"；
#    软件侧的时间戳检查与丢帧检测（SYS-06 §8）在 CameraSynchronizerWorker，
#    属 SYS-09 线程设计，不在本目录。
# ============================================================================

aps_add_module_library(device_trigger
    DEPENDS
        data
)
