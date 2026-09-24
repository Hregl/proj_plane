# ============================================================================
#  src/device/turntable/CMakeLists.txt  →  device_turntable
#
#  依据：ENG-03 §14（Device 子模块 CMake：turntable 管理转台 SDK）
#        ENG-01 §8、SYS-06 §10~§13、SYS-10
#        ENG-08 §11（第一阶段只实现 ITurntableController + VirtualTurntable）
#
#  依赖：data + aps::turntablesdk（可选）
#
#  本子模块包含：
#    ITurntableController.h      —— 接口（SYS-06 §10.1）
#    VirtualTurntable.h/.cpp     —— 虚拟转台（SYS-06 §12 / 1.md Phase 4）
#    PekoTurntableController.*   —— Peko_D 真实适配（ENG-08 §11 明确暂不实现）
#
#  ⚠ 第一阶段（ENG-08 §11）不实现：Peko_D 协议 / RS485 / 网络控制。
#  因此本目录第一阶段只有接口与虚拟实现，aps::turntablesdk 预期为未找到，
#  链接时被 if(TARGET) 静默跳过——这不是缺失，是既定范围。
#
#  为什么第一阶段就建好这个目录与接口：SYS-08 的状态机在 ALIGN / STABILIZE
#  两个状态需要转台反馈才能推进（转台到位 → TurntableMotionState::STABLE）。
#  若等到 Phase 2 才定义接口，则状态机的这两个状态无法在软件闭环中验证，
#  而它们恰好是 SYS-08 §7 收敛性证明中耗时最长的环节（合计约 20 s）。
#  用虚拟转台把时序跑通，Phase 2 换成真实 SDK 只替换实现，不动状态机。
# ============================================================================

aps_add_module_library(device_turntable
    DEPENDS
        data
        aps::turntablesdk
)
