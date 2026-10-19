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
#    ITurntableController.h          —— 接口（SYS-06 §10.1）
#    VirtualTurntable.h/.cpp         —— 虚拟转台（SYS-06 §12 / 1.md Phase 4）
#    PelcoDFrame.h/.cpp              —— Pelco-D 纯编解码（C-22）
#    IPelcoTransport.h               —— 传输 seam（设备层内部，不进冻结类清单）
#    UdpPelcoTransport.h/.cpp        —— POSIX UDP 实现
#    PelcoDSession.h/.cpp            —— 协议/状态机（无线程）
#    PelcoDTurntableController.*     —— ITurntableController 真实后端 + I/O 线程
#
#  ⚠ 历史说明：第一阶段（ENG-08 §11）曾明确不实现 Peko_D/RS485/网络控制，
#  因此当时只有接口与虚拟实现，aps::turntablesdk 预期为未找到并静默跳过。
#  V2.7 / C-22 起 Pelco-D over UDP 已接入；RS485 与通用 network 后端仍未实现。
#  aps::turntablesdk 仍是可选依赖：Pelco-D 后端不使用厂商 SDK。
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
