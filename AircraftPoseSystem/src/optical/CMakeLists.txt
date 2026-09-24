# ============================================================================
#  src/optical/CMakeLists.txt  →  liboptical
#
#  依据：ENG-03 §12.2（生成 liboptical；OpticalRig / CalibrationManager /
#                      CoordinateTransformer；依赖 data）
#        ENG-01 §6（optical 模块）、§19（新增 optical/rig）
#        C-18（OpticalRig 归属 optical 层，不是 Device 层）
#
#  依赖：data
#
#  C-18 冻结的语义边界——本模块**不含任何设备控制能力**：
#    OpticalRig = 静态描述（三相机通道注册表 + 光机标定数据 + 只读查询）
#    设备控制（连接、取流、触发、转动）全部在 device/。
#
#  这个边界直接决定了 ENG-01 §17 依赖链的方向：optical 在 device 之下，
#  device 依赖 optical。若把 OpticalRig 挪回 Device 层，device 就会同时
#  是"标定数据的生产者"和"标定数据的消费者"，SYS-11 的标定流程将无法
#  独立于相机在线状态运行（离线标定复算、Golden 数据回放都会失效）。
#
#  CoordinateTransformer 承担 SYS-13 / SYS-05 §11.4 的坐标链合成：
#      aircraftToShip = rigToShip · cameraToRig · aircraftToCamera
#  它是唯一允许做该合成的地方——坐标链若在多处各写一遍，方向约定
#  （AToB 表示 A→B）迟早会在某一处被写反，且不会有编译错误。
# ============================================================================

aps_add_module_library(optical
    DEPENDS
        data
)
