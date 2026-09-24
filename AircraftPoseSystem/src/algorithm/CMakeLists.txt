# ============================================================================
#  src/algorithm/CMakeLists.txt  →  libalgorithm
#
#  依据：ENG-03 §12.5（生成 libalgorithm；依赖 data + OpenCV）
#        ENG-03 §13（Algorithm 子模块 CMake）
#        ENG-01 §10 / §19（detection scale selection model feature matcher
#                          pose validation）
#        ENG-10（算法链接口与配置注入）
#
#  依赖：data + aps::opencv
#
#  ⚠ 关于 ENG-03 §13 的处理（本次刻意偏离，理由如下）：
#
#  §13 写"推荐：每个子模块独立 CMakeLists.txt"。本次（001）**未**创建这 8 个
#  子模块 CMakeLists.txt，而是让 libalgorithm 用 RECURSE 统一收集全部源文件。
#
#  理由是 §13 的"独立"只有在子模块**具有独立依赖**时才有意义：
#    · device 子模块必须独立 —— camera 链 ImvSDK、turntable 链 TurntableSDK、
#      trigger 两者都不链。这是真实的依赖差异，故 §14 的划分在此实现。
#    · algorithm 的 8 个子模块当前依赖完全一致（data + OpenCV）。拆成 8 个库
#      只增加 CMake 面积与链接顺序约束，不带来任何隔离。
#
#  何时改：当某个子模块引入独占依赖时（最可能是 detection 引入 YOLO 推理库，
#  如 ncnn / ONNXRuntime），届时创建该子模块的 CMakeLists.txt、去掉此处的
#  RECURSE、改为 add_subdirectory + 显式链接。**不要在无依赖差异时提前拆。**
#
#  ⚠ 本模块**刻意不依赖 infrastructure**（ENG-03 §12.5 只给 data + OpenCV）。
#  这条约束是 ENG-10 §5.1 配置注入设计的前提：
#    MeasurementSelector 需要读取历史匹配成功率统计表（M_hist），该表既要
#    被算法层读取、又必须由 infrastructure 落盘。若 algorithm 直接依赖
#    infrastructure，算法层就被绑死在文件系统上。
#  实际做法是：接口 IMatchStatsStore.h 定义在 data/（跨层接口下沉到依赖
#  最低公共层，同 C-07），实现 FileMatchStatsStore 在 infrastructure/，
#  由 app/ 在启动时注入。见 ENG-01 §11 的依赖分析图。
# ============================================================================

aps_add_module_library(algorithm
    RECURSE
    DEPENDS
        data
        aps::opencv
)
