# ============================================================================
#  src/runtime/CMakeLists.txt  →  libruntime
#
#  依据：ENG-01 §12（runtime 模块：计算后端 ComputeBackend / CpuRuntime /
#                    GpuRuntime）
#        1.md 目录树（src/runtime/）
#
#  依赖：data
#
#  ⚠ 本模块在两份基线文档间存在归属分歧，处理方式如下：
#
#    有：ENG-01 §4（src 模块表）、ENG-01 §12（模块定义）、1.md 目录树
#    无：ENG-03 §3（工程 CMake 结构图）、ENG-03 §12（无 §12.x runtime 小节）、
#        ENG-03 §12.9（app 链接表不含 runtime）、ENG-01 §17（依赖链不含 runtime）
#
#    取"有"：ENG-01 §1 声明自身为"CMake 组织 / 编码实现的工程基线"，
#    且 §4 与 §12 两处一致地列出本模块，分歧仅在 ENG-03 未同步。
#
#  **但 libruntime 当前不链入 app。** 理由：ENG-03 §12.9 冻结的链接表没有
#  它，且 ComputeBackend 目前无任何消费者——ENABLE_GPU 默认 OFF，V2.1
#  第一阶段无 GPU 算法实现。把无人调用的库链进交付物只会增大体积与
#  链接期不确定性。
#
#  何时需要修改：实现 ComputeBackend 并在算法模块中实际调用时，须同时
#  更新 ENG-03 §12.9 的链接表，再在 src/app/CMakeLists.txt 中补上 runtime。
#  不要"顺手"加上——那会让文档与构建脚本再次失同步，正是本轮初始化的
#  目的之一（消除文档间不一致）。
# ============================================================================

aps_add_module_library(runtime
    DEPENDS
        data
)
