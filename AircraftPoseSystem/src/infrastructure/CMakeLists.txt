# ============================================================================
#  src/infrastructure/CMakeLists.txt  →  libinfrastructure
#
#  依据：ENG-03 §12.7（生成 libinfrastructure；Logger / ConfigManager / Recorder）
#        ENG-01 §11（infrastructure 模块：logger / config / recorder /
#                    performance / persistence）
#
#  依赖：data
#
#  ⚠ ENG-03 §12.7 未声明依赖，此处按 ENG-01 §17 的冻结依赖链补 data：
#    Logger 要序列化 ImageFrame / ShipPoseResult 字段，Recorder 要落盘
#    MeasurementTask，无 data 无法编译。§12.7 是遗漏而非有意为之。
#
#  本模块的特殊职责（ENG-01 §11 / C-21）：
#  infrastructure 是**唯一**允许做文件 I/O 的层。ENG-10 §5.1 的
#  IMatchStatsStore 接口放在 data/（因为它的实现与被消费者跨层），
#  实现 FileMatchStatsStore 落在这里。
#  这一分工不是洁癖：算法层若自行读写文件，则无法在单元测试中替换为
#  内存实现，Golden 数据回放（ENG-06 §10）也就无从构造确定性输入。
#
#  ⚠ RECURSE 是必需的，不是可选项：
#    aps_add_module_library 默认只 GLOB 本目录的直接子文件（sources/*.cpp），
#    而本模块的源码按 ENG-01 §11 分布在 logger/ config/ recorder/
#    persistence/ 四个子目录中，本目录下**没有**任何 .cpp。
#    漏掉 RECURSE 的后果与 008 的 libui 完全相同：目标能生成、能链接，
#    但里面一个目标文件都没有 —— 链接期报"未定义符号"，
#    而错误信息不会提到 CMake，极易误判为代码问题。
# ============================================================================

aps_add_module_library(infrastructure
    RECURSE
    DEPENDS
        data
)
