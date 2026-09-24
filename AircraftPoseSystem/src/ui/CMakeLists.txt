# ============================================================================
#  src/ui/CMakeLists.txt  →  libui
#
#  依据：ENG-03 §12.8（生成 libui；依赖 Qt + application + preview）
#        ENG-01 §13（ui 模块：MainWindow / widgets / ImageViewer /
#                    PosePanel / StatusPanel / TurntablePanel）
#        1.md Phase 8（Qt 主窗口框架）
#
#  依赖：aps::qt5 + application + preview
#
#  ⚠ ENG-01 §18 第一条禁止依赖：**UI 不得访问 SDK**。
#  即 MainWindow 不出现 ImvSDK / PekoSdk 的任何头文件。本文件的依赖表
#  即为该禁令的构建期强制：libui 只链 Qt 与两个上层模块，物理上拿不到
#  device 层的真实后端符号，违反时直接链接失败。
#  这正是"依赖表即架构约束"的做法——靠代码评审记住禁令是不可靠的。
#
#  Qt 自动化工具开关只在本目录与 src/app/ 生效（普通变量，作用域为本目录
#  及其子目录）。不在根 CMakeLists 全局开启的理由：
#    AUTOMOC 会对每个目标的源文件做 Q_OBJECT 扫描，算法模块的源文件数量
#    多且不含 Qt 宏，全局开启纯属浪费；更糟的是它会给所有目标引入 Qt 的
#    包含路径，让"算法层是否偷偷用了 Qt"变得不可检查——而 algorithm 只
#    允许依赖 data + OpenCV（ENG-03 §12.5）。
# ============================================================================

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ⚠ RECURSE 是必需的，不是可选优化：
#   ui 的源文件分布在**三层**——MainWindow.cpp 在本目录，四个控件在
#   widgets/，两个工具在 utils/。aps_add_module_library 默认只收集本目录
#   的 *.cpp（见 src/CMakeLists.txt 的设计要点 (3)：保证源文件归属唯一），
#   不加 RECURSE 时 libui.a 里**只有 MainWindow.cpp**，四个控件的符号
#   一个都没有——而模块本身"非空"，所以也不会触发"暂无源文件，跳过生成"
#   的提示。结果是配置阶段一切正常，直到链接 AircraftPoseSystem 时
#   才出现 ImageViewer / PosePanel / … 的 undefined reference，
#   报错位置与真实原因（少了一个关键词）相距甚远。
#   这条在首次 cmake 时被实测踩到：输出为"模块 ui：1 个源文件"。
#
#   为何不用 4.4 的聚合方案：聚合是为了解决"根目录没有 .cpp 导致整个模块
#   被跳过"（device 的情形）。libui 不空（有 MainWindow.cpp），
#   故按 9.md §十二 的意图——**一个库装全部 UI 源文件**——用 RECURSE
#   一次收齐即可，无需再多一层子库。
aps_add_module_library(ui
    RECURSE
    DEPENDS
        application
        preview
        aps::qt5
)
