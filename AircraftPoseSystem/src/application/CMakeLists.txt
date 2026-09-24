# ============================================================================
#  src/application/CMakeLists.txt  →  libapplication
#
#  依据：ENG-03 §12.6（生成 libapplication；MeasurementController /
#                       AlignmentController / StateMachine；
#                       依赖 device + preview + optical + algorithm）
#        ENG-01 §9（application 模块）、SYS-08（状态机设计）
#
#  依赖：device + preview + optical + algorithm
#
#  ⚠ **刻意不依赖 Qt**，与 ui 层的分工在此冻结。
#
#  SYS-08 的 12 状态机（IDLE → SEARCH → TARGET_FOUND → ALIGN → STABILIZE →
#  MEASURE_SELECT → CAPTURE → POSE_SOLVE → VALIDATE → SAVE → COMPLETE/FAILED）
#  是本项目的业务核心，也是 SYS-08 §7 有界性证明的对象。把它放在 Qt 之外，
#  才能：
#    · 用 GoogleTest 直接推状态，不必启动 QApplication；
#    · 注入恒失败的算法桩，在 2 s 的短 T_task 下验证"任务在 T_task 内进入
#      FAILED"（ENG-08 §9 的验收要求，7 个收敛性用例）；
#    · 让状态机在无 GUI 的离线设备上运行（ENG-03 §20 第 6 条）。
#
#  ⚠ 同样**刻意不依赖 infrastructure**（ENG-03 §12.6 的依赖表未含它）。
#  配置注入按 ENG-10 §5 由 app/ 在启动时完成：application 只接收已构造好的
#  MeasurementConfig 值对象，不自行读 yaml。这样 config/ 的字段变动不会
#  波及状态机，五元组基线中的"配置版本"（ENG-09 §6.8）也才能在任务级冻结
#  ——若状态机自己读文件，同一次任务的不同阶段可能读到不同版本的配置。
#
#  ⚠ 关于 RetryManager（ENG-08 §9 新增必做项）：
#  它属于本模块，与 StateMachine 同层。SYS-08 §7 冻结的重试策略
#  （每状态次数上限 / 每条回退边上限 / 全局 T_task 截止时间）是状态机
#  有界性的唯一保证；没有它，全部恢复路径退化为无限循环。
#  其 16 个配置字段来自 ENG-09 §6.5，由 app/ 注入。
# ============================================================================

aps_add_module_library(application
    DEPENDS
        device
        preview
        optical
        algorithm
)
