# ============================================================================
#  tests/unit/CMakeLists.txt
#
#  依据：ENG-06 §5.2（StateMachine 测试：正常转换 / 非法转换 / 异常恢复）、
#        ENG-06 §2/§3（测试分层：unit = 纯逻辑单测）、ENG-01 §15、
#        ENG-08 §13、SYS-08 §7（重试与超时的判据来源）、
#        ENG-03 §15（推荐 GoogleTest）
#
#  依赖：data + application
#
#  ⚠ 本目录的测试**不依赖 Qt**：application 模块刻意不链接 Qt（ENG-03 §12.6），
#  故状态机、重试管理器、测量策略可以在没有 QApplication、没有事件循环的
#  条件下被直接推导。这正是把状态机放在 application 而不是 ui 的收益之一。
#
#  ⚠ 源文件显式列举，不用 GLOB —— 见 tests/CMakeLists.txt 中的约定说明。
# ============================================================================

aps_add_test(test_state_machine
    SOURCES
        StateMachineTest.cpp
    DEPENDS
        data
        application
)
