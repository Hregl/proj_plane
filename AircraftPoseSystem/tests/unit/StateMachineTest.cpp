// ============================================================================
//  tests/unit/StateMachineTest.cpp
//
//  覆盖：ENG-06 §5.2 要求的三类测试 —— 正常转换 / 非法转换 / 异常恢复 ——
//        以及 SYS-08 §7.1/§7.3/§7.4/§7.6/§11 的全部判据。
//
//  被测对象：application::StateMachine、application::RetryManager、
//            application::MeasurementStrategy（三者同层，且状态机的行为
//            无法脱离 RetryManager 定义 —— 每次进入状态都要向它报一次尝试，
//            所以三个类在同一个测试目标里验证，不拆成三个文件）。
//
//  SYS-08 §10 的 7 个收敛性用例属于**集成**层（需要注入式设备桩），
//  按 ENG-06 的分层在 tests/integration/MeasurementFlowTest.cpp。
//  本文件只覆盖不依赖设备的部分。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "application/MeasurementStrategy.h"
#include "application/RetryManager.h"
#include "application/StateMachine.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"

using aircraft::application::FailureKind;
using aircraft::application::MeasurementStrategy;
using aircraft::application::Recovery;
using aircraft::application::RecoveryAction;
using aircraft::application::RetryManager;
using aircraft::application::StateMachine;
using aircraft::data::ErrorInfo;
using aircraft::data::MeasurementConfig;
using aircraft::data::MeasurementState;
using S = aircraft::data::MeasurementState;

namespace
{

/// §6 冻结的正常流程。
const S kForwardChain[11] = {
    S::IDLE, S::SEARCH, S::TARGET_FOUND, S::ALIGN, S::STABILIZE,
    S::MEASURE_SELECT, S::CAPTURE, S::POSE_SOLVE, S::VALIDATE, S::SAVE,
    S::COMPLETE,
};

MeasurementConfig shortTaskConfig(uint64_t taskNs = 60000000000ULL)
{
    MeasurementConfig c;
    c.taskTimeoutNs = taskNs;
    return c;
}

}  // namespace

// ===========================================================================
//  1 正常转换（ENG-06 §5.2 第一类）
// ===========================================================================

TEST(StateMachineTest, 正常流程逐步推进到COMPLETE)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    EXPECT_EQ(sm.state(), S::IDLE);

    for (int i = 1; i < 11; ++i)
    {
        const S next = kForwardChain[i];
        EXPECT_TRUE(sm.transition(next, 0)) << "第 " << i << " 步转换被拒";
        EXPECT_EQ(sm.state(), next);
    }

    EXPECT_EQ(sm.state(), S::COMPLETE);

    // §5.1：COMPLETE 之后可以开始新任务（回到 IDLE）。
    EXPECT_TRUE(sm.transition(S::IDLE, 0));
    EXPECT_EQ(sm.state(), S::IDLE);
}

TEST(StateMachineTest, 每次进入状态都向RetryManager报一次尝试)
{
    // SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2。若不满足，各状态的次数上限就失去意义。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    sm.transition(S::SEARCH, 0);
    EXPECT_EQ(retry.attempts(S::SEARCH), 1);

    sm.transition(S::TARGET_FOUND, 0);
    EXPECT_EQ(retry.attempts(S::TARGET_FOUND), 1);

    sm.transition(S::ALIGN, 0);
    EXPECT_EQ(retry.attempts(S::ALIGN), 1);
}

// ===========================================================================
//  2 非法转换（ENG-06 §5.2 第二类）
// ===========================================================================

TEST(StateMachineTest, 跳状态被拒绝且不改变当前状态)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    EXPECT_FALSE(sm.transition(S::CAPTURE, 0));   // IDLE 不能直接到 CAPTURE
    EXPECT_EQ(sm.state(), S::IDLE);

    sm.transition(S::SEARCH, 0);
    EXPECT_FALSE(sm.transition(S::POSE_SOLVE, 0));  // SEARCH 不能跳过对准
    EXPECT_EQ(sm.state(), S::SEARCH);
}

TEST(StateMachineTest, 自转换被拒绝)
{
    // 同状态重试必须走 beginAttempt，不能走 transition —— 否则 ALIGN 的
    // 8 次上限会被"进入 1 次 + 自转换 7 次"耗尽，而真正的对准动作只做 4 次。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    sm.transition(S::SEARCH, 0);
    const int before = retry.attempts(S::SEARCH);

    EXPECT_FALSE(sm.transition(S::SEARCH, 0));
    EXPECT_EQ(sm.state(), S::SEARCH);
    EXPECT_EQ(retry.attempts(S::SEARCH), before) << "被拒的自转换不应消耗尝试次数";
}

TEST(StateMachineTest, COMPLETE不得转为FAILED)
{
    // 任务已成功终止后再置失败，会让 result.json 与状态记录互相矛盾。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    for (int i = 1; i < 11; ++i) { sm.transition(kForwardChain[i], 0); }
    ASSERT_EQ(sm.state(), S::COMPLETE);

    EXPECT_FALSE(sm.transition(S::FAILED, 0));
    EXPECT_EQ(sm.state(), S::COMPLETE);
}

TEST(StateMachineTest, 所有状态都可进入FAILED除COMPLETE外)
{
    // §7.7〔引用无效·依据待裁决·见 Q-D2〕 的恢复路径要求任意失败点都能落到 FAILED（COMPLETE 除外）。
    const S all[12] = {
        S::IDLE, S::SEARCH, S::TARGET_FOUND, S::ALIGN, S::STABILIZE,
        S::MEASURE_SELECT, S::CAPTURE, S::POSE_SOLVE, S::VALIDATE, S::SAVE,
        S::COMPLETE, S::FAILED,
    };

    for (S from : all)
    {
        const bool legal = StateMachine::isLegalTransition(from, S::FAILED);
        if (from == S::COMPLETE)
        {
            EXPECT_FALSE(legal) << "COMPLETE → FAILED 必须被禁止";
        }
        else if (from == S::FAILED)
        {
            EXPECT_FALSE(legal);   // 自转换
        }
        else
        {
            EXPECT_TRUE(legal) << "状态 " << static_cast<int>(from) << " 应能进入 FAILED";
        }
    }
}

// ===========================================================================
//  3 异常恢复（ENG-06 §5.2 第三类）
// ===========================================================================

// ⚠ 为什么用 CAPTURE → MEASURE_SELECT 这条边做"每边上限"的用例：
//
// §7.4 的每边上限是 2，而 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2 让"进入"也算一次尝试。于是"同一条边
// 试到第 3 次"要求**边的起点状态能被进入 ≥3 次**。查 §7.3〔引用无效·依据待裁决·见 Q-D2〕：TARGET_FOUND=1、
// STABILIZE=1、POSE_SOLVE=2、VALIDATE=2，只有 CAPTURE=3。
// 也就是说，**整个状态图里只有 CAPTURE → MEASURE_SELECT 这一条边能真正
// 走到"每边预算用尽"**；其余回退边的第 3 次尝试会先被起点状态的次数上限
// 拦下，看到的是该状态的超限码而不是 9002。
// （这正是 SYS-08 §10 用例 3/4 拿不到 9002 的原因，已在
//  tests/integration/MeasurementFlowTest.cpp 中逐 tick 记录，并登记 README §6。）
TEST(StateMachineTest, 回退转换须经beginRollback批准且受每边上限2约束)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    // 走到 CAPTURE（MEASURE_SELECT 的第 1 次进入）。
    sm.transition(S::SEARCH, 0);
    sm.transition(S::TARGET_FOUND, 0);
    sm.transition(S::ALIGN, 0);
    sm.transition(S::STABILIZE, 0);
    sm.transition(S::MEASURE_SELECT, 0);
    EXPECT_TRUE(sm.transition(S::CAPTURE, 0));

    // §7.4：同一回退边上限 2。
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0)) << "第 1 次同边回退应被批准";
    EXPECT_EQ(retry.rollbackCount(), 1);

    EXPECT_TRUE(sm.transition(S::CAPTURE, 0));
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0)) << "第 2 次同边回退应被批准";
    EXPECT_EQ(retry.rollbackCount(), 2);

    // 第 3 次：同一回退边的预算已用尽 —— 此时 CAPTURE 与 MEASURE_SELECT
    // 各自都还有进入次数（各用了 3 次中的 3 次，见下），故唯一能拦住它的
    // 就是每边预算本身。
    EXPECT_TRUE(sm.transition(S::CAPTURE, 0));
    const int msBefore = retry.attempts(S::MEASURE_SELECT);
    EXPECT_FALSE(sm.transition(S::MEASURE_SELECT, 0)) << "第 3 次同边回退必须被拒";
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrRollbackExhausted);
    EXPECT_EQ(sm.state(), S::CAPTURE) << "被拒的回退不得改变状态";

    // 被拒的回退同样不得消耗目标状态的尝试次数（同下一条用例的判据，
    // 在此顺带覆盖：一次"回退被拒"不应同时吃掉两份配额）。
    EXPECT_EQ(retry.attempts(S::MEASURE_SELECT), msBefore);

    // ⚠ MEASURE_SELECT 与 CAPTURE 各已用掉 3 次进入（上限即 3），
    // 故此后任何需要再次进入它们的转换都会被次数上限拦下 —— 这正是
    // 上面注释所说的"第 3 次尝试撞到的是别的上限"的现场。
    EXPECT_EQ(retry.attempts(S::MEASURE_SELECT), 3);
    EXPECT_EQ(retry.attempts(S::CAPTURE), 3);
}

TEST(StateMachineTest, 回退总预算用尽后任何回退都被拒)
{
    // §7.4 的总预算（默认 4）与每边预算（默认 2）是**两个独立的闸门**，
    // 取先到者。本用例把总预算压到 2、每边保持 2，于是两条**不同**的边
    // 各走一次就用光了总预算，第 3 条边还没碰过自己那份配额就被总预算拦下。
    MeasurementConfig cfg = shortTaskConfig();
    cfg.maxRollbackTotal = 2;
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    // ⚠ 链条的选择同样受"进入即计数"约束：STABILIZE 的上限是 1，
    // 所以一旦从 STABILIZE 回退出去就再也回不来（不能用它构造第二条边）。
    // 本用例只用 CAPTURE 与 POSE_SOLVE 这两条上限为 3/2 的通道。
    sm.transition(S::SEARCH, 0);
    sm.transition(S::TARGET_FOUND, 0);
    sm.transition(S::ALIGN, 0);
    sm.transition(S::STABILIZE, 0);
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0));
    ASSERT_TRUE(sm.transition(S::CAPTURE, 0));

    // 边 1：CAPTURE → MEASURE_SELECT（总预算 1/2）
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0));
    EXPECT_EQ(retry.rollbackCount(), 1);

    ASSERT_TRUE(sm.transition(S::CAPTURE, 0));
    ASSERT_TRUE(sm.transition(S::POSE_SOLVE, 0));

    // 边 2：POSE_SOLVE → CAPTURE（与边 1 不同，自己那份每边配额一次未用）
    ASSERT_TRUE(sm.transition(S::CAPTURE, 0));
    EXPECT_EQ(retry.rollbackCount(), 2);

    ASSERT_TRUE(sm.transition(S::POSE_SOLVE, 0));

    // 边 3：POSE_SOLVE → MEASURE_SELECT。这条边一次都没走过，
    // 它自己那份每边配额（2）也完好，但总预算已尽 —— 拦下它的是总预算。
    EXPECT_FALSE(sm.transition(S::MEASURE_SELECT, 0)) << "总预算用尽后任何回退都必须被拒";
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrRollbackExhausted);
    EXPECT_EQ(sm.state(), S::POSE_SOLVE);
}

TEST(StateMachineTest, 向前的转换不消耗回退预算)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    for (int i = 1; i < 11; ++i) { sm.transition(kForwardChain[i], 0); }
    EXPECT_EQ(retry.rollbackCount(), 0) << "正常流程不应消耗回退预算";
}

TEST(StateMachineTest, 被拒的回退不消耗目标状态的尝试次数)
{
    // StateMachine::transition 的顺序是先 beginRollback 再 beginAttempt。
    // 若顺序相反，一次被拒的回退会同时吃掉"回退预算"与"目标状态次数"两份配额，
    // §7.3 与 §7.4 之间的算术关系（37.8 s < 60 s）就失去意义。
    //
    // ⚠ 该顺序有一个**已知的副作用**（登记 README §6）：回退被批准、但随后
    // 目标状态的进入被次数上限拒绝时，回退预算已经扣掉了，而状态并没有变。
    // 之所以仍然选择这个顺序：§7.6〔引用无效·依据待裁决·见 Q-D2〕 冻结的接口只有 beginRollback/beginAttempt
    // 这一对"检查即提交"的方法，没有 canRollback/canAttempt 这样的纯查询，
    // 无法做真正的两段式提交；在两种不精确之间，选"多扣一次回退预算"
    //（回退预算更小、更该省着用，多扣会让系统更早收敛）而不是
    //"多扣一次目标状态的次数"（会让正常路径提前失败）。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    StateMachine sm(retry);
    retry.beginTask(0);

    sm.transition(S::SEARCH, 0);
    sm.transition(S::TARGET_FOUND, 0);
    sm.transition(S::ALIGN, 0);
    sm.transition(S::STABILIZE, 0);
    sm.transition(S::MEASURE_SELECT, 0);
    sm.transition(S::CAPTURE, 0);

    const int before = retry.attempts(S::MEASURE_SELECT);
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0));   // 第 1 次同边回退成功
    EXPECT_EQ(retry.attempts(S::MEASURE_SELECT), before + 1);  // 目标状态次数 +1

    sm.transition(S::CAPTURE, 0);
    ASSERT_TRUE(sm.transition(S::MEASURE_SELECT, 0));   // 第 2 次
    sm.transition(S::CAPTURE, 0);

    const int usedBefore = retry.attempts(S::MEASURE_SELECT);
    EXPECT_FALSE(sm.transition(S::MEASURE_SELECT, 0));  // 第 3 次：每边预算已尽
    EXPECT_EQ(retry.attempts(S::MEASURE_SELECT), usedBefore)
        << "被拒的回退不得消耗目标状态的尝试次数";
}

// ===========================================================================
//  4 RetryManager：§7.1 / §7.3 的次数与时限
// ===========================================================================

TEST(RetryManagerTest, SEARCH不设次数上限但受任务时限约束)
{
    MeasurementConfig cfg = shortTaskConfig(2000000000ULL);   // T_task = 2 s
    RetryManager retry(cfg);
    retry.beginTask(0);

    for (int i = 0; i < 10000; ++i)
    {
        ASSERT_TRUE(retry.beginAttempt(S::SEARCH, 1000)) << "第 " << i << " 次被拒";
    }
    EXPECT_EQ(retry.attempts(S::SEARCH), 10000) << "计数仍应累加（供界面上报进度）";

    // 时限一到即拒（§7.1 的 L3）。
    EXPECT_FALSE(retry.beginAttempt(S::SEARCH, 2000000000ULL));
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrTaskTimeout);
}

TEST(RetryManagerTest, 各状态次数上限与SYS08_7_3一致)
{
    struct Case { S state; int cap; };
    const Case kCases[6] = {
        {S::TARGET_FOUND,   1},
        {S::ALIGN,          8},
        {S::STABILIZE,      1},
        {S::MEASURE_SELECT, 3},
        {S::CAPTURE,        3},
        {S::POSE_SOLVE,     2},
    };

    for (const Case& c : kCases)
    {
        MeasurementConfig cfg = shortTaskConfig();
        RetryManager retry(cfg);
        retry.beginTask(0);

        for (int i = 0; i < c.cap; ++i)
        {
            EXPECT_TRUE(retry.beginAttempt(c.state, 0))
                << "状态 " << static_cast<int>(c.state) << " 第 " << (i + 1) << " 次应放行";
        }
        EXPECT_FALSE(retry.beginAttempt(c.state, 0))
            << "状态 " << static_cast<int>(c.state) << " 第 " << (c.cap + 1) << " 次应被拒";
        EXPECT_EQ(retry.attempts(c.state), c.cap);
    }
}

TEST(RetryManagerTest, VALIDATE与SAVE的上限来自配置)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    retry.beginTask(0);

    for (int i = 0; i < cfg.maxValidateAttempts; ++i)
    {
        EXPECT_TRUE(retry.beginAttempt(S::VALIDATE, 0));
    }
    EXPECT_FALSE(retry.beginAttempt(S::VALIDATE, 0));

    for (int i = 0; i < cfg.maxSaveAttempts; ++i)
    {
        EXPECT_TRUE(retry.beginAttempt(S::SAVE, 0));
    }
    EXPECT_FALSE(retry.beginAttempt(S::SAVE, 0));
}

TEST(RetryManagerTest, 超限时给出该状态的登记错误码)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    retry.beginTask(0);

    // ENG-09 §5.27 已登记的：ALIGN → 2003、POSE_SOLVE → 6001。
    for (int i = 0; i < cfg.maxAlignAttempts; ++i) { retry.beginAttempt(S::ALIGN, 0); }
    EXPECT_FALSE(retry.beginAttempt(S::ALIGN, 0));
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrAlignRetryExhausted);

    for (int i = 0; i < cfg.maxSolveAttempts; ++i) { retry.beginAttempt(S::POSE_SOLVE, 0); }
    EXPECT_FALSE(retry.beginAttempt(S::POSE_SOLVE, 0));
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrPnpRetryExhausted);
}

TEST(RetryManagerTest, 无登记码的状态如实置0且消息写明状态名)
{
    // 不新造错误码（ENG-09 §2.27 的变更流程），但必须让故障可定位。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    retry.beginTask(0);

    for (int i = 0; i < cfg.maxCaptureAttempts; ++i) { retry.beginAttempt(S::CAPTURE, 0); }
    EXPECT_FALSE(retry.beginAttempt(S::CAPTURE, 0));
    EXPECT_EQ(retry.lastError().code, 0);
    EXPECT_FALSE(retry.lastError().message.empty());
    EXPECT_TRUE(retry.lastError().message.find("CAPTURE") != std::string::npos)
        << "消息须含状态名，否则无法定位：" << retry.lastError().message;
}

TEST(RetryManagerTest, 各状态计数独立且不因进入新状态清零)
{
    // §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 5：计数只在 reset()/beginTask() 时清零。
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    retry.beginTask(0);

    retry.beginAttempt(S::ALIGN, 0);
    retry.beginAttempt(S::ALIGN, 0);
    retry.beginAttempt(S::CAPTURE, 0);

    EXPECT_EQ(retry.attempts(S::ALIGN), 2);
    EXPECT_EQ(retry.attempts(S::CAPTURE), 1);
    EXPECT_EQ(retry.attempts(S::SEARCH), 0);
}

TEST(RetryManagerTest, 时限优先于次数上限)
{
    // SYS-08 §10 用例 7：T_task 是硬保证，次数上限只是效率手段。
    MeasurementConfig cfg = shortTaskConfig(1000000000ULL);   // T_task = 1 s
    RetryManager retry(cfg);
    retry.beginTask(0);

    // 令 CAPTURE 的上限与时限在同一时刻同时可达。
    for (int i = 0; i < cfg.maxCaptureAttempts; ++i)
    {
        ASSERT_TRUE(retry.beginAttempt(S::CAPTURE, 500000000ULL));
    }

    EXPECT_FALSE(retry.beginAttempt(S::CAPTURE, 1000000000ULL));
    EXPECT_EQ(retry.lastError().code, aircraft::data::kErrTaskTimeout)
        << "两者同时成立时必须报 9001，而不是该状态的超限码";
}

TEST(RetryManagerTest, reset与beginTask清零全部计数)
{
    MeasurementConfig cfg = shortTaskConfig();
    RetryManager retry(cfg);
    retry.beginTask(0);

    retry.beginAttempt(S::ALIGN, 0);
    retry.beginRollback(S::VALIDATE, S::MEASURE_SELECT, 0);

    retry.reset();
    EXPECT_EQ(retry.attempts(S::ALIGN), 0);
    EXPECT_EQ(retry.rollbackCount(), 0);
    EXPECT_EQ(retry.lastError().code, 0);
    EXPECT_FALSE(retry.deadlineExceeded(1ULL << 62)) << "无活动任务时不存在超时";

    retry.beginTask(100);
    EXPECT_EQ(retry.attempts(S::ALIGN), 0) << "beginTask 必须内部清零";
    EXPECT_FALSE(retry.deadlineExceeded(100 + cfg.taskTimeoutNs - 1));
    EXPECT_TRUE(retry.deadlineExceeded(100 + cfg.taskTimeoutNs));
}

TEST(RetryManagerTest, 极大的任务时限不应饱和回绕)
{
    MeasurementConfig cfg = shortTaskConfig(~0ULL - 10);
    RetryManager retry(cfg);
    retry.beginTask(1000);   // 若直接相加会回绕成"一启动即超时"

    EXPECT_FALSE(retry.deadlineExceeded(1000));
    EXPECT_FALSE(retry.deadlineExceeded(1ULL << 62));
}

TEST(RetryManagerTest, 配置中把上限写成0表示一次都不放行)
{
    // 0 曾是"无限"的哨兵值，后果是误配变成静默无限重试（只在 T_task 到点时
    // 以无指向性的 9001 结束）。改为 -1 哨兵后 0 恢复字面语义。
    MeasurementConfig cfg = shortTaskConfig();
    cfg.maxCaptureAttempts = 0;
    RetryManager retry(cfg);
    retry.beginTask(0);

    EXPECT_FALSE(retry.beginAttempt(S::CAPTURE, 0));
}

// ===========================================================================
//  5 MeasurementStrategy：§7.2 / §7.3 / §7.7
// ===========================================================================

TEST(MeasurementStrategyTest, 失败三分类的处置完全不同)
{
    MeasurementStrategy strategy;

    // 瞬态 → 允许重试。
    EXPECT_EQ(strategy.recoveryFor(S::ALIGN, FailureKind::TRANSIENT, ErrorInfo{}).action,
              RecoveryAction::RETRY_IN_STATE);

    // 硬件故障 → 不重试，且错误码来自设备层。
    const Recovery hw = strategy.recoveryFor(
        S::ALIGN, FailureKind::HARDWARE,
        ErrorInfo{aircraft::data::kErrCameraInsufficient, "相机不足", 0});
    EXPECT_EQ(hw.action, RecoveryAction::FAIL);
    EXPECT_EQ(hw.error.code, aircraft::data::kErrCameraInsufficient);

    // 能力边界 → 不重试（越程 2002 是典型实例）。
    const Recovery cap = strategy.recoveryFor(
        S::ALIGN, FailureKind::CAPABILITY,
        ErrorInfo{aircraft::data::kErrTurntableOverTravel, "越程", 0});
    EXPECT_EQ(cap.action, RecoveryAction::FAIL);
    EXPECT_EQ(cap.error.code, aircraft::data::kErrTurntableOverTravel);
}

TEST(MeasurementStrategyTest, 硬件故障无码时按状态名给出可定位消息)
{
    MeasurementStrategy strategy;
    const Recovery r = strategy.recoveryFor(S::CAPTURE, FailureKind::HARDWARE, ErrorInfo{});
    EXPECT_EQ(r.action, RecoveryAction::FAIL);

    // ⚠ 本行原为 `== 0`，附言"不得新造错误码"。裁决 C-006 改写了判决：
    // 该分支是 FAIL 动作，`r.error` 会经 applyRecovery → failWith 直接成为
    // **终态错误**，置 0 等于让一台因硬件故障而失败的测量在 result.json 里
    // 报 "OK"。9004 不是"新造码"，是 C-006 登记的兜底码；而设备层没给出
    // 具体类别时，9004 正是它的正确用法。
    EXPECT_EQ(r.error.code, aircraft::data::kErrStateFailure)
        << "设备层未给码时应用兜底码，不得把 0 当失败码传下去";
    EXPECT_TRUE(r.error.message.find("CAPTURE") != std::string::npos);
}

TEST(MeasurementStrategyTest, 超限后果逐状态与SYS08_7_7一致)
{
    MeasurementStrategy strategy;

    EXPECT_EQ(strategy.onAttemptsExhausted(S::ALIGN).action, RecoveryAction::FAIL);
    EXPECT_EQ(strategy.onAttemptsExhausted(S::ALIGN).error.code,
              aircraft::data::kErrAlignRetryExhausted);

    EXPECT_EQ(strategy.onAttemptsExhausted(S::STABILIZE).action, RecoveryAction::ROLLBACK);
    EXPECT_EQ(strategy.onAttemptsExhausted(S::STABILIZE).target, S::ALIGN);

    EXPECT_EQ(strategy.onAttemptsExhausted(S::POSE_SOLVE).action, RecoveryAction::ROLLBACK);
    EXPECT_EQ(strategy.onAttemptsExhausted(S::POSE_SOLVE).target, S::MEASURE_SELECT);

    EXPECT_EQ(strategy.onAttemptsExhausted(S::SEARCH).action, RecoveryAction::FAIL);
    EXPECT_EQ(strategy.onAttemptsExhausted(S::SEARCH).error.code,
              aircraft::data::kErrTaskTimeout);
}

TEST(MeasurementStrategyTest, 验证失败的回退目标是重新选择通道)
{
    MeasurementStrategy strategy;
    const Recovery r = strategy.recoveryFor(S::VALIDATE, FailureKind::TRANSIENT, ErrorInfo{});
    EXPECT_EQ(r.action, RecoveryAction::ROLLBACK);
    EXPECT_EQ(r.target, S::MEASURE_SELECT);
}

TEST(MeasurementStrategyTest, 升级规则要求第2次换相机第3次放宽阈值)
{
    MeasurementStrategy strategy;

    const aircraft::application::Escalation e1 = strategy.escalationFor(1);
    EXPECT_TRUE(e1.refetchFrames);
    EXPECT_FALSE(e1.switchCamera);
    EXPECT_FALSE(e1.relaxThresholds);

    const aircraft::application::Escalation e2 = strategy.escalationFor(2);
    EXPECT_TRUE(e2.refetchFrames);
    EXPECT_TRUE(e2.switchCamera);
    EXPECT_FALSE(e2.relaxThresholds);

    const aircraft::application::Escalation e3 = strategy.escalationFor(3);
    EXPECT_TRUE(e3.refetchFrames);
    EXPECT_TRUE(e3.switchCamera);
    EXPECT_TRUE(e3.relaxThresholds);
}

TEST(MeasurementStrategyTest, 排除集幂等且可清零)
{
    MeasurementStrategy strategy;

    EXPECT_EQ(strategy.allowedCameras().size(), 3u);

    strategy.excludeCamera(aircraft::data::CameraRole::CAM50);
    strategy.excludeCamera(aircraft::data::CameraRole::CAM50);   // 幂等
    EXPECT_TRUE(strategy.isExcluded(aircraft::data::CameraRole::CAM50));
    EXPECT_EQ(strategy.allowedCameras().size(), 2u);

    strategy.excludeCamera(aircraft::data::CameraRole::CAM25);
    strategy.excludeCamera(aircraft::data::CameraRole::CAM100);
    EXPECT_TRUE(strategy.allowedCameras().empty()) << "三台都排除后候选为空（§7.4 的依据）";

    strategy.resetExclusions();
    EXPECT_EQ(strategy.allowedCameras().size(), 3u);
}

TEST(MeasurementStrategyTest, 终止态没有下一步也不会被推成失败)
{
    MeasurementStrategy strategy;

    EXPECT_EQ(strategy.nextState(S::COMPLETE), S::COMPLETE);
    EXPECT_EQ(strategy.nextState(S::FAILED), S::FAILED);
    EXPECT_TRUE(strategy.isTerminal(S::COMPLETE));
    EXPECT_TRUE(strategy.isTerminal(S::FAILED));
    EXPECT_FALSE(strategy.isTerminal(S::SAVE));

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(strategy.nextState(kForwardChain[i]), kForwardChain[i + 1]);
    }
}
