// ============================================================================
//  src/application/MeasurementStrategy.cpp
//
//  依据：SYS-08 §5（各状态的失败/成功去向）、§6（正常流程）、§7.2（失败三分类）、
//        §7.3（次数表 + 升级规则）、§7.7（恢复路径汇总，冻结表）
//
//  本文件的每条分支都能在 SYS-08 的表格里找到对应行，对应关系写在各 case 的
//  注释中。**没有任何一条来自推理之外的补充**；三处文档未覆盖的情形
//  （TARGET_FOUND 失败、CAPTURE 次数用尽、MEASURE_SELECT/SAVE 次数用尽）
//  以"取最保守的有界处置"处理并逐条注明。
// ============================================================================

#include "application/MeasurementStrategy.h"

#include <algorithm>
#include <string>
#include <utility>

namespace aircraft
{
namespace application
{

namespace
{

using data::MeasurementState;

const char* stateName(MeasurementState state)
{
    switch (state)
    {
    case MeasurementState::IDLE:           return "IDLE";
    case MeasurementState::SEARCH:         return "SEARCH";
    case MeasurementState::TARGET_FOUND:   return "TARGET_FOUND";
    case MeasurementState::ALIGN:          return "ALIGN";
    case MeasurementState::STABILIZE:      return "STABILIZE";
    case MeasurementState::MEASURE_SELECT: return "MEASURE_SELECT";
    case MeasurementState::CAPTURE:        return "CAPTURE";
    case MeasurementState::POSE_SOLVE:     return "POSE_SOLVE";
    case MeasurementState::VALIDATE:       return "VALIDATE";
    case MeasurementState::SAVE:           return "SAVE";
    case MeasurementState::COMPLETE:       return "COMPLETE";
    case MeasurementState::FAILED:         return "FAILED";
    }
    return "UNKNOWN_STATE";
}

Recovery makeFail(data::ErrorInfo error)
{
    Recovery r;
    r.action = RecoveryAction::FAIL;
    r.target = MeasurementState::FAILED;
    r.error = std::move(error);
    return r;
}

Recovery makeRetry()
{
    Recovery r;
    r.action = RecoveryAction::RETRY_IN_STATE;
    return r;
}

Recovery makeRollback(MeasurementState target, data::ErrorInfo error)
{
    Recovery r;
    r.action = RecoveryAction::ROLLBACK;
    r.target = target;
    r.error = std::move(error);
    return r;
}

}  // namespace

// ---------------------------------------------------------------------------
// 正常流程（SYS-08 §6）
// ---------------------------------------------------------------------------

data::MeasurementState
MeasurementStrategy::nextState(data::MeasurementState current) const
{
    switch (current)
    {
    case MeasurementState::IDLE:           return MeasurementState::SEARCH;
    case MeasurementState::SEARCH:         return MeasurementState::TARGET_FOUND;
    case MeasurementState::TARGET_FOUND:   return MeasurementState::ALIGN;
    case MeasurementState::ALIGN:          return MeasurementState::STABILIZE;
    case MeasurementState::STABILIZE:      return MeasurementState::MEASURE_SELECT;
    case MeasurementState::MEASURE_SELECT: return MeasurementState::CAPTURE;
    case MeasurementState::CAPTURE:        return MeasurementState::POSE_SOLVE;
    case MeasurementState::POSE_SOLVE:     return MeasurementState::VALIDATE;
    case MeasurementState::VALIDATE:       return MeasurementState::SAVE;
    case MeasurementState::SAVE:           return MeasurementState::COMPLETE;

    // 终止态没有下一步。返回自身而不是 FAILED（7.md §七 的实现返回 FAILED，
    // 会把一次已成功的测量推成失败）。终止判定请用 isTerminal()。
    case MeasurementState::COMPLETE:
    case MeasurementState::FAILED:
        return current;
    }

    return current;
}

bool MeasurementStrategy::isTerminal(data::MeasurementState state) const
{
    return state == MeasurementState::COMPLETE
        || state == MeasurementState::FAILED;
}

// ---------------------------------------------------------------------------
// 一级决策：瞬态重试 / 硬件不重试 / 能力边界不重试（§7.2）
// ---------------------------------------------------------------------------

Recovery MeasurementStrategy::recoveryFor(
    data::MeasurementState state,
    FailureKind kind,
    const data::ErrorInfo& deviceError) const
{
    // ---- 硬件故障：不重试（§7.2）----
    //
    // §7.2 原文的理由：对硬件故障重试只会耗尽时限后仍然失败，把"设备坏了"
    // 这个明确结论推迟到 60 秒之后，同时掩盖真实原因。
    //
    // ⚠ 单台相机故障**不走这里**：§7.5 要求"降级继续"而非失败，
    // 判定发生在采集之后（MeasurementController 检查可用相机数）。
    // 只有"可用相机数 ≤1"才以 1001 落到本分支 —— 此时设备层已在
    // deviceError 中填好了 1001。
    if (kind == FailureKind::HARDWARE)
    {
        data::ErrorInfo e = deviceError;
        if (e.code == 0)
        {
            // ⚠ 裁决 C-006 改写了本分支：原先是"如实置 0 并在消息中说明
            // 状态，不新造码（9000 段已占用 9001/9002/9003）"。该理由在
            // 9004 登记之后不再成立，而置 0 的后果是**具体的**：本分支是
            // FAIL 动作，e 会经 applyRecovery → failWith 成为终态错误，
            // 于是一台因硬件故障而失败的测量，其终态码是 0 —— 而 0 的
            // 冻结语义是"未设置"、渲染为 "OK"。
            //
            // 归 9004 而非某个设备码：设备层**没有给出**码，我们不知道
            // 是哪一类硬件故障，凭状态名猜一个（比如 CAPTURE 就报 1001）
            // 会把现场引向一个未经证实的根因。
            e.code = data::kErrStateFailure;
            e.message = std::string("硬件故障（") + stateName(state)
                + "），设备层未上报错误码";
        }
        else if (e.message.empty())
        {
            e.message = std::string("硬件故障（") + stateName(state) + "）";
        }
        return makeFail(std::move(e));
    }

    // ---- 能力边界：不重试（§7.2）----
    //
    // 典型实例：超出转台行程（2002）、超出全部焦段覆盖、距离超出 40~300 m。
    // 码同样来自检测方（转台越程由 AlignmentController 判定并置 2002）。
    if (kind == FailureKind::CAPABILITY)
    {
        data::ErrorInfo e = deviceError;
        if (e.code == 0)
        {
            // 同 HARDWARE 分支：本分支也是 FAIL 动作，置 0 会直接成为
            // 终态码。归 9004 而不是 2002 —— "能力边界"有好几种，
            // 检测方没说是哪一种，套用转台越程的码就是伪造根因。
            e.code = data::kErrStateFailure;
            e.message = std::string("能力边界（") + stateName(state)
                + "），超出系统物理能力，不重试";
        }
        return makeFail(std::move(e));
    }

    // ---- 瞬态失败：按 §7.7 的动作列处置 ----
    switch (state)
    {
    case MeasurementState::SEARCH:
        // §7.7 "搜索失败 | 瞬态 | 保持 SEARCH | 无限（受 T_task）"。
        return makeRetry();

    case MeasurementState::TARGET_FOUND:
        // §7.7 无该行，§5.3 也未给失败路径 —— 恢复路径汇总表里"没有行"
        // 就是"没有经批准的恢复动作"。
        //
        // ⚠ 因此这里**不能**取"回退至 SEARCH"，尽管它看起来更宽容：
        // §7.3 给 TARGET_FOUND 的次数上限是 1，而 §7.6 约束 2 要求"每次
        // **进入**状态都报一次尝试"、约束 5 要求"计数器不因进入新状态而
        // 清零"。两条合起来的后果是：一旦发生过一次 TARGET_FOUND，就再也
        // 进不去这个状态 —— 回退到 SEARCH 之后，SEARCH 唯一的前进边
        // （SEARCH → TARGET_FOUND）会被次数上限永久拒绝，任务只能在
        // SEARCH 里空转到 T_task，最后报一个**指向错误方向**的 9001，
        // 真正的起因（那一帧算不出 TargetOffset）反而看不到。
        // 与其绕一圈丢掉 57 秒再报错，不如就地失败。
        //
        // ⚠ 若将来有裁决允许 TARGET_FOUND 在回退后重新进入
        //（例如明确"§7.3 的上限 1 只约束状态内重试，不约束回退后的再次
        // 进入"），此处应改回 ROLLBACK 至 SEARCH 并同步恢复
        // StateMachine 的 TARGET_FOUND → SEARCH 边。该缺口已登记 README §6。
        //
        // ⚠ `deviceError` 的 message 必须**带上**（M1 实测发现的缺陷）：
        //    原先这里构造的是一个全新的 ErrorInfo，把调用方报上来的原因
        //    整个丢掉了。后果是现场只看到"TARGET_FOUND 没有恢复路径"
        //    这句**规则说明**，而真正的原因（到底是"检测框退化"还是
        //    "尺度估计失败"）永远查不到 —— 一个是检出质量问题，
        //    另一个是机型库缺失，处置方式完全不同。
        //    规则说明是"为什么不能重试"，原始原因是"坏了什么"，
        //    两句都要有，且原始原因在前。
        return makeFail(data::ErrorInfo{
            // ⚠ 传递原始码；但若调用方未给出码（0 = "未设置"），必须落到
            // 兜底码而不是把 0 传下去 —— 0 在 result.json 中渲染为 "OK"，
            // 会让这条**确实失败**的路径在机读层面看起来是成功的
            //（C-006 的验收判据正是"FAILED 任务的包里不再出现 code = 0"）。
            deviceError.code == 0 ? data::kErrStateFailure : deviceError.code,
            (deviceError.message.empty()
                 ? std::string("本次检测结果不可用（未给出具体原因）")
                 : deviceError.message) +
                "；TARGET_FOUND 无经批准的恢复路径（§7.7 无该行，"
                "§7.3 上限 1 不允许回退后重新进入），就地失败",
            deviceError.timestampNs});

    case MeasurementState::ALIGN:
        // §5.4 "失败：继续调整" + §7.7 "对准失败 | 瞬态 | 重新检测 +
        // 重算 TurntableCommand | 8 | FAILED(2003)"。
        // 8 次的用尽由 onAttemptsExhausted() 处置。
        return makeRetry();

    case MeasurementState::STABILIZE:
        // §7.7 "稳定失败 | 瞬态 | 重做一次稳定等待 | 1 | 回退至 ALIGN"。
        // §7.3 的次数上限为 1，故"重做一次"实际表现为
        // 次数耗尽后回退 ALIGN 再重新稳定（见 onAttemptsExhausted）。
        return makeRetry();

    case MeasurementState::MEASURE_SELECT:
        // §7.3 "候选相机集合为空 → 3"。集合为空可能是瞬时的
        //（例如某一帧三路图像同时无效），故允许状态内重试。
        return makeRetry();

    case MeasurementState::CAPTURE:
        // §7.3 "采集失败或有效帧数不足 → 3"。重采即 §7.3 升级规则第 1 条。
        return makeRetry();

    case MeasurementState::POSE_SOLVE:
        // §5.8 "失败：进入 FAILED 或重新 CAPTURE"；
        // §7.7 "PnP 失败 | 瞬态 | 重采 + 按 §7.3 升级规则 | 2 | 回退 MEASURE_SELECT"。
        // 2 次内的重试**必须换输入**（升级规则），升级动作见 escalationFor()。
        return makeRetry();

    case MeasurementState::VALIDATE:
        // §7.7 "验证失败 | 瞬态 | 回退 MEASURE_SELECT，排除当前相机 | 2（同边）| FAILED"。
        // ⚠ 该回退动作**每次都必须排除上一次失败的相机**（§5.9），
        // 由 MeasurementController 依据 isExcluded()/excludeCamera() 落实。
        //
        // ⚠ code 保持 0（裁决 C-006 点名）：回退是**非错误的处置动作**，
        // 不是失败。本条记录的价值在 message（"排除了哪台相机"这一事实），
        // 而不是错误分类。给回退也塞一个码，会污染 FailureTrace::history，
        // 使"哪些迁移由失败触发"这一判据失效。
        return makeRollback(MeasurementState::MEASURE_SELECT, data::ErrorInfo{
            0,
            "验证不通过，回退 MEASURE_SELECT 并排除当前相机",
            0});

    case MeasurementState::SAVE:
        // §7.3 "落盘失败 → 3"。磁盘瞬满/占用是可恢复的瞬时状态。
        return makeRetry();

    case MeasurementState::IDLE:
    case MeasurementState::COMPLETE:
    case MeasurementState::FAILED:
        // 这些状态不产生"失败"：IDLE 尚未开始，两个终止态已结束。
        // 走到这里说明调用方搞错了状态，如实失败并写明状态名。
        //
        // 为何无法归类：这是**调用方违约**（本状态不应上报失败），
        // 不属于任何设备/标定/模型/算法的故障类别，故用兜底码 9004。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            std::string("在 ") + stateName(state) + " 状态下收到失败上报（该状态不应产生失败）",
            0});
    }

    // 为何无法归类：状态名本身越出了本例举，调用方传入了未登记的枚举值。
    return makeFail(data::ErrorInfo{data::kErrStateFailure, "未知状态，无法给出恢复方案", 0});
}

// ---------------------------------------------------------------------------
// 二级决策：次数用尽（§7.7 的"超限后果"列）
// ---------------------------------------------------------------------------

Recovery MeasurementStrategy::onAttemptsExhausted(
    data::MeasurementState state) const
{
    switch (state)
    {
    case MeasurementState::ALIGN:
        // §7.7："对准失败 | 8 | FAILED（code 2003）"。
        // §5.4 同义："超限进入 FAILED（ErrorInfo{code=2003}）"。
        // ⚠ 2003 的**值**由 RetryManager 在拒绝时写入（它持有次数表），
        // 此处只声明动作；码在此重复一份就会与 §11 约束 6 冲突。
        return makeFail(data::ErrorInfo{
            data::kErrAlignRetryExhausted,
            "对准尝试次数用尽，目标未能进入 ±50 pixel 判据范围",
            0});

    case MeasurementState::STABILIZE:
        // §7.7："稳定失败 | 1 | 回退至 ALIGN"。
        // code 保持 0：回退是处置动作而非失败，理由同 recoveryFor() 的
        // VALIDATE 分支。
        return makeRollback(MeasurementState::ALIGN, data::ErrorInfo{
            0,
            "稳定等待未满足连续 N≥3 帧，回退 ALIGN 重新对准",
            0});

    case MeasurementState::POSE_SOLVE:
        // §7.7："PnP 失败 | 2 | 回退 MEASURE_SELECT"。
        // 6001 是该事件的登记码（ENG-09 §5.27），此处带上以便日志可检索。
        return makeRollback(MeasurementState::MEASURE_SELECT, data::ErrorInfo{
            data::kErrPnpRetryExhausted,
            "PnP 尝试次数用尽，回退 MEASURE_SELECT 更换输入",
            0});

    case MeasurementState::CAPTURE:
        // §7.7 无 CAPTURE 行。取"回退 MEASURE_SELECT"：连续 3 次采不到
        // 有效帧，说明当前焦段不可用，换一个候选通道是唯一有意义的动作
        //（§7.3 升级规则第 2 条"换用次优相机"的落地方式）。
        // 该回退受 §7.4 预算约束（同边 2 次、总计 4 次），故有界。
        // code 保持 0：回退是处置动作而非失败，理由同上。
        return makeRollback(MeasurementState::MEASURE_SELECT, data::ErrorInfo{
            0,
            "CAPTURE 次数用尽，当前焦段未产出有效帧，回退更换通道",
            0});

    case MeasurementState::TARGET_FOUND:
        // 上限 1：用尽即首次失败，与 recoveryFor() 的处置一致（就地失败），
        // 理由见 recoveryFor() 中该分支的注释 —— 回退 SEARCH 是条死路。
        //
        // 为何无法归类：§7.7 未给该状态任何恢复路径，也就没有任何对应的
        // 登记码（ENG-09 §5.27 表按"故障类别"分段，而这是**规则缺口**
        // 导致的失败，不是某类设备的故障）。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            "TARGET_FOUND 唯一一次机会已用尽（§7.7 未给该状态恢复路径）",
            0});

    case MeasurementState::SEARCH:
        // §7.3：SEARCH **不设次数上限**，其终止只可能来自 T_task。
        // 因此本分支按 §7.7 的"任务时限到 → FAILED"处置，码取 9001。
        return makeFail(data::ErrorInfo{
            data::kErrTaskTimeout,
            "搜索阶段未检出目标且已到任务时限",
            0});

    case MeasurementState::MEASURE_SELECT:
        // §7.7 无该行。候选集合连续 3 次无法产出可用通道，且更早的状态
        // 同样无相机可选，故失败。
        //
        // ⚠ 消息不写成"候选集合为空"：次数用尽**不等于**集合为空。
        // 该状态被重复进入（例如 VALIDATE 反复回退到此）时，
        // 次数会在集合始终非空的情况下用尽 —— 把原因写成"集合为空"
        // 会把现场排查引向一个不存在的事实。
        //
        // 为何无法归类：失败原因取决于前序哪条路径把它推进来，
        // 本状态自身看不到（见上句），故不能冒充某个更具体的码。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            "MEASURE_SELECT 尝试次数已用尽（§7.3 上限 3），未能产出可用测量通道",
            0});

    case MeasurementState::SAVE:
        // §7.7 无该行。落盘连续 3 次失败，无更早的状态能解决，故失败。
        //
        // 为何无法归类：ENG-09 §5.27 无"存储/IO"段。落盘失败的直接原因是
        // 磁盘状态（满/无权限），它既不是设备故障也不是算法故障；
        // 若日后该场景变得常见，应当为该类单开一个段，而不是继续用 9004。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            "SAVE 尝试次数已用尽（§7.3 上限 3），测量结果包未能落盘",
            0});

    case MeasurementState::VALIDATE:
        // §7.7："验证失败 | 2（同边）| FAILED"。
        //
        // ⚠ 该分支与 §7.4 的回退边上限（同为 2）同时到达时的**先到者**：
        // 进入 VALIDATE 时 transition() 已先记过一次尝试（§7.6 约束 2），
        // 故"VALIDATE 的次数用尽"总比"VALIDATE→MEASURE_SELECT 这条边的
        // 次数用尽"先一步发生，于是实际得到的**不是 9002**。
        // 这是 §7.3（上限 2）与 §7.4（同边 2）取同一数值的必然后果，
        // 不是实现取舍 —— 已记入 README §6 的偏离登记。
        //
        // 为何无法归类：该分支的语义是"两次验证都没过"，而验证不通过的
        // **具体原因**（重投影超限 / 内点率不足 / 置信度不足 / 非有限值）
        // 在 PoseValidator 里已判定但未向上传递。若能归入 6000 段的
        // "验证失败"码，应改用它 —— 目前 §5.27 无此码，故用兜底 9004。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            "VALIDATE 尝试次数已用尽（§7.3 上限 2），验证连续不通过",
            0});

    case MeasurementState::IDLE:
    case MeasurementState::COMPLETE:
    case MeasurementState::FAILED:
        // 为何无法归类：同 recoveryFor() 的对应分支，调用方违约。
        return makeFail(data::ErrorInfo{
            data::kErrStateFailure,
            std::string("终止/空闲状态 ") + stateName(state) + " 不应出现次数用尽",
            0});
    }

    return makeFail(data::ErrorInfo{data::kErrStateFailure, "未知状态，无法给出超限处置", 0});
}

// ---------------------------------------------------------------------------
// §7.3 升级规则
// ---------------------------------------------------------------------------

Escalation MeasurementStrategy::escalationFor(int attempt) const
{
    Escalation e;

    // 第 1 次（首次进入）也要"重新采集"：从 SEARCH 到 CAPTURE 之间转台已
    // 移动、光照与目标姿态都可能变化，SEARCH 时的那一帧不能当作测量帧。
    e.refetchFrames = true;

    if (attempt <= 1)
    {
        return e;
    }

    // §7.3 第 2 行：重新采集 + 换用 MeasurementSelector 的次优相机。
    // §5.8 亦明文："第 2 次必须换用次优相机"。
    e.switchCamera = true;

    // §7.3 第 3 行：重新采集 + 放宽算法内部阈值（RANSAC 迭代数、ratio 阈值）。
    // 该行未再要求换相机，故 switchCamera 保持 true 仅表示"仍处在上一步
    // 换过的相机上"，不表示再换一次 —— 是否还能换由调用方的排除集决定。
    if (attempt >= 3)
    {
        e.relaxThresholds = true;
    }

    return e;
}

// ---------------------------------------------------------------------------
// 候选相机可及集合
// ---------------------------------------------------------------------------

void MeasurementStrategy::excludeCamera(data::CameraRole role)
{
    if (isExcluded(role))
    {
        return;   // 幂等：重复排除不应让集合出现重复元素
    }
    excluded_.push_back(role);
}

bool MeasurementStrategy::isExcluded(data::CameraRole role) const
{
    return std::find(excluded_.begin(), excluded_.end(), role) != excluded_.end();
}

std::vector<data::CameraRole> MeasurementStrategy::allowedCameras() const
{
    // 顺序固定为枚举声明次序（CAM25 / CAM50 / CAM100），**不代表优劣**：
    // 候选评分与排序是 MeasurementSelector 的职责（SYS-14 §6）。
    // 此处只回答"哪些焦段允许参与选择"。
    const data::CameraRole kAll[3] = {
        data::CameraRole::CAM25,
        data::CameraRole::CAM50,
        data::CameraRole::CAM100,
    };

    std::vector<data::CameraRole> out;
    out.reserve(3);
    for (data::CameraRole role : kAll)
    {
        if (!isExcluded(role))
        {
            out.push_back(role);
        }
    }
    return out;
}

void MeasurementStrategy::resetExclusions()
{
    excluded_.clear();
}

}  // namespace application
}  // namespace aircraft
