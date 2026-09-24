// ============================================================================
//  src/application/MeasurementController.cpp
//
//  依据：SYS-08 §5（各状态详细设计）、§6（正常流程）、§7（重试与超时）、
//        §8（AUTO 模式的状态→焦段映射）、§9（运行于 Application 线程）、
//        §11（约束）
//        ENG-09 §2.4（像素坐标原点为图像中心）、§2.5（CLOCK_MONOTONIC）、
//               §5.27（错误码，禁止裸字面量）、§6.8（配置任务级冻结）
//        ENG-10 §5.3（算法阈值注入）
//
//  ⚠ 本文件的每个决策都能在 SYS-08 找到对应条目，对应关系写在各处注释中。
//  文档未覆盖的少数情形以"取最保守的有界处置"处理并逐条标注，
//  从不新造状态；错误码一律取 ENG-09 §5.27 的登记值（含裁决 C-006 新增的
//  4001 / 5001 / 9004 / 9005），从不使用裸数字字面量。
//
//  ⚠ 关于 kErrStateFailure（9004）的使用纪律（裁决 C-006 引入）：
//  它是**兜底码**，用于"确实无法归入任何更具体码"的失败。本文件此前把这些
//  失败一律记为裸 0，而 0 的冻结语义是"未设置"、在 result.json 中渲染为
//  "OK" —— 于是一台确实失败了的测量，其终态错误码在机读层面与成功无法区分。
//
//  归类优先级（自上而下，命中即止）：
//    ① 设备段 / 算法段已有登记码 → 用该码（1001/1002/2001/2002/3002/4001/5001/6001）
//    ② 有明确的上游码可传递 → 原样传递（见 handleFailure 的 deviceError）
//    ③ 以上都不成立 → 9004，且**必须**能逐条说清"为何归不进 ①②"
//  每个 9004 站点附近的注释即是这个"为何"。
//  ⚠ 单次 tick() 的**工作量上界**（§11 约束 9 的"必然到达终止态"依赖它）：
//  每个状态最多消耗一次尝试、推进一次转换、发起至多一次设备调用。
//  CAPTURE 是唯一有循环的状态，其循环次数由 captureFrameCount 界定。
//  没有任何状态会在 tick() 内部等待 —— 等待一律表现为"本次 tick 提前返回"。
// ============================================================================

#include "application/MeasurementController.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "data/MonotonicClock.h"

namespace aircraft
{
namespace application
{

namespace
{

using data::MeasurementState;

/// STABILIZE 的连续稳定帧下限（SYS-08 §5.5："建议 连续稳定帧 N >= 3"）。
/// 该值不由 MeasurementConfig 提供 —— ENG-09 §6.5 的重试/超时表里没有它，
/// 它是 §5.5 的判据常量而非可调参数。
constexpr int kStableFramesRequired = 3;

/// `MultiCameraFrame` 的通道取用（ENG-09 §5.5 的三个具名字段）。
const data::ImageFrame& frameOf(const data::MultiCameraFrame& frame,
                                data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM50:  return frame.cam50;
    case data::CameraRole::CAM100: return frame.cam100;
    case data::CameraRole::CAM25:  break;
    }
    return frame.cam25;
}

/// 通道名（日志用）。
const char* roleName(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return "CAM25";
    case data::CameraRole::CAM50:  return "CAM50";
    case data::CameraRole::CAM100: return "CAM100";
    }
    return "UNKNOWN_ROLE";
}

}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

MeasurementController::MeasurementController(
    device::IMultiCameraManager& cameras,
    device::ITurntableController& turntable,
    algorithm::IPosePipeline& pipeline,
    const optical::OpticalRig& rig,
    const data::MeasurementConfig& measurementCfg,
    const data::TurntableConfig& turntableCfg,
    preview::PreviewManager* preview,
    device::ITriggerController* trigger,
    IRecorderSink* recorder,
    bool modelsAvailable)
    : cameras_(cameras)
    , turntable_(turntable)
    , pipeline_(pipeline)
    , rig_(rig)
    , preview_(preview)
    , trigger_(trigger)
    , recorder_(recorder)
    , measurementConfig_(measurementCfg)
    , turntableConfig_(turntableCfg)
    , retry_(measurementCfg)
    , stateMachine_(retry_)
    , strategy_()
    , alignment_(turntableCfg)
    // ⚠ 放在列表末尾以匹配成员**声明**顺序（modelsAvailable_ 声明在
    // 后半段的降级状态附近）。C++ 的初始化顺序只认声明顺序，写反了
    // 会得到 -Wreorder；把它放在这里既无警告，也让"它不参与流程决策、
    // 只影响失败如何记"这一性质在视觉上也落在尾部。
    , modelsAvailable_(modelsAvailable)
{
    task_.state = MeasurementState::IDLE;

    // 统计量推送通道（裁决 C-002）。
    //
    // 在构造里登记而不是在 startMeasurement() 里：观察关系是**本对象与
    // pipeline_ 之间**的，与"是否正在测量"无关。若等到 startMeasurement()
    // 才登记，就存在一个"pipeline_ 会回调、而接收方尚未注册"的窗口 ——
    // 那时统计量会被静默丢弃（`statisticsObserver_` 为空即直接返回），
    // 且丢在哪一次任务取决于调用顺序，最难查。
    //
    // 本工程只装配一个 MeasurementController（ENG-01 §14 的单一装配点），
    // 故"覆盖已有观察者"不会误伤第二个接收方。
    pipeline_.setStatisticsObserver(this);

    // 配置一致性检查：只记录、不阻断启动。
    //
    // §7.1 要求 T_task > T_state > T_device。D 层时限（T_device）由设备层
    // 自持（ENG-09 §6.4），本类看不到，故只能校核前两级。
    // 若某状态超时 > T_task，该状态专属错误码（如 2003）将**永远无法产生**，
    // 任务会先撞上 9001 —— 那会让现场拿到"超时"这个没有指向性的结论。
    // 这属于配置错误而非运行错误，009 阶段的 ConfigManager 应在启动时
    // 直接拒绝（ENG-10 §5.3："取值超范围则启动失败"）；此处如实记录，
    // 以便无 ConfigManager 的离线运行也能看见问题。
    struct StateTimeout
    {
        MeasurementState state;
        uint64_t         ns;
    };
    const StateTimeout kTimeouts[9] = {
        {MeasurementState::SEARCH,         measurementConfig_.searchTimeoutNs},
        {MeasurementState::TARGET_FOUND,   measurementConfig_.targetFoundTimeoutNs},
        {MeasurementState::ALIGN,          measurementConfig_.alignTimeoutNs},
        {MeasurementState::STABILIZE,      measurementConfig_.stabilizeTimeoutNs},
        {MeasurementState::MEASURE_SELECT, measurementConfig_.selectTimeoutNs},
        {MeasurementState::CAPTURE,        measurementConfig_.captureTimeoutNs},
        {MeasurementState::POSE_SOLVE,     measurementConfig_.solveTimeoutNs},
        {MeasurementState::VALIDATE,       measurementConfig_.validateTimeoutNs},
        {MeasurementState::SAVE,           measurementConfig_.saveTimeoutNs},
    };

    for (const StateTimeout& t : kTimeouts)
    {
        if (t.ns > measurementConfig_.taskTimeoutNs)
        {
            note(std::string("配置异常：状态 ") + std::to_string(static_cast<int>(t.state))
                 + " 的超时大于 T_task，该状态的专属错误码将无法产生（§7.1）");
        }
    }

    if (measurementConfig_.captureFrameCount < 5
        || measurementConfig_.captureFrameCount > 10)
    {
        note("配置异常：captureFrameCount = "
             + std::to_string(measurementConfig_.captureFrameCount)
             + " 超出 ENG-09 §6.5 的约定范围 5~10");
    }

    if (measurementConfig_.taskTimeoutNs == 0)
    {
        note("配置异常：taskTimeoutNs = 0，任何任务都会在第一次 tick 即超时");
    }
}

MeasurementController::~MeasurementController()
{
    // 注销观察者。`pipeline_` 的生命周期长于本对象（app 装配），
    // 留着一个悬垂指针会让崩溃发生在算法层，而根因在本类的析构。
    // 见 MeasurementController.h 的析构声明。
    pipeline_.setStatisticsObserver(nullptr);
}

// ---------------------------------------------------------------------------
// algorithm::IPipelineObserver
// ---------------------------------------------------------------------------

void MeasurementController::onStatistics(
    const data::MeasurementStatistics& statistics)
{
    statistics_         = statistics;
    statisticsReceived_ = true;
}

data::MeasurementStatistics MeasurementController::statistics() const
{
    return statistics_;
}

bool MeasurementController::statisticsReceived() const
{
    return statisticsReceived_;
}

// ---------------------------------------------------------------------------
// SYS-04 §4.5 冻结的 UI 接口
// ---------------------------------------------------------------------------

void MeasurementController::startMeasurement()
{
    startMeasurement(data::monotonicNowNs());
}

void MeasurementController::startMeasurement(uint64_t nowNs)
{
    const MeasurementState s = stateMachine_.state();

    if (s != MeasurementState::IDLE && !finished())
    {
        // 重复点击"开始"不应打断进行中的任务：一次测量要动转台、
        // 要落盘，中途重启会让上一次的 result.json 与日志互相矛盾。
        note("已有测量任务在进行中（" + std::to_string(static_cast<int>(s))
             + "），忽略本次启动请求");
        return;
    }

    // ① 清零重试计数并启动 T_task 计时（§7.6：RetryManager 是唯一的记账者）。
    //
    // ⚠ 这一步必须排在"回到 IDLE"**之前**，顺序颠倒会让第二次测量永远
    //    无法开始 —— 且只在操作者隔了一会儿才点第二次时发作：
    //
    //    `stateMachine_.transition()` 会对**目标状态**上报一次尝试
    //    （§7.6 约束 2），而 `RetryManager::beginAttempt()` 的第一件事就是
    //    查 T_task（§7.1 的硬保证）。若此时窗口仍是**上一次任务**的
    //    （beginTask 尚未调用），那么上一次任务的 deadline 一旦过去，
    //    `COMPLETE → IDLE` 这条唯一入口就被它自己的超时判据关死；
    //    而 beginTask 恰恰排在它后面 —— 于是每次点击都在同一步返回，
    //    计数永不重置，**系统再也开不出第二次测量**。
    //    实测表现（tests/integration/MeasurementFlowTest.cpp 的
    //    "连续两次测量的曝光序号单调递增"用例）：第二次启动的
    //    run() 返回 0 个 tick，notices 里只有一条
    //    "无法回到 IDLE：任务时限 T_task 已到，不再允许任何重试"，
    //    而 state() 仍是 COMPLETE —— 看起来像"第二次测量瞬间完成"。
    //
    //    放在前面也不影响首次启动：s == IDLE 时本就要 beginTask。
    //    `reset()` 清掉的是**上一次任务**的计数，而此刻那一次已经结束
    //    （上面已排除"任务进行中"的情形），故不存在清掉在用预算的可能。
    retry_.beginTask(nowNs);

    // ② 回到 IDLE。§5.1："IDLE 的进入条件之一是上次任务结束"。
    //    COMPLETE → IDLE 与 FAILED → IDLE 都在 §6/§7.7 的转换表内。
    if (s != MeasurementState::IDLE
        && !stateMachine_.transition(MeasurementState::IDLE, nowNs))
    {
        note("无法回到 IDLE：" + stateMachine_.lastError().message);
        return;
    }

    // ③ 任务级状态复位。strategy_.resetExclusions() 是必须的 ——
    //    排除集若跨任务残留，第二次测量会在"少一台相机"的条件下开始（§5.9）。
    strategy_.resetExclusions();
    task_ = data::MeasurementTask{};
    // taskId 同时是结果目录名（ENG-09 §6.8）。用注入的单调时刻派生，
    // 使任务标识可复现（§11 约束 5：单次任务可完整回放）；
    // 若需要日历可读的目录名，那是 Recorder 的命名职责（SYS-09 §12）。
    task_.taskId = "measurement_" + std::to_string(nowNs);

    lastFrame_ = data::MultiCameraFrame{};
    pendingDetection_ = data::DetectionResult{};
    lastOffset_ = data::TargetOffset{};
    bestQuality_ = data::ImageQuality{};
    poseResult_ = data::ShipPoseResult{};
    validationResult_ = data::PoseValidationResult{};
    selectedCamera_ = data::CameraRole::CAM25;
    selection_ = data::MeasurementSelectionResult{};
    capturedFrames_.clear();
    bestFrame_ = data::MultiCameraFrame{};
    bestAcqIndex_ = -1;

    // 统计量必须随任务清零 —— 包括 `statisticsReceived_`。
    // 只清 `statistics_` 是不够的：上一次任务的"收到过统计量"会让
    // 本次任务的记录里带着上一轮的统计数字，而**看起来完全正常**
    // （见 MeasurementController.h 中该字段的说明）。
    statistics_         = data::MeasurementStatistics{};
    statisticsReceived_ = false;

    degraded_ = false;
    degradationNotice_ = data::ErrorInfo{};
    availableCameras_ = 0;
    lastError_ = data::ErrorInfo{};
    notices_.clear();

    // 失败轨迹必须随任务清零（裁决 C-007）。
    //
    // ⚠ 不清零的后果比"记录不准"更严重：第二次测量的 FailureTrace 会以
    // 上一次的轨迹**开头**（history 累积、firstError 还是上一次的根因），
    // 于是一台**成功**的第二次测量会带着一份完整的、指向第一次失败的
    // 根因记录。而它看起来完全正常 —— 这属于本项目反复出现的那一类缺陷
    // （字段存在、类型合法、数值看起来正常，但语义不成立）。
    // 注意 firstFailureSeen_ 也必须一起清：它是"首次失败"的判据开关。
    failureTrace_     = data::FailureTrace{};
    pendingCause_     = data::ErrorInfo{};
    firstFailureSeen_ = false;

    // ④ 任务期间启用硬件触发；任务结束时在 onEnteredState() 里关闭。
    //    §7.5 规定的"触发失效 → 降级为软触发（3001）"由设备层的
    //    CameraSynchronizer 判定并上报（ENG-05 §10），本类不重复推断 ——
    //    把 triggerTimestamp == 0 当作"触发失效"会把合法的软触发运行
    //    误报成降级。
    if (trigger_ != nullptr && !trigger_->enable())
    {
        note("硬件触发使能失败，本次任务按设备层的软触发路径采集（§7.5 的 3001 由设备层上报）");
    }

    // ⑤ 启动。§5.1 的退出条件"开始测量"即 IDLE → SEARCH。
    if (!transitionTo(MeasurementState::SEARCH, nowNs))
    {
        // FAILED 也是 IDLE 的合法去向（§7.5 的 ≤1 路相机），此处仅在
        // 转换表本身被破坏时才会失败；如实记录，不伪造状态。
        lastError_ = stateMachine_.lastError();
        note("无法进入 SEARCH：" + lastError_.message);
    }
}

void MeasurementController::stopMeasurement()
{
    if (finished() || stateMachine_.state() == MeasurementState::IDLE)
    {
        return;   // 无活动任务
    }

    const uint64_t nowNs = data::monotonicNowNs();

    // 先让机构停下来，再改软件状态。
    //
    // 顺序不可颠倒：转台此刻可能正在运动，若先把状态置为 FAILED 而
    // 停止命令因任何原因未发出，操作者会看到一个"已停止"的界面与一个
    // 仍在转动的机构。先发停止命令使"界面显示停止"成为事实的**结果**
    // 而不是对事实的**声明**。
    turntable_.stop();

    // SYS-08 §7.3：9003 = 人工取消。§7.3 原文强调"SEARCH 状态可由人工终止"
    //（因为 SEARCH 不设次数上限，没有人工出口就只能等 T_task）；
    // 其余状态的人工终止未被禁止，且从安全角度必须允许。
    failWith(data::ErrorInfo{data::kErrManualCancel, "人工取消测量任务", nowNs},
             nowNs);
}

data::MeasurementState MeasurementController::state() const
{
    return stateMachine_.state();
}

// ---------------------------------------------------------------------------
// 事件循环入口（SYS-08 §9 + §7.6 约束 3）
// ---------------------------------------------------------------------------

bool MeasurementController::tick(uint64_t nowNs)
{
    if (finished() || stateMachine_.state() == MeasurementState::IDLE)
    {
        return false;   // 没有活动任务
    }

    // ⚠ 硬保证优先，且**先于任何状态逻辑**（SYS-08 §10 用例 7 的答案）。
    //
    // §7.1 的 T_task 是"单次任务必然终止"的唯一依据（§11 约束 9）；
    // 各状态的次数上限只是效率约束。若某状态的"最后一次尝试"与 T_task
    // 同时到期，必须由 T_task 胜出 —— 否则会多跑一个状态动作，
    // "任务在 T_task 内终止"这条对外承诺就有了例外。
    // 因此 9001 的判定必须在这一行，而不是放在各 stepXxx() 里。
    if (retry_.deadlineExceeded(nowNs))
    {
        failWith(data::ErrorInfo{data::kErrTaskTimeout,
                                 "单次测量任务超出 T_task",
                                 nowNs},
                 nowNs);
        return true;
    }

    const MeasurementState before = stateMachine_.state();

    switch (before)
    {
    case MeasurementState::SEARCH:         stepSearch(nowNs);         break;
    case MeasurementState::TARGET_FOUND:   stepTargetFound(nowNs);    break;
    case MeasurementState::ALIGN:          stepAlign(nowNs);          break;
    case MeasurementState::STABILIZE:      stepStabilize(nowNs);      break;
    case MeasurementState::MEASURE_SELECT: stepMeasureSelect(nowNs);  break;
    case MeasurementState::CAPTURE:        stepCapture(nowNs);        break;
    case MeasurementState::POSE_SOLVE:     stepPoseSolve(nowNs);      break;
    case MeasurementState::VALIDATE:       stepValidate(nowNs);       break;
    case MeasurementState::SAVE:           stepSave(nowNs);           break;

    case MeasurementState::IDLE:
    case MeasurementState::COMPLETE:
    case MeasurementState::FAILED:
        break;   // 上面已排除
    }

    return stateMachine_.state() != before;
}

// ---------------------------------------------------------------------------
// 各状态的单步执行
// ---------------------------------------------------------------------------

void MeasurementController::stepSearch(uint64_t nowNs)
{
    // SEARCH 的"一次动作" = 一次三相机采集 + 一次 CAM25 检测。
    // 不设次数上限（§7.3），故每次 tick 都是一次新动作。
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // SEARCH 上限为 0（无限），此分支实际不可达
    }

    data::MultiCameraFrame frame;
    if (!acquire(frame, nowNs))
    {
        return;   // acquire 内部已按 §7.5 / §7.2 处置
    }
    lastFrame_ = frame;

    // §5.2："25mm 优先检测"。用 CAM25 的理由不依赖本类 ——
    // §8 的 AUTO 映射与 PreviewManager::mapStateToCamera 已冻结
    // "SEARCH → CAM25"：大视场才能让目标进入画面。
    data::DetectionResult detection;
    if (!detectOn(frame, data::CameraRole::CAM25, detection, nowNs))
    {
        return;
    }

    if (!detection.found)
    {
        // §5.2"失败：继续搜索"。**这不是失败**：目标可能还在视场外，
        // 故不调用 handleFailure()，也不记错误 —— 只返回，等下一次 tick。
        return;
    }

    // 检出目标，交给 TARGET_FOUND 做位置/尺度计算（§5.3）。
    pendingDetection_ = detection;
    transitionTo(MeasurementState::TARGET_FOUND, nowNs);
}

void MeasurementController::stepTargetFound(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 1（§7.3），用尽即回退 SEARCH
    }

    // 复用 SEARCH 检出的那一帧：目标位置与检出时刻必须对应同一帧，
    // 重新采集会让偏差与检测框来自不同时刻。
    const data::ImageFrame& image = frameOf(lastFrame_, data::CameraRole::CAM25);

    data::TargetOffset offset;
    if (!offsetFrom(pendingDetection_, image, offset))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      "TARGET_FOUND 的检测框退化（边框尺寸为 0 或图像为空），"
                                      "无法计算目标偏差",
                                      nowNs},
                      nowNs);
        return;
    }

    // §5.3："目标位置计算；尺度估计；初始方向计算"。
    // 尺度估计的结果是 POSE_SOLVE 的输入之一，且它的失败与偏差计算失败
    // 同属"这一帧不可用"，故同一处置（回退 SEARCH 重新检出）。
    data::TargetScaleEstimate scale;
    if (!pipeline_.estimateScale(image, pendingDetection_, scale))
    {
        // 为什么不写 9004 了事：尺度估计依赖机型库的点表
        //（PosePipeline::estimateScale 在机型库未加载时静默返回 false）。
        // 归 5001 的判据见 modelErrorOr() 的说明。
        handleFailure(FailureKind::TRANSIENT,
                      modelErrorOr("目标尺度估计失败", nowNs),
                      nowNs);
        return;
    }

    // centered 的判定逻辑不放在 data 层（TargetOffset.h 的明文规定：
    // data 不含业务逻辑），而是由 AlignmentController 按 SYS-08 §5.4 /
    // SYS-10 §7 的 ±centerThreshold 判据填写。
    alignment_.judgeCentered(offset);
    lastOffset_ = offset;

    transitionTo(MeasurementState::ALIGN, nowNs);
}

void MeasurementController::stepAlign(uint64_t nowNs)
{
    // §8：ALIGN 用 CAM50。
    const data::CameraRole camera =
        displayCameraFor(MeasurementState::ALIGN);

    // ---- ① 上一个指令是否还在路上 ----
    //
    // 转台运动是物理过程，一个动作要跨多次 tick。等待期间**不消耗尝试**
    // 也不做检测 —— 转台在动的时候拍到的偏差没有意义，用它算出的指令
    // 会让闭环在两个位置之间反复过冲。
    if (moveIssued_)
    {
        const data::TurntableState ts = turntable_.state();

        if (ts.motion == data::TurntableMotionState::ERROR)
        {
            // §7.5："转台故障 → 直接 FAILED，不重试"（转台无冗余）。
            moveIssued_ = false;
            handleFailure(FailureKind::HARDWARE,
                          data::ErrorInfo{data::kErrTurntableComm,
                                          "转台上报故障状态",
                                          nowNs},
                          nowNs);
            return;
        }

        if (ts.motion == data::TurntableMotionState::MOVING)
        {
            if (actionTimedOut(nowNs))
            {
                // §7.3 的"每次 2.5 s 超时"：一次指令在超时内未到位即算
                // 这次对准动作失败。按 §7.7 的 ALIGN 行，瞬态失败回状态内
                // 重试（下一次 tick 会发起新的指令，消耗下一次尝试）。
                moveIssued_ = false;
                handleFailure(FailureKind::TRANSIENT,
                              data::ErrorInfo{data::kErrStateFailure,
                                              "转台指令在 ALIGN 超时内未到位",
                                              nowNs},
                              nowNs);
                return;
            }
            return;   // 继续等
        }

        // 已到位（IDLE / STABLE）：本次动作结束，允许下一次动作记账。
        moveIssued_ = false;
        actionOpen_ = false;
    }

    // ---- ② 重新检测（§5.4 流程的最后一环）----
    data::MultiCameraFrame frame;
    if (!acquire(frame, nowNs))
    {
        return;
    }
    lastFrame_ = frame;

    data::DetectionResult detection;
    if (!detectOn(frame, camera, detection, nowNs))
    {
        return;
    }

    if (!detection.found)
    {
        // §5.4 未给"对准中丢失目标"的行。对准阶段目标本应在视场内居中，
        // 此时丢失只可能来自一次漏检或视场突变，属 §7.2 的**瞬态**：
        // 换一帧重算即可（§7.3 升级规则第 1 条）。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "ALIGN 过程中丢失目标", nowNs},
                      nowNs);
        return;
    }

    data::TargetOffset offset;
    if (!offsetFrom(detection, frameOf(frame, camera), offset))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "ALIGN 的检测框退化，无法计算偏差", nowNs},
                      nowNs);
        return;
    }

    alignment_.judgeCentered(offset);
    lastOffset_ = offset;

    // ---- ③ §5.4 的停止条件：目标中心误差 ≤ ±50 pixel ----
    if (offset.centered)
    {
        transitionTo(MeasurementState::STABILIZE, nowNs);
        return;
    }

    // ---- ④ 未达判据：发出新的转台指令（消耗一次尝试）----
    //
    // ⚠ 这是"8 次上限"被消耗的唯一位置。检测与等待都不消耗，
    // 故 8 次 = 8 条转台指令 = 最多 8 × 2.5 s = 20 s（§7.3 的时间预算）。
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 次数用尽 → 已按 §7.7 进入 FAILED(2003)
    }

    const data::TurntableCommand command =
        alignment_.calculate(offset, turntable_.state(),
                             calibrationOf(camera));

    if (!alignment_.commandValid())
    {
        // 命令不可信。两种情况：
        //  · 越程（2002）—— §7.7 的**能力边界**：转台到不了，重试 8 次
        //    也到不了，必须立刻停（"停止运动 + 报警 + FAILED"）；
        //  · 标定无效 —— 标定是任务级不变量，重试同样不改变结果。
        //    §7.2 的三分类里没有"配置/标定错误"，取 HARDWARE
        //    （理由正是"重试不改变结果"这一句）。
        const bool over =
            alignment_.overTravel();   // 必须在 commandValid() 之后读
        handleFailure(over ? FailureKind::CAPABILITY : FailureKind::HARDWARE,
                      alignment_.lastError(),
                      nowNs);
        return;
    }

    if (!turntable_.move(command))
    {
        // 2001 = 转台通信失败。ENG-09 §5.27 与 §7.7 都把它归为瞬态可重试
        //（与"转台报故障状态"不同：通信失败可能只是偶发丢包）。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrTurntableComm,
                                      "转台指令下发失败",
                                      nowNs},
                      nowNs);
        return;
    }

    moveIssued_ = true;
}

void MeasurementController::stepStabilize(uint64_t nowNs)
{
    // ⚠ STABILIZE 的"一次动作" = **一个稳定观察窗口**，而不是一帧。
    //
    // 依据 §5.5（判据是"连续稳定帧 N ≥ 3"）+ §7.3（STABILIZE 上限 1）。
    // 若把每一帧当作一次尝试，上限 1 会让"连续 3 帧"永远不可能满足 ——
    // 文档里这两个数字只有"一帧不是一次尝试"这一种解释才能同时成立。
    // 窗口的时限即 stabilizeTimeoutNs（3.0 s，§7.1 的状态级时限）。
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 1 用尽 → 已按 §7.7 回退 ALIGN
    }

    if (actionTimedOut(nowNs))
    {
        // 窗口到期而 §5.5 的条件未满足 → 本次稳定尝试失败。
        // 处置为 RETRY_IN_STATE，而下一次动作会被 RetryManager 拒绝
        //（上限 1），于是实际结果是 §7.7 的"回退至 ALIGN"。
        stableFrames_ = 0;
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      "稳定观察窗口超时，未满足连续 3 帧稳定",
                                      nowNs},
                      nowNs);
        return;
    }

    const data::TurntableState ts = turntable_.state();

    if (ts.motion == data::TurntableMotionState::ERROR)
    {
        stableFrames_ = 0;
        handleFailure(FailureKind::HARDWARE,
                      data::ErrorInfo{data::kErrTurntableComm,
                                      "转台上报故障状态（STABILIZE）",
                                      nowNs},
                      nowNs);
        return;
    }

    if (ts.motion == data::TurntableMotionState::MOVING)
    {
        // 还在动：本次采样不算稳定帧，但不消耗尝试窗口 —— §5.5 的
        // "等待转台停止"本身就是这个状态的功能。
        stableFrames_ = 0;
        return;
    }

    const data::CameraRole camera =
        displayCameraFor(MeasurementState::STABILIZE);

    data::MultiCameraFrame frame;
    if (!acquire(frame, nowNs))
    {
        stableFrames_ = 0;
        return;
    }
    lastFrame_ = frame;

    data::DetectionResult detection;
    if (!detectOn(frame, camera, detection, nowNs))
    {
        stableFrames_ = 0;
        return;
    }

    if (!detection.found)
    {
        stableFrames_ = 0;
        return;   // 目标暂不可见：不算稳定，但也不是失败
    }

    data::TargetOffset offset;
    if (!offsetFrom(detection, frameOf(frame, camera), offset))
    {
        stableFrames_ = 0;
        return;
    }

    alignment_.judgeCentered(offset);
    lastOffset_ = offset;

    if (!offset.centered)
    {
        // "图像稳定/目标稳定"未满足（§5.5 的三项等待里两项不成立）。
        stableFrames_ = 0;
        return;
    }

    ++stableFrames_;
    if (stableFrames_ >= kStableFramesRequired)
    {
        transitionTo(MeasurementState::MEASURE_SELECT, nowNs);
    }
}

void MeasurementController::stepMeasureSelect(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 3 用尽 → FAILED
    }

    const std::vector<data::CameraRole> allowed = strategy_.allowedCameras();
    if (allowed.empty())
    {
        // §7.3："候选相机集合为空 → 3 次"。三台相机都已被排除
        //（每次 VALIDATE 失败排除一台，§5.9）时到达这里。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "候选相机集合为空（全部通道已被排除）", nowNs},
                      nowNs);
        return;
    }

    // 重新采集：ALIGN/STABILIZE 期间转台已停止、光照可能已变，
    // 那一帧的评分不能代表当前的通道质量（同 §7.3 升级规则第 1 条的理由）。
    data::MultiCameraFrame frame;
    if (!acquire(frame, nowNs))
    {
        return;
    }
    lastFrame_ = frame;

    data::MeasurementSelectionResult selection;
    if (!pipeline_.selectCamera(frame, allowed, selection))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "测量通道评分失败", nowNs},
                      nowNs);
        return;
    }

    if (strategy_.isExcluded(selection.selectedCamera))
    {
        // 评分器返回了一个已被排除的通道。这违反 §7.4 的排除语义
        //（被排除的通道不得再被选中），如实失败而不是静默接受 ——
        // 静默接受会让"每次回退排除一台"这条保证失效。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      std::string("通道评分返回了已被排除的通道 ")
                                          + roleName(selection.selectedCamera),
                                      nowNs},
                      nowNs);
        return;
    }

    // R06：角色 + 得分 + 自动显示源**同时**写回（此前只写角色，
    // 完整结论落在局部变量里 → buildRecord 的 selectedScore 恒为 0）。
    adoptSelection(selection);

    transitionTo(MeasurementState::CAPTURE, nowNs);
}

void MeasurementController::stepCapture(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 3 用尽 → 回退 MEASURE_SELECT
    }

    // §5.7："多帧采集，5~10 frames，选择最佳帧"。
    //
    // ⚠ 上限由 100 收紧到 20（C-02 §2.6）。
    // 原上限 100 是"防 tick() 失控"的粗保护（§11 约束 9 要求单次 tick 工作量
    // 有界），但本函数现在每次采集留存**三路**（3 × 约 5 MB），100 帧 ≈ 1.5 GB
    // —— 一个配置笔误就能吃光内存，而失败表现为 cv::Mat 分配异常或返回空 Mat，
    // 不是一条能指向原因的"配置错误"。§5.7 的取值域本就是 5~10，取 20 已有
    // 一倍余量，且与"成对留三路"的内存量级相称。
    const int wanted = std::min(std::max(measurementConfig_.captureFrameCount, 1), 20);
    if (wanted != measurementConfig_.captureFrameCount)
    {
        // 夹取必须**可见**（与 §7.5"降级必须可见"同一条原则：既成事实不得静默）。
        note("captureFrameCount="
             + std::to_string(measurementConfig_.captureFrameCount)
             + " 超出允许范围 [1,20]，已夹取为 " + std::to_string(wanted));
    }

    capturedFrames_.clear();
    capturedFrames_.reserve(static_cast<std::size_t>(wanted));

    for (int i = 0; i < wanted; ++i)
    {
        data::MultiCameraFrame frame;
        if (!acquire(frame, nowNs))
        {
            return;   // 采集失败/降级越界，acquire 内部已处置
        }
        lastFrame_ = frame;

        // 留**整帧**（三路齐），不再只留选定焦段那一路（C-02 §1.2）：
        // 结果包要求 cam25/50/100 三路原图**对应同一次曝光**，
        // 只留一路时另外两路无从追溯，"结果与 raw 不是同一次测量"。
        //
        // ⚠ 此处**不做空帧跳过**（原实现是 `if (!image.image.empty())` 才 push）。
        // 跳过会破坏"capturedFrames_[k] 就是第 k 次采集"这一对应关系：若第 2 次
        // 采集的选定焦段为空，capturedFrames_[2] 实际变成第 3 次采集。而本函数
        // 下一步正是"用评分下标**回查**那一次采集"，在错位的下标上回查会取到
        // 另一次采集的三路图 —— 恰是 C-02 要修的缺陷换个位置复现
        // （C-02 §1.3 的 D-C02-3）。空帧的判定因此下移到按**角色**做：
        // 它只影响"这一帧能不能参与评分"，不影响"它是第几次采集"。
        capturedFrames_.push_back(frame);
    }

    // 建"选定焦段"视图 + 采集下标映射（C-02 §2.2）。
    // 视图只收非空的选定焦段帧（供算法评分），映射把视图下标还原为采集下标；
    // 二者顺序严格一致，viewToAcq 是**唯一**允许做这一换算的地方。
    // 算法层接口 `selectBestFrame(vector<ImageFrame>, …)` 保持不变 ——
    // "按焦段取图"是应用层的事，不该推进算法层。
    std::vector<data::ImageFrame> view;
    std::vector<int>              viewToAcq;
    view.reserve(capturedFrames_.size());
    viewToAcq.reserve(capturedFrames_.size());
    for (std::size_t i = 0; i < capturedFrames_.size(); ++i)
    {
        const data::ImageFrame& image = frameOf(capturedFrames_[i], selectedCamera_);
        if (!image.image.empty())
        {
            view.push_back(image);
            viewToAcq.push_back(static_cast<int>(i));
        }
    }

    if (view.empty())
    {
        // §7.3："采集失败或有效帧数不足 → 3 次"。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      std::string("采集未获得 ")
                                          + roleName(selectedCamera_)
                                          + " 的有效帧",
                                      nowNs},
                      nowNs);
        return;
    }

    int viewBest = -1;
    data::ImageQuality quality;
    if (!pipeline_.selectBestFrame(view, viewBest, quality))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "最佳帧选择失败", nowNs},
                      nowNs);
        return;
    }

    // 越界校核针对**视图**下标（selectBestFrame 的返回值定义域）。
    // 算法层返回越界下标时不得带着它去索引映射表。
    if (viewBest < 0 || viewBest >= static_cast<int>(viewToAcq.size()))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "最佳帧索引越界", nowNs},
                      nowNs);
        return;
    }

    // ⚠ 映射回**采集下标**，并把评分选中的那一次采集整帧留存。
    // 这两行是 C-02 的核心：此前 bestFrameIndex_ 只被赋值、无人读取，
    // POSE_SOLVE 用的是 lastFrame_（**最后一次**采集），于是
    // "评分最高的那一帧"与"被解算的那一帧"可以是两张不同的图，
    // 而记录里的质量描述的是前者（C-02 §1.1 的 D-C02-1）。
    // 现在：bestFrame_ 既是被解算的输入，也是 bestQuality_ 描述的对象，
    // 且三路齐 —— 证据、结论、评分三者同源。
    const int acq = viewToAcq[static_cast<std::size_t>(viewBest)];
    bestAcqIndex_ = acq;
    bestFrame_    = capturedFrames_[static_cast<std::size_t>(acq)];
    bestQuality_  = quality;

    transitionTo(MeasurementState::POSE_SOLVE, nowNs);
}

void MeasurementController::stepPoseSolve(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 2 用尽 → 回退 MEASURE_SELECT（6001 由策略置入）
    }

    // §5.8 / §7.3："最多 2 次尝试，按 §7.3 升级规则更换输入
    //（第 2 次必须换用次优相机）"。
    //
    // ⚠ 这里必须**换相机**，不能只是重算：PnP 的输入（内参、特征匹配）
    // 在第 2 次完全不变时，确定性算法必然给出同一个失败结果 ——
    // 不换输入的重试只消耗预算（§7.3 原文）。
    //
    // ⚠ 换相机用 bestFrame_（**被解算的那一次**采集），不用 lastFrame_。
    // 换的是**通道**、不是帧：硬触发下三路是同一次曝光，第 2 次尝试应当在
    // "同一瞬间的三路"里换一个焦段，而不是换到另一个时刻去。
    // 换相机后**不重跑** selectBestFrame（裁决 A-1）：同一次曝光里再选一次，
    // 引入的只是变量而不是依据。
    if (retry_.attempts(MeasurementState::POSE_SOLVE) >= 2)
    {
        if (!switchToNextCamera(bestFrame_))
        {
            // 无其他候选可换：第 2 次尝试无法满足"必须换相机"，
            // 继续算下去只是重复第 1 次。按 §5.8 的出口回退 MEASURE_SELECT，
            // 让通道选择重新评分。
            handleFailure(FailureKind::TRANSIENT,
                          data::ErrorInfo{data::kErrStateFailure,
                                          "PnP 第 2 次尝试无其他候选相机可换，回退重新选择通道",
                                          nowNs},
                          nowNs);
            return;
        }
    }

    // ⚠ 输入是 bestFrame_，不是 lastFrame_（C-02 的行为修正）。
    // §5.7 要求"多帧采集……**选择**最佳帧"，被选出的那一帧就是 PnP 的输入。
    // 用 lastFrame_（最后一次采集）会让"选择的结论"与"解算的输入"脱钩，
    // 且 bestQuality_ 描述的是另一张图 —— 记录与被解算图像不一致且不报错。
    // 因为这条改动，**同一场景的姿态数值会与 010/M1 的归档结果不同**
    // （已按 C-02 §3 登记为"已裁决的行为修正"，状态机轨迹不变）。
    data::ShipPoseResult pose;
    const bool solved =
        pipeline_.solvePose(bestFrame_, selectedCamera_,
                            calibrationOf(selectedCamera_), pose);

    if (!solved || !pose.success)
    {
        // 码的取法同尺度估计：机型库未加载 → 5001，否则 9004。
        // ⚠ 6001 不在此处：它的语义是"重试次数用尽"，由
        // MeasurementStrategy::onAttemptsExhausted 在次数**真正**用尽时给出。
        // 若这里就写 6001，第一次 PnP 失败会被记成"重试已用尽"，
        // 而实际上还允许再试一次 —— 那是把过程当结论。
        handleFailure(FailureKind::TRANSIENT,
                      modelErrorOr("PnP 姿态解算失败（通道 "
                                       + std::string(roleName(selectedCamera_))
                                       + "）",
                                   nowNs),
                      nowNs);
        return;
    }

    poseResult_ = pose;
    transitionTo(MeasurementState::VALIDATE, nowNs);
}

void MeasurementController::stepValidate(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 2 用尽 → FAILED
    }

    data::PoseValidationResult validation;

    // ⚠ 返回值**不参与控制流**（R05，2026-09-24 审查报告）。
    //
    // 契约（IPosePipeline::validate 的 @return，由本批裁决统一）：
    //     `validate()` 的 bool  ≡  `out.valid`  ≡  "是否通过全部判据"
    // 它**不是**"验证过程是否执行成功" —— 本接口没有表达执行失败的通道。
    // 真实 PoseValidator 的每一个返回点都满足该恒等（见 PoseValidator.h:70）。
    //
    // 此前此处写作 `if (!pipeline_.validate(poseResult_, validation))`，
    // 把"判不合格"读成"验证过程失败"，于是紧接着的两步
    // —— `validationResult_ = validation` 与 `strategy_.excludeCamera(...)`
    // —— **都执行不到**：真实验证器判不合格时返回 false ⇒ 生产路径必然踩中
    // （不排除相机、不记录验证详情，而是直接以 9004 报"过程失败"）。
    // 测试桩当时恒 `return true`，所以这条断点在所有测试里都看不见。
    const bool verifyReturned = pipeline_.validate(poseResult_, validation);

    // 不论合格与否、**甚至不论契约是否被违背**，本次验证的**详情**都要留存：
    // SAVE 落盘要它，失败包要它（saveFailurePackage），SystemInitializer 读
    // confidence / inlierRatio 也要它。这正是原实现提前 return 丢掉的东西。
    //
    // ⚠ 赋值点**必须在对账分支之前**（本条是 R05 修复的一部分，不是随手的位置）：
    //   对账分支同样是"提前 return"，若把赋值放在它后面，就等于在新的失败通路上
    //   原样复现了 R05 要修的那个缺陷 —— "验证过程出问题时，验证详情反而最不可得"。
    //   而在契约被违背时，实现到底报了什么（reprojectionError / inlierRatio /
    //   confidence）**正是定位该实现哪里坏了的唯一线索**，丢掉它会让对账分支
    //   只剩一句"true ≠ false"，无法判断是哪个字段伴随而来的。
    //   此处只**存**不**用**：下面的分支仍以 out.valid 之外无任何动作为前提，
    //   故把一个自相矛盾实现写出的结果留存下来不会污染任何控制流。
    validationResult_ = validation;

    // 对账：实现返回的 bool 与它写入的 out.valid 不一致 = 违背契约。
    //   `out.valid` 为**权威**，返回值只作对账。
    //   处置与本工程其它契约违背同构（评分器返回已被排除的通道、
    //   最佳帧索引越界）：必须**可见地**失败，不得静默按任一方继续。
    //   真实实现恒等，故本分支只在实现坏掉时可达。
    if (verifyReturned != validation.valid)
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      std::string("验证契约违背：validate() 返回 ")
                                          + (verifyReturned ? "true" : "false")
                                          + " 而 out.valid="
                                          + (validation.valid ? "true" : "false"),
                                      nowNs},
                      nowNs);
        return;
    }

    if (!validation.valid)
    {
        // §5.9 原文："每次必须**排除上一次失败的相机**"。
        // §7.4 给出该规则的数量依据：三台相机、每次排除一台，
        // 第 3 次已无候选可选 —— 这正是 VALIDATE 的回退上限取 2 的原因。
        // 排除动作必须发生在这里（而不是在 MEASURE_SELECT 里"猜"上一台），
        // 因为只有验证不通过的这一刻才知道是哪一台不行。
        strategy_.excludeCamera(selectedCamera_);

        // ⚠ code 保持 0（裁决 C-006 点名）：回退是**非错误的处置动作**，
        // 不是失败。本条记录的价值在 message（"排除了哪台相机"这一事实），
        // 不在错误分类。给它也塞一个码会污染 FailureTrace::history，
        // 使"哪些迁移由失败触发"这一判据失效。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{0,
                                      std::string("验证不通过，排除通道 ")
                                          + roleName(selectedCamera_)
                                          + " 并回退重新选择",
                                      nowNs},
                      nowNs);
        return;
    }

    transitionTo(MeasurementState::SAVE, nowNs);
}

data::MeasurementRecord MeasurementController::buildRecord() const
{
    data::MeasurementRecord record;

    // ---- 本类**能**填的部分：测量过程中产生的事实 ----
    record.task            = task_;
    record.bestFrame       = bestFrame_;
    record.statistics      = statistics_;
    record.turntable       = turntable_.state();
    record.selectedCamera  = selectedCamera_;
    record.selectedScore   = selection_.score;
    record.selectedQuality = bestQuality_;
    record.degraded        = degraded_;
    record.camerasAvailable = availableCameras_;

    // 失败根因与轨迹（裁决 C-007）。本函数同时服务于成功落盘（stepSave）
    // 与失败落盘（saveFailurePackage），故**如实填**，由调用方决定怎么用。
    //
    // ⚠ 成功任务必须是 firstError.code == 0（C-007 的验收原文）。本函数
    // 通过 `FAILED_TASK` 这个开关来保证，而不是靠"成功时轨迹恰好是空的"
    // —— 成功任务也可能有完整的 history（过程里失败并恢复过），
    // 用"轨迹为空"当判据等于用巧合冒充判据。
    if (failurePackage_)
    {
        record.failure = failureTrace_;
    }
    // 成功任务：只留轨迹，不留根因。理由见 FailureTrace.h ——
    // firstError/finalError 描述的是"任务以失败告终"这一结论，
    // 一次 COMPLETE 的任务没有这个结论，写进去就是伪造。
    // history 则不同：它是**路径**，成功任务的路径同样有留档价值
    // （"这次成功用了几次回退"是事后评估重试策略的唯一依据）。
    else
    {
        record.failure.history = failureTrace_.history;
    }

    // ---- 本类**填不了**的部分：标定版本 / 模型标识 / 软件版本 ----
    //
    // `calibrationId`、`modelId`、`modelType`、`softwareVersion` 四个字段
    // 在这里**故意留空**，由装配点（app）补齐 —— 它们不是测量过程的产物，
    // 而是"这次测量所依据的外部事实"，且本类**没有它们的来源**：
    //   · 标定标识：`OpticalRig` 未暴露 calibrationId（它由 ConfigManager
    //     读入并交给 Recorder 的构造参数），本类只拿得到 `CameraCalibration`
    //     的数值，拿不到它的版本名；
    //   · 模型标识：`TargetModelManager::modelId()` 在算法层**内部**，
    //     `IPosePipeline` 不暴露它（这是对的：它是模型库的元数据，
    //     不属于 IF-SW-02 的调用面）；
    //   · 软件版本：编译期宏，装配点读一次即可。
    // 与其在本类伪造（写空串、写 "unknown"、或去猜），不如留空并说明来源。
    // 装配点补齐后，到达 Recorder 的记录才是完整的（见
    // app/ApplicationContext.h 的 RecorderSinkAdapter）。

    return record;
}

data::MeasurementRecord MeasurementController::withTerminalHop(
    data::MeasurementRecord record,
    data::MeasurementState next,
    const data::ErrorInfo& terminalError,
    uint64_t nowNs) const
{
    const data::MeasurementState from = stateMachine_.state();

    // 不可达的终止迁移**不补**。留下一条没有发生的迁移，比"轨迹终点不是
    // 终点"更糟：前者是伪造，后者只是缺一跳到读者能自行接回。
    // （failWith() 对不可达的 FAILED 也走同一条路：note() 而不伪造状态。）
    if (!StateMachine::isLegalTransition(from, next))
    {
        return record;
    }

    data::StateTransition hop;
    hop.from        = from;
    hop.to          = next;
    hop.timestampNs = nowNs;
    hop.error       = terminalError;
    record.failure.history.push_back(hop);

    // ⚠ 终结点的状态与错误也要与轨迹**一致**：失败包的 `task.state` 由
    //   saveFailurePackage() 写死为 FAILED，成功包的由 stepSave() 写死为
    //   COMPLETE，而 failureTrace_ 的 finalFailedState / finalError 只有
    //   failWith() 填过。这里只补轨迹，不改它们 —— 后者由生产方负责
    //   （failWith 在写包之前就填好了）。
    return record;
}

void MeasurementController::stepSave(uint64_t nowNs)
{
    actionOpen_ = false;
    if (!beginAttemptForAction(nowNs))
    {
        return;   // 上限 3 用尽 → FAILED
    }

    task_.result = poseResult_;
    task_.validation = validationResult_;

    // ⚠ task_.state 记的是**任务的结果**，不是"此刻处于哪个状态"。
    //
    // 落盘发生在 SAVE 状态内，若照搬当前状态，result.json 里的 state 就是
    // "SAVE" —— 而 SAVE 只是通往 COMPLETE 的一个中间态，读者拿到结果包时
    // 需要的是"这次测量成功了"。ENG-09 §5.26 / SYS-05 §12 只给了字段，
    // 未规定取值语义，故此处明确：**成功落盘在 SAVE 内完成，其 state 为
    // COMPLETE**。
    //
    // ⚠ 原注释还有后半句"失败的任务不产生结果包，故不存在'包在但 state 是
    // FAILED'的情形"。**该论断已被裁决 C-007 推翻**：失败任务现在也落盘
    // （saveFailurePackage），且它的包如实写 FAILED。走到本行的任务必是
    // 成功路径 —— 失败在 failWith() 里就已经落盘并转入终态了，不会再回到
    // SAVE 来。故这里的 COMPLETE 仍是**如实取值**，而不是硬编码。
    task_.state = data::MeasurementState::COMPLETE;

    if (recorder_ == nullptr)
    {
        // ⚠ 不把"没有 Recorder"当作落盘成功。
        //
        // 009 阶段之前（本阶段正是）infrastructure 的 RecorderWorker 尚未
        // 实现，M1 闭环不需要它也能跑通。但若此处静默成功，一个
        // "每次测量都没有结果包"的系统会以 COMPLETE 结束，而操作者只在
        // 事后翻目录时才发现 —— 那是本文件反复避免的那类静默退化。
        // 记入 notices_ 使其可见（并由 009 阶段的 Logger 与界面呈现）。
        note("未挂载 Recorder（IRecorderSink），SAVE 状态记录结果但未落盘");
    }
    else if (!recorder_->save(withTerminalHop(
                 buildRecord(), data::MeasurementState::COMPLETE,
                 data::ErrorInfo{}, nowNs)))
    {
        // §7.3："落盘失败 → 3 次"。磁盘写不进去没有更早的状态能解决，
        // 3 次后由 MeasurementStrategy::onAttemptsExhausted 判 FAILED。
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "测量结果包落盘失败", nowNs},
                      nowNs);
        return;
    }

    transitionTo(MeasurementState::COMPLETE, nowNs);
}

// ---------------------------------------------------------------------------
// 协作者调用
// ---------------------------------------------------------------------------

bool MeasurementController::acquire(data::MultiCameraFrame& frame,
                                    uint64_t nowNs)
{
    if (!cameras_.capture(frame))
    {
        // §7.2 的**瞬态**：丢帧、超时都能靠换一帧解决。
        //
        // 码的来源（裁决 C-006）：**如实传递设备层给出的码**，与上面
        // `alignment_.lastError()` 的做法一致。设备层知道原因，本层不知道，
        // 故本层不猜：
        //   · 可用相机数不足 → 1001（硬件故障，不重试）
        //   · 三路时间戳超差 → 3002（瞬态，重采即可）
        //   · 设备层未填写   → 9004 兜底，不冒充上面任何一个
        // 若上层只凭 `false` 自己编一个码，就等于把这两类处置完全不同的
        // 失败混为一谈 —— 而那正是 C-006 要消除的。
        data::ErrorInfo deviceError = cameras_.lastError();
        if (deviceError.code == 0)
        {
            deviceError.code = data::kErrStateFailure;
        }
        if (deviceError.message.empty())
        {
            deviceError.message = "三相机同步采集失败";
        }
        deviceError.timestampNs = nowNs;
        handleFailure(FailureKind::TRANSIENT, deviceError, nowNs);
        return false;
    }

    // R04：测量进行中预览的**唯一**生产者。
    //
    // 位置理由：acquire() 是活动态全部采集的汇聚点（SEARCH / ALIGN /
    // STABILIZE / TARGET_FOUND / MEASURE_SELECT / CAPTURE 循环全经它），
    // 故补帧只写这一处即覆盖全部活动态，且**不新增任何 capture() 调用**。
    //
    // ⚠ 放在 updateDegradation() **之前**（而不是它的 true 分支里）：
    //   降级越界（可用 ≤1 路）会就地 FAILED，若补帧排在后面，
    //   **导致任务终止的那一帧永远到不了屏幕** —— 而那一刻画面正是唯一的
    //   现场证据。放在前面则最后一次采集总会显示；显示源那一路本帧为空时
    //   submitFrom() 自己返回 false 不提交、画面保留上一帧（这正是它不肯
    //   提交空帧的用意），降级情形自然退化正确。
    //
    // 顺序读法：acquire（设备事实）→ 预览（显示事实）→ 降级（策略）。
    submitPreview(frame);

    return updateDegradation(frame, nowNs);
}

void MeasurementController::submitPreview(const data::MultiCameraFrame& frame)
{
    if (preview_ == nullptr)
    {
        return;   // 未注入预览（大量单测）——与 setAutoCamera 的判空同构
    }

    // 返回 false = 当前显示源那一路本帧为空，**不是错误**
    // （相机降级 / 本轮未采）。故丢弃返回值，而不是记一个假的失败。
    //
    // ⚠ CAPTURE 会连采 5~10 帧，此处**逐帧全投递**，不做"只投最佳帧"的
    //   特殊处理。三条理由：
    //     ① acquire() 在原理上不知道哪一帧最好 —— 最佳帧由循环**之后**的
    //        selectBestFrame 对选定焦段的视图算出；"只投最佳帧"会让这个
    //        服务全部活动态的修复反向依赖 CAPTURE 的内部结构。
    //     ② 队列的冻结策略已把突发处理完（容量夹取 [3,5]、push 非阻塞、
    //        满载丢旧保新），突发后留下的正是"此刻相机看到了什么"的如实
    //        答案；中间帧也不是静默丢失，droppedCount() 精确计数。
    //     ③ "每拍只投一帧"需要一条并不存在的规则（投第几帧？），
    //        且会让预览滞后一整个突发。
    //   副作用（不是缺陷）：CAPTURE 期间 droppedCount() 会合法上升。
    //   别把它读成故障。
    (void)preview_->submitFrom(frame);
}

bool MeasurementController::updateDegradation(const data::MultiCameraFrame& frame,
                                             uint64_t nowNs)
{
    // 可用性判据取"该通道的图非空"。
    //
    // ⚠ 为什么不查 CameraChannel::enabled：本类不持有 OpticalRig 的通道表
    // 的所有权语义，而**空图是设备层对"这一帧没拿到"的唯一如实表达**。
    // 被管理性禁用的通道在采集结果中同样表现为空图，故两者殊途同归。
    int available = 0;
    const data::CameraRole kRoles[3] = {
        data::CameraRole::CAM25,
        data::CameraRole::CAM50,
        data::CameraRole::CAM100,
    };
    for (data::CameraRole role : kRoles)
    {
        if (!frameOf(frame, role).image.empty())
        {
            ++available;
        }
    }
    availableCameras_ = available;

    // SYS-08 §7.5 冻结的三行。
    if (available <= 1)
    {
        // 硬件故障类 —— 不重试。三相机体系退到 1 台时既无冗余也无交会，
        // 重试只会把"设备不够用"这个明确结论推迟到 T_task 之后。
        degraded_ = true;
        degradationNotice_ = data::ErrorInfo{
            data::kErrCameraInsufficient,
            "可用相机数 " + std::to_string(available) + " ≤ 1，无法继续测量",
            nowNs};
        handleFailure(FailureKind::HARDWARE, degradationNotice_, nowNs);
        return false;
    }

    if (available == 2)
    {
        // 降级**继续**运行。§7.5 的要求是"降级模式继续测量 + degraded=true
        // + 界面上可见"，不是失败。
        degraded_ = true;
        degradationNotice_ = data::ErrorInfo{
            data::kErrCameraDegraded,
            "可用相机数 2（少一台），以降级模式继续测量",
            nowNs};
        return true;
    }

    degraded_ = false;
    degradationNotice_ = data::ErrorInfo{};
    return true;
}

bool MeasurementController::detectOn(const data::MultiCameraFrame& frame,
                                     data::CameraRole role,
                                     data::DetectionResult& detection,
                                     uint64_t nowNs)
{
    const data::ImageFrame& image = frameOf(frame, role);

    if (image.image.empty())
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure,
                                      std::string("通道 ") + roleName(role)
                                          + " 本帧无图像",
                                      nowNs},
                      nowNs);
        return false;
    }

    if (!pipeline_.detect(image, detection))
    {
        handleFailure(FailureKind::TRANSIENT,
                      data::ErrorInfo{data::kErrStateFailure, "目标检测失败", nowNs},
                      nowNs);
        return false;
    }

    return true;
}

bool MeasurementController::offsetFrom(const data::DetectionResult& detection,
                                       const data::ImageFrame& frame,
                                       data::TargetOffset& offset) const
{
    if (frame.image.empty() || detection.bbox.width <= 0
        || detection.bbox.height <= 0)
    {
        return false;
    }

    // C-11 与 ENG-09 §2.4 共同冻结的口径：
    //   原点 = **图像中心**，X 向右为正，Y **向下**为正。
    // ⚠ 用 cols/2 而不是 (cols-1)/2：像素中心坐标的连续化写法。
    // 两者相差 0.5 pixel，对 ±50 pixel 的判据无影响，但对
    // SYS-15 的误差合成有影响，故在此处固定下来，不与 AlignmentController
    // 的 arctan 换算产生半像素级的系统偏差。
    const double centreX = static_cast<double>(frame.image.cols) / 2.0;
    const double centreY = static_cast<double>(frame.image.rows) / 2.0;

    const double pixelX = static_cast<double>(detection.bbox.x)
                        + static_cast<double>(detection.bbox.width) / 2.0
                        - centreX;
    const double pixelY = static_cast<double>(detection.bbox.y)
                        + static_cast<double>(detection.bbox.height) / 2.0
                        - centreY;

    if (!std::isfinite(pixelX) || !std::isfinite(pixelY))
    {
        return false;
    }

    offset.pixelX = pixelX;
    offset.pixelY = pixelY;
    offset.centered = false;   // 判据由 AlignmentController 填写
    return true;
}

data::CameraCalibration
MeasurementController::calibrationOf(data::CameraRole role) const
{
    const data::OpticalRigCalibration calibration = rig_.calibration();

    switch (role)
    {
    case data::CameraRole::CAM25:  return calibration.cam25;
    case data::CameraRole::CAM50:  return calibration.cam50;
    case data::CameraRole::CAM100: return calibration.cam100;
    }

    return data::CameraCalibration{};
}

data::CameraRole
MeasurementController::displayCameraFor(data::MeasurementState state) const
{
    // 复用 preview 层已冻结的映射函数（SYS-08 §8 的三行 + 9 个状态的补齐，
    // 其补齐依据见 PreviewManager::mapStateToCamera 的注释）。
    // 复用的必要性：操作者看到的画面必须与控制器检测所用的画面**同一焦段**，
    // 否则"界面上目标居中"与"控制器认为目标偏心"会同时成立。
    return preview::PreviewManager::mapStateToCamera(
        state, selectedCamera_, data::CameraRole::CAM25);
}

bool MeasurementController::switchToNextCamera(const data::MultiCameraFrame& frame)
{
    strategy_.excludeCamera(selectedCamera_);

    const std::vector<data::CameraRole> allowed = strategy_.allowedCameras();
    if (allowed.empty())
    {
        return false;
    }

    data::MeasurementSelectionResult selection;
    if (!pipeline_.selectCamera(frame, allowed, selection))
    {
        return false;
    }

    if (selection.selectedCamera == selectedCamera_)
    {
        // 评分器在剩余候选中又选了同一台 —— 说明它对这一帧的评分不区分
        // 通道，换机对第 2 次尝试毫无帮助。如实返回 false，
        // 由调用方按 §5.8 回退 MEASURE_SELECT。
        return false;
    }

    // R06：与 stepMeasureSelect 走同一个收口，保证两处写回永远成对。
    adoptSelection(selection);
    return true;
}

// ---------------------------------------------------------------------------
// 转换与副作用
// ---------------------------------------------------------------------------

void MeasurementController::adoptSelection(
    const data::MeasurementSelectionResult& selection)
{
    // R06（2026-09-24 审查报告）：角色与得分必须**同时**写回。
    //
    // 不变量：`selection_` 恒描述 `selectedCamera_`。分成两处赋值时，将来
    // 任一处被单独改动（例如换机路径只改角色），record 里就会是"新角色 +
    // 旧得分"—— 两个值都合法、都不报错、都"看起来正常"，正是 D-C02-5
    // 换个位置复现。
    //
    // 附带消除一处既有重复：原两个调用点各自写了 preview_->setAutoCamera，
    // 现收进本函数一处。
    selectedCamera_ = selection.selectedCamera;
    selection_      = selection;

    if (preview_ != nullptr)
    {
        preview_->setAutoCamera(selectedCamera_);
    }
}

bool MeasurementController::transitionTo(data::MeasurementState next,
                                         uint64_t nowNs)
{
    // ⚠ `from` 必须在 transition() **之前**取：转换成功后
    // `stateMachine_.state()` 已经是 `next`，那时再读就永远得到
    // "自己到自己"的迁移，整条轨迹会退化成一张常量表。
    const data::MeasurementState from = stateMachine_.state();

    if (!stateMachine_.transition(next, nowNs))
    {
        // 转换被拒的三种来源：自转换、非法边、预算/次数被拒。
        // 一律如实记录 —— 尤其是第三种：它意味着策略表与转换表不一致，
        // 是设计缺陷而非运行时故障，必须留下痕迹。
        lastError_ = stateMachine_.lastError();
        return false;
    }

    // 失败轨迹（裁决 C-007）：本类是全部迁移的唯一收口，故此处即完整记录。
    //
    // `error` 取 pendingCause_ —— 它由 handleFailure() 在本次失败处置的入口
    // 写入，若本次迁移是由失败触发的（回退/失败），它非空；正常前进时为空。
    //
    // ⚠ 被拒的迁移**不记**：上面的提前返回保证了这一点。被拒的迁移没有
    // 发生，把它写进"路径"会让轨迹出现一条现实中不存在的边 ——
    // 而 §7.4 的预算耗尽恰恰以"迁移被拒"为表征，那种情况下轨迹里
    // 会凭空多出几条来回弹跳，正是排查时最容易被误导的形态。
    data::StateTransition hop;
    hop.from        = from;
    hop.to          = next;
    hop.timestampNs = nowNs;
    hop.error       = pendingCause_;
    failureTrace_.history.push_back(hop);

    // 立即清空：一次失败的原因只属于紧接着的那一次迁移。
    // 不清空会让它黏到后续每一次正常前进上，轨迹比没有更误导。
    pendingCause_ = data::ErrorInfo{};

    onEnteredState(next);
    return true;
}

void MeasurementController::onEnteredState(data::MeasurementState state)
{
    task_.state = state;

    // 动作窗口复位：每次进入状态，transition() 已记过一次尝试，
    // 故第一次动作不再记数（见 beginAttemptForAction 的说明）。
    pendingEntryAttempt_ = true;
    actionOpen_ = false;
    actionStartNs_ = 0;
    moveIssued_ = false;
    stableFrames_ = 0;

    if (preview_ != nullptr)
    {
        // §8：AUTO 模式下由状态控制预览源。
        // setMeasurementState 与 setAutoCamera 一起给，是因为 §8 的
        // "MEASURE → Selected Camera"这一行需要控制器提供 selected；
        // 只给状态会让 PreviewManager 无从知道选中了哪一台。
        preview_->setMeasurementState(state);
        preview_->setAutoCamera(displayCameraFor(state));
    }

    if (state == MeasurementState::COMPLETE)
    {
        poseResult_ = task_.result;
    }

    if (state == MeasurementState::COMPLETE || state == MeasurementState::FAILED)
    {
        // 任务结束：关闭硬件触发，让相机回到自由运行以便继续预览。
        // 触发常开会占用带宽并让测距之外的时间也在曝光，没有理由保留。
        if (trigger_ != nullptr)
        {
            trigger_->disable();
        }

        // 任务结束：释放 CAPTURE 的采集缓存（C-02 §2.6）。
        //
        // ⚠ 必须在**此处**释放，且不得更早：结果是"任务级"的，而采集缓存
        // 现在每次留三路（约 75 MB / 5 帧），只在启动时清理会让它在任务之间
        // 一直挂着，连续测量逐次累积。
        //
        // ⚠ 也不得更晚：C-02 Step 5 起，结果包要记录"产出该姿态的那一次采集"
        // （MeasurementRecord::bestFrame）。届时**记录组装必须发生在本行之前**
        // （即在 SAVE 内完成），否则记录会拿到已被清空的 bestFrame_。
        // 现在还没有记录，故此处释放是安全的：终态之后没有任何代码再读它们。
        capturedFrames_.clear();
        capturedFrames_.shrink_to_fit();
        bestFrame_ = data::MultiCameraFrame{};
        bestAcqIndex_ = -1;
    }
}

// ---------------------------------------------------------------------------
// 动作记账（本类与 RetryManager 的唯一接缝）
// ---------------------------------------------------------------------------

bool MeasurementController::beginAttemptForAction(uint64_t nowNs)
{
    if (actionOpen_)
    {
        // 本次动作已记账：转台等待、稳定窗口的后续 tick 走这里。
        return true;
    }

    if (pendingEntryAttempt_)
    {
        // 进入状态时 transition() 已经记过一次（§7.6 约束 2），不再重复记。
        pendingEntryAttempt_ = false;
    }
    else if (!retry_.beginAttempt(stateMachine_.state(), nowNs))
    {
        // 次数用尽。处置方案由 §7.7 的"超限后果"列给出（表在
        // MeasurementStrategy 里），本类只负责执行。
        applyRecovery(strategy_.onAttemptsExhausted(stateMachine_.state()),
                      nowNs);
        return false;
    }

    actionOpen_ = true;
    actionStartNs_ = nowNs;
    return true;
}

bool MeasurementController::actionTimedOut(uint64_t nowNs) const
{
    if (actionStartNs_ == 0)
    {
        return false;   // 尚未开始任何动作
    }

    const uint64_t limit = stateTimeout(stateMachine_.state());
    if (limit == 0)
    {
        // 未配置超时：视为"不设状态级时限"，由 T_task 兜底。
        // 不把 0 当作"立即超时"—— 那会让一个没写全 yaml 的系统
        // 在每个状态上瞬间失败，而真实原因是配置缺失。
        return false;
    }

    return nowNs >= actionStartNs_ && (nowNs - actionStartNs_) >= limit;
}

uint64_t MeasurementController::stateTimeout(data::MeasurementState state) const
{
    // 唯一数据源是 MeasurementConfig（ENG-09 §6.5 / 裁决 C-20）。
    // 本函数不做任何算术推导，只做查表。
    switch (state)
    {
    case MeasurementState::SEARCH:         return measurementConfig_.searchTimeoutNs;
    case MeasurementState::TARGET_FOUND:   return measurementConfig_.targetFoundTimeoutNs;
    case MeasurementState::ALIGN:          return measurementConfig_.alignTimeoutNs;
    case MeasurementState::STABILIZE:      return measurementConfig_.stabilizeTimeoutNs;
    case MeasurementState::MEASURE_SELECT: return measurementConfig_.selectTimeoutNs;
    case MeasurementState::CAPTURE:        return measurementConfig_.captureTimeoutNs;
    case MeasurementState::POSE_SOLVE:     return measurementConfig_.solveTimeoutNs;
    case MeasurementState::VALIDATE:       return measurementConfig_.validateTimeoutNs;
    case MeasurementState::SAVE:           return measurementConfig_.saveTimeoutNs;

    case MeasurementState::IDLE:
    case MeasurementState::COMPLETE:
    case MeasurementState::FAILED:
        break;
    }
    return 0;   // 这三个状态不执行动作
}

// ---------------------------------------------------------------------------
// 失败处置
// ---------------------------------------------------------------------------

void MeasurementController::handleFailure(FailureKind kind,
                                          const data::ErrorInfo& deviceError,
                                          uint64_t nowNs)
{
    // ⚠ 先把原因记进 lastError_，再决定恢复动作。
    //
    // 顺序不能反。原因：**同状态重试（RETRY_IN_STATE）不改变状态、也不成功
    // 转换，于是没有任何一步会去更新 lastError_**。若不在入口处记录，
    // 一次"重试后仍失败"的现场就只剩下最后那条耗尽消息，而真正的原因
    //（"候选相机集合为空""三相机同步采集失败"……）被整条丢弃 ——
    // 排查时看到的是"次数用尽"，看不到"为什么这几次都没成"。
    //
    // 不担心覆盖：真正的终止原因由 failWith() 写入，回退成功则由
    // StateMachine::transition() 清空，二者都在本调用之内发生，顺序正确。
    data::ErrorInfo recorded = deviceError;
    if (recorded.timestampNs == 0)
    {
        recorded.timestampNs = nowNs;
    }
    lastError_ = recorded;

    // 失败轨迹（裁决 C-007）：本函数是全部失败的唯一入口，故"首次失败"
    // 的观测点就在这里 —— 且**只能**在这里。
    //
    // ⚠ 为什么不能放到 StateMachine::transition() 里：同状态重试
    // （§7.3 的 RETRY_IN_STATE）**不产生任何迁移**（applyRecovery 只把
    // actionOpen_ 置 false 就返回），而"首次失败"最常见的形态正是
    // 一次状态内重试。放到迁移里去记，会漏掉全部这一类。
    pendingCause_ = recorded;

    if (!firstFailureSeen_)
    {
        // 首个 handleFailure 即根因：后续的失败往往是它的**后果**
        //（重试仍失败、预算被耗尽）。见 FailureTrace.h 的"根因 vs 症状"。
        firstFailureSeen_          = true;
        failureTrace_.firstFailedState = stateMachine_.state();
        failureTrace_.firstError       = recorded;
    }

    applyRecovery(strategy_.recoveryFor(stateMachine_.state(), kind, deviceError),
                  nowNs);
}

void MeasurementController::applyRecovery(const Recovery& recovery,
                                          uint64_t nowNs)
{
    switch (recovery.action)
    {
    case RecoveryAction::RETRY_IN_STATE:
        // 状态不变，重来一次。**必须关闭动作窗口**：否则下一次
        // beginAttemptForAction() 会认为"本次动作仍在进行"而不记数，
        // 状态就会在同一个动作里无限重试（预算形同不存在）。
        actionOpen_ = false;
        return;

    case RecoveryAction::ROLLBACK:
        if (transitionTo(recovery.target, nowNs))
        {
            return;
        }
        // 回退被拒。两种原因必须区分，否则错误码会指错方向：
        //  · 9002 —— §7.4 的回退预算用尽（每边 2 次 / 总计 4 次）；
        //  · 其他 —— 目标状态的尝试次数已用尽（RETRY 与 ROLLBACK 在同一
        //    状态上竞争配额时的正常结果），此时应按目标状态的 §7.7
        //    超限后果继续处置，而不是伪报回退预算耗尽。
        if (retry_.lastError().code != 0)
        {
            failWith(retry_.lastError(), nowNs);
        }
        else
        {
            applyRecovery(strategy_.onAttemptsExhausted(recovery.target), nowNs);
        }
        return;

    case RecoveryAction::FAIL:
        failWith(recovery.error, nowNs);
        return;
    }
}

void MeasurementController::failWith(data::ErrorInfo error, uint64_t nowNs)
{
    if (error.timestampNs == 0)
    {
        error.timestampNs = nowNs;
    }
    lastError_ = error;

    if (stateMachine_.state() == MeasurementState::FAILED)
    {
        return;   // 已在终止态，无需也无法再转换
    }

    // 失败轨迹（裁决 C-007）：本函数是全部"终止为失败"的唯一收口。
    // finalFailedState 取**转换之前**的状态 —— 它回答的是"这套重试策略
    // 把任务耗在了哪一段"，写成 FAILED 就等于没回答。
    failureTrace_.finalFailedState = stateMachine_.state();
    failureTrace_.finalError       = error;

    // 失败包落盘（裁决 C-007 §2.3）。
    //
    // ⚠ 必须在 transitionTo(FAILED) **之前**：onEnteredState() 在终态会
    // 释放采集缓存并清空 bestFrame_，之后再组装记录只能拿到空帧。
    // 那是同一个释放点，见 buildRecord() 与 onEnteredState() 的说明。
    saveFailurePackage();

    if (!transitionTo(MeasurementState::FAILED, nowNs))
    {
        // 转换表不允许（例如自 COMPLETE 出发）。这是设计层面的不一致，
        // 如实记录错误本身而不伪造状态：把状态强置为 FAILED 会让
        // result.json 与状态记录互相矛盾（同 StateMachine 对
        // COMPLETE → FAILED 的禁止理由）。
        note("无法进入 FAILED：" + stateMachine_.lastError().message
             + "（原错误：" + error.message + "）");
    }
}

void MeasurementController::note(const std::string& text)
{
    if (std::find(notices_.begin(), notices_.end(), text) == notices_.end())
    {
        notices_.push_back(text);
    }
}

void MeasurementController::saveFailurePackage()
{
    if (recorder_ == nullptr)
    {
        // 与 stepSave 的处置一致：不把"没挂 Recorder"伪装成落盘成功。
        // 但**不**升级为失败 —— 任务已经因为别的原因失败了，
        // 缺 Recorder 只是让这次失败少一份留档。
        note("未挂载 Recorder（IRecorderSink），失败任务的原因与轨迹只存在于日志中");
        return;
    }

    failurePackage_ = true;
    data::MeasurementRecord record = buildRecord();
    failurePackage_ = false;

    // ⚠ task_.state 记的是**任务的结果**，不是"此刻处于哪个状态"。
    // 与 stepSave 把成功的包写成 COMPLETE 对称：失败包写成 FAILED。
    // 这一处此前是硬编码 COMPLETE（"失败的任务不产生结果包"），
    // 裁决 C-007 推翻了那句话，硬编码随之作废 —— 若留着，失败包
    // 会在 result.json 里声称自己是 COMPLETE 的结果。
    record.task.state = data::MeasurementState::FAILED;

    // 半成品结论如实带上（如果确实算出来过）。
    //
    // ⚠ 判据用 `poseResult_.success` 而**不是**"指针非空/字段非默认"：
    // 失败在 VALIDATE 的场景下，姿态是**算出来了但没通过验证**，
    // 那个数值是"为什么没通过"的第一手线索（重投影超限？偏航角离谱？）。
    // 若把 `record.task.result` 一律清成默认值，失败包就只能回答
    // "验证没过"，回答不了"差多少"。
    // 反之，`success == false` 时其余字段按 ShipPoseResult.h 的明文
    // **无意义**，写进包里等于制造噪声，故不带。
    if (poseResult_.success)
    {
        record.task.result     = poseResult_;
        record.task.validation = validationResult_;
    }

    // ② 失败包**不带原图**（C-007）。
    //
    // 判据不是"省空间"，而是失败包回答的问题是"为什么失败"，原图是
    // 几十 MB 级的**证据**、不是原因。Recorder::writeRawFrames 对空帧
    // 会跳过，故这一行同时是"不带原图"这条要求的全部实现 ——
    // 不需要给 IRecorderSink::save 加参数（C-002 刚改过该签名，不宜再动）。
    //
    // ⚠ bestQuality_/selectedScore 等**保留**：它们是"为什么失败"的
    // 关键线索（通道质量差 → 匹配不足 → PnP 不收敛），且都是几十字节。
    // 一起清掉会把失败包变成一张只有错误码的便条。
    record.bestFrame = data::MultiCameraFrame{};

    // 轨迹的最后一跳（X → FAILED）在此补上：它发生在 transitionTo() 里，
    // 而包必须更早写（见 withTerminalHop() 的说明）。
    record = withTerminalHop(std::move(record), data::MeasurementState::FAILED,
                             failureTrace_.finalError, failureTrace_.finalError.timestampNs);

    if (!recorder_->save(record))
    {
        // ③ **只记 note，绝不再触发 handleFailure**。
        //
        // 否则形成 "失败 → 失败包落盘失败 → handleFailure → 失败 →
        // 落盘失败 → …" 的无限递归：每一次递归都再写一次盘、
        // 再失败一次，最终以爆栈结束，而现场看到的是一堆重复的落盘错误。
        //
        // 这条路径不会被静默：note() 会进 notices_，由 UI 与日志呈现。
        note("失败任务的结果包落盘失败（原因与轨迹仅存在于日志中）");
    }
}

data::ErrorInfo MeasurementController::modelErrorOr(const std::string& what,
                                                    uint64_t nowNs) const
{
    // 机型库未加载：这是唯一能把"解算失败"归到 5001 的证据。
    // 见头文件 @param modelsAvailable 的说明 —— 算法接口只回一个 bool，
    // 这条根因在返回值里被吞掉了，只能由装配点补上。
    if (!modelsAvailable_)
    {
        return data::ErrorInfo{
            data::kErrModelMissing,
            what + "（机型库未加载，无法给出点表/尺度依据）",
            nowNs};
    }

    // 机型库已加载：不能冒充 5001（那会让现场去查一个不存在的模型问题）。
    // 归 9004 的理由：失败的具体成因（点表为空、匹配不足、PnP 退化……）
    // 在 `IPosePipeline` 内部判定后**只以一个 bool 返回**，控制器拿不到；
    // 而 ENG-09 §5.27 的 6000 段现有登记码（6001）语义是"重试次数用尽"，
    // 不是"解算失败"，套用会把过程当结论。
    return data::ErrorInfo{data::kErrStateFailure, what, nowNs};
}

// ---------------------------------------------------------------------------
// 观测接口
// ---------------------------------------------------------------------------

data::ErrorInfo MeasurementController::lastError() const
{
    return lastError_;
}

bool MeasurementController::degraded() const
{
    return degraded_;
}

data::ErrorInfo MeasurementController::degradationNotice() const
{
    return degradationNotice_;
}

int MeasurementController::availableCameraCount() const
{
    return availableCameras_;
}

data::CameraRole MeasurementController::selectedCamera() const
{
    return selectedCamera_;
}

data::TargetOffset MeasurementController::lastOffset() const
{
    return lastOffset_;
}

data::ImageQuality MeasurementController::bestQuality() const
{
    return bestQuality_;
}

data::ShipPoseResult MeasurementController::poseResult() const
{
    return poseResult_;
}

data::PoseValidationResult MeasurementController::validationResult() const
{
    return validationResult_;
}

data::MeasurementTask MeasurementController::task() const
{
    return task_;
}

std::vector<std::string> MeasurementController::notices() const
{
    return notices_;
}

uint64_t MeasurementController::remainingNs(uint64_t nowNs) const
{
    if (finished() || stateMachine_.state() == MeasurementState::IDLE)
    {
        return 0;
    }
    return retry_.remainingNs(nowNs);
}

int MeasurementController::attempts(data::MeasurementState state) const
{
    return retry_.attempts(state);
}

int MeasurementController::rollbackCount() const
{
    return retry_.rollbackCount();
}

bool MeasurementController::finished() const
{
    return strategy_.isTerminal(stateMachine_.state());
}

}  // namespace application
}  // namespace aircraft
