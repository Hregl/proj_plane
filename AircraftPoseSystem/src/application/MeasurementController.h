#pragma once

// ============================================================================
//  src/application/MeasurementController.h
//
//  依据：SYS-08 §3.4（职责）、§5（各状态详细设计）、§6（正常流程）、§7（重试与超时）、
//        §8（与 PreviewManager 关系）、§9（与线程关系）、§11（约束）
//        SYS-04 §4.5（**IF-SW-05 冻结的 UI 侧接口**）、§4.2（IF-SW-02）、
//               §4.4（IF-SW-04）、§2.2（IF-SW-01）
//        ENG-04 §10.1（职责：管理 StateMachine / Strategy / Algorithm / Recorder）
//        ENG-01 §9（本文件在 application 冻结清单内）
//        ENG-03 §12.6（P1 分类：依赖 device+preview+optical+algorithm，无 Qt）
//        ENG-10 §5.3（配置注入：算法阈值由本类在构造时注入）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类是**唯一**驱动状态机的东西，也是 UI 唯一允许触碰的对象            │
//  │（SYS-04 §4.5："UI 只与 MeasurementController 交互"）。               │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠⚠ 与 7.md 的两处结构性差异（都是 7.md 的实现不可用，而非风格选择）：
//
//  1) **状态机是事件驱动的，不是阻塞循环。**
//     7.md 的 startMeasurement() 里写 `while (running_) { ... }`，用普通
//     `bool running_` 控制。这段代码有两个独立的致命问题：
//       · 它要跑在 UI 线程上（MainWindow 直接调用 startMeasurement），
//         阻塞循环会让整个界面在测量的 60 秒内完全冻结 —— 而项目要求
//         转台运动、状态、预览都要实时可见（SYS-08 §7.5"降级必须可见"）；
//       · `bool` 跨线程读写是数据竞争（未同步访问非原子对象），
//         停止按钮无法可靠终止循环，UB 之下连"能不能停下来"都不确定。
//     本实现的依据是 SYS-08 §9（"状态机本身运行于 Application 线程"）
//     + §7.6 约束 3（"状态机必须在每个事件循环中检查 deadlineExceeded"）
//     —— "每个事件循环"这一措辞本身就说明状态机是被**反复进入**的，
//     而不是占着一个循环不放手。故本类只提供 tick()，由宿主的事件循环
//     （009 阶段的 QTimer）反复调用，单次调用为微秒级。
//
//  2) **tick() 接收当前时刻，自己不读时钟。**
//     与 RetryManager 同一理由（见 RetryManager.h）：注入时刻才能写出
//     "同一时刻两个判断只有一个生效"这类用例（SYS-08 §10 用例 7）。
//     若本类内部取 CLOCK_MONOTONIC，同一次 tick 中的多个判断会落在
//     不同时刻，§7.6 约束 3 的"先到者生效"就无从验证。
//
//  ⚠ 与 SYS-04 §4.5 冻结签名的差异（一处，且不影响调用方）：
//     `MeasurementState state()` → `data::MeasurementState state() const`。
//     仅增加 const 限定：原签名的**每一个调用点**（含经由非 const 指针调用）
//     都仍然合法，故 ui 层按 ICD 原文书写不会编译失败。加 const 是因为
//     UI 通常在只读上下文里查询状态，而 const 正确性由编译器强制。
//     另按 ENG-09 §2.1 补 `data::` 限定（ICD 代码段省略了命名空间）。
//
//  ⚠ 未纳入本类的冻结接口：`ITriggerController` 虽在 IF-SW-01 的连线表内，
//     但 SYS-08 §5 的 12 个状态里没有任何一个需要主动触发 —— 触发时序由
//     CameraSynchronizer 在设备层完成（ENG-05 §10）。故此处只把它作为
//     **可选的降级通知通道**（§7.5 的 3001），默认 nullptr。
// ============================================================================

#include <cstdint>
#include <vector>

#include "algorithm/pipeline/IPosePipeline.h"
#include "application/AlignmentController.h"
#include "application/MeasurementStrategy.h"
#include "application/RetryManager.h"
#include "application/StateMachine.h"
#include "data/CameraCalibration.h"
#include "data/CameraRole.h"
#include "data/DetectionResult.h"
#include "data/ErrorInfo.h"
#include "data/FailureTrace.h"
#include "data/ImageFrame.h"
#include "data/ImageQuality.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementRecord.h"
#include "data/MeasurementSelectionResult.h"
#include "data/MeasurementStatistics.h"
#include "data/MeasurementTask.h"
#include "data/MultiCameraFrame.h"
#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"
#include "data/TargetOffset.h"
#include "data/TurntableConfig.h"
#include "device/camera/IMultiCameraManager.h"
#include "device/trigger/ITriggerController.h"
#include "device/turntable/ITurntableController.h"
#include "optical/OpticalRig.h"
#include "preview/PreviewManager.h"

namespace aircraft
{
namespace application
{

/// 测量结果落盘通道（IF-THR-05：MeasurementController → RecorderWorker）。
///
/// ⚠ 为什么是抽象接口而不是直接用 infrastructure::Recorder：
/// ENG-03 §12.6 的依赖表里 **application 不依赖 infrastructure**。
/// 若在此直接引入 Recorder 类型，依赖方向会变成 application → infrastructure，
/// 而 infrastructure 又要用 data 的类型、Logger 又要被所有层调用 ——
/// 这条反向依赖会把 ENG-01 §18 的分层彻底打乱（同 MeasurementState 归属
/// data 的裁决 C-13 是同一类问题的同一类解法）。
/// 由 app/ 在 009 阶段把 infrastructure 的实现从外部注入。
///
/// 返回 false 表示落盘失败。按 SYS-08 §7.3，SAVE 的失败上限是 3 次。
class IRecorderSink
{
public:
    virtual ~IRecorderSink() = default;

    /// 保存一次测量的**完整记录**（SYS-08 §5.10 的 measurement_xxx/）。
    ///
    /// ⚠ 入参由 `MeasurementTask`（4 字段）改为 `MeasurementRecord`
    /// （裁决 C-002 的交付物；2026-09-23 由评审确认扩展）。
    /// 改动理由不是"多带几个字段"，而是旧签名使结果包**结构性地不完整**：
    /// SYS-04 §6.4 要求 result.json 含 camera_used / selected_camera /
    /// score / quality / feature_count / match_count，而 `MeasurementTask`
    /// 里一个都没有承载者 —— 于是 `Recorder` 只能把这些字段名写进
    /// `missing_required_fields`（见 infrastructure/recorder/Recorder.h，
    /// 该文件早已登记"是否扩展本入参"待裁决，本条即其结论）。
    /// 同理 cam25|50|100.raw 与 turntable.json 也因入参不够而写不出。
    virtual bool save(const data::MeasurementRecord& record) = 0;
};

/// 测量任务总控制（SYS-08 的 12 状态机宿主）。
///
/// ⚠ 同时实现 `algorithm::IPipelineObserver`：算法链在产生匹配统计量的
/// 那一刻把 `MeasurementStatistics` **推**给它（裁决 C-002 /
/// IPipelineObserver.h）。这是 application → algorithm 方向上的正常依赖
/// （ENG-01 §18 允许），不需要任何适配器。
class MeasurementController : public algorithm::IPipelineObserver
{
public:
    /// 构造。所有协作者均由外部注入（ENG-10 §5.3：算法阈值与本类同源注入）。
    ///
    /// @param cameras         三相机管理器（IF-SW-01）。**非空**。
    /// @param turntable       转台控制器（IF-SW-01）。**非空**。
    /// @param pipeline        算法链（IF-SW-02）。**非空**。
    /// @param rig             光机刚体（提供各焦段的 CameraCalibration）。
    ///                        **非空**；空标定不会被当作"可用的零标定"，
    ///                        而是使相关状态以显式错误停下（见 .cpp）。
    /// @param measurementCfg  测量配置（重试/超时/评分的唯一数据源，C-20）。
    ///                        构造时取副本：§6.8 要求配置在任务级冻结，
    ///                        取副本可避免任务进行中被外部改写。
    /// @param turntableCfg    转台配置（行程限位 + centerThreshold）。
    /// @param preview         预览管理器（IF-SW-04）。可为空 ——
    ///                        为空时本类不通知显示源，测量流程照常。
    ///                        为空的用途是无 GUI 的离线/测试运行（ENG-03 §20 第 6 条）。
    /// @param trigger         触发控制器。可为空，仅用于 §7.5 的降级通知。
    /// @param recorder        结果落盘通道。可为空 —— 为空时 SAVE 状态
    ///                        记录一条 notice 后继续（不把"没挂 Recorder"
    ///                        伪装成落盘成功，见 .cpp）。
    /// @param modelsAvailable 机型库是否加载成功（裁决 C-006）。
    ///
    /// ⚠ 这个 bool 之所以要由装配点注入，是因为 `IPosePipeline::solvePose`
    /// 的返回类型是纯 `bool`（IF-SW-02 冻结，不得改签名）：机型库未加载与
    /// 真正的 PnP 失败在返回值上**不可区分**，控制器仅凭返回值无法把
    /// "没装机型库"这条根因归到 5001。装配点（app/SystemInitializer）是
    /// 唯一知道加载结果的地方，故由它把这个事实交给本类。
    ///
    /// 默认 true（"假定机型库正常"）：这样未显式传参的既有调用方在 PnP
    /// 失败时得到 9004 兜底码，而**不会**被误报为 5001。宁可少报一个
    /// 具体码，也不要凭空造出一个不成立的根因。
    MeasurementController(device::IMultiCameraManager& cameras,
                          device::ITurntableController& turntable,
                          algorithm::IPosePipeline& pipeline,
                          const optical::OpticalRig& rig,
                          const data::MeasurementConfig& measurementCfg,
                          const data::TurntableConfig& turntableCfg,
                          preview::PreviewManager* preview = nullptr,
                          device::ITriggerController* trigger = nullptr,
                          IRecorderSink* recorder = nullptr,
                          bool modelsAvailable = true);

    MeasurementController(const MeasurementController&)            = delete;
    MeasurementController& operator=(const MeasurementController&) = delete;

    /// ⚠ 必须显式注销观察者：`pipeline_` 是外部注入的**长寿命**协作者
    /// （app 装配，生命周期长于本对象），若不注销，本对象析构后
    /// Pipeline 里就留着一个指向已释放内存的指针，而它会在下一次
    /// `solvePose` 里被调用 —— 崩在算法层，根因却在本类的析构。
    ~MeasurementController() override;

    // ---- SYS-04 §4.5 冻结的 UI 接口 ---------------------------------------

    /// 启动一次测量任务。
    ///
    /// 非阻塞：只做状态重置与 IDLE → SEARCH 的转换，随后立即返回。
    /// 真正的工作由宿主事件循环反复调用 tick() 推进。
    ///
    /// 前置：相机与转台已由 app/ 初始化（ENG-08 §4 的启动顺序）。
    /// 若启动时可用相机数 ≤ 1（§7.5 下限），本次调用即以 1001 进入 FAILED。
    void startMeasurement();

    /// 同上，但由调用方注入当前时刻（可测性，同 tick()）。
    /// 冻结签名的重载只是取 `data::monotonicNowNs()` 后转调本函数 ——
    /// 这样 SYS-04 §4.5 的签名与 M1 之后的确定性测试可以同时成立。
    void startMeasurement(uint64_t nowNs);

    /// 停止当前测量任务（SYS-08 §7.3 的 9003 人工取消）。
    ///
    /// 会同时发出转台停止命令 —— 停止按钮的物理语义必须包含"让机构停下来"，
    /// 只把软件状态改成 FAILED 而让转台继续走是危险的。
    /// 对终止态与 IDLE 是无操作。
    void stopMeasurement();

    /// 当前状态（SYS-04 §4.5）。见文件头关于 const 的说明。
    data::MeasurementState state() const;

    // ---- 事件循环入口 -----------------------------------------------------

    /// 推进状态机一步。
    ///
    /// @param nowNs 当前时刻，ns，CLOCK_MONOTONIC（ENG-09 §2.5）。
    /// @return 本次调用是否**改变了状态**（供宿主决定是否刷新界面）。
    ///
    /// 调用约定：宿主以固定节拍调用（建议 ≥20 Hz）。调用本身不做任何
    /// 阻塞等待 —— 需要等待物理过程（转台运动、稳定）的状态靠**下次调用**
    /// 观察结果，等待的时限由 MeasurementConfig 的各状态超时约束。
    bool tick(uint64_t nowNs);

    // ---- 观测接口（日志 / 界面 / 测试）------------------------------------

    /// 最近一次失败的原因。0 表示当前无错误。
    data::ErrorInfo lastError() const;

    /// §7.5 的降级标记。true 表示当前任务在"少一台相机"或"触发降级"下运行，
    /// **必须可见**（§7.5 原文要求界面上能看到）。
    bool degraded() const;

    /// 降级的具体记录（1002 / 3001）。未降级时 code 为 0。
    data::ErrorInfo degradationNotice() const;

    /// 未被降级/禁用标记为不可用的相机数量（§7.5 的三个分支依据）。
    /// 取自最近一次采集的 MultiCameraFrame 中图像非空的通道数。
    int availableCameraCount() const;

    /// MEASURE_SELECT 阶段选出的测量焦段。尚未选出时为 CAM25。
    data::CameraRole selectedCamera() const;

    /// 最近一次算出的目标像素偏差（C-11：原点为**图像中心**）。
    data::TargetOffset lastOffset() const;

    /// CAPTURE 阶段**评分最高那一帧**的质量指标（不是"最后一帧"的质量）。
    ///
    /// ⚠ 名称由 `lastQuality()` 改为 `bestQuality()`（C-02 §2.1）：
    /// 旧名与它描述的对象不符 —— 该值来自 `selectBestFrame()` 对最佳帧的评分，
    /// 而 PnP 此前实际解算的是**最后一次**采集。名称错误曾使这个不一致
    /// 看起来像是正常的（见 V2.1-C02_实施设计说明.md §1.1）。
    data::ImageQuality bestQuality() const;

    /// 最近一次姿态解算结果（COMPLETE 后即为本次任务的输出）。
    data::ShipPoseResult poseResult() const;

    /// 最近一次验证结论。
    data::PoseValidationResult validationResult() const;

    /// 本次任务的完整记录（SAVE 状态的落盘对象）。
    data::MeasurementTask task() const;

    // ---- algorithm::IPipelineObserver -------------------------------------

    /// 算法链推送的匹配统计量（裁决 C-002）。
    ///
    /// ⚠ 本方法**不供外部调用**，只由 `pipeline_` 在 `solvePose` 内部调用。
    /// 它是 public 的仅仅因为 `IPipelineObserver` 是公开基类；
    /// 手工调用它等于伪造一次统计事实。
    void onStatistics(const data::MeasurementStatistics& statistics) override;

    /// 最近一次被推送的匹配统计量。
    data::MeasurementStatistics statistics() const;

    /// 本次任务是否**收到过**统计量推送。
    ///
    /// ⚠ 存在的理由是"全零的统计量在这里是合法值"：没有这个标志，
    /// "算法没推送"与"算法推送了但一切为 0"在结果包里长得一模一样，
    /// 而前者意味着记录不完整。详见 IPipelineObserver.h 的同名说明。
    bool statisticsReceived() const;

    /// 阻塞/异常说明。例如"未挂载 Recorder"、"转台行程未配置"。
    /// 与 lastError() 的区别：这些不是错误，是**如实记录的偏离**，
    /// 使"某个保护没生效"成为可见事实而非静默假设。
    std::vector<std::string> notices() const;

    /// 剩余任务时间（ns）。已超时或无活动任务时返回 0。
    uint64_t remainingNs(uint64_t nowNs) const;

    /// 某状态已消耗的尝试次数（来自 RetryManager，见 C-20）。
    int attempts(data::MeasurementState state) const;

    /// 已消耗的总回退次数（§7.4 上限 4）。
    int rollbackCount() const;

private:
    // ---- 单步流程 ---------------------------------------------------------

    /// 终止态判定（COMPLETE / FAILED）。
    bool finished() const;

    /// 转换并执行进入副作用。转换被拒时返回 false（原因在 lastError_）。
    bool transitionTo(data::MeasurementState next, uint64_t nowNs);

    /// 进入某状态后的公共副作用（预览通知、任务记录、动作窗口复位）。
    void onEnteredState(data::MeasurementState state);

    /// 开始一次"状态动作"。返回 false 表示次数已用尽且已按 §7.7 处置。
    ///
    /// ⚠ 计数语义（本类与 RetryManager 的接缝，必须与 SYS-08 §7.6 约束 2
    /// 严格一致）：StateMachine::transition() 在**进入**状态时已记过一次尝试，
    /// 故进入后的第一次动作**不再**记数；此后每次动作都要记一次。
    /// 若不这样，ALIGN 的首次移动会被记两次，8 次上限退化为 4 次实际移动
    /// —— 而 8 次这个数正是 §7.3 时间预算（8 × 2.5 s = 20 s）的依据。
    ///
    /// 用 actionOpen_ 而非"每次调用都记数"的原因：转台运动与稳定等待要跨
    /// **多次 tick** 才完成，而它们是**一次**动作。若每个 tick 都记一次数，
    /// ALIGN 的 8 次尝试会在 8 个 tick 内（毫秒级）耗尽，转台根本来不及动。
    bool beginAttemptForAction(uint64_t nowNs);

    /// 当前动作是否已超出该状态的超时（§7.1 的 T_state）。
    bool actionTimedOut(uint64_t nowNs) const;

    /// 该状态的超时值。取值唯一来源为 MeasurementConfig（C-20）。
    uint64_t stateTimeout(data::MeasurementState state) const;

    // ---- 各状态的单步执行（SYS-08 §5.x）----------------------------------

    void stepSearch(uint64_t nowNs);
    void stepTargetFound(uint64_t nowNs);
    void stepAlign(uint64_t nowNs);
    void stepStabilize(uint64_t nowNs);
    void stepMeasureSelect(uint64_t nowNs);
    void stepCapture(uint64_t nowNs);
    void stepPoseSolve(uint64_t nowNs);
    void stepValidate(uint64_t nowNs);
    void stepSave(uint64_t nowNs);

    // ---- 协作者调用 -------------------------------------------------------

    /// 采集一帧同步三相机图像。失败时返回 false 并填写 lastError_。
    bool acquire(data::MultiCameraFrame& frame, uint64_t nowNs);

    /// 按 §7.5 统计可用相机数并设置降级状态。
    /// 返回 false 表示已达下限（≤1 路），调用方须以 1001 失败。
    bool updateDegradation(const data::MultiCameraFrame& frame, uint64_t nowNs);

    /// 在指定焦段上检测目标（§5.2 的 YOLO → DetectionResult）。
    bool detectOn(const data::MultiCameraFrame& frame,
                  data::CameraRole role,
                  data::DetectionResult& detection,
                  uint64_t nowNs);

    /// 由检测框算出目标像素偏差（C-11：原点为图像中心；ENG-09 §2.4：Y 向下为正）。
    /// 返回 false 表示框或图像退化（尺寸为 0 / 非有限），无法给出偏差。
    bool offsetFrom(const data::DetectionResult& detection,
                    const data::ImageFrame& frame,
                    data::TargetOffset& offset) const;

    /// 取某焦段的标定（来自 OpticalRig；未配置时返回默认构造的空标定）。
    data::CameraCalibration calibrationOf(data::CameraRole role) const;

    /// §8 的 AUTO 映射：当前状态应当显示哪个焦段。
    data::CameraRole displayCameraFor(data::MeasurementState state) const;

    /// §7.3 升级规则第 2 条：换用次优相机（排除当前相机后重新选择）。
    /// 返回 false 表示已无其他候选。
    bool switchToNextCamera(const data::MultiCameraFrame& frame);

    // ---- 失败处置 ---------------------------------------------------------

    /// 按 MeasurementStrategy 给出的方案执行。
    void applyRecovery(const Recovery& recovery, uint64_t nowNs);

    /// 直接进入 FAILED 并记录错误（error.timestampNs 由本函数补齐）。
    void failWith(data::ErrorInfo error, uint64_t nowNs);

    /// @param kind 为 §7.2 的分类；deviceError 由设备层或算法层提供。
    void handleFailure(FailureKind kind,
                       const data::ErrorInfo& deviceError,
                       uint64_t nowNs);

    /// 解算类失败的错误码归类（裁决 C-006）。
    ///
    /// 判据：`modelsAvailable_ == false` → 5001（机型库缺失）；
    ///       否则 → 9004（兜底，理由见 .cpp 文件头的归类优先级）。
    ///
    /// ⚠ 为什么必须带 `modelsAvailable_` 这一半判据：算法接口只回一个
    /// `bool`（见构造函数的 @param 说明），仅凭它无法区分两种失败。
    /// 另一半判据保证**不误报**：机型库已加载而 PnP 仍然失败时，
    /// 码是 9004 而不是 5001 —— 把一次真实的解算失败谎报成"没装模型"，
    /// 会让现场去查一个不存在的问题。
    data::ErrorInfo modelErrorOr(const std::string& what, uint64_t nowNs) const;

    /// 记录一条 notice（去重）。
    void note(const std::string& text);

    /// 失败任务的落盘（裁决 C-007）。
    ///
    /// ⚠ 三条纪律，缺一条都会造成比"没有失败包"更糟的后果：
    ///  ① **必须在 `transitionTo(FAILED)` 之前调用** —— `onEnteredState()`
    ///     在终态会释放采集缓存与 bestFrame_，之后再组装记录就只能拿到空帧
    ///     （与 buildRecord() 的注释是同一个释放点）。
    ///  ② **不带原图**：失败记录的 bestFrame 置空。判据不是"省空间"而是
    ///     §C-007 的原文 —— 失败包用于定性与归因，原图是几十 MB 级的证据，
    ///     没有它也能回答"为什么失败"。Recorder 对空帧会跳过写盘。
    ///  ③ **落盘失败只记 note，绝不再触发 handleFailure** —— 否则
    ///     "失败 → 落盘失败 → 失败"会无限递归，把一次可解释的失败
    ///     变成一个爆栈。
    void saveFailurePackage();

    /// 组装本次测量的完整记录（SAVE 的落盘对象，裁决 C-002）。
    ///
    /// ⚠ 必须在 `onEnteredState()` 对 COMPLETE / FAILED 的**释放点之前**调用：
    /// 那里会 `bestFrame_ = {}`，之后再组装就只能拿到一个空帧
    /// （记录里最要紧的"被解算的那一帧"会变成三张空图，且不报错）。
    data::MeasurementRecord buildRecord() const;

    /// 返回一份"轨迹补上即将发生的终止迁移"的记录**副本**（裁决 C-007）。
    ///
    /// ⚠ 为什么需要它 —— 记录写盘的时刻**必然早于**终止迁移：
    ///   包必须在 `onEnteredState()` 释放采集缓存（`bestFrame_ = {}`）之前
    ///   写，而那次释放正是 `transitionTo(COMPLETE|FAILED)` 触发的。
    ///   于是 `buildRecord()` 拿到的 `failureTrace_` 天然差最后一跳：
    ///   文件里写着 `"state": "FAILED"`，而它自己的轨迹停在
    ///   `MEASURE_SELECT` —— **同一份文件内部自相矛盾**，读者必须靠
    ///   `finalFailedState` / `task.state` 手工把最后一跳接回去。
    ///
    /// 两种"修法"都不成立，故只能在副本上补：
    ///   · 不能先把这一跳塞进 `failureTrace_` —— 终止迁移真的发生后
    ///     `transitionTo()` 会再记一条同样的，轨迹里出现重复；
    ///   · 不能等 `transitionTo()` 之后再写包 —— 那时缓存已释放，
    ///     记录里的 bestFrame 与统计量全空（比缺一跳严重得多）。
    ///
    /// 合法性由 `StateMachine::isLegalTransition()` 判定：**不可达时不补**。
    /// 留下一条没有发生的迁移，比缺一跳到"轨迹终点不是终点"更糟。
    ///
    /// @param record        已组装好的记录（按值传入，在副本上追加）
    /// @param next          即将迁移到的状态（COMPLETE / FAILED）
    /// @param terminalError 触发本次终止的错误；成功路径传默认值（code == 0，
    ///                      与 StateTransition::error "正常前进时为 0" 一致）
    data::MeasurementRecord withTerminalHop(data::MeasurementRecord record,
                                           data::MeasurementState next,
                                           const data::ErrorInfo& terminalError,
                                           uint64_t nowNs) const;

    // ---- 协作者（全部外部注入，本类不拥有）--------------------------------

    device::IMultiCameraManager&  cameras_;
    device::ITurntableController& turntable_;
    algorithm::IPosePipeline&     pipeline_;
    const optical::OpticalRig&    rig_;
    preview::PreviewManager*      preview_  = nullptr;
    device::ITriggerController*   trigger_  = nullptr;
    IRecorderSink*                recorder_ = nullptr;

    // ---- 配置快照（任务级冻结，ENG-09 §6.8）-------------------------------

    data::MeasurementConfig measurementConfig_;
    data::TurntableConfig   turntableConfig_;

    // ---- 状态机三件套（SYS-08 §7.6 / §3.3）--------------------------------

    RetryManager        retry_;
    StateMachine        stateMachine_;
    MeasurementStrategy strategy_;
    AlignmentController alignment_;

    // ---- 任务级运行数据 ---------------------------------------------------

    /// 本次任务的记录（SAVE 的落盘对象）。
    data::MeasurementTask task_;

    /// 最近一次采集的帧。作为 §7.3 升级规则"重试必须换输入"的比较基准，
    /// 也是换相机重选（switchToNextCamera）的输入。
    data::MultiCameraFrame lastFrame_;

    /// SEARCH 阶段检出的目标（供 TARGET_FOUND 计算偏差与尺度）。
    data::DetectionResult pendingDetection_;

    /// 最近一次算出的偏差 / 质量 / 姿态 / 验证。
    data::TargetOffset         lastOffset_;
    data::ShipPoseResult       poseResult_;
    data::PoseValidationResult validationResult_;

    /// CAPTURE 阶段**评分最高那一帧**的质量（名称由 `lastQuality_` 改为
    /// `bestQuality_`，理由见同名的公开访问器）。
    ///
    /// ⚠ 本字段与被解算的那一帧必须**同源**：若 PnP 用的是别的帧，
    /// 记录里的质量就是错的，且不报错（C-02 §1.1 的 D-C02-1）。
    data::ImageQuality bestQuality_;

    /// MEASURE_SELECT 选出的测量焦段。
    data::CameraRole selectedCamera_ = data::CameraRole::CAM25;

    /// MEASURE_SELECT 的完整结论（选中角色 + 得分），供结果包记录。
    /// 此前该结论是 `stepMeasureSelect()` 的**局部变量**，函数返回即丢弃
    /// （C-02 §1.5 的 D-C02-5），故 record 的 `selectedScore` 无来源。
    data::MeasurementSelectionResult selection_;

    /// 算法链推送来的匹配统计量（裁决 C-002）。
    ///
    /// ⚠ 与 `statisticsReceived_` 必须成对使用：`statistics_` 的默认值
    /// （全零）本身是合法统计量，单看它无法区分"没推送"与"推送了 0"。
    data::MeasurementStatistics statistics_;
    bool                        statisticsReceived_ = false;

    // ---- CAPTURE 的采集缓存（C-02 §2.1）----------------------------------
    //
    // ⚠ 以下三个字段的**语义已按 C-02 冻结**，改名与类型变更见设计文档 §2.1：
    //   capturedFrames_  待改为 `std::vector<data::MultiCameraFrame>`
    //                    （现状只留选定焦段一路，见 D-C02-2）
    //   bestFrame_       **新增**：产出该姿态的那一次采集（三路齐）
    //   bestAcqIndex_    `bestFrameIndex_` 改名 + 语义收紧为"capturedFrames_ 的
    //                    采集下标"；旧名"帧下标"掩盖了它与评分视图下标的区别
    //                    （D-C02-3：空帧会破坏二者的对应）
    //
    // ⚠ 旧注释"CAPTURE 阶段选出的最佳帧（供 POSE_SOLVE 使用）"是**错的**：
    // 它说的既不是 capturedFrames_ 的用途（那是候选集，不是选出的帧），
    // 也不是当时的行为（PnP 用的是 lastFrame_）。描述意图而非实现的注释
    // 比没有注释更危险——下一个人会照它推理（D-C02-4）。

    /// 采集到的多帧（每次采集留**三路**；数量 = captureFrameCount）。
    std::vector<data::MultiCameraFrame> capturedFrames_;

    /// 产出该姿态的那一次采集（三路齐，同一次曝光）。
    /// POSE_SOLVE 的输入即本字段（当前实现用 lastFrame_，属 D-C02-1 缺陷）。
    data::MultiCameraFrame bestFrame_;

    /// bestFrame_ 在 capturedFrames_ 中的下标；-1 表示尚未选出。
    int bestAcqIndex_ = -1;

    /// 进入当前状态时 transition() 记的那次尝试尚未被"动作"用掉。
    /// 语义见 beginAttemptForAction()。
    bool pendingEntryAttempt_ = false;

    /// 当前动作是否已开始（且尚未结束）。
    /// false 表示下一次 beginAttemptForAction() 需要重新消耗一次尝试。
    bool actionOpen_ = false;

    /// 当前动作的开始时刻。用于 §7.1 的状态级超时判定。
    uint64_t actionStartNs_ = 0;

    /// ALIGN：转台指令是否已发出、正在等待到位。
    /// 存在的必要性：转台运动是物理过程，一个动作要跨多次 tick 才完成；
    /// 没有这个标记就无法区分"该发指令"与"该等它走完"。
    bool moveIssued_ = false;

    /// STABILIZE：连续稳定帧计数（SYS-08 §5.5 的 N ≥ 3）。
    int stableFrames_ = 0;

    /// §7.5 的降级状态。degraded_ 为 true 时必须可见（界面 + 日志 + result.json）。
    bool             degraded_           = false;
    data::ErrorInfo  degradationNotice_;
    int              availableCameras_   = 0;

    /// 机型库是否加载成功（装配点注入，裁决 C-006）。
    /// 唯一的用途是 modelErrorOr() 的归类判据，不参与任何流程决策 ——
    /// 机型库缺失不改变重试/回退路径，只改变失败**怎么记**。
    bool modelsAvailable_ = true;

    /// 最近一次失败。
    data::ErrorInfo lastError_;

    // ---- 失败轨迹（裁决 C-007）-------------------------------------------
    //
    // ⚠ 填充点为什么是"本类的三个既有收口"而不是 `StateMachine::transition()`：
    //   ① `StateTransition::error` 的语义是"导致这次迁移的错误"，而
    //      `StateMachine` 在成功迁移时拿不到它 —— 那个错误此刻在本类的
    //      `pendingCause_` 里，状态机不可见；
    //   ② "首次失败"在 `transition()` 里**根本不可观测**：同状态重试
    //      （RETRY_IN_STATE）不产生任何迁移，而 firstFailedState/firstError
    //      恰恰就是首次失败。
    // 三个落点（transitionTo / handleFailure / failWith）都是唯一收口，
    // 不是散点插桩 —— 本类所有的迁移都经 transitionTo()，所有的失败都经
    // handleFailure()，所有的终止都经 failWith()。

    /// 本次任务的失败轨迹。在 startMeasurement() 里随状态机一起清零。
    data::FailureTrace failureTrace_;

    /// **挂起的失败原因**：由 handleFailure() 写入，由紧接着的那次
    /// transitionTo() 取走并挂到该条 StateTransition 上。
    ///
    /// ⚠ 它是必要的中间态而不是冗余：失败原因（handleFailure 的入参）与
    /// 迁移（applyRecovery 里的 transitionTo）发生在**两个不同的调用**里，
    /// 中间还隔着策略决策。没有它，轨迹里的每条迁移就只能带一个空 error，
    /// "哪一步由失败触发"这个判据随之失效。
    /// 每次迁移后立即清空 —— 否则一次失败的原因会黏到后续所有正常前进的
    /// 迁移上，轨迹反而比没有更误导。
    data::ErrorInfo pendingCause_;

    /// 本次 `buildRecord()` 是否在为**失败包**组装（裁决 C-007）。
    ///
    /// ⚠ 这一个 bool 决定 `record.failure` 填什么：失败包填完整的
    /// FailureTrace（含根因），成功包只填 history（路径）。详见 .cpp。
    /// 之所以用成员而不是给 buildRecord() 加参数：调用点有两处
    /// （stepSave 与 saveFailurePackage），这个区分是**组装语境**的属性，
    /// 而不是记录的属性，放在成员上更不容易在新增调用点时漏传。
    bool failurePackage_ = false;

    /// 是否已经记下首次失败（C-007）。
    ///
    /// ⚠ 为什么不用 `failureTrace_.firstError.code == 0` 当判据（这是与
    /// 实施计划的一处偏离，理由如下）：**并非每次失败都带码**。
    /// 裁决 C-006 明确保留了"验证不通过 → 排除相机并回退"这条 code == 0
    /// 的事实记录；若它恰好是本次任务的第一次 handleFailure()，
    /// 用 code 判据会认为"还没记过首次失败"，于是下一次真正的失败会把它
    /// 覆盖掉 —— 而那次覆盖掉的才是根因。显式布尔标志不受码值影响。
    bool firstFailureSeen_ = false;

    /// 如实记录的偏离（不静默）。
    std::vector<std::string> notices_;
};

}  // namespace application
}  // namespace aircraft
