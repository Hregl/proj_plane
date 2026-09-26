#pragma once

// ============================================================================
//  src/app/SystemInitializer.h
//
//  依据：ENG-01 §14（app 模块：依赖创建 / 系统启动）
//        ENG-02 §15（冻结的对象创建顺序）、§16（生命周期）
//        ENG-08 §3（CAM25 真实 + CAM50/100 虚拟的切换点）
//        ENG-10 §4.4（match_stats 的读写时机）、§4.6（解析失败的隔离）
//        ENG-10 §5.1（配置注入矩阵）、§5.2（注入后不可变）、§5.3（三档失败）
//        SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕（降级必须可见）、§9（状态机运行于 Application 线程）
//        10.md（009 系统集成与依赖注入）、11.md（010 首次完整闭环）
//
//  职责：把 ApplicationContext 里的对象**按序造出来、初始化、连上**，
//  并在运行期提供唯一的事件循环入口 tick()。
//
//  ---- 本类是全工程唯一"知道所有具体实现类"的地方 ----
//
//  ENG-08 §3 要求"虚拟/真实设备的切换点唯一"，ENG-10 §5.1 要求"配置注入
//  集中在一处"。这两条落在这个文件里：把 `VirtualCameraBackend` 换成
//  `ImvCameraBackend`、把 `VirtualTurntable` 换成 `PekoTurntableController`，
//  只需改本文件的 createDevices() 一段，其他任何文件都不动。
//  这也是 010 阶段"虚拟设备全通、真实设备未接"这件事不至于失控的原因。
//
//  ---- tick() 为什么要顺手抓一帧"预览用的图像" ----
//
//  这是 009 装配时才发现的一处**真实空洞**：整条采集链上只有
//  MeasurementController 会调用 `cameras.capture()`
//  （见其 stepSearch/stepCapture），而在 IDLE 状态下状态机不推进任何
//  采集动作。于是"程序刚启动、还没点测量"时，PreviewManager 永远拿不到帧，
//  界面上始终是"无图像"占位 —— 而 PreviewManager 的注释写着它服务的是
//  "实时预览"，SYS-08 §8 的 AUTO 映射表也把 IDLE 映射到 CAM25。
//  一个只在测量时才出图的预览，与其说是预览，不如说是一个 60 秒一次的
//  事后回放。
//
//  故 tick() 在**空闲态**（IDLE / COMPLETE / FAILED）额外采集一帧并提交给
//  预览。测量进行中则由 MeasurementController 负责
//  （它每拍自己采集并通知预览）。
//
//  ---- 空闲补帧的**准确**规则（R04 收尾，2026-09-24）----
//
//  两条，缺一不可，且都是可验证的：
//    ① 空闲与否按**推进后**的状态判定（`controller->tick()` 之后再读 state）：
//       一拍之内控制器可能刚刚转入 COMPLETE / FAILED，那正是操作者最需要
//       看到画面的时刻；
//    ② **转入终态当拍统一跳过空闲补帧，下一拍恢复**；这能避免 ALIGN 等
//       已采集分支重复补帧。
//
//       ⚠ 这里原先给的理由是"该拍的活动态采集已经产出并提交了帧，再补一次
//          会让同一拍出现两次 `capture()`" —— **那个理由不成立**：它只对
//          "当拍采集过的分支"（如 SEARCH / ALIGN / CAPTURE）为真，而
//          `POSE_SOLVE` / `VALIDATE` / `SAVE` 这些分支**会推进状态却不采集**
//          （它们消费的是上一拍留下的帧）。规则本身是对的（按**状态是否
//          推进**判定，而不是按"该拍有没有采集"判定），错的是把"推进"
//          解释成了"本拍已经采集"。故此处改为陈述规则自身。
//
//  ⚠ 这里原先写的是一条**过强且不成立**的规范性主张："测量进行中不得重复
//    采集 ⇒ 任一 tick 至多一次 capture()"。它有两处错：
//      · `stepCapture` 本来就一拍内连采 captureFrameCount（默认 5）帧；
//      · 空闲判定读的是推进后状态，故"活动态采集成功后当拍转入终态"那一拍
//        会既采集又补帧 —— 而这个组合在真实设备栈上**可达**：
//        ALIGN 的对准命令越过转台行程即当拍 FAILED（2002），
//        见 MeasurementController::stepAlign 的 ④ 段。
//    规则②正是针对该边界：**转入终态当拍不补帧，下一拍恢复空闲预览**。
//
//  ⚠ 本类的 tick() 必须只在 GUI 线程调用（MainWindow 的定时器）。
//    SYS-08 §9 冻结"状态机运行于 Application 线程"，而本工程的
//    Application 线程 = GUI 线程（Qt 的事件循环），这是 009 的选择，
//    已记入 README §6。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "app/ApplicationContext.h"
#include "data/ErrorInfo.h"
#include "ui/MeasurementView.h"

namespace aircraft
{
namespace app
{

/// 装配并驱动 ApplicationContext。
///
/// 用法（见 main.cpp）：
///      ApplicationContext ctx;
///      SystemInitializer  init(ctx);
///      if (!init.initialize(configDir)) { /* 打印 errorText 并退出 */ }
///      window.setPreviewManager(ctx.preview.get());
///      QTimer → init.tick(monotonicNowNs()) → window.setMeasurementView(...)
class SystemInitializer
{
public:
    explicit SystemInitializer(ApplicationContext& ctx);
    ~SystemInitializer();

    SystemInitializer(const SystemInitializer&)            = delete;
    SystemInitializer& operator=(const SystemInitializer&) = delete;

    /// 按 ENG-02 §15 的顺序创建并初始化全部对象。
    ///
    /// @param configDir 7 个 yaml 所在目录。相对路径相对**进程当前目录**
    ///        （见 config/system.yaml 的书写约定）。
    ///
    /// @return false 表示启动失败，原因见 errorText()。失败时**不做部分启动**：
    ///         已经创建的对象保持原样但不会被 tick() 使用（见 .cpp 的
    ///         `ready_` 标志）。启动失败按 ENG-10 §5.3 打印到 stderr 与日志。
    ///
    /// 失败判据（ENG-10 §5.3 的"文件缺失 → 启动失败"一档）：
    ///   · 7 个 yaml 中有任何一个打不开或结构不合法；
    ///   · 任一字段取值越界；
    ///   · log_dir / output_dir / model_dir / calibration_dir 无法创建或不可写；
    ///   · 标定文件缺失（calibrationMode == "file" 时）；
    ///   · 相机与标定的图像尺寸不一致（见 .cpp 的说明，此条**只告警不失败**）。
    bool initialize(const std::string& configDir);

    /// 推进一拍。返回 false 表示本次无需刷新界面（无变化）。
    ///
    /// @param nowNs 当前时刻，ns，CLOCK_MONOTONIC（ENG-09 §2.5）。
    ///        由调用方注入而非本类自取，理由与 MeasurementController::tick
    ///        完全相同（可测性，见其头文件）。
    bool tick(uint64_t nowNs);

    /// 组装当前界面应当显示的内容（纯读，无副作用）。
    ui::MeasurementView view() const;

    /// 请求启动一次测量（转发给 controller）。已就绪时返回 false。
    bool startMeasurement();

    /// 停止当前测量（转发给 controller）。
    void stopMeasurement();

    // ---- 诊断 ----

    /// 启动失败原因；成功时为空。
    const std::string& errorText() const { return errorText_; }

    /// 启动失败原因的**机读**形式（裁决 C-006）。
    ///
    /// ⚠ 为什么 errorText() 不够：启动失败发生在状态机启动之前，不会产生
    /// result.json，于是它唯一的痕迹就是一段人读字符串 —— 自动化运维
    /// 无法按码筛选"这批启动失败里有多少是配置问题"。本字段补上这个码
    /// （当前唯一取值：9005 系统配置加载失败）。
    /// code == 0 表示启动未失败。
    const data::ErrorInfo& startupError() const { return startupError_; }

    /// 启动期的告警（非致命）。**必须**被 main.cpp 打印出来 ——
    /// 静默的告警等于没有告警。
    const std::vector<std::string>& warnings() const { return warnings_; }

    /// initialize() 是否已成功完成。
    bool ready() const { return ready_; }

    /// 统计信息（启动后自检用，也供 010 的 M1 检查单核对）。
    struct Summary
    {
        /// **实际装配出来的后端数**（011-A1 改：此前硬编码为 3）。
        /// ⚠ 与 `enabledChannels`（rig 里 enabled 的**通道**数）不是同一件事：
        /// 通道来自配置，后端来自装配。两者都由事实推导，不写常数 ——
        /// 硬编码的 3 会在"某一路装配失败后仍报 3"时静默说谎。
        int  cameraCount       = 0;

        /// 三路里**实际就绪**（initialize 成功且未被禁用）的数目。
        /// ⚠ 与 `cameraCount` 分开记：「三路都装配了」与「三路都能用」
        /// 必须分别可见（ENG-09 V2.4 §5.30 第 3 条的装配摘要要求）。
        int  cameraReadyCount  = 0;

        int  enabledChannels   = 0;    ///< rig 里 enabled 的通道数
        bool calibrationLoaded = false;
        bool modelsLoaded      = false;
        bool statsLoaded       = false;
        bool degradedAtStart   = false;
        uint64_t ticks         = 0;    ///< tick() 被调用的次数
        uint64_t idleFrames    = 0;    ///< 空闲态额外采集并提交预览的帧数
    };
    const Summary& summary() const { return summary_; }

private:
    // ---- 装配的各步（拆开是为了让失败点可定位）----

    /// 步骤 1~2：配置与日志（ENG-10 §5.3 的三档失败在这里落地）。
    bool loadConfigAndLogger();

    /// 步骤 3：标定 + 光机。calibrationMode 决定走文件还是合成默认矩阵。
    bool buildOptical();

    /// 步骤 4~5：三相机、触发、转台（ENG-08 §3 的显式装配点）。
    bool buildDevices();

    /// 按 `CameraConfig::backend` 造一路后端（011-A1）。
    /// 失败时返回 nullptr 并写入 `error`。
    /// ⚠ `backend` 取值非法、或为 `imv` 而本次构建**未接入 SDK**
    /// ⇒ 都是失败（**不得**静默换成虚拟后端）。
    std::shared_ptr<device::ICameraBackend> makeBackend(
        const data::CameraConfig& config, std::string& error);

    /// 装配一路后端：**先看测试注入槽** `ApplicationContext::backendFactory`，
    /// 未设置时退回内置 `makeBackend()`（011-A1 九项缺口 §1 的裁决）。
    ///
    /// ⚠ 注入槽一旦设置，其返回值（包括**空**）就是结论：
    ///   返回空即该路装配失败，**不回落内置工厂**。静默回落会让测试断言的
    ///   对象与实际运行的对象不是同一个 —— 那种"在另一条路径上通过"的
    ///   结果比直接失败危险得多（它会被当成证据）。
    std::shared_ptr<device::ICameraBackend> assembleBackend(
        const data::CameraConfig& config, std::string& error);

    /// 逐路核对"**配置为 `imv` 的通道**是否真的就绪"，未就绪则整次启动失败。
    ///
    /// ⚠ 两处调用（011-A1 九项缺口 §1）：`initializeAll()` 之后、`startAll()`
    ///    **之前**（初检），以及 `startAll()` **之后**（复检）。
    ///    复检不可省：`startAll()` 在 `start()` 失败时只把该路 `available`
    ///    置假，可用数 ≥2 即返回 true —— 于是"CAM25 真实相机**取流启动失败**"
    ///    会以整次启动成功收场，而操作者以为在跑真实采集。
    /// ⚠ 检查范围**严格限定**在显式 `imv` 通道：虚拟通道的 `start()` 失败
    ///    仍按**现行待裁决策略**处理（保留现行待裁决策略，见 Q-D1），
    ///    本批不触碰该策略、也不改判。
    ///
    /// @param stage 用于文本的阶段名（"就绪"／"取流启动"）。
    /// @return true = 全部就绪（或没有显式 imv 通道）；false 时已写好
    ///         `errorText_`，**调用方负责** `rollbackDevices()`。
    bool verifyExplicitImvChannels(const char* stage);

    /// 整次启动失败时**逐路回滚**已建立的设备资源（011-A1，§3.4）。
    ///
    /// 顺序＝建立顺序的**逆序**：先 stop()（停流）再 close()（关设备 +
    /// 销毁句柄）。**不依赖**"哪一路失败"来决定要不要回滚 —— 启动被判
    /// 失败时，所有已建立的资源都要归还。
    void rollbackDevices();

    /// 装配摘要（011-A1）：每路一行，由**实际装配对象与设备回报**生成。
    /// 启动即打印，是"装配成什么样"的唯一可信来源。
    void logAssemblySummary();

    /// 步骤 6：预览。
    bool buildPreview();

    /// 步骤 7~9：机型库、统计表、算法链（ENG-10 §5.2 的一次性注入）。
    bool buildAlgorithm();

    /// 步骤 10~12：落盘、适配器、测量总控。
    bool buildApplication();

    /// 启动后自检：目录可写、相机数、标定与分辨率是否自洽。
    void selfCheck();

    /// 空闲态补一帧给预览（见文件头说明）。返回是否真的提交了。
    bool pumpIdlePreview(uint64_t nowNs);

    /// 写一条 INFO 日志（logger 未就绪时退化为 stderr）。
    void log(const std::string& module, const std::string& message);
    void warn(const std::string& message);

    ApplicationContext& ctx_;

    bool                     ready_ = false;
    std::string              errorText_;
    data::ErrorInfo          startupError_;
    std::vector<std::string> warnings_;
    Summary                  summary_;

    /// 上一拍的状态，用于判断"状态是否刚变化"（变化才值得刷新界面）。
    int lastState_ = -1;

    /// 上一拍是否已把 recorder 的结果路径报给界面。
    bool lastSaveReported_ = false;
};

}  // namespace app
}  // namespace aircraft
