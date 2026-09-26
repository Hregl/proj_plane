// ============================================================================
//  src/device/camera/MultiCameraManager.cpp
//
//  依据：SYS-06 §5 / §8、SYS-08 §7.5（†见文件末的引用勘误）、
//        ENG-09 V2.4 §2.5 / §4.3 / §4.4 / §5.6、裁决 C-01 v1.7
//
//  ⚠ 本批（011-A1）的三项实质改动：
//   ① **按分类消费取帧失败**（R09 部分修复）：超时/无帧/未就绪/帧损坏
//      **不再禁用通道**，只有"断连"与"归类不出"才禁用。原实现的
//      "任一 grabOne 失败即 available = false" 是把一次超时升级成永久
//      缺相机的根源。
//   ② **每路结果与聚合**：`CaptureRound` 如实保留三路各自发生了什么，
//      使上层能按原因处置，而不必从"图是否为空"反推。
//   ③ **期限由形参传入**：`capture(frame, deadlineNs)`，预算在**实际等待处**
//      取最小（单次上限 / 组剩余 / 距期限的剩余），且发软件触发令**之前**
//      也要判 —— 预算不足时既不发令也不取帧。
// ============================================================================

#include "device/camera/MultiCameraManager.h"

#include <algorithm>

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// SYS-08 §7.5 的降级下限：可用相机数 <2 时无法继续测量。
/// 该数值来自冻结的降级表，不是可调参数，故为文件内常量
/// （具有业务含义的数值不应散落在比较表达式中）。
constexpr int kMinUsableCameras = 2;

/// SYS-08 §7.5 的降级上限：低于此数即为降级。
constexpr int kFullCameraCount = 3;

/// 1 ms 的纳秒表示。预算判定的最小粒度。
constexpr uint64_t kOneMillisecondNs = 1000000ULL;

/// 三值取最小。
uint64_t minOf3(uint64_t a, uint64_t b, uint64_t c)
{
    return std::min(a, std::min(b, c));
}

/// `from` 之后还剩多少（已过则为 0）。
uint64_t remainingNs(uint64_t deadlineOrBudget, uint64_t elapsedNs)
{
    return deadlineOrBudget > elapsedNs ? deadlineOrBudget - elapsedNs : 0;
}

/// 该分类是否应当**禁用**通道（R09 收窄后的规则）。
///
/// ⚠ 规则的分界是"**设备是否还在，以及我们是否知道原因**"：
///   · `Disconnected` —— SDK 错误码注释明确表示设备未连接（−118）。
///     禁用是现行策略（§7.5 的"禁用故障相机"），本批不改。
///   · `SdkError` —— 已调用 SDK 但**归类不出**。保留现行禁用，但禁用
///     原因必须写成"分类未明确"，**不得**按错误码号段推断成物理断连。
///   · 其余（`Timeout`/`NoFrame`/`NotStarted`/`CorruptFrame`/
///     `NotImplemented`/本地错误）**都不禁用**：
///     它们是瞬态或本地问题，禁用等于把一次可自愈的失败升级成永久缺相机。
///     `CorruptFrame` 同列的理由：帧坏了但设备还在 —— 一次布局异常
///     不该让这台相机永久退出。
///   · `Unset` 是缺陷指示（漏赋值），不禁用而由聚合以最高严重度暴露。
bool shouldDisableChannel(data::OpStatus status)
{
    return status == data::OpStatus::Disconnected ||
           status == data::OpStatus::SdkError;
}

/// 禁用原因的**如实措辞**（§3.2；两条措辞不得互换，也不得写成"已确认"）。
const char* disableReasonText(data::OpStatus status)
{
    if (status == data::OpStatus::Disconnected)
    {
        // ⚠ 本批**不注册** `IMV_SubscribeConnectArg`（§2.6 第 1 条：
        //    回调在 SDK 自己的线程上执行，接它要先建跨线程投递路径，
        //    那属于"完整恢复策略"）。故**没有任何事件确认发生过** ——
        //    这句话必须留在文本里，否则日志会把"按码推断"说成"已确认断连"。
        return "按 SDK 错误码判定为断连，未经连接事件确认";
    }
    return "SDK 错误导致禁用（分类未明确）";
}

/// 「**释放/清理未获确认**」的如实措辞（与 `disableReasonText` 并列，
/// 两者**不得互换**，也不得写成"已确认"）。
///
/// ⚠ 两个不同的故障，两句不同的话：
///   · `disableReasonText(SdkError)` = "**调用失败、归不了类**" ——
///     说的是"我们看不懂设备为什么这样"；
///   · 本函数 = "**缓冲可能没还回去**" —— 说的是"资源状态不明"，
///     而这次调用本身可能是成功的（返回码为 0）或被抛出打断的。
///   上一版把第二种情形并进了第一种（因为 `mergeCleanup` 在主操作成功时
///   把状态提升为 `SdkError`），于是现场看到的是"分类未明确"，
///   真正该查的事（缓冲归属）一个字都没提。
std::string cleanupUnconfirmedText(const data::SdkFailure& failure)
{
    std::string text = "该通道已停止后续采集：";
    text += "帧缓冲释放**未获确认**（";
    // ⚠ `kCallThrewCode` 不是 SDK 返回码（见 `data::kCallThrewCode`），
    //    这里必须翻译成话，**不得**把那个数字当原码念出来。
    //    翻译只有一处实现（`data::sdkFailureCodeText`），本文件不再自带分支
    //    —— 两份实现迟早会分叉，而分叉的那一刻就会有一处又开始念数字。
    text += data::sdkFailureText(failure);
    text += "）—— 未归还的缓冲会被 SDK 内部缓存复用，续采会污染后续帧；"
            "本路的主失败与清理诊断均已保留（首因未被覆盖），"
            "自动恢复须有资源恢复依据后再做";
    return text;
}

// ---------------------------------------------------------------------------
//  触发前置判定（011-A1 九项缺口 §7 的裁决）
// ---------------------------------------------------------------------------

/// 本路触发前置检查的结论。
enum class TriggerPreflight
{
    /// 读回完整**且**与请求一致 ⇒ 按**实际读回**的模式执行（仅 Software 发令）。
    ActOnReadback,

    /// 读回不完整（`reported` 未知）⇒ 本轮**无法确认触发前置条件**。
    Unconfirmed,

    /// 读回完整但与请求**不一致** ⇒ 契约违背。
    Mismatch
};

/// 由读回状态判定本路**能不能继续**，并在不能时给出归属与原因文本。
///
/// ⚠ 这里取代上一版的三元表达式：
///     `reported ? (*reported == Software) : (requested == Software)`
///   它把"**读回未知**"悄悄换成"按请求值办"，于是"设了却没生效"这件
///   最该被检出的事被抹平 —— 读不到和读到了错的东西，两种现场动作
///   完全不同（前者查链路/读回调用，后者查设备配置），却得到同一个
///   行为：照常发令、照常取帧。回退到请求值还额外制造了一个假事实：
///   "我们确认过触发前置条件"。
///
/// 三分支（按裁决，**逐字**落到措辞上）：
///   · 读回不完整 ⇒ `NotStarted`／1004，文本"本轮无法确认触发前置条件"；
///   · 读回完整但与请求不一致 ⇒ `ContractViolation`／1006，记录请求值与实际值；
///   · 完整且一致 ⇒ 按实际读回模式执行。
/// 前两种都**不发令、不取帧**，且**都不永久禁用通道** —— 读回失败往往是
/// 瞬态的，禁用等于把一次可自愈的读失败升级成永久缺相机（R09 的形态）。
///
/// ⚠ 判据取 `reported` 而不是拿 `requested` 顶上：一致时二者同值，写成
///   `reported` 只是让"取值来源"只有一个答案（命令的对象是**设备**，
///   不是我们的配置）。而 `requested` 是 `FreeRun` 时**不豁免** ——
///   "请求自由运行就不必确认读回"正是同一个静默退化：读回未知时我们
///   同样不知道设备在等什么。
///
/// @param tms    该路的读回状态（含逐项读回现场）
/// @param status 输出：不能继续时的归属（`ActOnReadback` 时不动）
/// @param reason 输出：不能继续时的原因文本（人读，含请求值与实际值）
/// @return 结论
TriggerPreflight triggerPreflightOf(const data::TriggerModeState& tms,
                                    data::OpStatus&               status,
                                    std::string&                  reason)
{
    if (!tms.reported.has_value())
    {
        status = data::OpStatus::NotStarted;   // 1004：前置条件未确认
        // ⚠ 措辞纪律：**不得**写成"设备已停止取流" —— 我们只是**没读到**
        //    触发配置，设备可能一切正常。把"读不到"说成"已停流"会让现场
        //    去查一个不存在的问题（同时也把责任从读回调用推到了设备上）。
        reason = "本轮无法确认触发前置条件：触发模式读回不完整"
                 "（TriggerSelector／TriggerMode／TriggerSource 三项未能合成"
                 "出可判别的模式），未发令、未取帧";
        return TriggerPreflight::Unconfirmed;
    }

    if (*tms.reported != tms.requested)
    {
        status = data::OpStatus::ContractViolation;   // 1006：本地契约违背
        // ⚠ 请求值与实际值**都**要留下：只说"不一致"的话，现场还得
        //    自己去猜是哪一头错了（配置写错 vs 设备没接受）。
        reason = std::string("触发模式读回与请求不一致：请求 ") +
                 data::triggerModeName(tms.requested) + "，实际读回 " +
                 data::triggerModeName(*tms.reported) +
                 "；未发令、未取帧（设置未生效，按未生效的模式取帧会得到"
                 "与预期不同的曝光时序）";
        return TriggerPreflight::Mismatch;
    }

    status = data::OpStatus::Ok;
    return TriggerPreflight::ActOnReadback;
}

//  通道诊断文本的格式器**不在本文件**：`data::channelGrabRecordText()`
//  （`data/CaptureRound.h`）是唯一一份 —— 同一句话还要出现在应用层的
//  失败说明里（`MeasurementController::updateDegradation` 与
//  `acquire()` 的失败分支），两份措辞只要差一个字，离线核对时就得先判断
//  "这两行说的是不是同一件事"。
}  // namespace

MultiCameraManager::MultiCameraManager(std::shared_ptr<ICameraBackend> cam25,
                                      std::shared_ptr<ICameraBackend> cam50,
                                      std::shared_ptr<ICameraBackend> cam100)
    : clock_([] { return data::monotonicNowNs(); })
{
    cam25_.backend  = std::move(cam25);
    cam25_.role     = data::CameraRole::CAM25;
    cam50_.backend  = std::move(cam50);
    cam50_.role     = data::CameraRole::CAM50;
    cam100_.backend = std::move(cam100);
    cam100_.role    = data::CameraRole::CAM100;

    // 可用性初值为 false：必须由 initializeAll() 真正初始化成功才置真。
    // 若此处按"backend 非空即为可用"初始化，则 availableCameraCount()
    // 会在未初始化的实例上返回 3，使降级判据失去意义。
}

void MultiCameraManager::setClock(std::function<uint64_t()> clock)
{
    // 传空函数对象会让 clock_() 抛 bad_function_call；此时退回默认钟，
    // 而不是留下一个"注入了一个坏东西"的悬空状态。
    clock_ = clock ? std::move(clock)
                   : std::function<uint64_t()>(
                         [] { return data::monotonicNowNs(); });
}

void MultiCameraManager::setGrabBudget(uint32_t perGrabTimeoutMs,
                                       uint64_t groupBudgetNs)
{
    perGrabTimeoutMs_  = perGrabTimeoutMs;
    grabGroupBudgetNs_ = groupBudgetNs;
}

void MultiCameraManager::setDiagnosticSink(
    std::function<void(const std::string&)> sink)
{
    diagnosticSink_ = std::move(sink);
}

void MultiCameraManager::emitDiagnostic(const std::string& text)
{
    if (diagnosticSink_)
    {
        diagnosticSink_(text);
    }
}

uint64_t MultiCameraManager::nowNs() const
{
    return clock_ ? clock_() : data::monotonicNowNs();
}

MultiCameraManager::Channel& MultiCameraManager::channelOf(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:
        return cam25_;
    case data::CameraRole::CAM50:
        return cam50_;
    case data::CameraRole::CAM100:
        return cam100_;
    }
    // CameraRole 只有三个取值（ENG-09 §4.1），无 UNKNOWN 可落入此处。
    // 保留返回以保证在 -Wswitch 关闭或枚举被误扩展时不致未定义行为；
    // 返回 cam25_ 会掩盖问题，但因该分支不可达，仅作兜底。
    return cam25_;
}

const MultiCameraManager::Channel& MultiCameraManager::channelOf(
    data::CameraRole role) const
{
    return const_cast<MultiCameraManager*>(this)->channelOf(role);
}

bool MultiCameraManager::initializeAll()
{
    lastError_ = data::ErrorInfo{};

    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        // 无 backend = 该通道没有相机（见头文件对 nullptr 的说明）。
        // 这不是失败，只是少一台，交由下方的可用数判据统一处理 ——
        // 若在此处直接返回 false，则"只装了两台相机"这一正常配置
        // 会被当成初始化失败，而 §7.5〔引用无效·依据待裁决·见 Q-D2〕 明确允许 2 台降级工作。
        if (!ch->backend)
        {
            ch->available = false;
            continue;
        }

        const data::OperationResult r = ch->backend->initialize();
        ch->available                 = r.ok();

        // 初始化失败**不在这里**逐路记录错误文本：可用数判据（下方）
        // 才是决定性的。逐路文本会把"一台初始化慢"与"一台真的坏了"
        // 混进同一处 lastError_，而后者应由可用数判据统一表达。
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        // SYS-08 §7.5：≤1 台直接 FAILED，code=1001。
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，无法继续测量";
        lastError_.timestampNs = nowNs();
        return false;
    }

    // 2 台可用时进入降级，但**仍返回 true**（§7.5："降级继续"）。
    // 降级事实由 degraded() 提供，上层负责写入 result.json 并在 UI 显示
    // （§7.5〔引用无效·依据待裁决·见 Q-D2〕："降级必须在 UI 上可见，不得静默降级"）。
    //
    // ⚠ 本批起**不再**用 `lastError_` 传递降级通告（C-01 v1.7）：
    //    `1002` 的语义是"相机断连、已降级"，把"少一台相机"这条**装配事实**
    //    写进错误载体，会让"真的断连过"与"本来就只装了两台"在上层
    //    无法区分 —— 而 `lastError()` 的读者正是靠它决定要不要记故障。
    //    降级事实由 `degraded()` / `availableCameraCount()` / 上层的
    //    `degradationNotice_` 承载，语义各自清楚。
    return true;
}

bool MultiCameraManager::startAll()
{
    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        if (!ch->available || !ch->backend)
        {
            continue;
        }

        if (!ch->backend->start().ok())
        {
            // 启动失败的通道按硬件故障处理：标记不可用，由下方可用数判据
            // 决定是降级继续还是失败。不在此处直接返回 false，理由同
            // initializeAll() —— 单台启动失败在 §7.5 下是降级场景。
            ch->available = false;
        }
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        started_               = false;
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，启动失败";
        lastError_.timestampNs = nowNs();
        return false;
    }

    started_ = true;
    return true;
}

void MultiCameraManager::stopAll()
{
    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        if (ch->backend)
        {
            // 无论 available 与否都调用 stop()：SDK 的 stop 必须是幂等的
            // （ICameraBackend 的契约："允许在未 start() 时调用，须无副作用"），
            // 而跳过调用会让"初始化成功但未 start"的 backend 残留在
            // 未知状态，下次 start 时行为不确定。
            (void)ch->backend->stop();
        }
    }

    started_ = false;
}

data::CaptureRound MultiCameraManager::lastCaptureRound() const
{
    return lastRound_;
}

data::GrabResult MultiCameraManager::grabOne(Channel&          ch,
                                            data::ImageFrame& out,
                                            uint64_t&         timestampOut,
                                            uint32_t          timeoutMs)
{
    data::ImageFrame tmp;
    const data::GrabResult r = ch.backend->grab(tmp, timeoutMs);
    if (!r.ok())
    {
        return r;
    }

    // 校正身份字段：backend 的实现可能不知道自己的角色
    // （ImvCameraBackend 由 SDK 句柄构造，未必携带 CameraRole）。
    // 在管理器这一层统一补齐，使上层永远可以信任
    // ImageFrame::role 与它所在的 frame 字段一致 ——
    // 否则会出现"frame.cam25 里装的其实是 CAM100 的图"这类
    // 无法从数据本身察觉的错位，而三相机评分的正确性完全依赖它。
    tmp.role = ch.role;

    timestampOut = tmp.timestampNs;
    out          = std::move(tmp);
    return r;
}

bool MultiCameraManager::capture(data::MultiCameraFrame& frame,
                                 uint64_t                deadlineNs)
{
    lastError_ = data::ErrorInfo{};

    const uint64_t startedNs = nowNs();

    // 每轮从零开始构造本轮记录：不重置会让上一轮的失败残留到本轮，
    // 而"本轮三路各自如何"是上层据以处置的唯一依据。
    lastRound_          = data::CaptureRound{};
    lastRound_.startedNs = startedNs;

    const data::CameraRole kRoles[3] = {data::CameraRole::CAM25,
                                        data::CameraRole::CAM50,
                                        data::CameraRole::CAM100};
    for (int i = 0; i < 3; ++i)
    {
        lastRound_.channels[static_cast<std::size_t>(i)].role = kRoles[i];
    }

    // ---- 入口就绪性检查：不满足时**每一路都要显式赋值** ----
    //
    // ⚠ 未尝试的通道**不得**留着默认构造的 `Unset`（§4.3）：
    //   `Unset` 的语义是"漏赋值"，是缺陷指示 —— 用它表示"没尝试"
    //   会让聚合把一次普通的"通道不可用"报成最高严重度的代码缺陷。
    //   故此处逐路写 `NotStarted` 并写明**为什么没尝试**。
    auto fillAllSkipped = [this](const char* reason) {
        for (auto& rec : lastRound_.channels)
        {
            rec.attempted     = false;
            rec.skippedReason = reason;
            rec.result        = data::GrabResult{
                {data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
        }
    };

    const auto finishRound = [this](int captured) {
        lastRound_.capturedCount = captured;
        lastRound_.aggregate     = data::OpStatus::Ok;
        int worst                = -1;
        for (const auto& rec : lastRound_.channels)
        {
            const int sev = data::aggregationSeverity(rec.result.status);
            if (sev > worst)
            {
                worst                 = sev;
                lastRound_.aggregate  = rec.result.status;
            }
        }
        lastRound_.succeeded  = captured >= kMinUsableCameras;
        // 收尾时**重新取一次时钟**：`finishedNs` 必须反映"真的什么时候结束"，
        // 而不是复用入口时刻。末次取帧之后越过期限同样要如实留下痕迹
        // （读法：`finishedNs > deadlineNs`），不能因为"已经在收尾"就放过。
        lastRound_.finishedNs = nowNs();

        // ---- 诊断出口（§9）：只输出**需要记录**的事件 ----
        //
        // 判据两条：该路 `status != Ok`，**或**本次诊断非空。
        // ⚠ **正常事实的存在本身不得触发输出**：一轮三路全好时这里
        //   一行都不写（"三路都采到、帧状态为 0、触发读回一致"是正常
        //   事实，不是异常）。逐路写 WARN 会让日志被正常帧淹没，
        //   而"异常才有行"正是它能被当成信号的前提。
        // ⚠ 这里**不读**后端的 `lastErrorText()`（那可能属于更早的调用），
        //   文本只由本轮记录拼出。
        for (const auto& rec : lastRound_.channels)
        {
            if (rec.result.status == data::OpStatus::Ok && rec.diagnosis.empty())
            {
                continue;
            }
            // 文本由 `data::channelGrabRecordText()` 统一给出（含
            // `skippedReason`），保证此处与应用层失败说明**逐字一致**。
            emitDiagnostic(data::channelGrabRecordText(rec));
        }
    };

    if (!started_)
    {
        fillAllSkipped("未成功 startAll()，本轮未尝试该通道");
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "采集前未成功 startAll()";
        lastError_.timestampNs = nowNs();
        finishRound(0);
        return false;
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        fillAllSkipped("可用相机数不足 2，本轮未尝试该通道");
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，无法采集";
        lastError_.timestampNs = nowNs();
        finishRound(0);
        return false;
    }

    // 每轮采集都从零开始构造 frame：不清零会让上一轮某路的数据残留，
    // 而本轮该路若采集失败（返回的 ImageFrame 内容未定义、实现可能
    // 只写了部分字段），残留的旧图会被当成新图使用 —— 且时间戳是旧的，
    // 恰好会被同步判据判为"不同步"，故障现象指向触发而根因在复用。
    frame = data::MultiCameraFrame{};

    uint64_t maxTs = 0;
    int      captured = 0;
    bool     present[3] = {false, false, false};

    Channel* channels[] = {&cam25_, &cam50_, &cam100_};
    data::ImageFrame* outputs[] = {&frame.cam25, &frame.cam50, &frame.cam100};

    for (std::size_t i = 0; i < 3; ++i)
    {
        Channel*              ch  = channels[i];
        data::ChannelGrabRecord& rec = lastRound_.channels[i];

        if (!ch->available || !ch->backend)
        {
            rec.attempted     = false;
            rec.skippedReason = "通道不可用（未初始化成功、已被禁用，或无 backend）";
            rec.result        = data::GrabResult{
                {data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
            continue;
        }

        // ---- 预算：三个量在**每次实际等待处**取最小（§2.2／§3）----
        //
        // ⚠ 期限**只从形参来**，本函数不缓存、成员里也没有期限字段：
        //   缓存会把"上一次的截止时刻"用到这一轮，而两轮之间的期限
        //   通常不同（本轮更晚），表现为"明明还有预算却整轮不取帧"。
        //
        // ⚠ 三处检查点**各自重读时钟**（§3 的裁决）：模式读回（真实后端是
        //   三次阻塞的 SDK 读）与软件触发令本身都要花时间，"入口算出来的
        //   剩余"到用的时候可能已经过期 —— 若三处复用同一个读数，就会
        //   拿着一个已经不成立的等待上限去调 SDK，而它恰恰是在**期限之后**
        //   才生效的。重读后重算，是本项修复的全部内容。
        uint64_t groupRemainingNs  = 0;
        uint64_t deadlineRemainNs  = 0;
        const auto nowNsForBudget  = [&](uint64_t atNow) -> uint64_t {
            const uint64_t sinceStart =
                atNow > startedNs ? atNow - startedNs : 0;
            groupRemainingNs  = remainingNs(grabGroupBudgetNs_, sinceStart);
            deadlineRemainNs  = remainingNs(deadlineNs, atNow);
            return minOf3(
                static_cast<uint64_t>(perGrabTimeoutMs_) * kOneMillisecondNs,
                groupRemainingNs, deadlineRemainNs);
        };

        // "无期限"是 `data::kNoDeadlineNs`（极大值，与 `RetryManager` 的
        // `kNoDeadline` 同值同义），`remainingNs()` 会原样返回它 ⇒
        // 它**不参与取小**（任何实际预算都小于它），故无需特判；
        // 而"**已到期**"（`deadline <= now`，含 0）在 `remainingNs()`
        // 里返回 0 ⇒ 落到下面的 `< 1 ms` 分支。两者由此分开。

        if (perGrabTimeoutMs_ == 0)
        {
            // 单次上限配成 0：接口**不接受 0**（§2.2；SDK 对 timeoutMS = 0
            // 的语义未文档化，项目也不定义它），且**不取整成 0** 去调它。
            // 这是本地参数错误 —— 未调用 SDK，故 `sdkError` 为空。
            // ⚠ 它排在预算判定**之前**：这是配置错，不是"这一轮没预算"，
            //   报成超时会把现场引到设备侧。
            rec.attempted     = false;
            rec.skippedReason = "grab_timeout_ms 配置为 0（接口不接受 0，未调用 SDK）";
            rec.result        = data::GrabResult{
                {data::OpStatus::InvalidArgument, std::nullopt, std::nullopt}};
            continue;
        }

        // ---- 检查点 1：入口（**连模式读回都不执行**）----
        const uint64_t entryBudgetNs = nowNsForBudget(nowNs());
        if (entryBudgetNs < kOneMillisecondNs)
        {
            // ⚠ **剩余不足 1 ms ⇒ 既不发令也不取帧**，且**不去取整成 0**
            //   再调一个已禁止 0 的接口。
            // ⚠ 这一条在软件触发**之前**判：否则会"发出了触发脉冲却收不回帧"，
            //   把一次预算不足变成设备侧的悬空触发（下一拍的触发时序随之错乱）。
            rec.attempted = false;
            rec.result    = data::GrabResult{
                {data::OpStatus::Timeout, std::nullopt, std::nullopt}};
            // ⚠ 三个量**逐个列出来**，并写明"取小者"是谁 —— 只说
            //   "预算耗尽"会让读日志的人分不清是组预算用完了、期限到了、
            //   还是单次上限配得太小；三者的处置完全不同（等下一轮 /
            //   状态该收尾了 / 改配置）。
            rec.skippedReason =
                std::string("预算耗尽（入口：组剩余 ") +
                std::to_string(groupRemainingNs / kOneMillisecondNs) +
                " ms，距期限 " +
                std::to_string(deadlineRemainNs / kOneMillisecondNs) +
                " ms，单次上限 " + std::to_string(perGrabTimeoutMs_) +
                " ms；三者取小 = " +
                std::to_string(entryBudgetNs / kOneMillisecondNs) +
                " ms < 1 ms），未发令、未取帧";
            continue;
        }

        // ---- 触发前置判定（§7；**取代**上一版的三元回退）----
        //
        // ⚠ 读回**逐项**带回来（特性名／是否调用过／失败原码／是否返回空串），
        //   原样进本轮记录：三项走的是同一个 SDK 调用，只有
        //   `{call, code}` 时说不出失败的是哪一项。
        const data::TriggerModeState tms = ch->backend->triggerModeState();
        rec.readbacks                    = tms.readbacks;

        data::OpStatus preflightStatus = data::OpStatus::Ok;
        std::string    preflightReason;
        const TriggerPreflight pre =
            triggerPreflightOf(tms, preflightStatus, preflightReason);

        if (pre != TriggerPreflight::ActOnReadback)
        {
            // 读回不完整／不一致：**不发令、不取帧**，且**不永久禁用通道**
            // （下一次采集允许重新读回 —— 读回失败往往是瞬态的）。
            rec.attempted     = false;
            rec.skippedReason = preflightReason;
            rec.result        = data::GrabResult{preflightStatus, std::nullopt,
                                                 std::nullopt};
            continue;
        }

        // ---- 检查点 2：读回之后、**发令之前**（重读时钟）----
        const uint64_t preTriggerBudgetNs = nowNsForBudget(nowNs());
        if (preTriggerBudgetNs < kOneMillisecondNs)
        {
            rec.attempted = false;
            rec.result    = data::GrabResult{
                {data::OpStatus::Timeout, std::nullopt, std::nullopt}};
            // ⚠ 与检查点 1 的措辞**必须分开**：这里已经花掉了读回的时间，
            //   "入口就没预算"与"读回把预算吃掉了"的处置不同（后者说明
            //   读回本身太慢，是要查的东西）。
            rec.skippedReason =
                std::string("发令前预算耗尽（读回耗时计入后：组剩余 ") +
                std::to_string(groupRemainingNs / kOneMillisecondNs) +
                " ms，距期限 " +
                std::to_string(deadlineRemainNs / kOneMillisecondNs) +
                " ms，单次上限 " + std::to_string(perGrabTimeoutMs_) +
                " ms；三者取小 = " +
                std::to_string(preTriggerBudgetNs / kOneMillisecondNs) +
                " ms < 1 ms），未发令、未取帧";
            continue;
        }

        uint32_t timeoutMs = static_cast<uint32_t>(preTriggerBudgetNs /
                                                  kOneMillisecondNs);

        // ---- 软件触发（**唯一执行者**，§2.4）----
        //
        // 判据取该路**读回**的模式（上面的前置判定已保证它与请求一致；
        // 一致时二者同值，取 `reported` 只是让取值来源只有一个答案）。
        const bool isSoftware = (*tms.reported == data::CameraTriggerMode::Software);

        if (isSoftware)
        {
            rec.attempted = true;
            rec.trigger   = ch->backend->triggerSoftware();

            if (!rec.trigger.ok())
            {
                // ⚠ 发令失败 ⇒ **不取帧**（该路 `IMV_GetFrame` 次数为 0）。
                //    "等待一帧"不等于"发出了一次软件触发"：既然令没发出去，
                //    就不该去等一个不会到来的帧 —— 否则会把发令失败
                //    伪装成一次超时，而超时是可以重采的、发令失败不是。
                rec.result = data::GrabResult{rec.trigger.status,
                                              rec.trigger.sdkError,
                                              std::nullopt};
                rec.skippedReason = "软件触发令失败，未取帧";
                if (shouldDisableChannel(rec.result.status))
                {
                    ch->available = false;
                }
                continue;
            }

            // ---- 检查点 3：**发令返回之后**、取帧之前（重读时钟）----
            //
            // ⚠ 这一种**不得**写成"既未发令也未取帧"：令**确实发出去了**，
            //   这是本轮必须留下的事实（`attempted = true`、
            //   `rec.trigger` 保留）—— 否则现场会以为设备这一拍没被触发过，
            //   而它其实已经收到脉冲、只是我们没等帧。
            const uint64_t preGrabBudgetNs = nowNsForBudget(nowNs());
            if (preGrabBudgetNs < kOneMillisecondNs)
            {
                rec.result = data::GrabResult{
                    {data::OpStatus::Timeout, std::nullopt, std::nullopt}};
                rec.skippedReason =
                    std::string("已发令、未取帧（取帧前预算耗尽：组剩余 ") +
                    std::to_string(groupRemainingNs / kOneMillisecondNs) +
                    " ms，距期限 " +
                    std::to_string(deadlineRemainNs / kOneMillisecondNs) +
                    " ms，单次上限 " + std::to_string(perGrabTimeoutMs_) +
                    " ms；三者取小 = " +
                    std::to_string(preGrabBudgetNs / kOneMillisecondNs) +
                    " ms < 1 ms）";
                continue;
            }

            timeoutMs = static_cast<uint32_t>(preGrabBudgetNs /
                                              kOneMillisecondNs);
        }

        // ---- 取帧（本函数只取帧，绝不发令）----
        rec.attempted = true;

        uint64_t ts = 0;
        rec.result  = grabOne(*ch, *outputs[i], ts, timeoutMs);
        // ⚠ 诊断与帧状态原值**原样转发**（§9 的传递路径：
        //   `SDK 帧视图 → GrabResult（本次诊断） → ChannelGrabRecord → 日志`）。
        //   管理器**不**读后端的 `lastErrorText()`，也**不**补造
        //   `{ImvGetFrame, 0}` —— 它调的是 `ICameraBackend::grab()`，
        //   虚拟/脚本后端可能根本没调用过 SDK，补造会把"未调用"记成"调用过"。
        rec.diagnosis      = rec.result.diagnosis;
        rec.frameStatusRaw = rec.result.frameStatusRaw;

        if (rec.result.ok())
        {
            // 空图 ⇒ **契约违背**（§8 的裁决），不是"一次成功的取帧"。
            //
            // ⚠ 上一版在这里只 `continue`，`rec.result` 保持 `Ok`
            //   （`{{Ok, …}}`），于是：
            //     ① 聚合取严重度最大者 ⇒ 整轮 `aggregate == Ok`，
            //        "三路全正常"与"三路全给空图"在上层**无法区分**；
            //     ② 控制器只对 `status != Ok` 的通道拼 `failedDetail` ⇒
            //        空图的原因**进不了降级说明**，现场只看到"少了两路"。
            //   接口契约是"`status == Ok` ⇒ 该帧满足一切后置条件"，
            //   而空图不满足 ⇒ 交付这样一个 `Ok` 本身就是契约违背，
            //   与设备好坏无关（设备可能一直在正常工作，是我们的
            //   判据与实现不一致）。
            //
            // ⚠ 归属是 `ContractViolation`（本地契约，1006），**不**改成
            //   `CorruptFrame`／`Timeout`：它不是设备报的故障，也不是超时。
            // ⚠ 后端的**调用诊断原样保留**（`rec.diagnosis`／
            //   `frameStatusRaw`／`sdkError`／`cleanupError` 已在上面转发）：
            //   管理器**不补造** `{ImvGetFrame, 0}` —— 虚拟/脚本后端可能
            //   根本没调用过 SDK，补造会把"未调用"记成"调用过"。
            if (outputs[i]->image.empty())
            {
                rec.result.status = data::OpStatus::ContractViolation;
                rec.timestampNs   = 0;   // 没有画面，就没有画面时刻
                rec.skippedReason =
                    "取帧返回 Ok 但交付的图为空：与接口契约（Ok ⇒ 帧满足"
                    "全部后置条件）矛盾，记为契约违背（不计入本轮有效帧、"
                    "不作为曝光序号的参考路；通道仍可用）";
                continue;
            }

            ++captured;
            present[i]        = true;
            rec.timestampNs   = ts;
            if (ts > maxTs)
            {
                maxTs = ts;
            }
            continue;
        }

        // ---- 清理未获确认 ⇒ 停止该路后续采集（V2.5 的统一规则）----
        //
        // ⚠ 为什么这一条**必须排在状态分类之前**、且**只看 `cleanupError`**：
        //   在上一版，"释放未获确认"这件事落在哪个字段**取决于主操作的结果**：
        //     · 帧检查通过而释放失败 ⇒ `mergeCleanup` 把状态提升成 `SdkError`
        //       ⇒ 走下面的 `shouldDisableChannel` ⇒ **禁用**；
        //     · 帧已损坏（`CorruptFrame`）而释放失败 ⇒ 保留首因、失败只进
        //       `cleanupError` ⇒ 状态不在禁用集合里 ⇒ **继续可用**。
        //   同一件事、两种处置，区别只在于"帧本身好不好" —— 而帧好不好
        //   与"缓冲还回去了没有"是**两个不相干的问题**。
        // ∴ 判据改成直接看"清理是否有未获确认的失败"（V2.5 起，
        //   主操作成功时的清理失败也进 `cleanupError`，两个入口统一）。
        //
        // ⚠ 处置是"**停止该路后续采集**"而不是"判定这台相机坏了"：
        //   设备本身可能完全正常（甚至那次释放其实成功了，只是返回码
        //   没拿到），未知的是**资源状态**。故文本只谈资源，
        //   且**不覆盖首因** —— `status` 与 `sdkError` 一字不动，
        //   现场同时看到"本来因为什么失败"与"资源没还回去"。
        //   自动恢复（重新启用）须先有资源恢复的依据，本批不做。
        if (rec.result.cleanupError.has_value())
        {
            ch->available = false;
            rec.skippedReason = cleanupUnconfirmedText(*rec.result.cleanupError);
        }
        // ---- 分类消费（R09 部分修复）----
        else if (shouldDisableChannel(rec.result.status))
        {
            ch->available = false;
            rec.skippedReason =
                std::string("该通道已禁用：") +
                disableReasonText(rec.result.status);
        }
        else if (rec.result.status == data::OpStatus::Unset)
        {
            // ⚠ `Unset` **不是**一种失败，是一次漏赋值：后端的
            //   `GrabResult` 被默认构造后直接返回（忘了填 `status`）。
            //   故它既不能与"瞬态失败"共用一句话（那会把代码缺陷说成
            //   设备抖动，现场动作完全不同），也不能被当成"未尝试"
            //   （`Unset` 只表示 bug，§2.2 状态全表）。
            //
            //   处置：**不禁用**通道（设备本身没问题），但如实点名是哪一路，
            //   并让聚合以最高严重度（100）把它顶出来 ⇒ 应用码 9004。
            //   "哪一路"由**记录自身**回答（`ChannelGrabRecord::role`），
            //   与本节其余措辞同一写法：管理器只说"是什么"，上层在拼接
            //   用户可见文本时点名通道（见 `updateDegradation()`）。
            rec.skippedReason =
                "本路返回未赋值的结果（Unset）：后端在 grab() 的返回路径上"
                "漏填了 status —— 这是代码缺陷，不是设备故障；"
                "通道未被禁用";
        }
        else
        {
            // 瞬态/本地失败：**不禁用**，只如实记录。
            // 上一版在此处一律 `available = false` —— 一次超时就把通道
            // 永久判死，与真断连得到完全相同的处置（R09 的失效形态）。
            rec.skippedReason = "本轮取帧失败（瞬态/本地，通道仍可用）";
        }
    }

    finishRound(captured);

    if (captured < kMinUsableCameras)
    {
        // ⚠ 1001 在此处的含义是**就绪性**而非本轮某一类的失败：
        //   走到这里说明可用相机数已跌破 §7.5 的下限，"还能不能测"这个
        //   问题的答案与"这一轮为什么没取到"是两个不同层面的问题。
        //   本轮各路的**原因**在本轮记录里（`lastCaptureRound()`），
        //   上层按 `appErrorCodeOf(aggregate)` 取原因码。
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = std::string("本轮采集可用相机数不足 2（聚合状态 ") +
                                 data::opStatusName(lastRound_.aggregate) +
                                 "，原因码 " +
                                 std::to_string(
                                     data::appErrorCodeOf(lastRound_.aggregate)) +
                                 "）";
        lastError_.timestampNs = nowNs();
        return false;
    }

    // ENG-09 §2.5：triggerTimestamp = 各相机 timestampNs 的最大值。
    // 只统计本轮真正采到的通道 —— 被跳过的通道其 timestampNs 为 0，
    // 若不排除，在只有 2 台可用时 max 仍正确（0 不是最大值），
    // 但逻辑上把"未采集"混入"已采集"的统计范围本身就是错的，
    // 一旦未来某处的默认值不再是 0 就会静默出错。
    frame.triggerTimestamp = maxTs;

    // 裁决 D-C02-6：本组三帧的"同一次曝光序号"。放在 triggerTimestamp
    // 之后、成功返回之前 —— 走到这里意味着本轮确实产出了一组可用帧，
    // 失败路径上的 frame 内容未定义，不应产出序号。
    frame.exposureIndex = updateExposureIndex(frame, present);

    return true;
}

uint64_t MultiCameraManager::updateExposureIndex(
    const data::MultiCameraFrame& frame,
    const bool present[3])
{
    const data::ImageFrame* channels[] = {&frame.cam25, &frame.cam50, &frame.cam100};

    // 参考路：按 CAM25 → CAM50 → CAM100 顺序的第一路可用者。
    // 为什么是固定顺序而非"随便哪一路"：序号必须在整轮任务中来自**同一条**
    // 通道，否则某轮 CAM25 掉线会让序号来源悄悄换人，而两条链的偏移量
    // 在同一次采集里未必相等 —— 序号就会出现无法解释的跳变。
    int ref = -1;
    for (int i = 0; i < 3; ++i)
    {
        if (present[i])
        {
            ref = i;
            break;
        }
    }
    if (ref < 0)
    {
        return 0;   // 无任何可用路：未定
    }

    const uint64_t frameId = channels[ref]->frameId;

    if (!frameIdBaseLocked_[ref])
    {
        frameIdBase_[ref]       = frameId;
        frameIdBaseLocked_[ref] = true;
    }
    else if (frameId < frameIdBase_[ref])
    {
        // 设备计数器回退（相机重连 / 重启 / 回绕）：基准失效，就地重锁。
        // 不重锁会让减法下溢成一个巨大的序号，而那个值看起来仍然"合法"。
        frameIdBase_[ref]       = frameId;
        exposureIndexDegraded_  = true;
    }

    // 1 起计数：0 保留给"未定"（MultiCameraFrame 的字段说明）。
    uint64_t candidate = frameId - frameIdBase_[ref] + 1;

    if (candidate <= exposureIndex_)
    {
        // 设备帧号没有前进（例如某后端的 grab() 尚未填 frameId，
        // 使三路都恒为基值）。此时若照用，序号会恒等于 1 ——
        // 而它承载的正是"这几张 raw 是不是同一次曝光"这个判断，
        // 恒值会让该判断永远为"是"。
        // 处置：保住单调性（调用方依赖这条不变量），并把
        // "序号已不来自设备帧号"这件事**记录成可见事实**。
        exposureIndexDegraded_ = true;
        candidate              = exposureIndex_ + 1;
    }

    exposureIndex_ = candidate;
    return exposureIndex_;
}

bool MultiCameraManager::exposureIndexDegraded() const
{
    return exposureIndexDegraded_;
}

int MultiCameraManager::availableCameraCount() const
{
    int n = 0;
    if (cam25_.available && cam25_.backend)
    {
        ++n;
    }
    if (cam50_.available && cam50_.backend)
    {
        ++n;
    }
    if (cam100_.available && cam100_.backend)
    {
        ++n;
    }
    return n;
}

bool MultiCameraManager::degraded() const
{
    const int n = availableCameraCount();
    return n >= kMinUsableCameras && n < kFullCameraCount;
}

data::DeviceState MultiCameraManager::state() const
{
    if (availableCameraCount() < kMinUsableCameras)
    {
        return data::DeviceState::ERROR;
    }

    if (!started_)
    {
        return data::DeviceState::READY;
    }

    return data::DeviceState::RUNNING;
}

data::ErrorInfo MultiCameraManager::lastError() const
{
    return lastError_;
}

bool MultiCameraManager::channelAvailable(data::CameraRole role) const
{
    // ⚠ 无 backend 与"有 backend 但没就绪"都返回 false，但**两者不同**：
    // 前者是"这一路根本没装配"，后者是"装配了但初始化失败或被禁用"。
    // 装配摘要把它们分别写出（用"是否有 backend 指针"区分，那由装配层
    // 自己持有，不必问本类），不把两种情形合并成一个 false ——
    // 否则摘要会把"没装"读成"装了但坏了"。
    const Channel& ch = channelOf(role);
    return ch.backend != nullptr && ch.available;
}

bool MultiCameraManager::disableChannel(data::CameraRole role)
{
    Channel& ch = channelOf(role);
    if (!ch.backend)
    {
        return false;
    }

    ch.available = false;

    // 与"采集途中掉线"一致地记录：这是 §7.5 的硬件故障降级路径，
    // 置 code=1002（已降级）而不是错误 —— 上层据 availableCameraCount()
    // 判断是否已跌破下限。
    if (availableCameraCount() < kMinUsableCameras)
    {
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "禁用该通道后可用相机数不足 2";
        lastError_.timestampNs = nowNs();
    }
    else
    {
        lastError_.code        = data::kErrCameraDegraded;
        lastError_.message     = "通道已禁用，降级继续";
        lastError_.timestampNs = nowNs();
    }

    return true;
}

}  // namespace device
}  // namespace aircraft

// ============================================================================
//  引用勘误（2026-09-26）
//
//  ⚠ 本文件（及本目录多份文件）引用的 **SYS-08 §7.x 经核实为悬空／撞号引用**：
//  SYS-08 V2.1 的 `# 7` 是 TARGET_FOUND状态（§7.1 状态说明 / §7.2 执行动作 /
//  §7.3 输出 / §7.4 失败处理），与本工程长期引用的"§7.1 三级超时 /
//  §7.2 失败三分类 / §7.3 次数表 / §7.4 回退预算"**内容完全不同**（节号碰撞）；
//  §7.5 硬件降级、§7.6 RetryManager 接口、§7.7 则**不存在**。
//  另：`# 17 异常处理设计`（§17.1 相机异常）的正文是"断连；/ 无帧；/ 超时。
//  → 处理：进入：FAILED。"—— 三类**一律 FAILED**，无重试/重连/掉线-vs-故障
//  的区分，与本文件现行的"两路可用降级继续"**冲突**。
//  处置：依据待裁决，见《待裁决问题汇总》Q-D1 / Q-D2 与《SYS-08-§7引用勘误.md》；
//  正文引用**仅描述现行行为**，不作为冻结依据。
// ============================================================================
