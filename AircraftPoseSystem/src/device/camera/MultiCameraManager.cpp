// ============================================================================
//  src/device/camera/MultiCameraManager.cpp
//
//  依据：SYS-06 §5 / §8、SYS-08 §7.5（†见文件末的引用勘误）、
//        ENG-09 V2.3 §2.5 / §4.3 / §4.4 / §5.6、裁决 C-01 v1.7
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

        // ---- 预算：三个量在实际等待处取最小（§2.2） ----
        //
        // ⚠ 期限**只从形参来**，本函数不缓存、成员里也没有期限字段：
        //   缓存会把"上一次的截止时刻"用到这一轮，而两轮之间的期限
        //   通常不同（本轮更晚），表现为"明明还有预算却整轮不取帧"。
        const uint64_t now              = nowNs();
        const uint64_t sinceStartNs     = now > startedNs ? now - startedNs : 0;
        const uint64_t groupRemainingNs = remainingNs(grabGroupBudgetNs_,
                                                      sinceStartNs);
        const uint64_t deadlineRemainNs = remainingNs(deadlineNs, now);

        if (perGrabTimeoutMs_ == 0)
        {
            // 单次上限配成 0：接口**不接受 0**（§2.2；SDK 对 timeoutMS = 0
            // 的语义未文档化，项目也不定义它），且**不取整成 0** 去调它。
            // 这是本地参数错误 —— 未调用 SDK，故 `sdkError` 为空。
            rec.attempted     = false;
            rec.skippedReason = "grab_timeout_ms 配置为 0（接口不接受 0，未调用 SDK）";
            rec.result        = data::GrabResult{
                {data::OpStatus::InvalidArgument, std::nullopt, std::nullopt}};
            continue;
        }

        const uint64_t perGrabCapNs =
            static_cast<uint64_t>(perGrabTimeoutMs_) * kOneMillisecondNs;
        const uint64_t budgetNs =
            minOf3(perGrabCapNs, groupRemainingNs, deadlineRemainNs);

        if (budgetNs < kOneMillisecondNs)
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
                std::string("预算耗尽（组剩余 ") +
                std::to_string(groupRemainingNs / kOneMillisecondNs) +
                " ms，距期限 " +
                std::to_string(deadlineRemainNs / kOneMillisecondNs) +
                " ms，单次上限 " + std::to_string(perGrabTimeoutMs_) +
                " ms；三者取小 = " +
                std::to_string(budgetNs / kOneMillisecondNs) +
                " ms < 1 ms），未发令、未取帧";
            continue;
        }

        const uint32_t timeoutMs = static_cast<uint32_t>(budgetNs / kOneMillisecondNs);

        // ---- 软件触发（**唯一执行者**，§2.4）----
        //
        // 判据取该路**读回**的模式；读回不可用时退回请求值 ——
        // 理由：读不到时我们手里没有比请求更好的信息，而**不发令**会让
        // 一台真正处于软件触发的相机永远等不到触发（比"多发一次令"更坏：
        // 前者是整轮无图，后者最多引起一次无效命令）。
        const data::TriggerModeState tms = ch->backend->triggerModeState();
        const bool isSoftware =
            tms.reported.has_value()
                ? (*tms.reported == data::CameraTriggerMode::Software)
                : (tms.requested == data::CameraTriggerMode::Software);

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
        }

        // ---- 取帧（本函数只取帧，绝不发令）----
        rec.attempted = true;

        uint64_t ts = 0;
        rec.result  = grabOne(*ch, *outputs[i], ts, timeoutMs);

        if (rec.result.ok())
        {
            // 修正（011-A1）：`ok()` 是**必要而不足**的条件。
            //
            // `capturedCount` 的冻结判据是"该路 `status == Ok` **且**交付的
            // 图合法（非空）"（CaptureRound.h、§3.2），而此处原来的实现只数
            // `ok()`，**从不看图**：于是"每次都取帧成功、但每次都返回空图"的
            // 后端会被记成"三路全部采到"，`captured >= 2` 也照样成立 ——
            // 上层据此认为链路正常，而实际上一帧可用数据都没有。
            // 这正是一个"计数口径与判据不一致"的缺陷：判据写在文档与
            // 上层（`!image.empty()`），计数写在管理器，两处各说各话。
            //
            // 处置：空图**不计入**有效帧、**不作为**曝光序号的参考路
            // （`present[]` 的语义是"本轮真的采到了"，空图没采到），
            // 但 `rec.result` **保持 `Ok`** —— 调用确实成功了，这是事实，
            // 不得为了凑计数把它改写成失败。原因写在 `skippedReason`
            // 里供人排查。
            if (outputs[i]->image.empty())
            {
                rec.timestampNs   = 0;   // 没有画面，就没有画面时刻
                rec.skippedReason =
                    "取帧成功但交付的图为空（不计入本轮有效帧、"
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

        // ---- 分类消费（R09 部分修复）----
        if (shouldDisableChannel(rec.result.status))
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
