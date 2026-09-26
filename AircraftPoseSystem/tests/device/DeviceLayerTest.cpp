// ============================================================================
//  tests/device/DeviceLayerTest.cpp
//
//  依据：ENG-06（测试工程设计）、ENG-08 §13
//        SYS-06（设备抽象层）、SYS-08 §7.5（降级表）、§10（收敛性测试）
//
//  覆盖范围（本文件聚焦**设备层自身的契约**）：
//    · MultiCameraManager 的三种相机数量情形（3 / 2 / ≤1），
//      对应 SYS-08 §7.5 降级表的三行；
//    · triggerTimestamp 的最大值规则（ENG-09 §2.5）；
//    · VirtualTurntable 的行程限位（SYS-08 §7.7 能力边界，不重试）
//      与运动时序（MOVING → STABLE）；
//    · 触发失效的错误码（SYS-08 §7.5 的 3001）；
//    · 真实设备桩必须如实报告不可用。
//
//  ⚠ 不在本文件覆盖的内容：
//    · SYS-08 §10 的 7 个**状态机收敛性**用例 —— 它们需要
//      application 层的 StateMachine 与 RetryManager（010 阶段），
//      属于 tests/integration/ 的范围；
//    · 三相机同步判据（syncToleranceNs）—— 归 optical 层的
//      CameraSynchronizer，属于 tests/optical/ 的范围。
//    在此重复实现会让同一判据有两份实现，而两份实现迟早会不一致。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <gtest/gtest.h>

#include <sys/mman.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "data/CameraConfig.h"
#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"
#include "data/TriggerConfig.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "device/camera/ImvCameraBackend.h"
#include "device/camera/MultiCameraManager.h"
#include "device/camera/VirtualCameraBackend.h"
#include "device/turntable/PekoTurntableController.h"
#include "device/turntable/VirtualTurntable.h"
#include "device/trigger/HardwareTriggerController.h"
#include "device/trigger/VirtualTriggerController.h"

// ===========================================================================
//  分配失败注入器（011-A1 §5 的异常路径证据）
// ===========================================================================
//
// 为什么需要它：取帧的异常路径（复制载荷、OpenCV 建图）**只有在真的抛出
// 时**才会走到，而"抛异常"在正常情况下不可复现。§5 要求证明"异常时帧仍被
// 释放、且异常与清理结果能被读到" ⇒ 必须有一个**受控、定点、可立刻复位**
// 的失败注入点。
//
// ⚠ 不靠"伪造超长缓冲区"、更不靠"真的把内存耗尽"：前者要靠长度校验的
//   未定义行为（而且会被长度检查先拦住，根本走不到分配），后者会波及
//   整台机器的其它进程，得到的证据还不可复现。
//
// ⚠ 实现方式是替换全局 `operator new` —— 这会作用于**本测试可执行文件
//    （含它链接的静态库）**。故注入器默认**关闭**（倒计时 0），
//    只在单个用例里由替身的开关打开（`failNextAllocationAfterFrame` ＝
//    紧随其后的一次；`secondaryFailureCount` ＝ 紧随其后的那几次改用
//    另一种异常种类），用完即归零；用例捕获异常后的第一条语句再显式
//    撤防一次，确保倒计时不会漏进测试框架自身的分配。
namespace
{
/// 还要让接下来多少次分配失败；**0 表示关闭**。
int g_allocFailRemaining = 0;

/// 注入失败时抛出的异常种类：`0` = `std::bad_alloc`，`1` = `std::length_error`。
///
/// ⚠ 为什么要**两种**种类（V2.5 新增）：只注入一种异常时，"原异常被二次
///   异常顶掉"与"原异常正常上抛"是**同一个类型**，用例根本分辨不出自己
///   看到的是哪一个 —— 那样的用例即使全绿也证明不了"保住了原异常"。
///   两种种类让这两件事可区分：原异常是 `bad_alloc`，而"描述异常／写诊断
///   这两步自己失败"造成的是 `length_error`。
int g_allocThrowKind = 0;

/// 第一段用尽后自动装载的第二段（次数、种类）。
int g_allocFailPhase2Remaining = 0;
int g_allocThrowKindPhase2     = 1;

/// 两段式布防：接下来 `firstCount` 次分配抛 `firstKind`，
/// **紧接着的** `restCount` 次抛 `restKind`。
///
/// ⚠ 为什么必须"两段"：异常出口的三步里，第 ① 步（描述异常）紧跟在
///   抛出之后、**早于**任何替身回调 —— 若只布一种种类，"
///   原异常被二次异常顶掉"与"原异常正常上抛"就是同一个类型，用例
///   分辨不出自己收到的是哪一个。两段式让**原异常**（第一段，
///   `bad_alloc`）与**二次失败**（第二段，`length_error`）可区分，
///   且各自的落点可控（复制载荷 / 描述 / 写诊断）。
/// ⚠ 消息用短字面量（`"SECONDARY"`，9 字符 < SSO 上限）：**抛异常本身
///   也可能分配**，若它再撞上还没用完的倒计时就会自我递归。
void armAllocFailurePhased(int firstCount, int firstKind, int restCount,
                           int restKind)
{
    g_allocFailRemaining       = firstCount;
    g_allocThrowKind           = firstKind;
    g_allocFailPhase2Remaining = restCount;
    g_allocThrowKindPhase2     = restKind;
}

/// 撤防（用例在捕获到异常后的**第一条**语句就调用它）。
void disarmAllocFailure()
{
    g_allocFailRemaining       = 0;
    g_allocThrowKind           = 0;
    g_allocFailPhase2Remaining = 0;
}
}  // namespace

void* operator new(std::size_t size)
{
    if (g_allocFailRemaining > 0)
    {
        const int kind = g_allocThrowKind;
        if (--g_allocFailRemaining == 0)
        {
            // 第一段用尽 ⇒ 装载第二段（可能有，也可能没有）。
            g_allocFailRemaining       = g_allocFailPhase2Remaining;
            g_allocThrowKind           = g_allocThrowKindPhase2;
            g_allocFailPhase2Remaining = 0;
        }
        if (kind == 1)
        {
            throw std::length_error("SECONDARY");
        }
        throw std::bad_alloc();
    }
    if (void* p = std::malloc(size == 0 ? 1 : size))
    {
        return p;
    }
    throw std::bad_alloc();
}

void operator delete(void* p) noexcept
{
    std::free(p);
}

void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}

namespace aircraft
{
namespace device
{
namespace
{

// ---------------------------------------------------------------------------
//  测试夹具辅助
// ---------------------------------------------------------------------------

data::CameraConfig makeCameraConfig(data::CameraRole role, const std::string& id)
{
    data::CameraConfig c;
    c.cameraId     = id;
    c.role         = role;
    c.width        = 640;
    c.height       = 480;
    c.exposureTime = 0.005;  // s
    c.gain         = 3.0;    // dB
    // ⚠ 本批起 `triggerMode` 是三值枚举（ENG-09 V2.3 §5.31）。
    //    原 `true` 的语义是"触发使能"（即硬触发），故映射为 `Hardware`：
    //    本文件的多路取帧用例**不关心**触发方式，而 `Hardware` 下
    //    `MultiCameraManager` 不会插进一条软件触发令（那是 PH-01 的范围），
    //    取帧行为与改动前逐字相同。触发路径本身的用例见 §4.4 的软件触发组。
    c.triggerMode  = data::CameraTriggerMode::Hardware;
    return c;
}

std::shared_ptr<VirtualCameraBackend> makeVirtualCamera(data::CameraRole role,
                                                        const std::string& id)
{
    return std::make_shared<VirtualCameraBackend>(makeCameraConfig(role, id));
}

/// 构造三路虚拟相机的管理器。
std::unique_ptr<MultiCameraManager> makeManager()
{
    return std::make_unique<MultiCameraManager>(
        makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
        makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
        makeVirtualCamera(data::CameraRole::CAM100, "cam100"));
}

/// 取帧期限：本文件绝大多数用例只关心"正常取到帧没有"，故给一个足够远的
/// 期限，使期限判据不会成为这些用例的**伪因**。
///
/// ⚠ 时基与 `MultiCameraManager` 的**默认**钟同源（真实单调钟）——
///    本文件不注入假钟，故"现在"必须取真实单调钟；若这里改用某个常数，
///    期限会被真实墙钟一上来就判过期，整文件会以与用例意图无关的原因变红。
///    期限判据本身（三个量取最小、到期不取帧、不缓存上一轮期限）由
///    `CaptureDeadline*` 那组用例**注入假钟**专门验证。
uint64_t farDeadlineNs()
{
    return data::monotonicNowNs() + 60ULL * 1000ULL * 1000ULL * 1000ULL;
}

data::TurntableConfig makeTurntableConfig()
{
    data::TurntableConfig tc;
    tc.protocol       = "sdk";
    tc.azimuthMin     = -30.0;   // deg
    tc.azimuthMax     = 30.0;    // deg
    tc.elevationMin   = -10.0;   // deg
    tc.elevationMax   = 40.0;   // deg
    tc.coarseSpeed    = 5.0;     // deg/s
    tc.fineSpeed      = 0.5;     // deg/s
    tc.centerThreshold = 50.0;   // pixel
    return tc;
}

/// 按**脚本**报出 frameId 的相机替身（裁决 D-C02-6 的验证手段）。
///
/// ⚠ 为什么不用 `VirtualCameraBackend`：它的计数器恒从 0 起且只增，
/// 于是 `MultiCameraManager` 的"首帧基准锁定"退化为恒等映射
/// （基准恒为 0，减法与不减法得到同一个值），而**真机上必然出现**的
/// 两种情形在测试里根本无法出现：
///   · 三台相机的计数器各有任意起点（相差成千上万）；
///   · 相机重连 / 计数器回绕导致帧号**回退**。
/// 只测虚拟相机会得到一个"看起来通过"的结论。
///
/// 脚本用尽后重复最后一个值 —— 用于构造"帧号不前进"的后端
/// （即 `ImvCameraBackend` 尚未填 frameId 时的真实表现：恒为 0）。
class ScriptedCameraBackend : public ICameraBackend
{
public:
    ScriptedCameraBackend(data::CameraRole role, std::vector<uint64_t> frameIds)
        : role_(role)
        , frameIds_(std::move(frameIds))
    {
    }

    /// 令接下来的 N 次 grab 失败（构造"该路中途掉线"）。
    int failNextGrabs = 0;

    /// 令接下来的 N 次 grab 按指定的**分类**失败。
    ///
    /// ⚠ 为什么需要它（而不是只靠 `failNextGrabs`）：本批的核心修正正是
    ///    "失败要有分类"，而 `failNextGrabs` 只能构造一种失败 ——
    ///    上一版用例因此钉住了"任何失败都永久禁用该通道"这条被 R09
    ///    点名的失效形态。要按分类断言（超时不禁用、断连才禁用），
    ///    失败就必须**可分类地**注入。
    /// 队列用尽后回到 `failNextGrabs` 的语义（`Timeout`）。
    std::vector<data::OpStatus> failStatusQueue;

    /// 令该路在同一轮内**取帧成功但图为空**（一次"取到空图"）。
    bool returnEmptyImage = false;

    /// 令该路的结果带上**清理失败**（帧缓冲**释放未获确认**）。
    ///
    /// ⚠ V2.5 起它与主状态**分开记录**，而管理器的通道处置**只看它** ——
    ///    本桩据此如实给出两种形态（见 `applyCleanupFailure`）：
    ///      · 主操作成功 + 清理失败 ⇒ 整体失败（`SdkError`）、帧不交付；
    ///      · 已有主失败 + 清理失败 ⇒ **保留首因**，失败只进 `cleanupError`。
    ///    上一版这两条路径分别落在 `sdkError` 与 `cleanupError`，
    ///    于是同一种"释放未获确认"在管理器那里拿到了两种处置。
    std::optional<data::SdkFailure> cleanupFailure;

    /// `grab()` 被调用的次数（断言"未尝试时一次都没有"）。
    int grabCalls = 0;

    /// 最近一次 `grab()` 收到的 `timeoutMs` 实参。
    ///
    /// ⚠ 必须记录**实参**而不只是"被调用过"：本批的核心行为之一
    ///    （§2.2 三个量取最小）只体现在这个数上 —— 调用次数相同而
    ///    传入的等待上限不同，行为完全不同，而次数断言对此全无感觉。
    uint32_t lastTimeoutMs = 0;

    /// `triggerSoftware()` 被调用的次数与顺序。
    int                      triggerCalls = 0;
    std::vector<std::string> callOrder;   ///< "trigger" / "grab" 的调用顺序

    /// `triggerModeState()` 被调用的次数（**读写回**的次数）。
    ///
    /// ⚠ §3 的检查点 2（读回之后、发令之前）只能用这个计数来驱动假钟：
    ///    真实后端的三项读回是三次阻塞 SDK 读、要花时间，而"读回把预算
    ///    吃掉了"这一情形只有让时间**随读回推进**才构造得出来。
    ///    `mutable` 是因为接口把 `triggerModeState()` 定为 const。
    mutable int triggerModeStateCalls = 0;

    /// `triggerSoftware()` 的注入返回值（默认 `Ok`）。
    /// 非 `Ok` 时用于验证"发令失败 ⇒ 本路不取帧"。
    data::OperationResult triggerResult{data::OpStatus::Ok, std::nullopt,
                                        std::nullopt};

    /// 令 `grab()` 返回**默认构造**的结果（`status == Unset`）。
    /// 这是"后端漏填 status"这一代码缺陷的替身：它不是任何一种设备故障，
    /// 故必须与"一次瞬态失败"得到不同的记录（§4.2 的 `Unset` 用例）。
    bool returnUnsetResult = false;

    /// `lastErrorText()` 的返回值（默认空串＝"本后端没有可说的"）。
    /// 脚本可设成一句话，用于验证装配层**确实把后端的说明贴进了**
    /// 用户可见的失败文本（而不是贴一句"明细见各后端自身日志"）。
    std::string errorText;

    /// 令 `triggerModeState()` 报出**读不到模式**（§7 的第一种失败）。
    ///
    /// ⚠ 造的是"**调用成功但没读到值**"（`returnedEmpty`），**不是**"调用失败"
    ///    （`failure`）—— 两者必须可区分，是 §7 裁决的明文要求。
    bool reportUnknownTriggerMode = false;

    /// 令 `triggerModeState()` 报出**与请求不同**的模式（§7 的第二种失败）。
    ///
    /// ⚠ 与 `reportUnknownTriggerMode` 互斥优先：本项一旦有值，就按"读回完整
    ///    但与请求不一致"回报（不是"读不到"）。
    std::optional<data::CameraTriggerMode> reportOtherTriggerMode;

    // ---- ICameraBackend（011-A1 的六个方法）----

    data::OperationResult initialize() override
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }
    data::OperationResult start() override
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }
    data::OperationResult stop() override
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }
    data::OperationResult close() override
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    /// 本桩的"设备"就是它自己：请求什么就回报什么，故三项都能读回。
    ///
    /// ⚠ 三项**逐项**填 `readbacks`（§7 裁决）：只有取值时，一次读失败在
    ///    记录里只剩一个"空"，回答不了"是哪一项、是调用失败还是回了空串"。
    data::TriggerModeState triggerModeState() const override
    {
        ++triggerModeStateCalls;
        data::TriggerModeState s;
        s.requested = mode_;

        // 本桩"三项都调过"，故 `callAttempted` 恒为真；名字取自 data 层的
        // 规范名（下标与 readbacks 一一对应），不在这里手写第二份字面量。
        for (std::size_t i = 0; i < data::kTriggerFeatureCount; ++i)
        {
            s.readbacks[i].feature       = data::triggerFeatureName(i);
            s.readbacks[i].callAttempted = true;
        }

        if (reportUnknownTriggerMode)
        {
            // "调用**成功**但没读到值"：`failure` 保持空、以 `returnedEmpty`
            // 表达 —— 与"SDK 调用失败"必须可区分（§7 裁决）。
            // `switchReported` 留空串 ⇒ `composeTriggerModeState` 合成
            // `reported = nullopt`（诚实地说"不知道"）。
            for (std::size_t i = 0; i < data::kTriggerFeatureCount; ++i)
            {
                s.readbacks[i].returnedEmpty = true;
            }
            s.selectorReported = std::string("FrameStart");
            s.switchReported   = std::string("");
            return data::composeTriggerModeState(s);
        }

        // 读回**完整**：有 `reportOtherTriggerMode` 时回报那个（"设了却没生效"），
        // 否则如实回报本桩持有的模式。
        const data::CameraTriggerMode reported = reportOtherTriggerMode.value_or(mode_);
        s.selectorReported = std::string("FrameStart");
        s.switchReported   = std::string(data::triggerModeSymbol(reported));
        s.readbacks[0].value = *s.selectorReported;
        s.readbacks[1].value = *s.switchReported;
        if (const char* src = data::triggerSourceSymbol(reported))
        {
            s.sourceReported     = std::string(src);
            s.readbacks[2].value = *s.sourceReported;
        }
        return data::composeTriggerModeState(s);
    }

    data::OperationResult setTriggerMode(data::CameraTriggerMode mode) override
    {
        mode_ = mode;
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    data::DeviceIdentity deviceIdentity() const override
    {
        // 本桩不连任何设备：`queried = false` 表示"没问过 SDK"，
        // 与"问过但没取到"是两件事（后者 `queried = true` 而字段为空）。
        return data::DeviceIdentity{};
    }

    /// 本桩的"失败说明"由脚本自己给：产物要能被 `SystemInitializer` 直接
    /// 贴进用户可见的失败文本，故这里返回**脚本设置的那句话**（默认空）。
    /// ⚠ 若返回空串，装配层的取值顺序会落到管理器的错误文本上 —— 正是
    ///    本方法存在的意义：让"后端自己知道原因"这件事有出口。
    std::string lastErrorText() const override { return errorText; }

    data::OperationResult triggerSoftware() override
    {
        ++triggerCalls;
        callOrder.push_back("trigger");
        return triggerResult;
    }

    data::GrabResult grab(data::ImageFrame& frame, uint32_t timeoutMs) override
    {
        ++grabCalls;
        lastTimeoutMs = timeoutMs;
        callOrder.push_back("grab");

        if (returnUnsetResult)
        {
            // ⚠ 返回**默认构造**的结果（`status == Unset`），并且**不碰**
            //    `frame` —— 这正是一个"忘了写返回值"的实现会做的事。
            return data::GrabResult{};
        }

        if (!failStatusQueue.empty())
        {
            const data::OpStatus st = failStatusQueue.front();
            failStatusQueue.erase(failStatusQueue.begin());
            // 失败路径**不碰 frame**（接口契约）。
            return applyCleanupFailure(
                data::GrabResult{{st, std::nullopt, std::nullopt}}, frame);
        }
        if (failNextGrabs > 0)
        {
            --failNextGrabs;
            return applyCleanupFailure(
                data::GrabResult{
                    {data::OpStatus::Timeout, std::nullopt, std::nullopt}},
                frame);
        }
        if (frameIds_.empty())
        {
            return applyCleanupFailure(
                data::GrabResult{
                    {data::OpStatus::Timeout, std::nullopt, std::nullopt}},
                frame);
        }

        const std::size_t i = std::min(cursor_, frameIds_.size() - 1);
        ++cursor_;

        frame              = data::ImageFrame{};
        frame.role         = role_;
        frame.cameraId     = "scripted";
        frame.frameId      = frameIds_[i];
        frame.timestampNs  = 1000;
        frame.exposureTime = 0.005;
        frame.image        = returnEmptyImage ? cv::Mat{}
                                              : cv::Mat::ones(8, 8, CV_8UC1);
        // ⚠ 正常取帧时**照样**填帧状态原值 `0`（§2／§9）：它是"取到了、
        //    设备说这帧没问题"，与失败路径的"未取得"（`nullopt`）必须可区分。
        //    本桩虽然不调 SDK，但它替身的就是"一台正常出图的相机"。
        //
        // ⚠ `returnEmptyImage` 那一路**不填**：它替身的是一个"报了 Ok 却没有
        //    帧"的后端 —— 那种情形下根本不存在"设备对某一帧的判定"，
        //    填 0 就等于替它编了一个事实（见 DeviceLayerTest 的空图用例）。
        data::GrabResult ok{{data::OpStatus::Ok, std::nullopt, std::nullopt}};
        if (!returnEmptyImage)
        {
            ok.frameStatusRaw = 0;
        }
        return applyCleanupFailure(ok, frame);
    }

private:
    /// 把 `cleanupFailure` 注入应用到一条结果上（V2.5 的两种形态见字段说明）。
    data::GrabResult applyCleanupFailure(data::GrabResult r,
                                         data::ImageFrame& frame) const
    {
        if (!cleanupFailure.has_value())
        {
            return r;
        }
        r.cleanupError = cleanupFailure;
        if (r.ok())
        {
            // 忠实模拟真实后端 `mergeCleanup` 的输出：主操作成功而必要清理
            // 失败 ⇒ **整体返回失败**，以该清理调用为错误来源，帧**不交付**。
            // （`classify(ImvReleaseFrame, −119)` 就是 `SdkError`，与这里
            //   写死的分类一致；本桩不调 SDK，故按同一映射给出同一结果。）
            r.status   = data::OpStatus::SdkError;
            r.sdkError = cleanupFailure;
            r.frameStatusRaw.reset();
            frame = data::ImageFrame{};
        }
        return r;
    }

    data::CameraRole       role_;
    std::vector<uint64_t>  frameIds_;
    std::size_t            cursor_ = 0;

    /// 本桩持有的触发模式（`triggerModeState()` 据此如实回报）。
    data::CameraTriggerMode mode_ = data::CameraTriggerMode::Hardware;
};

/// 用脚本相机装配一个三路管理器（三路起点互不相同，模拟真机）。
std::unique_ptr<MultiCameraManager> makeScriptedManager(
    std::vector<uint64_t> cam25,
    std::vector<uint64_t> cam50,
    std::vector<uint64_t> cam100)
{
    return std::make_unique<MultiCameraManager>(
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM25,
                                               std::move(cam25)),
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM50,
                                               std::move(cam50)),
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM100,
                                               std::move(cam100)));
}

// ---------------------------------------------------------------------------
//  MultiCameraManager —— SYS-08 §7.5 降级表第 1 行：3 台正常
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, ThreeCamerasCaptureAllChannels)
{
    auto mgr = makeManager();

    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    EXPECT_EQ(mgr->availableCameraCount(), 3);
    EXPECT_FALSE(mgr->degraded());
    EXPECT_EQ(mgr->state(), data::DeviceState::RUNNING);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));

    EXPECT_FALSE(frame.cam25.image.empty());
    EXPECT_FALSE(frame.cam50.image.empty());
    EXPECT_FALSE(frame.cam100.image.empty());
}

TEST(MultiCameraManagerTest, CaptureFillsRoleFieldOnEveryChannel)
{
    // role 由管理器按通道补齐，而非依赖 backend 自行填写。
    // 若缺失，会出现"frame.cam25 里装的是 CAM100 的图"这类
    // 无法从数据本身察觉的错位，而三相机评分链完全依赖它。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));

    EXPECT_EQ(frame.cam25.role, data::CameraRole::CAM25);
    EXPECT_EQ(frame.cam50.role, data::CameraRole::CAM50);
    EXPECT_EQ(frame.cam100.role, data::CameraRole::CAM100);
}

TEST(MultiCameraManagerTest, TriggerTimestampIsMaxOfChannelTimestamps)
{
    // ENG-09 §2.5 冻结：triggerTimestamp = 各相机 timestampNs 的最大值。
    // 取最大而非均值，保证它不早于任何一路的实际曝光时刻。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));

    const uint64_t expected = std::max({frame.cam25.timestampNs,
                                       frame.cam50.timestampNs,
                                       frame.cam100.timestampNs});
    EXPECT_EQ(frame.triggerTimestamp, expected);
}

// ---------------------------------------------------------------------------
//  SYS-08 §7.5 降级表第 2 行：2 台 → 降级继续
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, TwoCamerasDegradeButCaptureSucceeds)
{
    // 关键判据：capture() **必须返回 true**。
    // 把"少一台"当作失败会白白丢弃 §7.5〔引用无效·依据待裁决·见 Q-D2〕 明确允许的降级能力，
    // 让一次本可完成的测量变成 FAILED。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    ASSERT_TRUE(mgr->disableChannel(data::CameraRole::CAM100));

    EXPECT_EQ(mgr->availableCameraCount(), 2);
    EXPECT_TRUE(mgr->degraded());

    data::MultiCameraFrame frame;
    EXPECT_TRUE(mgr->capture(frame, farDeadlineNs()));

    // 不可用的通道留默认构造的 ImageFrame，以 image.empty() 判定 ——
    // 而不是引入哨兵枚举值（见 CameraRole.h 的说明）。
    EXPECT_TRUE(frame.cam100.image.empty());
    EXPECT_FALSE(frame.cam25.image.empty());
    EXPECT_FALSE(frame.cam50.image.empty());
}

TEST(MultiCameraManagerTest, CaptureMidwayFaultDisablesChannelWithoutRetrying)
{
    // SYS-08 §7.5：采集途中掉线属硬件故障，**不重试**，直接禁用。
    // 若在此处重试，一个物理上不可恢复的故障会消耗掉 §7.3 的重试预算，
    // 使真正属于瞬态类的失败失去重试机会。
    auto cam100 = makeVirtualCamera(data::CameraRole::CAM100, "cam100");
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
                           cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    cam100->simulateFault(true);

    data::MultiCameraFrame frame;
    EXPECT_TRUE(mgr.capture(frame, farDeadlineNs()));  // 2 台仍可用 → 降级继续
    EXPECT_EQ(mgr.availableCameraCount(), 2);
    EXPECT_TRUE(mgr.degraded());
}

// ---------------------------------------------------------------------------
//  SYS-08 §7.5 降级表第 3 行：≤1 台 → 直接失败，code=1001
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, InsufficientCamerasFailsWithoutConsumingBudget)
{
    auto cam50  = makeVirtualCamera(data::CameraRole::CAM50, "cam50");
    auto cam100 = makeVirtualCamera(data::CameraRole::CAM100, "cam100");
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           cam50, cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    cam50->simulateFault(true);
    cam100->simulateFault(true);

    data::MultiCameraFrame frame;
    EXPECT_FALSE(mgr.capture(frame, farDeadlineNs()));
    EXPECT_EQ(mgr.availableCameraCount(), 1);

    // ENG-09 §5.27：代码中不得出现裸错误码字面量，一律用具名常量
    EXPECT_EQ(mgr.lastError().code, data::kErrCameraInsufficient);
}

TEST(MultiCameraManagerTest, NullBackendsCountAsUnavailable)
{
    // 只注入两路：这是"现场只装了两台相机"的正常配置，
    // 应降级工作而非初始化失败。
    MultiCameraManager mgr(makeVirtualCamera(data::CameraRole::CAM25, "cam25"),
                           makeVirtualCamera(data::CameraRole::CAM50, "cam50"),
                           nullptr);
    EXPECT_TRUE(mgr.initializeAll());
    EXPECT_EQ(mgr.availableCameraCount(), 2);
    EXPECT_TRUE(mgr.degraded());
}

TEST(MultiCameraManagerTest, CaptureBeforeStartFails)
{
    // 未 start 就采集：拒绝，而不是把 SDK 的未定义行为泄漏给上层。
    auto mgr = makeManager();
    ASSERT_TRUE(mgr->initializeAll());

    data::MultiCameraFrame frame;
    EXPECT_FALSE(mgr->capture(frame, farDeadlineNs()));
}

// ---------------------------------------------------------------------------
//  MultiCameraFrame::exposureIndex —— 裁决 D-C02-6
//
//  这组用例的全部意义在于：`exposureIndex` 是"这几张 raw 是不是同一次
//  曝光"的**唯一**判据，而它一旦取错（用绝对帧号、用某一路的帧号、
//  帧号回退时不重锁），错误值**看起来仍然是一个合法的正整数** ——
//  没有任何下游会报错。故必须用可控的帧号序列把每条边都走到。
// ---------------------------------------------------------------------------

TEST(MultiCameraManagerTest, ExposureIndexStartsAtOneAndIgnoresDeviceFrameIdOrigin)
{
    // 三台相机的计数器起点互不相同（真机上必然如此，SYS-06 §8 只要求
    // 各机自身单调，从未要求三机同起点）。
    auto mgr = makeScriptedManager({100, 101, 102}, {7, 8, 9}, {0, 1, 2});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));

    // 一次 `capture()` 恰对应一次触发曝光 ⇒ 第一次采集的序号必须是 1。
    // 0 的含义是**未定**（MultiCameraFrame 的字段说明），不得占用。
    EXPECT_EQ(frame.exposureIndex, 1u)
        << "首次曝光应为 1；若等于某一路的绝对帧号（100/7/0），"
           "说明基准锁定没有生效";

    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));
    EXPECT_EQ(frame.exposureIndex, 2u) << "第二次曝光应为 2";
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));
    EXPECT_EQ(frame.exposureIndex, 3u);

    // 基准锁定是正常路径，不得借"降级"标记蒙混过关
    // （它一旦置位，调用方就没有别的办法判断序号是否可信）。
    EXPECT_FALSE(mgr->exposureIndexDegraded())
        << "帧号单调前进时不应报降级";
}

TEST(MultiCameraManagerTest, ExposureIndexFlagsDegradedOnFrameIdRollbackWithoutLosingMonotonicity)
{
    // 相机重连 / 计数器回绕：第 3 帧的帧号回到 3（小于基准 5）。
    auto mgr = makeScriptedManager({5, 6, 3, 4}, {0, 1, 2, 3}, {0, 1, 2, 3});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    uint64_t previous = 0;
    const uint64_t expected[4] = {1, 2, 3, 4};

    for (int i = 0; i < 4; ++i)
    {
        ASSERT_TRUE(mgr->capture(frame, farDeadlineNs())) << "第 " << (i + 1) << " 次采集失败";

        // 不变量：序号**严格递增**（调用方靠它区分"同一次曝光"与"下一次"）。
        EXPECT_GT(frame.exposureIndex, previous)
            << "第 " << (i + 1) << " 次采集的序号没有前进";
        EXPECT_EQ(frame.exposureIndex, expected[i])
            << "第 " << (i + 1) << " 次采集：帧号回退后应就地重锁基准，"
               "而不是下溢成一个巨大的序号";
        previous = frame.exposureIndex;
    }

    // 重锁是"序号已不能直接反映设备帧号"这件事，必须可见。
    EXPECT_TRUE(mgr->exposureIndexDegraded())
        << "帧号回退后未置降级标记 —— 序号来源已经换人却无人知晓";
}

TEST(MultiCameraManagerTest, ExposureIndexFlagsDegradedWhenFrameIdDoesNotAdvance)
{
    // 后端不填 frameId（`ImvCameraBackend` 实施前的真实表现：恒为 0）。
    // 此时减法结果恒等于 1 —— 而 1 恰好是一个"合法"的序号，
    // 于是"这几张 raw 是不是同一次曝光"的判断会**恒为真**。
    auto mgr = makeScriptedManager({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));
    EXPECT_EQ(frame.exposureIndex, 1u);

    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));
    ASSERT_TRUE(mgr->capture(frame, farDeadlineNs()));

    // 保住单调性（这是调用方依赖的不变量），同时把"序号不再来自设备帧号"
    // 记录成可见事实。
    EXPECT_EQ(frame.exposureIndex, 3u)
        << "帧号不前进时序号不得停在 1（恒值会让同源判断永远为真）";
    EXPECT_TRUE(mgr->exposureIndexDegraded())
        << "帧号不前进却未报降级";
}

TEST(MultiCameraManagerTest, ExposureIndexKeepsAdvancingWhenReferenceChannelDrops)
{
    // 第 3 次采集起 CAM25 掉线：参考路按 CAM25→CAM50→CAM100 顺序换成
    // CAM50，而 CAM50 的计数器起点与 CAM25 不同（10 vs 0）。
    auto cam25 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM25, std::vector<uint64_t>{10, 11, 12, 13});
    auto cam50 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM50, std::vector<uint64_t>{0, 1, 2, 3});
    auto cam100 = std::make_shared<ScriptedCameraBackend>(
        data::CameraRole::CAM100, std::vector<uint64_t>{0, 1, 2, 3});

    MultiCameraManager mgr(cam25, cam50, cam100);
    ASSERT_TRUE(mgr.initializeAll());
    ASSERT_TRUE(mgr.startAll());

    data::MultiCameraFrame frame;
    ASSERT_TRUE(mgr.capture(frame, farDeadlineNs()));
    EXPECT_EQ(frame.exposureIndex, 1u);
    ASSERT_TRUE(mgr.capture(frame, farDeadlineNs()));
    EXPECT_EQ(frame.exposureIndex, 2u);

    cam25->failNextGrabs = 2;   // 第 3 次起 CAM25 连续丢帧

    uint64_t previous = frame.exposureIndex;
    for (int i = 0; i < 2; ++i)
    {
        ASSERT_TRUE(mgr.capture(frame, farDeadlineNs())) << "两路可用仍应采集成功（§7.5 降级）";
        EXPECT_GT(frame.exposureIndex, previous)
            << "换参考路后序号必须继续前进，否则两次曝光会被判为同一次";
        previous = frame.exposureIndex;
    }

    EXPECT_TRUE(mgr.exposureIndexDegraded())
        << "参考路换人后未置降级标记";
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §4.2 分类消费、每路结果与聚合
// ===========================================================================

namespace
{
/// 假钟的基准时刻。取一个非零值：**零**在本工程里是"未定/未设置"的
/// 惯用哨兵（`MultiCameraFrame::exposureIndex`、`timestampNs` 都是），
/// 用它当"现在几点"会让"期限为 0"之类的判断看不出区别。
constexpr uint64_t kFakeEpochNs = 1000000000ULL;   // 1 s

/// 三路脚本相机的装配体：用例可直接拿到每一路的替身来注入。
///
/// ⚠ 为什么需要它（而不是复用 `makeScriptedManager()`）：本节用例要
///    **逐路**注入不同的失败分类、不同的触发模式、不同的返回结果，
///    而 `makeScriptedManager()` 把三个替身直接交给了管理器，
///    装配完之后用例就再也拿不到它们了。
struct ScriptedRig
{
    std::shared_ptr<ScriptedCameraBackend> cam25;
    std::shared_ptr<ScriptedCameraBackend> cam50;
    std::shared_ptr<ScriptedCameraBackend> cam100;
    std::unique_ptr<MultiCameraManager>    mgr;

    /// 本装配体的假钟（常量，`kFakeEpochNs`）。需要让钟走动的用例
    /// 用**自己的**变量重新 `setClock()`（见 §4.3 那几条）。
    uint64_t nowNs = kFakeEpochNs;

    explicit ScriptedRig(std::vector<uint64_t> ids = {1, 2, 3})
    {
        cam25  = std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM25, ids);
        cam50  = std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM50, ids);
        cam100 = std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM100, ids);
        mgr    = std::make_unique<MultiCameraManager>(cam25, cam50, cam100);

        // ⚠ **默认就注入假钟**，不让用例各自记着去注：
        //   本节的期限实参来自假时基（`kFakeEpochNs + …`），而管理器的
        //   默认钟是真实单调钟 —— 两者混用的结果是"模拟期限一上来就被
        //   墙钟判过期"，于是**每一路都取不到帧**，而失败现象看起来像
        //   "相机不工作"。把注入放进装配体，使"同一时基"成为默认事实。
        mgr->setClock([this] { return nowNs; });

        mgr->initializeAll();
        mgr->startAll();
    }
};

}  // namespace

TEST(MultiCameraManagerTest, ReleaseUnconfirmedStopsChannelEvenWhenFrameChecksPassed)
{
    // V2.5 的统一规则：**释放未获确认 ⇒ 停止该路后续采集**，且**不覆盖首因**。
    // 本用例是"帧检查**通过**、而释放失败"那一行（A1 不一致的两行之一）：
    //   · 主状态被提升为失败（`SdkError`）—— 帧**不交付**、不计入有效帧；
    //   · 通道**停止**；
    //   · 措辞必须是"释放未获确认"，**不得**写成"分类未明确" ——
    //     后者会让人去查设备，而真正该查的是**缓冲归属**。
    ScriptedRig rig;
    rig.cam100->cleanupFailure =
        data::SdkFailure{data::SdkCall::ImvReleaseFrame, -119};

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_FALSE(rig.mgr->channelAvailable(data::CameraRole::CAM100))
        << "释放未获确认的通道必须停止后续采集";
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM25))
        << "正常通道不得被牵连";

    const data::CaptureRound      r   = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = r.channel(data::CameraRole::CAM100);
    EXPECT_EQ(rec.result.status, data::OpStatus::SdkError)
        << "主操作成功而必要清理失败 ⇒ 整体失败（帧不交付）";
    ASSERT_TRUE(rec.result.cleanupError.has_value())
        << "释放未获确认必须进 cleanupError（管理器的处置只看它）";
    EXPECT_EQ(rec.result.cleanupError->call, data::SdkCall::ImvReleaseFrame);
    EXPECT_NE(rec.skippedReason.find("未获确认"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rec.skippedReason.find("分类未明确"), std::string::npos)
        << "释放问题不得被写成「分类未明确」：" << rec.skippedReason;
    EXPECT_EQ(r.capturedCount, 2) << "未交付的帧不得计入有效帧";
}

TEST(MultiCameraManagerTest, ReleaseUnconfirmedStopsChannelAndKeepsFirstCause)
{
    // 同一规则的另一行：**帧已损坏、且释放也失败**。
    //
    // ⚠ 上一版这里"**继续可用**"（`mergeCleanup` 保留首因，失败只进
    //   `cleanupError`，而管理器只按 `status` 判处置）—— 同一种
    //   "释放未获确认"，与上一行得到**相反**的处置，区别只在于
    //   "帧本身好不好"，而帧好不好与"缓冲还回去了没有"是两个不相干的问题。
    //   V2.5 起两行统一：都停止，且**首因一字不动**（仍是 `CorruptFrame`）。
    ScriptedRig rig;
    rig.cam100->failStatusQueue.push_back(data::OpStatus::CorruptFrame);
    rig.cam100->cleanupFailure =
        data::SdkFailure{data::SdkCall::ImvReleaseFrame, -119};

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_FALSE(rig.mgr->channelAvailable(data::CameraRole::CAM100))
        << "释放未获确认的通道必须停止后续采集 —— 与主失败是什么无关";

    const data::CaptureRound      r   = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = r.channel(data::CameraRole::CAM100);
    EXPECT_EQ(rec.result.status, data::OpStatus::CorruptFrame)
        << "首因不得被清理失败覆盖";
    EXPECT_FALSE(rec.result.sdkError.has_value())
        << "帧状态非零是设备对帧自身的判定，不是 SDK 调用失败 ⇒ 不得伪造调用失败";
    ASSERT_TRUE(rec.result.cleanupError.has_value());
    EXPECT_EQ(rec.result.cleanupError->code, -119);
    EXPECT_NE(rec.skippedReason.find("未获确认"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(r.capturedCount, 2);
}

TEST(MultiCameraManagerTest, ThrownReleaseMarkerNeverReachesHumanReadableText)
{
    // 011-A1 缺口 5／6 的显示责任：`kCallThrewCode` 是**标记值、不是 SDK 返回码**。
    // 这条路径**真实可达**（不是构造出来的）：`FrameLeaseGuard::cleanup()` 是
    // `noexcept` 的，它把"释放调用抛出"如实记成 `{ImvReleaseFrame, kCallThrewCode}`，
    // 于是**正常返回**的结果里就带着这个标记值进了本轮记录。
    //
    // ⚠ 只要有一处把它当数字打印，现场就会看到 `-2147483648`，
    //   然后拿着这个值去查一个不存在的 SDK 错误码 —— 这是"假的具体值"
    //   比"缺值"更糟的又一例。故断言**两处人读出口**都只出现措辞。
    ScriptedRig rig;
    rig.cam100->cleanupFailure =
        data::SdkFailure{data::SdkCall::ImvReleaseFrame, data::kCallThrewCode};

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_FALSE(rig.mgr->channelAvailable(data::CameraRole::CAM100))
        << "释放未获确认（这里是「调用抛出」）同样必须停止该路后续采集";

    const data::CaptureRound      r   = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = r.channel(data::CameraRole::CAM100);
    ASSERT_TRUE(rec.result.cleanupError.has_value());
    EXPECT_EQ(rec.result.cleanupError->code, data::kCallThrewCode)
        << "标记值必须原样保留在**机器读**的字段里（读包方按同一定义解释）";

    // ① 交付/记录文本（`data::ChannelGrabRecord` 的人读出口）
    const std::string recordText = data::channelGrabRecordText(rec);
    EXPECT_NE(recordText.find("调用抛出异常、无返回码"), std::string::npos) << recordText;
    EXPECT_EQ(recordText.find("-2147483648"), std::string::npos)
        << "标记值**不得**被当 SDK 原码念出来：" << recordText;

    // ② 该路被停止时的原因文本（管理器的出口）
    EXPECT_NE(rec.skippedReason.find("调用抛出异常、无返回码"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rec.skippedReason.find("-2147483648"), std::string::npos)
        << "标记值**不得**被当 SDK 原码念出来：" << rec.skippedReason;
    EXPECT_NE(rec.skippedReason.find("未获确认"), std::string::npos)
        << "处置措辞仍须是「释放未获确认」：" << rec.skippedReason;
}

TEST(MultiCameraManagerTest, PerChannelStatusTableDecidesAvailabilityAndAggregate)
{
    // §2.2 的**状态全表逐行**：每一行都要能从上层的角度观察到三件事 ——
    //   ①该路是否被禁用  ②聚合状态  ③应用错误码。
    // 三件事缺一条，这张表在文档里就只是一段散文。
    //
    // ⚠ 只让 CAM100 一路失败：另外两路成功 ⇒ `captured == 2` 恰好达标，
    //   于是"失败分类"与"就绪性不足（1001）"被彻底分开 —— 若三路一起注入，
    //   绝大多数用例都会撞在 `captured < 2` 那条分支上，分类是否被正确消费
    //   就无从观察了。
    struct Row
    {
        data::OpStatus status;
        bool           disables;
        int            appCode;
        const char*    name;
    };
    const Row rows[] = {
        {data::OpStatus::Timeout,           false, 1003, "Timeout"},
        {data::OpStatus::NoFrame,           false, 1003, "NoFrame"},
        {data::OpStatus::NotStarted,        false, 1004, "NotStarted"},
        {data::OpStatus::CorruptFrame,      false, 1005, "CorruptFrame"},
        {data::OpStatus::SdkError,          true,  1005, "SdkError"},
        {data::OpStatus::Disconnected,      true,  1002, "Disconnected"},
        {data::OpStatus::NotImplemented,    false, 1007, "NotImplemented"},
        {data::OpStatus::InvalidArgument,   false, 1006, "InvalidArgument"},
        {data::OpStatus::ContractViolation, false, 1006, "ContractViolation"},
    };

    for (const Row& row : rows)
    {
        ScriptedRig rig;
        rig.cam100->failStatusQueue.push_back(row.status);

        data::MultiCameraFrame frame;
        EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL))
            << row.name << "：两路成功时本轮必须整体成功";

        EXPECT_EQ(rig.mgr->channelAvailable(data::CameraRole::CAM100),
                  !row.disables)
            << row.name << "：通道可用性判定错（禁用集合见 §3.2 的分类消费表）";
        EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM25))
            << row.name << "：正常通道不得被牵连禁用";

        const data::CaptureRound r = rig.mgr->lastCaptureRound();
        EXPECT_EQ(r.channel(data::CameraRole::CAM100).result.status, row.status)
            << row.name << "：管理器改写了后端给的分类";
        EXPECT_EQ(r.aggregate, row.status) << row.name << "：聚合取错了来源";
        EXPECT_EQ(data::appErrorCodeOf(r.aggregate), row.appCode)
            << row.name << "：应用错误码与状态不符";
        EXPECT_EQ(r.capturedCount, 2) << row.name << "：有效帧计数错";
    }
}

TEST(MultiCameraManagerTest, LocalErrorsCarryNoSdkErrorAndDoNotMasqueradeAsSdkError)
{
    // §2.2：本地判定的错误（`InvalidArgument`／`ContractViolation`）
    // **根本没有调用 SDK**，故 `sdkError` 必须是 `nullopt` —— 伪造一个
    // "调用过"会让离线排查去找一个不存在的 SDK 返回码。
    // 且应用码是 1006 而**不是** 1005：把"我们自己的代码/配置有问题"
    // 与"相机给的帧有问题"记成同一个码，现场动作会被引到设备侧。
    ScriptedRig rig;
    rig.cam50->failStatusQueue.push_back(data::OpStatus::ContractViolation);

    data::MultiCameraFrame frame;
    rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL);

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM50);
    EXPECT_EQ(rec.result.status, data::OpStatus::ContractViolation);
    EXPECT_FALSE(rec.result.sdkError.has_value())
        << "本地契约错误不得带 SDK 失败归属";
    EXPECT_EQ(data::appErrorCodeOf(data::OpStatus::ContractViolation), 1006);
    EXPECT_NE(data::appErrorCodeOf(data::OpStatus::ContractViolation), 1005)
        << "本地契约错误不得冒充 SDK 错误";
}

TEST(MultiCameraManagerTest, DisabledChannelReasonWordingTellsTheTruth)
{
    // §3.2：两条禁用措辞**不得互换**，也不得写成"已确认断连"。
    // 断言文本而不是只断言"被禁用了"：措辞就是本批要固化的事实本身 ——
    // 本批**不注册** `IMV_SubscribeConnectArg`，故没有任何事件确认发生过。
    {
        ScriptedRig rig;
        rig.cam50->failStatusQueue.push_back(data::OpStatus::Disconnected);
        data::MultiCameraFrame frame;
        rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL);

        const std::string why =
            rig.mgr->lastCaptureRound().channel(data::CameraRole::CAM50).skippedReason;
        EXPECT_NE(why.find("未经连接事件确认"), std::string::npos) << why;
        EXPECT_EQ(why.find("已确认"), std::string::npos)
            << "断连是**按错误码判定**的，本批没有任何连接事件确认：" << why;
    }
    {
        ScriptedRig rig;
        rig.cam50->failStatusQueue.push_back(data::OpStatus::SdkError);
        data::MultiCameraFrame frame;
        rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL);

        const std::string why =
            rig.mgr->lastCaptureRound().channel(data::CameraRole::CAM50).skippedReason;
        EXPECT_NE(why.find("分类未明确"), std::string::npos) << why;
        EXPECT_EQ(why.find("断连"), std::string::npos)
            << "`SdkError` 的语义是**归不了类**，不得按号段推断成物理断连："
            << why;
    }
}

TEST(MultiCameraManagerTest, UnsetResultIsReportedAsCodeDefectNotDeviceFault)
{
    // `Unset` 是"漏赋值"（后端默认构造了 `GrabResult` 就返回），
    // 属**代码缺陷**。它既不能被当成"未尝试"，也不能与"一次瞬态失败"
    // 共用一句话 —— 两者的现场动作完全不同（改代码 vs 重采）。
    ScriptedRig rig;
    rig.cam50->returnUnsetResult = true;

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = r.channel(data::CameraRole::CAM50);
    EXPECT_EQ(rec.result.status, data::OpStatus::Unset);
    EXPECT_EQ(r.aggregate, data::OpStatus::Unset)
        << "`Unset` 是最高严重度，必须压过另两路的 Ok";
    EXPECT_EQ(data::appErrorCodeOf(r.aggregate), 9004)
        << "`Unset` 必须落系统级兜底码，而不是任何一种设备故障码";
    EXPECT_TRUE(rec.attempted)
        << "`Unset` 发生在**已经尝试过**的一条路上，不得记成未尝试";
    EXPECT_NE(rec.skippedReason.find("代码缺陷"), std::string::npos)
        << rec.skippedReason;
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM50))
        << "设备本身没问题，不得因为一次漏赋值把它禁用";
}

TEST(MultiCameraManagerTest, OkGrabWithEmptyImageIsNotAValidFrame)
{
    // "取帧成功但交付的图为空"：**接口契约是 `status == Ok` ⇒ 该帧满足
    // 全部后置条件**，而空图不满足 ⇒ 交付这样一个 `Ok` 本身就是契约违背，
    // 与设备好坏无关（设备可能一切正常，是我们的判据与交付不一致）。
    //
    // ⚠ 011-A1 九项缺口 §8 之前，这里只 `continue`、`result.status` 保持
    //   `Ok`：于是 ① 聚合取严重度最大者 ⇒ 整轮 `aggregate == Ok`，
    //   "三路全正常"与"三路全给空图"在上层**无法区分**；
    //   ② 控制器只对 `status != Ok` 的通道拼失败说明 ⇒ 空图的原因
    //   进不了降级说明，现场只看到"少了两路"。
    ScriptedRig rig;
    rig.cam50->returnEmptyImage = true;

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = r.channel(data::CameraRole::CAM50);
    EXPECT_EQ(rec.result.status, data::OpStatus::ContractViolation)
        << "空图与 `Ok` 的后置条件矛盾 ⇒ 本地契约违背，不得留在 Ok";
    EXPECT_EQ(data::appErrorCodeOf(rec.result.status), 1006)
        << "本地契约错误取 1006，不得冒充 1005（SDK 错误与帧不可用）";
    EXPECT_TRUE(rec.attempted) << "这一路确实取过帧，不得记成未尝试";
    EXPECT_EQ(rec.timestampNs, 0u) << "没有画面就没有画面时刻";
    EXPECT_EQ(r.capturedCount, 2) << "空图不得计入有效帧";
    EXPECT_EQ(r.aggregate, data::OpStatus::ContractViolation)
        << "聚合必须如实反映这一路，而不是被 Ok 淹没";

    // ⚠ **调用诊断逐字段保持后端给出的事实**：脚本后端这一次没有调用过
    //   任何 SDK ⇒ `sdkError` 保持 `nullopt`。管理器**不得**为了"让文本
    //   好看"补造一个 `{ImvGetFrame, 0}` —— 那会把"没调用"记成"调用过"，
    //   是离线排查的头号误导。
    EXPECT_FALSE(rec.result.sdkError.has_value())
        << "不得补造 {ImvGetFrame, 0}：这个后端根本没调用过 SDK";
    EXPECT_FALSE(rec.result.cleanupError.has_value());
    EXPECT_FALSE(rec.frameStatusRaw.has_value())
        << "`nullopt` = 本次未取得帧状态，与「取到了、值为 0」必须可区分";
    EXPECT_TRUE(rec.diagnosis.empty()) << "后端没给诊断，管理器不得自己编一条";

    EXPECT_NE(rec.skippedReason.find("图为空"), std::string::npos)
        << "必须留下可查的原因：" << rec.skippedReason;
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM50))
        << "空图不是设备故障 ⇒ 不得因此禁用通道";
}

TEST(MultiCameraManagerTest, PartialFailureKeepsTheFailingChannelEvidence)
{
    // "两路成功、一条超时"：成功返回，但**那条超时的证据必须还在**。
    // 这正是 R09 失效形态的反面 —— 旧实现会把整轮的失败原因收敛成
    // 一句"可用相机数不足"，出问题的是哪一路、为什么，全都丢失。
    ScriptedRig rig;
    rig.cam100->failStatusQueue.push_back(data::OpStatus::Timeout);

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    EXPECT_EQ(r.capturedCount, 2);
    EXPECT_EQ(r.aggregate, data::OpStatus::Timeout);
    EXPECT_EQ(data::appErrorCodeOf(r.aggregate), 1003);

    for (data::CameraRole role : {data::CameraRole::CAM25, data::CameraRole::CAM50})
    {
        const data::ChannelGrabRecord& ok = r.channel(role);
        EXPECT_EQ(ok.result.status, data::OpStatus::Ok);
        EXPECT_TRUE(ok.attempted);
        EXPECT_NE(ok.timestampNs, 0u) << "成功的那两路必须留下画面时刻";
        EXPECT_TRUE(ok.skippedReason.empty());
    }

    const data::ChannelGrabRecord& bad = r.channel(data::CameraRole::CAM100);
    EXPECT_EQ(bad.result.status, data::OpStatus::Timeout);
    EXPECT_TRUE(bad.attempted) << "它被尝试过，只是超时了";
    EXPECT_NE(bad.skippedReason.find("瞬态"), std::string::npos)
        << "超时是瞬态，措辞不得指向永久故障：" << bad.skippedReason;
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM100))
        << "一次超时把通道永久判死正是 R09 要消除的形态";
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §4.3 期限与预算（注入假钟）
// ===========================================================================

TEST(MultiCameraManagerTest, PerGrabCapIsUsedWhenItIsTheTightest)
{
    // 三个量取最小的第一种情形：单次上限最紧。
    // ⚠ 用**常量**假钟：本组用例断言的是 `grab()` 收到的实参数值，
    //   而时钟一旦在调用之间走动，数值就会同时受"读了几次钟"影响 ——
    //   那样断言到的就不是"取最小"这条规则，而是实现的读取次数。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_EQ(rig.cam25->lastTimeoutMs, 100u);
    EXPECT_EQ(rig.cam50->lastTimeoutMs, 100u);
    EXPECT_EQ(rig.cam100->lastTimeoutMs, 100u);
}

TEST(MultiCameraManagerTest, GroupBudgetIsUsedWhenItIsTheTightest)
{
    // 第二种情形：组预算（三路合计）最紧 —— 单次上限 500 ms 远大于它。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/500, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_EQ(rig.cam25->lastTimeoutMs, 300u)
        << "组预算 300 ms 才是三个量里最小的那个";
    EXPECT_EQ(rig.cam50->lastTimeoutMs, 300u);
    EXPECT_EQ(rig.cam100->lastTimeoutMs, 300u);
}

TEST(MultiCameraManagerTest, DeadlineIsUsedWhenItIsTheTightest)
{
    // 第三种情形：距**期限**的剩余最紧（40 ms < 单次上限 500 ms < 组预算 3 s）。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/500, /*groupBudgetNs=*/3000000000ULL);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 40000000ULL));

    EXPECT_EQ(rig.cam25->lastTimeoutMs, 40u);
    EXPECT_EQ(rig.cam50->lastTimeoutMs, 40u);
    EXPECT_EQ(rig.cam100->lastTimeoutMs, 40u);
}

TEST(MultiCameraManagerTest, ExhaustedBudgetSendsNeitherTriggerNorGrab)
{
    // 预算耗尽的那一路：**既不发令也不取帧**（`attempted == false`），
    // 且记账为一条**本地** `Timeout`（未调用 SDK ⇒ `sdkError` 为 `nullopt`）。
    //
    // ⚠ 这里必须让钟**走动**：所有等待上限都是从"现在"算出来的，
    //   常量钟下三路的剩余预算完全相同，永远走不到"耗尽"。
    //   真机上让它耗尽的是三路各自真实的等待耗时，测试里用钟的推进代替。
    //
    // ⚠ 推进的**触发条件必须是"已完成的取帧"**，而不是"读了一次钟"
    //   （011-A1 九项缺口 §3 改动后）：新实现为了在发令前、取帧前重算
    //   剩余预算而**多读几次钟**，若按"每读一次推进 120 ms"来驱动，
    //   走到哪一路耗尽就取决于实现的读取次数 —— 那是在拿读取次数当
    //   被测行为，实现一改（哪怕改得完全正确）用例就红。
    //   改为"时间 = 基准 + 每次**已完成**取帧 250 ms"后，时刻是取帧
    //   次数的纯函数，与读几次钟无关。
    //   算术：组预算 300 ms／单次上限 3000 ms ——
    //     CAM25 入场 0 ms 已耗时 ⇒ 取到帧（1 次）
    //     CAM50 入场 250 ms ⇒ 剩 50 ms ⇒ 仍能取（2 次）
    //     CAM100 入场 500 ms > 300 ms ⇒ **入场即耗尽**
    ScriptedRig rig;
    rig.mgr->setClock([&rig] {
        const int grabs = rig.cam25->grabCalls + rig.cam50->grabCalls +
                          rig.cam100->grabCalls;
        return kFakeEpochNs + 250000000ULL * static_cast<uint64_t>(grabs);
    });
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/500, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 3000000000ULL));

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& last = r.channel(data::CameraRole::CAM100);
    EXPECT_FALSE(last.attempted) << "预算耗尽时不得尝试";
    EXPECT_EQ(last.result.status, data::OpStatus::Timeout)
        << "未尝试也必须显式记一条本地超时，不得留 `Unset`";
    EXPECT_FALSE(last.result.sdkError.has_value())
        << "没调用 SDK，不得伪造失败归属";
    EXPECT_NE(last.skippedReason.find("预算耗尽"), std::string::npos)
        << last.skippedReason;
    EXPECT_EQ(rig.cam100->grabCalls, 0);
    EXPECT_EQ(rig.cam100->triggerCalls, 0) << "预算不足时连触发令都不发";

    // ⚠ `attempted == false` 的本地超时**仍要进聚合**：只挑"尝试过且失败"
    //   会把它整条漏掉，于是"这一轮其实只采到两路"这件事在上层不可见。
    EXPECT_EQ(r.aggregate, data::OpStatus::Timeout);
    EXPECT_EQ(r.capturedCount, 2);
    EXPECT_GT(r.finishedNs, r.startedNs)
        << "收尾时刻必须**重新读钟**，而不是复用入口时刻";
}

TEST(MultiCameraManagerTest, ExpiredDeadlineMeansNoSdkCallAtAll)
{
    // 期限**已经过期**：一次 SDK 调用都不该发生 —— 包括软件触发令。
    // ⚠ 判在发令之前：否则会"发出触发脉冲却收不回帧"，
    //   把一次预算不足变成设备侧的悬空触发（下一拍的触发时序随之错乱）。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }

    data::MultiCameraFrame frame;
    EXPECT_FALSE(rig.mgr->capture(frame, kFakeEpochNs))
        << "期限已到、三路都没取到 ⇒ 本轮整体失败";

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    EXPECT_EQ(r.capturedCount, 0);
    EXPECT_FALSE(r.succeeded);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        EXPECT_EQ(b->grabCalls, 0);
        EXPECT_EQ(b->triggerCalls, 0);
        EXPECT_TRUE(b->callOrder.empty());
    }
    for (const data::ChannelGrabRecord& rec : r.channels)
    {
        EXPECT_FALSE(rec.attempted);
        EXPECT_EQ(rec.result.status, data::OpStatus::Timeout);
        EXPECT_FALSE(rec.result.sdkError.has_value());
        EXPECT_NE(rec.skippedReason.find("预算耗尽"), std::string::npos);
    }
    EXPECT_EQ(r.aggregate, data::OpStatus::Timeout);
    // 三路都没采到 ⇒ 就绪性不足，报 1001（**不是**某一路的分类码）。
    EXPECT_EQ(rig.mgr->lastError().code, data::kErrCameraInsufficient);
}

TEST(MultiCameraManagerTest, ManagerDoesNotCacheThePreviousRoundsDeadline)
{
    // §3.2：期限**只从形参来**，管理器不得缓存上一轮的截止时刻。
    //
    // 判据必须这样构造：第一轮给一个**已过期**的期限（三路全被跳过），
    // 第二轮给一个**宽裕**的期限。若实现把第一轮的期限存进了成员，
    // 第二轮会被它继续判为过期 ⇒ 三路又全跳过 ⇒ 本用例转红。
    // 反过来说，只有"第一轮紧、第二轮松"这一个方向能把这个缺陷逼出来。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame first;
    EXPECT_FALSE(rig.mgr->capture(first, kFakeEpochNs));   // 已过期
    ASSERT_EQ(rig.cam25->grabCalls, 0);

    data::MultiCameraFrame second;
    EXPECT_TRUE(rig.mgr->capture(second, kFakeEpochNs + 300000000ULL))
        << "第二轮的期限更晚，必须真的去取帧";
    EXPECT_EQ(rig.cam25->grabCalls, 1);
    EXPECT_EQ(rig.cam50->grabCalls, 1);
    EXPECT_EQ(rig.cam100->grabCalls, 1);
    EXPECT_EQ(rig.cam25->lastTimeoutMs, 100u);
    EXPECT_EQ(rig.mgr->lastCaptureRound().capturedCount, 3);
}

TEST(MultiCameraManagerTest, RoundRecordShowsTheOverrunWhenTheLastCallReturnedPastTheDeadline)
{
    // §3.2：**每次调用返回后都要复查期限**（不只在下一次循环开始时）。
    // 末次取帧之后越过期限，必须能在本轮记录里读出来 ——
    // 否则"实际耗时已突破期限"这件事只剩日志里一条看不出异常的时间差。
    uint64_t fakeNowNs = kFakeEpochNs;
    ScriptedRig rig;
    rig.mgr->setClock([&fakeNowNs] {
        fakeNowNs += 120000000ULL;   // 每次读钟推进 120 ms
        return fakeNowNs;
    });
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/3000000000ULL);

    const uint64_t deadline = kFakeEpochNs + 400000000ULL;   // 400 ms 之后
    data::MultiCameraFrame frame;
    rig.mgr->capture(frame, deadline);

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    EXPECT_GT(r.finishedNs, r.startedNs);
    EXPECT_GT(r.finishedNs, deadline)
        << "收尾时刻已经越过期限，本轮记录必须如实反映（读法：finishedNs > deadlineNs）";
    EXPECT_EQ(r.aggregate, data::OpStatus::Timeout)
        << "越期后剩下的那一路只能记本地超时";
}

TEST(MultiCameraManagerTest, ZeroPerGrabTimeoutIsALocalParameterError)
{
    // `timeoutMs` 不接受 0（§2.2；SDK 对 0 的语义未文档化，项目也不定义它）。
    // 配置成 0 时**不发令、不取帧**，且**不取整成 0** 去调一个已禁止 0 的接口。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/0, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame frame;
    EXPECT_FALSE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    const data::CaptureRound r = rig.mgr->lastCaptureRound();
    for (const data::ChannelGrabRecord& rec : r.channels)
    {
        EXPECT_FALSE(rec.attempted);
        EXPECT_EQ(rec.result.status, data::OpStatus::InvalidArgument)
            << "这是本地参数错误，不是超时、也不是设备故障";
        EXPECT_FALSE(rec.result.sdkError.has_value());
        EXPECT_NE(rec.skippedReason.find("不接受 0"), std::string::npos)
            << rec.skippedReason;
    }
    EXPECT_EQ(r.aggregate, data::OpStatus::InvalidArgument);
    EXPECT_EQ(data::appErrorCodeOf(r.aggregate), 1006)
        << "本地参数错误取 1006（本地契约与参数错误），不得取 1005";
    EXPECT_EQ(rig.cam25->grabCalls, 0);
    EXPECT_EQ(rig.cam50->grabCalls, 0);
    EXPECT_EQ(rig.cam100->grabCalls, 0);
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §4.4 软件触发（单一执行者）
// ===========================================================================

TEST(MultiCameraManagerTest, SoftwareTriggerIsSentExactlyOncePerChannelBeforeItsGrab)
{
    // §2.4：软件触发令的**唯一执行者**是 `MultiCameraManager::capture()`，
    // 且每路在取帧**之前**恰好发令一次。后端自身绝不发令。
    //
    // ⚠ 断言的是**调用顺序**而不是"次数相等"：发令失败时这条路的取帧
    //   次数为 0（下一条用例），两者本来就不该相等 —— 若把它写成
    //   "次数相等"，那条正确行为反而会把本用例打红。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        EXPECT_EQ(b->triggerCalls, 1) << "一次 capture() 内恰好发令一次";
        EXPECT_EQ(b->grabCalls, 1);
        const std::vector<std::string> expect{"trigger", "grab"};
        EXPECT_EQ(b->callOrder, expect) << "必须先发令、后取帧";
    }
}

TEST(MultiCameraManagerTest, FreeRunAndHardwareSendNoTriggerCommand)
{
    // 只有 `Software` 模式才发令：`FreeRun` 由相机自己出图，
    // `Hardware` 由外部脉冲触发，两者都不该收到软件触发命令。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    rig.cam25->setTriggerMode(data::CameraTriggerMode::Software);
    rig.cam50->setTriggerMode(data::CameraTriggerMode::Hardware);
    rig.cam100->setTriggerMode(data::CameraTriggerMode::FreeRun);

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    EXPECT_EQ(rig.cam25->triggerCalls, 1);
    EXPECT_EQ(rig.cam50->triggerCalls, 0);
    EXPECT_EQ(rig.cam100->triggerCalls, 0);
    // 三路都照常取帧："不发令"不等于"不去取"。
    EXPECT_EQ(rig.cam25->grabCalls, 1);
    EXPECT_EQ(rig.cam50->grabCalls, 1);
    EXPECT_EQ(rig.cam100->grabCalls, 1);
}

TEST(MultiCameraManagerTest, TriggerFailureSkipsThatChannelsGrabAndNamesTheCall)
{
    // 发令失败 ⇒ **不取帧**：既然令没发出去，就不该去等一个不会到来的帧。
    // 否则会把"发令失败"伪装成一次超时，而超时是可以重采的、发令失败不是。
    // 归属必须**带操作名**：同样是 −119，`ExecuteCommandFeature` 超时与
    // `GetFrame` 超时是完全不同的故障。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    rig.cam50->triggerResult = data::OperationResult{
        data::OpStatus::SdkError,
        data::SdkFailure{data::SdkCall::ImvExecuteCommandFeature, -118},
        std::nullopt};

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL))
        << "另两路仍然成功 ⇒ 本轮整体成功";

    EXPECT_EQ(rig.cam50->grabCalls, 0) << "发令失败不得去等帧";
    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM50);
    EXPECT_TRUE(rec.attempted) << "它被尝试过：令确实发了（只是失败了）";
    EXPECT_EQ(rec.result.status, data::OpStatus::SdkError);
    ASSERT_TRUE(rec.result.sdkError.has_value());
    EXPECT_EQ(rec.result.sdkError->call, data::SdkCall::ImvExecuteCommandFeature)
        << "归属必须指向发令，而不是取帧";
    EXPECT_EQ(rec.result.sdkError->code, -118);
    EXPECT_NE(rec.skippedReason.find("未取帧"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rig.cam25->grabCalls, 1);
    EXPECT_EQ(rig.cam100->grabCalls, 1);
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §7 触发前置判定（取代三元回退）
// ===========================================================================

TEST(MultiCameraManagerTest, UnknownTriggerReadbackIsNotStartedAndSendsNoCommand)
{
    // §7 情形①：**读回不完整**（三项合成不出可判别的模式）⇒ `NotStarted`／1004，
    // **不发令、不取帧**。
    //
    // ⚠ 措辞纪律（本用例的一半价值在这里）：**不得**写成"设备已停止取流"。
    //    我们只是**没读到**触发配置，设备可能一切正常；把"读不到"说成"已停流"
    //    会让现场去查一个不存在的问题，同时把责任从读回调用推给设备。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    rig.cam50->reportUnknownTriggerMode = true;

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL))
        << "另两路仍成功 ⇒ 本轮整体成功（读回失败不改变整轮成败判据）";

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM50);
    EXPECT_EQ(rec.result.status, data::OpStatus::NotStarted)
        << "前置条件未确认，不是超时、也不是设备故障";
    EXPECT_EQ(data::appErrorCodeOf(rec.result.status), 1004);
    EXPECT_FALSE(rec.result.sdkError.has_value())
        << "本路既没发令也没取帧 ⇒ 不得伪造任何 SDK 调用记录";
    EXPECT_NE(rec.skippedReason.find("无法确认触发前置条件"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rec.skippedReason.find("已停止取流"), std::string::npos)
        << "「读不到」不得被写成「设备已停止取流」：" << rec.skippedReason;

    EXPECT_EQ(rig.cam50->triggerCalls, 0) << "不得发令";
    EXPECT_EQ(rig.cam50->grabCalls, 0) << "不得取帧";
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM50))
        << "读回失败**不永久禁用通道**";

    // 下一轮重新读回：本桩恢复正常 ⇒ CAM50 立刻照常工作（"瞬时失败"的实证）。
    rig.cam50->reportUnknownTriggerMode = false;
    data::MultiCameraFrame frame2;
    EXPECT_TRUE(rig.mgr->capture(frame2, kFakeEpochNs + 300000000ULL));
    EXPECT_EQ(rig.cam50->triggerCalls, 1) << "下一轮允许重新读回并执行";
    EXPECT_EQ(rig.cam50->grabCalls, 1);
}

TEST(MultiCameraManagerTest, TriggerReadbackMismatchIsContractViolationAndSendsNoCommand)
{
    // §7 情形②：读回**完整但与请求不一致** ⇒ `ContractViolation`／1006，
    // 请求值与实际值**都**要留下，**不发令、不取帧**。
    //
    // 为什么是契约违背而不是设备故障：设备如实回报了它现在的模式，
    // 是我们的"设置已生效"这一前提不成立 —— 按未生效的模式取帧会得到
    // 与预期不同的曝光时序，那是错数据，不是坏设备。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    // 请求 Software，读回 Hardware（"设了却没生效"）。
    rig.cam50->reportOtherTriggerMode = data::CameraTriggerMode::Hardware;

    data::MultiCameraFrame frame;
    EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL));

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM50);
    EXPECT_EQ(rec.result.status, data::OpStatus::ContractViolation);
    EXPECT_EQ(data::appErrorCodeOf(rec.result.status), 1006);
    EXPECT_FALSE(rec.result.sdkError.has_value());
    EXPECT_NE(rec.skippedReason.find("Software"), std::string::npos)
        << "请求值必须可见：" << rec.skippedReason;
    EXPECT_NE(rec.skippedReason.find("Hardware"), std::string::npos)
        << "实际读回值必须可见：" << rec.skippedReason;

    EXPECT_EQ(rig.cam50->triggerCalls, 0) << "不得按未生效的模式发令";
    EXPECT_EQ(rig.cam50->grabCalls, 0);
    EXPECT_TRUE(rig.mgr->channelAvailable(data::CameraRole::CAM50))
        << "不一致不永久禁用通道（可能是本拍设置刚下发、读回尚未收敛）";

    // 下一轮读回一致 ⇒ 照常执行（"不永久禁用"的实证）。
    rig.cam50->reportOtherTriggerMode.reset();
    data::MultiCameraFrame frame2;
    EXPECT_TRUE(rig.mgr->capture(frame2, kFakeEpochNs + 300000000ULL));
    EXPECT_EQ(rig.cam50->triggerCalls, 1);
    EXPECT_EQ(rig.cam50->grabCalls, 1);
}

TEST(MultiCameraManagerTest, PerChannelReadbackEvidenceReachesTheRoundRecord)
{
    // §7／§9：读回现场（特性名／是否调用过／失败信息）必须**进本轮记录**，
    // 否则"为什么这一路本轮没动作"在上层只剩一句结论、没有取证。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    rig.cam50->reportUnknownTriggerMode = true;

    data::MultiCameraFrame frame;
    rig.mgr->capture(frame, kFakeEpochNs + 300000000ULL);

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM50);
    ASSERT_EQ(rec.readbacks.size(), data::kTriggerFeatureCount);
    for (std::size_t i = 0; i < data::kTriggerFeatureCount; ++i)
    {
        EXPECT_EQ(rec.readbacks[i].feature, data::triggerFeatureName(i));
        EXPECT_TRUE(rec.readbacks[i].callAttempted) << "本桩三项都调过";
    }
    EXPECT_TRUE(rec.readbacks[1].returnedEmpty);

    // ⚠ 正常的一路也照样带上**本次**读回（"一致"是**取得的事实**，
    //    不是"没有信息"）—— 它不会因此触发任何 WARN（见 sink 判据）。
    const data::ChannelGrabRecord& ok = roundSnap.channel(data::CameraRole::CAM25);
    EXPECT_TRUE(ok.readbacks[1].callAttempted);
    EXPECT_EQ(ok.readbacks[1].value, "On");
    EXPECT_FALSE(ok.readbacks[1].returnedEmpty);
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §3 三个预算检查点（各自重读时钟）
// ===========================================================================

TEST(MultiCameraManagerTest, BudgetExhaustedAfterReadbackBeforeTriggerNamesThatStage)
{
    // §3 表的第二行：**读回之后、发令之前**重读时钟 ⇒ 剩余 < 1 ms
    // ⇒ 发令 0 次、取帧 0 次，措辞必须是"**发令前**预算耗尽"。
    //
    // ⚠ 与检查点 1 的措辞**必须分开**：这里已经花掉了读回的时间
    //   （真实后端是三次阻塞的 SDK 读），"入口就没预算"与"读回把预算
    //   吃掉了"的现场动作不同（后者说明读回本身太慢，是要查的东西）。
    //
    // 构造：时钟**随 CAM25 的读回次数推进** 200 ms，而组预算只有 150 ms
    // ⇒ 入口（读回前）还有 150 ms，读回一结束就没了。
    ScriptedRig rig;
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/150000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    rig.mgr->setClock([&rig] {
        return kFakeEpochNs +
               200000000ULL * static_cast<uint64_t>(rig.cam25->triggerModeStateCalls);
    });

    data::MultiCameraFrame frame;
    EXPECT_FALSE(rig.mgr->capture(frame, kFakeEpochNs + 3000000000ULL))
        << "三路都取不到帧 ⇒ 本轮整体失败";

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM25);
    EXPECT_FALSE(rec.attempted);
    EXPECT_EQ(rec.result.status, data::OpStatus::Timeout);
    EXPECT_FALSE(rec.result.sdkError.has_value());
    EXPECT_NE(rec.skippedReason.find("发令前预算耗尽"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rig.cam25->triggerModeStateCalls, 1) << "读回确实执行过（这正是耗时来源）";
    EXPECT_EQ(rig.cam25->triggerCalls, 0);
    EXPECT_EQ(rig.cam25->grabCalls, 0);
    // ⚠ CAM50／CAM100 必然撞在**检查点 1**：钟已经越过组预算，
    //    它们连读回都不该做。这不是附带现象，正是"检查点在入口"的意义。
    EXPECT_EQ(rig.cam50->triggerModeStateCalls, 0) << "入口就没预算 ⇒ 连读回都不执行";
    EXPECT_EQ(rig.cam100->triggerModeStateCalls, 0);
    EXPECT_EQ(rig.cam50->grabCalls, 0);
    EXPECT_EQ(rig.cam100->grabCalls, 0);
}

TEST(MultiCameraManagerTest, BudgetExhaustedAfterTriggerBeforeGrabKeepsTheCommandFact)
{
    // §3 表的第三行：**发令返回之后**、取帧之前重读时钟 ⇒ 剩余 < 1 ms
    // ⇒ 发令 **1** 次、取帧 **0** 次，且**保留发令结果**、
    // 措辞必须是"**已发令、未取帧**（取帧前预算耗尽）"。
    //
    // ⚠ 这一种**不得**写成"既未发令也未取帧"：令**确实发出去了**，
    //   设备这一拍已经收到脉冲 —— 把它写成"没发令"会让现场以为
    //   触发时序没错，而下一拍的对齐问题正是从这里来的。
    ScriptedRig rig;
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/150000000ULL);
    for (const auto& b : {rig.cam25, rig.cam50, rig.cam100})
    {
        b->setTriggerMode(data::CameraTriggerMode::Software);
    }
    rig.mgr->setClock([&rig] {
        return kFakeEpochNs +
               200000000ULL * static_cast<uint64_t>(rig.cam25->triggerCalls);
    });

    data::MultiCameraFrame frame;
    EXPECT_FALSE(rig.mgr->capture(frame, kFakeEpochNs + 3000000000ULL));

    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM25);
    EXPECT_TRUE(rec.attempted) << "令确实发出去了 —— 这是本轮必须留下的事实";
    EXPECT_EQ(rig.cam25->triggerCalls, 1);
    EXPECT_EQ(rig.cam25->grabCalls, 0) << "预算已尽，不得再去等帧";
    EXPECT_EQ(rec.trigger.status, data::OpStatus::Ok)
        << "发令结果必须保留（它是本路这一拍对设备做过的唯一动作）";
    EXPECT_EQ(rec.result.status, data::OpStatus::Timeout);
    EXPECT_NE(rec.skippedReason.find("已发令、未取帧"), std::string::npos)
        << rec.skippedReason;
    EXPECT_EQ(rec.skippedReason.find("未发令"), std::string::npos)
        << "已发令的事实不得被写成「未发令」：" << rec.skippedReason;
}

TEST(MultiCameraManagerTest, SubMillisecondRemainingCountsAsExhaustedAndExactlyOneMsProceeds)
{
    // §3：判据是"剩余 **< 1 ms** 即视为耗尽"，而**恰好 1 ms 仍要去做**，
    // 且 `timeoutMs` 就是那个 1 —— 取整规则（向下取整到 ms）在边界上的
    // 表现必须固定：既不能把 1 ms 算成 0 去调一个已禁止 0 的接口，
    // 也不能把 0.5 ms 算成 1 ms 去等一个明知不成立的等待。
    {
        ScriptedRig rig;   // 常量假钟（§ ScriptedRig 说明）
        rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);

        data::MultiCameraFrame frame;
        // 距期限只剩 500 000 ns = 0.5 ms。
        EXPECT_FALSE(rig.mgr->capture(frame, kFakeEpochNs + 500000ULL));

        const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
        const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM25);
        EXPECT_FALSE(rec.attempted);
        EXPECT_EQ(rec.result.status, data::OpStatus::Timeout);
        EXPECT_EQ(rig.cam25->grabCalls, 0) << "0.5 ms 不足以等一帧，不得取整成 1 ms 硬调";
        EXPECT_EQ(rig.cam25->triggerCalls, 0);
    }
    {
        ScriptedRig rig;   // 常量假钟（§ ScriptedRig 说明）
        rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);

        data::MultiCameraFrame frame;
        // 距期限恰好 1 ms。
        EXPECT_TRUE(rig.mgr->capture(frame, kFakeEpochNs + 1000000ULL));

        EXPECT_EQ(rig.cam25->grabCalls, 1) << "恰好 1 ms 仍是一次可用的等待";
        EXPECT_EQ(rig.cam25->lastTimeoutMs, 1u) << "实参必须是 1，不是 0、也不是 100";
    }
}

// ===========================================================================
//  MultiCameraManager —— 011-A1 §9 诊断出口（端到端一条链）
// ===========================================================================

TEST(MultiCameraManagerTest, NormalRoundProducesNoSinkOutputAtAll)
{
    // §9：**全部正常的一轮不产生任何 sink 输出**。
    // ⚠ 这一条与上一条同等重要：逐路写 WARN 会让日志被正常帧淹没，
    //   而"异常才有行"正是这一行输出能被当成信号的前提。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    std::vector<std::string> sinkLines;
    rig.mgr->setDiagnosticSink([&sinkLines](const std::string& t) {
        sinkLines.push_back(t);
    });

    data::MultiCameraFrame frame;
    ASSERT_TRUE(rig.mgr->capture(frame, farDeadlineNs()));
    EXPECT_TRUE(sinkLines.empty())
        << "三路都采到、帧状态为 0、触发读回一致 —— 这是正常事实，不是异常";
}

TEST(MultiCameraManagerTest, SuccessfulRoundCarriesNoStaleDiagnosisFromPreviousRound)
{
    // §9：每轮记录只含**本次**取得的事实，成功轮**不携带上一轮的陈旧文本**。
    // ⚠ 这一条针对的是一种很容易写出来的实现：把失败文本存进成员、
    //   下一轮忘记清 —— 表现是"这一路成功了，却带着上一轮的故障说明"，
    //   而现场会去查一个已经自愈的问题。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);
    rig.cam25->failStatusQueue.push_back(data::OpStatus::Timeout);

    data::MultiCameraFrame first;
    ASSERT_TRUE(rig.mgr->capture(first, kFakeEpochNs + 300000000ULL));
    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& bad = roundSnap.channel(data::CameraRole::CAM25);
    EXPECT_EQ(bad.result.status, data::OpStatus::Timeout);
    EXPECT_FALSE(bad.skippedReason.empty());

    // 下一轮该路恢复正常。
    data::MultiCameraFrame second;
    ASSERT_TRUE(rig.mgr->capture(second, kFakeEpochNs + 300000000ULL));
    const data::CaptureRound roundSnap2 = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& good = roundSnap2.channel(data::CameraRole::CAM25);
    EXPECT_EQ(good.result.status, data::OpStatus::Ok);
    EXPECT_TRUE(good.skippedReason.empty()) << good.skippedReason;
    EXPECT_TRUE(good.diagnosis.empty()) << good.diagnosis;
    ASSERT_TRUE(good.frameStatusRaw.has_value())
        << "取到了帧 ⇒ 帧状态是**取得的 0**，不是「未取得」（nullopt）";
    EXPECT_EQ(*good.frameStatusRaw, 0u);
    EXPECT_FALSE(good.result.sdkError.has_value());
}

TEST(MultiCameraManagerTest, EntryBudgetShortfallClearsThePreviousRoundsReadbackEvidence)
{
    // §9 的表：**入口就预算不足**时，本轮**没有取得**任何读回／触发／帧状态
    // 信息 —— 记录里不得留着上一轮的读回现场（那会让"这一轮为什么没动"
    // 看起来像"读过了、都正常"）。
    ScriptedRig rig;   // 默认注入常量假钟（见 ScriptedRig 的说明）
    rig.mgr->setGrabBudget(/*perGrabTimeoutMs=*/100, /*groupBudgetNs=*/300000000ULL);

    data::MultiCameraFrame first;
    ASSERT_TRUE(rig.mgr->capture(first, kFakeEpochNs + 300000000ULL));
    const data::CaptureRound roundSnap = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& before = roundSnap.channel(data::CameraRole::CAM25);
    EXPECT_TRUE(before.readbacks[1].callAttempted) << "上一轮确实读过";

    // 第二轮：期限已过 ⇒ 入口就判耗尽。
    data::MultiCameraFrame second;
    EXPECT_FALSE(rig.mgr->capture(second, kFakeEpochNs));
    const data::CaptureRound roundSnap2 = rig.mgr->lastCaptureRound();
    const data::ChannelGrabRecord& after = roundSnap2.channel(data::CameraRole::CAM25);
    EXPECT_FALSE(after.attempted);
    EXPECT_FALSE(after.readbacks[1].callAttempted)
        << "本轮没读回 ⇒ 不得继承上一轮的读回现场";
    EXPECT_TRUE(after.readbacks[1].value.empty());
    EXPECT_FALSE(after.frameStatusRaw.has_value())
        << "本轮没取帧 ⇒ 帧状态是「未取得」";
    EXPECT_TRUE(after.diagnosis.empty());
}

// ---------------------------------------------------------------------------
//  VirtualTurntable
// ---------------------------------------------------------------------------

TEST(VirtualTurntableTest, MoveTransitionsToMovingThenStable)
{
    // SYS-08 的 ALIGN → STABILIZE 两状态切分假定 move() **不阻塞到到位**，
    // 到位与否由轮询 state() 判断。本用例固定该契约。
    uint64_t fakeNowNs = 0;

    VirtualTurntable turntable(makeTurntableConfig());
    turntable.setClock([&fakeNowNs] { return fakeNowNs; });

    ASSERT_TRUE(turntable.initialize());
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::IDLE);

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 10.0;  // deg
    cmd.elevationCommand = 5.0;   // deg

    ASSERT_TRUE(turntable.move(cmd));
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::MOVING);

    // 快进足够长的时间（5 s 远超 20 deg / 5 deg/s = 4 s）
    fakeNowNs += 5ULL * 1000000000ULL;

    const data::TurntableState s = turntable.state();
    EXPECT_EQ(s.motion, data::TurntableMotionState::STABLE);
    EXPECT_NEAR(s.azimuth, 10.0, 1e-9);
    EXPECT_NEAR(s.elevation, 5.0, 1e-9);
}

TEST(VirtualTurntableTest, OutOfTravelCommandIsRejectedAsCapabilityLimit)
{
    // SYS-08 §7.7：转台超行程属**能力边界**，不重试。
    // 检查必须发生在**命令下发前** —— 运动结束后才发现超限，
    // 意味着真机上转台已经撞到机械限位。
    VirtualTurntable turntable(makeTurntableConfig());
    ASSERT_TRUE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 45.0;  // 超出 [-30, 30] deg
    cmd.elevationCommand = 0.0;

    EXPECT_FALSE(turntable.move(cmd));
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::ERROR);
    EXPECT_EQ(turntable.lastError().code, data::kErrTurntableOverTravel);
}

TEST(VirtualTurntableTest, InvalidLimitsAreRejectedAtInitialize)
{
    // 上限不大于下限属配置错误。若不在此处拒绝，错误会推迟到第一次
    // move() 时以"超行程"形式出现，排查方向会被引向对准算法而非配置。
    data::TurntableConfig bad = makeTurntableConfig();
    bad.azimuthMax = bad.azimuthMin;

    VirtualTurntable turntable(bad);
    EXPECT_FALSE(turntable.initialize());
}

TEST(VirtualTurntableTest, StopDoesNotClearErrorState)
{
    // 故障需要显式处理。一次 stop() 不应把 2002 的记录抹掉 ——
    // 否则任务收尾时的 stop() 调用会让超行程的事实消失。
    VirtualTurntable turntable(makeTurntableConfig());
    ASSERT_TRUE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 999.0;
    cmd.elevationCommand = 0.0;
    ASSERT_FALSE(turntable.move(cmd));

    turntable.stop();
    EXPECT_EQ(turntable.state().motion, data::TurntableMotionState::ERROR);
}

// ---------------------------------------------------------------------------
//  触发
// ---------------------------------------------------------------------------

TEST(VirtualTriggerControllerTest, TriggerRequiresEnable)
{
    data::TriggerConfig cfg;
    cfg.source          = "virtual";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    VirtualTriggerController trigger(cfg);
    ASSERT_TRUE(trigger.initialize());

    EXPECT_FALSE(trigger.trigger());  // 未使能
    ASSERT_TRUE(trigger.enable());
    EXPECT_TRUE(trigger.trigger());
    EXPECT_TRUE(trigger.trigger());
    EXPECT_EQ(trigger.triggerCount(), 2u);
}

TEST(VirtualTriggerControllerTest, SimulatedFailureReportsDegradeCode)
{
    // SYS-08 §7.5：触发故障 → 降级为软触发并记录 code=3001。
    // 该路径必须可测，否则在只有虚拟设备的第一阶段永远不被执行，
    // 其正确性只能等现场接入真实硬触发时才知道。
    data::TriggerConfig cfg;
    cfg.source          = "virtual";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    VirtualTriggerController trigger(cfg);
    ASSERT_TRUE(trigger.initialize());
    ASSERT_TRUE(trigger.enable());

    trigger.simulateFailure(true);
    EXPECT_FALSE(trigger.trigger());
    EXPECT_EQ(trigger.lastError().code, data::kErrTriggerDegraded);
}

// ---------------------------------------------------------------------------
//  真实设备桩：必须如实报告不可用
// ---------------------------------------------------------------------------

// ===========================================================================
//  SDK 调用替身（`IImvApi`）—— 011-A1 §4.1/§4.2 的验证手段
// ===========================================================================
//
//  ⚠ 为什么本机没有相机却要写这一整段：本批的关键行为 —— 错误分类、
//    复制先于释放、清理失败进返回值、长度溢出不复制、触发顺序 ——
//    **全部落在真实后端的代码里**，而它们与"有没有相机"无关。
//    只有经这条 seam 才能在本机驱动它们；否则这些逻辑只在实机上
//    才第一次运行，而实机验收本轮受阻。
//
//  ⚠ 本段**不覆盖**"本次构建没有 SDK"这一分支（`api_ == nullptr`）：
//    本机 `third_party/imvsdk/` 实际存在，SDK 构建下 `makeRealImvApi()`
//    必定返回非空，故那是一条**在本构建里不可达**的路径。它的证据是
//    §6 的**无 SDK 构建**（`-DIMV_SDK_ROOT=/nonexistent`）与
//    §4.4 的"`backend: imv` + SDK 缺失 ⇒ 启动失败"，不是这里。
//    在这里用替身去断言一条本构建不可能出现的路径，等于把不可达状态
//    伪装成已验证。

/// 可编程的 `IImvApi` 替身：按调用名配置返回码、记录调用序列与次数。
class StubImvApi : public IImvApi
{
public:
    // ---- 设备列表（`enumDevices` 的返回） ----
    std::vector<ImvDeviceEntry> devices;

    /// `enumDevices` 自身的返回码（0 = 成功，即便列表为空）。
    int enumDevicesRet = 0;

    /// 按调用名覆盖返回码；未配置的调用返回 0（成功）。
    /// ⚠ 键是 `data::sdkCallName()` 的名称（"IMV_GetFrame" 等），
    ///   使"哪个调用失败了"在读用例时直接可见。
    std::map<std::string, int> retCode;

    /// `getDeviceInfo` 回报的身份（打开后复核用）。
    /// 留空 = 回报 `devices[matchedIndex]` 的身份（即"一致"）。
    std::string reportSerial;
    std::string reportModel;

    // ---- 帧 ----
    /// `getFrame` 交付的字节（**替身拥有**；释放时会被改写，见下）。
    std::vector<uint8_t> frameBytes;

    /// `getFrame` 交付的帧描述字段（`data`/`size` 由本类填）。
    ImvFrameView frameView;

    /// 释放时把交付缓冲改写为 0xEE。
    /// ⚠ 这是"复制必须先于释放"这条契约的**唯一**可观测证明：
    ///    后端若把交付帧挂在 SDK 缓冲上（而不是复制），释放之后
    ///    它读到的就是 0xEE —— 而"逐字节等于 SDK 交付缓冲"的 RAW
    ///    判据会当场变成假的。
    bool poisonOnRelease = true;

    /// 令 `getFrame` **成功返回之后**紧接着的下一次分配抛 `std::bad_alloc`
    /// （§5 的异常安全证据；注入器见文件顶部的说明）。
    ///
    /// ⚠ 在**替身内部**布防而不是在用例里数"第 N 次分配"：
    ///    这样一来"注入点落在 `GetFrame` 成功之后、释放之前"这件事
    ///    是**由构造保证**的，而不是靠数字碰巧对上 —— 后者的失败形态是
    ///    "注入点其实在 `GetFrame` 之前"，而那时用例会以一条含义不明的
    ///    断言失败收场（本用例的 `getFrameCalls == 1` 断言会当场揭穿它）。
    bool failNextAllocationAfterFrame = false;

    /// §5 异常证据（V2.5）：`IMV_ReleaseFrame` 被调用时**抛出**
    /// `std::out_of_range("THREW")`。用于"释放调用本身抛出"那一条 ——
    /// 那时在飞的原异常**必须**原样上抛，这次抛出只被如实记成
    /// "释放未获确认"。**类型必须与原异常可区分**（见 §5 的用例说明）。
    bool throwOnReleaseFrame = false;

    /// §5 异常证据（V2.5）：在"复制载荷"那次失败**之后**，让紧接着的
    /// `secondaryFailureCount` 次分配抛 `std::length_error`（种类可区分）——
    /// 即异常出口的"描述原异常"（第 ① 步）与"写诊断"（第 ③ 步）各自失败。
    /// `0` = 关闭（只有原异常那一次注入）；用完自复位。
    /// · `1`：只有描述失败 ⇒ 诊断仍应写出"描述失败"的占位文本；
    /// · `2`：描述与写诊断都失败 ⇒ 诊断可以是空的，
    ///        **只要求原异常仍是原异常**。
    /// ⚠ 第二段必须**在这次注入里一起布防**（见 `getFrame`）：第 ① 步紧跟
    ///   在抛出之后、早于任何替身回调，等到"释放"时才布防就已经晚了。
    int secondaryFailureCount = 0;

    /// `getFrame` 交付的 `data` 指针改由本字段指定（默认指向 `frameBytes`）。
    /// 用于越界／溢出用例：把它指向一块**不可读**的地址，
    /// 于是"先复制再校验"的实现会当场崩溃，而正确的实现不碰它。
    const unsigned char* dataOverride = nullptr;

    // ---- 调用记录 ----
    std::vector<std::string> callLog;      ///< 依序的调用名
    int getFrameCalls       = 0;
    int releaseFrameCalls   = 0;
    int openCalls           = 0;
    int closeCalls          = 0;
    int destroyHandleCalls  = 0;
    int executeCommandCalls = 0;
    int startGrabbingCalls  = 0;
    int stopGrabbingCalls   = 0;
    int enumDevicesCalls    = 0;
    int createHandleCalls   = 0;

    /// 最近一次 `createHandle` 收到的下标（断言"按匹配到的下标建句柄"）。
    unsigned int lastHandleIndex = 0xFFFFFFFFu;

    /// 最近一次 `grab` 收到的 `timeoutMs`（断言"三个量取最小"）。
    unsigned int lastGrabTimeoutMs = 0;

    /// 枚举特性的当前值（`set` 写、`get` 读 —— 因此默认"读回一致"）。
    std::map<std::string, std::string> enumValues;

    /// 读回覆盖：`getEnumFeatureSymbol` 命中该特性名时**返回**此值。
    /// 用来模拟"**设了却没生效**"（设备侧实际值与写入值不同）——
    /// 这是读回校验存在的唯一理由，也是本机唯一能驱动它的方式。
    std::map<std::string, std::string> enumReadbackOverride;

    // ---- IImvApi ----

    int enumDevices(std::vector<ImvDeviceEntry>& out) override
    {
        ++enumDevicesCalls;
        callLog.push_back("IMV_EnumDevices");
        const int ret = retCode.count("IMV_EnumDevices")
                            ? retCode.at("IMV_EnumDevices")
                            : enumDevicesRet;
        if (ret != 0)
        {
            return ret;
        }
        out = devices;
        return 0;
    }

    int createHandle(void*& handle, ImvHandleMode mode, void* identifier) override
    {
        ++createHandleCalls;
        callLog.push_back("IMV_CreateHandle");
        EXPECT_EQ(mode, ImvHandleMode::ByIndex);
        if (identifier != nullptr)
        {
            lastHandleIndex = *static_cast<unsigned int*>(identifier);
        }
        const int ret = codeOf("IMV_CreateHandle");
        if (ret != 0)
        {
            return ret;
        }
        static int dummy = 0;   // 句柄内容无关紧要，非空即可
        handle           = &dummy;
        return 0;
    }

    int open(void* handle) override
    {
        ++openCalls;
        callLog.push_back("IMV_Open");
        (void)handle;
        return codeOf("IMV_Open");
    }

    int close(void* handle) override
    {
        ++closeCalls;
        callLog.push_back("IMV_Close");
        (void)handle;
        return codeOf("IMV_Close");
    }

    int destroyHandle(void* handle) override
    {
        ++destroyHandleCalls;
        callLog.push_back("IMV_DestroyHandle");
        (void)handle;
        return codeOf("IMV_DestroyHandle");
    }

    int getDeviceInfo(void* handle, ImvDeviceEntry& out) override
    {
        callLog.push_back("IMV_GetDeviceInfo");
        (void)handle;
        const int ret = codeOf("IMV_GetDeviceInfo");
        if (ret != 0)
        {
            return ret;
        }
        // 默认回报**被匹配到的那一台**（即"一致"）——不是"第 0 台"：
        // 真实 SDK 的 `IMV_GetDeviceInfo` 问的是这个句柄指向的设备，
        // 而句柄按下标建立。答复成第 0 台会让"按序列号绑定"这条
        // 契约的验证变成假绿。用例可覆盖成别的序列号。
        const std::size_t idx =
            (lastHandleIndex < devices.size()) ? lastHandleIndex : 0;
        out = devices.empty() ? ImvDeviceEntry{} : devices[idx];
        if (!reportSerial.empty())
        {
            out.serialNumber = reportSerial;
        }
        if (!reportModel.empty())
        {
            out.modelName = reportModel;
        }
        return 0;
    }

    int setEnumFeatureSymbol(void* handle, const char* feature,
                             const char* symbol) override
    {
        callLog.push_back(std::string("IMV_SetEnumFeatureSymbol:") + feature);
        (void)handle;
        const int ret = codeOf("IMV_SetEnumFeatureSymbol");
        if (ret != 0)
        {
            return ret;
        }
        enumValues[feature] = symbol;
        return 0;
    }

    int getEnumFeatureSymbol(void* handle, const char* feature,
                             std::string& out) override
    {
        callLog.push_back(std::string("IMV_GetEnumFeatureSymbol:") + feature);
        (void)handle;
        const int ret = codeOf("IMV_GetEnumFeatureSymbol");
        if (ret != 0)
        {
            return ret;
        }
        if (enumReadbackOverride.count(feature))
        {
            out = enumReadbackOverride.at(feature);
            return 0;
        }
        out = enumValues.count(feature) ? enumValues.at(feature) : std::string();
        return 0;
    }

    int setIntFeatureValue(void* handle, const char* feature, int64_t value) override
    {
        callLog.push_back(std::string("IMV_SetIntFeatureValue:") + feature);
        (void)handle;
        (void)value;
        return codeOf("IMV_SetIntFeatureValue");
    }

    int getIntFeatureValue(void* handle, const char* feature, int64_t& out) override
    {
        callLog.push_back(std::string("IMV_GetIntFeatureValue:") + feature);
        (void)handle;
        (void)out;
        return codeOf("IMV_GetIntFeatureValue");
    }

    int setDoubleFeatureValue(void* handle, const char* feature,
                              double value) override
    {
        callLog.push_back(std::string("IMV_SetDoubleFeatureValue:") + feature);
        (void)handle;
        const int ret = codeOf("IMV_SetDoubleFeatureValue");
        if (ret != 0)
        {
            return ret;
        }
        doubleValues[feature] = value;
        return 0;
    }

    int getDoubleFeatureValue(void* handle, const char* feature,
                              double& out) override
    {
        callLog.push_back(std::string("IMV_GetDoubleFeatureValue:") + feature);
        (void)handle;
        const int ret = codeOf("IMV_GetDoubleFeatureValue");
        if (ret != 0)
        {
            return ret;
        }
        out = doubleValues.count(feature) ? doubleValues.at(feature) : 0.0;
        return 0;
    }

    int executeCommandFeature(void* handle, const char* feature) override
    {
        ++executeCommandCalls;
        callLog.push_back(std::string("IMV_ExecuteCommandFeature:") + feature);
        (void)handle;
        return codeOf("IMV_ExecuteCommandFeature");
    }

    int startGrabbing(void* handle) override
    {
        ++startGrabbingCalls;
        callLog.push_back("IMV_StartGrabbing");
        (void)handle;
        return codeOf("IMV_StartGrabbing");
    }

    int stopGrabbing(void* handle) override
    {
        ++stopGrabbingCalls;
        callLog.push_back("IMV_StopGrabbing");
        (void)handle;
        return codeOf("IMV_StopGrabbing");
    }

    int getFrame(void* handle, ImvFrameView& out, unsigned int timeoutMs) override
    {
        ++getFrameCalls;
        lastGrabTimeoutMs = timeoutMs;
        callLog.push_back("IMV_GetFrame");
        (void)handle;

        // ⚠ 失败路径**仍然**填一份哨兵 view：SDK 未保证失败时输出字段有效，
        //    而用例要断言"实现没有拿这些字段去影响结果"。
        //    （这**不能**证明"实现一个字段都没读"—— 填了哨兵只证明
        //    "读了也不会影响结果"，见用例里的说明。）
        out          = frameView;
        out.data     = dataOverride != nullptr ? dataOverride : frameBytes.data();
        out.size     = static_cast<unsigned int>(frameBytes.size());

        const int ret = codeOf("IMV_GetFrame");
        if (ret != 0)
        {
            return ret;
        }
        if (failNextAllocationAfterFrame || secondaryFailureCount > 0)
        {
            // 帧已交出（调用成功）⇒ 从这里往后就是"复制载荷／建显示图"
            // 那一段 —— 让它的**第一次**分配失败，正是要在那里制造异常
            // （＝**原异常**）。`secondaryFailureCount` 是第二段：紧接着的
            // 那几次分配（＝异常出口里"描述异常"与"写诊断"两步各自的
            // 分配）改成抛 `length_error`，使**二次失败与原异常类型不同**。
            const int rest               = secondaryFailureCount;
            failNextAllocationAfterFrame = false;
            secondaryFailureCount        = 0;
            armAllocFailurePhased(1, 0, rest, 1);
        }
        return 0;
    }

    int releaseFrame(void* handle) override
    {
        ++releaseFrameCalls;
        callLog.push_back("IMV_ReleaseFrame");
        (void)handle;
        if (throwOnReleaseFrame)
        {
            // ⚠ 短字面量：抛异常自身若分配，会干扰下面的分配注入倒计时。
            throw std::out_of_range("THREW");
        }
        const int ret = codeOf("IMV_ReleaseFrame");
        if (ret == 0 && poisonOnRelease)
        {
            std::fill(frameBytes.begin(), frameBytes.end(), 0xEEu);
            if (dataOverride != nullptr)
            {
                // 不可读的替身缓冲改不了 —— 但那一类用例本来就不释放成功。
            }
        }
        return ret;
    }

private:
    std::map<std::string, double> doubleValues;

    int codeOf(const std::string& call) const
    {
        return retCode.count(call) ? retCode.at(call) : 0;
    }
};

/// 一台"在枚举结果里、序列号匹配"的设备。
ImvDeviceEntry makeDeviceEntry(const std::string& serial,
                               const std::string& model = "A7A20MU201")
{
    ImvDeviceEntry e;
    e.serialNumber = serial;
    e.modelName    = model;
    e.vendorName   = "Huaray";
    e.cameraKey    = "Huaray:" + serial;
    return e;
}

/// 一块**不可读**的内存（`PROT_NONE`），返回首地址；失败返回 `nullptr`。
///
/// ⚠ 用途只有一个：把 `ImvFrameView::data` 指向它，使"先按长度复制、
///    再检查合法性"的实现**当场崩溃**。这是本机能给出的**直接**证据 ——
///    替身的调用计数器只能数"调了几次"，证明不了后端"读没读"那块缓冲。
/// ⚠ 静态块只映射一次：每次调用都映射会在一堆用例里泄漏地址空间。
const unsigned char* guardPagePointer()
{
    static const unsigned char* page = []() -> const unsigned char* {
        void* m = ::mmap(nullptr, 4096, PROT_NONE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        return (m == MAP_FAILED) ? nullptr
                                 : static_cast<const unsigned char*>(m);
    }();
    return page;
}

/// 打开一个已成功 `initialize()`＋`start()` 的真实后端（无相机，走替身）。
struct OpenRealBackend
{
    std::shared_ptr<StubImvApi>       api;
    data::CameraConfig                config;
    std::unique_ptr<ImvCameraBackend> backend;

    /// @param frameW/frameH 写进配置的画幅。**默认为 0 ＝ 关闭画幅比对**：
    ///        本组用例关心的是取帧、分类、复制与释放，帧尺寸取最小的
    ///        2×2／4×4 即可读；若让配置里的 640×480 生效，
    ///        每一帧都会先撞上"画幅与配置不一致"而变成 `ContractViolation`，
    ///        整组用例会以与各自意图无关的原因失败。画幅比对本身
    ///        由 `FrameGeometryMismatchWithConfigIsContractViolation` 专门验证。
    /// @param mode 触发模式。默认 `Software`：**真实后端只实现 Software**
    ///        （`FreeRun`／`Hardware` 返回 `NotImplemented`，见
    ///        `applyTriggerMode`），故任何"能打开成功"的用例都必须用它。
    bool open(const std::string& serial = "SN-0001",
              data::CameraRole role      = data::CameraRole::CAM25,
              uint32_t frameW = 0, uint32_t frameH = 0,
              data::CameraTriggerMode mode = data::CameraTriggerMode::Software)
    {
        api = std::make_shared<StubImvApi>();
        api->devices.push_back(makeDeviceEntry(serial));

        config              = makeCameraConfig(role, "cam-real");
        config.serialNumber = serial;
        config.backend      = "imv";
        config.triggerMode  = mode;
        config.width        = static_cast<int>(frameW);
        config.height       = static_cast<int>(frameH);

        backend = std::make_unique<ImvCameraBackend>(config, api);
        return backend->initialize().ok() && backend->start().ok();
    }
};

// ---------------------------------------------------------------------------
//  打开与身份绑定（§2.3、§4.4）
// ---------------------------------------------------------------------------

TEST(ImvCameraBackendTest, UnboundSerialIsRejectedWithoutAnySdkCall)
{
    // 序列号为空 ⇒ 明确失败。绝不取"第 0 个设备"：三台同型号相机
    // 会因此随机互换通道，而互换之后**没有任何错误**，只表现为
    // 标定对不上、角度系统性偏差。
    auto api = std::make_shared<StubImvApi>();

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber.clear();
    cfg.backend = "imv";

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::InvalidArgument);
    // 本地判定 ⇒ **不伪造**"调用过 SDK"。
    EXPECT_FALSE(r.sdkError.has_value());
    // 而且一次 SDK 调用都没发生（连枚举都不做）。
    EXPECT_EQ(api->enumDevicesCalls, 0);
    EXPECT_TRUE(api->callLog.empty());
}

TEST(ImvCameraBackendTest, MissingSerialInEnumerationYieldsDisconnectedWithoutSdkError)
{
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-OTHER"));
    api->devices.push_back(makeDeviceEntry("SN-THIRD"));

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-WANTED";
    cfg.backend      = "imv";

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    // 这是**直接观察到的事实**（枚举结果里没有它），不是从错误码推断的断连。
    EXPECT_EQ(r.status, data::OpStatus::Disconnected);
    // ⚠ SDK 调用本身成功了 ⇒ 不得伪造 sdkError。
    EXPECT_FALSE(r.sdkError.has_value());
    EXPECT_EQ(backend.state(), data::DeviceState::DISCONNECTED);

    // 错误文本必须把**枚举到的**序列号列出来 —— 否则现场无法判断
    // "是相机没插上，还是配置写错了序列号"。
    const std::string text = backend.lastErrorText();
    EXPECT_NE(text.find("SN-OTHER"), std::string::npos);
    EXPECT_NE(text.find("SN-THIRD"), std::string::npos);
    // ⚠ 措辞纪律：本批不注册连接事件回调，**不得**写成"已确认断连"。
    EXPECT_EQ(text.find("已确认"), std::string::npos);

    // 未打开 ⇒ 取帧是本地判定，且**不调用 SDK**。
    data::ImageFrame frame;
    const data::GrabResult g = backend.grab(frame, 100);
    EXPECT_EQ(g.status, data::OpStatus::NotStarted);
    EXPECT_FALSE(g.sdkError.has_value());
    EXPECT_EQ(api->getFrameCalls, 0);
}

TEST(ImvCameraBackendTest, DuplicateSerialIsContractViolationAndIsNotGuessed)
{
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-DUP"));
    api->devices.push_back(makeDeviceEntry("SN-DUP"));

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-DUP";
    cfg.backend      = "imv";

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::ContractViolation);
    EXPECT_FALSE(r.sdkError.has_value());
    // 不猜哪一台是对的：连句柄都不建。
    EXPECT_EQ(api->createHandleCalls, 0);
}

TEST(ImvCameraBackendTest, CreatesHandleByMatchedIndexNotByFirstDevice)
{
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-A"));
    api->devices.push_back(makeDeviceEntry("SN-B"));
    api->devices.push_back(makeDeviceEntry("SN-C"));

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM50, "cam50");
    cfg.serialNumber = "SN-B";        // 第 1 个（0 基）
    cfg.backend      = "imv";
    cfg.triggerMode  = data::CameraTriggerMode::Software;

    ImvCameraBackend backend(cfg, api);
    ASSERT_TRUE(backend.initialize().ok());
    EXPECT_EQ(api->lastHandleIndex, 1u);

    // 身份只填**设备回报**，不从配置复制期望值。
    const data::DeviceIdentity id = backend.deviceIdentity();
    EXPECT_TRUE(id.queried);
    ASSERT_TRUE(id.serialNumber.has_value());
    EXPECT_EQ(*id.serialNumber, "SN-B");
    EXPECT_EQ(id.modelName, "A7A20MU201");
}

TEST(ImvCameraBackendTest, IdentityMismatchAfterOpenAbandonsHandle)
{
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-WANTED"));
    api->reportSerial = "SN-SOMETHING-ELSE";   // 枚举与打开之间设备变了

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-WANTED";
    cfg.backend      = "imv";

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::ContractViolation);
    // 半初始化不许留在句柄上：已打开 ⇒ 必须关掉并销毁。
    EXPECT_EQ(api->closeCalls, 1);
    EXPECT_EQ(api->destroyHandleCalls, 1);
    EXPECT_EQ(backend.state(), data::DeviceState::ERROR);
}

TEST(ImvCameraBackendTest, OpenFailureSelfCleansAndDoesNotLeaveHandleOpen)
{
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-0001"));
    api->retCode["IMV_Open"] = -103;   // IMV_INVALID_PARAM

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-0001";
    cfg.backend      = "imv";

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::InvalidArgument);
    ASSERT_TRUE(r.sdkError.has_value());
    // ⚠ 两个字段都断言：**操作名 + 返回码**。只断码的话，
    //    同为 −103 的"枚举失败"与"打开失败"会看起来一样。
    EXPECT_EQ(r.sdkError->call, data::SdkCall::ImvOpen);
    EXPECT_EQ(r.sdkError->code, -103);

    // 打开失败 ⇒ 自清理：句柄已建立（createHandle 成功）故必须销毁。
    EXPECT_EQ(api->destroyHandleCalls, 1);
    // 未成功打开 ⇒ 不该调 IMV_Close（句柄从未打开）。
    EXPECT_EQ(api->closeCalls, 0);

    // 再关一次**不产生第二次底层销毁**（幂等包装允许被多调，资源不许重复释放）。
    EXPECT_TRUE(backend.close().ok());
    EXPECT_EQ(api->destroyHandleCalls, 1);
}

// ---------------------------------------------------------------------------
//  配置与读回（§2.3）
// ---------------------------------------------------------------------------

TEST(ImvCameraBackendTest, TriggerIsConfiguredSelectorThenSourceThenModeAndReadBack)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // 触发配置的**次序**冻结为 Selector → Source → Mode（§2.3，依据是
    // 7 处 SDK 样例一致地先设 Selector）。跳过 Selector 会让触发设置
    // 落到别的选择器上，表现为"设了却不起作用"。
    const std::vector<std::string> expected = {
        "IMV_SetEnumFeatureSymbol:TriggerSelector",
        "IMV_GetEnumFeatureSymbol:TriggerSelector",
        "IMV_SetEnumFeatureSymbol:TriggerSource",
        "IMV_GetEnumFeatureSymbol:TriggerSource",
        "IMV_SetEnumFeatureSymbol:TriggerMode",
        "IMV_GetEnumFeatureSymbol:TriggerMode",
    };
    for (const std::string& name : expected)
    {
        EXPECT_NE(std::find(f.api->callLog.begin(), f.api->callLog.end(), name),
                  f.api->callLog.end())
            << "缺少调用：" << name;
    }

    const auto pos = [&](const std::string& name) {
        return std::find(f.api->callLog.begin(), f.api->callLog.end(), name);
    };
    EXPECT_LT(pos("IMV_SetEnumFeatureSymbol:TriggerSelector"),
              pos("IMV_SetEnumFeatureSymbol:TriggerSource"));
    EXPECT_LT(pos("IMV_SetEnumFeatureSymbol:TriggerSource"),
              pos("IMV_SetEnumFeatureSymbol:TriggerMode"));

    // 像素格式在触发之前就已设好（格式决定一帧的长度，必须最先确定）。
    EXPECT_LT(pos("IMV_SetEnumFeatureSymbol:PixelFormat"),
              pos("IMV_SetEnumFeatureSymbol:TriggerSelector"));
}

TEST(ImvCameraBackendTest, ReadbackMismatchIsContractViolationNotSilentAcceptance)
{
    // "设了却没生效"的唯一探测器：set 之后必须 get 并比对。
    // 只 set 不 get 会让一次静默失效一路走到测量结果里，
    // 而现场只会怀疑镜头或标定。
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-0001"));
    // 写入成功，但设备回报的是另一个格式（"设了没生效"）。
    api->enumReadbackOverride["PixelFormat"] = "Mono8";

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-0001";
    cfg.backend      = "imv";
    cfg.triggerMode  = data::CameraTriggerMode::Software;

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::ContractViolation);
    // SDK 调用都成功了 ⇒ 没有"失败的调用"可归。伪造一个会让排查
    // 去找一个不存在的 SDK 返回码。
    EXPECT_FALSE(r.sdkError.has_value());
    EXPECT_EQ(api->closeCalls, 1);   // 已打开 ⇒ 自清理
}

TEST(ImvCameraBackendTest, TriggerModeStateReportsAllThreeFieldsFromDevice)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    const data::TriggerModeState s = f.backend->triggerModeState();

    // ⚠ 只读 `TriggerMode == "On"` 分不出软件触发与硬件触发 ——
    //    两者都是 "On"，区别只在 `TriggerSource`。故三项都要读回。
    EXPECT_TRUE(s.selectorReported.has_value());
    EXPECT_TRUE(s.switchReported.has_value());
    EXPECT_TRUE(s.sourceReported.has_value());
    ASSERT_TRUE(s.reported.has_value());
    EXPECT_EQ(*s.reported, data::CameraTriggerMode::Software);
    EXPECT_TRUE(s.consistent);

    // ⚠ 三项必须来自**读回**，不是把请求值抄回去：
    //    读到的触发源是 "Software"（而非配置里的任何字符串），
    //    且这三项各自都在 `callLog` 里留下过 `IMV_GetEnumFeatureSymbol`。
    EXPECT_EQ(*s.sourceReported, "Software");
    EXPECT_EQ(*s.switchReported, "On");
}

TEST(ImvCameraBackendTest, UnreadableTriggerModeIsUnknownNotEmptySuccess)
{
    // 读不到 ⇒ "未知"，**不得**默认成某个合法模式：
    // 那会让一次读失败伪装成"模式正确"。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->retCode["IMV_GetEnumFeatureSymbol"] = -116;   // IMV_NOT_AVAILABLE

    const data::TriggerModeState s = f.backend->triggerModeState();
    EXPECT_FALSE(s.reported.has_value());
    EXPECT_FALSE(s.consistent);
    // ⚠ 读失败**不等于断连**：−116 的注释是"连接不可达"，
    //    不足以判 `Disconnected`（§2.2 表）。此处只要求它不被猜成合法模式。

    // §7：读失败必须**逐项**留下取证 —— 三项走同一个 SDK 调用，
    // 只有 {call, code} 时说不出失败的是哪一项。
    for (std::size_t i = 0; i < data::kTriggerFeatureCount; ++i)
    {
        EXPECT_EQ(s.readbacks[i].feature, data::triggerFeatureName(i));
        EXPECT_TRUE(s.readbacks[i].callAttempted);
        ASSERT_TRUE(s.readbacks[i].failure.has_value()) << "第 " << i << " 项应记为调用失败";
        EXPECT_EQ(s.readbacks[i].failure->call,
                  data::SdkCall::ImvGetEnumFeatureSymbol);
        EXPECT_EQ(s.readbacks[i].failure->code, -116);
        EXPECT_FALSE(s.readbacks[i].returnedEmpty);
    }
}

TEST(ImvCameraBackendTest, UnreadableReadbackDistinguishesEmptyReturnFromCallFailure)
{
    // §7 情形③：**"调用成功但返回空串"** 与 **"SDK 调用失败"** 必须在记录里
    // 可区分 —— 两者的排查方向相反（前者是特性存在/取值语义问题，
    // 后者是链路/SDK 问题），混成"值为空"会把一次调用失败当成设备没配该特性。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // (a) 调用成功、返回空串 ⇒ `returnedEmpty`，且**没有** `failure`。
    f.api->enumReadbackOverride["TriggerMode"] = "";
    const data::TriggerModeState s = f.backend->triggerModeState();
    EXPECT_FALSE(s.reported.has_value()) << "空串不是一种模式，不得被猜成合法值";
    EXPECT_EQ(s.readbacks[1].feature, "TriggerMode") << "取值必须对得上特性名";
    EXPECT_TRUE(s.readbacks[1].callAttempted);
    EXPECT_TRUE(s.readbacks[1].returnedEmpty) << "调用成功但没读到值";
    EXPECT_FALSE(s.readbacks[1].failure.has_value());
    EXPECT_TRUE(s.readbacks[1].value.empty());

    // (b) 同一项改为**调用失败** ⇒ `failure` 带操作名与原码，不再是 returnedEmpty。
    f.api->enumReadbackOverride.erase("TriggerMode");
    f.api->retCode["IMV_GetEnumFeatureSymbol"] = -103;   // IMV_INVALID_PARAM
    const data::TriggerModeState t = f.backend->triggerModeState();
    EXPECT_FALSE(t.reported.has_value());
    ASSERT_TRUE(t.readbacks[1].failure.has_value());
    EXPECT_EQ(t.readbacks[1].failure->call, data::SdkCall::ImvGetEnumFeatureSymbol);
    EXPECT_EQ(t.readbacks[1].failure->code, -103) << "原码必须保留";
    EXPECT_FALSE(t.readbacks[1].returnedEmpty)
        << "调用失败与返回空串不得混为一谈 —— 它们的现场动作相反";
}

TEST(ImvCameraBackendTest, FreeRunRequestIsNotImplementedOnRealBackend)
{
    // 自由运行本批**不实施**：SDK 全树没有 "FreeRun" 这个词，
    // 而"Off 即自由运行"只由样例代码支持、无文档保证。
    // 故真实后端明确拒绝，**不静默当自由运行**。
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-0001"));

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-0001";
    cfg.backend      = "imv";
    cfg.triggerMode  = data::CameraTriggerMode::FreeRun;

    ImvCameraBackend backend(cfg, api);
    const data::OperationResult r = backend.initialize();

    EXPECT_EQ(r.status, data::OpStatus::NotImplemented);
    EXPECT_FALSE(r.sdkError.has_value());
}

// ---------------------------------------------------------------------------
//  取帧、分类与释放（§2.2、§4.2）
// ---------------------------------------------------------------------------

TEST(ImvCameraBackendTest, TimeoutIsTimeoutOnlyWhenGetFrameReturnedIt)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->retCode["IMV_GetFrame"] = -119;   // IMV_TIMEOUT

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    EXPECT_EQ(g.status, data::OpStatus::Timeout);
    ASSERT_TRUE(g.sdkError.has_value());
    EXPECT_EQ(g.sdkError->call, data::SdkCall::ImvGetFrame);
    EXPECT_EQ(g.sdkError->code, -119);

    // 帧未被触碰（接口契约：status != Ok 时不得写入部分字段）。
    EXPECT_TRUE(frame.image.empty());
    EXPECT_EQ(frame.frameId, 0u);
    EXPECT_EQ(frame.role, data::CameraRole::CAM25);   // 默认值，未被改写

    // 取帧失败 ⇒ 没有帧可释放。
    EXPECT_EQ(f.api->releaseFrameCalls, 0);
}

TEST(ImvCameraBackendTest, AmbiguousSdkCodesStaySdkErrorAndKeepTheRawCode)
{
    // ⚠ 负面用例：这几条**曾经**容易被按错误码号段猜成"断连"或"未就绪"。
    //    §2.2 的判据是"该码自身的**文档化含义**"：
    //    −114/−115 的注释是"恢复中"、−116 是"不可达"、−122 是"调用时序错误"
    //    —— 都不足以支撑 `Disconnected`／`NotStarted`。
    const int codes[] = {-114, -115, -116, -122, -130, -101};
    for (const int code : codes)
    {
        OpenRealBackend f;
        ASSERT_TRUE(f.open());
        f.api->retCode["IMV_GetFrame"] = code;

        data::ImageFrame frame;
        const data::GrabResult g = f.backend->grab(frame, 100);

        EXPECT_EQ(g.status, data::OpStatus::SdkError) << "码 " << code;
        EXPECT_NE(g.status, data::OpStatus::Disconnected) << "码 " << code;
        EXPECT_NE(g.status, data::OpStatus::NotStarted) << "码 " << code;
        ASSERT_TRUE(g.sdkError.has_value()) << "码 " << code;
        EXPECT_EQ(g.sdkError->code, code);   // 原码必须保留
    }
}

TEST(ImvCameraBackendTest, NotSupportIsNotImplementedAndNotConnectedIsDisconnected)
{
    // 这两条有**明确**的常量注释支持，故必须分类（不是 SdkError）。
    {
        OpenRealBackend f;
        ASSERT_TRUE(f.open());
        f.api->retCode["IMV_GetFrame"] = -113;   // "设备不支持的功能"
        data::ImageFrame frame;
        const data::GrabResult g = f.backend->grab(frame, 100);
        EXPECT_EQ(g.status, data::OpStatus::NotImplemented);
        ASSERT_TRUE(g.sdkError.has_value());
        EXPECT_EQ(g.sdkError->code, -113);
    }
    {
        OpenRealBackend f;
        ASSERT_TRUE(f.open());
        f.api->retCode["IMV_GetFrame"] = -118;   // "设备未连接"
        data::ImageFrame frame;
        const data::GrabResult g = f.backend->grab(frame, 100);
        EXPECT_EQ(g.status, data::OpStatus::Disconnected);
        ASSERT_TRUE(g.sdkError.has_value());
        EXPECT_EQ(g.sdkError->call, data::SdkCall::ImvGetFrame);
        EXPECT_EQ(g.sdkError->code, -118);
    }
}

TEST(ImvCameraBackendTest, ZeroTimeoutIsRejectedLocallyWithoutCallingSdk)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 0);

    EXPECT_EQ(g.status, data::OpStatus::InvalidArgument);
    EXPECT_FALSE(g.sdkError.has_value());
    EXPECT_EQ(f.api->getFrameCalls, 0);   // 本地判定，没调 SDK
}

TEST(ImvCameraBackendTest, FrameIsCopiedBeforeReleaseSoReleaseCannotCorruptIt)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // Mono12：一帧 2x2，16 位容器低位对齐。
    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;   // gvspPixelMono12
    f.api->frameView.status      = 0;
    f.api->frameBytes            = {0x00, 0x08, 0xFF, 0x0F, 0x00, 0x00, 0x40, 0x00};
    f.api->poisonOnRelease       = true;   // 释放时把那块缓冲改成 0xEE

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);
    ASSERT_TRUE(g.ok()) << data::opStatusName(g.status);

    EXPECT_EQ(f.api->releaseFrameCalls, 1);
    // 释放**之后**才读这段字节：若实现把交付帧挂在 SDK 缓冲上，
    // 这里读到的就是 0xEE。
    ASSERT_TRUE(frame.raw.bytes != nullptr);
    const std::vector<uint8_t>& bytes = *frame.raw.bytes;
    ASSERT_EQ(bytes.size(), 8u);
    EXPECT_EQ(bytes[0], 0x00);
    EXPECT_EQ(bytes[1], 0x08);
    EXPECT_EQ(bytes[2], 0xFF);
    EXPECT_EQ(bytes[3], 0x0F);
    EXPECT_EQ(bytes[6], 0x40);
    EXPECT_EQ(bytes[7], 0x00);   // 释放后的 0xEE 一个都没渗进来
}

TEST(ImvCameraBackendTest, Mono12KnownBytesConvertByShiftWithoutScaling)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // 低位对齐、高位补零（PFNC 2.4 §6.1.1，见 §2.1）。
    // 逐字节小端组装：0x0800=2048 → >>4 = 128；0x0FFF=4095 → 255；
    // 0x0000=0 → 0；0x0400=1024 → 64。
    // ⚠ 这**不依赖相机**，是本批"由标准推实现"那一步的对照用例。
    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;
    f.api->frameBytes = {0x00, 0x08,   // 2048
                         0xFF, 0x0F,   // 4095
                         0x00, 0x00,   // 0
                         0x00, 0x04};  // 1024

    data::ImageFrame frame;
    ASSERT_TRUE(f.backend->grab(frame, 100).ok());

    ASSERT_FALSE(frame.image.empty());
    EXPECT_EQ(frame.image.type(), CV_8UC1);
    ASSERT_EQ(frame.image.rows, 2);
    ASSERT_EQ(frame.image.cols, 2);
    // 不缩放、不直方图拉伸、不饱和：就是右移 4 位。
    EXPECT_EQ(frame.image.at<uint8_t>(0, 0), 128);
    EXPECT_EQ(frame.image.at<uint8_t>(0, 1), 255);
    EXPECT_EQ(frame.image.at<uint8_t>(1, 0), 0);
    EXPECT_EQ(frame.image.at<uint8_t>(1, 1), 64);

    // 原始载荷**逐字节**等于 SDK 交付缓冲（RAW 保真判据）。
    ASSERT_TRUE(frame.raw.bytes != nullptr);
    EXPECT_EQ(frame.raw.bytes->size(), 8u);
    EXPECT_EQ(frame.raw.format, data::PixelFormat::Mono12);
    EXPECT_EQ(frame.raw.validBits, 12);
    EXPECT_EQ(frame.raw.packing, data::Packing::Unpacked);
    EXPECT_EQ(frame.raw.bitAlignment, data::BitAlignment::LsbZeroPadded);
    EXPECT_EQ(frame.raw.sdkPayloadBytes, 8u);
    EXPECT_EQ(frame.raw.expectedCompactBytes, 8u);
    // ⚠ `compactSizeMatches` **只是校验条件**，不是"布局已确认"。
    EXPECT_TRUE(frame.raw.compactSizeMatches);
    EXPECT_EQ(frame.rawPolicy, data::RawDataPolicy::RawRequired);
}

TEST(ImvCameraBackendTest, OkReturnWithWrongLengthIsCorruptFrameNotTimeout)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 4;
    f.api->frameView.height      = 4;
    f.api->frameView.pixelFormat = 0x01100005;   // Mono12 ⇒ 期望 32 字节
    f.api->frameBytes.assign(30, 0x11);          // 实际 30 字节

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    // ⚠ 调用成功但帧不可用 ⇒ `CorruptFrame`，**绝不算超时**：
    //    超时是可重采的瞬态，长度不符是布局问题，处置完全不同。
    EXPECT_EQ(g.status, data::OpStatus::CorruptFrame);
    ASSERT_TRUE(g.sdkError.has_value());
    // SDK 调用本身确实成功了 ⇒ 如实保留 {GetFrame, IMV_OK}。
    EXPECT_EQ(g.sdkError->call, data::SdkCall::ImvGetFrame);
    EXPECT_EQ(g.sdkError->code, 0);
    EXPECT_FALSE(g.cleanupError.has_value());

    // 失败即**不发布帧**（§2.2 修正 2）。
    EXPECT_TRUE(frame.image.empty());
    EXPECT_TRUE(frame.raw.bytes == nullptr);

    // 帧已取出 ⇒ 必须释放（失败路径同样在唯一出口处释放）。
    EXPECT_EQ(f.api->releaseFrameCalls, 1);

    // §2／§9：本次取帧的诊断随结果走 —— 长度不符的现场必须能从这里读到，
    // 而不是只留在后端的 `lastErrorText_` 里等一个不会来读它的人。
    EXPECT_NE(g.diagnosis.find("recomputed_compact_bytes=32"), std::string::npos)
        << g.diagnosis;
    EXPECT_NE(g.diagnosis.find("sdk_payload_bytes=30"), std::string::npos);
    EXPECT_NE(g.diagnosis.find("padding_x=0"), std::string::npos);
    // ⚠ 调用**成功**过 ⇒ 帧状态是"取到了、值为 0"，**不是**"未取得"。
    ASSERT_TRUE(g.frameStatusRaw.has_value());
    EXPECT_EQ(*g.frameStatusRaw, 0u);
}

TEST(ImvCameraBackendTest, NonZeroSdkFrameStatusIsCorruptFrameNotNoFrame)
{
    // §2：`IMV_FrameInfo::status != 0` ⇒ `CorruptFrame`（1005），
    // 措辞为"**SDK 调用成功，但帧不满足可交付条件**"，
    // **不宣称**知道具体损坏原因（该字段的非零取值位含义未经 SDK 文档化，
    // 全树无枚举、无位定义），**原值进入本次取帧诊断**。
    //
    // ⚠ 删除的旧表述："任意非零帧状态即可推出 `NoFrame`"。据此推断
    //   "本拍无帧"是把一个未文档化的位串当成了自己认识的语义。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // 指针／尺寸／格式／长度**全部正常**，只有帧状态非零 ——
    // 这样它不会被别的校验提前拦住，命中的一定是状态检查本身。
    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;   // Mono12 ⇒ 8 字节
    f.api->frameView.status      = 0x00020001;   // 非零（本项目不解释它）
    f.api->frameBytes            = {0x00, 0x08, 0xFF, 0x0F,
                                    0x00, 0x00, 0x40, 0x00};

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    EXPECT_EQ(g.status, data::OpStatus::CorruptFrame)
        << "既不是 Ok，也不是 Timeout／NoFrame";
    EXPECT_EQ(data::appErrorCodeOf(g.status), 1005);
    ASSERT_TRUE(g.sdkError.has_value());
    // ⚠ SDK 调用**确实成功** ⇒ 如实保留 {GetFrame, IMV_OK}；
    //    帧状态原值**不得**冒充 SDK 返回码。
    EXPECT_EQ(g.sdkError->call, data::SdkCall::ImvGetFrame);
    EXPECT_EQ(g.sdkError->code, 0);
    EXPECT_FALSE(g.cleanupError.has_value());

    // 原值必须能区分"未取得"与"正常值 0"。
    ASSERT_TRUE(g.frameStatusRaw.has_value());
    EXPECT_EQ(*g.frameStatusRaw, 0x00020001u);
    EXPECT_NE(g.diagnosis.find("131073"), std::string::npos) << g.diagnosis;
    EXPECT_NE(g.diagnosis.find("不满足可交付条件"), std::string::npos)
        << g.diagnosis;

    // 失败即**不发布帧**（不得为了传诊断去交付一个失败帧）。
    EXPECT_TRUE(frame.image.empty());
    EXPECT_TRUE(frame.raw.bytes == nullptr);

    // 帧已取出 ⇒ 仍须释放，且**恰好一次**。
    EXPECT_EQ(f.api->releaseFrameCalls, 1);
}

TEST(ImvCameraBackendTest, CallFailureLeavesFrameStatusUnreadNotZero)
{
    // §2：`IMV_GetFrame` 返回非 0 ⇒ 我们**没有取得**帧描述，
    // SDK 也未保证失败时输出字段有效 ⇒ **不读** `view` 的任何字段，
    // `frameStatusRaw` 必须是 `nullopt`。
    //
    // ⚠ 这一条与"取到了、状态为 0"是**相反**的两件事：若把二者都记成 0，
    //    上层就再也分不出"设备说这帧没问题"与"这一帧根本没拿到"。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->retCode["IMV_GetFrame"] = -119;   // IMV_TIMEOUT
    // 故意把 view 填成一份"看起来正常"的帧 —— 实现**不该**去读它。
    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;
    f.api->frameView.status      = 0;
    f.api->frameBytes            = {0x00, 0x08, 0xFF, 0x0F,
                                    0x00, 0x00, 0x40, 0x00};

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    EXPECT_EQ(g.status, data::OpStatus::Timeout);
    EXPECT_FALSE(g.frameStatusRaw.has_value())
        << "未取得帧描述 ⇒ 是「未取得」，不是「值为 0」";
    EXPECT_TRUE(g.diagnosis.empty()) << "调用失败没有帧现场可留：" << g.diagnosis;
    EXPECT_EQ(f.api->releaseFrameCalls, 0) << "没取到帧，就没有帧要释放";
}

TEST(MultiCameraManagerTest, DiagnosticSinkCarriesCallNameRawCodeAndRecomputedLength)
{
    // §9 的**端到端一条链**：故障从 `StubImvApi` 注入（**不是**替身直接
    // 返回一段预拼好的错误文本），经**真实** `ImvCameraBackend`
    // → 管理器 → sink。
    //
    // ⚠ 为什么必须走真后端：替身返回的文本是我们自己写进去的，只能证明
    //   "文本能被搬运"，证明不了"现场信息确实从 SDK 视图里取到了"。
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-0001"));

    data::CameraConfig realCfg = makeCameraConfig(data::CameraRole::CAM25, "cam-real");
    realCfg.serialNumber = "SN-0001";
    realCfg.backend      = "imv";
    realCfg.triggerMode  = data::CameraTriggerMode::Software;
    realCfg.width        = 0;   // 关闭画幅比对，本用例只关心长度
    realCfg.height       = 0;

    auto realBackend = std::make_shared<ImvCameraBackend>(realCfg, api);
    // 一路真、两路脚本：整轮仍要靠另外两路达标，才能单独观察这一路的诊断。
    auto mgr = std::make_unique<MultiCameraManager>(
        realBackend,
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM50,
                                               std::vector<uint64_t>{7}),
        std::make_shared<ScriptedCameraBackend>(data::CameraRole::CAM100,
                                               std::vector<uint64_t>{9}));

    std::vector<std::string> sinkLines;
    mgr->setDiagnosticSink([&sinkLines](const std::string& text) {
        sinkLines.push_back(text);
    });

    ASSERT_TRUE(mgr->initializeAll());
    ASSERT_TRUE(mgr->startAll());

    // 4×4 Mono12 ⇒ 紧凑契约要求 32 字节；SDK 交回 30 字节。
    api->frameView.width       = 4;
    api->frameView.height      = 4;
    api->frameView.pixelFormat = 0x01100005;   // gvspPixelMono12
    api->frameView.status      = 0;
    api->frameBytes.assign(30, 0x11);

    data::MultiCameraFrame frame;
    EXPECT_TRUE(mgr->capture(frame, farDeadlineNs()))
        << "另两路可用 ⇒ 整轮仍成功，而这一路的故障必须已经落在日志里";

    ASSERT_EQ(sinkLines.size(), 1u)
        << "只该有这一条异常行（正常两路不产生输出）";
    const std::string& line = sinkLines.front();
    EXPECT_NE(line.find("CAM25"), std::string::npos) << line;
    EXPECT_NE(line.find("CorruptFrame"), std::string::npos) << line;
    EXPECT_NE(line.find("IMV_GetFrame"), std::string::npos)
        << "必须带**调用名**（同一个错误码在不同调用下是完全不同的故障）：" << line;
    EXPECT_NE(line.find("调用 IMV_GetFrame 返回 0"), std::string::npos)
        << "SDK 调用**成功**，故原码是 0（不得伪造非零码）：" << line;
    EXPECT_NE(line.find("recomputed_compact_bytes=32"), std::string::npos)
        << "必须带**重算长度** —— 只报实到长度看不出差在哪：" << line;
    EXPECT_NE(line.find("sdk_payload_bytes=30"), std::string::npos) << line;
    EXPECT_NE(line.find("padding_x=0"), std::string::npos)
        << "padding 是布局判据的一部分，必须可见：" << line;

    // 帧状态原值也必须随记录上到这一层（本次确实取得了帧描述）。
    const data::CaptureRound roundSnap = mgr->lastCaptureRound();
    const data::ChannelGrabRecord& rec = roundSnap.channel(data::CameraRole::CAM25);
    ASSERT_TRUE(rec.frameStatusRaw.has_value());
    EXPECT_EQ(*rec.frameStatusRaw, 0u);
    EXPECT_NE(rec.diagnosis.find("recomputed_compact_bytes=32"), std::string::npos);
}

TEST(ImvCameraBackendTest, OverflowingDimensionsAreRejectedWithoutCopying)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // 让 `width × height × bypp` 在 64 位里溢出：3 × (2^32−1)^2 ≈ 5.5e19 > 2^64。
    f.api->frameView.width       = 0xFFFFFFFFu;
    f.api->frameView.height      = 0xFFFFFFFFu;
    f.api->frameView.pixelFormat = 0x02180015;   // BGR8 ⇒ 每像素 3 字节
    f.api->frameBytes.assign(64, 0x22);

    // ⚠ 把交付指针指向一块**不可读**的地址：正确的实现（先校验乘法、
    //    溢出即拒绝）**一个字节都不会读**；而"先按长度复制、再检查"
    //    的实现会在这里当场崩溃。
    //    ⚠ 为什么不用"拷贝次数计数器"：后端没有任何渠道向替身汇报
    //    "我读没读你的缓冲"，计数器只能数调用次数，证明不了这件事。
    //    一块读不得的内存是本机唯一能给出的**直接**证据。
    ASSERT_NE(guardPagePointer(), nullptr)
        << "无法映射 PROT_NONE 页 —— 本用例的证据力依赖它，故直接判失败，"
           "而不是退化成一条证明不了任何事的断言";
    f.api->dataOverride = guardPagePointer();

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    EXPECT_EQ(g.status, data::OpStatus::CorruptFrame);
    EXPECT_TRUE(frame.image.empty());
    EXPECT_TRUE(frame.raw.bytes == nullptr);
    EXPECT_EQ(f.api->releaseFrameCalls, 1);
}

TEST(ImvCameraBackendTest, FrameGeometryMismatchWithConfigIsContractViolation)
{
    // 配置里的宽高是"标定时的分辨率"。实际画幅与它不符意味着内参主点
    // 整体平移，而内参不会因此失效 ⇒ 必须拦住（见 `buildFrameFromView`）。
    OpenRealBackend f;
    ASSERT_TRUE(f.open("SN-0001", data::CameraRole::CAM25,
                       /*frameW=*/640, /*frameH=*/480));

    f.api->frameView.width       = 2;   // 交付 2×2，而不是配置的 640×480
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;
    f.api->frameBytes.assign(8, 0x55);

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    EXPECT_EQ(g.status, data::OpStatus::ContractViolation);
    // SDK 调用成功了（帧本身没问题），是**设备与配置不符** ⇒ 原码保留 `IMV_OK`。
    ASSERT_TRUE(g.sdkError.has_value());
    EXPECT_EQ(g.sdkError->code, 0);
    EXPECT_TRUE(frame.image.empty());
    EXPECT_EQ(f.api->releaseFrameCalls, 1);
}

TEST(ImvCameraBackendTest, UnsupportedFormatsFailExplicitlyInsteadOfGuessing)
{
    struct Case
    {
        int32_t           code;
        data::OpStatus    expect;
        const char*       why;
    };
    const Case cases[] = {
        {0x010C0006, data::OpStatus::NotImplemented,
         "Mono12Packed 归 GigE Vision 2.0（PFNC 的 Mono12p 是 0x010C0047），本批按范围不做"},
        {0x00000000, data::OpStatus::NotImplemented, "未知格式码：明确失败，不回落、不猜"},
        {0x01100007, data::OpStatus::NotImplemented, "未知格式码"},
    };

    for (const Case& c : cases)
    {
        OpenRealBackend f;
        ASSERT_TRUE(f.open());
        f.api->frameView.width       = 2;
        f.api->frameView.height      = 2;
        f.api->frameView.pixelFormat = c.code;
        f.api->frameBytes.assign(16, 0x33);

        data::ImageFrame frame;
        const data::GrabResult g = f.backend->grab(frame, 100);
        EXPECT_EQ(g.status, c.expect) << c.why;
        EXPECT_TRUE(frame.image.empty());
        // 已取得的帧仍要归还。
        EXPECT_EQ(f.api->releaseFrameCalls, 1);
    }
}

TEST(ImvCameraBackendTest, SuccessfulGetFrameWithFailingReleaseFailsWholeOperation)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01080001;   // Mono8
    f.api->frameBytes.assign(4, 0x44);
    f.api->retCode["IMV_ReleaseFrame"] = -118;   // 释放失败

    data::ImageFrame frame;
    const data::GrabResult g = f.backend->grab(frame, 100);

    // ⚠ 原操作成功、必要清理失败 ⇒ **整体返回失败**，以该清理调用为
    //    错误来源。把"资源没还回去"说成成功是错的。
    EXPECT_FALSE(g.ok());
    EXPECT_EQ(g.status, data::OpStatus::Disconnected);
    ASSERT_TRUE(g.sdkError.has_value());
    EXPECT_EQ(g.sdkError->call, data::SdkCall::ImvReleaseFrame);
    EXPECT_EQ(g.sdkError->code, -118);
    // ⚠ V2.5 起，**同一个**失败也进 `cleanupError`（本用例上一版断言它为空）。
    //   理由：两个字段回答的是**两个不同的问题** ——
    //     · `sdkError`     = 失败的首因是什么（这里就是这次释放）；
    //     · `cleanupError` = **资源还回去了没有**（这里：没有）。
    //   而管理器的通道处置判据只看后者（它与"已有主失败 + 清理也失败"
    //   那条路径必须共用同一条规则），故这一处**不能不写**：
    //   少写它，"释放未获确认"就会在两条路径上拿到两种处置
    //   （帧检查通过 ⇒ 禁用；帧已损坏 ⇒ 继续用）—— 那正是本批要消除的
    //   不一致（011-A1 缺口 5 的收尾项）。
    ASSERT_TRUE(g.cleanupError.has_value())
        << "释放失败必须同时进 cleanupError：管理器的通道处置只看这个字段";
    EXPECT_EQ(g.cleanupError->call, data::SdkCall::ImvReleaseFrame);
    EXPECT_EQ(g.cleanupError->code, -118);

    // 帧**不交付**：输出帧只在成功条件全部满足之后才写入。
    EXPECT_TRUE(frame.image.empty());
    EXPECT_TRUE(frame.raw.bytes == nullptr);
}

TEST(ImvCameraBackendTest, CleanupFailureDoesNotOverwriteFirstCause)
{
    // "已有主失败 + 清理也失败" ⇒ 保留首因，失败另记。
    // 用 close() 构造（取帧失败路径不释放——没有帧可释放）。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->retCode["IMV_StopGrabbing"]  = -118;   // 首因
    f.api->retCode["IMV_Close"]         = -103;   // 清理失败
    f.api->retCode["IMV_DestroyHandle"] = 0;

    const data::OperationResult r = f.backend->close();

    EXPECT_FALSE(r.ok());
    ASSERT_TRUE(r.sdkError.has_value());
    EXPECT_EQ(r.sdkError->call, data::SdkCall::ImvStopGrabbing);   // 首因未被覆盖
    EXPECT_EQ(r.sdkError->code, -118);
    ASSERT_TRUE(r.cleanupError.has_value());
    EXPECT_EQ(r.cleanupError->call, data::SdkCall::ImvClose);
    EXPECT_EQ(r.cleanupError->code, -103);
}

TEST(ImvCameraBackendTest, InitFailureWithCleanupFailureKeepsFirstCauseAndReportsBoth)
{
    // §5 ①：`openDevice()` 的失败出口（本批把**七处**一律丢弃清理结果的
    // `(void)closeDevice();` 改成合并）——
    // 主因占 `status`／`sdkError`，清理失败只进 `cleanupError`，
    // 而**文本必须同时说得出两个原因**：只留首因会让"句柄没还回去"
    // 这件事消失，只留清理会让排查从错误的一头开始。
    OpenRealBackend f;

    // ⚠ 不能用 `f.open()` 之后再改注入码来构造这个场景：那时设备**已经**
    //    打开成功了。本用例要的是"打开流程中途失败"，故自己装一台替身。
    auto api = std::make_shared<StubImvApi>();
    api->devices.push_back(makeDeviceEntry("SN-0001"));

    // 打开流程在"配置触发"这一步失败（本批合并的七处出口之一）。
    api->retCode["IMV_SetEnumFeatureSymbol"] = -103;   // IMV_INVALID_PARAM
    // 清理也失败：设备关不掉。
    api->retCode["IMV_Close"] = -118;                  // IMV_NOT_CONNECTED

    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-0001";
    cfg.backend      = "imv";
    ImvCameraBackend backend(cfg, api);

    const data::OperationResult r = backend.initialize();

    EXPECT_FALSE(r.ok());
    ASSERT_TRUE(r.sdkError.has_value()) << "首因必须保留";
    EXPECT_EQ(r.sdkError->call, data::SdkCall::ImvSetEnumFeatureSymbol);
    EXPECT_EQ(r.sdkError->code, -103);
    ASSERT_TRUE(r.cleanupError.has_value())
        << "清理失败**不得**被丢弃（丢弃它 = 一句「以为还回去了」的谎）";
    EXPECT_EQ(r.cleanupError->call, data::SdkCall::ImvClose);
    EXPECT_EQ(r.cleanupError->code, -118);

    const std::string text = backend.lastErrorText();
    EXPECT_NE(text.find("IMV_SetEnumFeatureSymbol"), std::string::npos) << text;
    EXPECT_NE(text.find("-103"), std::string::npos) << text;
    EXPECT_NE(text.find("清理失败"), std::string::npos) << text;
    EXPECT_NE(text.find("IMV_Close"), std::string::npos) << text;
    EXPECT_NE(text.find("-118"), std::string::npos) << text;
}

TEST(ImvCameraBackendTest, IndependentCloseFailureTextDoesNotLeakHistoricalFault)
{
    // §5 ③：`close()` 是**独立**调用（拥有者关闭时走它），它的失败文本
    // 必须只说自己这一次的事 —— **不得**混进任何历史故障。
    //
    // ⚠ 这针对的是一个很容易写出来的实现：`closeDevice()` 无条件写一句
    //    "关闭时有未完成的步骤" —— 于是操作者看到一次与旧故障无关的
    //    关闭失败里出现了旧故障的字样，而返回值里根本没有那个故障。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    // 先制造一次有**特征文本**的取帧失败。
    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;
    f.api->frameView.status      = 0x00020001;   // 非零帧状态
    f.api->frameBytes            = {0x00, 0x08, 0xFF, 0x0F,
                                    0x00, 0x00, 0x40, 0x00};
    data::ImageFrame frame;
    ASSERT_EQ(f.backend->grab(frame, 100).status, data::OpStatus::CorruptFrame);
    ASSERT_NE(f.backend->lastErrorText().find("不满足可交付条件"), std::string::npos)
        << "先确认历史故障文本确实存在，否则本用例证明不了任何事";

    // 再让**独立**的 close() 失败。
    f.api->retCode["IMV_StopGrabbing"] = -118;
    const data::OperationResult r = f.backend->close();
    EXPECT_FALSE(r.ok());

    const std::string text = f.backend->lastErrorText();
    EXPECT_NE(text.find("IMV_StopGrabbing"), std::string::npos)
        << "本次关闭的失败必须说得出：" << text;
    EXPECT_EQ(text.find("不满足可交付条件"), std::string::npos)
        << "独立关闭的文本里不得出现历史故障：" << text;
    EXPECT_EQ(text.find("frameInfo.status"), std::string::npos) << text;
}

TEST(ImvCameraBackendTest, GrabExceptionReleasesExactlyOnceAndLeavesDiagnosticsBehind)
{
    // §5 ②：取帧过程中抛异常 ⇒
    //   · 帧**仍被释放**，且**恰好一次**（守卫兜底，不再是"跳过释放"）；
    //   · 异常**原样上抛**（本批不把它转成状态码）；
    //   · 本次异常与清理结果留在后端诊断状态里，**栈展开之后仍能读到** ——
    //     局部 `GrabResult` 已随展开销毁，`lastErrorText()` 是唯一出口。
    //
    // ⚠ 注入点必须在**目标窗口内**（`GetFrame` 成功之后、释放之前）。
    //    证据不是"第 N 次分配失败"这句自述，而是下面两条计数：
    //    `getFrameCalls == 1`（说明已经取到帧）且 `releaseFrameCalls == 1`
    //    （说明那次释放发生在异常路径上）。若注入落在 `GetFrame` **之前**，
    //    前者会变成 0，本用例当场转红。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01100005;   // Mono12 ⇒ 8 字节
    f.api->frameView.status      = 0;
    f.api->frameBytes            = {0x00, 0x08, 0xFF, 0x0F,
                                    0x00, 0x00, 0x40, 0x00};
    f.api->failNextAllocationAfterFrame = true;

    data::ImageFrame frame;
    bool threwBadAlloc = false;
    try
    {
        (void)f.backend->grab(frame, 100);
    }
    catch (const std::bad_alloc&)
    {
        threwBadAlloc = true;
    }

    EXPECT_TRUE(threwBadAlloc) << "异常必须原样上抛，不得被转成状态码";
    EXPECT_EQ(f.api->getFrameCalls, 1) << "注入点必须在取帧**之后**（否则本用例没验到目标路径）";
    EXPECT_EQ(f.api->releaseFrameCalls, 1) << "异常路径上仍须释放，且**恰好一次**";
    EXPECT_TRUE(frame.image.empty()) << "异常路径不交付帧";

    const std::string text = f.backend->lastErrorText();
    EXPECT_NE(text.find("抛出异常"), std::string::npos) << text;
    EXPECT_NE(text.find("bad_alloc"), std::string::npos)
        << "异常类型必须留下（只写「未知异常」会让离线排查从零开始）：" << text;
    EXPECT_NE(text.find("帧释放：成功"), std::string::npos) << text;
}

TEST(ImvCameraBackendTest, GrabExceptionSurvivesReleaseCallThatThrows)
{
    // §5 的**回归项**（V2.5，评审要求）：异常出口的每一步各自失败时，
    // **原异常必须仍是抛出去的那一个**。
    //
    // 本用例打第 ② 步 —— **释放调用本身抛出**：
    //   · 正确实现：`cleanup()` 捕获它并如实记为"未获确认"，原异常继续上抛；
    //   · 上一版的实现：`lease.cleanup()` 直接把替身的异常抛出去，
    //     末尾的 `throw;` **执行不到** ⇒ 调用方收到的是**替身异常**，
    //     原异常的现场永久丢失。
    //
    // ⚠ 为什么必须有**两种**注入种类：只注入一种异常时，"原异常被顶掉"与
    //   "原异常正常上抛"是同一个类型，下面那条断言**无论如何都会绿** ——
    //   它证明不了任何事。这里原异常是 `bad_alloc`（复制载荷时注入），
    //   二次异常是 `out_of_range`（替身在释放时抛出），类型可区分。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01080001;   // Mono8
    f.api->frameView.status      = 0;
    f.api->frameBytes.assign(4, 0x44);

    f.api->failNextAllocationAfterFrame = true;   // 原异常：复制载荷时失败
    f.api->throwOnReleaseFrame          = true;   // 二次异常：释放调用抛出

    data::ImageFrame frame;
    bool             sawOriginal  = false;
    bool             sawSecondary = false;
    std::string      other;
    try
    {
        (void)f.backend->grab(frame, 100);
    }
    catch (const std::bad_alloc&)
    {
        disarmAllocFailure();   // 第一条语句：别让倒计时漏进测试框架
        sawOriginal = true;
    }
    catch (const std::exception& e)
    {
        disarmAllocFailure();
        sawSecondary = true;
        other        = e.what();
    }

    EXPECT_TRUE(sawOriginal)
        << "原异常（bad_alloc）必须原样上抛；实际收到的是二次异常：" << other;
    EXPECT_FALSE(sawSecondary) << "释放抛出的异常顶掉了原异常：" << other;

    // 抛出之后**不得重试**：那次调用是否已经归还了缓冲无法判断，
    // 重试就是"一个帧释放两次"。
    EXPECT_EQ(f.api->releaseFrameCalls, 1)
        << "释放调用抛出后不得重试（不得再释放一次）";

    const std::string text = f.backend->lastErrorText();
    EXPECT_NE(text.find("调用抛出异常、无返回码"), std::string::npos)
        << "释放抛出的那次调用必须被如实记成「未获确认」，且不得把"
           "标记值当 SDK 原码念出来："
        << text;
    EXPECT_NE(text.find("std::bad_alloc"), std::string::npos)
        << "原异常的类型仍须留在诊断里：" << text;
}

TEST(ImvCameraBackendTest, GrabExceptionSurvivesFailingExceptionDescription)
{
    // §5 第 ① 步：`describeCurrentException()` 自己**要分配内存**，
    // 分配失败时不得顶掉原异常，且诊断里要留下"描述失败"的**占位说明**
    // （写空串会让人以为异常文本本身是空的）。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01080001;   // Mono8
    f.api->frameView.status      = 0;
    f.api->frameBytes.assign(4, 0x44);

    f.api->failNextAllocationAfterFrame = true;   // 原异常
    f.api->secondaryFailureCount        = 1;      // 只让"描述异常"这一步失败

    data::ImageFrame frame;
    bool             sawOriginal  = false;
    bool             sawSecondary = false;
    std::string      other;
    try
    {
        (void)f.backend->grab(frame, 100);
    }
    catch (const std::bad_alloc&)
    {
        disarmAllocFailure();
        sawOriginal = true;
    }
    catch (const std::exception& e)
    {
        disarmAllocFailure();
        sawSecondary = true;
        other        = e.what();
    }

    EXPECT_TRUE(sawOriginal)
        << "描述异常这一步失败不得顶掉原异常；实际收到：" << other;
    EXPECT_FALSE(sawSecondary) << other;
    EXPECT_EQ(f.api->releaseFrameCalls, 1);

    const std::string text = f.backend->lastErrorText();
    EXPECT_NE(text.find("描述本身构造失败"), std::string::npos)
        << "描述失败必须留下**说明这一点的**占位文本，而不是空串或"
           "一句看不出问题的话："
        << text;
    EXPECT_NE(text.find("帧释放：成功"), std::string::npos)
        << "释放本来成功了，这句话不能被别的失败抹掉：" << text;
}

TEST(ImvCameraBackendTest, GrabExceptionSurvivesFailingDiagnosticWrite)
{
    // §5 第 ③ 步：写诊断这一步同样要分配 —— 它失败时**诊断可以是空的**
    //（分配不出来就写不出字，这是硬限制），但**原异常必须是原异常**。
    // 这是本组用例的底线：诊断是附加信息，原异常才是调用方要处理的东西。
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    f.api->frameView.width       = 2;
    f.api->frameView.height      = 2;
    f.api->frameView.pixelFormat = 0x01080001;   // Mono8
    f.api->frameView.status      = 0;
    f.api->frameBytes.assign(4, 0x44);

    f.api->failNextAllocationAfterFrame = true;   // 原异常
    f.api->secondaryFailureCount        = 2;      // 描述与诊断两步都失败

    data::ImageFrame frame;
    bool             sawOriginal  = false;
    bool             sawSecondary = false;
    std::string      other;
    try
    {
        (void)f.backend->grab(frame, 100);
    }
    catch (const std::bad_alloc&)
    {
        disarmAllocFailure();
        sawOriginal = true;
    }
    catch (const std::exception& e)
    {
        disarmAllocFailure();
        sawSecondary = true;
        other        = e.what();
    }

    EXPECT_TRUE(sawOriginal)
        << "诊断写不出来时原异常更须原样上抛；实际收到：" << other;
    EXPECT_FALSE(sawSecondary) << other;
    EXPECT_EQ(f.api->releaseFrameCalls, 1)
        << "诊断失败**不得**影响释放：释放是资源问题，与诊断无关";

    // 诊断文本这里允许为空；但**不得**把二次异常写进去冒充原异常。
    const std::string text = f.backend->lastErrorText();
    EXPECT_EQ(text.find("SECONDARY"), std::string::npos)
        << "二次异常不得出现在诊断里：" << text;
}

TEST(ImvCameraBackendTest, CloseIsIdempotentAndDestroysUnderlyingExactlyOnce)
{
    OpenRealBackend f;
    ASSERT_TRUE(f.open());

    EXPECT_TRUE(f.backend->close().ok());
    EXPECT_EQ(f.api->stopGrabbingCalls, 1);
    EXPECT_EQ(f.api->closeCalls, 1);
    EXPECT_EQ(f.api->destroyHandleCalls, 1);

    // ⚠ 断言的是**底层资源**，不是包装调用次数：
    //    `close()` 这层幂等包装允许被拥有者与析构兜底各调一次，
    //    要防的是"第二次底层销毁"（重复释放），不是包装被多调。
    EXPECT_TRUE(f.backend->close().ok());
    EXPECT_EQ(f.api->closeCalls, 1);
    EXPECT_EQ(f.api->destroyHandleCalls, 1);
}

TEST(ImvCameraBackendTest, CloseWithoutInitializeIsOkAndCallsNoSdk)
{
    auto api = std::make_shared<StubImvApi>();
    data::CameraConfig cfg = makeCameraConfig(data::CameraRole::CAM25, "cam25");
    cfg.serialNumber = "SN-0001";
    cfg.backend      = "imv";

    ImvCameraBackend backend(cfg, api);
    EXPECT_TRUE(backend.close().ok());
    EXPECT_TRUE(api->callLog.empty());
}

TEST(PekoTurntableControllerTest, ReportsUnavailableAndDoesNotFakeMotion)
{
    // 转台无冗余（SYS-08 §7.5）。若桩假装成功，对准算法会在永远不动的
    // 转台上反复下发命令，直到耗尽 8 次 ALIGN 重试报 code=2003 ——
    // **错误码指向算法，根因却在设备桩**。
    data::TurntableConfig cfg = makeTurntableConfig();
    cfg.protocol = "pekod";

    PekoTurntableController turntable(cfg);
    EXPECT_FALSE(turntable.initialize());

    data::TurntableCommand cmd;
    cmd.azimuthCommand   = 5.0;
    cmd.elevationCommand = 5.0;
    EXPECT_FALSE(turntable.move(cmd));

    // state() 绝不回放 lastCommand_：那会让对准判据看到"已在目标上"，
    // 使一台没接的转台让整个流程继续走到给出错误结果。
    const data::TurntableState s = turntable.state();
    EXPECT_EQ(s.motion, data::TurntableMotionState::IDLE);
    EXPECT_DOUBLE_EQ(s.azimuth, 0.0);
}

TEST(HardwareTriggerControllerTest, ReportsUnavailable)
{
    data::TriggerConfig cfg;
    cfg.source          = "hardware";
    cfg.periodMs        = 100.0;
    cfg.syncToleranceNs = 1000000ULL;

    HardwareTriggerController trigger(cfg);
    EXPECT_FALSE(trigger.initialize());
    EXPECT_FALSE(trigger.enable());
    EXPECT_FALSE(trigger.trigger());
    EXPECT_EQ(trigger.lastError().code, data::kErrTriggerDegraded);
}

}  // namespace
}  // namespace device
}  // namespace aircraft
