#pragma once

// ============================================================================
//  src/data/OpStatus.h
//
//  依据：ENG-09 V2.4 §5.29（取帧结果与诊断出口，冻结）、§5.27（错误码分段）
//        SYS-04 V2.5 §6.1 / SYS-06 V2.3 §5.1 / §6.1（接口返回语义）
//        裁决 C-01 v1.7（R09 部分修复：取帧失败分类）
//
//  作用：给**每一次设备操作**一个可判别的结果，使上层能按**原因**处置，
//  而不是只看一个 bool。
//
//  ⚠ 为什么必须分成这些取值（R09 的根因）：
//  R09 的已知根因**不是**"SDK 错误码是负数"，而是"**所有取帧失败被统一
//  禁用**" —— `MultiCameraManager` 里任一 `grabOne` 失败即
//  `ch->available = false`，于是一次**超时**（瞬态）就把通道永久判死，
//  与一次**断连**（硬件故障）得到完全相同的处置。
//  只在返回类型上加一个 bool 无法消除这个混同：调用方拿不到"为什么"，
//  就只能一律按最严重的情形处理。本文件把原因变成类型。
//
//  ⚠ **唯一权威状态**：结果对象只有 `status` 一个状态字段。
//  上一版设计同时有 `OperationResult::kind` 与 `GrabResult::status`，
//  于是 `kind == Ok && status == Timeout` 是**可表示的状态**，而 `ok()`
//  只看其中一个 —— 这类"两个字段各说各话"的结构必然在某个分支上
//  被写反，且编译器不会拦。现在 `ok()` 与所有分支判断**一律**看
//  `status`，不存在第二个可以与之矛盾的状态。
//
//  ⚠ 默认构造是 `Unset`（**不是** `Ok`）：`Unset` 语义为"**忘了赋值**"，
//  是一个缺陷指示，不是"未尝试"的合法取值。未尝试的通道必须**显式**
//  写入它未尝试的原因（`Timeout` + 预算耗尽 / `NotStarted` + 通道不可用），
//  见 MultiCameraManager 的聚合规则。若默认成 `Ok`，"忘了赋值"
//  会被当成成功，而成功路径下游会去读一个从未被填充的帧。
// ============================================================================

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace aircraft
{
namespace data
{

/// 设备操作的结果状态（ENG-09 V2.4 §5.29）。
enum class OpStatus
{
    /// 默认构造值 = **漏赋值**。任何已完成的调用都不得返回它。
    /// ⚠ 处置是**记录 + 聚合以最高档（100）暴露**（应用码 9004），
    /// 并在文本里点名是哪一路的哪个调用 —— 本仓**没有**运行期
    /// `assert()`（全仓仅 `PreviewManager.cpp` 有一处注释说明为何
    /// 用显式检查代替它），故"出现即断言"这种说法与实现不符，已改。
    Unset,

    /// 成功，且该操作要求的一切后置条件都已满足（取帧：帧已通过全部检查）。
    Ok,

    /// 本地判定：参数非法（例如 `timeoutMs == 0`、序列号为空）。
    ///
    /// ⚠ **判定类别与实际调用历史是两个问题，分别记录**（ENG-09 V2.5
    /// §5.29 的原则）：本字段的类别由**谁做出的判定**决定，而
    /// `sdkError` 由**这次操作有没有真的调用过 SDK**决定。本地判定
    /// **不代表此前未调用 SDK** ⇒ 旧版那句"本地判定 ⇒ `sdkError` 必须为空"
    /// 已废除：当判定所依据的数据**正是某次成功调用产出的**时，
    /// 那次调用必须如实保留（否则现场无从知道"是哪次调用给了这份数据"）。
    InvalidArgument,

    /// 本地判定：数据违反 ENG-09 V2.5 §5.28 第 1 条的契约（组合表矛盾、
    /// 帧状态非零却被交付、**后端报成功却交付空图**、
    /// `capturedCount` 与帧数不符）。
    ///
    /// ⚠ 本状态**可达**（空图与帧状态两条路径都在运行期出现过），
    /// 故它**不是**"只在代码缺陷时才出现"的指示；它表示"**某处的契约
    /// 被违反**"—— 可能是本项目的代码缺陷，也可能是某个后端实现
    /// 不满足接口契约。出现即需人看。
    /// ⚠ 三种情形**携带非空 `sdkError`**（依据见上一条原则）：画幅与配置
    /// 不一致、后端组装出的组合表矛盾（`{ImvGetFrame, IMV_OK}`）、
    /// 管理器收到"报成功却空图"而原样转发后端的调用诊断。
    ContractViolation,

    /// 本地判定：请求的格式/模式本批未实现（`Mono12Packed`、`FreeRun`、
    /// 未知格式码）。本路本次**未调用取帧**；`sdkError` 按实际调用情况
    /// 给出（同样**不**由本类别推出）。
    NotImplemented,

    /// 等待超时：SDK 返回 `IMV_TIMEOUT`（仅当失败调用是 `IMV_GetFrame`），
    /// 或**本地预算耗尽** —— 后者指**本路本次没有发起取帧**（预算在
    /// 发令前后就已用尽，见 §5.29 的三个检查点），故 `sdkError` 为空；
    /// 它说的是"**这一次取帧**没发生"，不是"本轮什么都没调用过"。
    Timeout,

    /// 调用成功，但设备侧表示**本拍没有有效帧**。
    ///
    /// ⚠ 本批**没有任何路径**能据此判定（011-A1 九项缺口 §2 的裁决）：
    ///   `IMV_FrameInfo::status` 的非零取值**位含义未经 SDK 文档化**
    ///   （`IMVDefines.h:659` 只写明"0 是正常状态"，全树无枚举、无位定义），
    ///   而"非零"既**不能**证明"本拍无帧"、也**不能**证明"帧损坏" ⇒
    ///   一律按 `CorruptFrame` 处理，原值放进 `GrabResult::frameStatusRaw`。
    ///   `NoFrame` 保留给将来**有明确"无有效帧"依据**的判据；真实后端目前不产出它。
    NoFrame,

    /// 通道未就绪/未启动：SDK 返回 `IMV_NOT_GRABBING`，或本管理器判定
    /// 该通道不可用/未 `startAll()`。
    NotStarted,

    /// 断连：**仅**用于 SDK 错误码注释**明确**表示断连的情形
    /// （`IMV_NOT_CONNECTED` −118）。⚠ 本批**不注册**连接事件回调，
    /// 故一切断连判定都只能写"**按 SDK 错误码判定，未经连接事件确认**"，
    /// 不得写成"已确认断连"。
    Disconnected,

    /// 调用成功，但取得的帧**不满足可交付条件**（维度为零、长度不符、
    /// 格式未知、`IMV_FrameInfo::status != 0` 等）。
    ///
    /// ⚠ 措辞纪律：它表达"**SDK 调用成功，但帧不可交付**"，**不宣称**已经知道
    ///   具体的损坏原因 —— `status` 的非零取值**位含义未经文档化**，本项目
    ///   **不解释**它，只如实保留原值（`GrabResult::frameStatusRaw`）供离线排查。
    ///   "设备报了个非零状态"足以拒绝交付，但**不足以**推断出"本拍无帧"
    ///   （那需要另外的依据，见 `NoFrame`）。
    ///
    /// **不得**归为 `Timeout` —— "调用成功但帧不可交付"与"没等到帧"
    /// 的处置不同：前者要查格式/布局，后者要查触发与链路。
    CorruptFrame,

    /// 已调用 SDK，且按**有依据的**语义（常量注释）无法归类 ⇒
    /// 原码保留在 `SdkFailure::code` 里，不做推断。
    /// 本批归入此类的有：−114「取图恢复中」、−115「重连恢复中」、
    /// −116「连接不可达」、−122「调用时序错误」以及全部未登记码。
    SdkError
};

/// 一次 SDK 调用的**操作名**。与 `SdkFailure` 一起使用。
///
/// ⚠ 为什么失败记录必须带操作名而不只是返回码：同一个 −119，
/// `IMV_GetFrame` 超时与 `IMV_ReleaseFrame` 超时是**完全不同**的故障 ——
/// 前者意味着"没等到图"，后者意味着"资源没还回去"。只有一个码时，
/// 离线排查无法区分这两件事，而它们的现场动作截然相反。
enum class SdkCall
{
    None,
    ImvEnumDevices,
    ImvCreateHandle,
    ImvOpen,
    ImvGetDeviceInfo,
    ImvSetEnumFeatureSymbol,
    ImvGetEnumFeatureSymbol,
    /// ⚠ 下列四个名字**逐字取自 SDK 头文件**（`IMVApi.h:845/789/901/859`）。
    /// 本批原写的 `ImvSetIntFeature`／`ImvGetIntFeature`／
    /// `ImvSetFloatFeature`／`ImvGetFloatFeature` 是**臆造的名字** ——
    /// SDK 里没有这四个符号（`IMV_SetIntFeature` 不存在，
    /// 且 SDK **没有**任何 Float 特性 API，只有 Double）。
    /// 这不是命名口味问题：`sdkCallName()` 的用途是让人**照着名字去查
    /// SDK 文档**，名字不存在时该字段的价值为零，还会让人以为
    /// "是不是自己查错了版本"。
    ImvSetIntFeatureValue,
    ImvGetIntFeatureValue,
    ImvSetDoubleFeatureValue,
    ImvGetDoubleFeatureValue,
    ImvExecuteCommandFeature,  ///< 软件触发（`"TriggerSoftware"`）
    ImvStartGrabbing,
    ImvGetFrame,
    ImvReleaseFrame,
    ImvStopGrabbing,
    ImvClose,
    ImvDestroyHandle
};

/// 一次失败的 SDK 调用：**操作名 + 返回码**，不留裸码。
struct SdkFailure
{
    SdkCall call = SdkCall::None;
    int32_t code = 0;
};

/// 「**调用抛出、没有返回码**」的标记值（**不是** SDK 返回码）。
///
/// SDK 从不返回 `INT32_MIN`；用它是为了让 `SdkFailure{call, code}` 这个
/// 形状继续成立：清理调用（`IMV_ReleaseFrame` 等）**抛出异常**时，
/// 既拿不到返回码，又不能把这次清理当成"成功"（那会把"缓冲可能没还回去"
/// 说成"还回去了"）。
///
/// ⚠ 显示时必须走**专用措辞**（"调用抛出异常、无返回码"），**不得**把这个数字
/// 当 SDK 原码念出来 —— 现场看到 `-2147483648` 只会以为设备报了怪码。
inline constexpr int32_t kCallThrewCode = (std::numeric_limits<int32_t>::min)();

/// 设备操作的结果（ENG-09 V2.4 §5.29）。
struct OperationResult
{
    /// 消费者**只**依据本字段（`ok()` 也只看它）。
    OpStatus status = OpStatus::Unset;

    /// **首次失败**的归属。清理阶段的失败**不覆盖**它（见下）。
    /// · 未调用 SDK 的本地错误 ⇒ `nullopt`（不伪造"调用过"）；
    /// · 调用了且返回 `IMV_OK` ⇒ 如实保留 `{该调用, 0}`；
    /// · 调用了且失败 ⇒ `{该调用, 原码}`。
    std::optional<SdkFailure> sdkError;

    /// 清理（停流/释放帧/关闭设备）失败的单独留存。
    ///
    /// ⚠ 四条语义（缺一不可，见 ENG-09 V2.5 §5.29）：
    ///   ① **已有主失败**时清理也失败 ⇒ 保留首因（`status` 与 `sdkError`
    ///      都不变），失败记在这里；
    ///   ② **原操作成功、而必要清理失败** ⇒ **整体返回失败**，
    ///      以该清理调用为错误来源（`sdkError` 记它）——
    ///      把"资源没还回去"说成成功是错的；
    ///   ③ **清理失败一律进本字段**（含 ② 的情形）：`sdkError` 回答
    ///      "失败的首因是什么"，本字段回答"**资源还回去了没有**" ——
    ///      两个不同的问题，合并成一个字段就会丢一个答案；
    ///   ④ 清理成功 ⇒ 本字段为 `nullopt`。
    ///
    /// ⚠ ③ 是本批（V2.5）新增的**统一入口**：在此之前，只有"已有主失败"
    ///   时清理失败才进这里，于是"释放未获确认"这件事在
    ///   "帧检查通过而释放失败"与"帧已损坏且释放失败"两条路径上分别落在
    ///   `sdkError` 与 `cleanupError`，**管理器只按 `status` 决定通道可用性**
    ///   ⇒ 同样一件事拿到两种处置（一条禁用、一条继续用）。
    std::optional<SdkFailure> cleanupError;

    bool ok() const { return status == OpStatus::Ok; }
};

/// 取一帧的结果。**不再有第二个状态字段**（`GrabStatus` 已删除）。
///
/// ⚠ **诊断随本次结果走**（011-A1 九项缺口 §2/§9 的裁决）：取帧的现场信息由
/// **这一次调用**携带，上层**不得**改去读后端的 `lastErrorText()` —— 那是
/// "最近一次失败文本"，可能属于更早的调用（陈旧），当作本轮事实会张冠李戴。
/// 传递路径固定为：
///   `SDK 帧视图 → GrabResult（本次诊断） → ChannelGrabRecord → 日志/失败记录`。
/// ⚠ 失败时**不修改**输出 `ImageFrame`（`ICameraBackend.h` 的"`status != Ok`
/// 时 frame 不被触碰"）—— **不**为了传诊断而交付失败帧。
struct GrabResult : OperationResult
{
    /// SDK 回报的帧状态**原值**（`IMV_FrameInfo::status`）。
    ///
    /// ⚠ 用 `optional` 而不是 `uint32_t`：必须能区分"**未取得**"（例如
    /// `IMV_GetFrame` 返回非 `IMV_OK`，本批**不读**失败的输出字段 ⇒ 没取得）
    /// 与"**取到了、值为 0**"（正常帧）。用 0 兜底会把这两种情形混成一件，
    /// 而它们的现场动作相反。
    ///
    /// ⚠ 默认值写成 `= std::nullopt`（而不是留空）是**有意的**：
    /// 本结构大量以 `GrabResult{{status, sdkError, cleanupError}}` 的形式
    /// 构造（只给三项通用字段），而有默认成员初始值才使这种构造
    /// **不触发** `-Wmissing-field-initializers`；否则全仓十几处构造点
    /// 都会各冒一条警告，久而久之大家对警告脱敏 —— 而真正该看的那条
    /// 也会被淹没。
    std::optional<uint32_t> frameStatusRaw = std::nullopt;

    /// 本次取帧的现场信息（人读）：长度、期望长度、重算长度、SDK 格式码、
    /// padding 等只在失败时才有内容的量。**成功取帧时为空串**。
    std::string diagnosis{};
};

/// 结果状态的稳定名称（日志与离线排查用，返回编译期字面量）。
inline const char* opStatusName(OpStatus status)
{
    switch (status)
    {
    case OpStatus::Unset:
        return "Unset";
    case OpStatus::Ok:
        return "Ok";
    case OpStatus::InvalidArgument:
        return "InvalidArgument";
    case OpStatus::ContractViolation:
        return "ContractViolation";
    case OpStatus::NotImplemented:
        return "NotImplemented";
    case OpStatus::Timeout:
        return "Timeout";
    case OpStatus::NoFrame:
        return "NoFrame";
    case OpStatus::NotStarted:
        return "NotStarted";
    case OpStatus::Disconnected:
        return "Disconnected";
    case OpStatus::CorruptFrame:
        return "CorruptFrame";
    case OpStatus::SdkError:
        return "SdkError";
    }
    return "Unknown";
}

/// SDK 调用名的稳定名称。
inline const char* sdkCallName(SdkCall call)
{
    switch (call)
    {
    case SdkCall::None:
        return "None";
    case SdkCall::ImvEnumDevices:
        return "IMV_EnumDevices";
    case SdkCall::ImvCreateHandle:
        return "IMV_CreateHandle";
    case SdkCall::ImvOpen:
        return "IMV_Open";
    case SdkCall::ImvGetDeviceInfo:
        return "IMV_GetDeviceInfo";
    case SdkCall::ImvSetEnumFeatureSymbol:
        return "IMV_SetEnumFeatureSymbol";
    case SdkCall::ImvGetEnumFeatureSymbol:
        return "IMV_GetEnumFeatureSymbol";
    case SdkCall::ImvSetIntFeatureValue:
        return "IMV_SetIntFeatureValue";
    case SdkCall::ImvGetIntFeatureValue:
        return "IMV_GetIntFeatureValue";
    case SdkCall::ImvSetDoubleFeatureValue:
        return "IMV_SetDoubleFeatureValue";
    case SdkCall::ImvGetDoubleFeatureValue:
        return "IMV_GetDoubleFeatureValue";
    case SdkCall::ImvExecuteCommandFeature:
        return "IMV_ExecuteCommandFeature";
    case SdkCall::ImvStartGrabbing:
        return "IMV_StartGrabbing";
    case SdkCall::ImvGetFrame:
        return "IMV_GetFrame";
    case SdkCall::ImvReleaseFrame:
        return "IMV_ReleaseFrame";
    case SdkCall::ImvStopGrabbing:
        return "IMV_StopGrabbing";
    case SdkCall::ImvClose:
        return "IMV_Close";
    case SdkCall::ImvDestroyHandle:
        return "IMV_DestroyHandle";
    }
    return "Unknown";
}

/// 聚合严重度（**大者胜**），用于把三路结果收敛成一个 `aggregate`。
///
/// ⚠ 本函数是**全部** `OpStatus` 取值的完备映射（无 default 兜底分支，
/// 枚举扩展时编译器会以 `-Wswitch` 提示）。上一版遗漏了
/// `InvalidArgument` / `ContractViolation` / `NotImplemented` / `Unset`，
/// 于是这几类失败在聚合时"不在表内"，只能靠某个兜底分支处理 ——
/// 而兜底分支的取值往往正好是最轻的那一档，使本地契约错误被静默降级。
///
/// 排序理由（从重到轻）：
///   `Unset`            —— 缺陷指示，必须压过一切（出现即需人看）
///   `ContractViolation` / `InvalidArgument` —— 本地判定，**正常运行时不可达**，
///                        出现即说明代码或配置有错，不能与"设备抖动"同级
///   `Disconnected`     —— 通道被禁用（现行策略），影响后续所有轮次
///   `SdkError`         —— 已调用 SDK 但归不了类 ⇒ 需要人看原码
///   `CorruptFrame`     —— 帧坏了但设备在，不改变通道可用性
///   `NotImplemented`   —— 请求本身本批不支持，属稳态事实而非故障
///   `Timeout` / `NoFrame` —— 瞬态，下一轮可能自愈
///   `NotStarted`       —— 通道不可用/未启动，是**起点状态**而非新故障
///   `Ok`               —— 不参与聚合
constexpr int aggregationSeverity(OpStatus status)
{
    switch (status)
    {
    case OpStatus::Unset:
        return 100;
    case OpStatus::ContractViolation:
        return 90;
    case OpStatus::InvalidArgument:
        return 85;
    case OpStatus::Disconnected:
        return 80;
    case OpStatus::SdkError:
        return 70;
    case OpStatus::CorruptFrame:
        return 60;
    case OpStatus::NotImplemented:
        return 50;
    case OpStatus::Timeout:
        return 40;
    case OpStatus::NoFrame:
        return 30;
    case OpStatus::NotStarted:
        return 20;
    case OpStatus::Ok:
        return 0;
    }
    return 0;
}

/// `OpStatus` → 应用错误码（ENG-09 §5.27）的**唯一**映射点。
///
/// ⚠ 本地契约错误**不得冒充 SDK 错误**：`ContractViolation` /
/// `InvalidArgument` 取 1006（本地契约与参数错误），而**不是** 1005
/// （SDK 错误与帧不可用）。二者在 result.json 里若同码，离线排查就
/// 无法区分"相机给的帧有问题"与"我们自己的代码/配置有问题"，
/// 而这两件事的现场动作完全不同。
///
/// 返回 0 表示"不置错"（`Ok`）。`Unset` 返回系统级兜底码 9004
/// （见 ErrorInfo.h 对 9004 的使用纪律：message 必须写明为何无法归类）。
inline int appErrorCodeOf(OpStatus status)
{
    switch (status)
    {
    case OpStatus::Ok:
        return 0;
    case OpStatus::Disconnected:
        return 1002;   // kErrCameraDegraded：断连（已降级）
    case OpStatus::Timeout:
    case OpStatus::NoFrame:
        return 1003;
    case OpStatus::NotStarted:
        return 1004;
    case OpStatus::SdkError:
    case OpStatus::CorruptFrame:
        return 1005;
    case OpStatus::ContractViolation:
    case OpStatus::InvalidArgument:
        return 1006;
    case OpStatus::NotImplemented:
        return 1007;
    case OpStatus::Unset:
        return 9004;   // kErrStateFailure：兜底码
    }
    return 9004;
}

/// `SdkFailure` 的**码**渲染成一句人话（**唯一**出口）。
///
/// ⚠ 为什么必须有这个助手，而不是各处直接 `std::to_string(code)`：
///   `code` 可能是 `kCallThrewCode` —— 那是**标记值、不是 SDK 返回码**。
///   直接打印会输出 `-2147483648`，现场只会把它读成一个**真实错误码**，
///   然后去查一个不存在的码。这正是本批要避免的形态
///   （与"用 `0` 冒充一个测出来的长度"同构：**假的具体值胜过缺值**是错的）。
///
/// 用法：所有把 `SdkFailure::code` 写进人读文本的地方**一律**走本函数；
/// 需要机器读的（如 `result.json` 的 `"code":`）不用它 —— 那里保留原值，
/// 由读包方按同一份 `data` 层定义解释。
inline std::string sdkFailureCodeText(const SdkFailure& failure)
{
    if (failure.code == kCallThrewCode)
    {
        return "调用抛出异常、无返回码";
    }
    return std::to_string(failure.code);
}

/// `SdkFailure` 的完整人读文本：`<调用名> 返回 <码>`／`<调用名> 调用抛出异常、无返回码`。
///
/// ⚠ 抛出情形的动词**不是**"返回"：那次调用没有返回任何东西，"返回 调用抛出…"
///   会把"没有返回值"重新说成一个返回值。措辞里的"调用抛出异常、无返回码"
///   与 `kCallThrewCode` 的定义同源，**不得**在别处另写一套。
inline std::string sdkFailureText(const SdkFailure& failure)
{
    const std::string call = sdkCallName(failure.call);
    if (failure.code == kCallThrewCode)
    {
        return call + " " + sdkFailureCodeText(failure);
    }
    return call + " 返回 " + sdkFailureCodeText(failure);
}

}  // namespace data
}  // namespace aircraft
