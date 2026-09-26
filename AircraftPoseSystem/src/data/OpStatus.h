#pragma once

// ============================================================================
//  src/data/OpStatus.h
//
//  依据：ENG-09 V2.3 §5.29（取帧结果与诊断出口，冻结）、§5.27（错误码分段）
//        SYS-04 V2.4 §6.1 / SYS-06 V2.3 §5.1 / §6.1（接口返回语义）
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
#include <optional>

namespace aircraft
{
namespace data
{

/// 设备操作的结果状态（ENG-09 V2.3 §5.29）。
enum class OpStatus
{
    /// 默认构造值 = **漏赋值**。任何已完成的调用都不得返回它。
    /// 发布路径上出现即断言；release 中被放过则记 9004 并点名是
    /// 哪一路的哪个调用（见 `aggregationSeverity()` 的最高档）。
    Unset,

    /// 成功，且该操作要求的一切后置条件都已满足（取帧：帧已通过全部检查）。
    Ok,

    /// 本地判定：参数非法（例如 `timeoutMs == 0`、序列号为空）。
    /// **未调用 SDK**，故 `sdkError` 必须为空。
    InvalidArgument,

    /// 本地判定：数据违反 ENG-09 V2.3 §5.28 第 1 条的契约（组合表矛盾、
    /// `compactSizeMatches` 为假后仍被交付、`capturedCount` 与帧数不符）。
    /// **未调用 SDK**，故 `sdkError` 必须为空。
    /// 正常运行时**不可达** ⇒ 出现即需人看。
    ContractViolation,

    /// 本地判定：请求的格式/模式本批未实现（`Mono12Packed`、`FreeRun`、
    /// 未知格式码）。**未调用 SDK**。
    NotImplemented,

    /// 等待超时：SDK 返回 `IMV_TIMEOUT`（仅当失败调用是 `IMV_GetFrame`），
    /// 或**本地预算耗尽**（此时未调用 SDK）。
    Timeout,

    /// 调用成功，但设备侧表示**本拍没有有效帧**（`IMV_FrameInfo::status`）。
    NoFrame,

    /// 通道未就绪/未启动：SDK 返回 `IMV_NOT_GRABBING`，或本管理器判定
    /// 该通道不可用/未 `startAll()`。
    NotStarted,

    /// 断连：**仅**用于 SDK 错误码注释**明确**表示断连的情形
    /// （`IMV_NOT_CONNECTED` −118）。⚠ 本批**不注册**连接事件回调，
    /// 故一切断连判定都只能写"**按 SDK 错误码判定，未经连接事件确认**"，
    /// 不得写成"已确认断连"。
    Disconnected,

    /// 调用成功，但取得的帧不可用（维度为零、长度不符、格式未知等）。
    /// **不得**归为 `Timeout` —— "调用成功但帧坏了"与"没等到帧"
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

/// 设备操作的结果（ENG-09 V2.3 §5.29）。
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
    /// ⚠ 三条语义（缺一不可，见 ENG-09 V2.3 §5.29）：
    ///   ① **已有主失败**时清理也失败 ⇒ 保留首因（`status` 与 `sdkError`
    ///      都不变），失败记在这里；
    ///   ② **原操作成功、而必要清理失败** ⇒ **整体返回失败**，
    ///      以该清理调用为错误来源（`sdkError` 记它）——
    ///      把"资源没还回去"说成成功是错的；
    ///   ③ 清理成功 ⇒ 本字段为 `nullopt`。
    std::optional<SdkFailure> cleanupError;

    bool ok() const { return status == OpStatus::Ok; }
};

/// 取一帧的结果。**不再有第二个状态字段**（`GrabStatus` 已删除）。
struct GrabResult : OperationResult
{
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

}  // namespace data
}  // namespace aircraft
