#pragma once

// ============================================================================
//  src/data/ErrorInfo.h
//
//  依据：ENG-09 §4.1、§5.27（类型定义冻结 + 错误码分段冻结）、裁决 C-09
//        SYS-08 §7.3 / §7.4 / §7.5 / §7.7
//
//  裁决 C-09：SYS-06 §15 的相机异常、转台异常、触发异常三条链路都引用
//  `ErrorInfo`，但全套文档**从未定义**它。本文件即冻结补定义，
//  作为设备层向上层回报的统一错误载体。
//
//  ⚠ ENG-09 §5.27 的强制要求（不是建议）：
//  "状态机进入 FAILED 时，ErrorInfo.code 必须取自本表，
//    **不得使用裸数字字面量**；代码中以具名常量引用。"
//
//  为什么这条要求是硬性的：错误码是本系统唯一的机读故障分类依据
//  （SYS-08 §7.7 的三分类：瞬态 / 硬件故障 / 能力边界直接决定"是否重试"）。
//  裸字面量 2002 写成 2001 不会有任何编译错误，但会把"转台超行程"——
//  一个**不该重试**的能力边界故障 —— 变成"转台通信失败"这个可重试的瞬态
//  故障，于是系统会对一个物理上不可能到达的角度反复重试 8 次，直到耗尽
//  T_task（60 s）才报错。具名常量把这类错误变成编译期可见的符号差异。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <string>

namespace aircraft
{
namespace data
{

// ---------------------------------------------------------------------------
//  错误码（ENG-09 §5.27，冻结）
//
//  分段（段基址亦具名化，便于按范围归类判断）：
//      1000~1999  相机设备
//      2000~2999  转台设备
//      3000~3999  触发 / 同步
//      4000~4999  标定
//      5000~5999  模型与特征库
//      6000~6999  算法
//      9000~9999  系统级
// ---------------------------------------------------------------------------

/// 相机设备段基址。
inline constexpr int kErrSegCameraBase = 1000;
/// 转台设备段基址。
inline constexpr int kErrSegTurntableBase = 2000;
/// 触发/同步段基址。
inline constexpr int kErrSegTriggerBase = 3000;
/// 标定段基址。
inline constexpr int kErrSegCalibrationBase = 4000;
/// 模型与特征库段基址。
inline constexpr int kErrSegModelBase = 5000;
/// 算法段基址。
inline constexpr int kErrSegAlgorithmBase = 6000;
/// 系统级段基址。
inline constexpr int kErrSegSystemBase = 9000;

// ---- 已占用的具体错误码（ENG-09 §5.27 表，冻结，不得增改）----

/// 1001 | 可用相机数不足（<2），无法继续 | SYS-08 §7.5
/// 硬件故障类 —— **不重试**，直接 FAILED。
inline constexpr int kErrCameraInsufficient = 1001;

/// 1002 | 相机断连（已降级） | SYS-08 §7.5
/// 注意语义：这是**降级已生效**的记录，不是致命错误。
/// 3 台相机降为 2 台时仍可测量（degraded=true），此时置此码并继续。
inline constexpr int kErrCameraDegraded = 1002;

// ---- 取帧失败分类（011-A1，裁决 C-01 v1.7；编码见 ENG-09 V2.3 §5.27）----
//
// ⚠ 这五个码存在的**唯一**理由：R09 的已知根因是"**所有取帧失败被统一
// 处置**"（任一 `grabOne` 失败即永久禁用该通道），于是一次**超时**
// 与一次**断连**得到完全相同的后果。只加一个 bool 无法消除混同，
// 故把"失败的原因"变成可判别的码 —— 上位映射见 OpStatus.h 的
// `appErrorCodeOf()`（该函数是 `OpStatus` → 错误码的**唯一**映射点）。

/// 1003 | 相机取帧超时或无有效帧 | 011-A1
/// 涵盖两种：SDK 的取帧超时（`IMV_TIMEOUT`，且失败调用是 `IMV_GetFrame`），
/// 以及调用成功但设备表示**本拍无有效帧**。
/// 两者都是**瞬态** —— 不改变通道可用性。
inline constexpr int kErrGrabTimeout = 1003;

/// 1004 | 相机通道未就绪或未启动 | 011-A1
/// 涵盖：SDK 的 `IMV_NOT_GRABBING`、该通道不可用、以及入口处
/// `availableCount < 2` 使某些通道**根本没被尝试**的情形。
/// 后者的 message 必须写明"**为什么没尝试**"（否则 1004 会退化成
/// 一个说不出所以然的码）。
inline constexpr int kErrCameraNotStarted = 1004;

/// 1005 | 相机 SDK 错误或帧不可用 | 011-A1
/// 两类合并是有意的：**已调用 SDK 但归不了类**（原码保留在
/// `SdkFailure::code`，含 −114/−115/−116/−122 等），
/// 以及**调用成功但帧不可用**（维度为零、长度不符、格式未实现之外的
/// 字段问题）。两者的共同点是"设备侧的事实,需要人看"。
inline constexpr int kErrCameraSdkError = 1005;

/// 1006 | 取帧契约错误或参数非法（**本地判定，未调用 SDK**） | 011-A1
/// 涵盖：组合表矛盾、`compactSizeMatches` 为假后仍被交付、
/// `timeoutMs == 0` 这类参数错误。
/// ⚠ 与 1005 **不得混用**：1005 是"相机给的帧有问题"，
/// 1006 是"我们自己的代码或配置有问题" —— 现场动作完全不同。
inline constexpr int kErrGrabContract = 1006;

/// 1007 | 请求的相机格式/模式本批未实现 | 011-A1
/// 涵盖：`Mono12Packed`、`FreeRun`、以及未知的像素格式码。
/// 这是**稳态事实**（请求本身本批不支持），不是设备故障。
inline constexpr int kErrCameraUnsupported = 1007;

/// 2001 | 转台通信失败 | SYS-08 §7.7
/// 瞬态类，可重试；但转台无冗余（SYS-08 §7.5），重试用尽即 FAILED。
inline constexpr int kErrTurntableComm = 2001;

/// 2002 | 转台超出行程（能力边界） | SYS-08 §7.7
/// 能力边界类 —— **不重试**。见文件头"为什么具名常量是硬性要求"。
inline constexpr int kErrTurntableOverTravel = 2002;

/// 2003 | 对准重试次数用尽 | SYS-08 §7.3 / §7.7
/// 对应 MeasurementConfig::maxAlignAttempts（默认 8，经裁决 C-20 为唯一数据源）。
inline constexpr int kErrAlignRetryExhausted = 2003;

/// 3001 | 触发失效，已降级为软触发 | SYS-08 §7.5
/// 同 1002，是降级记录而非致命错误。
inline constexpr int kErrTriggerDegraded = 3001;

/// 3002 | 三路时间戳同步超差 | 裁决 C-006（2026-09-23）
/// 落点：CameraSynchronizer 判定三路帧时间戳的超差超出容限。
///
/// ⚠ 为什么它**不能**与 9004（kErrStateFailure）共用一个码：
/// 这是一个**可重试的瞬态**（下一帧可能就同步上了），而 9004 的语义是
/// "确实无法归类的兜底"。若二者同码，结果包里"同步超差"与"真的不知道
/// 为什么失败"将无法区分 —— 而那正是 C-006 要消除的模糊。
inline constexpr int kErrSyncOutOfTolerance = 3002;

// ---- 标定段（裁决 C-006 新增）----

/// 4001 | 标定数据缺失或无效，对准无法进行 | 裁决 C-006（2026-09-23）
/// 落点：`AlignmentController` 在 `!calibration.valid` 时的拒绝路径。
///
/// ⚠ 此前该路径借用裸 0（"未设置"）。裸 0 在 result.json 中渲染为 "OK"，
/// 于是一台**因缺标定而失败**的测量，其终态错误码看起来像"一切正常"。
/// 注意 4000~4999 段此前**一个码也没登记** —— 本码是该段第一个。
inline constexpr int kErrCalibrationMissing = 4001;

// ---- 模型与特征库段（裁决 C-006 新增）----

/// 5001 | 机型模型/特征库缺失，无法解算 | 裁决 C-006（2026-09-23）
/// 落点：控制器在 `solvePose` 返回 false **且**装配点注入的
/// `modelsAvailable == false` 时置此码。
///
/// ⚠ 判据为什么要带 `modelsAvailable` 这一半：`IPosePipeline::solvePose`
/// 的返回类型是纯 `bool`（IF-SW-02 冻结，不得改签名），机型库未加载与
/// 真正的 PnP 失败在返回值上**不可区分**。装配点是唯一知道机型库加载结果
/// 的地方，故由它把该事实注入控制器。判据的另一半保证不误报：
/// 机型库已加载而 PnP 失败时，码是 9004 而不是 5001。
inline constexpr int kErrModelMissing = 5001;

/// 6001 | PnP 重试次数用尽 | SYS-08 §7.3
/// 对应 MeasurementConfig::maxSolveAttempts（默认 2）。
inline constexpr int kErrPnpRetryExhausted = 6001;

/// 9001 | 单次测量任务超出 T_task | SYS-08 §7.1
/// 对应 MeasurementConfig::taskTimeoutNs（默认 60e9，测试可注入短值）。
/// 这是**任务级**时限，高于任何状态级时限，且不受状态重试次数约束。
inline constexpr int kErrTaskTimeout = 9001;

/// 9002 | 回退预算用尽 | SYS-08 §7.4
/// 对应 MeasurementConfig::maxRollbackTotal（默认 4）/ maxRollbackPerEdge（默认 2）。
inline constexpr int kErrRollbackExhausted = 9002;

/// 9003 | 人工取消 | SYS-08 §7.3
/// SEARCH 状态可由人工终止（其余状态不可）。
inline constexpr int kErrManualCancel = 9003;

/// 9004 | 状态级失败（**兜底码**） | 裁决 C-006（2026-09-23）
///
/// ⚠ 这是本表**唯一**允许用于"确实无法归入任何更具体码"的失败码。
/// 它取代的是此前被当作失败码使用的**裸 0** —— 裸 0 的冻结语义是
/// "未设置"，在 result.json 中渲染为 "OK"，导致失败任务的终态错误码
/// 在机读层面与成功任务无法区分。
///
/// ⚠ 使用纪律（不是建议）：每一处使用都必须在 message 里写明
/// "为何无法归类"。若某条失败路径能归入更具体的码（1001/2003/4001/
/// 5001/6001/9001/9002 等），**必须**用它而不是 9004 ——
/// 否则 9004 会退化成新的"未设置"，本码的引入就白做了。
/// 判据：最终 result.json 中 9004 的占比应可数且逐条可解释。
inline constexpr int kErrStateFailure = 9004;

/// 9005 | 系统配置加载失败 | 裁决 C-006（2026-09-23）
/// 落点：装配期 `SystemInitializer::loadConfigAndLogger()` 的配置加载失败。
///
/// ⚠ 这是**装配期**错误，与 9004 的"运行期状态级失败"不同段不同时机：
/// 它发生时状态机尚未启动，度量任务根本不会开始。此前该路径只写
/// `errorText_` 字符串，没有任何机读码，故启动失败在日志中无法按码筛出。
inline constexpr int kErrSystemConfig = 9005;

// ---------------------------------------------------------------------------

/// 统一错误载体（ENG-09 §5.27）。
struct ErrorInfo
{
    /// 错误码，取值见上文 kErr* 常量（**禁止裸字面量**）。
    /// 0 表示无错误 —— 这是唯一允许的裸值，语义为"未设置"。
    int code = 0;

    /// 人读描述。允许为空；机读判断一律以 code 为准。
    /// 不要把可解析的结构化信息塞进本字段（那会让机读逻辑依赖字符串格式）。
    std::string message;

    /// 发生时刻，单位 ns，主机 CLOCK_MONOTONIC（ENG-09 §2.5）。
    /// **禁止使用墙钟** —— 墙钟受 NTP 校时与人工改时影响，
    /// 用于超时判断时会产生跨越数十秒的假差值。
    uint64_t timestampNs = 0;
};

/// 错误码的稳定名称（用于日志与 result.json，便于离线排查时按名检索）。
///
/// 返回的是**编译期字符串字面量**，调用方不需要也不得释放。
/// 未在 ENG-09 §5.27 表中登记的非零码返回 "UNREGISTERED"，
/// 0 返回 "OK"。
///
/// 实现位于 ErrorInfo.cpp —— 这是 data 模块**唯一的编译单元**。
/// 它存在的原因：ENG-01 §5 把 data 列为纯头文件模块，而 ENG-03 §12.1
/// 与 3.md §15 要求产出 libAircraftData.a / libdata.a。二者只能靠"至少一个
/// TU"调和：这个 .cpp 只提供本查询函数，不引入任何业务逻辑，
/// 因此既不违反 ENG-01 §5 的"data 只有值类型"约束，
/// 又让 libdata 成为真实的归档文件而非空壳。
const char* errorCodeName(int code);

}  // namespace data
}  // namespace aircraft
