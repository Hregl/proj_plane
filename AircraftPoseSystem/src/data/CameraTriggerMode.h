#pragma once

// ============================================================================
//  src/data/CameraTriggerMode.h
//
//  依据：ENG-09 V2.3 §5.31（触发模式契约，冻结）、§6.1（camera.yaml）
//        SYS-04 V2.4 §6.1 / SYS-06 V2.3 §6.1 / SYS-17 V1.1 §5
//        裁决 C-01 v1.7
//
//  作用：把相机触发模式从 `bool`（`true` = 硬触发）扩为三值，并把
//  "**实际读回了什么**"与"**请求了什么**"分开承载。
//
//  ⚠ 为什么要三值而不是 bool：本系统有三种真实存在的取帧方式
//  （自由运行 / 软件触发 / 硬件触发），而 `bool` 只能表达两种。
//  当前 CAM25 走**软件触发**、CAM50/100 走虚拟自由运行、
//  PH-01 才实施硬件触发 —— 用 bool 表达时，"软触发"与"自由运行"
//  必然有一个要被写成"硬触发的反面"，于是配置里那个 `false`
//  在两台设备上意味着完全不同的事，而没有任何一处能发现。
//
//  ⚠ 为什么**只读 `TriggerMode` 一个开关不够**（本文件的核心）：
//  软件触发与硬件触发在设备上**都是 `TriggerMode = "On"`**，
//  区别只在 `TriggerSource`（`"Software"` vs `"Line1"`）。
//  只读开关会让"设了 `TriggerSource` 但没生效"这类失效**读不出来** ——
//  而它正是本项目最警惕的形态：设备回报成功、行为却按另一种模式走。
//  故 `TriggerModeState` 要求**三项都读**（选择器/开关/触发源），
//  且**读不到要能表达"未知"**（`nullopt`），**不得**把"没读到"
//  默认成某个合法模式（那会让一次读失败伪装成"模式正确"）。
// ============================================================================

#include <optional>
#include <string>

namespace aircraft
{
namespace data
{

/// 相机触发模式（项目定义）。
enum class CameraTriggerMode
{
    /// 自由运行：不等待任何触发，设备连续出图。
    FreeRun,

    /// 软件触发：由上层发 `"TriggerSoftware"` 命令特性令后出图。
    /// 本批 CAM25 采用此模式。
    Software,

    /// 硬件触发：由外部电平/脉冲触发（`TriggerSource = "Line1"`）。
    /// ⚠ 本批**不实施**（归 PH-01），真实后端返回 `NotImplemented`。
    Hardware
};

/// 由枚举映射出的 GenICam 特性字符串（SDK 侧的实际下发值）。
///
/// ⚠ 依据（§2.3/§2.6 已核）：SDK **没有** `TriggerMode_Off` /
/// `TriggerSource_Software` 这类 C 枚举成员，模式是**运行时字符串特性名**；
/// 且 SDK 全树**没有 "FreeRun" 这个词**。故映射固化如下 ——
///
///   | `CameraTriggerMode` | `TriggerMode` | `TriggerSource` |
///   |---|---|---|
///   | `FreeRun`  | `"Off"` | 不设置 |
///   | `Software` | `"On"`  | `"Software"` |
///   | `Hardware` | `"On"`  | `"Line1"` |
///
/// ⚠ "Off 即自由运行"这一语义**只由样例代码支持、无 SDK 文档保证**，
/// 故 `TriggerModeState::consistent` 必须**如实反映实测差异**，
/// 不得在文档里把该语义写成 SDK 保证。
inline const char* triggerModeSymbol(CameraTriggerMode mode)
{
    return mode == CameraTriggerMode::FreeRun ? "Off" : "On";
}

/// 由枚举映射出的 `TriggerSource` 字符串。
///
/// `FreeRun` 返回 `nullptr` 表示**不下发** `TriggerSource`
/// （自由运行下该特性无意义，下发反而可能被设备拒绝或落到别的选择器上）。
inline const char* triggerSourceSymbol(CameraTriggerMode mode)
{
    switch (mode)
    {
    case CameraTriggerMode::FreeRun:
        return nullptr;
    case CameraTriggerMode::Software:
        return "Software";
    case CameraTriggerMode::Hardware:
        // SDK 自带样例中唯一出现的硬触发源（LineTrigger.cpp:73）。
        return "Line1";
    }
    return nullptr;
}

/// 触发模式的稳定名称（日志与离线排查用）。
inline const char* triggerModeName(CameraTriggerMode mode)
{
    switch (mode)
    {
    case CameraTriggerMode::FreeRun:
        return "FreeRun";
    case CameraTriggerMode::Software:
        return "Software";
    case CameraTriggerMode::Hardware:
        return "Hardware";
    }
    return "Unknown";
}

/// 由 `TriggerSource` 字符串反推模式（`nullptr`/空 ⇒ 未读到）。
inline std::optional<CameraTriggerMode> triggerModeFromSource(
    const std::optional<std::string>& sourceReported)
{
    if (!sourceReported.has_value() || sourceReported->empty())
    {
        return std::nullopt;
    }
    if (*sourceReported == "Software")
    {
        return CameraTriggerMode::Software;
    }
    if (*sourceReported == "Line1")
    {
        return CameraTriggerMode::Hardware;
    }
    // 其它触发源（Line2…/Counter…/PLC…）本批未使用，**不做猜测** ——
    // 猜成 Hardware 会让一个未验证的线号看起来像已被正确配置。
    return std::nullopt;
}

/// 触发模式的**请求值与实际读回值**。
struct TriggerModeState
{
    /// 请求值（来自配置/装配）。
    CameraTriggerMode requested = CameraTriggerMode::FreeRun;

    /// 实际生效的 `TriggerSelector`（如 `"FrameStart"`）。
    std::optional<std::string> selectorReported;

    /// 实际生效的 `TriggerMode`（`"On"`/`"Off"`）。
    std::optional<std::string> switchReported;

    /// 实际生效的 `TriggerSource`（`"Software"`/`"Line1"`…）。
    std::optional<std::string> sourceReported;

    /// 由上面**三项合成**的模式；**读不全 ⇒ `nullopt`（"未知"）**。
    std::optional<CameraTriggerMode> reported;

    /// 读全了**且**与 `requested` 一致才为 `true`。
    bool consistent = false;
};

/// 由已读到的三项**合成** `reported` 与 `consistent`。
///
/// 单一实现，两个后端与上层共用 —— 各处各写一份必然会在某次修改后
/// 分叉，而分叉的表现是"后端说一致、上层说不一致"。
///
/// 合成规则：
///   · `switchReported == "Off"` ⇒ `FreeRun`（**不需要**触发源）。
///   · `switchReported == "On"`  ⇒ 由 `sourceReported` 定（Software /
///     Line1），读不到 ⇒ `reported = nullopt`（**未知**）。
///   · `switchReported` 缺失，或取值既非 "On" 也非 "Off" ⇒ 未知。
///
/// ⚠ "`Off` 即自由运行"这一条**只由 SDK 样例代码支持、无文档保证**
/// （§2.6）：故它只用于**合成读回值**，不得据此断言"设备一定在自由运行"。
/// 若实机读回的取值与样例不同，`reported` 会是 `nullopt`（诚实地说"不知道"），
/// 而不是被硬塞进某个枚举值。
///
/// @param state 已填好 requested 与三项读回值的对象（本函数就地补全）
/// @return 补全后的对象
inline TriggerModeState composeTriggerModeState(TriggerModeState state)
{
    state.reported.reset();

    if (!state.switchReported.has_value() || state.switchReported->empty())
    {
        state.consistent = false;
        return state;
    }

    if (*state.switchReported == "Off")
    {
        state.reported = CameraTriggerMode::FreeRun;
    }
    else if (*state.switchReported == "On")
    {
        state.reported = triggerModeFromSource(state.sourceReported);
    }

    state.consistent =
        state.reported.has_value() && *state.reported == state.requested;
    return state;
}

}  // namespace data
}  // namespace aircraft
