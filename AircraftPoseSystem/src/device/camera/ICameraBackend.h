#pragma once

// ============================================================================
//  src/device/camera/ICameraBackend.h
//
//  依据：SYS-04 V2.4 §6.1 ／ SYS-06 V2.3 §5.1 / §6.1（接口冻结）
//        ENG-09 V2.3 §4.3（类清单冻结）
//        ENG-01 §6（device 文件清单）、裁决 C-01 v1.7
//
//  作用：屏蔽具体 SDK。MultiCameraManager 只面对本接口，
//  因此"换成另一家相机"或"用虚拟相机跑测试"都不触及上层代码。
//
//  ⚠ 归属偏离（工作流文档 4.md 的 `src/interfaces/`）：
//  4.md 把四个设备接口集中在 `src/interfaces/`，本工程**不采纳**，
//  接口与实现同目录（`device/camera/`、`device/turntable/`、`device/trigger/`）。
//
//  不采纳的理由（与 001 阶段已登记的偏离 #1 同源）：
//    1. ENG-01 §6 的文件清单里**没有** `src/interfaces/` 这一级 ——
//       而 ENG-01 是目录结构的权威文档（ENG-01 §1 的权威性规则）。
//       新增一级目录会让"目录结构以 ENG-01 为准"这一约定在第一个
//       实现阶段就失效。
//    2. 集中式接口目录会使**跨层接口**失去层级归属。本工程上游的
//       `IMatchStatsStore`（裁决 C-21）正因如此才按"依赖关系的最低公共层"
//       落在 `data/`；若另设 interfaces/ 集中层，同一问题会出现两种解法。
//    3. 命名空间与目录路径保持一致（`device/camera/ICameraBackend.h`
//       → `aircraft::device::ICameraBackend`）后，读者由 include 路径即可
//       推知所属层，而 `interfaces::ICameraBackend` 这条路径无法回答
//       "这个接口属于哪一层"。
//
//  ⚠ 命名空间偏离：4.md 用 `aircraft::interfaces`。本工程用 `aircraft::device`
//  —— 全套冻结文档均**未定义**命名空间约定（ENG-09、ENG-03 中无相关条款），
//  故此处按"命名空间 = 目录路径"的工程惯例确定。接口与其实现同命名空间，
//  使 `ICameraBackend` 与 `VirtualCameraBackend` 无需跨命名空间引用。
//
//  ⚠ 签名依据（2026-09-26 改写，裁决 C-01 v1.7）：
//  本行原文写"接口签名**逐字**遵循 SYS-06 §6.1，未增删任何方法"。
//  该说法自本批起**不再成立**（本批增删了方法并改了返回类型），
//  故改为：**签名按 SYS-04 V2.4 §6.1 与 SYS-06 V2.3 §5.1 冻结；
//  相对 V2.3 的增删见 C-01 v1.7 的"源码接口"兼容性类别。**
//  把这句话留在原处会继续声称一个已被本批推翻的事实。
// ============================================================================

#include <cstdint>

#include "data/CameraTriggerMode.h"
#include "data/DeviceIdentity.h"
#include "data/ImageFrame.h"
#include "data/OpStatus.h"

namespace aircraft
{
namespace device
{

/// 单台相机的 SDK 适配接口（SYS-04 V2.4 §6.1 / SYS-06 V2.3 §5.1）。
///
/// 生命周期（SYS-09 §3 的设备线程约定）：
///     initialize() → start() → grab() … → stop() → close()
/// 本接口的实现**不负责线程安全**：每个 backend 归其所属设备线程独占，
/// 不允许多线程并发调用同一实例（SYS-09 §14 的数据所有权规则）。
///
/// ⚠ 三级释放的归属（011-A1，见下表）：`stop()` 只停流，`close()`
/// 才关设备并销毁句柄。**`stop()` 不等同于 `close()`** —— 前者可以
/// 反复调用、可以在未 start 时调用，后者结束该设备在本次进程中的
/// 全部占用。两者分开的理由：一次测量中的 start/stop 循环是常规操作，
/// 而"关设备"只在设备不再使用时发生；若合并，则每次 stop 都要重建
/// 句柄（真实 SDK 上表现为每次重连都要重新枚举与配置，耗时且易失败）。
///
/// | 层次 | 方法 | 幂等要求 |
/// |---|---|---|
/// | 停流（`IMV_StopGrabbing`） | `stop()` | 允许未 start 时调用、须无副作用 |
/// | 关设备 + 销毁句柄（`IMV_Close`/`IMV_DestroyHandle`） | `close()` | 允许未 initialize／已 close 时调用、须无副作用 |
class ICameraBackend
{
public:
    /// ⚠ 派生类的析构**必须**兜底调用一次 `close()`（幂等）：
    /// 拥有者正常关闭时会显式调 `close()`，但"启动中途失败"、
    /// "异常路径"都会绕过那条路径，而句柄一旦泄漏，同一台设备
    /// 在本次进程内再也打不开。
    virtual ~ICameraBackend() = default;

    /// 初始化：加载 SDK、按序列号打开设备、配置并**读回**参数。
    ///
    /// ⚠ 失败出口**必须自行 `close()`**：半初始化（已 `IMV_Open`
    /// 但配置/启动未完成）的句柄不许留在对象上 —— 它会让下一次
    /// `initialize()` 面对一个"看起来已打开"的设备。
    ///
    /// @return `status` 见 `OpStatus`；失败时 `sdkError` 记
    ///         **操作名 + 原始码**（本地参数错误时为空）。
    virtual data::OperationResult initialize() = 0;

    /// 开始采集（进入自由运行或等待触发）。
    virtual data::OperationResult start() = 0;

    /// 停止采集。**允许在未 start() 时调用（须无副作用，幂等）。**
    ///
    /// ⚠ 本方法的契约**未因本批改动**（ENG-09 V2.3 §4.3）：它只停流，
    /// 不关设备。`MultiCameraManager::stopAll()` 照旧"无论 available
    /// 与否都调"，理由见该处注释。
    virtual data::OperationResult stop() = 0;

    /// 关闭设备并销毁句柄（**新增**，011-A1）。
    ///
    /// 幂等：允许在未 `initialize()`、已 `close()` 之后调用，无副作用。
    /// 由调用方在"设备不再使用"时显式调用；后端自身析构再兜底一次。
    virtual data::OperationResult close() = 0;

    /// 取一帧图像，填入 frame。
    ///
    /// 实现须填满以下字段（ENG-09 §5.5 的 7 个字段 + §4.1 的 3 个新字段）：
    ///     image / frameId / timestampNs / deviceTimestampNs /
    ///     cameraId / role / exposureTime / raw / captureFormat / rawPolicy
    ///
    /// ⚠ timestampNs 必须取**主机 CLOCK_MONOTONIC**（ENG-09 §2.5），
    ///   禁止墙钟 —— 墙钟受 NTP 校时阶跃影响，会制造虚假的同步偏差，
    ///   而三相机同步判据（TriggerConfig::syncToleranceNs）正是建立在此
    ///   字段之上。
    /// ⚠ `image` 与 `raw.bytes` 都必须**真正拥有**数据：`IMV_GetFrame`
    ///   交出的是 SDK 内部缓存，复制必须在 `IMV_ReleaseFrame` 之前完成。
    ///
    /// ⚠ **本方法不发软件触发令**（§2.4 单一执行者）：软件触发由
    ///   `MultiCameraManager::capture()` 在调用本方法**之前**发一次。
    ///   后端再发一次会变成"两拍"，而两拍的表现是"偶发多出一帧"。
    ///
    /// @return `status != Ok` 时 **frame 不被触碰**（不写入任何字段，
    ///         包括不写部分字段）—— 写入半成品会诱导调用方误用。
    ///         `status` 的逐值语义见 ENG-09 V2.3 §5.29 的状态全表。
    virtual data::GrabResult grab(data::ImageFrame& frame,
                                  uint32_t         timeoutMs) = 0;

    /// 设置触发模式（三值，ENG-09 V2.3 §6.1）。
    ///
    /// ⚠ 硬触发失效时上层降级为软触发并置 `kErrTriggerDegraded(3001)`。
    /// **该降级的决策点不在本接口**，而在 `VirtualTriggerController::failing_`
    /// 与上层的降级路径（本批不动）。本方法只负责"把请求的模式真正
    /// 下发并读回"，读回不一致即失败（不静默接受"设了却没生效"）。
    virtual data::OperationResult setTriggerMode(data::CameraTriggerMode mode) = 0;

    /// 发一次软件触发命令（`IMV_ExecuteCommandFeature(h, "TriggerSoftware")`）。
    ///
    /// ⚠ 唯一调用者是 `MultiCameraManager::capture()`，且每路每轮**恰好一次**
    /// （§2.4）。调用方在预算不足时**既不发令也不取帧** —— 否则会
    /// "发出了触发脉冲却收不回帧"，把一次预算不足变成设备侧的悬空触发。
    ///
    /// @return 失败时 `sdkError = {ImvExecuteCommandFeature, 原码}`；
    ///         调用方据此把该路的取帧次数保持为 **0**。
    virtual data::OperationResult triggerSoftware() = 0;

    /// 设备身份（型号 + 序列号），来自 SDK 枚举回报。
    ///
    /// ⚠ **不得**从配置复制期望值来填实际身份：未取得就写"未取得"
    /// （`queried = true` + 空字段），否则"读不到序列号"会被伪装成
    /// "序列号与配置一致"，而设备身份绑定就此失去意义。
    /// 虚拟后端返回 `queried = false`（它没有设备可问）。
    virtual data::DeviceIdentity deviceIdentity() const = 0;

    /// 触发模式的**实际读回**（必须真的读设备，不是回抄请求值）。
    ///
    /// ⚠ 必须读**三项**（选择器 / 开关 / 触发源）：软件触发与硬件触发
    /// 在设备上都是 `TriggerMode = "On"`，只读开关分不出两者。
    /// 任一项读失败或返回空 ⇒ 对应 `optional` 置空、`reported` 为
    /// `nullopt`、`consistent = false` —— **不得**默认成某个合法模式。
    virtual data::TriggerModeState triggerModeState() const = 0;

    /// 本后端**最近一次失败的说明文本**（人类可读，可为空）。
    ///
    /// 为什么要放在接口上（2026-09-26 补，由一次实机失败路径实测得出）：
    /// 装配层要回答"这一路为什么没就绪"，而**唯一**知道原因的是后端自己
    /// —— 例如"按序列号精确匹配"失败时，后端手里有完整的**枚举结果**
    /// （型号／序列号／厂商／设备键），这正是操作者排查接错线序所需的
    /// 全部信息。若该文本只在 `ImvCameraBackend` 这个具体类上，
    /// 而 `ApplicationContext` 持有的是 `ICameraBackend`，装配层就只能
    /// 贴一句"（无附加说明——明细见各后端自身日志）"：**后端并不写那种
    /// 日志**，于是操作者眼前只剩"未就绪"三个字。实测复现：
    /// `backend: imv` + 一个本机不存在的序列号 ⇒ 进程级输出里没有任何
    /// 枚举信息，而计划 §3.2 明确要求"把枚举到的型号／序列号全列进错误信息"。
    ///
    /// ⚠ 与 `initialize()` 等的 `OperationResult` 分工：那是**机器可判**的
    /// 分类（`OpStatus` / `SdkCall` / 原码），这是**给人看**的说明；两者
    /// 都要有，不能拿一个当另一个。措辞纪律同样适用：不得把"未在枚举
    /// 结果里"写成"已确认断连"（见 `ImvCameraBackend` 的相应实现）。
    /// 虚拟后端返回自身状态说明（如"被模拟禁用"），无失败时返回空串。
    virtual std::string lastErrorText() const = 0;
};

}  // namespace device
}  // namespace aircraft
