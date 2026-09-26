#pragma once

// ============================================================================
//  src/device/camera/ImvCameraBackend.h
//
//  依据：SYS-06 §7（ImvCameraBackend）、ENG-09 V2.4 §4.3 / §5.5 / §2.5 / §14 附录 B
//        SYS-04 V2.5 §6.1（ICameraBackend 签名）、裁决 C-01 v1.7
//
//  作用：华睿 A7A20MU201 的 SDK 适配层。
//
//  ⚠ 实现状态（2026-09-26，011-A1）：**真实适配已实现**。
//  打开、配置并读回、取帧、分类、复制、释放、身份与触发模式读回均已写出；
//  SDK 符号只出现在 `ImvApiReal.cpp`（经 `IImvApi` 缝，见 IImvApi.h）。
//
//  ⚠ 本文件**不引入任何 SDK 头**（既有约束，保持不变）：本头文件会被
//  device 层其它文件包含，而 SDK 是**可选**依赖（ENG-03 §12.3）——
//  一旦 SDK 缺失，整个 device 模块（含虚拟相机）都编译不了。
//  ∴ 句柄保持 `void*`，SDK 类型被限制在 `ImvApiReal.cpp` 内部。
//
//  ⚠ 旧文更正（2026-09-26）：本文件原写"当前构建中 `APS_HAVE_IMVSDK`
//  未定义（SDK 未安装，见 Dependencies.cmake）"，该句**已失效**：
//  本机 `third_party/imvsdk/` 实际存在（含 `include/` 与 `lib/m64x86/`），
//  `build/` 中的 `APS_HAVE_IMVSDK` 为 **1**。规则改为如实陈述：
//  **找到 SDK 则编译真实实现；未找到则 `makeRealImvApi()` 返回空指针，
//  本节走"诚实桩"路径（明确失败 + 可操作提示）。**
//  该宏由 `camera/CMakeLists.txt:67-73` **总是**定义 0/1（不是"有则定义"），
//  故 `#if` 能区分"没 SDK"与"宏名拼错"—— 与 `src/device/CMakeLists.txt`
//  旧注释"当前开发机两者都没有"同属已失效的环境描述，两处一并更正。
//
//  ⚠ 三个实例，一个类（SYS-06 §7）：
//  该节写"三个实例：ImvCameraBackend25 / 50 / 100"，而 ENG-09 §4.3
//  的冻结类清单只有 `ImvCameraBackend` 一个类。二者并不冲突 ——
//  "三个实例"指的是**实例数量**而非三个类。本工程按 ENG-09 §4.3
//  实现单一类，由 CameraConfig 区分角色（与 VirtualCameraBackend 同构）。
//  若真按三个类实现，则三段代码的 99% 是同一份 SDK 调用序列，
//  任何 SDK 调用顺序的修正都要改三处并分别验证。
//
//  ⚠ 为什么 `initialize()` 不是"假的成功"：
//  若它返回 Ok 却不真正采集，`MultiCameraManager` 会把该通道计入
//  "可用相机数"（SYS-08 §7.5），于是可用数虚高、降级判据失效，
//  而 `grab()` 的失败又会被当成"采集途中掉线"触发降级 ——
//  两条路径互相矛盾，表现为"三台都在却反复降级"。
//  本类的每条成功路径都对应一次**真实**完成的 SDK 调用与读回。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <memory>
#include <string>

#include "data/CameraConfig.h"
#include "data/DeviceIdentity.h"
#include "data/DeviceState.h"
#include "data/ImageFrame.h"
#include "data/OpStatus.h"   // OperationResult / GrabResult / SdkFailure / SdkCall
#include "device/camera/ICameraBackend.h"
#include "device/camera/IImvApi.h"

namespace aircraft
{
namespace device
{

/// 华睿相机 SDK 适配（SYS-06 §7）。
class ImvCameraBackend : public ICameraBackend
{
public:
    /// @param config 相机配置（ENG-09 §6.1）。真实后端**不使用**
    ///        width/height 的回退值 —— 画幅由设备回报，
    ///        配置中的尺寸仅用于与该回报值比对（不一致即报错，
    ///        因为内参矩阵是按标定时的分辨率标定的，见
    ///        CameraCalibration.h 的说明）。
    /// @param api SDK 访问层。传 `nullptr`（默认）表示"用真实 SDK"：
    ///        构造时会调 `makeRealImvApi()`，SDK 未接入即得空指针，
    ///        于是 `initialize()` 返回 `NotImplemented` 并给出可操作提示。
    ///        ⚠ 传入非空对象即**替换**整条 SDK 通路（测试替身路径，§4.2）——
    ///        这是本类可被验证的前提：本机没有相机。
    explicit ImvCameraBackend(const data::CameraConfig& config,
                              std::shared_ptr<IImvApi>    api = nullptr);

    ~ImvCameraBackend() override;

    // ---- ICameraBackend ----

    data::OperationResult initialize() override;
    data::OperationResult start() override;
    data::OperationResult stop() override;
    data::OperationResult close() override;
    data::GrabResult grab(data::ImageFrame& frame, uint32_t timeoutMs) override;
    data::OperationResult setTriggerMode(data::CameraTriggerMode mode) override;
    data::OperationResult triggerSoftware() override;
    data::DeviceIdentity deviceIdentity() const override;
    data::TriggerModeState triggerModeState() const override;

    // ---- 诊断 ----

    /// 最近一次失败的原因（人读）。用于日志与 UI，
    /// **不参与任何控制流判断** —— 控制流一律以返回值为准。
    ///
    /// ⚠ 本批起**也是** `ICameraBackend` 的虚方法（见那里的说明：装配层
    /// 只持有接口指针，"这一路为什么没就绪"必须能从接口上问到）。
    std::string lastErrorText() const override;

    /// 设备状态（SYS-06 §14）。⚠ **不是** `ICameraBackend` 的虚方法 ——
    /// 该接口的方法面里**没有** `state()`（已核 ICameraBackend.h）。
    /// 故本方法与 `VirtualCameraBackend::state()` 一样，是具象类各自的
    /// 查询能力，不参与接口契约；`MultiCameraManager::state()` 也不经
    /// 接口调它（它按 `availableCameraCount()` 与 `started_` 自行判定）。
    data::DeviceState state() const;

private:
    /// 打开设备：枚举 → 按序列号精确匹配 → 建句柄 → 打开 → 复核身份 →
    /// 配置并读回（像素格式／曝光／增益／触发）。
    /// 失败时**自行清理**已建立的资源（不许留下半初始化的句柄）。
    data::OperationResult openDevice();

    /// 关闭设备：停流 → 关设备 → 销毁句柄，逐项保留失败原因。
    /// 幂等；未打开时返回 `Ok` 且**不调用任何 SDK**。
    ///
    /// ⚠ 本方法**不写 `lastErrorText_`**（011-A1 九项缺口 §5）：它是被
    /// `openDevice()` 的七处失败出口与 `close()` **共用**的收尾动作，
    /// 若在这里无条件写文本，"独立关闭一个正常设备时失败"就会被贴上
    /// 上一次初始化失败的原因（那件事与本次无关）。首因文本由**调用方**
    /// 合并：初始化出口见 `mergeInitCleanup()`，独立 `close()` 自己写。
    data::OperationResult closeDevice();

    /// 按 `triggerMode` 写触发配置。Software 时按
    /// `TriggerSelector("FrameStart") → TriggerSource → TriggerMode` 顺序写。
    data::OperationResult applyTriggerMode(data::CameraTriggerMode mode);

    /// 配置并**读回**一个枚举特性；读回值不等于 `expect` 即失败。
    data::OperationResult configureEnumAndVerify(const char* feature,
                                                const char* expect,
                                                data::SdkCall setCall,
                                                data::SdkCall getCall);

    /// 配置并读回曝光（配置单位 s，SDK 单位 µs）。
    data::OperationResult configureExposure();

    /// 配置并读回增益（单位 dB，与 SDK 同名特性）。
    data::OperationResult configureGain();

    /// 把一帧视图按本批契约校验、复制并组装成 `ImageFrame`。
    ///
    /// ⚠ 五项校验**全部在复制之前**完成（§3.2 步骤 3）：指针非空、
    /// 宽高非零、每像素字节可定、乘法**以 64 位做并检查溢出**、
    /// 长度与紧凑契约相符。任一项不过即返回失败且**不发生复制**。
    /// @param nowNs 主机接收时刻（在 `getFrame()` 返回后立即取得，
    ///        **早于**复制与转换 —— 它表示"何时收到"，不是"何时处理完"）。
    /// @param out   输出帧。⚠ **只在全部检查通过后**才被写入
    ///        （`status != Ok` 时 `out` 一字不动，见接口契约"失败不发布帧"）。
    /// ⚠ 参数类型是 `ImvFrameView`（本层），**不是** `data::ImvFrameView`：
    /// 帧视图是"某个厂商 SDK 交付缓冲的裸描述"，属设备层内部描述，
    /// 不是跨层类型（`data::` 里那些是契约类型）。
    data::GrabResult buildFrameFromView(const ImvFrameView& view, uint64_t nowNs,
                                       data::ImageFrame& out);

    /// 释放最近取得的帧，返回清理失败（`nullopt` = 成功）。
    std::optional<data::SdkFailure> releaseFrame();

    /// 把清理结果并进主结果（§2.2 修正 4 的**唯一**落点）。
    /// @param frame 仅当合并后仍为 Ok 时才被写入（见 .cpp 的唯一交付点）。
    void mergeCleanup(data::GrabResult&                     result,
                      const std::optional<data::SdkFailure>& cleanup);

    /// `openDevice()` 的**统一失败出口**（011-A1 九项缺口 §5）。
    ///
    /// 七处失败出口（打开／取设备信息／身份复核／像素格式／曝光／增益／
    /// 触发配置）此前一律 `(void)closeDevice();` —— 清理结果被丢弃，
    /// 于是"设备已经半开、还关不掉"这件事在返回值里**完全看不见**。
    ///
    /// 合并规则（与 `mergeCleanup` 同义，方向相反）：`first` 是**本次失败**
    /// 的局部快照，它占 `status`／`sdkError`；清理失败只进 `cleanupError`。
    /// 文本 = **本次首因快照**（进入本函数时的 `lastErrorText_`）＋
    /// （清理失败时）"；清理失败：<调用名> 返回 <码>"。
    /// ⚠ 首因取自**局部快照**而不是 `closeDevice()` 之后再去读
    /// `lastErrorText_`：后者在"清理也失败"时已被覆盖，读到的会是清理的
    /// 原因，而返回值里的 `status` 仍是首因 —— 返回状态与错误文本
    /// 指向不同原因，正是本项要消除的形态。
    data::OperationResult mergeInitCleanup(const data::OperationResult& first);

    /// 把**异常路径**的现场写进后端诊断状态（§5）。
    ///
    /// ⚠ 为什么不能只写进 `GrabResult`：异常从 `grab()` 抛出后，
    /// 局部 `GrabResult` 随栈展开销毁，调用方**永远读不到**它 ——
    /// 于是"帧释放了没有、为什么又失败了"在异常路径上完全不可见。
    /// 故异常路径的出口是**后端自身的诊断状态**，由既有的
    /// `lastErrorText()` 读出（不新增接口面）。
    ///
    /// ⚠ 本函数**只记录**，不改变任何控制流：原异常必须原样上抛
    /// （本批**不把异常转成状态码**，那不在九项缺口内、亦未经裁决）。
    /// 记录取帧异常（**唯一**在栈展开后仍可读的出口）。
    ///
    /// ⚠ `exceptionText` 用 `const char*` 且允许 `nullptr`：本函数是
    ///   异常出口的第 ③ 步，而"描述原异常"（第 ① 步）自己就可能因为
    ///   分配失败而拿不到文本 —— 那时传 `nullptr`，本函数写一句
    ///   **说明"描述失败"**的占位文本，而不是写成空的一般化句子。
    /// ⚠ 调用方（catch 出口）必须把本调用关在 try/catch 里：它要分配
    ///   字符串，抛出会顶掉在飞的原异常。
    /// ⚠ `kCallThrewCode` 的措辞**只由 `data::sdkFailureCodeText()` 给出**
    ///   （本函数走的就是它）。⚠ 这条纪律的上一版写成"只在本函数里被翻译"
    ///   —— 那句话当时是**假的**：另有 9 处直接打印了码值。措辞不落在调用点上，
    ///   落在助手那里才是可核对的（同批已把 10 处全部改走助手）。
    void noteGrabException(const char*                            exceptionText,
                           const std::optional<data::SdkFailure>& cleanup);

    /// 取帧期间的"帧租约"守卫（§5 异常安全）。
    ///
    /// 从 `IMV_GetFrame` 返回 `IMV_OK` 的那一刻起，帧归 SDK 持有，
    /// **必须恰好释放一次**；而其后到 `releaseFrame()` 之间有内存分配
    /// （复制载荷、OpenCV 建图）⇒ 一旦抛出，原实现会**跳过释放**，
    /// 未释放的帧会被 SDK 内部缓存复用并污染后续取帧。
    ///
    /// 分工（缺一不可）：
    ///   · 正常出口 —— 函数体显式 `cleanup()`，把释放结果 `mergeCleanup`
    ///     进返回值（`GetFrame` 成功而 `ReleaseFrame` 失败 ⇒ 整体失败、不交付帧）；
    ///   · 异常出口 —— **三步各自独立保护**（描述异常／显式清理／写诊断，
    ///     每步内部的失败只损失那一步的产物），末尾的 `throw;` **无条件**
    ///     执行 ⇒ **原异常必定原样上抛**；
    ///   · 析构兜底 —— 只在"显式清理**根本没跑**"时才动手（`armed_` 仍真），
    ///     绝不抛异常。
    ///
    /// ⚠ 兜底的**可达性**不要用"变异全绿"来论证：`M14`（只关掉析构里那次
    ///   释放）全绿只说明**现有用例没覆盖它**。修复前确实存在一个真实窗口
    ///   使其运行 —— `describeCurrentException()` 分配失败时，显式清理
    ///   尚未执行就离开了 catch；本批已把该窗口关闭（三步各自保护），
    ///   但登记结论只能写成"**现有测试未覆盖**"，不能写成"不可达"。
    ///
    /// ⚠ 为什么是嵌套类而不是 .cpp 里的自由类：它要调 `releaseFrame()`
    /// 与 `noteGrabException()`（都是私有），而经 `std::function` 之类的
    /// 间接层会**在守卫构造时引入一次可能失败的分配** ——
    /// 守卫自己没建起来，帧就没人释放了。
    class FrameLeaseGuard
    {
    public:
        explicit FrameLeaseGuard(ImvCameraBackend& owner) : owner_(owner) {}

        /// 析构兜底：仍持有租约即释放。**绝不抛异常**
        /// （栈展开中再抛即 `std::terminate`），也绝不掩盖原异常。
        ~FrameLeaseGuard();

        /// 显式清理（正常出口与 catch 出口都用它）：释放并解除租约，
        /// 返回清理失败（`nullopt` = 成功）。**重复调用只释放一次**。
        ///
        /// ⚠ `noexcept`：本函数在**栈展开中**被调用（catch 出口），
        ///   从这里抛出去会让 `grab()` 尾部的 `throw;` 执行不到，
        ///   于是调用方收到的是一个**二次异常**（例如
        ///   `IMV_ReleaseFrame` 抛出时收到的 `bad_alloc`），原异常**丢失**。
        ///   ∴ 释放调用本身抛出时：如实返回
        ///   `{ImvReleaseFrame, kCallThrewCode}`（"调用抛出异常、无返回码"），
        ///   并**不再重试** —— 那次调用是否已经把缓冲还回去了**无法判断**，
        ///   盲目重试就是"一个帧释放两次"（SDK 内部缓存计数错乱，
        ///   而错误码可能仍是 0 —— 无声的破坏）。
        std::optional<data::SdkFailure> cleanup() noexcept;

    private:
        FrameLeaseGuard(const FrameLeaseGuard&)            = delete;
        FrameLeaseGuard& operator=(const FrameLeaseGuard&) = delete;

        ImvCameraBackend& owner_;
        bool              armed_ = true;
    };

    /// 设置/读取失败的统一出口（`SdkCall` + 原码 → `status`）。
    static data::OperationResult sdkFailure(data::SdkCall call, int code);

    /// 本路本次**确实没有发起 SDK 调用**时的本地失败
    /// （⇒ `sdkError` 必为 `nullopt`，不伪造"调用过"）。
    ///
    /// ⚠ 反向不成立（ENG-09 V2.5 §5.29）：`sdkError == nullopt` 不是
    ///   "本地判定"的判据 —— 判定类别与调用历史是两个问题。若判定所依据的
    ///   数据来自一次**成功**的调用，那条路径**不得**用本助手，必须如实
    ///   保留 `{该调用, IMV_OK}`。
    static data::OperationResult localFailure(data::OpStatus status);

    data::CameraConfig       config_;
    std::shared_ptr<IImvApi> api_;

    /// 设备身份（由枚举与 `IMV_GetDeviceInfo` 的回报填充）。
    /// ⚠ **不**从配置复制目标序列号来填：未取得即如实为"未取得"。
    data::DeviceIdentity deviceIdentity_;

    data::DeviceState state_ = data::DeviceState::UNKNOWN;
    std::string       lastErrorText_;

    /// SDK 设备句柄。保持 `void*` 而不引入 SDK 类型（见文件头）。
    void* handle_ = nullptr;

    /// 句柄已建立（`IMV_CreateHandle` 成功）。
    bool handleCreated_ = false;

    /// 设备已打开（`IMV_Open` 成功）。
    bool deviceOpened_ = false;

    /// 正在取图（`IMV_StartGrabbing` 成功且未 stop）。
    bool grabbing_ = false;

    /// 请求的触发模式（来自配置／上层）。读回值**不缓存**：
    /// 每次 `triggerModeState()` 都**实际读回**，否则"读回"这件事
    /// 会退化成"把请求值抄回去"，而本批要检出的正是
    /// "设了却没生效"这一失效形态。
    data::CameraTriggerMode requestedMode_ = data::CameraTriggerMode::FreeRun;
};

}  // namespace device
}  // namespace aircraft
