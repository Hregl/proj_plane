#pragma once

// ============================================================================
//  src/device/camera/IImvApi.h
//
//  依据：ENG-08 §3 / §11、SYS-06 §7、ENG-09 V2.3 §2.5 / §2.6
//        011-A1 计划 §3.2 第 8 条（内部 SDK 访问层 seam）
//
//  作用：把**对华睿 SDK 的每一次调用**收敛到这一个小接口上，使
//  `ImvCameraBackend` 的状态机（打开 → 配置 → 读回 → 取帧 → 分类 →
//  复制 → 释放）**不直接触碰 SDK 符号**。
//
//  ⚠ 为什么需要这条缝（两条理由，缺一不可）：
//
//   ① **可测**。本批必须验证的几条关键行为——"`GetFrame` 成功而
//      `ReleaseFrame` 失败 ⇒ 整体失败且不交付帧"、"长度溢出时不复制"、
//      "发令失败时取帧次数为 0"、"底层销毁恰好一次"——**本机没有相机**，
//      只能靠替身驱动。若这些逻辑与 `IMV_*` 符号直接耦合，则它们**只在
//      装了 SDK 的构建里**能跑，而无 SDK 构建（本批的必验路径之一）
//      会静默跳过整段逻辑 —— 那正是"验证覆盖随构建配置悄悄变化"。
//
//   ② **不污染头文件**。`ICameraBackend.h`／`ImvCameraBackend.h` 会被
//      device 层其它文件包含，而 SDK 是**可选**依赖（ENG-03 §12.3）：
//      一旦 SDK 缺失，整个 device 模块（含虚拟相机）都编译不了。
//      ∴ 本头文件**只使用 C++ 标准类型**，SDK 类型仅出现在
//      `ImvApiReal.cpp`（由 `#if APS_HAVE_IMVSDK` 二选一）。
//
//  ⚠ 这是**设备层内部 seam**，不进冻结 ICD（ENG-09 §4.3 的类清单里
//  没有它，也不该有）：它描述的是"本适配层怎么访问某个厂商 SDK"，
//  不是系统内任何两个模块之间的契约。
//
//  ⚠ 返回码一律是 **SDK 原始返回码**（`int`，华睿为 −100 段负数），
//  **不做**任何归类 —— 归类是 `ImvCameraBackend` 的职责（§2.2 映射表），
//  若在此处先归一次，两处映射迟早会漂移，而漂移之后"原码"就不可考了。
// ============================================================================

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace aircraft
{
namespace device
{

/// 创建设备句柄的方式（镜像 SDK 的 `IMV_ECreateHandleMode`，但不含 SDK 类型）。
///
/// ⚠ 只列本工程**用到**的一种。`ByCameraKey`／`ByDeviceUserID` 不列：
/// 本工程按**序列号精确匹配**后再用下标建句柄（见 ImvCameraBackend 的
/// 打开流程），不使用"按设备键"这条依赖厂商前缀格式的路径。
enum class ImvHandleMode
{
    ByIndex = 0
};

/// 一帧的**只读视图**：SDK 交付缓冲的裸描述。
///
/// ⚠ 字段与 `IMV_FrameInfo` 一一对应，**语义照抄**，不额外解释：
///   · `size`        —— 整帧字节数（**总长的唯一权威来源**）。
///   · `paddingX/Y`  —— 原样带出为**诊断信息**。其"行内 vs 整帧"语义在
///                      SDK 头文件里**未文档化**，且 SDK 自身的
///                      `Rotate`/`Flip`（按 `width*height*channels`）与
///                      `PixelConvert`（收 `paddingX`）约定**不一致**
///                      ⇒ **禁止**用于任何寻址或长度计算。
///   · 本结构体**没有** `step`/`stride` —— SDK 确实不提供该字段。
///
/// ⚠ `data` 指向**SDK 拥有**的内存，仅在 `releaseFrame()` 之前有效。
/// 这正是"复制必须发生在释放之前"这条契约的物理来源：调用方一旦把
/// 交付帧挂在 `data` 上，释放之后它就指向已被 SDK 复用的缓冲。
struct ImvFrameView
{
    const unsigned char* data      = nullptr;
    uint64_t             blockId   = 0;
    unsigned int         status    = 0;   ///< 0 为正常（照抄 SDK 注释）
    unsigned int         width     = 0;
    unsigned int         height    = 0;
    unsigned int         size      = 0;   ///< 整帧字节数
    int32_t              pixelFormat = 0; ///< SDK 格式码（PFNC 体系）
    uint64_t             timeStamp = 0;
    unsigned int         paddingX  = 0;
    unsigned int         paddingY  = 0;
    unsigned int         recvFrameTime = 0;
};

/// 枚举到的一台设备的身份字段（`IMV_DeviceInfo` 的可见部分）。
struct ImvDeviceEntry
{
    std::string cameraKey;      ///< "厂商:序列号"
    std::string cameraName;     ///< 用户自定义名
    std::string serialNumber;   ///< 设备序列号（**匹配依据**）
    std::string vendorName;
    std::string modelName;      ///< 设备型号
    int32_t     interfaceType = 0;
};

/// SDK 访问层。
///
/// ⚠ 每个方法的返回码都是 SDK 原始码（见文件头）。
/// ⚠ 本接口**不规定线程安全**：与 `ICameraBackend` 一致，由设备线程独占使用。
class IImvApi
{
public:
    virtual ~IImvApi() = default;

    // ---- 枚举与句柄 ----

    /// 枚举设备。返回码非 0 时 `out` 内容未定义（调用方不得使用）。
    virtual int enumDevices(std::vector<ImvDeviceEntry>& out) = 0;

    /// 创建设备句柄。
    /// @param identifier `ByIndex` 时指向一个 `unsigned int` 下标。
    virtual int createHandle(void*& handle, ImvHandleMode mode,
                             void* identifier) = 0;

    virtual int open(void* handle)  = 0;
    virtual int close(void* handle) = 0;

    /// 销毁句柄。⚠ 返回码同样要保留：销毁失败是资源未归还，不能静默。
    virtual int destroyHandle(void* handle) = 0;

    /// 复核设备身份（`IMV_GetDeviceInfo`）。
    virtual int getDeviceInfo(void* handle, ImvDeviceEntry& out) = 0;

    // ---- 特性读写 ----
    // ⚠ 函数名与 SDK 实际函数名一一对应（`IMV_SetEnumFeatureSymbol` 等），
    //    便于日志与 SDK 文档对照；`ImvSetIntFeatureValue` 对应
    //    `IMV_SetIntFeatureValue`，**不是** `IMV_SetIntFeature`（后者不存在）。
    //    「增益」走 double 通道：SDK 没有 `IMV_SetFloatFeatureValue`，
    //    只有 `IMV_SetDoubleFeatureValue`（IMVApi.h:901）。

    virtual int setEnumFeatureSymbol(void* handle, const char* feature,
                                     const char* symbol) = 0;

    /// 读回枚举特性。返回 0 时 `out` 为读到的符号串。
    /// ⚠ 返回 0 但 `out` 为空串**也要**按"没读到"处理（调用方判定）。
    virtual int getEnumFeatureSymbol(void* handle, const char* feature,
                                     std::string& out) = 0;

    virtual int setIntFeatureValue(void* handle, const char* feature,
                                   int64_t value) = 0;
    virtual int getIntFeatureValue(void* handle, const char* feature,
                                   int64_t& out) = 0;

    virtual int setDoubleFeatureValue(void* handle, const char* feature,
                                      double value) = 0;
    virtual int getDoubleFeatureValue(void* handle, const char* feature,
                                      double& out) = 0;

    /// 执行命令特性（软件触发的**唯一入口**：`"TriggerSoftware"`）。
    virtual int executeCommandFeature(void* handle, const char* feature) = 0;

    // ---- 取图 ----

    virtual int startGrabbing(void* handle) = 0;
    virtual int stopGrabbing(void* handle)  = 0;

    /// 取一帧。成功（返回 0）时 `out.data` 指向 SDK 拥有的缓冲，
    /// 必须经 `releaseFrame()` 归还（同一句柄上"最近一次取得的帧"）。
    /// @param timeoutMs 等待上限（ms）。**不得传 0**：SDK 对 0 的语义
    ///        未文档化（见 ImvCameraBackend 的参数校验）。
    virtual int getFrame(void* handle, ImvFrameView& out,
                         unsigned int timeoutMs) = 0;

    /// 归还最近一次 `getFrame()` 取得的帧。
    virtual int releaseFrame(void* handle) = 0;
};

/// 取得**真实** SDK 访问层。
///
/// @return SDK 未接入时返回 `nullptr` —— 调用方据此走"诚实桩"路径
///         （明确失败 + 可操作提示），而**不是**假装成功。
///         ⚠ 返回 `nullptr` 与"返回一个每步都失败的对象"不同：
///         前者让"这台机器上根本没有 SDK"成为**唯一且真实**的事实。
std::shared_ptr<IImvApi> makeRealImvApi();

}  // namespace device
}  // namespace aircraft
