// ============================================================================
//  src/device/camera/ImvCameraBackend.cpp
//
//  依据：SYS-06 §7、ENG-09 V2.3 §2.5 / §2.6 / §4.1 / §4.3 / §5.5 / §6.1
//        裁决 C-01 v1.7
//
//  本文件实现真实相机的**完整通路**：
//      枚举 → 按序列号精确匹配 → 建句柄 → 打开 → 复核身份 →
//      配置并读回（像素格式／曝光／增益／触发）→ 取帧 → 分类 →
//      校验 → 复制 → 释放 → 组装 ImageFrame
//
//  ⚠ 本文件**不出现任何 `IMV_*` 符号**：SDK 调用一律经 `IImvApi`
//  （`ImvApiReal.cpp` 是唯一引用 SDK 的翻译单元）。理由见 IImvApi.h ——
//  核心是"错误映射／复制时机／释放顺序这些必须被验证的逻辑，
//  不能只在装了 SDK 的构建里才存在"。
//
//  ⚠ 本文件**不含** `#if APS_HAVE_IMVSDK`：该宏只决定
//  `makeRealImvApi()` 返回真实对象还是空指针。这正是上面那条纪律的
//  落地方式 —— 若把整段逻辑包进条件编译，则无 SDK 构建（本批的必验
//  路径之一）会**静默跳过**全部错误映射与释放逻辑，而"验证覆盖随
//  构建配置悄悄变化"是最难发现的失效形态。
//
//  ⚠ 与 SDK 相关的三条硬约束（011-A0.1 核验报告，本批照录进 ENG-09）：
//    ① 运行期须 `LD_LIBRARY_PATH` 指向 CMake 生成的 `imvsdk_runtime/`；
//    ② U3V 相机须先设 `usbfs_memory_mb=1000`；
//    ③ SDK 错误码是**负数**（−100 段），**不能**直接当 ENG-09 §5.27 的
//       1000 段应用码返回 —— 这正是 `SdkFailure` 与应用码分开的原因。
// ============================================================================

#include "device/camera/ImvCameraBackend.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "data/MonotonicClock.h"
#include "data/PixelFormat.h"
#include "data/RawImagePayload.h"

namespace aircraft
{
namespace device
{
namespace
{

// ---------------------------------------------------------------------------
//  SDK 原始码（逐个核对 IMVDefines.h，**不是**凭记忆写的）
// ---------------------------------------------------------------------------
constexpr int kImvOk              = 0;      ///< IMVDefines.h:33
constexpr int kImvInvalidParam    = -103;   ///< :36  错误的参数
constexpr int kImvNotSupport      = -113;   ///< :46  设备不支持的功能
constexpr int kImvRestoreStream   = -114;   ///< :47  取图恢复中
constexpr int kImvReconnectDevice = -115;   ///< :48  重连恢复中
constexpr int kImvNotAvailable    = -116;   ///< :49  连接不可达
constexpr int kImvNotGrabbing     = -117;   ///< :50  相机已停止取图
constexpr int kImvNotConnected    = -118;   ///< :51  设备未连接
constexpr int kImvTimeout         = -119;   ///< :52  超时

// ---------------------------------------------------------------------------
//  SDK 像素格式码（逐位取自 IMVDefines.h，与 PFNC 编码体系一致）
// ---------------------------------------------------------------------------
constexpr int32_t kSdkPixelMono8 = 0x01080001;   ///< IMVDefines.h:256
constexpr int32_t kSdkPixelMono12 = 0x01100005;  ///< :260（OCCUPY16BIT）
constexpr int32_t kSdkPixelMono12Packed = 0x010C0006;  ///< :261（OCCUPY12BIT）
constexpr int32_t kSdkPixelBGR8 = 0x02180015;    ///< :293

// ---------------------------------------------------------------------------
//  GenICam 特性名（样例与手头资料已核，逐条标注依据强度）
// ---------------------------------------------------------------------------
const char* const kFeaturePixelFormat = "PixelFormat";
/// ⚠ `"ExposureTime"` 的单位是 **µs**（SFNC 约定），而 `CameraConfig`
/// 的单位是 **s**（ENG-09 §6.1 冻结）⇒ 两处之间必须显式换算，
/// 且换算点只有一处（`configureExposure`）。样例依据：
/// `CommPropAccess.cpp:34/76/85` 对同一特性做 Get/Set/Get。
const char* const kFeatureExposureTime = "ExposureTime";
/// ⚠ `"Gain"` 取 **SFNC 标准特性名**，单位 dB（与 `CameraConfig::gain` 同）。
/// **无本 SDK 样例支持**：全量样例中 `"Gain"` 零命中。
/// ∴ 它是**实机核实项**（登记于 §7 表）。之所以仍然实现而不是跳过：
/// 读回校验会让"特性名不对"在**启动时**就明确失败，而不是静默地
/// 让增益从未生效 —— 后者只有在画面偏暗时才被怀疑。
const char* const kFeatureGain        = "Gain";
const char* const kFeatureTriggerSelector = "TriggerSelector";
const char* const kFeatureTriggerMode     = "TriggerMode";
const char* const kFeatureTriggerSource   = "TriggerSource";
/// 软件触发的**唯一入口**是命令特性 `"TriggerSoftware"`
/// （`IMVApi.h:1067` 的 `IMV_ExecuteCommandFeature`；样例
/// `SoftTrigger.cpp:43`）。SDK 没有 `IMV_TriggerSoftware` 这类符号。
const char* const kFeatureTriggerSoftware = "TriggerSoftware";

/// 各样例一致地先设 `TriggerSelector = "FrameStart"`，再设模式
/// （`SoftTrigger.cpp:88`、`LineTrigger.cpp:55`、`ClearFrameBuffer.cpp:190`、
/// `ChunkData.cpp:115`）—— 跳过选择器会让设置落到别的选择器上，
/// 表现为"设了却不起作用"，而那是本项目最警惕的失效形态。
const char* const kTriggerSelectorFrameStart = "FrameStart";

/// 请求的像素格式（GenICam 符号名）。
///
/// ⚠ 本批**不设配置键**：`camera.yaml` 的冻结键表（SYS-17 §5）里没有
/// 像素格式，而 ENG-09 §6.1 的 `CameraConfig` 字段清单亦无此项，
/// 故不擅自新增键（那会让"配置有什么"出现第二个事实来源）。
/// 固定请求 `Mono12` 的理由正是 A1 的存在理由：真实相机交付 12 位
/// 原始载荷，而 8U 显示图只能由它派生；若请求 8 位，则"RAW 保真"
/// 这一整条契约在真机上**无从成立**。
/// ⚠ 配置化（含 `Mono8` 的显式选择）随 ENG-09 §6.1 键表扩展另批登记。
/// ⚠ 符号名 `"Mono12"` 取 SFNC 标准名，**无本 SDK 样例支持**
/// （样例从不设 `PixelFormat`）⇒ 属实机核实项；读回校验保证
/// "名字不对"在启动时即失败。
const char* const kRequestedPixelFormatSymbol = "Mono12";

/// 把 SDK 返回码归类为 `OpStatus`（ENG-09 V2.3 §5.29 的状态全表）。
///
/// ⚠ **只按码自身的文档化含义归类**，证据不足的一律 `SdkError` + 原码保留：
///   · `-114`／`-115`／`-116` —— 注释是"取图恢复中／重连恢复中／连接不可达"，
///     **没说断连**，也没说调用方该做什么 ⇒ **不得**归 `Disconnected`。
///     （"不可达"≠"已断连"：一次网络抖动也表现为不可达，而断连要禁用通道。）
///   · `-122` 调用时序错误 —— 不等于"未就绪"，归 `NotStarted` 属推断。
///   · `-119` 超时 —— 仅在 `IMV_GetFrame` 上是 `Timeout`：该常量自身的
///     注释只说"超时"，把它与某个具体函数挂钩是**推断**（该函数的注释
///     并未说明它返回 −119，且 11 处样例**无一处**判 `IMV_TIMEOUT`）。
///     ∴ 这是实机核实项；核实若否，只改这张表。
data::OpStatus classify(data::SdkCall call, int code)
{
    switch (code)
    {
    case kImvOk:
        return data::OpStatus::Ok;
    case kImvTimeout:
        return call == data::SdkCall::ImvGetFrame ? data::OpStatus::Timeout
                                                 : data::OpStatus::SdkError;
    case kImvNotGrabbing:
        return data::OpStatus::NotStarted;
    case kImvNotConnected:
        return data::OpStatus::Disconnected;
    case kImvNotSupport:
        return data::OpStatus::NotImplemented;
    case kImvInvalidParam:
        return data::OpStatus::InvalidArgument;
    case kImvRestoreStream:
    case kImvReconnectDevice:
    case kImvNotAvailable:
        // 见上：注释不足以支撑"断连"这一结论。
        return data::OpStatus::SdkError;
    default:
        // 其余码（−101／−102／−104~−112／−122~−130 等）无"调用方该如何
        // 处置"的依据 ⇒ 原码保留，交由人看。
        return data::OpStatus::SdkError;
    }
}

/// `OperationResult` → `GrabResult`（两者只差在多一层类型，字段完全相同）。
///
/// ⚠ 存在的理由：`grab()` 需要 `GrabResult`，而 `sdkFailure()` 产出的通用结果
/// 类型是 `OperationResult`。**不**为此把 `sdkFailure` 复制一份返回
/// `GrabResult` —— 那会让"操作名 + 原码 → 状态"这个映射出现第二个实现点，
/// 而 §2.2 的映射表只有一处才是它可以被核对的前提。
data::GrabResult asGrab(const data::OperationResult& r)
{
    return data::GrabResult{{r.status, r.sdkError, r.cleanupError}};
}

/// SDK 格式码 → 项目定义的格式。
/// @return false 表示本批**不认识**该码（未知格式 ⇒ 明确失败，不回落、不猜）。
bool mapSdkPixelFormat(int32_t code, data::PixelFormat& out)
{
    switch (code)
    {
    case kSdkPixelMono8:
        out = data::PixelFormat::Mono8;
        return true;
    case kSdkPixelMono12:
        out = data::PixelFormat::Mono12;
        return true;
    case kSdkPixelMono12Packed:
        out = data::PixelFormat::Mono12Packed;
        return true;
    case kSdkPixelBGR8:
        out = data::PixelFormat::BGR8;
        return true;
    default:
        return false;
    }
}

/// 生成 8U 显示图（SYS-04 §6.1 第 6 条：`8U = 像素值 >> (validBits − 8)`）。
///
/// ⚠ **不缩放、不直方图拉伸、不饱和** —— 本转换只做右移，故同一载荷
/// 在任何时候都得到同一张图。若将来有人想"提亮一点"，那是显示层的
/// 事，不是这里：把它混进来的后果是"结果包里的显示图与原始载荷的
/// 关系不可复现"，而离线复核正是靠这个关系。
///
/// ⚠ **不委派 `IMV_PixelConvert`**（尽管样例一律走它）：它会把
/// 缩放／对齐规则藏进 SDK 内部，而我们无法核实其是否缩放 ——
/// 直接冲突于上面那条"不缩放"的冻结要求。对照比对（同一帧两法出图）
/// 登记为实机核实项。
///
/// ⚠ 16 位容器按 `declaredByteOrder` 组装。本批只声明 `LittleEndian`
/// （依据与核实方式见 ENG-09 V2.3 §5.28 第 4 条），`BigEndian` 由调用方**拒绝**。
bool buildDisplayImage(const std::vector<uint8_t>& bytes,
                       data::PixelFormat          format,
                       uint32_t                   width,
                       uint32_t                   height,
                       cv::Mat&                   out)
{
    const int w = static_cast<int>(width);
    const int h = static_cast<int>(height);

    switch (format)
    {
    case data::PixelFormat::Mono8:
    {
        // 8 位整字节：直接按行连续解释，再 clone 成自有内存。
        cv::Mat view(h, w, CV_8UC1, const_cast<uint8_t*>(bytes.data()));
        out = view.clone();
        return true;
    }
    case data::PixelFormat::Mono12:
    {
        // 低位对齐、高位补零（PFNC 2.4 §6.1.1 / 图 6-3），16 位容器。
        // 右移 4 位取高 8 位作为显示值 —— 这是**唯一**的转换规则。
        cv::Mat img(h, w, CV_8UC1);
        for (int y = 0; y < h; ++y)
        {
            uint8_t* row = img.ptr<uint8_t>(y);
            for (int x = 0; x < w; ++x)
            {
                const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 2;
                // 小端：低字节在前。`declaredByteOrder` 为 BigEndian 的
                // 情况在调用方已被拒绝，故此处的组装顺序与声明一致。
                const uint16_t v = static_cast<uint16_t>(bytes[i]) |
                                   static_cast<uint16_t>(bytes[i + 1] << 8);
                row[x] = static_cast<uint8_t>(v >> (12 - 8));
            }
        }
        out = std::move(img);
        return true;
    }
    case data::PixelFormat::BGR8:
    {
        cv::Mat view(h, w, CV_8UC3, const_cast<uint8_t*>(bytes.data()));
        out = view.clone();
        return true;
    }
    case data::PixelFormat::Mono12Packed:
        // 本批按范围不做（其格式码 0x010C0006 属 GigE Vision 2.0，
        // 与 PFNC 的 Mono12p = 0x010C0047 是两项，不能套用同一套打包规则）。
        return false;
    }
    return false;
}

}  // namespace

ImvCameraBackend::ImvCameraBackend(const data::CameraConfig& config,
                                  std::shared_ptr<IImvApi>    api)
    : config_(config), api_(api ? std::move(api) : makeRealImvApi())
{
    requestedMode_ = config_.triggerMode;
}

ImvCameraBackend::~ImvCameraBackend()
{
    // 兜底关闭（幂等）。见 ICameraBackend 的析构说明：拥有者正常关闭时
    // 会显式调 close()，但"启动中途失败"与异常路径会绕过它，
    // 而句柄一旦泄漏，同一台设备在本次进程内再也打不开。
    (void)closeDevice();
}

// ---------------------------------------------------------------------------
//  失败出口
// ---------------------------------------------------------------------------

data::OperationResult ImvCameraBackend::sdkFailure(data::SdkCall call, int code)
{
    data::OperationResult r;
    r.status   = classify(call, code);
    r.sdkError = data::SdkFailure{call, static_cast<int32_t>(code)};
    return r;
}

data::OperationResult ImvCameraBackend::localFailure(data::OpStatus status)
{
    // ⚠ `sdkError` 为 `nullopt`：本函数只用于**没有调用 SDK** 的情形。
    // 伪造一个"调用过"会让离线排查去找一个不存在的 SDK 返回码。
    data::OperationResult r;
    r.status = status;
    return r;
}

// ---------------------------------------------------------------------------
//  打开
// ---------------------------------------------------------------------------

data::OperationResult ImvCameraBackend::initialize()
{
    if (deviceOpened_)
    {
        // 已初始化：返回 Ok 而**不重复打开**。重复打开会让后续所有
        // 配置写在一个新句柄上，而旧句柄（连同其已生效的参数）被丢弃 ——
        // 表现为"配置看起来成功了，相机却在按旧参数跑"。
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    if (!api_)
    {
        // 未接入 SDK：明确失败 + 可操作提示，**绝不**假装成功。
        // ⚠ 措辞只针对"没有 SDK"这一种情形。此处**不再**有第二种文本
        // （011-A0 曾要求区分"未找到 SDK"与"找到但适配未实现"）：
        // 后者描述的状态**已不存在**（适配本批已实现），保留一句
        // 永远不可达的提示会让读者以为还有一条未走通的路。
        state_ = data::DeviceState::ERROR;
        lastErrorText_ =
            "ImvCameraBackend 未接入 SDK：当前构建未找到华睿 SDK。"
            "请用 -DIMV_SDK_ROOT=<SDK 根目录> 重新配置，"
            "或在软件闭环阶段使用 VirtualCameraBackend（ENG-08 §3）。";
        return localFailure(data::OpStatus::NotImplemented);
    }

    state_ = data::DeviceState::INIT;
    const data::OperationResult r = openDevice();
    if (!r.ok())
    {
        // 状态：只有"设备不在"才置 DISCONNECTED（见 DeviceState.h 的纪律）；
        // 其余失败是本地状态而非设备不在，置 ERROR。
        state_ = (r.status == data::OpStatus::Disconnected)
                     ? data::DeviceState::DISCONNECTED
                     : data::DeviceState::ERROR;
        return r;
    }

    state_ = data::DeviceState::READY;
    return r;
}

data::OperationResult ImvCameraBackend::openDevice()
{
    // ---- 1. 序列号必须已绑定 ------------------------------------------------
    if (config_.serialNumber.empty())
    {
        // 不取"第 0 个设备"：三台相机是同型号，取第 0 个会让
        // 25/50/100 三路随机互换，而互换之后**没有任何错误**，
        // 只表现为标定对不上、角度系统性偏差。
        lastErrorText_ =
            "配置未绑定设备序列号（camera.yaml 的 serial 为空）："
            "拒绝打开设备 —— 按序号打开会在同型号设备间随机互换通道。"
            "请在 camera.yaml 中为该通道填写设备标签上的序列号。";
        return localFailure(data::OpStatus::InvalidArgument);
    }

    // ---- 2. 枚举 ------------------------------------------------------------
    std::vector<ImvDeviceEntry> devices;
    int ret = api_->enumDevices(devices);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("枚举相机失败：") +
                         data::sdkCallName(data::SdkCall::ImvEnumDevices) +
                         " 返回 " + std::to_string(ret);
        return sdkFailure(data::SdkCall::ImvEnumDevices, ret);
    }

    // ---- 3. 按序列号**精确**匹配 -------------------------------------------
    std::size_t matchIndex  = 0;
    int         matchCount  = 0;
    for (std::size_t i = 0; i < devices.size(); ++i)
    {
        if (devices[i].serialNumber == config_.serialNumber)
        {
            matchIndex = i;
            ++matchCount;
        }
    }

    std::string enumerated;
    for (const auto& d : devices)
    {
        enumerated += "\n  - 序列号=\"" + d.serialNumber + "\" 型号=\"" +
                      d.modelName + "\" 厂商=\"" + d.vendorName + "\" 键=\"" +
                      d.cameraKey + "\"";
    }
    if (devices.empty())
    {
        enumerated = "\n  （未枚举到任何设备）";
    }

    if (matchCount == 0)
    {
        // ⚠ 这是**直接观察到的事实**（枚举结果里没有它），不是从错误码
        // 推断出来的断开。∴ 归 `Disconnected` 是如实的；同时**不**带
        // `sdkError`（SDK 调用本身成功了）。
        // ⚠ 措辞纪律同样适用：这里说的是"不在枚举结果里"，
        // **不写**"已确认断连"（本批不注册连接事件回调）。
        lastErrorText_ = "未找到序列号为 \"" + config_.serialNumber +
                         "\" 的相机（通道 " + config_.cameraId +
                         "）。已枚举到 " + std::to_string(devices.size()) +
                         " 台设备：" + enumerated;
        return localFailure(data::OpStatus::Disconnected);
    }
    if (matchCount > 1)
    {
        // 同一序列号出现多次：设备侧或枚举结果异常。**不猜**哪一台是对的。
        // 归 `ContractViolation`（本地契约：我们假定序列号唯一），
        // 且 `sdkError` 为空 —— 没有任何 SDK 调用失败。
        lastErrorText_ = "序列号 \"" + config_.serialNumber +
                         "\" 匹配到 " + std::to_string(matchCount) +
                         " 台设备，无法确定绑定对象：" + enumerated;
        return localFailure(data::OpStatus::ContractViolation);
    }

    // ---- 4. 建句柄 ----------------------------------------------------------
    // ⚠ 用 `modeByIndex` + 匹配到的下标，而**不是** `modeByCameraKey`：
    // 后者要求拼出 SDK 的 `cameraKey`（"厂商:序列号"），即依赖厂商
    // 前缀的书写格式 —— 而该格式不在我们的控制之下，换一个厂商或
    // 一台摄像机名不同的设备就要改代码。用下标则只依赖"枚举顺序在
    // 本次枚举内稳定"这一件事，而它由同一次 `IMV_EnumDevices` 保证。
    unsigned int index = static_cast<unsigned int>(matchIndex);
    ret                = api_->createHandle(handle_, ImvHandleMode::ByIndex,
                                          static_cast<void*>(&index));
    if (ret != kImvOk)
    {
        handle_ = nullptr;
        lastErrorText_ = std::string("创建设备句柄失败：") +
                         data::sdkCallName(data::SdkCall::ImvCreateHandle) +
                         " 返回 " + std::to_string(ret);
        return sdkFailure(data::SdkCall::ImvCreateHandle, ret);
    }
    handleCreated_ = true;

    // ---- 5. 打开（失败即自清理：半初始化不许留在句柄上）--------------------
    ret = api_->open(handle_);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("打开相机失败：") +
                         data::sdkCallName(data::SdkCall::ImvOpen) + " 返回 " +
                         std::to_string(ret);
        const data::OperationResult r = sdkFailure(data::SdkCall::ImvOpen, ret);
        (void)closeDevice();   // 自清理，见 ICameraBackend::initialize 的契约
        return r;
    }
    deviceOpened_ = true;

    // ---- 6. 复核设备身份 ----------------------------------------------------
    ImvDeviceEntry info;
    ret = api_->getDeviceInfo(handle_, info);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("读取设备信息失败：") +
                         data::sdkCallName(data::SdkCall::ImvGetDeviceInfo) +
                         " 返回 " + std::to_string(ret);
        const data::OperationResult r =
            sdkFailure(data::SdkCall::ImvGetDeviceInfo, ret);
        (void)closeDevice();
        return r;
    }

    // ⚠ 身份只填**设备回报**，不从配置复制期望值：把配置里的目标序列号
    // 抄进"实际身份"会让"读不到序列号"伪装成"序列号与配置一致"，
    // 而设备身份绑定就此失去意义。
    deviceIdentity_.queried      = true;
    deviceIdentity_.modelName    = info.modelName;
    deviceIdentity_.serialNumber = info.serialNumber.empty()
                                       ? std::optional<std::string>{}
                                       : std::optional<std::string>(info.serialNumber);

    if (info.serialNumber != config_.serialNumber)
    {
        // 打开之后复核：句柄指向的设备必须仍是配置指定的那一台。
        // 不复核则"枚举与建句柄之间设备变化"这类情况无人发现。
        lastErrorText_ = "打开后复核身份不符：配置期望序列号 \"" +
                         config_.serialNumber + "\"，设备回报 \"" +
                         info.serialNumber + "\"（型号 \"" + info.modelName +
                         "\"）。已放弃该句柄。";
        const data::OperationResult r = localFailure(data::OpStatus::ContractViolation);
        (void)closeDevice();
        return r;
    }

    // ---- 7. 配置并读回 ------------------------------------------------------
    // 像素格式：固定请求 Mono12（见 kRequestedPixelFormatSymbol 的说明）。
    data::OperationResult cfg = configureEnumAndVerify(
        kFeaturePixelFormat, kRequestedPixelFormatSymbol,
        data::SdkCall::ImvSetEnumFeatureSymbol,
        data::SdkCall::ImvGetEnumFeatureSymbol);
    if (!cfg.ok())
    {
        (void)closeDevice();
        return cfg;
    }

    cfg = configureExposure();
    if (!cfg.ok())
    {
        (void)closeDevice();
        return cfg;
    }

    cfg = configureGain();
    if (!cfg.ok())
    {
        (void)closeDevice();
        return cfg;
    }

    cfg = applyTriggerMode(requestedMode_);
    if (!cfg.ok())
    {
        (void)closeDevice();
        return cfg;
    }

    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

// ---------------------------------------------------------------------------
//  配置
// ---------------------------------------------------------------------------

data::OperationResult ImvCameraBackend::configureEnumAndVerify(
    const char*    feature,
    const char*    expect,
    data::SdkCall  setCall,
    data::SdkCall  getCall)
{
    int ret = api_->setEnumFeatureSymbol(handle_, feature, expect);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("配置特性 \"") + feature + "\" = \"" +
                         expect + "\" 失败（" + data::sdkCallName(setCall) +
                         " 返回 " + std::to_string(ret) + "）";
        return sdkFailure(setCall, ret);
    }

    std::string readback;
    ret = api_->getEnumFeatureSymbol(handle_, feature, readback);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("读回特性 \"") + feature + "\" 失败（" +
                         data::sdkCallName(getCall) + " 返回 " +
                         std::to_string(ret) + "）";
        return sdkFailure(getCall, ret);
    }

    if (readback != expect)
    {
        // ⚠ 这一条是"设了却没生效"的**唯一**探测器。若只 set 不 get，
        // 一次静默失效会一路走到测量结果里，而现场只会怀疑镜头或标定。
        // 归 `ContractViolation`：SDK 调用都成功了，是**设备状态与我们的
        // 期望不一致**。
        lastErrorText_ = std::string("特性 \"") + feature + "\" 读回值 \"" +
                         readback + "\" 与请求值 \"" + expect +
                         "\" 不一致：设置未生效，拒绝继续（不静默接受）。";
        return localFailure(data::OpStatus::ContractViolation);
    }

    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::configureExposure()
{
    // ⚠ 单位换算集中在这一处：配置是 **s**（ENG-09 §6.1 冻结），
    // SDK 的 `ExposureTime` 是 **µs**（SFNC 约定）。两处之间若漏了
    // 换算，相机不会报错，只会按 5 µs（= 5e-6 s 的数值当 µs 用）
    // 曝光，画面全黑 —— 而黑图的特征提取失败会被归因为"目标太远"。
    const double sdkExposureUs = config_.exposureTime * 1e6;

    int ret = api_->setDoubleFeatureValue(handle_, kFeatureExposureTime,
                                        sdkExposureUs);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("配置曝光失败（") +
                         data::sdkCallName(data::SdkCall::ImvSetDoubleFeatureValue) +
                         " 返回 " + std::to_string(ret) + "）";
        return sdkFailure(data::SdkCall::ImvSetDoubleFeatureValue, ret);
    }

    double readbackUs = 0.0;
    ret = api_->getDoubleFeatureValue(handle_, kFeatureExposureTime, readbackUs);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("读回曝光失败（") +
                         data::sdkCallName(data::SdkCall::ImvGetDoubleFeatureValue) +
                         " 返回 " + std::to_string(ret) + "）";
        return sdkFailure(data::SdkCall::ImvGetDoubleFeatureValue, ret);
    }

    // 容差 1 µs（换算回 s 后即 1e-6）：设备的曝光寄存器量化步长通常是
    // 微秒级，因此**不能**要求逐位相等。本校验的目的是检出"设置未生效"，
    // 不是追究量化误差 —— 把容差设成 0 只会让每台设备都在启动时失败。
    const double readbackS = readbackUs / 1e6;
    if (std::fabs(readbackS - config_.exposureTime) > 1e-6)
    {
        lastErrorText_ = "曝光读回不一致：请求 " +
                         std::to_string(config_.exposureTime) + " s（= " +
                         std::to_string(sdkExposureUs) + " µs），设备回报 " +
                         std::to_string(readbackS) + " s。设置未生效。";
        return localFailure(data::OpStatus::ContractViolation);
    }

    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::configureGain()
{
    int ret = api_->setDoubleFeatureValue(handle_, kFeatureGain, config_.gain);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("配置增益失败（特性 \"") + kFeatureGain +
                         "\"，" +
                         data::sdkCallName(data::SdkCall::ImvSetDoubleFeatureValue) +
                         " 返回 " + std::to_string(ret) +
                         "）。注意该特性名无本 SDK 样例支持，可能是名称不符。";
        return sdkFailure(data::SdkCall::ImvSetDoubleFeatureValue, ret);
    }

    double readback = 0.0;
    ret = api_->getDoubleFeatureValue(handle_, kFeatureGain, readback);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("读回增益失败（特性 \"") + kFeatureGain +
                         "\"，" +
                         data::sdkCallName(data::SdkCall::ImvGetDoubleFeatureValue) +
                         " 返回 " + std::to_string(ret) + "）";
        return sdkFailure(data::SdkCall::ImvGetDoubleFeatureValue, ret);
    }

    // 容差 0.05 dB：设备的增益步长通常为 0.1 dB 量级。
    if (std::fabs(readback - config_.gain) > 0.05)
    {
        lastErrorText_ = "增益读回不一致：请求 " + std::to_string(config_.gain) +
                         " dB，设备回报 " + std::to_string(readback) +
                         " dB。设置未生效。";
        return localFailure(data::OpStatus::ContractViolation);
    }

    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::applyTriggerMode(
    data::CameraTriggerMode mode)
{
    switch (mode)
    {
    case data::CameraTriggerMode::FreeRun:
        // ⚠ 本批**不实现**自由运行：SDK 全树没有 "FreeRun" 这个词，
        // "`TriggerMode = "Off"` 即自由运行"这一语义**只有样例代码
        // 支持、无文档保证**（`ClearFrameBuffer.cpp:201` 设 "Off"）。
        // ∴ 明确返回 `NotImplemented`，**不静默当成自由运行** ——
        // 后者会让"以为在等硬触发、其实相机在自己跑"成为无声的事实。
        lastErrorText_ =
            "触发模式 FreeRun 未实现（011-A1）：SDK 无 \"FreeRun\" 这一特性取值，"
            "\"TriggerMode=Off 即自由运行\" 仅有样例支持、无文档保证。"
            "请在 camera.yaml 中改用 software（硬件触发归 PH-01）。";
        return localFailure(data::OpStatus::NotImplemented);

    case data::CameraTriggerMode::Hardware:
        // 硬触发的符号映射是已知的（TriggerSource="Line1" 另需
        // TriggerActivation="RisingEdge"，`LineTrigger.cpp:73`），
        // 但本批按范围不实施（PH-01），且**本批不验证**其真机行为。
        lastErrorText_ =
            "触发模式 Hardware 未实现（011-A1）：硬件触发归 PH-01，"
            "本批不实施（符号映射已记入 ENG-09，以免 PH-01 时重新发现）。"
            "请在 camera.yaml 中改用 software。";
        return localFailure(data::OpStatus::NotImplemented);

    case data::CameraTriggerMode::Software:
        break;
    }

    // ⚠ 顺序：`TriggerSelector` → `TriggerSource` → `TriggerMode`。
    // 先设选择器的理由：`TriggerSource` 是**针对当前选中的选择器**的；
    // 若先设源再设选择器，源可能落到切换前的那个选择器上。
    // ⚠ 与样例的差异如实记录：4 份样例中有 3 份先设 `TriggerSource`
    // 再设 `TriggerSelector`（`SoftTrigger.cpp:76/88`、
    // `LineTrigger.cpp:46/55`、`ChunkData.cpp:106/115`），
    // 而 `ClearFrameBuffer.cpp:190-221` 更谨慎：先设选择器，再把
    // `TriggerMode` 置 "Off"，改完源再置 "On"。本批采用"先选择器"
    // 这一更保险的顺序，但**不**加 Mode Off→On 那一步（本批不在
    // 运行中切换触发配置，故那一步的收益不成立）。若实机出现
    // "设了不生效"，第一件事就是按 `ClearFrameBuffer` 的写法
    // 补上 Mode Off→On —— 该对照已登记为实机核实项。
    data::OperationResult r = configureEnumAndVerify(
        kFeatureTriggerSelector, kTriggerSelectorFrameStart,
        data::SdkCall::ImvSetEnumFeatureSymbol,
        data::SdkCall::ImvGetEnumFeatureSymbol);
    if (!r.ok())
    {
        return r;
    }

    r = configureEnumAndVerify(kFeatureTriggerSource,
                               data::triggerSourceSymbol(
                                   data::CameraTriggerMode::Software),
                               data::SdkCall::ImvSetEnumFeatureSymbol,
                               data::SdkCall::ImvGetEnumFeatureSymbol);
    if (!r.ok())
    {
        return r;
    }

    r = configureEnumAndVerify(kFeatureTriggerMode,
                              data::triggerModeSymbol(
                                  data::CameraTriggerMode::Software),
                              data::SdkCall::ImvSetEnumFeatureSymbol,
                              data::SdkCall::ImvGetEnumFeatureSymbol);
    if (!r.ok())
    {
        return r;
    }

    requestedMode_ = data::CameraTriggerMode::Software;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

// ---------------------------------------------------------------------------
//  启动 / 停止 / 关闭
// ---------------------------------------------------------------------------

data::OperationResult ImvCameraBackend::start()
{
    if (!deviceOpened_)
    {
        // 未 initialize 就 start：拒绝，而不是隐式初始化。
        // 隐式初始化会让"忘记调 initialize()"这一缺陷不可见，
        // 而它在真实后端上表现为 SDK 未加载即操作设备。
        // ⚠ `sdkError` 为空：**没有调用任何 SDK**。
        lastErrorText_ = "未成功 initialize 即调用 start()：拒绝（不做隐式初始化）。";
        return localFailure(data::OpStatus::NotStarted);
    }

    if (grabbing_)
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    const int ret = api_->startGrabbing(handle_);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("开始取图失败：") +
                         data::sdkCallName(data::SdkCall::ImvStartGrabbing) +
                         " 返回 " + std::to_string(ret);
        return sdkFailure(data::SdkCall::ImvStartGrabbing, ret);
    }

    grabbing_ = true;
    state_    = data::DeviceState::RUNNING;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::stop()
{
    // 契约：允许在未 start() 时调用、须无副作用（ICameraBackend）。
    // ⚠ 未打开设备时**不调用任何 SDK**：此时连句柄都没有，
    //    对空句柄调 StopGrabbing 只会拿到一个无意义的错误码。
    if (!deviceOpened_ || !grabbing_)
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    const int ret = api_->stopGrabbing(handle_);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("停止取图失败：") +
                         data::sdkCallName(data::SdkCall::ImvStopGrabbing) +
                         " 返回 " + std::to_string(ret);
        // 停流失败 ⇒ 仍在取图，`grabbing_` 保持真（不谎报已停）。
        return sdkFailure(data::SdkCall::ImvStopGrabbing, ret);
    }

    grabbing_ = false;
    if (state_ == data::DeviceState::RUNNING)
    {
        state_ = data::DeviceState::READY;
    }
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::closeDevice()
{
    // 幂等：未打开也无句柄 ⇒ 直接 Ok，**不调用任何 SDK**。
    if (handle_ == nullptr)
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    data::OperationResult result{data::OpStatus::Ok, std::nullopt, std::nullopt};

    // 记录一次失败：**首次**失败进入 `sdkError`（保留首因），
    // 其后进入 `cleanupError`（各自留痕，不覆盖首因）。
    const auto record = [&result](data::SdkCall call, int code) {
        data::SdkFailure f{call, static_cast<int32_t>(code)};
        if (result.ok())
        {
            result.status   = classify(call, code);
            result.sdkError = f;
        }
        else if (!result.cleanupError.has_value())
        {
            result.cleanupError = f;
        }
    };

    // 1. 停流
    if (grabbing_)
    {
        const int ret = api_->stopGrabbing(handle_);
        if (ret != kImvOk)
        {
            record(data::SdkCall::ImvStopGrabbing, ret);
        }
        grabbing_ = false;
    }

    // 2. 关设备
    if (deviceOpened_)
    {
        const int ret = api_->close(handle_);
        if (ret != kImvOk)
        {
            record(data::SdkCall::ImvClose, ret);
        }
        deviceOpened_ = false;
    }

    // 3. 销毁句柄
    if (handleCreated_)
    {
        const int ret = api_->destroyHandle(handle_);
        if (ret != kImvOk)
        {
            record(data::SdkCall::ImvDestroyHandle, ret);
            // ⚠ 销毁失败 ⇒ **不清空 `handle_`**：句柄仍存在（已泄漏但可重试），
            //    清掉它会让"再调一次 close()"无从重试，而泄漏是永久的。
        }
        else
        {
            handle_        = nullptr;
            handleCreated_ = false;
        }
    }

    if (!result.ok())
    {
        lastErrorText_ = "关闭相机时有未完成的步骤（首因：" +
                         std::string(data::sdkCallName(result.sdkError->call)) +
                         " 返回 " +
                         std::to_string(result.sdkError->code) + "）";
    }

    state_ = (handle_ == nullptr) ? data::DeviceState::UNKNOWN
                                 : data::DeviceState::ERROR;
    return result;
}

data::OperationResult ImvCameraBackend::close()
{
    return closeDevice();
}

// ---------------------------------------------------------------------------
//  触发
// ---------------------------------------------------------------------------

data::OperationResult ImvCameraBackend::triggerSoftware()
{
    if (!deviceOpened_)
    {
        lastErrorText_ = "未打开设备即调用 triggerSoftware()：拒绝。";
        return localFailure(data::OpStatus::NotStarted);
    }

    // ⚠ 本方法**不做模式判定**，尽管"模式不是 Software 时发令无意义"。
    // 理由：§2.4 冻结了**唯一执行者**是 `MultiCameraManager::capture()`，
    // 它已按读回模式判定是否发令。后端再判一次，会让"该不该发令"
    // 出现两个判断点 —— 而两个判断点迟早不一致，那正是 §2.4 要消除的形态。
    // 误用（在非 Software 模式下发令）会得到设备的错误码并被归为
    // `SdkError`，从而**响亮地**暴露，不会被静默吞掉。
    const int ret = api_->executeCommandFeature(handle_, kFeatureTriggerSoftware);
    if (ret != kImvOk)
    {
        lastErrorText_ = std::string("软件触发失败：") +
                         data::sdkCallName(data::SdkCall::ImvExecuteCommandFeature) +
                         " 返回 " + std::to_string(ret);
        return sdkFailure(data::SdkCall::ImvExecuteCommandFeature, ret);
    }

    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult ImvCameraBackend::setTriggerMode(
    data::CameraTriggerMode mode)
{
    requestedMode_ = mode;

    if (!deviceOpened_)
    {
        lastErrorText_ = "未打开设备即调用 setTriggerMode()：拒绝。";
        return localFailure(data::OpStatus::NotStarted);
    }

    return applyTriggerMode(mode);
}

data::TriggerModeState ImvCameraBackend::triggerModeState() const
{
    data::TriggerModeState s;
    s.requested = requestedMode_;

    if (!deviceOpened_ || !api_)
    {
        // 读不到就是读不到：三项留空、`reported` 为 nullopt、
        // `consistent = false`。**不得**把未打开默认成某个合法模式。
        return data::composeTriggerModeState(s);
    }

    // 三项分别读回。任一项读失败或返回空串 ⇒ 保留 nullopt
    // （"读不到"与"读到了某个值"必须可区分）。
    std::string v;
    if (api_->getEnumFeatureSymbol(handle_, kFeatureTriggerSelector, v) == kImvOk &&
        !v.empty())
    {
        s.selectorReported = v;
    }

    v.clear();
    if (api_->getEnumFeatureSymbol(handle_, kFeatureTriggerMode, v) == kImvOk &&
        !v.empty())
    {
        s.switchReported = v;
    }

    v.clear();
    if (api_->getEnumFeatureSymbol(handle_, kFeatureTriggerSource, v) == kImvOk &&
        !v.empty())
    {
        s.sourceReported = v;
    }

    return data::composeTriggerModeState(s);
}

// ---------------------------------------------------------------------------
//  取帧
// ---------------------------------------------------------------------------

std::optional<data::SdkFailure> ImvCameraBackend::releaseFrame()
{
    const int ret = api_->releaseFrame(handle_);
    if (ret == kImvOk)
    {
        return std::nullopt;
    }
    return data::SdkFailure{data::SdkCall::ImvReleaseFrame,
                            static_cast<int32_t>(ret)};
}

void ImvCameraBackend::mergeCleanup(
    data::GrabResult&                     result,
    const std::optional<data::SdkFailure>& cleanup)
{
    if (!cleanup.has_value())
    {
        return;
    }

    if (result.ok())
    {
        // ② 主操作成功、而必要清理失败 ⇒ **整体失败**，以该清理调用为
        //    错误来源。把"资源没还回去"说成成功是错的（§2.2 修正 4）：
        //    未释放的帧会污染后续 `IMV_GetFrame`（SDK 用内部缓存），
        //    故这**不是**可以留到下一次再说的收尾问题。
        result.status   = classify(cleanup->call, cleanup->code);
        result.sdkError = cleanup;
    }
    else
    {
        // ① 已有主失败 ⇒ 保留首因，清理失败单独留存。
        //    覆盖首因会让"本来因为什么失败"永久丢失，而清理失败
        //    往往正是首因的**后果**（设备已经没了，当然也释放不了）。
        result.cleanupError = cleanup;
    }
}

data::GrabResult ImvCameraBackend::grab(data::ImageFrame& frame,
                                       uint32_t         timeoutMs)
{
    // ---- 参数校验：不允许 0 -------------------------------------------------
    // SDK 对 `timeoutMS = 0` 的语义**未文档化**（`IMVApi.h:612-633` 的参数
    // 注释只提 `INFINITE`，而 `INFINITE` 宏在 SDK 里**未定义**），
    // 故本项目也不定义它：拒绝，**不**静默当成"无限等待"。
    // ⚠ 本地判定 ⇒ `sdkError` 为空。
    if (timeoutMs == 0)
    {
        lastErrorText_ =
            "grab() 的 timeoutMs 为 0：本项目不接受该取值（SDK 未文档化其语义，"
            "且 INFINITE 宏在 SDK 中未定义）。请由上层给出正的等待上限。";
        return data::GrabResult{
            {data::OpStatus::InvalidArgument, std::nullopt, std::nullopt}};
    }

    // ---- 前置状态：未打开/未启动 ⇒ 本地 NotStarted（不调用 SDK）-------------
    if (!deviceOpened_)
    {
        lastErrorText_ = "未打开设备即调用 grab()：拒绝。";
        return data::GrabResult{
            {data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
    }
    if (!grabbing_)
    {
        // 与"设备不在"区分开：这里设备在、句柄在，只是没在取图。
        // 混成 Disconnected 会让一次"忘了 start"被记成硬件故障。
        lastErrorText_ = "未 start()（未开始取图）即调用 grab()：拒绝。";
        return data::GrabResult{
            {data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
    }

    // ---- 取帧 --------------------------------------------------------------
    ImvFrameView view;
    const int ret = api_->getFrame(handle_, view, timeoutMs);

    // ⚠ 主机接收时刻在 `getFrame` 返回后**立即**取得：它表示"何时收到"，
    // 而不是"何时处理完"。放到复制之后取会让 timestampNs 把复制耗时
    // 算进去，而三相机同步判据（TriggerConfig::syncToleranceNs）正是
    // 建立在这个字段上 —— 三条链的复制耗时不同，误差就会被记成"不同步"。
    const uint64_t recvNs = data::monotonicNowNs();

    if (ret != kImvOk)
    {
        // ⚠ **先分类，再谈帧**：`ret != IMV_OK` 时**不读** `view` 的任何
        //    字段。SDK 未保证失败时输出字段有效（头文件没有相应说明），
        //    据此"交叉判断是否超时"是上一版做过的错事，已删除。
        //    帧侧判据只在**调用成功**之后使用。
        lastErrorText_ = std::string("取帧失败：") +
                         data::sdkCallName(data::SdkCall::ImvGetFrame) +
                         " 返回 " + std::to_string(ret);
        return asGrab(sdkFailure(data::SdkCall::ImvGetFrame, ret));
    }

    // 到这里帧已归 SDK 持有，**必须**释放。为让"释放结果能并回返回值"，
    // 组装写进**局部**对象，只有全部成功时才把它交给调用方（唯一交付点）。
    data::ImageFrame built;
    data::GrabResult result = buildFrameFromView(view, recvNs, built);

    const std::optional<data::SdkFailure> cleanup = releaseFrame();
    mergeCleanup(result, cleanup);

    if (result.ok())
    {
        // 唯一交付点：走到这里意味着"帧通过全部检查 **且** 释放成功"。
        // ⚠ 两个条件缺一不可 —— `GetFrame` 成功而 `ReleaseFrame` 失败时
        //    `result.status` 已被 `mergeCleanup` 改为失败，帧**不交付**。
        frame = std::move(built);
    }
    return result;
}

data::GrabResult ImvCameraBackend::buildFrameFromView(const ImvFrameView& view,
                                                     uint64_t            nowNs,
                                                     data::ImageFrame&   out)
{
    // ---- ① 指针非空 --------------------------------------------------------
    if (view.data == nullptr)
    {
        lastErrorText_ = "SDK 返回的帧数据指针为空（frameInfo.status=" +
                         std::to_string(view.status) + "）。";
        // ⚠ 归 `CorruptFrame` 而不是 `Timeout`：SDK 调用**成功了**，
        //    是取回的帧不可用。归成超时会让人去重采，而重采拿到的
        //    还是同一类帧 —— 白等一轮。
        return data::GrabResult{
            {data::OpStatus::CorruptFrame,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- ② 宽高非零 --------------------------------------------------------
    if (view.width == 0 || view.height == 0)
    {
        lastErrorText_ = "SDK 返回的帧宽或高为 0（width=" +
                         std::to_string(view.width) + " height=" +
                         std::to_string(view.height) + "）。";
        return data::GrabResult{
            {data::OpStatus::CorruptFrame,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- ③ 每像素字节可定 --------------------------------------------------
    data::PixelFormat format = data::PixelFormat::Mono8;
    if (!mapSdkPixelFormat(view.pixelFormat, format))
    {
        lastErrorText_ = "未知的 SDK 像素格式码 0x" +
                         [](int32_t c) {
                             char buf[16];
                             std::snprintf(buf, sizeof(buf), "%08X",
                                           static_cast<unsigned>(c));
                             return std::string(buf);
                         }(view.pixelFormat) +
                         "：本批不支持，明确失败（不回落、不猜测布局）。";
        // 未知格式 ⇒ `NotImplemented`（不是 CorruptFrame）：帧本身可能
        // 完全正常，只是**我们没有解读它的依据**。
        return data::GrabResult{
            {data::OpStatus::NotImplemented,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    if (format == data::PixelFormat::Mono12Packed)
    {
        // 本批按范围不做。⚠ 归属要写对：0x010C0006 在官方格式值表中是
        // **GigE Vision 2.0** 的 Mono12Packed；PFNC 的 `Mono12p =
        // 0x010C0047` 是**另一项**，两者的打包规则不通用。
        lastErrorText_ =
            "像素格式 Mono12Packed（SDK 码 0x010C0006）本批未实现"
            "（按范围不做）。该码属 GigE Vision 2.0 的 Mono12Packed，"
            "与 PFNC 的 Mono12p(0x010C0047) 是两项，将来实现时须按"
            "其实际所属规范，不能套用同一套打包规则。";
        return data::GrabResult{
            {data::OpStatus::NotImplemented,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- ④ 64 位乘法并检查溢出（**在复制之前**）----------------------------
    uint64_t expectedBytes = 0;
    if (!data::computeExpectedCompactBytes(view.width, view.height, format,
                                          expectedBytes))
    {
        lastErrorText_ = "载荷长度计算溢出或格式不受支持：width=" +
                         std::to_string(view.width) + " height=" +
                         std::to_string(view.height) +
                         "。拒绝复制（按溢出后的偏小长度复制会读写越界）。";
        return data::GrabResult{
            {data::OpStatus::CorruptFrame,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- ⑤ 长度与本批紧凑契约相符 ------------------------------------------
    // ⚠ 本批**声明**的契约：Mono8/Mono12/BGR8 三种格式按 1/2/3 字节每像素
    //    的紧凑布局解释，**无行尾填充、无整帧填充**。这是本批明确的声明，
    //    不是从字节数"推"出来的结论，也不是 SDK 的保证。
    //    `compactSizeMatches` **只是校验条件** —— 它说明"长度与紧凑契约
    //    相容"，**证明不了像素怎么排列**（相同字节数对应多种排列是可能的），
    //    故该布尔量在冻结文档与元数据里都不得改名为"布局已确认"。
    const uint64_t sdkBytes = static_cast<uint64_t>(view.size);
    const bool compactSizeMatches = (sdkBytes == expectedBytes);

    if (!compactSizeMatches)
    {
        // 把"长度不符"这一**可查事实**记进错误记录与日志。
        // （不承诺它进结果包 —— 失败即不发布帧，没有交接通道。）
        lastErrorText_ =
            "帧长度不符合本批紧凑契约：sdkPayloadBytes=" +
            std::to_string(sdkBytes) + "，expectedCompactBytes=" +
            std::to_string(expectedBytes) + "，格式码=0x" +
            [](int32_t c) {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "%08X",
                              static_cast<unsigned>(c));
                return std::string(buf);
            }(view.pixelFormat) +
            "，paddingX=" + std::to_string(view.paddingX) +
            "，paddingY=" + std::to_string(view.paddingY) +
            "（padding 语义未文档化，不参与任何计算）。";
        return data::GrabResult{
            {data::OpStatus::CorruptFrame,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- 与配置的画幅比对 --------------------------------------------------
    // 配置里的宽高是"标定时的分辨率"，与实际画幅不符意味着内参的主点
    // 会整体平移，而内参不会因此失效（见 CameraCalibration.h）⇒ 必须拦住。
    if (config_.width > 0 && config_.height > 0 &&
        (static_cast<uint32_t>(config_.width) != view.width ||
         static_cast<uint32_t>(config_.height) != view.height))
    {
        lastErrorText_ = "画幅与配置不一致：设备回报 " +
                         std::to_string(view.width) + "×" +
                         std::to_string(view.height) + "，配置为 " +
                         std::to_string(config_.width) + "×" +
                         std::to_string(config_.height) +
                         "（内参按配置的分辨率标定，不一致即主点平移）。";
        return data::GrabResult{
            {data::OpStatus::ContractViolation,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- 复制（此前全部校验已通过）----------------------------------------
    // ⚠ 复制**必须发生在 `IMV_ReleaseFrame` 之前**（调用方随后立即释放）：
    //    帧缓冲是 SDK 内部缓存，释放后会被复用。本步存在的**全部理由**
    //    就是这一点。
    // ⚠ 每帧一份独立缓冲（不做池复用）：池复用会把 SDK 的缓冲复用问题
    //    搬进后端内部，而调用方仍可能持有引用。
    auto bytes = std::make_shared<std::vector<uint8_t>>(
        view.data, view.data + static_cast<std::size_t>(sdkBytes));

    // ---- 显示图 ------------------------------------------------------------
    cv::Mat display;
    if (!buildDisplayImage(*bytes, format, view.width, view.height, display))
    {
        lastErrorText_ = "生成 8U 显示图失败（格式与载荷组合不受支持）。";
        return data::GrabResult{
            {data::OpStatus::CorruptFrame,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // ---- 组装（先写局部对象 `built`，最后才交给 `out`）---------------------
    data::ImageFrame built;
    built.image             = std::move(display);
    // 设备帧号：取 SDK 的 `blockId`（对 U3V/GigE 有效）。
    // ⚠ 若设备不提供它（恒为 0），`MultiCameraManager` 的曝光序号会
    //    退化为内部计数**并置位可见标志** `exposureIndexDegraded()`
    //    （见该处说明），不会静默给出一串恒定的序号。
    built.frameId           = view.blockId;
    built.timestampNs       = nowNs;
    built.deviceTimestampNs = view.timeStamp;
    built.cameraId          = config_.cameraId;
    built.role              = config_.role;
    built.exposureTime      = config_.exposureTime;

    built.raw.bytes               = bytes;
    built.raw.format              = format;
    built.raw.validBits           = data::validBitsOf(format);
    built.raw.packing             = data::requiredPackingOf(format);
    built.raw.bitAlignment        = data::bitAlignmentOf(format);
    built.raw.width               = view.width;
    built.raw.height              = view.height;
    built.raw.sdkPayloadBytes     = sdkBytes;
    built.raw.expectedCompactBytes = expectedBytes;
    built.raw.compactSizeMatches  = compactSizeMatches;
    // 字节序是**适配层声明**（依据：本工程主机为 x86_64/UOS），
    // 不是 SDK 保证。`BigEndian` 在本批不被支持 —— 若声明为它，
    // 显示转换的组装顺序就与本声明不符，故此处只可能得到 LittleEndian。
    built.raw.declaredByteOrder   = data::ByteOrder::LittleEndian;
    built.raw.sdkPixelFormatCode  = view.pixelFormat;
    built.raw.sdkPaddingX         = static_cast<int32_t>(view.paddingX);
    built.raw.sdkPaddingY         = static_cast<int32_t>(view.paddingY);

    built.captureFormat = format;
    built.rawPolicy     = data::requiredRawPolicyOf(format);

    // 发布前的组合断言（ENG-09 §4.1 的组合表）：两处各写一份判定必然分叉，
    // 而分叉的表现是"后端放行的帧被 Recorder 拒绝"，看起来像 I/O 故障。
    if (!data::isValidCombination(built.raw.format, built.raw.packing,
                                  built.raw.validBits, built.raw.bitAlignment))
    {
        // 走到这里说明**本文件自身**的组合与契约矛盾 —— 是代码缺陷，
        // 不是设备问题。归 `ContractViolation`（本地契约错误，
        // `sdkError` 保留 `{ImvGetFrame, IMV_OK}`：SDK 调用确实成功了）。
        lastErrorText_ = "本后端组装出的 raw 元数据违反组合表（代码缺陷）。";
        return data::GrabResult{
            {data::OpStatus::ContractViolation,
             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
             std::nullopt}};
    }

    // 到这里"帧已通过全部检查"——**唯一**写 `out` 的地方。
    out = std::move(built);
    return data::GrabResult{{data::OpStatus::Ok,
                             data::SdkFailure{data::SdkCall::ImvGetFrame, kImvOk},
                             std::nullopt}};
}

// ---------------------------------------------------------------------------
//  诊断
// ---------------------------------------------------------------------------

data::DeviceIdentity ImvCameraBackend::deviceIdentity() const
{
    return deviceIdentity_;
}

std::string ImvCameraBackend::lastErrorText() const
{
    return lastErrorText_;
}

data::DeviceState ImvCameraBackend::state() const
{
    return state_;
}

}  // namespace device
}  // namespace aircraft
