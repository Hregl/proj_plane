// ============================================================================
//  src/device/camera/ImvCameraBackend.cpp
//
//  依据：SYS-06 §7、ENG-08 §3 / §11、ENG-09 §2.5 / §5.5
//
//  本文件在**任何** `APS_HAVE_IMVSDK` 取值下都提供**诚实桩**：
//  每个方法都明确失败并说明原因，绝不假装成功（理由见头文件）。
//
//  该宏由 src/device/CMakeLists.txt 依据 Dependencies.cmake 的查找结果
//  导出，因此 C++ 侧与 CMake 侧的判断**同源** ——
//  不会出现"CMake 认为有 SDK、C++ 认为没有"这种以 0/1 编译期常量
//  表达的静默不一致。
//
//  ⚠ 2026-09-23（011-A0，评审裁定）—— 原先 `APS_HAVE_IMVSDK` 分支是
//    `#error` **阻断编译**。那是错的，两个理由：
//
//    ① **它使框架不能编译**。一旦 FindImvSdk 找到 SDK（例如本机已按 O-19
//       解包到 third_party/imvsdk），整个工程就连不上，虚拟相机路径也被
//       一起打死 —— 而 ENG-08 §3 明确要求软件闭环在无真实适配时仍可运行。
//    ② **它只在构建期说一次话，运行期什么都不说**。而"SDK 在、适配没写"
//       这个事实在运行期同样重要：调用方看到采集失败时，必须能区分
//       "设备不在"与"代码没写"，否则会去查硬件和配置。
//
//    改法（评审原话："不要简单删除"）：保留分支，把阻断改为**构建期可见、
//    运行期可辨**的诚实桩。用 `#pragma message` 而不是 `#warning`：
//    `-Werror`（ENABLE_WERROR，发布前会开）会把 `#warning` 提升为错误，
//    那就把问题原样搬到了发布构建里；`#pragma message` 在 GCC 下永远
//    是 note，不受 `-Werror` 影响。
//
//    接入步骤（ENG-08 Sprint 2 / 011-A1）：
//     1. include 实际 SDK 头（`IMVApi.h`，路径由 aps::imvsdk 提供）；
//     2. 按 SDK 文档实现 initialize()（`IMV_EnumDevices` → 按序列号
//        `IMV_CreateHandle`/`IMV_Open` → 配置曝光/增益/触发模式）、
//        start()/stop()（`IMV_StartGrabbing`/`IMV_StopGrabbing`）、
//        grab()（`IMV_GetFrame` → 转 cv::Mat → 填 7 个字段）、
//        setTriggerMode()；
//     3. 把下面的桩整体换成真实实现，并删除本段说明。
//    ⚠ 与 SDK 相关的三条硬约束见 011-A0.1 核验报告：
//      运行期须 LD_LIBRARY_PATH 指向 CMake 生成的 imvsdk_runtime/；
//      U3V 相机须先设 usbfs_memory_mb=1000；
//      SDK 错误码是**负数**（-100s），不能直接当 ENG-09 §5.27 的 1000 段码返回。
// ============================================================================

#include "device/camera/ImvCameraBackend.h"

#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

#if APS_HAVE_IMVSDK
// SDK 已找到，但适配尚未实现（011-A1）。此消息在**每次构建**都打印一次，
// 使"设备在、代码没写"这件事不会因为配置成功而消失。
#    pragma message("APS_HAVE_IMVSDK=1：已找到华睿 SDK，但 ImvCameraBackend 的 SDK 适配尚未实现（011-A1）—— 当前按诚实桩返回 false（框架可编译、不假装采集成功）。")
#endif

namespace
{
/// 统一的失败说明。集中一处，避免各方法给出措辞不同的原因，
/// 使日志中同一根因出现多种表述而不易归并。
///
/// ⚠ 两种"不可用"必须给出**不同**的文本（011-A0 评审裁定）：
///   它们的处置完全不同 ——
///     · 未找到 SDK：去配 SDK 路径，或用虚拟相机；
///     · 已找到 SDK 但适配未实现：**别去查硬件和配置**，是代码没写。
///   把两者合并成一句话，会让排查方向被引向设备侧。
#if APS_HAVE_IMVSDK
const char* kUnavailableHint =
    "ImvCameraBackend 不可用：已找到华睿 SDK，但 SDK 适配尚未实现"
    "（ENG-08 Sprint 2 / 011-A1）。当前按诚实桩返回 false，"
    "不假装采集成功。在适配完成前请使用 VirtualCameraBackend（ENG-08 §3）。";
#else
const char* kUnavailableHint =
    "ImvCameraBackend 未接入 SDK：当前构建未找到华睿 SDK。"
    "请用 -DIMV_SDK_ROOT=<SDK 根目录> 重新配置，"
    "或在第一阶段使用 VirtualCameraBackend（ENG-08 §3）。";
#endif
}  // namespace

ImvCameraBackend::ImvCameraBackend(const data::CameraConfig& config)
    : config_(config)
{
}

bool ImvCameraBackend::initialize()
{
    // 不假装成功。返回 false 使 SYS-08 §7.5 的可用相机数如实反映现实。
    state_         = data::DeviceState::ERROR;
    lastErrorText_ = kUnavailableHint;
    return false;
}

bool ImvCameraBackend::start()
{
    // 未成功 initialize 就 start 必须失败（与 VirtualCameraBackend 一致）。
    lastErrorText_ = kUnavailableHint;
    return false;
}

void ImvCameraBackend::stop()
{
    // 契约要求无副作用且可重复调用。
    // 此处不改变 state_：设备从未初始化成功，把 ERROR 改成 READY
    // 会谎报"已经就绪"，而 state() 的读者（UI 状态栏）会据此显示正常。
    if (state_ == data::DeviceState::RUNNING)
    {
        state_ = data::DeviceState::READY;
    }
}

bool ImvCameraBackend::grab(data::ImageFrame& frame)
{
    // 不填 frame 的任何字段：契约规定"返回 false 时 frame 内容未定义，
    // 调用方不得使用"。写入部分字段反而会诱导调用方误用半成品数据。
    (void)frame;
    lastErrorText_ = kUnavailableHint;
    return false;
}

bool ImvCameraBackend::setTriggerMode(bool enable)
{
    // 记录意图但报告失败：若返回 true，上层会认为触发模式已切换，
    // 于是不会再走 SYS-08 §7.5 的降级路径，而硬件实际仍处于未知状态。
    (void)enable;
    lastErrorText_ = kUnavailableHint;
    return false;
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
