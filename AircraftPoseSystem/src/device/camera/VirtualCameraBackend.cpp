// ============================================================================
//  src/device/camera/VirtualCameraBackend.cpp
//
//  依据：SYS-06 §4 / §14、SYS-08 §10、ENG-09 §2.5 / §5.5
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "device/camera/VirtualCameraBackend.h"

#include <algorithm>
#include <string>

#include <opencv2/imgproc.hpp>

#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// 虚拟相机的默认分辨率：width/height 未配置时使用。
/// 选 1280×1024 的理由：它与华睿 A7A20MU201 的常见工作分辨率同量级，
/// 使预览与算法链的性能表现接近真机（若默认成 320×240，
/// 则特征提取耗时、队列行为、预览帧率都会与现场相差一个量级，
/// M1 测出的"闭环跑通"就失去了对现场的参考价值）。
constexpr int kDefaultWidth  = 1280;
constexpr int kDefaultHeight = 1024;

/// 合成目标（模拟飞机）在图像中的基准边长，单位 pixel。
/// 按角色区分，模拟不同焦距下的成像大小差异：
/// 焦距越长，同一目标成像越大。比例取 25:50:100 的近似对数折中
/// （并非线性 1:2:4 —— 真实成像在长焦端会因距离截断与视场限制而
///  不至于等比放大），目的是让三个候选的 TargetScaleEstimate
///  有可区分的输入，从而评分链的选择逻辑有实际内容可跑。
int targetBaseSizeFor(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:
        return 60;
    case data::CameraRole::CAM50:
        return 120;
    case data::CameraRole::CAM100:
        return 220;
    }
    return 60;
}

const char* roleLabel(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:
        return "CAM25";
    case data::CameraRole::CAM50:
        return "CAM50";
    case data::CameraRole::CAM100:
        return "CAM100";
    }
    return "CAM?";
}
}  // namespace

VirtualCameraBackend::VirtualCameraBackend(const data::CameraConfig& config)
    : config_(config)
{
    // 尺寸解析放在构造函数而非 initialize()：
    // 若放在 initialize()，则构造完成到 initialize() 之间的实例
    // 处于"尺寸未知"状态，而这个窗口内任何试图了解画幅的代码
    // （如上层为预览窗口预留尺寸）都会拿到 0。构造即确定尺寸，
    // 使对象从诞生起就是完整的。
    width_  = config_.width  > 0 ? config_.width  : kDefaultWidth;
    height_ = config_.height > 0 ? config_.height : kDefaultHeight;
}

VirtualCameraBackend::~VirtualCameraBackend()
{
    // 兜底关闭（幂等）。见 ICameraBackend 的析构说明：
    // 拥有者正常关闭时会显式调 close()，但启动中途失败等路径会绕过它。
    (void)close();
}

data::OperationResult VirtualCameraBackend::initialize()
{
    lifecycleLog_.push_back("initialize");   // 见头文件：重复调用必须留痕

    // 尺寸在构造函数中已回退，此处不可能失败 ——
    // 但仍显式返回 Ok 而非省略：契约上 initialize() 的结果
    // 参与 SYS-08 §7.5 的"可用相机数"统计，若将来加入别的检查，
    // 调用方不需要改动。
    state_ = data::DeviceState::READY;

    // 重新 initialize()（例如 close() 之后再用）：清掉关闭标志。
    closed_ = false;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult VirtualCameraBackend::start()
{
    lifecycleLog_.push_back("start");

    if (state_ == data::DeviceState::UNKNOWN)
    {
        // 未 initialize 就 start：拒绝，而不是隐式初始化。
        // 隐式初始化会让"忘记调 initialize()"这一缺陷不可见，
        // 而该缺陷在真实后端上表现为 SDK 未加载即操作设备（可能崩溃）。
        // ⚠ `sdkError` 为空：本类**没有调用任何 SDK**（§4.3 第一条语义），
        //    伪造一个"调用过"会让离线排查去找一个不存在的 SDK 返回码。
        return data::OperationResult{data::OpStatus::NotStarted, std::nullopt,
                                     std::nullopt};
    }

    state_ = data::DeviceState::RUNNING;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult VirtualCameraBackend::stop()
{
    lifecycleLog_.push_back("stop");

    // 契约要求"允许在未 start() 时调用且无副作用"（ICameraBackend）。
    // 因此这里不检查前置状态，也不重置 frameId_（见头文件说明）。
    if (state_ == data::DeviceState::RUNNING)
    {
        state_ = data::DeviceState::READY;
    }
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult VirtualCameraBackend::close()
{
    lifecycleLog_.push_back("close");   // 幂等分支也要留下"被调过第二次"的事实

    // 虚拟后端没有 SDK 可关，也没有句柄可销毁 ⇒ 本方法只清自身状态。
    // 幂等：已关闭时直接返回 Ok（不得因为"被调了第二次"而报错 ——
    // 拥有者与析构兜底各调一次是**设计内**的行为）。
    if (closed_)
    {
        return data::OperationResult{data::OpStatus::Ok, std::nullopt,
                                     std::nullopt};
    }

    closed_ = true;
    state_  = data::DeviceState::UNKNOWN;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::GrabResult VirtualCameraBackend::grab(data::ImageFrame& frame,
                                            uint32_t         timeoutMs)
{
    ++grabCallCount_;

    // ⚠ `timeoutMs` 在本类上不产生等待：虚拟相机是**立即**合成一帧的，
    // 没有"等设备出图"这件事。保留参数是为了满足接口契约，且**不**据此
    // 假装"按超时等待了" —— 预算的判定权在 MultiCameraManager（§2.2），
    // 本类只是被动地被调用（或在预算不足时**不被调用**）。
    (void)timeoutMs;

    if (injectedGrabStatus_.has_value() &&
        *injectedGrabStatus_ != data::OpStatus::Ok)
    {
        const data::OpStatus injected = *injectedGrabStatus_;
        if (injected == data::OpStatus::Disconnected)
        {
            state_ = data::DeviceState::DISCONNECTED;
        }
        // 注入的失败**不带 sdkError**：本类没有 SDK 可调，
        // 伪造一个原码会让"分类来自哪里"变得不可考。
        return data::GrabResult{{injected, std::nullopt, std::nullopt}};
    }

    if (faulty_)
    {
        // 模拟断连：状态同步置 DISCONNECTED，使 state() 的观察者也
        // 能看到故障 —— 否则会出现"grab 恒失败但 state() 报 RUNNING"
        // 的矛盾，而 SYS-06 §14 的 DeviceState 正是为了暴露这类情况。
        //
        // ⚠ 这是"虚拟后端的**模拟**禁用"，**不是**"已确认断连"：
        //    本类不接任何 SDK，没有任何设备侧的确认发生过。
        state_ = data::DeviceState::DISCONNECTED;
        return data::GrabResult{
            {data::OpStatus::Disconnected, std::nullopt, std::nullopt}};
    }

    // ⚠ **断连要先于"未就绪"判定**（2026-09-26 修正）。
    //
    // `simulateFault(true)` 与"注入 `Disconnected`"都会把状态置为
    // `DISCONNECTED`。若下面那条 `state_ != RUNNING ⇒ NotStarted` 排在前面，
    // 它会先命中，于是**一次断连被报成 `NotStarted`**：
    //   · `MultiCameraManager` 按分类处置 ⇒ `NotStarted` **不禁用**通道
    //     （那是瞬态/本地类别），断连与"忘了 start"得到完全相同的处置，
    //     正是 R09 要消除的失效形态；
    //   · 日志写下"瞬态/本地，通道仍可用"—— 对一个已经不在的设备说它可用。
    // ∴ 状态与分类必须**一一对应**：`DISCONNECTED` ⇒ `Disconnected`，
    // 其余非 RUNNING（含 `UNKNOWN`＝未 initialize）才是 `NotStarted`。
    if (state_ == data::DeviceState::DISCONNECTED)
    {
        return data::GrabResult{
            {data::OpStatus::Disconnected, std::nullopt, std::nullopt}};
    }

    if (state_ != data::DeviceState::RUNNING)
    {
        // 未 start 就取帧：`NotStarted`（不是超时，也不是断连）。
        // ⚠ 不碰 frame 的任何字段（接口契约）。
        return data::GrabResult{
            {data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
    }

    // ---- 合成图像 ----
    cv::Mat img(height_, width_, CV_8UC3);

    // 背景：垂直渐变。用渐变而非纯色，是为了让画面在**静止**时
    // 也含有结构（纯色背景下特征提取无点可提，且预览看不出是否收到数据）。
    for (int y = 0; y < height_; ++y)
    {
        const uchar v = static_cast<uchar>(40 + (y * 60) / std::max(1, height_));
        img.row(y).setTo(cv::Scalar(v, v, v));
    }

    // 网格：每 64 pixel 一条暗线。网格使转台移动、画面偏移**肉眼可辨**，
    // 这是 M1 验证闭环时唯一的直观依据。
    const cv::Scalar gridColor(90, 90, 90);
    for (int x = 0; x < width_; x += 64)
    {
        cv::line(img, cv::Point(x, 0), cv::Point(x, height_ - 1), gridColor, 1);
    }
    for (int y = 0; y < height_; y += 64)
    {
        cv::line(img, cv::Point(0, y), cv::Point(width_ - 1, y), gridColor, 1);
    }

    // 图像中心十字：TargetOffset 的原点即此处（裁决 C-11）。
    // 把原点画出来，使"对准是否真的在把目标往中心驱"可以直接看出来。
    const cv::Point center(width_ / 2, height_ / 2);
    cv::line(img, cv::Point(center.x - 20, center.y),
             cv::Point(center.x + 20, center.y), cv::Scalar(0, 0, 200), 1);
    cv::line(img, cv::Point(center.x, center.y - 20),
             cv::Point(center.x, center.y + 20), cv::Scalar(0, 0, 200), 1);

    // 目标：亮色矩形，按角色决定基准尺寸。
    const int    base = targetBaseSizeFor(config_.role);
    const int    halfW = base / 2;
    const int    halfH = (base * 3) / 8;  // 略扁，近似机身侧影
    const cv::Point tgt(static_cast<int>(center.x + targetOffsetX_),
                        static_cast<int>(center.y + targetOffsetY_));

    cv::Rect box(tgt.x - halfW, tgt.y - halfH, base, (base * 3) / 4);
    // 与画幅求交后再绘制：偏移量可能把目标推出画面
    // （对准测试会给出 100 pixel 级偏移），而 cv::rectangle 对
    // 越界矩形会抛异常或静默裁剪（依版本而异）。显式求交使行为确定，
    // 且越界时画面里就是"看不到目标"—— 这正是对准失败应有的表现。
    box &= cv::Rect(0, 0, width_, height_);
    if (box.width > 0 && box.height > 0)
    {
        cv::rectangle(img, box, cv::Scalar(210, 210, 210), cv::FILLED);
        cv::rectangle(img, box, cv::Scalar(255, 255, 255), 2);

        // 机身内部的几条短横线：提供可提取的纹理。
        // 纯色矩形在 SIFT 下几乎没有稳定关键点，
        // 而算法链的 F 分项（特征数量）需要一个非退化的输入。
        for (int i = 1; i < 4; ++i)
        {
            const int y = box.y + (box.height * i) / 4;
            cv::line(img, cv::Point(box.x + 6, y),
                     cv::Point(box.x + box.width - 6, y),
                     cv::Scalar(120, 120, 120), 1);
        }
    }

    // 叠加文字：角色 + 帧号。这两项是"数据是否串路"的肉眼判据（见头文件）。
    const std::string label =
        std::string(roleLabel(config_.role)) + "  #" + std::to_string(frameId_);
    cv::putText(img, label, cv::Point(16, 36),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // 触发模式文字：三种模式各自可见。用 `triggerModeName()` 而不是
    // 三元表达式 —— 三值枚举用 bool 表达必然把两种模式显示成同一个词。
    const std::string status =
        std::string("TRIG:") + data::triggerModeName(reportedMode_);
    cv::putText(img, status, cv::Point(16, 72),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 200, 255), 2);

    // ---- 填写 ImageFrame 的 7 个字段（ENG-09 §5.5）----
    frame.image             = std::move(img);
    frame.frameId           = frameId_++;
    frame.cameraId          = config_.cameraId;
    frame.role              = config_.role;
    frame.exposureTime      = config_.exposureTime;

    // ---- 011-A1 的三个字段（ENG-09 V2.3 §5.5）----
    // 虚拟相机是 8 位 BGR：显示图就是原始数据，故 `raw` **留空**、
    // `rawPolicy` 为 `RawOptional`。Recorder 按 §4.1 的第二分支
    // 照常保存 `image` 并标 `data_source = image`（既有行为不变）。
    // ⚠ `raw.format` 与 `captureFormat` 都**必须**如实填 BGR8 ——
    //    若偷懒留默认的 Mono8，则"后端实际给的格式"与"帧声称的格式"
    //    会在 Recorder 的一致性检查处对不上，而报错会指向配置。
    frame.raw.format      = data::PixelFormat::BGR8;
    frame.raw.validBits   = 8;
    frame.raw.packing     = data::Packing::Unpacked;
    frame.raw.bitAlignment = data::BitAlignment::LsbZeroPadded;
    frame.raw.width       = static_cast<uint32_t>(width_);
    frame.raw.height      = static_cast<uint32_t>(height_);
    frame.captureFormat   = data::PixelFormat::BGR8;
    frame.rawPolicy       = data::requiredRawPolicyOf(data::PixelFormat::BGR8);

    // 两个时间戳语义不同（ENG-09 §2.5），虚拟后端必须如实体现这个差别，
    // 否则依赖二者之差的代码（曝光延迟估计）在真机上才会第一次被检验。
    const uint64_t now      = data::monotonicNowNs();
    frame.timestampNs       = now;                      // 主机接收时刻
    frame.deviceTimestampNs = now > deviceLatencyNs_
                                  ? now - deviceLatencyNs_
                                  : 0;                  // 设备曝光时刻

    return data::GrabResult{
        {data::OpStatus::Ok, std::nullopt, std::nullopt}};
}

data::OperationResult VirtualCameraBackend::setTriggerMode(
    data::CameraTriggerMode mode)
{
    // 虚拟设备"就是它自己"：设置即生效。故 requested 与 reported 同时更新，
    // 但两者是**两个独立的成员** —— 读回值有独立的事实来源，
    // 不是把请求值抄回去（那会让"读回"这件事在本类上无从检验）。
    requestedMode_ = mode;
    reportedMode_  = mode;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::OperationResult VirtualCameraBackend::triggerSoftware()
{
    // 虚拟相机不区分触发方式，但**如实计数**：调用方（§4.4 的用例）
    // 要断言"一次 capture() 内恰好发令一次"，这需要次数可见。
    ++softwareTriggerCount_;
    return data::OperationResult{data::OpStatus::Ok, std::nullopt, std::nullopt};
}

data::DeviceIdentity VirtualCameraBackend::deviceIdentity() const
{
    // ⚠ `queried = false`：本类**没有设备可问**，不能声称"查过了但没读到"。
    // ⚠ `modelName` **留空**，不填 `"virtual"`：型号栏只放型号，
    //    后端类型由装配摘要的独立一栏表达（见 DeviceIdentity.h）。
    return data::DeviceIdentity{};
}

data::TriggerModeState VirtualCameraBackend::triggerModeState() const
{
    data::TriggerModeState s;
    s.requested = requestedMode_;

    // 三项都"读回"——本类就是设备，故它如实回报自己实际持有的取值。
    s.selectorReported = std::string("FrameStart");
    s.switchReported   = std::string(data::triggerModeSymbol(reportedMode_));

    const char* src = data::triggerSourceSymbol(reportedMode_);
    if (src != nullptr)
    {
        s.sourceReported = std::string(src);
    }

    return data::composeTriggerModeState(s);
}

std::string VirtualCameraBackend::lastErrorText() const
{
    // 措辞纪律（与本文件其余处一致）：说清是**模拟/注入**，不得写成
    // "已确认断连" —— 本类不接任何 SDK，没有任何设备侧的确认发生过。
    if (faulty_)
    {
        return "虚拟后端的模拟禁用：本类不接任何 SDK，这是测试/演示用的"
               "模拟断开，不是设备侧报告的断连";
    }
    if (injectedGrabStatus_.has_value())
    {
        return std::string("虚拟后端注入了取帧失败分类 ") +
               data::opStatusName(*injectedGrabStatus_) + "（测试用，非设备报告）";
    }
    return std::string();
}

void VirtualCameraBackend::setInjectedGrabStatus(
    std::optional<data::OpStatus> status)
{
    injectedGrabStatus_ = status;
}

void VirtualCameraBackend::simulateFault(bool faulty)
{
    faulty_ = faulty;

    if (faulty)
    {
        state_ = data::DeviceState::DISCONNECTED;
    }
    else if (state_ == data::DeviceState::DISCONNECTED ||
             state_ == data::DeviceState::ERROR)
    {
        // 恢复时回到 READY 而非 RUNNING：调用方必须重新 start()。
        // 直接回到 RUNNING 会掩盖"恢复后未重新启动采集"这一缺陷，
        // 而真实相机断连重连后确实需要重新开始采集。
        state_ = data::DeviceState::READY;
    }
}

void VirtualCameraBackend::setTargetPixelOffset(double dx, double dy)
{
    targetOffsetX_ = dx;
    targetOffsetY_ = dy;
}

void VirtualCameraBackend::setDeviceLatencyNs(uint64_t ns)
{
    deviceLatencyNs_ = ns;
}

data::DeviceState VirtualCameraBackend::state() const
{
    return state_;
}

uint64_t VirtualCameraBackend::frameCount() const
{
    return frameId_;
}

uint64_t VirtualCameraBackend::softwareTriggerCount() const
{
    return softwareTriggerCount_;
}

uint64_t VirtualCameraBackend::grabCallCount() const
{
    return grabCallCount_;
}

const std::vector<std::string>& VirtualCameraBackend::lifecycleLog() const
{
    return lifecycleLog_;
}

}  // namespace device
}  // namespace aircraft
