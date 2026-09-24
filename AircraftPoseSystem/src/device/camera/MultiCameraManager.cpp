// ============================================================================
//  src/device/camera/MultiCameraManager.cpp
//
//  依据：SYS-06 §5 / §8、SYS-08 §7.5、ENG-09 §2.5 / §5.6
// ============================================================================

#include "device/camera/MultiCameraManager.h"

#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace device
{

namespace
{
/// SYS-08 §7.5 的降级下限：可用相机数 <2 时无法继续测量。
/// 该数值来自冻结的降级表，不是可调参数，故为文件内常量
/// （具有业务含义的数值不应散落在比较表达式中）。
constexpr int kMinUsableCameras = 2;

/// SYS-08 §7.5 的降级上限：低于此数即为降级。
constexpr int kFullCameraCount = 3;
}  // namespace

MultiCameraManager::MultiCameraManager(std::shared_ptr<ICameraBackend> cam25,
                                      std::shared_ptr<ICameraBackend> cam50,
                                      std::shared_ptr<ICameraBackend> cam100)
{
    cam25_.backend  = std::move(cam25);
    cam25_.role     = data::CameraRole::CAM25;
    cam50_.backend  = std::move(cam50);
    cam50_.role     = data::CameraRole::CAM50;
    cam100_.backend = std::move(cam100);
    cam100_.role    = data::CameraRole::CAM100;

    // 可用性初值为 false：必须由 initializeAll() 真正初始化成功才置真。
    // 若此处按"backend 非空即为可用"初始化，则 availableCameraCount()
    // 会在未初始化的实例上返回 3，使降级判据失去意义。
}

MultiCameraManager::Channel& MultiCameraManager::channelOf(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:
        return cam25_;
    case data::CameraRole::CAM50:
        return cam50_;
    case data::CameraRole::CAM100:
        return cam100_;
    }
    // CameraRole 只有三个取值（ENG-09 §4.1），无 UNKNOWN 可落入此处。
    // 保留返回以保证在 -Wswitch 关闭或枚举被误扩展时不致未定义行为；
    // 返回 cam25_ 会掩盖问题，但因该分支不可达，仅作兜底。
    return cam25_;
}

const MultiCameraManager::Channel& MultiCameraManager::channelOf(
    data::CameraRole role) const
{
    return const_cast<MultiCameraManager*>(this)->channelOf(role);
}

bool MultiCameraManager::initializeAll()
{
    lastError_ = data::ErrorInfo{};

    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        // 无 backend = 该通道没有相机（见头文件对 nullptr 的说明）。
        // 这不是失败，只是少一台，交由下方的可用数判据统一处理 ——
        // 若在此处直接返回 false，则"只装了两台相机"这一正常配置
        // 会被当成初始化失败，而 §7.5 明确允许 2 台降级工作。
        if (!ch->backend)
        {
            ch->available = false;
            continue;
        }

        ch->available = ch->backend->initialize();
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        // SYS-08 §7.5：≤1 台直接 FAILED，code=1001。
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，无法继续测量";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    // 2 台可用时进入降级，但**仍返回 true**（§7.5："降级继续"）。
    // 降级事实由 degraded() 提供，上层负责写入 result.json 并在 UI 显示
    // （§7.5："降级必须在 UI 上可见，不得静默降级"）。
    // 记录一条非致命的降级信息：code=1002 的语义是"已降级"而非"失败"。
    if (degraded())
    {
        lastError_.code        = data::kErrCameraDegraded;
        lastError_.message     = "相机数量不足 3，已降级继续";
        lastError_.timestampNs = data::monotonicNowNs();
        return true;
    }

    return true;
}

bool MultiCameraManager::startAll()
{
    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        if (!ch->available || !ch->backend)
        {
            continue;
        }

        if (!ch->backend->start())
        {
            // 启动失败的通道按硬件故障处理：标记不可用，由下方可用数判据
            // 决定是降级继续还是失败。不在此处直接返回 false，理由同
            // initializeAll() —— 单台启动失败在 §7.5 下是降级场景。
            ch->available = false;
        }
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        started_               = false;
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，启动失败";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    started_ = true;
    return true;
}

void MultiCameraManager::stopAll()
{
    Channel* channels[] = {&cam25_, &cam50_, &cam100_};

    for (Channel* ch : channels)
    {
        if (ch->backend)
        {
            // 无论 available 与否都调用 stop()：SDK 的 stop 必须是幂等的
            // （ICameraBackend 的契约："允许在未 start() 时调用，须无副作用"），
            // 而跳过调用会让"初始化成功但未 start"的 backend 残留在
            // 未知状态，下次 start 时行为不确定。
            ch->backend->stop();
        }
    }

    started_ = false;
}

bool MultiCameraManager::capture(data::MultiCameraFrame& frame)
{
    lastError_ = data::ErrorInfo{};

    if (!started_)
    {
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "采集前未成功 startAll()";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    if (availableCameraCount() < kMinUsableCameras)
    {
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "可用相机数不足 2，无法采集";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    // 每轮采集都从零开始构造 frame：不清零会让上一轮某路的数据残留，
    // 而本轮该路若采集失败（返回的 ImageFrame 内容未定义、实现可能
    // 只写了部分字段），残留的旧图会被当成新图使用 —— 且时间戳是旧的，
    // 恰好会被同步判据判为"不同步"，故障现象指向触发而根因在复用。
    frame = data::MultiCameraFrame{};

    uint64_t maxTs = 0;
    int      captured = 0;
    bool     present[3] = {false, false, false};

    Channel* channels[] = {&cam25_, &cam50_, &cam100_};
    data::ImageFrame* outputs[] = {&frame.cam25, &frame.cam50, &frame.cam100};

    for (std::size_t i = 0; i < 3; ++i)
    {
        Channel* ch = channels[i];
        if (!ch->available || !ch->backend)
        {
            continue;
        }

        uint64_t ts = 0;
        if (grabOne(*ch, *outputs[i], ts))
        {
            ++captured;
            present[i] = true;
            if (ts > maxTs)
            {
                maxTs = ts;
            }
        }
        else
        {
            // 采集途中掉线：按 §7.5 归类为硬件故障 —— **不重试**，
            // 直接禁用该通道并继续。若在此处重试，则一个物理上不可能
            // 恢复的故障会消耗掉 §7.3 的重试预算，
            // 使真正属于瞬态类的失败（如 PnP 未收敛）失去重试机会。
            ch->available = false;
        }
    }

    if (captured < kMinUsableCameras)
    {
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "本轮采集可用相机数不足 2";
        lastError_.timestampNs = data::monotonicNowNs();
        return false;
    }

    // ENG-09 §2.5：triggerTimestamp = 各相机 timestampNs 的最大值。
    // 只统计本轮真正采到的通道 —— 被跳过的通道其 timestampNs 为 0，
    // 若不排除，在只有 2 台可用时 max 仍正确（0 不是最大值），
    // 但逻辑上把"未采集"混入"已采集"的统计范围本身就是错的，
    // 一旦未来某处的默认值不再是 0 就会静默出错。
    frame.triggerTimestamp = maxTs;

    // 裁决 D-C02-6：本组三帧的"同一次曝光序号"。放在 triggerTimestamp
    // 之后、成功返回之前 —— 走到这里意味着本轮确实产出了一组可用帧，
    // 失败路径上的 frame 内容未定义，不应产出序号。
    frame.exposureIndex = updateExposureIndex(frame, present);

    return true;
}

uint64_t MultiCameraManager::updateExposureIndex(
    const data::MultiCameraFrame& frame,
    const bool present[3])
{
    const data::ImageFrame* channels[] = {&frame.cam25, &frame.cam50, &frame.cam100};

    // 参考路：按 CAM25 → CAM50 → CAM100 顺序的第一路可用者。
    // 为什么是固定顺序而非"随便哪一路"：序号必须在整轮任务中来自**同一条**
    // 通道，否则某轮 CAM25 掉线会让序号来源悄悄换人，而两条链的偏移量
    // 在同一次采集里未必相等 —— 序号就会出现无法解释的跳变。
    int ref = -1;
    for (int i = 0; i < 3; ++i)
    {
        if (present[i])
        {
            ref = i;
            break;
        }
    }
    if (ref < 0)
    {
        return 0;   // 无任何可用路：未定
    }

    const uint64_t frameId = channels[ref]->frameId;

    if (!frameIdBaseLocked_[ref])
    {
        frameIdBase_[ref]       = frameId;
        frameIdBaseLocked_[ref] = true;
    }
    else if (frameId < frameIdBase_[ref])
    {
        // 设备计数器回退（相机重连 / 重启 / 回绕）：基准失效，就地重锁。
        // 不重锁会让减法下溢成一个巨大的序号，而那个值看起来仍然"合法"。
        frameIdBase_[ref]       = frameId;
        exposureIndexDegraded_  = true;
    }

    // 1 起计数：0 保留给"未定"（MultiCameraFrame 的字段说明）。
    uint64_t candidate = frameId - frameIdBase_[ref] + 1;

    if (candidate <= exposureIndex_)
    {
        // 设备帧号没有前进（例如某后端的 grab() 尚未填 frameId，
        // 使三路都恒为基值）。此时若照用，序号会恒等于 1 ——
        // 而它承载的正是"这几张 raw 是不是同一次曝光"这个判断，
        // 恒值会让该判断永远为"是"。
        // 处置：保住单调性（调用方依赖这条不变量），并把
        // "序号已不来自设备帧号"这件事**记录成可见事实**。
        exposureIndexDegraded_ = true;
        candidate              = exposureIndex_ + 1;
    }

    exposureIndex_ = candidate;
    return exposureIndex_;
}

bool MultiCameraManager::exposureIndexDegraded() const
{
    return exposureIndexDegraded_;
}

bool MultiCameraManager::grabOne(Channel& ch,
                                data::ImageFrame& out,
                                uint64_t& timestampOut)
{
    data::ImageFrame tmp;
    if (!ch.backend->grab(tmp))
    {
        return false;
    }

    // 校正身份字段：backend 的实现可能不知道自己的角色
    // （ImvCameraBackend 由 SDK 句柄构造，未必携带 CameraRole）。
    // 在管理器这一层统一补齐，使上层永远可以信任
    // ImageFrame::role 与它所在的 frame 字段一致 ——
    // 否则会出现"frame.cam25 里装的其实是 CAM100 的图"这类
    // 无法从数据本身察觉的错位，而三相机评分的正确性完全依赖它。
    tmp.role = ch.role;

    timestampOut = tmp.timestampNs;
    out          = std::move(tmp);
    return true;
}

int MultiCameraManager::availableCameraCount() const
{
    int n = 0;
    if (cam25_.available && cam25_.backend)
    {
        ++n;
    }
    if (cam50_.available && cam50_.backend)
    {
        ++n;
    }
    if (cam100_.available && cam100_.backend)
    {
        ++n;
    }
    return n;
}

bool MultiCameraManager::degraded() const
{
    const int n = availableCameraCount();
    return n >= kMinUsableCameras && n < kFullCameraCount;
}

data::DeviceState MultiCameraManager::state() const
{
    if (availableCameraCount() < kMinUsableCameras)
    {
        return data::DeviceState::ERROR;
    }

    if (!started_)
    {
        return data::DeviceState::READY;
    }

    return data::DeviceState::RUNNING;
}

data::ErrorInfo MultiCameraManager::lastError() const
{
    return lastError_;
}

bool MultiCameraManager::disableChannel(data::CameraRole role)
{
    Channel& ch = channelOf(role);
    if (!ch.backend)
    {
        return false;
    }

    ch.available = false;

    // 与"采集途中掉线"一致地记录：这是 §7.5 的硬件故障降级路径，
    // 置 code=1002（已降级）而不是错误 —— 上层据 availableCameraCount()
    // 判断是否已跌破下限。
    if (availableCameraCount() < kMinUsableCameras)
    {
        lastError_.code        = data::kErrCameraInsufficient;
        lastError_.message     = "禁用该通道后可用相机数不足 2";
        lastError_.timestampNs = data::monotonicNowNs();
    }
    else
    {
        lastError_.code        = data::kErrCameraDegraded;
        lastError_.message     = "通道已禁用，降级继续";
        lastError_.timestampNs = data::monotonicNowNs();
    }

    return true;
}

}  // namespace device
}  // namespace aircraft
