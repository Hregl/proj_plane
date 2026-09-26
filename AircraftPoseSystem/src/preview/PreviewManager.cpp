// ============================================================================
//  src/preview/PreviewManager.cpp
//
//  依据：SYS-01 §12、SYS-03 §7、SYS-08 §8（AUTO 映射表，冻结）
//        SYS-09 §2.1 / §13.1、ENG-02 §9.1 / ENG-04 §9.1
// ============================================================================

#include "preview/PreviewManager.h"

#include "data/CameraChannel.h"
#include "data/MonotonicClock.h"

namespace aircraft
{
namespace preview
{

namespace
{
/// 兜底相机名。rig 为空或角色缺失时使用，使界面始终有可显示的文本。
const char* fallbackCameraId(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return "CAM25";
    case data::CameraRole::CAM50:  return "CAM50";
    case data::CameraRole::CAM100: return "CAM100";
    }
    return "CAM25";
}
}  // namespace

PreviewManager::PreviewManager(const PreviewConfig&       config,
                               const optical::OpticalRig* rig)
    : config_(config)
    , rig_(rig)
    , queue_(static_cast<std::size_t>(config.queueSize > 0 ? config.queueSize : 0))
    , mode_(config.mode)
    , manualCamera_(config.defaultCamera)
    , autoCamera_(config.defaultCamera)
    , lastDisplayCamera_(config.defaultCamera)
{
    if (queue_.clamped())
    {
        notices_.push_back(
            "PreviewConfig::queueSize = " + std::to_string(config.queueSize) +
            " 超出 SYS-09 §13.1 冻结的区间 [3, 5]，已夹取为 " +
            std::to_string(queue_.capacity()) + "。队列容量即允许积压的帧数，"
            "过大时预览会显示过期画面。");
    }

    if (config.defaultCamera != data::CameraRole::CAM25)
    {
        // SYS-01 §12 / SYS-03 §7 冻结默认 CAM25；这里不阻止改，
        // 但记录下来，因为"默认不是 CAM25"通常意味着配置被误改。
        notices_.push_back(
            "默认显示源被配置为 " + std::string(fallbackCameraId(config.defaultCamera)) +
            "，而 SYS-01 §12 / SYS-03 §7 冻结的默认值为 CAM25。");
    }
}

// ---------------------------------------------------------------------------
// 模式
// ---------------------------------------------------------------------------

void PreviewManager::setMode(PreviewMode mode)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (mode_ == mode)
    {
        return;
    }

    const data::CameraRole before = lastDisplayCamera_;
    mode_ = mode;
    const data::CameraRole after = displayCameraLocked();
    noteDisplaySourceChanged(before, after);
}

PreviewMode PreviewManager::mode() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

// ---------------------------------------------------------------------------
// 显示源选择
// ---------------------------------------------------------------------------

bool PreviewManager::setCamera(data::CameraRole role)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cameraEnabled(role))
    {
        return false;
    }

    if (manualCamera_ == role)
    {
        return true;
    }

    const data::CameraRole before = lastDisplayCamera_;
    manualCamera_ = role;
    const data::CameraRole after = displayCameraLocked();
    noteDisplaySourceChanged(before, after);
    return true;
}

data::CameraRole PreviewManager::camera() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return manualCamera_;
}

void PreviewManager::setAutoCamera(data::CameraRole role)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (autoCamera_ == role)
    {
        return;
    }

    const data::CameraRole before = lastDisplayCamera_;
    autoCamera_ = role;
    const data::CameraRole after = displayCameraLocked();
    noteDisplaySourceChanged(before, after);
}

data::CameraRole PreviewManager::autoCamera() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return autoCamera_;
}

void PreviewManager::setMeasurementState(data::MeasurementState state)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == state)
    {
        return;
    }

    const data::CameraRole before = lastDisplayCamera_;
    state_ = state;
    const data::CameraRole after = displayCameraLocked();
    noteDisplaySourceChanged(before, after);
}

data::MeasurementState PreviewManager::measurementState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

data::CameraRole PreviewManager::displayCamera() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return displayCameraLocked();
}

data::CameraRole PreviewManager::displayCameraLocked() const
{
    if (mode_ == PreviewMode::MANUAL)
    {
        return manualCamera_;
    }
    return mapStateToCamera(state_, autoCamera_, lastDisplayCamera_);
}

data::CameraRole PreviewManager::mapStateToCamera(data::MeasurementState state,
                                                  data::CameraRole       selected,
                                                  data::CameraRole       fallback)
{
    // SYS-08 §8 冻结的三行（原文照录）：
    //     SEARCH  → CAM25
    //     ALIGN   → CAM50
    //     MEASURE → Selected Camera
    //
    // ⚠ 表未覆盖 SEARCH/ALIGN/MEASURE 之外的 9 个状态。补齐的依据是
    // 同一张表的物理意图（大视场找目标 → 中焦对准 → 测量焦段出结果），
    // 而非另立规则：
    switch (state)
    {
    // 空闲：什么都没有在跑，用默认焦段（SYS-01 §12 默认 CAM25）。
    case data::MeasurementState::IDLE:
    // 搜索：必须用大视场，否则目标根本进不了画面（§8 第一行）。
    case data::MeasurementState::SEARCH:
    // 目标刚检出、尚未对准：仍在搜索的画面里，继续用大视场 ——
    // 此时切到 50mm 会因为视场骤然变小而把目标挤出画面，
    // 紧接着的 ALIGN 就失去了输入。
    case data::MeasurementState::TARGET_FOUND:
        return data::CameraRole::CAM25;

    // 对准与稳定：§8 第二行给出 ALIGN → CAM50。
    // STABILIZE 是对准之后的等稳过程，画面必须与对准时一致 ——
    // 否则操作者会看到"对准完画面跳了一下"。
    case data::MeasurementState::ALIGN:
    case data::MeasurementState::STABILIZE:
    // 通道选择正在三焦段之间评分，尚无结论，沿用中焦。
    case data::MeasurementState::MEASURE_SELECT:
        return data::CameraRole::CAM50;

    // 采集与之后的一切：§8 第三行 MEASURE → Selected Camera。
    // CAPTURE 起的各状态都发生在"测量"之内，故都用测量焦段。
    case data::MeasurementState::CAPTURE:
    case data::MeasurementState::POSE_SOLVE:
    case data::MeasurementState::VALIDATE:
    case data::MeasurementState::SAVE:
    case data::MeasurementState::COMPLETE:
        return selected;

    // 失败：**保持原显示源**（fallback = 当前显示源）。
    // 故障时把画面切到别的焦段，会让操作者看不到出问题的那一路，
    // 而这正是排查时唯一想看的东西。
    case data::MeasurementState::FAILED:
        return fallback;
    }

    return fallback;
}

// ---------------------------------------------------------------------------
// 采集侧入口
// ---------------------------------------------------------------------------

void PreviewManager::submitFrame(const data::PreviewFrame& frame)
{
    // 显示时刻在此处统一盖写（理由见头文件）。
    data::PreviewFrame stamped = frame;
    stamped.displayTimestamp   = data::monotonicNowNs();

    queue_.push(stamped);
}

bool PreviewManager::submitFrom(const data::MultiCameraFrame& frame)
{
    // ⚠ displayCamera() 只读**一次**，取值同时用于"选哪一路"和"写什么角色"。
    // 读两次会留出一个窗口：若两次读取之间用户切换了相机，第一次选到的是
    // 旧焦段的图像，第二次写入的是新焦段的角色 —— 于是 CAM100 的图像被
    // 标成 CAM25，而 workerDeliver 的角色过滤恰好会把它当成 CAM25 的帧
    // 放行。这正是本函数试图消除的那类错配，不能在函数内部再造一个。
    const data::CameraRole role = displayCamera();

    const data::ImageFrame* src = nullptr;
    switch (role)
    {
    case data::CameraRole::CAM25:  src = &frame.cam25;  break;
    case data::CameraRole::CAM50:  src = &frame.cam50;  break;
    case data::CameraRole::CAM100: src = &frame.cam100; break;
    }

    if (src == nullptr || src->image.empty())
    {
        // 该焦段当前没有图像（相机降级或本轮未采集）。
        // 不提交空帧：空帧进入队列后会占掉一格容量、并被 worker 当作
        // 一帧"正常数据"发布出去，界面于是显示一块空白而不是保留
        // 上一帧，看起来像预览自己坏了。
        return false;
    }

    // ⚠ 显示链路的边界条件：进入预览的 `image` **必须**是 8U。
    //   `image` 是**显示图**（12 位格式经 `>> (validBits − 8)` 得到的
    //   加工产物），而"这一帧真实的采集格式"在 `captureFormat` / `raw` 里 ——
    //   两者不是一回事（ENG-09 V2.3 §5.28 第 6 条）。一幅 16U/32F 的
    //   图进到这里，说明某个后端把**原始载荷**当成了显示图发布，
    //   而症状是"预览的颜色/亮度不对"这类看不出根因的现象。
    //
    //   ⚠ 用**显式检查 + 拒绝**而不是 `assert()`：本仓从无运行期 `assert`
    //     （只有 `static_assert`）；且本函数跑在 GUI 线程上 —— 一次断言
    //     失败会把整个界面带走，而"这一路暂时不显示"是能承受的后果。
    //     不用返回 false 之外的表达（不加 `notices_`）：`notices_` 没有
    //     互斥保护，而本函数由采集线程调用。
    if (src->image.depth() != CV_8U)
    {
        return false;
    }

    data::PreviewFrame pf;
    // ⚠ 这里是**浅拷贝**（`cv::Mat` 共享像素数据；`raw.bytes` 共享一个
    //   `shared_ptr<const vector>`）。它成立的条件正是本批冻结的三条
    //   （ENG-09 V2.3 §5.28 第 5 条），缺一不可：
    //     ① **自有**：`image` 的像素数据由采集侧自有一份 —— 后端在归还
    //        SDK 缓冲区（`IMV_ReleaseFrame`）**之前**已把字节复制出来
    //        （`IMV_GetFrame` 交出的是 SDK 内部缓存，释放后会被**就地复用**）；
    //     ② **发布后不再修改**：一旦提交，采集侧不得再写这份像素数据；
    //     ③ `raw.bytes` 是 `shared_ptr<const vector>` —— 共享的是**读**
    //        权限，类型上就无法从任何持有者处改写。
    //   ⚠ 安全**不**来自"`shared_ptr` 使浅拷贝天然安全"：若 ① 不成立
    //     （`image` 悬在 SDK 缓冲上），这里拷到的是一个会被下一次取帧
    //     改写的头部，而症状是"预览偶尔显示别的时刻的画面" ——
    //     排查方向会指向渲染，而不是取帧时的复制。
    pf.frame = *src;
    // 显式覆盖 role：src 指向 MultiCameraFrame 的某个成员，其 role 字段
    // 依赖采集侧正确填写（漏填时默认 CAM25）。以本次选中的角色为准写入。
    pf.frame.role = role;

    submitFrame(pf);
    return true;
}

PreviewQueue& PreviewManager::queue()
{
    return queue_;
}

const PreviewQueue& PreviewManager::queue() const
{
    return queue_;
}

// ---------------------------------------------------------------------------
// 显示侧出口
// ---------------------------------------------------------------------------

PreviewManager::DeliverResult PreviewManager::workerDeliver(const data::PreviewFrame& frame)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (frame.frame.role != displayCameraLocked())
    {
        ++skipped_;
        return DeliverResult::SKIPPED_ROLE_MISMATCH;
    }

    displayFrame_    = frame;
    hasDisplayFrame_ = true;
    ++displaySequence_;
    return DeliverResult::PUBLISHED;
}

bool PreviewManager::getFrame(data::PreviewFrame& frame)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!hasDisplayFrame_ || displaySequence_ == deliveredSequence_)
    {
        return false;
    }

    frame              = displayFrame_;
    deliveredSequence_ = displaySequence_;
    return true;
}

bool PreviewManager::latestFrame(data::PreviewFrame& frame) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!hasDisplayFrame_)
    {
        return false;
    }

    frame = displayFrame_;
    return true;
}

std::uint64_t PreviewManager::displaySequence() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return displaySequence_;
}

std::uint64_t PreviewManager::displayGeneration() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return displayGeneration_;
}

// ---------------------------------------------------------------------------
// 诊断
// ---------------------------------------------------------------------------

std::uint64_t PreviewManager::publishedCount() const
{
    return displaySequence();
}

std::uint64_t PreviewManager::skippedCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return skipped_;
}

bool PreviewManager::cameraEnabled(data::CameraRole role) const
{
    // 调用者已持锁的路径也会走到这里，故本函数不加锁（只读 rig_）。
    if (rig_ == nullptr)
    {
        return true;  // 无 rig 即不校验（见构造函数的说明）
    }

    const data::CameraChannel* ch = rig_->getCamera(role);
    return ch != nullptr && ch->enabled;
}

std::string PreviewManager::cameraId(data::CameraRole role) const
{
    if (rig_ != nullptr)
    {
        const data::CameraChannel* ch = rig_->getCamera(role);
        if (ch != nullptr && !ch->cameraId.empty())
        {
            return ch->cameraId;
        }
    }
    return fallbackCameraId(role);
}

double PreviewManager::focalLengthMetres(data::CameraRole role) const
{
    if (rig_ == nullptr)
    {
        return 0.0;
    }

    const data::CameraChannel* ch = rig_->getCamera(role);
    return ch != nullptr ? ch->focalLength : 0.0;
}

std::vector<std::string> PreviewManager::configNotices() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return notices_;
}

const optical::OpticalRig* PreviewManager::rig() const
{
    return rig_;
}

// ---------------------------------------------------------------------------

void PreviewManager::noteDisplaySourceChanged(data::CameraRole previous,
                                              data::CameraRole current)
{
    lastDisplayCamera_ = current;

    if (previous == current)
    {
        return;
    }

    // 显示源变了：作废当前显示帧并递增代次。
    //
    // 为什么必须作废：displayFrame_ 里还留着上一路的图像，而界面会按
    // **新的**显示源去写标签。不作废就可能出现"CAM25 的图像 + CAM100 的
    // 标签"，这种错配在界面上与"测量结果不对"完全无法区分。
    // 作废之后 getFrame/latestFrame 返回 false，UI 保持空白直到新焦段
    // 的第一帧到达；同时在途的旧帧会被 workerDeliver 的角色过滤挡掉。
    hasDisplayFrame_ = false;
    ++displayGeneration_;
}

}  // namespace preview
}  // namespace aircraft
