#pragma once

// ============================================================================
//  src/preview/PreviewManager.h
//
//  依据：SYS-01 §12、SYS-03 §7（PreviewManager 职责与数据流）
//        SYS-08 §8（**AUTO 模式的状态→焦段映射表，冻结**）
//        SYS-09 §2.1 / §13.1、ENG-05 §2.1 / §13.1（双流水线、队列策略）
//        ENG-02 §9.1 / ENG-04 §9.1（类设计）、ENG-01 §7（文件清单）
//        ENG-03 §12.4（preview 依赖 data + optical）
//
//  职责（6.md §二）：实时图像缓存、显示源选择、UI 数据输出、预览模式管理。
//  **不负责**：YOLO / SIFT / PnP / 测量流程（6.md §二）。
//
//  ---- 与 Qt 的边界（本文件最重要的一条约定）----
//
//  PreviewManager **不含任何 Qt 代码**，也不持有任何 Qt 类型。
//  依据是两条冻结条目：
//    · ENG-03 §12.4：preview 的依赖是 `data` + `optical`，**没有 Qt**；
//    · ENG-01 §17 的依赖链把 Qt 放在 ui 层（ui → Qt + application + preview）。
//  6.md §12 的 CMakeLists 也只链接 AircraftData。
//
//  ⚠ 与 SYS-09 §15 / ENG-05 §15 的偏离（该两条要求 worker 用
//  "QThread + QObject Worker，不继承 QThread"）：
//  让 preview 链接 Qt 会直接违反 ENG-03 §12.4 这一**针对本模块的**
//  依赖冻结；而 ENG-03 §12 是更具体、且以"哪个模块可以链接什么"为
//  主题的条款，故按"更具体者优先"取 ENG-03。SYS-09 §15 的**禁止项**
//  （不得继承 QThread）本实现完全遵守 —— 本模块根本不用 QThread。
//  该条的目的（生命周期清晰、易测试、信号槽自然）中：
//    · 生命周期清晰：PreviewWorker 用 RAII + join，stop() 返回即线程已退出；
//    · 易测试：tests/preview 无需 QCoreApplication 即可跑完整链路；
//    · 信号槽自然：由 ui 层（009/010）在 QObject 适配器里完成，
//      那里链接 Qt 是冻结允许的（ENG-03 §12.8）。
//
//  ---- 数据流（SYS-03 §7、ENG-04 §9.2 冻结的顺序）----
//
//      Camera ──submitFrame()──▶ PreviewQueue ──waitPop()──▶ PreviewWorker
//                                                                 │
//                                                       workerDeliver()
//                                                                 ▼
//                                    Qt  ◀──getFrame()──── PreviewManager
//
//  队列夹在采集侧与 worker 之间，Manager 是显示侧的终点 ——
//  与 SYS-09 §2.1 的 `Camera ↓ PreviewQueue ↓ PreviewWorker ↓
//  PreviewManager ↓ Qt` 一致。
// ============================================================================

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "data/CameraRole.h"
#include "data/MeasurementState.h"
#include "data/MultiCameraFrame.h"
#include "data/PreviewFrame.h"
#include "optical/OpticalRig.h"
#include "preview/PreviewConfig.h"
#include "preview/PreviewQueue.h"

namespace aircraft
{
namespace preview
{

/// 实时预览管理（SYS-01 §12 / SYS-03 §7 / ENG-02 §9.1）。
class PreviewManager
{
public:
    /// 把一帧交给预览流水线的结果。
    enum class DeliverResult
    {
        PUBLISHED,            ///< 已发布为当前显示帧
        SKIPPED_ROLE_MISMATCH ///< 该帧不属于当前显示源，已丢弃
    };

    /// @param config 预览配置（ENG-09 §6.x）。
    /// @param rig    光机描述，**可为 nullptr**。
    ///        非空时用于两件事：判断某焦段的通道是否 enabled
    ///        （setCamera 拒绝选择已禁用相机）、以及供 UI 取相机标识与
    ///        焦距（getCameraId/focalLengthMetres）。
    ///        为空表示"不校验可用性"，此时 setCamera 一律接受 ——
    ///        这是为了 tests/preview 与早期集成（010）可以先不建 OpticalRig。
    ///        调用方须保证其生命周期长于本对象。
    explicit PreviewManager(const PreviewConfig&    config = PreviewConfig{},
                            const optical::OpticalRig* rig    = nullptr);

    PreviewManager(const PreviewManager&)            = delete;
    PreviewManager& operator=(const PreviewManager&) = delete;

    // ---- 模式 ----

    /// 切换 AUTO / MANUAL（SYS-08 §8）。
    void setMode(PreviewMode mode);
    PreviewMode mode() const;

    // ---- 显示源选择 ----

    /// 设置 MANUAL 模式下的显示源。
    ///
    /// @return false 表示该焦段不可用（rig 非空且其通道 enabled=false），
    ///         此时保持原显示源不变。返回 bool 而非静默接受：
    ///         界面点了"切到 CAM100"而画面不变，若没有返回值，
    ///         操作者只会看到"按钮没反应"。
    ///
    /// ⚠ AUTO 模式下本设置**被记住但不生效**（SYS-08 §8：AUTO 由状态机
    /// 控制预览）。记住它使得切回 MANUAL 时立即生效，无需再点一次。
    bool setCamera(data::CameraRole role);

    /// MANUAL 模式下的显示源（与当前是否处于 AUTO 无关）。
    data::CameraRole camera() const;

    /// 设置 AUTO 模式下"测量阶段"的目标相机，即 SYS-08 §8 表中的
    /// **Selected Camera**（由焦段选择的结果给出，SYS-14）。
    /// 默认与 defaultCamera 相同，故未接测量链时 AUTO 也能工作。
    void setAutoCamera(data::CameraRole role);
    data::CameraRole autoCamera() const;

    /// 上报测量状态机的状态，AUTO 模式据此推导显示源（SYS-08 §8）。
    /// MANUAL 模式下**仅记录不改变显示源** —— SYS-08 §8 明确
    /// "用户控制预览，不影响测量状态机"，反向亦然。
    void setMeasurementState(data::MeasurementState state);
    data::MeasurementState measurementState() const;

    /// 当前**实际**显示源。AUTO 下由 mapStateToCamera() 推导，
    /// MANUAL 下为 camera()。
    ///
    /// 采集侧应当用它来挑选要提交的那一路图像（或直接用 submitFrom），
    /// 以免把三路都塞进预览队列 —— 预览永远只显示一路，
    /// 多提交两路只是白白消耗队列容量与丢帧计数。
    ///
    /// ⚠ 本函数会加锁（可被采集线程调用）。持锁路径上必须改用
    /// displayCameraLocked()，否则自死锁。
    data::CameraRole displayCamera() const;

    /// SYS-08 §8 的 AUTO 映射表，独立暴露以便单测与 UI 复用。
    ///
    /// 冻结的三行是：SEARCH → CAM25、ALIGN → CAM50、
    /// MEASURE → Selected Camera。表未覆盖的中间态按同一物理意图补齐
    /// （见 .cpp 的说明）：搜索与检出前后用大视场、对准与稳定用中焦、
    /// 采集与解算用测量焦段。IDLE → CAM25（SYS-01 §12 默认 CAM25）；
    /// FAILED → 返回传入的 fallback，即**保持原显示源不变**
    /// （故障时把画面切走，会让操作者看不到出问题的那一路）。
    static data::CameraRole mapStateToCamera(data::MeasurementState state,
                                             data::CameraRole selected,
                                             data::CameraRole fallback);

    // ---- 采集侧入口 ----

    /// 提交一帧候选预览帧（非阻塞，满载丢旧保新）。
    ///
    /// ⚠ displayTimestamp 由本函数**统一盖写**为主机单调时钟当前值。
    /// 若交由调用方填写，采集侧与控制侧将对"何时进入预览队列"给出
    /// 不一致的值，而该字段的唯一用途是与 frame.timestampNs 求差得到
    /// 端到端时延（data/PreviewFrame.h），值不一致则延迟统计失真。
    void submitFrame(const data::PreviewFrame& frame);

    /// 从多相机帧中挑出**当前显示源那一路**并提交。
    ///
    /// 存在的意义：MultiCameraFrame 有 cam25/cam50/cam100 三个成员，
    /// 而"当前显示哪一路"是动态的。若让调用方自己写 if/else 选成员，
    /// 漏写其中一支就会固定显示 CAM25，现象是"切换相机没反应"，
    /// 且不会有任何报错。把选择收进本函数，这个错就写不出来了。
    ///
    /// @return false 表示该路图像为空（相机降级或未采集），未提交任何帧。
    bool submitFrom(const data::MultiCameraFrame& frame);

    /// 预览队列（PreviewWorker 从中取帧；诊断与测试也会用到）。
    PreviewQueue&       queue();
    const PreviewQueue& queue() const;

    // ---- 显示侧出口 ----

    /// PreviewWorker 的交付入口：按当前显示源过滤后发布。
    ///
    /// 过滤放在 Manager 而不是 Worker，是因为"当前显示源"是 Manager 的
    /// 状态。Worker 保持成一个只管"取帧→交付"的哑线程，切换相机的
    /// 逻辑就只存在于一处。
    ///
    /// 为什么采集侧已经按 displayCamera() 预筛过、这里还要再筛一次：
    /// 两者之间存在时间窗 —— 采集侧读到 displayCamera() 之后、
    /// 这一帧被 worker 取出之前，用户可能已经切换了相机。若不做第二次
    /// 过滤，切换后画面会先闪一下旧焦段的图像，再跳到新焦段。
    DeliverResult workerDeliver(const data::PreviewFrame& frame);

    /// UI 取帧：**仅当有新帧发布后**返回 true。
    ///
    /// 这样设计是为了配合 Qt 的定时重绘：界面以 60 Hz 定时调用本函数，
    /// 只有返回 true 才重绘。若总是返回当前帧并返回 true，界面就会以
    /// 60 Hz 重绘同一张图，纯属浪费（国产 CPU 上这是可观测的占用）。
    ///
    /// ⚠ 单消费者约定：服务于**一个**调用者（UI 线程）。它记录了
    /// "上次已交付的序号"，若从两个线程调用，先调者会把帧消费掉，
    /// 后者永远拿不到。UI 侧只有一个预览控件，故成立；
    /// 若将来出现第二个消费方，应各自持有 latestFrame() 并按
    /// displaySequence() 自行判重。
    virtual bool getFrame(data::PreviewFrame& frame);

    /// 取当前显示帧，**不论新旧**（不改变交付序号）。
    /// 供切换相机后重绘、面板按需取图等场合使用。
    /// @return false 表示自上次切换显示源以来还没有发布过任何帧。
    bool latestFrame(data::PreviewFrame& frame) const;

    /// 已发布帧的序号，每次 workerDeliver 成功即 +1。
    std::uint64_t displaySequence() const;

    /// 显示源代次：每当**实际显示源发生变化**（模式切换、用户切相机、
    /// AUTO 下状态推进到另一个焦段）即 +1。
    ///
    /// 用途：显示源一变，上一路的最后一帧就不该再停留在画面上了 ——
    /// 否则界面会用新焦段的标签配旧焦段的图像，而这种错配看起来
    /// 完全像是"测量结果不对"。UI 在发现代次变化时应先清空控件。
    std::uint64_t displayGeneration() const;

    // ---- 诊断 ----

    /// 累计发布帧数。
    std::uint64_t publishedCount() const;

    /// 累计因焦段不匹配被 workerDeliver 丢弃的帧数。
    std::uint64_t skippedCount() const;

    /// 某焦段的通道是否可用（rig 为空时恒为 true）。
    bool cameraEnabled(data::CameraRole role) const;

    /// 相机标识（"cam25" 等）；rig 为空或角色缺失时返回 "CAM25"/"CAM50"/
    /// "CAM100" 这一兜底名字，便于界面始终有东西可显示。
    std::string cameraId(data::CameraRole role) const;

    /// 该焦段镜头的焦距，单位 **米**（ENG-09 §2.3）；
    /// rig 为空时返回 0.0，界面应显示为"未知"而不是 "0.00 m"。
    double focalLengthMetres(data::CameraRole role) const;

    /// 构造期的配置问题记录（例如 queueSize 越界被夹取）。
    /// 与 CalibrationManager::lastWarnings() 同一约定：
    /// 降级与配置疏漏必须可被读出，不得静默。
    std::vector<std::string> configNotices() const;

    const optical::OpticalRig* rig() const;

private:
    /// displayCamera() 的**不加锁**版本。
    ///
    /// 必须分成两个函数而不是让 displayCamera() 自己 "按需加锁"：
    /// setCamera/setMode 等已经持锁，若再调加锁版就是 std::mutex 上
    /// 的自死锁（非递归锁），表现为界面一点相机按钮就整个卡死 ——
    /// 而这种卡死不会产生任何错误码或日志，只会"没反应"。
    data::CameraRole displayCameraLocked() const;

    /// 记录一次显示源变化：清空显示槽并递增代次。调用者须持锁。
    void noteDisplaySourceChanged(data::CameraRole previous,
                                  data::CameraRole current);

    PreviewConfig              config_;
    const optical::OpticalRig* rig_ = nullptr;

    PreviewQueue queue_;

    mutable std::mutex mutex_;

    PreviewMode           mode_               = PreviewMode::AUTO;
    data::CameraRole      manualCamera_       = data::CameraRole::CAM25;
    data::CameraRole      autoCamera_         = data::CameraRole::CAM25;
    data::MeasurementState state_             = data::MeasurementState::IDLE;

    /// 上一次推导出的显示源，用于识别"显示源变化"这一事件。
    /// 单独存一份而不是每次重算，是因为变化**事件**（边沿）无法由
    /// 当前值（电平）恢复 —— 代次递增与清空显示槽都依赖它。
    data::CameraRole      lastDisplayCamera_  = data::CameraRole::CAM25;

    data::PreviewFrame    displayFrame_;
    bool                  hasDisplayFrame_    = false;
    std::uint64_t         displaySequence_    = 0;
    std::uint64_t         deliveredSequence_  = 0;
    std::uint64_t         displayGeneration_  = 0;
    std::uint64_t         skipped_            = 0;

    std::vector<std::string> notices_;
};

}  // namespace preview
}  // namespace aircraft
