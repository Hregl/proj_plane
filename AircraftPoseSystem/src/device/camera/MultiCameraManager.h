#pragma once

// ============================================================================
//  src/device/camera/MultiCameraManager.h
//
//  依据：SYS-06 §5（MultiCameraManager）、§8（三相机同步采集）
//        SYS-08 §7.5（硬件故障与降级）
//        ENG-09 §4.3、ENG-01 §6、ENG-02 §2.3 / ENG-04 §2.3（依赖注入）
//
//  ⚠ 与工作流文档 4.md 的两处偏离：
//
//  偏离 A —— 构造函数带参（4.md 为 `MultiCameraManager();` 无参，
//  并在内部 new 出三个 backend）：
//
//    本工程改为**依赖注入**，理由不是设计偏好而是**测试需求**：
//    SYS-08 §10 的收敛性测试表明确要求
//      "上述用例依赖注入式桩（VirtualCameraBackend / VirtualTurntable /
//        算法桩）"
//    其中"硬件故障不重试"用例要求"中途将 CAM100 置为断连"、
//    "降级下限"用例要求"断开两台相机"。若 manager 在内部写死
//    `new ImvCameraBackend(...)`，则测试无法把某个通道换成可控桩，
//    上述两个必测用例**无法实现**。
//    同时 ENG-02 §2.3 / ENG-04 §2.3 明确要求依赖注入。
//
//  偏离 B —— 增加了三个**非虚查询方法**（availableCameraCount /
//  degraded / state）：
//    冻结接口 IMultiCameraManager（SYS-06 §5.2）只有 4 个方法，
//    本类**完整实现且未修改**该契约；新增的查询是具象类自己的能力，
//    用于满足 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 对结果包的要求
//      "result.json 记录 degraded=true 与 cameras_available"
//    以及"降级必须在 UI 上可见，不得静默降级"。
//    任何按冻结接口编写的消费者都不受影响（新增方法不改变虚表契约）。
//
//  ⚠ 本类**不做同步判据**。SYS-06 §8 把"时间戳检查、丢帧检测、同步验证"
//  列为软件职责，但该职责归属 optical 层的 CameraSynchronizer
//  （ENG-01 §8），本类只负责"把三路图取回来并填好时间戳"。
//  这样切分的理由：同步判据需要 TriggerConfig::syncToleranceNs，
//  而 device 层不应依赖配置的语义解释；且同步失败时应由上层决定
//  "本轮作废重采"还是"降级接受"，这属于应用层策略。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "data/CameraChannel.h"
#include "data/CaptureRound.h"
#include "data/DeviceState.h"
#include "data/ErrorInfo.h"
#include "data/MultiCameraFrame.h"
#include "data/OpStatus.h"
#include "device/camera/ICameraBackend.h"
#include "device/camera/IMultiCameraManager.h"

namespace aircraft
{
namespace device
{

/// 三相机管理器（SYS-06 §5）。
class MultiCameraManager : public IMultiCameraManager
{
public:
    /// 三个通道各自注入一个 backend。
    ///
    /// @param cam25  CAM25（25 mm）通道的 backend。
    /// @param cam50  CAM50（50 mm）通道的 backend。
    /// @param cam100 CAM100（100 mm）通道的 backend。
    ///
    /// **允许传入 nullptr**，语义为"该通道无相机"。这不是容错让步，
    /// 而是三个必测场景的前提（SYS-08 §10）：
    ///   - 单相机故障  → 某一路 backend->initialize() 返回 false；
    ///   - 降级下限    → 只注入 1 个 backend（或 2 个后断掉 1 个）；
    ///   - 无硬件开发  → 只注入 VirtualCameraBackend（M1 的实际形态）。
    ///
    /// 若改为"必须三个非空"，则上述场景只能靠"先构造再设法使其失败"，
    /// 而"断开两台相机"这一动作就无从表达。
    explicit MultiCameraManager(std::shared_ptr<ICameraBackend> cam25,
                               std::shared_ptr<ICameraBackend> cam50,
                               std::shared_ptr<ICameraBackend> cam100);

    // ---- IMultiCameraManager ----

    bool initializeAll() override;
    bool startAll() override;
    void stopAll() override;
    bool capture(data::MultiCameraFrame& frame, uint64_t deadlineNs) override;
    data::CaptureRound lastCaptureRound() const override;

    // ---- 具象类追加能力（011-A1）----

    /// 注入当前时间的来源（默认 `data::monotonicNowNs`）。
    ///
    /// ⚠ 形状**照抄仓内既有先例** `VirtualTurntable::setClock`
    /// （`src/device/turntable/VirtualTurntable.h:72`，已被
    /// `tests/device/DeviceLayerTest.cpp` 使用）—— 本仓没有任何通用时钟
    /// 接口（`IClock`／`std::function<int64_t>` 在管理器与控制器中零命中），
    /// 故不为这一个用途新造抽象。
    ///
    /// 为什么必须有这个 seam：本批起 `capture()` 要按**绝对期限**判定
    /// 是否还值得取帧。测试把模拟时间从 0 推进（既有用例全是
    /// `tick(now); now += step;` 的形态），若管理器直读真实单调钟，
    /// 模拟期限会被真实墙钟**一上来就判过期**，整套时序用例会以与本次
    /// 改动无关的原因变红。
    ///
    /// ⚠ 生产与测试**各自只用一种来源**，不得在同一段逻辑里混用：
    /// 控制器、管理器、测试共用同一个时间域。
    void setClock(std::function<uint64_t()> clock);

    /// 注入**采集诊断**的出口（默认空 ⇒ 什么都不做）。
    ///
    /// ⚠ 为什么是注入而不是本类直接写日志：`src/device/` 全树**不包含**
    /// `Logger.h`（ENG-01 §17：device 不依赖 infrastructure），而本类
    /// 恰恰是**唯一**掌握"这一路取帧现场"的地方（`GrabResult::diagnosis`、
    /// 触发读回、长度不符的具体数值）。若不给出出口，这些现场就只剩
    /// "留在后端内部、管理器从不读"这一条路 —— 于是离线排查看不到
    /// 任何可核对的数字。注入把"记录"这件事交给上层（app 层接到
    /// `Logger::warn("device", …)`），分层不变。
    ///
    /// ⚠ 形状照抄 `setClock`（本仓唯一的注入先例）。
    ///
    /// ⚠ 输出**只针对需要记录的事件**（该路 `status != Ok` 或本次诊断
    /// 非空时才输出）：全部正常的一轮**不产生任何输出** ——
    /// 逐路写 WARN 会让日志被正常帧淹没，而"异常才有行"正是它能被
    /// 当成信号的前提。
    void setDiagnosticSink(std::function<void(const std::string&)> sink);

    /// 设置取帧预算（由装配点从 `MeasurementConfig` 注入）。
    ///
    /// @param perGrabTimeoutMs 单次取帧上限（传给 `ICameraBackend::grab`）。
    ///        **不接受 0** —— SDK 对 `timeoutMS = 0` 的语义未文档化
    ///        （§2.5 已核：`IMV_GetFrame` 的参数注释只提 `INFINITE`，
    ///        而 `INFINITE` 宏甚至未定义），故项目也不定义它。
    ///        传 0 ⇒ 各路按 `InvalidArgument`（本地判定，未调用 SDK）处置，
    ///        **不是**静默当成"无限等待"。
    /// @param groupBudgetNs 一次 `capture()` **三路合计**的总预算。
    void setGrabBudget(uint32_t perGrabTimeoutMs, uint64_t groupBudgetNs);

    // ---- 具象类追加查询（见文件头"偏离 B"）----

    /// 当前可用相机数（已成功 initialize 且未被禁用），取值 0~3。
    /// 即 SYS-08 §7.5 表中记录的 `cameras_available`。
    int availableCameraCount() const;

    /// 是否处于降级状态：可用相机数在 1~2 之间（<3 但 ≥2）。
    /// 即 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求写入 result.json 的 `degraded` 字段。
    /// 可用数为 0~1 时不返回 true —— 那是失败而非降级
    /// （§7.5：≤1 直接 FAILED，code=1001）。
    bool degraded() const;

    /// 设备状态（SYS-06 §14）。任一路 backend 处于故障态即整体 ERROR；
    /// 全部就绪且已 start 为 RUNNING；仅供参考与 UI 显示，
    /// **不参与降级判定**（降级判定以 availableCameraCount() 为准）。
    data::DeviceState state() const;

    /// 最近一次失败的原因。capture() 返回 false 后由上层读取，
    /// 用于填充状态机的 ErrorInfo（IMultiCameraManager 的同名接口）。
    data::ErrorInfo lastError() const override;

    /// `exposureIndex` 是否已退化为内部计数（设备帧号不前进或回退）。
    ///
    /// 置位后**不自动清除**：它表达的是"本次运行期间这件事发生过"，
    /// 与"当前这一轮的序号从哪来"不同 —— 后者每次都可能变化，
    /// 而"这个系统是否可信地给出了曝光序号"是一个累积事实。
    bool exposureIndexDegraded() const;

    /// 把指定通道显式标记为不可用（禁用）。
    ///
    /// 用途：SYS-08 §7.5 的"禁用故障相机"，以及 §10 的
    /// "中途将 CAM100 置为断连"这一测试动作。
    /// 判据是 CameraChannel::enabled 的运行时对应物
    /// （裁决 C-03：enabled 的存在意义就是承载本降级机制）。
    ///
    /// @return false 表示该角色本就没有 backend（无可禁用者）。
    bool disableChannel(data::CameraRole role);

    /// 某一路当前是否可用（011-A1 新增）。
    ///
    /// ⚠ **不是** `IMultiCameraManager` 的虚方法：本方法存在的理由是装配层
    /// 需要**逐路**的就绪事实，而那个接口的既有方法面（`initializeAll` /
    /// `startAll` / `stopAll` / `capture` / `lastError` + 本批新增的
    /// `lastCaptureRound`）只给出**合计**。故与后端类的 `state()` 一样，
    /// 它是具象类各自的查询能力，不进冻结 ICD。
    ///
    /// 为什么装配层需要它（而不是自己数）：装配要回答两个问题——
    ///   ① 装配摘要里每一路到底就绪没有（"三路都装配了"与"三路都能用"
    ///      必须分开写）；
    ///   ② **显式请求的真实后端若没打开，整次启动必须失败** —— 这一条
    ///      不能靠"另外两路虚拟相机还在"来满足（`initializeAll()` 的
    ///      判据是**合计** ≥2，2 虚拟 + 1 失败的真实 = 恰好 2 ⇒ 合计
    ///      达标，而"真实接入成功"却是不成立的）。
    /// 这两个问题的答案只有本类知道，故问本类，而不在装配层重数一遍。
    bool channelAvailable(data::CameraRole role) const;

private:
    /// 一个通道的运行时状态。把"backend 指针"与"是否可用"绑在一起，
    /// 保证二者不会失配 —— 若分成两个并列成员（如 backends_[3] 与
    /// available_[3]），则存在"backend 非空但可用标志为真却从未成功
    /// 初始化"的中间态，而该状态会让 availableCameraCount() 偏大，
    /// 使降级判据误判为"3 台正常"。
    struct Channel
    {
        std::shared_ptr<ICameraBackend> backend;
        data::CameraRole role = data::CameraRole::CAM25;
        bool available = false;
    };

    /// 取该角色对应的通道；有效角色恒定返回非空指针。
    Channel&       channelOf(data::CameraRole role);
    const Channel& channelOf(data::CameraRole role) const;

    /// 把一条**本轮采集诊断**交给出口（未注入时为 no-op）。
    ///
    /// ⚠ 单一出口：本类写诊断**只经本方法**，使"设备层不直接持有日志"
    /// 这条分层约束在一处可核。
    void emitDiagnostic(const std::string& text);

    /// 当前时间（ns），来自 `clock_`。
    ///
    /// ⚠ 全类**只经本方法**读时间，且**不缓存**任何时刻：
    /// 缓存会让 `startedNs`／`finishedNs` 反映"入口那一次"而不是
    /// "真的什么时候结束"，本批正是要测这段实际耗时（§2.2 删除
    /// "组预算＝GUI 线程硬上限"承诺之后它成为唯一可测事实）。
    uint64_t nowNs() const;

    /// 采集单路；成功则写入 frame 中对应的字段。
    ///
    /// ⚠ 本函数**只取帧，不发软件触发令**（§2.4 单一执行者）。
    /// 发令由 `capture()` 在调用本函数**之前**完成 ——
    /// "等待一帧"不等于"发出了一次软件触发"，这两件事必须在代码上
    /// 也分开，否则以后有人说"加个重试吧"时，会在本函数里再发一次令。
    ///
    /// @param timeoutMs 已经取过最小的单次等待上限（见 capture() 的推导）。
    /// @return 完整结果（分类 + 原码）；调用方据分类决定是否禁用该通道。
    data::GrabResult grabOne(Channel&          ch,
                             data::ImageFrame& out,
                             uint64_t&         timestampOut,
                             uint32_t          timeoutMs);

    /// 产出本轮的 `MultiCameraFrame::exposureIndex`（裁决 D-C02-6）。
    ///
    /// 做法（首帧偏移锁定）：每路首次采到时记下它的 `ImageFrame::frameId`
    /// 作为该路基准，之后取 `frameId − 基准`；组的值取**参考路**
    /// （按 CAM25→CAM50→CAM100 顺序的第一路可用者）的偏移量，再 +1
    /// 使其 **1 起**计数（0 保留给"未定"，见 MultiCameraFrame 的说明）。
    ///
    /// ⚠ 为什么不用三路 `frameId` 相等：那是每台相机**自己的**计数器，
    /// 三台起点不同，真机上恒不成立。偏移锁定把"各自的起点"归一化到
    /// "本机的第几次曝光"，虚拟相机（计数器从 0 起）与真实相机同时成立。
    ///
    /// ⚠ 某路丢帧会让该路偏移量跳变；本方法**只取参考路**，不做跨路
    /// 一致性判定 —— 那属于 `optical::CameraSynchronizer`（SYS-06 §8），
    /// 本类不重复推断（见文件头）。
    ///
    /// @param frame 已填好三路图像与本轮真正采到的路掩码 `present`
    /// @return 本轮的曝光序号；无任何可用路时返回 0（未定）
    uint64_t updateExposureIndex(const data::MultiCameraFrame& frame,
                                 const bool present[3]);

    Channel cam25_;
    Channel cam50_;
    Channel cam100_;

    /// 各路 `frameId` 的首帧基准与是否已锁定，索引 0/1/2 = CAM25/50/100。
    /// 与 `Channel` 分开存放：`Channel` 表达"这条通道能不能用"，
    /// 基准表达"这台相机从哪个数开始计数"，二者生命周期不同
    /// （通道可被 disable 后重新 enable，而基准一旦锁定就不该重来）。
    uint64_t frameIdBase_[3]       = {0, 0, 0};
    bool     frameIdBaseLocked_[3] = {false, false, false};

    /// 上一次产出的曝光序号。用于保证**单调递增**这条不变量。
    uint64_t exposureIndex_ = 0;

    /// 是否发生过"设备帧号不前进 / 回退"而被迫改用内部计数。
    ///
    /// ⚠ 必须可见：这表示"曝光序号不再来自设备帧号"，即
    /// D-C02-6 的语义在本次运行中已退化。若不可见，一个恒为 1 的
    /// `exposureIndex` 会一路写进结果包而不报错 ——
    /// 而它承载的正是"这几张 raw 是不是同一次曝光"这个判断。
    bool exposureIndexDegraded_ = false;

    /// 是否已成功 start()。capture() 在未 start 时返回 false ——
    /// 否则会去调用 backend 的 grab()，而 backend 在未 start 时的
    /// 行为由各 SDK 决定（通常返回错误或阻塞），把 SDK 的行为差异
    /// 泄漏成上层可见的不确定行为。
    bool started_ = false;

    /// 当前时间的来源（注入）。默认单调钟，测试注入假钟。
    /// 只读不改：本成员**不缓存任何期限**（期限只从 capture() 的形参来）。
    std::function<uint64_t()> clock_;

    /// 采集诊断的出口（注入）。默认空 ⇒ no-op。
    /// ⚠ 本类**不缓存诊断文本**：它表达的是"刚刚发生了什么"，
    /// 缓存会让下一次读取拿到一条已经不成立的现场描述。
    std::function<void(const std::string&)> diagnosticSink_;

    /// 单次取帧上限（ms）。默认 100 —— 与 MeasurementConfig 的默认值一致，
    /// 使未显式注入的既有调用方行为不变。
    uint32_t perGrabTimeoutMs_ = 100;

    /// 一次 capture() 三路合计的总预算（ns）。默认 3.0e8（300 ms）。
    uint64_t grabGroupBudgetNs_ = 300000000ULL;

    /// 最近一轮 capture() 的每路结果与聚合（`lastCaptureRound()` 的返回）。
    /// 每轮覆盖一次 —— 它表达"**本轮**发生了什么"，不是累积状态。
    data::CaptureRound lastRound_;

    data::ErrorInfo lastError_;
};

}  // namespace device
}  // namespace aircraft
