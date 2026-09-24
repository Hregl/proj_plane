#pragma once

// ============================================================================
//  src/algorithm/pipeline/IPipelineObserver.h
//
//  依据：裁决 C-002（MeasurementRecord.statistics 的来源）
//        V2.1-C02_实施设计说明.md §11.1（Step 5 的阻塞项）
//        SYS-04 §7 第 1 条（IF-SW-01~05 的**签名**变更视为不兼容变更）
//        SYS-04 §4.2（IF-SW-02：application ↔ algorithm 的唯一调用面）
//        ENG-10 §2.4（B 类可用性判据依赖 cadCount / textureCount 分项）
//        ENG-01 §18（依赖方向：application → algorithm）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本接口解决的问题：`MeasurementRecord::statistics` 需要一个来源。      │
//  │ 它**不是**"让外部去查算法内部状态"，而是"算法在产生统计量的那一刻   │
//  │ 把它推出来"。这个区别不是措辞问题，它决定了接口会不会膨胀。         │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 为什么**不**给 IPosePipeline 加 `lastMatchResult()`（评审否决的第一版方案）：
//    1. 那是"内部状态查询"：Pipeline 的职责是 输入 → 计算 → 输出，
//       不是"把历史结果存起来等人问"。开了这个口，接下来必然是
//       `lastFeatureResult()` / `lastPnPResult()` / `lastValidationResult()`，
//       接口逐个膨胀，而每一个都只在**某一条**调用路径之后才有意义
//       （"上次调用"到底指哪次，接口本身说不清）；
//    2. 它与本项目已修复的一类缺陷同源但方向相反：先前是"算出来了，
//       但没接到下游"，若用"让算法暴露更多内部状态"来解，等于把
//       数据链断裂的责任从应用层挪到算法层，断口依旧存在；
//    3. 实测 `PosePipeline::lastMatchResult()` 在**生产代码里零调用者**，
//       只有测试在读它 —— 也就是说它当时已经是一个只服务于"测试想知道
//       内部发生了什么"的出口，再加一个同类的只会再多一个。
//       （2026-09-23 的"死诊断接口清理"把这一整组 6 个访问器都删了：
//        `lastCadResult` / `lastCadAssisted` 全工程零消费者，其余四个
//        **仅测试**在消费。删除不涉及任何冻结接口，因为这一组本就不属
//       IF-SW-02 —— 见 PosePipeline.h 中该处的说明。）
//
//  ⚠ 为什么**不**给 `solvePose` 加输出参数（评审给出的"临时方案"）：
//    那会改动 IF-SW-02 一个**已冻结方法**的签名。`IPosePipeline.h` 文件头
//    明文："**不允许**删除或改名已有方法、改动已有签名或返回类型
//    （SYS-04 §7 第 1 条）"。而本接口是**新增纯虚方法**，
//    文件头同样明文允许（"允许增加纯虚方法"，`setCoarseAttitude` 是先例），
//    且 `solvePose` 的签名一字未动 —— 不需要登记任何不兼容变更。
//
//  ⚠ 推送时机：在算法**算出**统计量的那一刻（`solvePose` 内部完成匹配之后），
//    不是在调用返回之后。这使统计量与产出它的那次计算**同源** ——
//    若等到返回后再取，中间任何一次重算都会让"记录里的统计"与
//    "记录里的姿态"指向两次不同的计算（这正是 C-02 修掉的 D-C02-1 的形态）。
//
//  ⚠ 线程：`IPosePipeline` 的全部方法都是同步阻塞的（见其文件头），
//    故 `onStatistics()` 在**调用方线程**上被调用，不会跨线程。
//    实现方不需要加锁，但**不得**在回调里重入 Pipeline（未定义）。
//
//  ⚠ 观察者是**可选**的：未设置时算法照常工作，只是没有统计量输出。
//    因此"忘记接观察者"是一个**静默**失败 —— 故引入它的同时必须有一条
//    测试断言统计量确实送达（见 tests/integration/MeasurementFlowTest.cpp），
//    否则它会变成又一个"算出来了但没人读"。
// ============================================================================

#include "data/MeasurementStatistics.h"

namespace aircraft
{
namespace algorithm
{

/// 算法链统计量的推送出口（裁决 C-002）。
///
/// 实现在**应用层**（`MeasurementController`）：它持有本次测量的记录，
/// 把收到的统计量填进 `data::MeasurementRecord::statistics`。
///
/// ⚠ 依赖方向：本接口定义在 algorithm 层（生产者一侧），
/// 由 application 层实现（ENG-01 §18 允许 application → algorithm）。
/// 反过来定义在 application 会让 algorithm 反向依赖 —— 与
/// `IRecorderSink` / `Recorder` 是同一类落层问题，但这里的结论相反：
/// 那个是"消费者在 application"，故只能由 app 侧适配器桥接；
/// 本接口的**消费者是生产者自己**（算法主动调用），定义在算法层不需要
/// 任何适配器。
class IPipelineObserver
{
public:
    virtual ~IPipelineObserver() = default;

    /// 一次姿态解算过程中匹配环节的统计量。
    ///
    /// @param statistics 与本次 `solvePose` **同源**的统计量
    ///        （见 MeasurementStatistics 的字段说明）。
    ///
    /// @note 只应在**确实算出了**统计量时调用。算法在匹配之前失败
    ///       （图像为空、无对应）时**不应**推送一个全零的统计量 ——
    ///       全零在这里是合法值，推送它等于把"没算"伪装成"算了但是 0"。
    ///       调用方据此把"未收到推送"与"收到了 0"区分开。
    virtual void onStatistics(const data::MeasurementStatistics& statistics) = 0;
};

}  // namespace algorithm
}  // namespace aircraft
