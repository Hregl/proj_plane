#pragma once

// ============================================================================
//  src/data/IMatchStatsStore.h
//
//  依据：ENG-10 §4（历史匹配成功率统计）、ENG-09 §6.5（分桶边界冻结）
//        裁决 C-21 / ENG-10 §5.1（接口落层原则）
//
//  ⚠ 落层理由（ENG-10 §5.1 的组织原则："跨层的接口必须落在依赖关系的
//      最低公共层"）：
//
//  本接口的**使用者**是 algorithm 层的 MeasurementCandidate 评分链
//  （M 分项 = M_hist，ENG-10 §3.3），而 M_hist 又是 E 项中 N_est 的折扣
//  系数 —— 也就是说评分链直接依赖它。
//  本接口的**实现者**是 infrastructure 层的 FileMatchStatsStore
//  （文件读写，落盘到 runtime/match_stats.yaml）。
//
//  algorithm 与 infrastructure 之间**没有依赖关系**（ENG-01 §17 的链
//  `... → algorithm → infrastructure → data` 中 algorithm 在 infrastructure
//  之上，反向依赖被 ENG-01 §18 禁止）。因此：
//    - 接口若放 algorithm：infrastructure 无法实现它（要依赖上层）
//    - 接口若放 infrastructure：algorithm 无法引用它（同样是向上依赖）
//  唯一可行位置是两者的公共下层 —— **data 层**。
//
//  ⚠ ENG-01 §5 的 data 文件清单中已包含本文件名，即该裁决已被目录结构
//  文档确认，不是本次实现的自创。
//
//  ⚠ 本接口只含抽象方法，不含数据成员 —— data 层的"值类型"约束
//  （ENG-09 §2.7）针对的是结构体；纯接口没有状态，不与之冲突。
// ============================================================================

#include <cstdint>
#include <string>

namespace aircraft
{
namespace data
{

/// 历史匹配成功率（M_hist）的读写接口（ENG-10 §4）。
///
/// 数据组织（ENG-10 §4.1~§4.2，冻结）：
///   按 target_model_id 分组，每组 36 个桶
///   = 3 个 CameraRole × 4 个距离带 × 3 个光照带。
///
/// 读写时机（ENG-10 §4.4，冻结 —— 这是本接口最重要的使用约束）：
///   **任务期间只读，仅在 SAVE / FAILED 时写回。**
///   原因：M_hist 参与评分，而评分决定选中哪台相机。若采集过程中就写回，
///   则同一任务内后续帧的评分会受本任务早先帧的影响，
///   使"重试"不再是独立重复实验 —— SYS-08 §7.3 的重试上限
///   （如 ALIGN 8 次）就失去了统计意义：第 8 次尝试面对的是一个
///   已经被前 7 次污染的评分表。
///
/// 写入方式（ENG-10 §4.5）：临时文件 + rename（原子替换）。
/// 解析失败处理（ENG-10 §4.6）：重命名为 match_stats.yaml.corrupt.<ts>
///   并从空表重启 —— **不得静默丢弃**（否则统计量会无声退化到冷启动先验，
///   而使用者无从知道）。
class IMatchStatsStore
{
public:
    virtual ~IMatchStatsStore() = default;

    /// 读取指定分桶的历史成功率。
    ///
    /// @param targetModelId 机型 + 外形状态标识（TargetModel::modelId）。
    ///        不同机型**必须**使用独立的统计表（ENG-10 §4.1）——
    ///        混用会让 M_hist 反映另一架飞机的纹理可匹配性。
    /// @param role   相机角色（25 / 50 / 100 mm）。
    /// @param distanceBand 距离带索引，[0,3]，边界见
    ///        MeasurementConfig::distanceBandEdges = {80,150,220} m。
    /// @param illumBand    光照带索引，[0,2]，边界见
    ///        MeasurementConfig::illumBandEdges。
    /// @return M_hist ∈ [0,1]。
    ///         样本数 < N_min（MeasurementConfig::matchStatsMinSamples，
    ///         默认 10）时返回收缩估计值，样本为 0 时退化为冷启动先验
    ///         matchStatsColdStartPrior（默认 0.5）。
    ///         索引越界返回冷启动先验，不抛出 —— 分桶索引由评分链
    ///         依据同一份配置计算，越界说明配置与调用点不一致，
    ///         返回先验可让流程继续并由上层日志暴露问题。
    virtual double successRate(const std::string& targetModelId,
                               int role,
                               int distanceBand,
                               int illumBand) const = 0;

    /// 累加一次观测结果。**仅在任务结束（SAVE / FAILED）时调用**
    /// （ENG-10 §4.4，见类注释）。
    ///
    /// @param success 本次是否成功（matched / total 的分子增量）。
    virtual void record(const std::string& targetModelId,
                        int role,
                        int distanceBand,
                        int illumBand,
                        bool success) = 0;

    /// 把内存中的统计表写回持久化介质。
    /// 实现须为原子替换（临时文件 + rename），避免写一半掉电后
    /// 留下一个既非旧值也非新值的文件。
    virtual bool save() = 0;

    /// 从持久化介质加载。解析失败须按 ENG-10 §4.6 改名保留现场后
    /// 以空表启动，返回 false 让调用方记录日志。
    virtual bool load() = 0;
};

}  // namespace data
}  // namespace aircraft
