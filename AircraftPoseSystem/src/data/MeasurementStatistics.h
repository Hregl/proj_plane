#pragma once

// ============================================================================
//  src/data/MeasurementStatistics.h
//
//  依据：裁决 C-002（V2.1-C01_架构裁决变更说明.md §C-002，2026-09-23 已批准）
//        ENG-10 §2.4（B 类可用性判据）、§2.5（冲突消解规则）
//        SYS-04 §6.4（result.json 的匹配统计）
//        README §6 第 31 行（8.md §十 的 MatchResult 只有计数 → 改为分项统计）
//
//  作用：一次测量中匹配环节的**统计事实**，是结果包的可追溯内容之一。
//
//  ⚠ 命名：本类型**不叫** MatchResult。
//  MatchResult 是 FeatureMatcher（算法层）的输出，"匹配"是算法内部概念；
//  用它命名 data 层类型，会让 data 层隐含"匹配"这一实现假设 ——
//  而 ENG-01 §5 冻结的 data 只放**值类型与业务事实**，不含算法语义。
//  本类型只声明"这次测量数出了什么"，不声明这些数是怎么数出来的。
//
//  ⚠ 6 个字段**一个都不能裁**（裁决 C-002 明确：不接受裁剪）：
//  它们不是"顺手多带的统计"，而是事后离线分析的**唯一**依据：
//      cadCount / textureCount  → 判断 CAD 辅助与自然纹理各自是否有效。
//           ENG-10 §2.4 的可用性判据与 §2.5 的冲突消解规则都依赖**分项**；
//           只有总数时，"结构点全丢导致退化为纯 A 类"这一情形无法归因
//      droppedByConflict        → 冲突规则的**实际作用量**。恒为 0 说明该规则
//           在这批数据上从未触发（README §6 第 31 行的实测背景）
//      spreadPx                 → 匹配点空间展布，是通道选择 E 分项的原始量。
//           README §6 第 47 行记录：选择阶段尚无匹配结果、只能用检测框作代理，
//           **真实展布只有到这里才有**
//      featureCount / matchCount → 匹配率
//  裁剪任何一项，都会让上表对应的那条分析在事后**无法进行**；而原图一旦归档，
//  统计量不可重算（重算需要重新跑匹配，而匹配依赖当时的内存态）。
//  "当时不留、事后想要"是本项目反复出现的一类数据缺失，代价远高于多存 6 个标量。
//
//  ⚠ matchRatio 是**访问器而非字段**：
//  它是 featureCount 与 matchCount 的导出量。存成字段就存在"两个值不一致"的
//  可能，而结果包一旦自相矛盾就无法自证 —— 与裁决 C-04 删除
//  TurntableState::moving 是同一条理由：同一个事实两处存储，必然出现
//  二者不一致的状态。
//
//  ⚠ 生产者为算法层（FeatureMatcher::MatchResult），经**推送**到达应用层。
//
//  第一条路径走的是"查询"：应用层想读 `PosePipeline::lastMatchResult()`。
//  该方案被评审否定 —— 让 Pipeline 暴露内部历史状态会使接口不断膨胀
//  （lastFeatureResult / lastPnPResult / lastValidationResult……），
//  且直接违反 IPosePipeline 文件头"不允许改动已有签名"。
//  现改为 `IPipelineObserver`：算法层在**产生**统计量的那一刻推送出来，
//  接收方是应用层。取值的形状由"结果输出"决定，不由"内部状态"决定。
//  那组访问器（含 `lastMatchResult()`）已于 2026-09-23 的"死诊断接口
//  清理"中全部删除，本字段的来源从此**只有**推送通道这一条。
//
//  两处字段一一对应，**不做裁剪也不做换算**；单位与量纲保持原样
//  （spreadPx 为 pixel）。
// ============================================================================

namespace aircraft
{
namespace data
{

/// 匹配环节统计（裁决 C-002）。纯标量，不含任何算法语义。
struct MeasurementStatistics
{
    /// 候选特征总数（对应 FeatureMatcher::MatchResult::totalCandidates，
    /// 按裁决 C-002 由评审更名为 featureCount）。
    ///
    /// 语义是"**进入融合与冲突规则之前**的 A+B 类候选对应总数"，
    /// 与 `ImageQuality::featureCount`（本帧提取到的关键点数）**不是同一个量**。
    int featureCount = 0;

    /// 融合后**保留**的对应数。
    ///
    /// ⚠ 它不是内点数 —— 此处尚未经 RANSAC（`MatchResult::matchCount`
    /// 的原义即如此）。**与 `ImageQuality::matchCount` 同名而不同义**：
    /// 后者是"匹配成功的内点数"（过 RANSAC 之后）。
    /// 二者相差一整个 RANSAC 环节，且都叫 matchCount —— 这正是本项目
    /// 反复出现的"字段存在、类型合法、数值看起来正常，但语义不成立"
    /// 的温床：把二者当同一个数用（例如拿本字段去算 ImageQuality::matchRatio）
    /// 会得到一个**能算出来、看上去合理、但毫无意义**的比值，且不报错。
    /// 故本字段的**唯一**消费方式是 `matchRatio()`（下面），
    /// 不得与 ImageQuality 的任何字段互相代入。
    int matchCount = 0;

    /// 因 ENG-10 §2.5 冲突消解规则被丢弃的候选数。
    int droppedByConflict = 0;

    /// 分项：CAD 结构点（B 类）的贡献数。
    int cadCount = 0;

    /// 分项：自然纹理点（A 类）的贡献数。
    int textureCount = 0;

    /// 匹配点的空间展布，单位 pixel。
    double spreadPx = 0.0;

    /// 匹配率 = matchCount / featureCount，[0,1]；featureCount == 0 时返回 0.0。
    double matchRatio() const
    {
        return featureCount > 0
                   ? static_cast<double>(matchCount) / static_cast<double>(featureCount)
                   : 0.0;
    }
};

}  // namespace data
}  // namespace aircraft
