#pragma once

// ============================================================================
//  src/preview/PreviewConfig.h
//
//  依据：6.md §三 / §四、ENG-09 §6.x（配置类型）、SYS-09 §13.1（队列容量）
//        ENG-01 §7（preview 模块冻结文件清单）
//
//  ⚠ 与 6.md 的一处结构偏离：`enum class PreviewMode` 按 6.md §三 单列
//  一个 PreviewMode.h，本实现把它与 PreviewConfig 合并到本文件。
//
//  理由：ENG-01 §7 把 preview 模块的冻结文件清单逐字固定为
//    PreviewManager.h / PreviewQueue.h / PreviewWorker.h / PreviewConfig.h
//  四项，且 ENG-09 §249 明确"preview 的类清单沿用 ENG-01 §7~§14，
//  唯一变更为 algorithm/selection/MeasurementSelector"——即该清单是
//  排他的。**全部冻结文档（ENG-02 §9、ENG-04 §9、SYS-01 §12、
//  SYS-03 §7、SYS-09）都没有出现过 PreviewMode 或 PreviewProvider**，
//  它们是 6.md 自己的增补。
//  按"冻结文档优先于工作流文档"的处理规则，不新增该文件；
//  PreviewMode 只有 2 个取值、且 6.md 自己就把它当作 PreviewConfig 的
//  一个字段（§四 的 mode），合并在语义上也是自洽的。
//
//  同理，6.md §六 的 PreviewProvider.h 未予建立：冻结文档从未提及该接口，
//  而 ENG-02 §2.3 / ENG-04 §2.3 的注入约定是 UI 直接持有具体的
//  PreviewManager。为保住"UI 不依赖具体实现"这一意图，PreviewManager::
//  getFrame() 声明为 virtual（见 PreviewManager.h），使 UI 侧测试可以
//  提供替身，而不必为此在冻结清单外新增一个文件。
// ============================================================================

#include "data/CameraRole.h"

namespace aircraft
{
namespace preview
{

/// 预览模式（SYS-01 §12、SYS-03 §7、SYS-08 §8）。
enum class PreviewMode
{
    /// 由测量状态机决定显示源（SYS-08 §8 的三行映射表）。
    AUTO,

    /// 由用户选择显示源。**不影响测量状态机**（SYS-08 §8）。
    MANUAL
};

/// 预览配置（ENG-09 §6.x）。
struct PreviewConfig
{
    /// 初始显示源。SYS-01 §12 / SYS-03 §7 / SYS-01 §12 均冻结"默认 CAM25"。
    data::CameraRole defaultCamera = data::CameraRole::CAM25;

    /// 预览模式，默认 AUTO。
    PreviewMode mode = PreviewMode::AUTO;

    /// 预览队列容量。
    ///
    /// ⚠ SYS-09 §13.1 / ENG-05 §13.1 冻结该值为 **3~5**：
    /// 容量的物理含义是"允许积压多少帧"，按 60 fps 计，容量 5 相当于
    /// 最多 83 ms 的显示延迟，容量 100 则接近 1.7 s —— 预览会变成
    /// "回放过去"。故 PreviewQueue 会把越界值夹取到 [3,5] 并记录，
    /// 而不是照单全收（也不是拒绝启动：预览不是测量通路，
    /// 一个队列容量配置不当不应使整套系统无法开始工作）。
    int queueSize = 5;
};

}  // namespace preview
}  // namespace aircraft
