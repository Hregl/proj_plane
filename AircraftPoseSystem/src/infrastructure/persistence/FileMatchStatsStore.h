#pragma once

// ============================================================================
//  src/infrastructure/persistence/FileMatchStatsStore.h
//
//  依据：ENG-01 §11（persistence/，裁决 C-21）、ENG-10 §4（历史匹配成功率）、
//        §4.5（临时文件 + rename）、§4.6（解析失败隔离）、
//        data/IMatchStatsStore.h（实现的接口，落层理由见该文件）
//
//  本类是 `data::IMatchStatsStore` 的**唯一实现**：接口在 data/，
//  实现在 infrastructure/ —— 因为文件 I/O 是 infrastructure 的职责，
//  而算法层（MeasurementSelector）不能反向依赖 infrastructure。
//
//  分桶结构（ENG-10 §4.2，冻结）：
//      按 target_model_id 分组，每组 3 角色 × 4 距离带 × 3 光照带 = 36 桶。
//      桶内保存 attempts / successes 两个计数（而非直接存成功率）——
//      因为 M_hist 用收缩估计，需要样本数 n 才能算
//          M = (n·p̂ + N_min·prior) / (n + N_min)
//      只存成功率会让"观测 1 次成功率 1.0"与"观测 100 次成功率 1.0"
//      变得不可区分，而前者根本不该被相信。
//
//  ⚠ 分桶索引的 `role` 取值存在**文档与调用点不一致**：
//    data/IMatchStatsStore.h 的注释写"相机角色（25 / 50 / 100 mm）"，
//    而唯一调用点 MeasurementSelector.cpp 传的是
//    `static_cast<int>(c.camera)` —— 即 CameraRole 的**枚举序号** 0/1/2。
//    本实现两种编码都接受（0/1/2 与 25/50/100），因为按其中之一写的
//    调用方都不该拿到错桶。两者的映射是唯一的，不存在歧义；
//    但"接口注释与实际调用不符"本身就是缺陷，已登记待裁决。
//    越界索引返回先验（不抛异常），与接口契约一致。
//
//  ⚠ 读写时机（ENG-10 §4.4，冻结）：**任务期间只读，仅 SAVE/FAILED 写回**。
//    这不由本类强制（record() 只累加内存计数，save() 才落盘），
//    而由调用方遵守：采集过程中写回会让同一任务内后续帧的评分受本任务
//    早先帧影响，使"重试"不再是独立重复实验（SYS-08 §7.3 的 ALIGN 上限
//    就失去统计意义）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "data/IMatchStatsStore.h"
#include "data/MeasurementConfig.h"

namespace aircraft
{
namespace infrastructure
{

/// `runtime/match_stats.yaml` 的读写实现。
class FileMatchStatsStore : public data::IMatchStatsStore
{
public:
    /// 距离带 4 档、光照带 3 档、角色 3 个（ENG-10 §4.2 冻结的 36 桶）。
    static constexpr int kRoleCount     = 3;
    static constexpr int kDistanceBands = 4;
    static constexpr int kIllumBands    = 3;
    static constexpr int kBucketCount   = kRoleCount * kDistanceBands * kIllumBands;

    /// 单个统计桶：只存计数，成功率在读取时算（见文件头说明）。
    struct Bucket
    {
        uint64_t attempts  = 0;
        uint64_t successes = 0;
    };

    /// @param path 统计表文件路径（通常 "runtime/match_stats.yaml"）。
    ///        目录不存在时由 save() 递归创建。
    /// @param config 取 `matchStatsMinSamples`（N_min）与
    ///        `matchStatsColdStartPrior`（空表先验）。两个值都属
    ///        MeasurementConfig（ENG-10 §4.2/§4.3），本类不另设常量 ——
    ///        否则"收缩强度"会有两个来源。
    FileMatchStatsStore(std::string path, const data::MeasurementConfig& config);

    // ---- data::IMatchStatsStore ----
    double successRate(const std::string& targetModelId,
                       int role,
                       int distanceBand,
                       int illumBand) const override;
    void record(const std::string& targetModelId,
                int role,
                int distanceBand,
                int illumBand,
                bool success) override;
    bool save() override;
    bool load() override;

    // ---- 诊断（供日志与测试使用）----

    const std::string& path() const { return path_; }

    /// load() 的结论：true = 文件存在且解析成功；false = 文件不存在
    /// （冷启动，正常情况）或解析失败（此时 lastCorruptPath() 非空）。
    bool loadedFromFile() const;

    /// 已知机型数。
    std::size_t modelCount() const;

    /// 指定机型的非空桶数。
    std::size_t nonEmptyBucketCount(const std::string& targetModelId) const;

    /// 指定桶的观测次数（测试与日志用；不参与评分）。
    uint64_t attempts(const std::string& targetModelId,
                      int role,
                      int distanceBand,
                      int illumBand) const;

    /// 最近一次失败的文字说明（save/load）。成功时为空。
    std::string lastErrorText() const;

    /// 解析失败时被改名的现场文件路径（ENG-10 §4.6）；无则空。
    std::string lastCorruptPath() const;

    /// 内存表是否发生过变化（record 调用过且未 save）。测试用。
    bool dirty() const;

private:
    /// 桶下标。越界返回 -1（调用方据此走先验分支）。
    static int bucketIndex(int role, int distanceBand, int illumBand);

    /// 由外部编码解析出角色序号 0/1/2；无法识别返回 -1。
    static int roleIndexOf(int roleValue);

    /// 写 YAML 文本（供 save 的临时文件）。
    std::string serializeLocked() const;

    mutable std::mutex mutex_;
    std::string        path_;
    double             minSamples_    = 10.0;
    double             coldStartPrior_ = 0.5;

    /// model_id -> 36 个桶。
    std::map<std::string, std::vector<Bucket>> table_;
    bool        dirty_          = false;
    bool        loadedFromFile_ = false;
    std::string lastErrorText_;
    std::string lastCorruptPath_;
};

}  // namespace infrastructure
}  // namespace aircraft
