#pragma once

// ============================================================================
//  src/data/SystemConfig.h
//
//  依据：ENG-09 §4.1、§6.7（类型定义冻结）、裁决 C-16
//
//  裁决 C-16：`system.yaml` 在此之前**仅出现在 ENG-01 §3.2** 的配置文件
//  清单中，而 SYS-05 §13.1 的配置清单里没有它。本表裁决以 ENG-01 §3.2
//  为准（含 system.yaml），**共 7 个 yaml**。
//
//  因此本结构体是必需的 —— 若采纳 SYS-05 的清单，logDir / outputDir /
//  modelDir / logLevel 四个参数就没有任何配置文件承载，
//  只能硬编码在代码里，而"输出目录"这类参数在部署现场必然要改
//  （UOS 目标机的可写分区与开发机不同）。
//
//  ⚠ 四个目录必须**在启动时校验可写性**（属于 ENG-10 §5.3 的
//  "文件缺失 → 启动失败"一类）：
//  输出目录不可写时若继续运行，故障会推迟到 SAVE 状态才暴露 ——
//  那时已经完成了转台对准、采图、PnP 解算（数十秒），
//  而结果无处可写，整次测量白做。启动时一次 mkdir + 写测试即可避免。
// ============================================================================

#include <string>

namespace aircraft
{
namespace data
{

/// 系统级配置（ENG-09 §6.7）。
struct SystemConfig
{
    /// 日志目录。
    std::string logDir;

    /// 测量结果输出目录。measurement_xxx/ 建在此目录下。
    std::string outputDir;

    /// 目标模型与特征库目录（feature25/50/100.bin 所在处）。
    std::string modelDir;

    /// 日志级别。取值与 infrastructure/logger 的级别枚举一致，
    /// 加载时须做范围校验（ENG-10 §5.3：超范围启动失败）。
    int logLevel = 0;
};

}  // namespace data
}  // namespace aircraft
