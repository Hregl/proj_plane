#pragma once

// ============================================================================
//  src/data/TurntableConfig.h
//
//  依据：ENG-09 §4.1、§6.3（类型定义冻结）、§2.2（角度单位）
//        裁决 C-20、C-04
//
//  ⚠ 裁决 C-20（本文件最重要的一条）：
//  原 TurntableConfig 中的 `maxIterations`（对准最大迭代次数）**已删除**。
//
//  理由：它与 SYS-08 §7.3 的 ALIGN 最大尝试次数是**同一语义**，
//  保留两个名字意味着两处数值可能不一致（SYS-10 §14 与 ENG-09 各定义一份），
//  而一旦不一致，"对准到底能试几次"就没有唯一答案 —— 实际生效的取决于
//  哪个模块先读到哪个字段，且不会有任何报错。
//
//  冻结结论：唯一数据源是 **MeasurementConfig::maxAlignAttempts**（默认 8）。
//  AlignmentController 通过 RetryManager 查询剩余次数，
//  **不得自持计数器**（SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 1）。
//  本结构体中因此没有任何 attempts/iterations 类字段 —— 这不是遗漏。
//
//  ⚠ 角度单位一律 **deg**（ENG-09 §2.2），不是弧度。
//  在这里混淆的代价：若把 deg 当 rad 传入转台 SDK，
//  指令会被解读为 0~360 度范围内的极小角度，转台几乎不动，
//  而软件层面看不出任何异常（不会报错，只是对准永远不收敛，
//  最终耗尽 8 次 ALIGN 重试后报 kErrAlignRetryExhausted 2003 ——
//  故障现象指向算法，根因却在单位）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <string>

namespace aircraft
{
namespace data
{

/// 转台配置（ENG-09 §6.3）。
struct TurntableConfig
{
    /// 通信协议："sdk" | "pelco_d_udp" | "rs485" | "network"。
    /// * "sdk"          ⇒ VirtualTurntable（默认；虚拟件）
    /// * "pelco_d_udp"  ⇒ PelcoDTurntableController（真实 Pelco-D/UDP 后端）
    /// * "rs485"/"network" ⇒ 已登记但**本批未实现**，装配时显式失败，
    ///                      **不得**静默回落虚拟后端（C-22）
    ///
    /// 旧取值 "pekod" 已删除；ConfigManager 遇到时必须报迁移错误
    /// （"已移除，请改为 pelco_d_udp"），不得当作 "sdk" 或默认值。
    ///
    /// 冻结为 std::string 而非枚举：协议集合随工程实施阶段扩展
    /// （SYS-10 的现场协议由供货方确定），字符串可在不改冻结表的前提下
    /// 增加取值，代价是拼写错误要到运行时才暴露 ——
    /// 因此 ConfigManager 加载时必须做取值白名单校验（ENG-10 §5.3：
    /// 取值超范围时启动失败，而非取默认值）。
    std::string protocol;

    /// 方位角下限，单位 **deg**。
    double azimuthMin = 0.0;    // deg

    /// 方位角上限，单位 **deg**。
    double azimuthMax = 0.0;    // deg

    /// 俯仰角下限，单位 **deg**。
    double elevationMin = 0.0;  // deg

    /// 俯仰角上限，单位 **deg**。
    double elevationMax = 0.0;  // deg

    /// 粗调速度，单位 **deg/s**。用于大角度快速指向（SYS-10 §6）。
    double coarseSpeed = 0.0;   // deg/s

    /// 精调速度，单位 **deg/s**。用于对准末段的小步修正。
    /// 精调速度偏高会导致越过目标后反复来回（极限环），
    /// 表现为对准永不进入 stable，最终耗尽 ALIGN 重试。
    double fineSpeed = 0.0;     // deg/s

    /// 中心判据阈值，单位 **pixel**，默认 50（ENG-09 §6.3 冻结）。
    /// 判据形式：|TargetOffset::pixelX| <= centerThreshold
    ///       且 |TargetOffset::pixelY| <= centerThreshold。
    ///
    /// ⚠ 该判据的成立**完全依赖 TargetOffset 以图像中心为原点**
    /// （裁决 C-11）。若原点被误改为检测框中心，则 pixelX/pixelY
    /// 恒为 0，判据恒真，转台永不移动，而对准会被报告为成功 ——
    /// 见 TargetOffset.h 的详细说明。
    double centerThreshold = 50.0;  // pixel

    // ---- V2.7 新增（裁决 C-22）：Pelco-D/UDP 通信端点与到位判定 ----
    //
    // ⚠ 这些字段**只对 protocol == "pelco_d_udp" 生效**。其他协议下
    //   ConfigManager 不读取它们（避免既有 sdk 配置每次启动多出
    //   10 条"取默认值"记录）；若显式出现则给"仅 pelco_d_udp 使用"告警。
    //
    // ⚠ host 默认空是刻意的：出厂内网 IP 不得进入版本库。
    //   实机配置放在仓库外的本地目录，通过 argv[1] 指定配置目录启动。

    /// 转台 IPv4 点分十进制地址。空表示未配置。
    std::string host;

    /// UDP 端口（默认 6666）。
    int port = 6666;

    /// Pelco-D 设备地址，1..255。**不是 IP 地址** ——
    /// 这是 7 字节帧里的"地址"字节，同一条 UDP 链路上可挂多台设备。
    int pelcoAddress = 1;

    /// 单次查询/回显等待时间，ms。
    int replyTimeoutMs = 50;

    /// 后台轮询周期，ms。
    int pollIntervalMs = 50;

    /// 判 STABLE 所需的连续一致读数次数。
    int stableReadCount = 3;

    /// 相邻两次读数被视为"没变"的阈值，deg。
    double stableThresholdDeg = 0.02;

    /// 与目标角的到位容差，deg（与 SYS-10 §3.3 的 ±0.2° 对齐）。
    double arrivalToleranceDeg = 0.2;

    /// 连续多少个轮询周期无有效回复即置 ERROR。
    int commFailureThreshold = 5;

    /// false = 表 3.5（设备默认速度，本批默认）；
    /// true  = 表 3.65（指令携带转速，本批只做离线测试）。
    bool useSpeedControl = false;
};

}  // namespace data
}  // namespace aircraft
