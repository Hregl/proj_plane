#pragma once

// ============================================================================
//  src/device/turntable/PelcoDFrame.h
//
//  依据：SYS-10 V1.1 §6.3（帧格式）、§6.4（角度/速度映射）、§6.6（禁止指令）
//        ENG-09 V2.7 §4.3 / §6.3、裁决 C-22
//        厂商手册表 3.3 / 3.5 / 3.6 / 3.7 / 3.10 / 3.11 / 3.40 / 3.41 / 3.65
//        （短摘录见 AircraftPoseSystem/evidence/转台_PelcoD_v1/01_说明书_帧结构摘录.txt）
//
//  作用：Pelco-D 的纯编解码。**不做 I/O、不起线程、不持有 socket**，
//  因而可以在没有转台、没有网络的机器上逐字节测试。
//
//  ⚠ 本文件最容易写反的一条：**指令码的位置不是统一的**。
//    · 标准 Pelco-D 指令码放 **Data2**（Data1 = 0x00）：
//        0x51 / 0x53 查询水平/垂直角
//        0x4B / 0x4D 绝对定位（表 3.5）
//        0x00        停止
//    · 厂商自定义指令码放 **Data1**：
//        0xE0        工作模式查询/回复
//        0xC5        到位反馈（本批不使用）
//        0xE1        实时角度流（本批不开启）
//  把 0x51 写进 Data1、或把 0xE0 写进 Data2，帧的校验和都可能是对的，
//  设备只会静默不响应 —— 这是"命令发出去了但没反应"的典型来源。
//
//  ⚠ 校验码 = (地址 + Data1 + Data2 + Data3 + Data4) & 0xFF，
//  **不含起始位 0xFF**（表 3.3 下方原文与 §3 协议格式说明）。
//  含起始位的写法会让每一帧都差 0xFF，设备回 `FF FF FF FF`。
//
//  ⚠ 角度一律 ×100 取整、高字节在前；水平为无符号 0～360，
//  垂直为有符号 16 位。本批不做零位偏移，软件零点 = 设备零点。
// ============================================================================

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aircraft
{
namespace device
{

/// Pelco-D 的 7 字节帧（不含起始位与校验码，二者由编解码器负责）。
///
/// 保留 address 而不是在编码时另传：回显判据要求"收到的帧与发出的帧
/// 逐字节一致"，若结构里不存地址，构造测试帧时就得再拼一次地址，
/// 两份来源迟早会不一致。
struct PelcoDFrame
{
    std::uint8_t address = 0;
    std::uint8_t data1   = 0;
    std::uint8_t data2   = 0;
    std::uint8_t data3   = 0;
    std::uint8_t data4   = 0;
};

/// 根据帧内容计算校验码（不含起始位）。
std::uint8_t pelcoDChecksum(const PelcoDFrame& frame);

/// 编码为 7 字节：`FF | address | data1 | data2 | data3 | data4 | checksum`。
std::array<std::uint8_t, 7> encodePelcoD(const PelcoDFrame& frame);

/// 收到的报文解析结果类别。
///
/// ⚠ `Echo` 的语义是"一个校验正确的 7 字节帧，但不匹配任何已知回传格式"。
/// 它**不等于**"设备确认了我们的指令" —— 是否回显必须由会话层把
/// `frame` 与**刚发出的那一帧**逐字节比较，codec 不知道发过什么。
enum class PelcoDReplyKind
{
    Echo,             ///< 校验正确的普通帧；由调用方与已发指令比对
    AngleHorizontal,  ///< 0x00 / 0x59，水平角回传
    AngleVertical,    ///< 0x00 / 0x5B，垂直角回传
    WorkMode,         ///< 0xE0，工作模式回传
    InvalidAngle,     ///< 角度帧合法但数值越界（如水平 raw > 35999）
    ChecksumError,    ///< 7 字节帧的 data1..4 == FF FF FF FF，或裸 4 字节 FF FF FF FF
    Rejected,         ///< 7 字节帧的 data1..4 == EE EE EE EE，或裸 4 字节 EE EE EE EE
    Unknown           ///< 无法识别的垃圾字节
};

struct PelcoDReply
{
    PelcoDReplyKind kind = PelcoDReplyKind::Unknown;
    PelcoDFrame     frame{};
    double          angleDeg = 0.0;  ///< 仅 AngleHorizontal / AngleVertical 有效
    std::uint8_t    workMode = 0;    ///< 仅 WorkMode 有效
};

/// 解析一个 UDP 报文，可含多个 7 字节帧。
///
/// 规则（SYS-10 V1.1 §6.3）：
///  * 逐字节寻找 0xFF 起始；校验失败的候选帧**只前进 1 字节**，
///    避免把"垃圾前缀里的 0xFF"当成帧头后整段跳过真正的帧；
///  * 截断的尾帧丢弃，不跨报文重组（UDP 报文边界就是语义边界）；
///  * 仅当**整个报文恰好 4 字节**时，才把裸 `FF FF FF FF` / `EE EE EE EE`
///    当作错误标记。若不做这个长度限制，角度回传后的任意 4 个 0xFF
///    都可能被误判为校验错误。
std::vector<PelcoDReply> parsePelcoDDatagram(const std::uint8_t* data,
                                             std::size_t size);

// ---------------------------------------------------------------------------
//  角度与速度映射
// ---------------------------------------------------------------------------

/// 设备水平角（无符号 0～35999）→ 软件角（−180～+180）。
///
/// @return raw > 35999 时返回 false 且不写 out —— 非法样本必须能被
///         调用方看见，不得静默折回 0（折回会把"读数坏了"伪装成"在零点"）。
bool deviceAzimuthToSoftware(std::uint16_t raw, double& out);

/// 软件水平角 → 设备无符号角（0～35999）。
/// `180.00` 与 `−180.00` 是同一物理点，编码均为 18000；读回为 `+180`。
std::uint16_t softwareAzimuthToDevice(double deg);

/// 角度 ×100 编码。
/// @param signedAxis true = 垂直轴（有符号，直接 ×100）；false = 水平轴
///        （先规整到 [0, 360) 再 ×100）。
std::uint16_t encodeAngle100(double deg, bool signedAxis);

/// 由垂直角原始 16 位值还原软件角（有符号）。
double deviceElevationToSoftware(std::uint16_t raw);

/// 速度换算：`rpm10 = round(degPerSec * 10 / 6)`（1 r/min = 6 °/s）。
/// 负值按 0 处理；结果夹在 0～255（指令只有一个字节）。
std::uint8_t speedDegPerSecToRpm10(double degPerSec);

// ---------------------------------------------------------------------------
//  指令构造
//
//  只提供本批允许发送的指令。**故意不提供** 0xC9（自检）、0xDE（重启）、
//  0xBD（转圈/方向控制）以及扫描/预置位/限位逃逸指令的构造函数 ——
//  SYS-10 V1.1 §6.6 禁止发送，不给出"顺手就能调用"的入口。
// ---------------------------------------------------------------------------

PelcoDFrame makeQueryHorizontal(std::uint8_t address);
PelcoDFrame makeQueryVertical(std::uint8_t address);
PelcoDFrame makeQueryWorkMode(std::uint8_t address);
PelcoDFrame makeStop(std::uint8_t address);

/// 表 3.5：绝对定位，设备内部默认速度（use_speed_control=false）。
PelcoDFrame makeAbsolutePosition(std::uint8_t address, bool horizontal,
                                 double angleDeg);

/// 表 3.65：绝对定位并携带转速（use_speed_control=true）。
PelcoDFrame makeAbsolutePositionWithSpeed(std::uint8_t address,
                                          bool horizontal, double angleDeg,
                                          double speedDegPerSec);

}  // namespace device
}  // namespace aircraft
