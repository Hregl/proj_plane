// ============================================================================
//  src/device/turntable/PelcoDFrame.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、ENG-09 V2.7 §6.3、裁决 C-22
// ============================================================================

#include "device/turntable/PelcoDFrame.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aircraft
{
namespace device
{

namespace
{
constexpr std::uint8_t kStartByte = 0xFF;

constexpr std::uint8_t kOpQueryHorizontal = 0x51;  ///< 表 3.6，Data2
constexpr std::uint8_t kOpQueryVertical   = 0x53;  ///< 表 3.6，Data2
constexpr std::uint8_t kOpPositionH       = 0x4B;  ///< 表 3.5，Data2
constexpr std::uint8_t kOpPositionV       = 0x4D;  ///< 表 3.5，Data2
constexpr std::uint8_t kOpReplyH          = 0x59;  ///< 表 3.7，Data2
constexpr std::uint8_t kOpReplyV          = 0x5B;  ///< 表 3.7，Data2
constexpr std::uint8_t kOpStop            = 0x00;  ///< 表 3.10，Data2

constexpr std::uint8_t kCustomWorkMode = 0xE0;  ///< 表 3.40/3.41，Data1

/// 水平角合法设备值上限：359.99° × 100。
constexpr std::uint16_t kMaxHorizontalRaw = 35999;

void appendAngleReply(std::vector<PelcoDReply>& out, const PelcoDFrame& frame,
                      PelcoDReplyKind kind, double angleDeg)
{
    PelcoDReply reply;
    reply.kind     = kind;
    reply.frame    = frame;
    reply.angleDeg = angleDeg;
    out.push_back(reply);
}

}  // namespace

std::uint8_t pelcoDChecksum(const PelcoDFrame& frame)
{
    // 表 3.3 / §3 原文：校验码 =（地址 + Data1 + Data2 + Data3 + Data4）& 0x00FF。
    // 起始位 0xFF **不在**求和内。写成含起始位的样子不会在编译期报错，
    // 只会在设备侧表现为"永远回 FF FF FF FF"。
    const int sum = static_cast<int>(frame.address) + static_cast<int>(frame.data1) +
                    static_cast<int>(frame.data2) + static_cast<int>(frame.data3) +
                    static_cast<int>(frame.data4);
    return static_cast<std::uint8_t>(sum & 0xFF);
}

std::array<std::uint8_t, 7> encodePelcoD(const PelcoDFrame& frame)
{
    return {kStartByte, frame.address, frame.data1, frame.data2, frame.data3,
            frame.data4, pelcoDChecksum(frame)};
}

std::vector<PelcoDReply> parsePelcoDDatagram(const std::uint8_t* data,
                                             std::size_t size)
{
    std::vector<PelcoDReply> out;
    if (data == nullptr || size == 0)
    {
        return out;
    }

    // ---- 裸 4 字节错误标记（表 3.5 设备回复约定）----
    //
    // 只在**整个报文恰好 4 字节**时判定。若不做长度限制，角度回传中
    // 连续四个 0xFF 的片段会被当成校验错误，把一个正常的 7 字节帧
    // 拆成"错误 + 垃圾"。
    if (size == 4)
    {
        const bool allFF = data[0] == 0xFF && data[1] == 0xFF &&
                           data[2] == 0xFF && data[3] == 0xFF;
        const bool allEE = data[0] == 0xEE && data[1] == 0xEE &&
                           data[2] == 0xEE && data[3] == 0xEE;
        if (allFF || allEE)
        {
            PelcoDReply reply;
            reply.kind = allFF ? PelcoDReplyKind::ChecksumError
                               : PelcoDReplyKind::Rejected;
            out.push_back(reply);
            return out;
        }
    }

    std::size_t i = 0;
    while (i + 7 <= size)
    {
        if (data[i] != kStartByte)
        {
            ++i;
            continue;
        }

        PelcoDFrame frame;
        frame.address = data[i + 1];
        frame.data1   = data[i + 2];
        frame.data2   = data[i + 3];
        frame.data3   = data[i + 4];
        frame.data4   = data[i + 5];

        if (data[i + 6] != pelcoDChecksum(frame))
        {
            // 校验失败：**只前进 1 字节**。若直接跳过 7 字节，垃圾前缀里
            // 偶然出现的 0xFF 会吞掉紧随其后的真正帧。
            ++i;
            continue;
        }

        // ---- 错误标记（7 字节形式）----
        if (frame.data1 == 0xFF && frame.data2 == 0xFF && frame.data3 == 0xFF &&
            frame.data4 == 0xFF)
        {
            PelcoDReply reply;
            reply.kind  = PelcoDReplyKind::ChecksumError;
            reply.frame = frame;
            out.push_back(reply);
            i += 7;
            continue;
        }
        if (frame.data1 == 0xEE && frame.data2 == 0xEE && frame.data3 == 0xEE &&
            frame.data4 == 0xEE)
        {
            PelcoDReply reply;
            reply.kind  = PelcoDReplyKind::Rejected;
            reply.frame = frame;
            out.push_back(reply);
            i += 7;
            continue;
        }

        // ---- 角度回传（表 3.7：Data1=0x00，Data2=0x59/0x5B）----
        if (frame.data1 == 0x00 && frame.data2 == kOpReplyH)
        {
            const std::uint16_t raw =
                static_cast<std::uint16_t>((frame.data3 << 8) | frame.data4);
            double angle = 0.0;
            if (deviceAzimuthToSoftware(raw, angle))
            {
                appendAngleReply(out, frame, PelcoDReplyKind::AngleHorizontal,
                                 angle);
            }
            else
            {
                // 非法样本单独成类，由会话层决定是否记为通信失败。
                // 静默折回 0 会把"编码器/线路坏了"伪装成"转台在零点"。
                appendAngleReply(out, frame, PelcoDReplyKind::InvalidAngle, 0.0);
            }
            i += 7;
            continue;
        }

        if (frame.data1 == 0x00 && frame.data2 == kOpReplyV)
        {
            const std::uint16_t raw =
                static_cast<std::uint16_t>((frame.data3 << 8) | frame.data4);
            appendAngleReply(out, frame, PelcoDReplyKind::AngleVertical,
                             deviceElevationToSoftware(raw));
            i += 7;
            continue;
        }

        // ---- 工作模式回传（自定义指令，Data1=0xE0）----
        if (frame.data1 == kCustomWorkMode)
        {
            PelcoDReply reply;
            reply.kind     = PelcoDReplyKind::WorkMode;
            reply.frame    = frame;
            reply.workMode = frame.data2;
            out.push_back(reply);
            i += 7;
            continue;
        }

        // ---- 其余校验正确的帧：交给会话层与已发指令比对 ----
        PelcoDReply reply;
        reply.kind  = PelcoDReplyKind::Echo;
        reply.frame = frame;
        out.push_back(reply);
        i += 7;
    }

    return out;
}

bool deviceAzimuthToSoftware(std::uint16_t raw, double& out)
{
    if (raw > kMaxHorizontalRaw)
    {
        return false;
    }
    double az = static_cast<double>(raw) / 100.0;
    if (az > 180.0)
    {
        az -= 360.0;
    }
    out = az;
    return true;
}

bool softwareAzimuthToDeviceChecked(double deg, std::uint16_t& out)
{
    // ⚠ std::lround(NaN) 是**实现定义值**（通常为 LONG_MIN），再强转成
    // uint16_t 会静默变成一个“看起来正常”的角度。必须在取整之前拒绝。
    if (!std::isfinite(deg))
    {
        return false;
    }
    double d = std::fmod(deg, 360.0);
    if (d < 0.0)
    {
        d += 360.0;
    }
    long raw = std::lround(d * 100.0);
    // d < 360 时 raw 最大 36000（359.999… 四舍五入）。36000 与 0 是同一
    // 物理点；这里折回 0，避免把越界值发给设备。
    if (raw >= 36000)
    {
        raw -= 36000;
    }
    out = static_cast<std::uint16_t>(raw);
    return true;
}

std::uint16_t softwareAzimuthToDevice(double deg)
{
    std::uint16_t raw = 0;
    if (!softwareAzimuthToDeviceChecked(deg, raw))
    {
        return 0xFFFF;  // 越界哨兵：调用方应改用 Checked 版本
    }
    return raw;
}

bool encodeAngle100Checked(double deg, bool signedAxis, std::uint16_t& out)
{
    if (!std::isfinite(deg))
    {
        return false;
    }
    if (!signedAxis)
    {
        return softwareAzimuthToDeviceChecked(deg, out);
    }

    const long raw = std::lround(deg * 100.0);
    const long clamped =
        std::max<long>(std::numeric_limits<std::int16_t>::min(),
                       std::min<long>(std::numeric_limits<std::int16_t>::max(),
                                      raw));
    out = static_cast<std::uint16_t>(static_cast<std::int16_t>(clamped));
    return true;
}

std::uint16_t encodeAngle100(double deg, bool signedAxis)
{
    std::uint16_t raw = 0;
    if (!encodeAngle100Checked(deg, signedAxis, raw))
    {
        return 0xFFFF;
    }
    return raw;
}

double deviceElevationToSoftware(std::uint16_t raw)
{
    return static_cast<double>(static_cast<std::int16_t>(raw)) / 100.0;
}

bool speedDegPerSecToRpm10Checked(double degPerSec, std::uint8_t& out)
{
    if (!std::isfinite(degPerSec))
    {
        return false;
    }
    if (degPerSec <= 0.0)
    {
        out = 0;
        return true;
    }
    // 1 r/min = 6 °/s；指令值 = r/min × 10 = degPerSec × 10 / 6。
    // 写成 ×6 会把 5 °/s 发成 30 r/min（180 °/s），是 36 倍的超速。
    const long rpm10 = std::lround(degPerSec * 10.0 / 6.0);
    out = static_cast<std::uint8_t>(
        std::max<long>(0, std::min<long>(255, rpm10)));
    return true;
}

std::uint8_t speedDegPerSecToRpm10(double degPerSec)
{
    std::uint8_t rpm10 = 0;
    if (!speedDegPerSecToRpm10Checked(degPerSec, rpm10))
    {
        return 0;  // 调用方应改用 Checked 版本
    }
    return rpm10;
}

// ---------------------------------------------------------------------------
//  指令构造
// ---------------------------------------------------------------------------

PelcoDFrame makeQueryHorizontal(std::uint8_t address)
{
    PelcoDFrame frame;
    frame.address = address;
    frame.data2   = kOpQueryHorizontal;  // 标准指令在 Data2
    return frame;
}

PelcoDFrame makeQueryVertical(std::uint8_t address)
{
    PelcoDFrame frame;
    frame.address = address;
    frame.data2   = kOpQueryVertical;
    return frame;
}

PelcoDFrame makeQueryWorkMode(std::uint8_t address)
{
    PelcoDFrame frame;
    frame.address = address;
    frame.data1   = kCustomWorkMode;  // 自定义指令在 Data1
    return frame;
}

PelcoDFrame makeStop(std::uint8_t address)
{
    PelcoDFrame frame;
    frame.address = address;
    frame.data2   = kOpStop;
    return frame;
}

bool makeAbsolutePositionChecked(std::uint8_t address, bool horizontal,
                                 double angleDeg, PelcoDFrame& out)
{
    std::uint16_t raw = 0;
    if (!encodeAngle100Checked(angleDeg, !horizontal, raw))
    {
        return false;
    }
    PelcoDFrame frame;
    frame.address = address;
    frame.data2   = horizontal ? kOpPositionH : kOpPositionV;
    frame.data3 = static_cast<std::uint8_t>(raw >> 8);
    frame.data4 = static_cast<std::uint8_t>(raw & 0xFF);
    out = frame;
    return true;
}

PelcoDFrame makeAbsolutePosition(std::uint8_t address, bool horizontal,
                                 double angleDeg)
{
    PelcoDFrame frame;
    if (!makeAbsolutePositionChecked(address, horizontal, angleDeg, frame))
    {
        // 无效帧：水平角 0xFFFF 越界、垂直角同样是非法样本。
        // 调用方不得发送；生产路径使用 Checked 版本并走 2004。
        frame = PelcoDFrame{};
        frame.address = address;
        frame.data2   = horizontal ? kOpPositionH : kOpPositionV;
        frame.data3   = 0xFF;
        frame.data4   = 0xFF;
    }
    return frame;
}

bool makeAbsolutePositionWithSpeedChecked(std::uint8_t address,
                                          bool horizontal, double angleDeg,
                                          double speedDegPerSec,
                                          PelcoDFrame& out)
{
    std::uint8_t rpm10 = 0;
    if (!speedDegPerSecToRpm10Checked(speedDegPerSec, rpm10))
    {
        return false;
    }
    std::uint16_t raw = 0;
    if (!encodeAngle100Checked(angleDeg, !horizontal, raw))
    {
        return false;
    }
    PelcoDFrame frame;
    frame.address = address;
    frame.data1   = horizontal ? kOpPositionH : kOpPositionV;  // 表 3.65
    frame.data2   = rpm10;
    frame.data3 = static_cast<std::uint8_t>(raw >> 8);
    frame.data4 = static_cast<std::uint8_t>(raw & 0xFF);
    out = frame;
    return true;
}

PelcoDFrame makeAbsolutePositionWithSpeed(std::uint8_t address, bool horizontal,
                                          double angleDeg, double speedDegPerSec)
{
    PelcoDFrame frame;
    if (!makeAbsolutePositionWithSpeedChecked(address, horizontal, angleDeg,
                                              speedDegPerSec, frame))
    {
        frame = PelcoDFrame{};
        frame.address = address;
        frame.data1   = horizontal ? kOpPositionH : kOpPositionV;
        frame.data3   = 0xFF;
        frame.data4   = 0xFF;
    }
    return frame;
}

}  // namespace device
}  // namespace aircraft
