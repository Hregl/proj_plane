// ============================================================================
//  tests/device/PelcoDCodecTest.cpp
//
//  依据：SYS-10 V1.1 §6.3～§6.6、ENG-09 V2.7 §6.3、裁决 C-22
//        evidence/转台_PelcoD_v1/01_说明书_帧结构摘录.txt（表 3.3/3.5/3.6/3.7/
//        3.10/3.11/3.40/3.41/3.65）
//
//  覆盖：逐字节帧构造、校验码不含起始位、角度映射与边界、
//        回复识别、垃圾前缀/截断/多帧、禁止指令不可构造。
//
//  ⚠ 本文件**不碰网络、不碰线程**：PelcoDFrame 是纯函数，
//  所有用例都可以在无转台的机器上重复运行。
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "device/turntable/PelcoDFrame.h"

using aircraft::device::PelcoDFrame;
using aircraft::device::PelcoDReply;
using aircraft::device::PelcoDReplyKind;

namespace
{

std::vector<std::uint8_t> bytes(std::initializer_list<int> values)
{
    std::vector<std::uint8_t> out;
    for (int v : values)
    {
        out.push_back(static_cast<std::uint8_t>(v));
    }
    return out;
}

void expectFrame(const std::array<std::uint8_t, 7>& actual,
                 std::initializer_list<int> expected)
{
    ASSERT_EQ(actual.size(), expected.size());
    std::size_t i = 0;
    for (int value : expected)
    {
        EXPECT_EQ(actual[i], static_cast<std::uint8_t>(value));
        ++i;
    }
}

}  // namespace

// ===========================================================================
//  1. 帧布局与校验码（表 3.3）
// ===========================================================================

TEST(PelcoDCodec, ChecksumExcludesStartByte)
{
    PelcoDFrame frame;
    frame.address = 0x12;
    frame.data1   = 0x34;
    frame.data2   = 0x56;
    frame.data3   = 0x78;
    frame.data4   = 0x9A;

    // 0x12 + 0x34 + 0x56 + 0x78 + 0x9A = 0x1AE → 0xAE
    EXPECT_EQ(aircraft::device::pelcoDChecksum(frame),
              static_cast<std::uint8_t>(0xAE));

    const auto encoded = aircraft::device::encodePelcoD(frame);
    EXPECT_EQ(encoded[0], static_cast<std::uint8_t>(0xFF));
    EXPECT_EQ(encoded[1], static_cast<std::uint8_t>(0x12));
    EXPECT_EQ(encoded[2], static_cast<std::uint8_t>(0x34));
    EXPECT_EQ(encoded[3], static_cast<std::uint8_t>(0x56));
    EXPECT_EQ(encoded[4], static_cast<std::uint8_t>(0x78));
    EXPECT_EQ(encoded[5], static_cast<std::uint8_t>(0x9A));
    EXPECT_EQ(encoded[6], static_cast<std::uint8_t>(0xAE));

    // 若实现把起始位也算进校验和，结果会是 0xAD（0xAE + 0xFF = 0x1AD）。
    EXPECT_NE(encoded[6], static_cast<std::uint8_t>(0xAD));
}

TEST(PelcoDCodec, QueryFramesPlaceOpcodesInData2)
{
    // 表 3.6：FF 01 00 51 00 00 52 / FF 01 00 53 00 00 54
    const auto horizontal = aircraft::device::encodePelcoD(
        aircraft::device::makeQueryHorizontal(1));
    expectFrame(horizontal, {0xFF, 0x01, 0x00, 0x51, 0x00, 0x00, 0x52});

    const auto vertical = aircraft::device::encodePelcoD(
        aircraft::device::makeQueryVertical(1));
    expectFrame(vertical, {0xFF, 0x01, 0x00, 0x53, 0x00, 0x00, 0x54});
}

TEST(PelcoDCodec, WorkModeFramePlacesOpcodeInData1)
{
    // 表 3.40/3.41：自定义指令码在 Data1（这里最易与标准帧写反）
    const auto query = aircraft::device::encodePelcoD(
        aircraft::device::makeQueryWorkMode(1));
    expectFrame(query, {0xFF, 0x01, 0xE0, 0x00, 0x00, 0x00, 0xE1});
}

TEST(PelcoDCodec, StopFrame)
{
    // 表 3.10：FF 01 00 00 00 00 01
    const auto stop =
        aircraft::device::encodePelcoD(aircraft::device::makeStop(1));
    expectFrame(stop, {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01});
}

TEST(PelcoDCodec, DefaultSpeedPositionUsesTable35)
{
    // 水平 4.72° → 472 = 0x01D8；校验 0x25
    const auto horizontal = aircraft::device::encodePelcoD(
        aircraft::device::makeAbsolutePosition(1, true, 4.72));
    expectFrame(horizontal, {0xFF, 0x01, 0x00, 0x4B, 0x01, 0xD8, 0x25});

    // 垂直 −14.51° → −1451 = 0xFA55；校验 0x9D
    const auto vertical = aircraft::device::encodePelcoD(
        aircraft::device::makeAbsolutePosition(1, false, -14.51));
    expectFrame(vertical, {0xFF, 0x01, 0x00, 0x4D, 0xFA, 0x55, 0x9D});
}

TEST(PelcoDCodec, SpeedControlPositionUsesTable365)
{
    // 表 3.65：Data1 = 0x4B/0x4D，Data2 = 转速（r/min ×10）
    // 5 °/s → round(50/6) = 8
    const auto horizontal = aircraft::device::encodePelcoD(
        aircraft::device::makeAbsolutePositionWithSpeed(1, true, 4.72, 5.0));
    expectFrame(horizontal, {0xFF, 0x01, 0x4B, 0x08, 0x01, 0xD8, 0x2D});

    const auto vertical = aircraft::device::encodePelcoD(
        aircraft::device::makeAbsolutePositionWithSpeed(1, false, -14.51, 5.0));
    expectFrame(vertical, {0xFF, 0x01, 0x4D, 0x08, 0xFA, 0x55, 0xA5});
}

TEST(PelcoDCodec, SpeedConversionUsesDivideBySix)
{
    EXPECT_EQ(aircraft::device::speedDegPerSecToRpm10(5.0),
              static_cast<std::uint8_t>(8));
    EXPECT_EQ(aircraft::device::speedDegPerSecToRpm10(12.0),
              static_cast<std::uint8_t>(20));
    EXPECT_EQ(aircraft::device::speedDegPerSecToRpm10(0.0),
              static_cast<std::uint8_t>(0));
    EXPECT_EQ(aircraft::device::speedDegPerSecToRpm10(-3.0),
              static_cast<std::uint8_t>(0));
    // 超过一个字节的转速必须夹住，而不是回绕成慢速
    EXPECT_EQ(aircraft::device::speedDegPerSecToRpm10(1000.0),
              static_cast<std::uint8_t>(255));
}

// ===========================================================================
//  2. 角度映射（SYS-10 V1.1 §6.4）
// ===========================================================================

TEST(PelcoDCodec, HorizontalRoundTrip)
{
    const std::uint16_t samples[] = {0, 100, 472, 18000, 18100, 27000, 35999};
    for (std::uint16_t raw : samples)
    {
        double software = 0.0;
        ASSERT_TRUE(aircraft::device::deviceAzimuthToSoftware(raw, software));
        EXPECT_EQ(aircraft::device::softwareAzimuthToDevice(software), raw);
    }
}

TEST(PelcoDCodec, HorizontalWrapsToSignedRange)
{
    double value = 0.0;
    ASSERT_TRUE(aircraft::device::deviceAzimuthToSoftware(18100, value));
    EXPECT_NEAR(value, -179.0, 1e-9);

    ASSERT_TRUE(aircraft::device::deviceAzimuthToSoftware(35999, value));
    EXPECT_NEAR(value, -0.01, 1e-9);

    ASSERT_TRUE(aircraft::device::deviceAzimuthToSoftware(18000, value));
    EXPECT_NEAR(value, 180.0, 1e-9);
}

TEST(PelcoDCodec, PlusMinus180AreSameDevicePoint)
{
    EXPECT_EQ(aircraft::device::softwareAzimuthToDevice(180.0),
              static_cast<std::uint16_t>(18000));
    EXPECT_EQ(aircraft::device::softwareAzimuthToDevice(-180.0),
              static_cast<std::uint16_t>(18000));
}

TEST(PelcoDCodec, InvalidHorizontalRawIsRejected)
{
    double value = 123.0;
    EXPECT_FALSE(aircraft::device::deviceAzimuthToSoftware(36000, value));
    // 失败时不得改写输出，否则调用方会把"没读到"当成"在 123°"
    EXPECT_NEAR(value, 123.0, 1e-9);
}

TEST(PelcoDCodec, VerticalIsSigned)
{
    EXPECT_NEAR(aircraft::device::deviceElevationToSoftware(0xFA55), -14.51,
                1e-9);
    EXPECT_NEAR(aircraft::device::deviceElevationToSoftware(1451), 14.51,
                1e-9);
    EXPECT_EQ(aircraft::device::encodeAngle100(-14.51, true),
              static_cast<std::uint16_t>(0xFA55));
}

// ===========================================================================
//  3. 回复解析（表 3.7 / 3.41）
// ===========================================================================

TEST(PelcoDCodec, ParsesAngleReplies)
{
    // 水平 4.72°：FF 01 00 59 01 D8 33
    // 垂直 −14.51°：FF 01 00 5B FA 55 AB
    const auto datagram =
        bytes({0xFF, 0x01, 0x00, 0x59, 0x01, 0xD8, 0x33, 0xFF, 0x01, 0x00, 0x5B,
               0xFA, 0x55, 0xAB});

    const auto replies =
        aircraft::device::parsePelcoDDatagram(datagram.data(), datagram.size());
    ASSERT_EQ(replies.size(), static_cast<std::size_t>(2));

    EXPECT_EQ(replies[0].kind, PelcoDReplyKind::AngleHorizontal);
    EXPECT_NEAR(replies[0].angleDeg, 4.72, 1e-9);

    EXPECT_EQ(replies[1].kind, PelcoDReplyKind::AngleVertical);
    EXPECT_NEAR(replies[1].angleDeg, -14.51, 1e-9);
}

TEST(PelcoDCodec, ParsesWorkModeReply)
{
    // 表 3.41：FF 01 E0 11 00 00 F2 → WorkMode=0x11（限位逃逸）
    const auto datagram = bytes({0xFF, 0x01, 0xE0, 0x11, 0x00, 0x00, 0xF2});
    const auto replies =
        aircraft::device::parsePelcoDDatagram(datagram.data(), datagram.size());
    ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(replies[0].kind, PelcoDReplyKind::WorkMode);
    EXPECT_EQ(replies[0].workMode, static_cast<std::uint8_t>(0x11));
}

TEST(PelcoDCodec, InvalidHorizontalReplyIsItsOwnKind)
{
    // 36000 = 0x8CA0（越界）；FF 01 00 59 8C A0 86
    const auto datagram = bytes({0xFF, 0x01, 0x00, 0x59, 0x8C, 0xA0, 0x86});
    const auto replies =
        aircraft::device::parsePelcoDDatagram(datagram.data(), datagram.size());
    ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(replies[0].kind, PelcoDReplyKind::InvalidAngle);
}

TEST(PelcoDCodec, ChecksumErrorMarkers)
{
    // 7 字节形式：FF 01 FF FF FF FF FD
    {
        const auto datagram = bytes({0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD});
        const auto replies = aircraft::device::parsePelcoDDatagram(
            datagram.data(), datagram.size());
        ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
        EXPECT_EQ(replies[0].kind, PelcoDReplyKind::ChecksumError);
    }

    // 裸 4 字节形式
    {
        const auto datagram = bytes({0xFF, 0xFF, 0xFF, 0xFF});
        const auto replies = aircraft::device::parsePelcoDDatagram(
            datagram.data(), datagram.size());
        ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
        EXPECT_EQ(replies[0].kind, PelcoDReplyKind::ChecksumError);
    }
}

TEST(PelcoDCodec, RejectedMarkers)
{
    // 7 字节形式：FF 01 EE EE EE EE B9
    {
        const auto datagram = bytes({0xFF, 0x01, 0xEE, 0xEE, 0xEE, 0xEE, 0xB9});
        const auto replies = aircraft::device::parsePelcoDDatagram(
            datagram.data(), datagram.size());
        ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
        EXPECT_EQ(replies[0].kind, PelcoDReplyKind::Rejected);
    }

    // 裸 4 字节形式
    {
        const auto datagram = bytes({0xEE, 0xEE, 0xEE, 0xEE});
        const auto replies = aircraft::device::parsePelcoDDatagram(
            datagram.data(), datagram.size());
        ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
        EXPECT_EQ(replies[0].kind, PelcoDReplyKind::Rejected);
    }
}

TEST(PelcoDCodec, GarbagePrefixAndTruncatedTailAreDropped)
{
    // 前缀 0x12 0x34，随后一个完整查询帧，再跟一个截断帧
    const auto datagram = bytes({0x12, 0x34, 0xFF, 0x01, 0x00, 0x51, 0x00, 0x00,
                                 0x52, 0xFF, 0x01, 0x00});
    const auto replies =
        aircraft::device::parsePelcoDDatagram(datagram.data(), datagram.size());
    ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(replies[0].kind, PelcoDReplyKind::Echo);
    EXPECT_EQ(replies[0].frame.data2, static_cast<std::uint8_t>(0x51));
}

TEST(PelcoDCodec, BadChecksumSkipsOnlyOneByte)
{
    // 垃圾前缀 0xFF 0x12 构成一个校验失败的候选；真帧从下标 2 开始。
    // 若失败后跳过 7 字节而不是 1 字节，真帧会被整段吞掉。
    const auto datagram = bytes({0xFF, 0x12, 0xFF, 0x01, 0x00, 0x51, 0x00, 0x00,
                                 0x52});
    const auto replies =
        aircraft::device::parsePelcoDDatagram(datagram.data(), datagram.size());
    ASSERT_EQ(replies.size(), static_cast<std::size_t>(1));
    EXPECT_EQ(replies[0].frame.data2, static_cast<std::uint8_t>(0x51));
}

TEST(PelcoDCodec, EmptyAndNullInput)
{
    EXPECT_TRUE(aircraft::device::parsePelcoDDatagram(nullptr, 0).empty());
    const std::uint8_t one = 0xFF;
    EXPECT_TRUE(aircraft::device::parsePelcoDDatagram(&one, 1).empty());
}

// ===========================================================================
//  4. 禁止指令（SYS-10 V1.1 §6.6）
// ===========================================================================

TEST(PelcoDCodec, ForbiddenOpcodesAreNeverGenerated)
{
    const PelcoDFrame frames[] = {
        aircraft::device::makeQueryHorizontal(1),
        aircraft::device::makeQueryVertical(1),
        aircraft::device::makeQueryWorkMode(1),
        aircraft::device::makeStop(1),
        aircraft::device::makeAbsolutePosition(1, true, 4.72),
        aircraft::device::makeAbsolutePosition(1, false, -14.51),
        aircraft::device::makeAbsolutePositionWithSpeed(1, true, 4.72, 5.0),
        aircraft::device::makeAbsolutePositionWithSpeed(1, false, -14.51, 5.0),
    };

    for (const auto& frame : frames)
    {
        // 标准帧看 Data2，自定义帧看 Data1。绝不能出现 0xC9/0xDE/0xBD。
        EXPECT_NE(frame.data1, static_cast<std::uint8_t>(0xC9));
        EXPECT_NE(frame.data1, static_cast<std::uint8_t>(0xDE));
        EXPECT_NE(frame.data1, static_cast<std::uint8_t>(0xBD));
        EXPECT_NE(frame.data2, static_cast<std::uint8_t>(0xC9));
        EXPECT_NE(frame.data2, static_cast<std::uint8_t>(0xDE));
        EXPECT_NE(frame.data2, static_cast<std::uint8_t>(0xBD));
    }
}
