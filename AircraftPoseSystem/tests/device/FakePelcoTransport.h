#pragma once

// ============================================================================
//  tests/device/FakePelcoTransport.h
//
//  测试用的设备替身：实现 `IPelcoTransport`，在进程内模拟一台 Pelco-D 转台。
//  没有 socket、没有真实等待 —— 超时由"收件箱为空"立即表达，
//  因此所有会话/控制器测试都能在毫秒级跑完。
//
//  ⚠ 这不是产品代码；`send()` 记录出的每一帧正是"软件到底发了什么"的
//    唯一证据，测试用它断言只读边界与禁止指令。
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/PelcoDFrame.h"

namespace aircraft
{
namespace device
{
namespace test
{

class FakePelcoTransport : public IPelcoTransport
{
public:
    bool openOk = true;
    bool sendOk = true;
    std::string openError = "模拟打开失败";

    /// 收件箱为空时，`receive()` 先睡这么久再返回 Timeout。
    /// 用于验证 `state()` 在 worker 阻塞时仍然不阻塞。
    /// atomic：控制器测试会在 worker 运行时从主线程修改它。
    std::atomic<int> receiveBlockMs{0};

    std::vector<std::vector<std::uint8_t>> sent;
    std::deque<std::vector<std::uint8_t>> inbox;

    /// 每发出一帧调用一次，测试在这里生成设备回复。
    std::function<void(const std::vector<std::uint8_t>&)> onSend;

    int openCalls = 0;
    int closeCalls = 0;
    int sendCalls = 0;
    int receiveCalls = 0;

    bool open(const std::string& host, std::uint16_t port,
              std::string& error) override
    {
        (void)host;
        (void)port;
        ++openCalls;
        if (!openOk)
        {
            error = openError;
            return false;
        }
        opened_ = true;
        return true;
    }

    bool send(const std::uint8_t* data, std::size_t size,
              std::string& error) override
    {
        ++sendCalls;
        if (!opened_)
        {
            error = "socket 未打开";
            return false;
        }
        if (!sendOk)
        {
            error = "模拟发送失败";
            return false;
        }
        std::vector<std::uint8_t> bytes(data, data + size);
        sent.push_back(bytes);
        if (onSend)
        {
            onSend(bytes);
        }
        return true;
    }

    TransportResult receive(std::uint8_t* buffer, std::size_t capacity,
                            int timeoutMs) override
    {
        (void)timeoutMs;
        ++receiveCalls;

        if (!inbox.empty())
        {
            const auto datagram = inbox.front();
            inbox.pop_front();
            const std::size_t n = std::min(capacity, datagram.size());
            std::memcpy(buffer, datagram.data(), n);
            TransportResult result;
            result.status = TransportStatus::Data;
            result.size   = n;
            return result;
        }

        const int blockMs = receiveBlockMs.load();
        if (blockMs > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(blockMs));
        }

        TransportResult result;
        result.status = TransportStatus::Timeout;
        return result;
    }

    void close() override
    {
        ++closeCalls;
        opened_ = false;
    }

    bool opened() const { return opened_; }

    /// 已发帧中满足条件的数量（用于断言"没有发送控制指令"）。
    int countSent(
        const std::function<bool(const PelcoDFrame&)>& predicate) const
    {
        int count = 0;
        for (const auto& bytes : sent)
        {
            const auto frame = decode(bytes);
            if (predicate(frame))
            {
                ++count;
            }
        }
        return count;
    }

    static PelcoDFrame decode(const std::vector<std::uint8_t>& bytes)
    {
        PelcoDFrame frame;
        if (bytes.size() >= 7)
        {
            frame.address = bytes[1];
            frame.data1   = bytes[2];
            frame.data2   = bytes[3];
            frame.data3   = bytes[4];
            frame.data4   = bytes[5];
        }
        return frame;
    }

private:
    bool opened_ = false;
};

/// 模拟一台正常转台：查询有回复、定位回显、停止不回显。
struct FakePelcoDevice
{
    FakePelcoTransport* transport = nullptr;

    double azimuth   = 0.0;   // 软件角 deg
    double elevation = 0.0;   // 软件角 deg
    std::uint8_t workMode = 0x00;

    // atomic：控制器测试会在 worker 运行时从主线程置位，
    // 模拟"从某一刻起设备不再回复"。
    std::atomic<bool> dropHorizontal{false};
    std::atomic<bool> dropVertical{false};

    /// 0 = 不拒绝；1 = 只拒绝水平定位；2 = 只拒绝垂直定位；
    /// 3 = 两轴都拒绝（设备回 `EE EE EE EE`）。
    int rejectMoveAxis = 0;

    /// 前 N 个定位帧回复 `FF FF FF FF`（模拟一次可恢复的校验错误）。
    int positioningChecksumErrors = 0;

    /// true = 定位成功后立即把模拟角度设为目标（默认瞬动）。
    bool snapToTarget = true;

    int stopFrames = 0;

    void install()
    {
        transport->onSend = [this](const std::vector<std::uint8_t>& bytes) {
            handle(bytes);
        };
    }

    static std::vector<std::uint8_t> angleReply(bool horizontal, double deg)
    {
        PelcoDFrame frame;
        frame.address = 0x01;
        frame.data1   = 0x00;
        frame.data2   = horizontal ? 0x59 : 0x5B;
        const std::uint16_t raw =
            encodeAngle100(deg, /*signedAxis=*/!horizontal);
        frame.data3 = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        frame.data4 = static_cast<std::uint8_t>(raw & 0xFF);
        const auto encoded = encodePelcoD(frame);
        return std::vector<std::uint8_t>(encoded.begin(), encoded.end());
    }

    static std::vector<std::uint8_t> workModeReply(std::uint8_t mode)
    {
        PelcoDFrame frame;
        frame.address = 0x01;
        frame.data1   = 0xE0;
        frame.data2   = mode;
        const auto encoded = encodePelcoD(frame);
        return std::vector<std::uint8_t>(encoded.begin(), encoded.end());
    }

    static std::vector<std::uint8_t> marker(std::uint8_t value)
    {
        return std::vector<std::uint8_t>(4, value);
    }

private:
    void handle(const std::vector<std::uint8_t>& bytes)
    {
        if (bytes.size() != 7)
        {
            return;
        }
        const PelcoDFrame frame = FakePelcoTransport::decode(bytes);

        // 标准指令码在 Data2、Data1=0x00
        if (frame.data1 == 0x00 && frame.data2 == 0x51)
        {
            if (!dropHorizontal)
            {
                transport->inbox.push_back(angleReply(true, azimuth));
            }
            return;
        }
        if (frame.data1 == 0x00 && frame.data2 == 0x53)
        {
            if (!dropVertical)
            {
                transport->inbox.push_back(angleReply(false, elevation));
            }
            return;
        }
        if (frame.data1 == 0x00 && frame.data2 == 0x00 &&
            frame.data3 == 0x00 && frame.data4 == 0x00)
        {
            ++stopFrames;
            return;  // 停止帧不回显；会话本来就不同步等它
        }

        // 自定义指令码在 Data1
        if (frame.data1 == 0xE0)
        {
            transport->inbox.push_back(workModeReply(workMode));
            return;
        }

        // 定位：表 3.5（Data1=0、Data2=0x4B/0x4D）或表 3.65（Data1=0x4B/0x4D）
        bool horizontal = false;
        bool positioning = false;
        if (frame.data1 == 0x00 &&
            (frame.data2 == 0x4B || frame.data2 == 0x4D))
        {
            horizontal  = frame.data2 == 0x4B;
            positioning = true;
        }
        else if ((frame.data1 == 0x4B || frame.data1 == 0x4D) &&
                 frame.data2 != 0x00)
        {
            horizontal  = frame.data1 == 0x4B;
            positioning = true;
        }

        if (!positioning)
        {
            return;
        }

        if (rejectMoveAxis == 3 ||
            (rejectMoveAxis == 1 && horizontal) ||
            (rejectMoveAxis == 2 && !horizontal))
        {
            transport->inbox.push_back(marker(0xEE));
            return;
        }
        if (positioningChecksumErrors > 0)
        {
            --positioningChecksumErrors;
            transport->inbox.push_back(marker(0xFF));
            return;
        }

        // 回显：会话要求收到与发出帧逐字节一致的 7 字节帧。
        transport->inbox.push_back(bytes);

        if (snapToTarget)
        {
            const std::uint16_t raw =
                static_cast<std::uint16_t>((frame.data3 << 8) | frame.data4);
            if (horizontal)
            {
                double software = 0.0;
                if (deviceAzimuthToSoftware(raw, software))
                {
                    azimuth = software;
                }
            }
            else
            {
                elevation = deviceElevationToSoftware(raw);
            }
        }
    }
};

}  // namespace test
}  // namespace device
}  // namespace aircraft
