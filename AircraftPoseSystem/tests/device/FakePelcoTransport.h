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

    /// true = 像真实 UDP socket 一样，按调用方给的 `timeoutMs` 等待
    /// （上限 500 ms），而不是立即返回 Timeout。
    /// 用于验证轮询期间的停止请求能在一个接收切片内被处理；
    /// 默认 false，保持既有用例的毫秒级运行。
    std::atomic<bool> honorReceiveTimeout{false};

    /// 抑制定位帧回显：0 = 正常；1 = 水平轴；2 = 垂直轴。
    /// 用于制造“第一轴已交付、第二轴仍在等回显”的部分发送窗口。
    std::atomic<int> dropMoveEchoAxis{0};

    /// 接下来 N 次**停止帧**发送失败（模拟停止未交付）。
    /// 按帧内容识别，不影响查询与定位帧。
    std::atomic<int> failStopSends{0};
    std::atomic<int> stopSendAttempts{0};
    /// socket 已关闭后仍尝试发送的**停止帧**次数。正常路径应为 0：
    /// 内部清理（worker 退出、析构）不得在 close() 之后重发停止。
    std::atomic<int> stopSendsAfterClose{0};

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
        const std::vector<std::uint8_t> bytes(data, data + size);
        const bool stopFrame = isStopFrame(bytes);
        if (!opened_)
        {
            if (stopFrame)
            {
                ++stopSendsAfterClose;
            }
            error = "socket 未打开";
            return false;
        }
        if (!sendOk)
        {
            error = "模拟发送失败";
            return false;
        }
        if (stopFrame)
        {
            ++stopSendAttempts;
            int remaining = failStopSends.load();
            while (remaining > 0 &&
                   !failStopSends.compare_exchange_weak(remaining,
                                                        remaining - 1))
            {
            }
            if (remaining > 0)
            {
                error = "模拟停止帧发送失败";
                return false;
            }
        }
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
        else if (honorReceiveTimeout.load() && timeoutMs > 0)
        {
            // 模拟真实 UDP：在调用方给的切片内等待。会话层的切片是
            // ≤10 ms，故这里的事件粒度就是“一个接收切片”。
            const int slice =
                std::min(timeoutMs, 500);
            std::this_thread::sleep_for(std::chrono::milliseconds(slice));
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

    /// 停止帧：标准指令 Data2=0x00，且 Data1/Data3/Data4 均为 0。
    static bool isStopFrame(const std::vector<std::uint8_t>& bytes)
    {
        if (bytes.size() < 7)
        {
            return false;
        }
        return bytes[2] == 0x00 && bytes[3] == 0x00 && bytes[4] == 0x00 &&
               bytes[5] == 0x00;
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

    /// 三类回复各自使用的地址字节。与配置不一致时，会话必须把该回复
    /// 当作“没有有效回复”（C3）。
    std::uint8_t horizontalReplyAddress = 0x01;
    std::uint8_t verticalReplyAddress   = 0x01;
    std::uint8_t workModeReplyAddress   = 0x01;
    std::uint8_t replyAddress           = 0x01;  ///< 兼容：同时改三类

    /// true = 工作模式查询完全不回包（用于 NoReply 可信度）。
    bool dropWorkMode = false;

    /// 工作模式回复的 Data3。非 0 时回复与查询**字节不同**（Data2 仍可为 0），
    /// 用于构造“独立回复但无法证明正常”的 Unknown 状态。
    std::uint8_t workModeReplyData3 = 0x00;

    /// 每次角度查询前两轴同时漂移的角度（deg）。用于模拟“停止没有
    /// 效果”：停止帧已交付，但观测窗口内角度持续变化 ⇒ 2006。
    double driftPerQueryDeg = 0.0;

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

    std::vector<std::uint8_t> angleReply(bool horizontal, double deg) const
    {
        PelcoDFrame frame;
        frame.address = horizontal ? horizontalReplyAddress
                                   : verticalReplyAddress;
        frame.data1   = 0x00;
        frame.data2   = horizontal ? 0x59 : 0x5B;
        const std::uint16_t raw =
            encodeAngle100(deg, /*signedAxis=*/!horizontal);
        frame.data3 = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        frame.data4 = static_cast<std::uint8_t>(raw & 0xFF);
        const auto encoded = encodePelcoD(frame);
        return std::vector<std::uint8_t>(encoded.begin(), encoded.end());
    }

    std::vector<std::uint8_t> workModeReply(std::uint8_t mode) const
    {
        PelcoDFrame frame;
        frame.address = workModeReplyAddress;
        frame.data1   = 0xE0;
        frame.data2   = mode;
        frame.data3   = workModeReplyData3;
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
            // 每轮只在水平查询到达时推进一次，两轴同步漂移：
            // 模拟“停止帧没有效果”时，两轴都在变化。
            if (driftPerQueryDeg != 0.0)
            {
                azimuth += driftPerQueryDeg;
                elevation += driftPerQueryDeg;
            }
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
            if (!dropWorkMode)
            {
                transport->inbox.push_back(workModeReply(workMode));
            }
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

        // 可抑制某一轴的回显：制造“部分发送后取消”的窗口。
        // 此时设备不改变角度（等同于未受理/未回复）。
        const int dropAxis = transport->dropMoveEchoAxis.load();
        if ((dropAxis == 1 && horizontal) || (dropAxis == 2 && !horizontal))
        {
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
