// ============================================================================
//  tests/device/PelcoDTransportTest.cpp
//
//  依据：SYS-10 V1.1 §6.3 / §6.5、C-22
//
//  覆盖：本机 UDP 往返、超时、close 幂等与重开、非法地址/端口拒绝、
//        未打开即收发、多帧报文完整交付。
//
//  ⚠ 只用 127.0.0.1 的临时端口，不碰实机、不发往 192.168.*。
// ============================================================================

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/UdpPelcoTransport.h"

using aircraft::device::IPelcoTransport;
using aircraft::device::TransportStatus;

namespace
{

class LocalUdpServer
{
public:
    ~LocalUdpServer()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    bool start()
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
        {
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = 0;  // 临时端口
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            return false;
        }

        socklen_t len = sizeof(addr);
        if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) < 0)
        {
            return false;
        }
        port_ = ntohs(addr.sin_port);

        timeval timeout{};
        timeout.tv_sec  = 1;
        timeout.tv_usec = 0;
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        return true;
    }

    std::uint16_t port() const { return port_; }

    /// 等待客户端报文；返回长度，失败返回 -1。
    ssize_t receive(std::uint8_t* buffer, std::size_t capacity)
    {
        socklen_t len = sizeof(client_);
        const ssize_t n =
            ::recvfrom(fd_, buffer, capacity, 0,
                       reinterpret_cast<sockaddr*>(&client_), &len);
        if (n >= 0)
        {
            haveClient_ = true;
        }
        return n;
    }

    bool reply(const std::uint8_t* data, std::size_t size)
    {
        if (!haveClient_)
        {
            return false;
        }
        const ssize_t n =
            ::sendto(fd_, data, size, 0,
                     reinterpret_cast<const sockaddr*>(&client_), sizeof(client_));
        return n == static_cast<ssize_t>(size);
    }

private:
    int fd_ = -1;
    std::uint16_t port_ = 0;
    sockaddr_in client_{};
    bool haveClient_ = false;
};

}  // namespace

TEST(PelcoDTransport, RoundTripAndMultiFrameDatagram)
{
    LocalUdpServer server;
    ASSERT_TRUE(server.start());

    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;
    ASSERT_TRUE(transport->open("127.0.0.1", server.port(), error)) << error;

    const std::uint8_t request[7] = {0xFF, 0x01, 0x00, 0x51, 0x00, 0x00, 0x52};
    ASSERT_TRUE(transport->send(request, sizeof(request), error)) << error;

    std::uint8_t serverBuffer[64] = {};
    const ssize_t received = server.receive(serverBuffer, sizeof(serverBuffer));
    ASSERT_EQ(received, static_cast<ssize_t>(7));
    EXPECT_EQ(std::memcmp(serverBuffer, request, 7), 0);

    // 两个 7 字节帧在**一个** UDP 报文里；接收方必须一次拿到 14 字节，
    // 由编解码器负责拆分（传输层不假设"一报文一帧"）。
    const std::uint8_t reply[14] = {0xFF, 0x01, 0x00, 0x59, 0x01, 0xD8, 0x33,
                                    0xFF, 0x01, 0x00, 0x5B, 0xFA, 0x55, 0xAB};
    ASSERT_TRUE(server.reply(reply, sizeof(reply)));

    std::uint8_t clientBuffer[64] = {};
    const auto result = transport->receive(clientBuffer, sizeof(clientBuffer), 200);
    ASSERT_EQ(result.status, TransportStatus::Data);
    EXPECT_EQ(result.size, static_cast<std::size_t>(14));
    EXPECT_EQ(std::memcmp(clientBuffer, reply, 14), 0);
}

TEST(PelcoDTransport, ReceiveTimesOut)
{
    LocalUdpServer server;
    ASSERT_TRUE(server.start());

    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;
    ASSERT_TRUE(transport->open("127.0.0.1", server.port(), error)) << error;

    std::uint8_t buffer[64] = {};
    const auto begin = std::chrono::steady_clock::now();
    const auto result = transport->receive(buffer, sizeof(buffer), 50);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - begin);

    EXPECT_EQ(result.status, TransportStatus::Timeout);
    EXPECT_EQ(result.size, static_cast<std::size_t>(0));
    // 允许调度抖动，但必须真的等过（不是立即返回），也不能远超超时值。
    EXPECT_GE(elapsed.count(), 40);
    EXPECT_LT(elapsed.count(), 500);
}

TEST(PelcoDTransport, CloseIsIdempotentAndReopenWorks)
{
    LocalUdpServer server;
    ASSERT_TRUE(server.start());

    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;
    ASSERT_TRUE(transport->open("127.0.0.1", server.port(), error)) << error;

    transport->close();
    transport->close();  // 幂等

    ASSERT_TRUE(transport->open("127.0.0.1", server.port(), error)) << error;

    const std::uint8_t request[7] = {0xFF, 0x01, 0x00, 0x53, 0x00, 0x00, 0x54};
    ASSERT_TRUE(transport->send(request, sizeof(request), error)) << error;

    std::uint8_t serverBuffer[64] = {};
    EXPECT_EQ(server.receive(serverBuffer, sizeof(serverBuffer)),
              static_cast<ssize_t>(7));
}

TEST(PelcoDTransport, RejectsInvalidHost)
{
    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;

    EXPECT_FALSE(transport->open("", 6666, error));
    EXPECT_FALSE(error.empty());

    EXPECT_FALSE(transport->open("not-an-ip", 6666, error));
    EXPECT_FALSE(error.empty());

    // 组播 / 广播 / 任意地址都不是单播转台目标
    EXPECT_FALSE(transport->open("0.0.0.0", 6666, error));
    EXPECT_FALSE(transport->open("255.255.255.255", 6666, error));
    EXPECT_FALSE(transport->open("224.0.0.1", 6666, error));
}

TEST(PelcoDTransport, RejectsPortZero)
{
    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;
    EXPECT_FALSE(transport->open("127.0.0.1", 0, error));
    EXPECT_FALSE(error.empty());
}

TEST(PelcoDTransport, SendAndReceiveRequireOpenSocket)
{
    auto transport = aircraft::device::makeUdpPelcoTransport();
    std::string error;
    const std::uint8_t frame[7] = {0xFF, 0x01, 0x00, 0x51, 0x00, 0x00, 0x52};

    EXPECT_FALSE(transport->send(frame, sizeof(frame), error));
    EXPECT_FALSE(error.empty());

    std::uint8_t buffer[16] = {};
    const auto result = transport->receive(buffer, sizeof(buffer), 10);
    EXPECT_EQ(result.status, TransportStatus::Error);
    EXPECT_FALSE(result.error.empty());
}
