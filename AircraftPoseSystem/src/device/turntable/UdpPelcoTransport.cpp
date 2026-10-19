// ============================================================================
//  src/device/turntable/UdpPelcoTransport.cpp
//
//  依据：SYS-10 V1.1 §6.3 / §6.5、ENG-03、裁决 C-22
// ============================================================================

#include "device/turntable/UdpPelcoTransport.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace aircraft
{
namespace device
{

namespace
{
std::string errnoText(const char* what)
{
    return std::string(what) + "：" + std::strerror(errno);
}

/// 拒绝明显不是单播目标设备的地址。
/// ConfigManager 也会校验；这里再挡一次，避免测试替身或别的调用方
/// 绕过配置层直接把广播/组播地址交给 socket。
bool isUnicastIPv4(const in_addr& addr)
{
    const std::uint32_t hostOrder = ntohl(addr.s_addr);

    // 0.0.0.0（本机任意地址）与 255.255.255.255（受限广播）
    if (hostOrder == 0U || hostOrder == 0xFFFFFFFFU)
    {
        return false;
    }
    // 224.0.0.0/4 组播
    if ((hostOrder & 0xF0000000U) == 0xE0000000U)
    {
        return false;
    }
    return true;
}
}  // namespace

UdpPelcoTransport::~UdpPelcoTransport()
{
    close();
}

bool UdpPelcoTransport::open(const std::string& host, std::uint16_t port,
                             std::string& error)
{
    close();
    error.clear();

    if (host.empty())
    {
        error = "转台地址为空";
        return false;
    }
    if (port == 0)
    {
        error = "转台端口为 0";
        return false;
    }

    in_addr addr{};
    if (::inet_pton(AF_INET, host.c_str(), &addr) != 1)
    {
        error = "转台地址不是合法的 IPv4 点分十进制：" + host;
        return false;
    }
    if (!isUnicastIPv4(addr))
    {
        error = "转台地址不是单播地址：" + host;
        return false;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        error = errnoText("创建 UDP socket 失败");
        return false;
    }

    // SOCK_CLOEXEC 在严格 -std=c++17 下未必可见；用 fcntl 显式设置，
    // 避免子进程继承转台 socket。
    const int flags = ::fcntl(fd, F_GETFD, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0)
    {
        error = errnoText("设置 UDP socket FD_CLOEXEC 失败");
        ::close(fd);
        return false;
    }

    sockaddr_in target{};
    target.sin_family      = AF_INET;
    target.sin_port        = htons(port);
    target.sin_addr.s_addr = addr.s_addr;

    if (::connect(fd, reinterpret_cast<sockaddr*>(&target), sizeof(target)) < 0)
    {
        error = errnoText("连接转台 UDP 地址失败");
        ::close(fd);
        return false;
    }

    fd_ = fd;
    return true;
}

bool UdpPelcoTransport::send(const std::uint8_t* data, std::size_t size,
                             std::string& error)
{
    error.clear();
    if (fd_ < 0)
    {
        error = "UDP socket 未打开";
        return false;
    }
    if (data == nullptr || size == 0)
    {
        error = "发送缓冲为空";
        return false;
    }

    for (;;)
    {
        const ssize_t sent = ::send(fd_, data, size, 0);
        if (sent < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            error = errnoText("发送 UDP 报文失败");
            return false;
        }
        if (static_cast<std::size_t>(sent) != size)
        {
            // UDP 数据报要么整包发送，要么失败；短写意味着调用方
            // 传入了非法长度或内核状态异常，不能静默接受。
            error = "UDP 报文短写";
            return false;
        }
        return true;
    }
}

TransportResult UdpPelcoTransport::receive(std::uint8_t* buffer,
                                           std::size_t capacity,
                                           int timeoutMs)
{
    TransportResult result;
    if (fd_ < 0)
    {
        result.status = TransportStatus::Error;
        result.error  = "UDP socket 未打开";
        return result;
    }
    if (buffer == nullptr || capacity == 0)
    {
        result.status = TransportStatus::Error;
        result.error  = "接收缓冲为空";
        return result;
    }

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeoutMs > 0 ? timeoutMs : 0);

    for (;;)
    {
        int remainingMs = 0;
        if (timeoutMs > 0)
        {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            remainingMs = remaining.count() > 0 ? static_cast<int>(remaining.count()) : 0;
        }

        pollfd pfd{};
        pfd.fd     = fd_;
        pfd.events = POLLIN;

        const int ready = ::poll(&pfd, 1, remainingMs);
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                if (timeoutMs > 0 &&
                    std::chrono::steady_clock::now() >= deadline)
                {
                    result.status = TransportStatus::Timeout;
                    return result;
                }
                continue;
            }
            result.status = TransportStatus::Error;
            result.error  = errnoText("等待 UDP 报文失败");
            return result;
        }
        if (ready == 0)
        {
            result.status = TransportStatus::Timeout;
            return result;
        }

        // POLLERR 也要尝试 recv：connected UDP 的对端不可达时，
        // 错误通过 errno=ECONNREFUSED 在 recv 上报告。
        if ((pfd.revents & (POLLIN | POLLERR)) == 0)
        {
            result.status = TransportStatus::Error;
            result.error  = "UDP socket 出现非读事件";
            return result;
        }

        const ssize_t received = ::recv(fd_, buffer, capacity, 0);
        if (received < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                result.status = TransportStatus::Timeout;
                return result;
            }
            result.status = TransportStatus::Error;
            result.error  = errnoText("接收 UDP 报文失败");
            return result;
        }

        result.status = TransportStatus::Data;
        result.size   = static_cast<std::size_t>(received);
        return result;
    }
}

void UdpPelcoTransport::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

std::shared_ptr<IPelcoTransport> makeUdpPelcoTransport()
{
    return std::make_shared<UdpPelcoTransport>();
}

}  // namespace device
}  // namespace aircraft
