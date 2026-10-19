#pragma once

// ============================================================================
//  src/device/turntable/UdpPelcoTransport.h
//
//  依据：SYS-10 V1.1 §6.3、ENG-03（CMake 规范）、裁决 C-22
//
//  生产实现：POSIX IPv4 UDP。**只支持点分十进制 IPv4 字面量**：
//  不做 DNS 解析 —— 现场设备地址是静态配置的，引入 DNS 只会增加
//  启动期不可控的阻塞与失败模式。
//
//  ⚠ 使用 connected UDP（connect() 到目标）：内核只把目标地址的报文
//  交给本 socket，避免同机其它进程/端口误发的报文被当成转台回复。
//  代价是收不到其它来源的报文 —— 本场景不需要。
// ============================================================================

#include "device/turntable/IPelcoTransport.h"

namespace aircraft
{
namespace device
{

class UdpPelcoTransport final : public IPelcoTransport
{
public:
    UdpPelcoTransport() = default;
    ~UdpPelcoTransport() override;

    UdpPelcoTransport(const UdpPelcoTransport&) = delete;
    UdpPelcoTransport& operator=(const UdpPelcoTransport&) = delete;

    bool open(const std::string& host, std::uint16_t port,
              std::string& error) override;
    bool send(const std::uint8_t* data, std::size_t size,
              std::string& error) override;
    TransportResult receive(std::uint8_t* buffer, std::size_t capacity,
                            int timeoutMs) override;
    void close() override;

private:
    int fd_ = -1;
};

}  // namespace device
}  // namespace aircraft
