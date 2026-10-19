#pragma once

// ============================================================================
//  src/device/turntable/IPelcoTransport.h
//
//  依据：SYS-10 V1.1 §6.3 / §6.5、ENG-02 V2.2 §8.2、裁决 C-22
//
//  作用：Pelco-D 的**传输 seam**。会话层只依赖本接口，因而可以用一个
//  内存替身（或本地 UDP 服务端）测试超时、拒绝、丢包与多帧报文，
//  而不需要真实转台。
//
//  ⚠ 与 `IImvApi` 一样，本接口是**设备层内部 seam**，
//  不进 ENG-09 §4.3 的冻结类清单，也不跨层暴露。
//
//  ⚠ 本接口**不负责**帧的编解码：`send()` 收到的是已经编码好的
//  7 字节（或其它）字节流，`receive()` 交付的是原始报文。
//  把解码塞进传输层会让"校验和错误"与"网络错误"混为一谈。
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace aircraft
{
namespace device
{

enum class TransportStatus
{
    Data,     ///< 收到一个完整报文
    Timeout,  ///< 在给定时间内没有报文（UDP 的正常情形，不是故障）
    Error     ///< socket 层错误（地址不可达、连接被拒等）
};

struct TransportResult
{
    TransportStatus status = TransportStatus::Timeout;
    std::size_t     size   = 0;
    std::string     error;
};

class IPelcoTransport
{
public:
    virtual ~IPelcoTransport() = default;

    /// 打开并连接目标（connected UDP：只接收目标地址的报文）。
    /// @param error 失败原因（可读文本）。
    virtual bool open(const std::string& host, std::uint16_t port,
                      std::string& error) = 0;

    /// 发送一个报文。成功返回 true。
    virtual bool send(const std::uint8_t* data, std::size_t size,
                      std::string& error) = 0;

    /// 等待并接收一个报文。
    /// @param timeoutMs 最多等待的毫秒数。
    /// @return status=Data 时 size 为报文长度；Timeout 时 size=0。
    virtual TransportResult receive(std::uint8_t* buffer, std::size_t capacity,
                                    int timeoutMs) = 0;

    /// 关闭。幂等；可再次 open()。
    virtual void close() = 0;
};

/// 生产实现：POSIX IPv4 UDP（不依赖 QtNetwork、不依赖厂商 SDK）。
std::shared_ptr<IPelcoTransport> makeUdpPelcoTransport();

}  // namespace device
}  // namespace aircraft
