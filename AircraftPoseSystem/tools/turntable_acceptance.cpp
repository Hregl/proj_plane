// ============================================================================
//  tools/turntable_acceptance.cpp
//
//  独立转台实机验收工具（不进入生产装配，不被 AircraftPoseSystem 链接）。
//
//  依据：SYS-10 §6.5（运动门禁）、SYS-04 §8/§21、ENG-09 §6.3、
//        裁决 C-23、2026-10-21 实机验收批次要求。
//
//  ⚠ 复用生产实现，不写第二套协议：
//    `PelcoDTurntableController` → `PelcoDSession` → `UdpPelcoTransport`
//    全部是生产代码。本文件只做三件事：
//      1. 读取真实配置与**本次验收许可**（显式文件，不是 config/ 键）；
//      2. 在传输 seam 外面包一层日志/计数（线上字节的唯一证据）；
//      3. 提供单步交互：move / stop / status / quit。
//
//  ⚠ 安全边界：
//    · 不给 `--permit` ⇒ 只读会话，初始化/轮询/失联/停止/退出全部
//      零控制帧（`sendPolicy` 保持 `ReadOnly`）；
//    · 给 `--permit` ⇒ 工具在**代码里**把 `sendPolicy` 设为
//      `MotionEnabled`，再安装许可。许可缺失/不符/越界/过期时安装失败，
//      工具在打开 socket 之前退出 —— 不会发出任何控制帧；
//    · 本工具**不调用** `setTestMotionGate()`；
//    · 不自动循环、不自动重试整段流程、不自动回零。每条 move 只下发一次；
//      协议层的单帧回显重试是生产会话既有行为，不是“重跑测试序列”；
//    · 许可只约束新的运动。停止/退出/故障收口不看许可期限。
//
//  ⚠ 速度：`use_speed_control=false`（默认）= 表 3.5 设备默认速度；
//    `use_speed_control=true` = 表 3.65，帧内携带转速。deg/s → r/min×10
//    的换算由生产 codec 完成，本工具不另写一套。启用 3.65 必须在本轮
//    书面授权中明确；配置仍须显式给出 coarse_speed/fine_speed
//    （位移 >1.0° 用粗调，否则用精调）。
//
//  ⚠ 证据：
//    工具输出**不含设备地址原文**（只打印端口/地址号）。完整原始记录
//    连同 `sha256sum` 结果保存在仓库根 `验收证据/`（gitignored）；
//    进库摘录须按约定脱敏并登记逐件 SHA-256。
// ============================================================================

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <unistd.h>

#include "data/TurntableAcceptancePermit.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableDiagnostics.h"
#include "data/TurntableMotionState.h"
#include "device/turntable/IPelcoTransport.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "infrastructure/config/ConfigManager.h"

namespace
{

using aircraft::data::TurntableAcceptancePermit;
using aircraft::data::TurntableCommand;
using aircraft::data::TurntableCommandOutcome;
using aircraft::data::TurntableConfig;
using aircraft::data::TurntableDiagnostics;
using aircraft::data::TurntableMotionState;
using aircraft::data::TurntableSendPolicy;
using aircraft::data::TurntableState;
using aircraft::device::IPelcoTransport;
using aircraft::device::makeUdpPelcoTransport;
using aircraft::device::PelcoDTurntableController;
using aircraft::device::TransportResult;
using aircraft::device::TransportStatus;
using aircraft::infrastructure::ConfigManager;

constexpr const char* kToolVersion = "1.0.4-C24";

using Clock = std::chrono::steady_clock;

Clock::time_point g_start;

// SIGINT/SIGTERM 只置标志；主循环看到后走与 `quit` 相同的
// “请求停止 → 观测停止 → 关闭”路径，避免 Ctrl-C 直接杀进程后
// 转台仍在执行上一条运动指令。
volatile std::sig_atomic_t g_interrupted = 0;

extern "C" void onTerminationSignal(int)
{
    g_interrupted = 1;
}

long long elapsedMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now() - g_start)
        .count();
}

const char* motionName(TurntableMotionState m)
{
    switch (m)
    {
    case TurntableMotionState::IDLE:   return "IDLE";
    case TurntableMotionState::MOVING: return "MOVING";
    case TurntableMotionState::STABLE: return "STABLE";
    case TurntableMotionState::ERROR:  return "ERROR";
    }
    return "?";
}

const char* outcomeName(TurntableCommandOutcome o)
{
    switch (o)
    {
    case TurntableCommandOutcome::None:       return "None";
    case TurntableCommandOutcome::Delivered:  return "Delivered";
    case TurntableCommandOutcome::Superseded: return "Superseded";
    case TurntableCommandOutcome::Cancelled:  return "Cancelled";
    case TurntableCommandOutcome::Failed:     return "Failed";
    }
    return "?";
}

const char* trustName(aircraft::data::WorkModeTrust t)
{
    using aircraft::data::WorkModeTrust;
    switch (t)
    {
    case WorkModeTrust::Unknown:      return "Unknown";
    case WorkModeTrust::EchoOnly:     return "EchoOnly";
    case WorkModeTrust::ReplyNonZero: return "ReplyNonZero";
    case WorkModeTrust::NoReply:      return "NoReply";
    }
    return "?";
}

/// 把 7 字节 Pelco-D 帧归类。只用于统计/证据，不参与协议判据。
enum class FrameKind { Query, Control, Unknown };

FrameKind classify(const std::uint8_t* data, std::size_t size)
{
    if (size < 7)
    {
        return FrameKind::Unknown;
    }
    const std::uint8_t d1 = data[2];
    const std::uint8_t d2 = data[3];
    if (d1 == 0x00 && d2 == 0x00)
    {
        return FrameKind::Control;  // stop
    }
    if (d1 == 0x00 && (d2 == 0x4B || d2 == 0x4D))
    {
        return FrameKind::Control;  // absolute positioning
    }
    if ((d1 == 0x4B || d1 == 0x4D) && d2 != 0x00)
    {
        return FrameKind::Control;  // speed-control positioning
    }
    if (d1 == 0x00 && (d2 == 0x51 || d2 == 0x53))
    {
        return FrameKind::Query;
    }
    if (d1 == 0xE0)
    {
        return FrameKind::Query;    // work mode
    }
    return FrameKind::Unknown;
}

/// 传输装饰器：打印线上字节并按“查询/控制”计数。
/// 它是 `IPelcoTransport` 的唯一包装，不改变协议，也不解释设备语义。
class LoggingCountingTransport final : public IPelcoTransport
{
public:
    explicit LoggingCountingTransport(std::shared_ptr<IPelcoTransport> inner)
        : inner_(std::move(inner))
    {
    }

    bool open(const std::string& host, std::uint16_t port,
              std::string& error) override
    {
        // 不打印 host：设备地址属于现场信息，只落本地原始证据。
        std::printf("NET_OPEN t=%lldms port=%u host=masked\n", elapsedMs(),
                    static_cast<unsigned>(port));
        std::fflush(stdout);
        return inner_->open(host, port, error);
    }

    bool send(const std::uint8_t* data, std::size_t size,
              std::string& error) override
    {
        const FrameKind kind = classify(data, size);
        if (kind == FrameKind::Control)
        {
            controlFrames.fetch_add(1);
        }
        else if (kind == FrameKind::Query)
        {
            queryFrames.fetch_add(1);
        }
        else
        {
            unknownFrames.fetch_add(1);
        }
        printHex("TX", data, size, kind);
        return inner_->send(data, size, error);
    }

    TransportResult receive(std::uint8_t* buffer, std::size_t capacity,
                            int timeoutMs) override
    {
        TransportResult r = inner_->receive(buffer, capacity, timeoutMs);
        if (r.status == TransportStatus::Data)
        {
            printHex("RX", buffer, r.size, FrameKind::Unknown);
        }
        else if (r.status == TransportStatus::Error)
        {
            std::printf("RX_ERROR t=%lldms msg=\"%s\"\n", elapsedMs(),
                        r.error.c_str());
            std::fflush(stdout);
        }
        return r;
    }

    void close() override
    {
        std::printf("NET_CLOSE t=%lldms\n", elapsedMs());
        std::fflush(stdout);
        inner_->close();
    }

    std::atomic<int> controlFrames{0};
    std::atomic<int> queryFrames{0};
    std::atomic<int> unknownFrames{0};

private:
    void printHex(const char* tag, const std::uint8_t* data, std::size_t size,
                  FrameKind kind)
    {
        const char* k = kind == FrameKind::Control
                            ? "control"
                            : (kind == FrameKind::Query ? "query" : "unknown");
        std::printf("%s t=%lldms n=%zu kind=%s bytes", tag, elapsedMs(), size,
                    k);
        for (std::size_t i = 0; i < size; ++i)
        {
            std::printf(" %02X", data[i]);
        }
        std::printf("\n");
        std::fflush(stdout);
    }

    std::shared_ptr<IPelcoTransport> inner_;
};

// ---------------------------------------------------------------------------
// 许可文件：严格的 key=value，拒绝缺项、重复项与未知项。
// ---------------------------------------------------------------------------

std::string trim(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return std::string();
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

bool parseLongLong(const std::string& text, long long& out)
{
    if (text.empty())
    {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const long long value = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0')
    {
        return false;
    }
    out = value;
    return true;
}

bool parseDouble(const std::string& text, double& out)
{
    if (text.empty())
    {
        return false;
    }
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == nullptr || *end != '\0' ||
        !std::isfinite(value))
    {
        return false;
    }
    out = value;
    return true;
}

bool loadPermitFile(const std::string& path, TurntableAcceptancePermit& permit,
                    std::string& error)
{
    std::ifstream input(path);
    if (!input)
    {
        error = "无法打开许可文件：" + path;
        return false;
    }

    std::vector<std::string> seen;
    auto markSeen = [&](const std::string& key) {
        for (const auto& s : seen)
        {
            if (s == key)
            {
                return false;
            }
        }
        seen.push_back(key);
        return true;
    };

    std::string line;
    int lineNo = 0;
    while (std::getline(input, line))
    {
        ++lineNo;
        const std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#')
        {
            continue;
        }
        const auto eq = stripped.find('=');
        if (eq == std::string::npos)
        {
            error = "许可第 " + std::to_string(lineNo) + " 行缺少 '='";
            return false;
        }
        const std::string key   = trim(stripped.substr(0, eq));
        std::string value       = trim(stripped.substr(eq + 1));
        if (!value.empty() && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.size() - 2);
        }
        if (!markSeen(key))
        {
            error = "许可键重复：" + key;
            return false;
        }

        long long i = 0;
        double    d = 0.0;
        if (key == "scope_id")
        {
            permit.scopeId = value;
        }
        else if (key == "device_host")
        {
            permit.deviceHost = value;
        }
        else if (key == "device_port")
        {
            if (!parseLongLong(value, i))
            {
                error = "device_port 不是整数";
                return false;
            }
            permit.devicePort = static_cast<int>(i);
        }
        else if (key == "pelco_address")
        {
            if (!parseLongLong(value, i))
            {
                error = "pelco_address 不是整数";
                return false;
            }
            permit.pelcoAddress = static_cast<int>(i);
        }
        else if (key == "allow_absolute_position")
        {
            if (!parseLongLong(value, i) || (i != 0 && i != 1))
            {
                error = "allow_absolute_position 必须是 0 或 1";
                return false;
            }
            permit.allowAbsolutePosition = i == 1;
        }
        else if (key == "allow_stop_at_rest")
        {
            if (!parseLongLong(value, i) || (i != 0 && i != 1))
            {
                error = "allow_stop_at_rest 必须是 0 或 1";
                return false;
            }
            permit.allowStopFrameAtRest = i == 1;
        }
        else if (key == "azimuth_min_deg")
        {
            if (!parseDouble(value, d)) { error = key + " 不是有限数"; return false; }
            permit.azimuthMinDeg = d;
        }
        else if (key == "azimuth_max_deg")
        {
            if (!parseDouble(value, d)) { error = key + " 不是有限数"; return false; }
            permit.azimuthMaxDeg = d;
        }
        else if (key == "elevation_min_deg")
        {
            if (!parseDouble(value, d)) { error = key + " 不是有限数"; return false; }
            permit.elevationMinDeg = d;
        }
        else if (key == "elevation_max_deg")
        {
            if (!parseDouble(value, d)) { error = key + " 不是有限数"; return false; }
            permit.elevationMaxDeg = d;
        }
        else if (key == "max_displacement_deg")
        {
            if (!parseDouble(value, d)) { error = key + " 不是有限数"; return false; }
            permit.maxDisplacementDeg = d;
        }
        else if (key == "max_executions")
        {
            if (!parseLongLong(value, i))
            {
                error = "max_executions 不是整数";
                return false;
            }
            permit.maxExecutions = static_cast<int>(i);
        }
        else if (key == "valid_from_epoch_ms")
        {
            if (!parseLongLong(value, i))
            {
                error = "valid_from_epoch_ms 不是整数毫秒";
                return false;
            }
            permit.validFromEpochMs = i;
        }
        else if (key == "valid_until_epoch_ms")
        {
            if (!parseLongLong(value, i))
            {
                error = "valid_until_epoch_ms 不是整数毫秒";
                return false;
            }
            permit.validUntilEpochMs = i;
        }
        else
        {
            error = "许可含未知键：" + key;
            return false;
        }
    }

    const std::vector<std::string> required = {
        "scope_id", "device_host", "device_port", "pelco_address",
        "allow_absolute_position", "allow_stop_at_rest",
        "azimuth_min_deg", "azimuth_max_deg",
        "elevation_min_deg", "elevation_max_deg", "max_displacement_deg",
        "max_executions", "valid_from_epoch_ms", "valid_until_epoch_ms"};
    for (const auto& key : required)
    {
        bool present = false;
        for (const auto& s : seen)
        {
            if (s == key) { present = true; break; }
        }
        if (!present)
        {
            error = "许可缺少必需键：" + key;
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// 现场记录用的辅助
// ---------------------------------------------------------------------------

struct MoveTrack
{
    bool          active          = false;
    bool          stopRequested   = false;
    bool          outcomeReported = false;
    bool          arrivalReported = false;
    std::uint64_t seq             = 0;
    double        targetAz        = 0.0;
    double        targetEl        = 0.0;
    std::uint64_t refCycle        = 0;
    double        refAz           = 0.0;
    double        refEl           = 0.0;
    long long     acceptedAtMs    = 0;
};

struct StopWatch
{
    bool          awaitingStationary = false;
    std::uint64_t stopTxn            = 0;
    long long     requestedAtMs      = 0;
};

double azDelta(double a, double b)
{
    // [0, 180]，与生产判据一致。
    double d = std::fabs(a - b);
    while (d > 360.0)
    {
        d -= 360.0;
    }
    if (d > 180.0)
    {
        d = 360.0 - d;
    }
    return d;
}

bool parseNumber(const std::string& token, double& out)
{
    return parseDouble(token, out);
}

void printDiagnostics(const TurntableDiagnostics& d)
{
    std::printf(
        "DIAG t=%lldms sample_valid=%d fresh=%d cycle=%llu az=%.4f el=%.4f "
        "txn=%llu stop_txn=%llu stop_delivered=%d stop_stationary=%d "
        "uncertain=%d latched=%d wm=%s cmd_seq=%llu cmd=%s msg=\"%s\" "
        "stop_req=%llu stop_done=%llu inhibited=%d "
        "permit_installed=%d permit_expired=%d permit_remaining=%d "
        "permit_scope=\"%s\" first_fault=%d \"%s\"\n",
        elapsedMs(), d.sample.valid ? 1 : 0, d.sampleFresh ? 1 : 0,
        static_cast<unsigned long long>(d.sample.cycleId),
        d.sample.azimuth, d.sample.elevation,
        static_cast<unsigned long long>(d.motionTransactionId),
        static_cast<unsigned long long>(d.stopFactsTransactionId),
        d.stopFrameDelivered ? 1 : 0, d.stopObservedStationary ? 1 : 0,
        d.motionUncertain ? 1 : 0, d.errorLatched ? 1 : 0,
        trustName(d.workModeTrust),
        static_cast<unsigned long long>(d.lastCommandSeq),
        outcomeName(d.lastCommandOutcome), d.lastCommandMessage.c_str(),
        static_cast<unsigned long long>(d.stopRequestSeq),
        static_cast<unsigned long long>(d.stopCompletedSeq),
        d.motionInhibited ? 1 : 0,
        d.acceptancePermitInstalled ? 1 : 0,
        d.acceptancePermitExpired ? 1 : 0,
        d.acceptancePermitRemaining, d.acceptancePermitScope.c_str(),
        d.firstFault.code, d.firstFault.message.c_str());
    std::fflush(stdout);
}

void printState(const TurntableState& s, const TurntableDiagnostics& d)
{
    std::printf(
        "STATE t=%lldms az=%.4f el=%.4f motion=%s valid=%d fresh=%d "
        "cycle=%llu txn=%llu permit_remaining=%d\n",
        elapsedMs(), s.azimuth, s.elevation, motionName(s.motion),
        d.sample.valid ? 1 : 0, d.sampleFresh ? 1 : 0,
        static_cast<unsigned long long>(d.sample.cycleId),
        static_cast<unsigned long long>(d.motionTransactionId),
        d.acceptancePermitRemaining);
    std::fflush(stdout);
}

/// 读取一行；EOF 返回 false。超时（timeoutMs）返回 true 但 line 为空。
bool readLineWithTimeout(int timeoutMs, std::string& line, bool& eof)
{
    eof = false;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval tv;
    tv.tv_sec  = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    const int ready = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
    if (ready < 0)
    {
        // 信号中断不是 EOF：主循环据此走“请求停止 → 正常退出”路径，
        // 而不是把 SIGINT 当成输入结束。
        if (errno == EINTR)
        {
            line.clear();
            return true;
        }
        eof = true;
        return false;
    }
    if (ready == 0)
    {
        line.clear();
        return true;  // timeout
    }

    line.clear();
    char c = 0;
    while (true)
    {
        const ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n <= 0)
        {
            eof = true;
            return false;
        }
        if (c == '\n')
        {
            break;
        }
        line.push_back(c);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    g_start = Clock::now();

    std::signal(SIGINT, onTerminationSignal);
    std::signal(SIGTERM, onTerminationSignal);

    std::string configDir = "config";
    std::string permitPath;
    long long observeMs = 0;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc)
        {
            configDir = argv[++i];
        }
        else if (arg == "--permit" && i + 1 < argc)
        {
            permitPath = argv[++i];
        }
        else if (arg == "--observe-ms" && i + 1 < argc)
        {
            observeMs = std::atoll(argv[++i]);
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::printf(
                "usage: %s [--config DIR] [--permit FILE] [--observe-ms N]\n"
                "  --config DIR     配置目录（默认 config）\n"
                "  --permit FILE    仅限本次验收的运动许可；缺省 = 只读\n"
                "  --observe-ms N   初始化后只观测 N ms 再正常退出\n"
                "交互命令：status | move <az> <el> | move az <deg> |\n"
                "          move el <deg> | stop | stop_at_rest | quit\n"
                "  stop         正常停止：运动中发帧，静止时无帧 no-op\n"
                "  stop_at_rest 验收专用：静止时实际发一帧停止（需许可\n"
                "               allow_stop_at_rest=1；每次会话最多一次）\n",
                argv[0]);
            return 0;
        }
        else
        {
            std::fprintf(stderr, "未知参数：%s\n", arg.c_str());
            return 2;
        }
    }

    std::printf("TOOL turntable_acceptance version=%s\n", kToolVersion);
    std::fflush(stdout);

    // ---- 1. 真实配置 ----
    ConfigManager config;
    if (!config.load(configDir))
    {
        std::printf("CONFIG_FAILED dir=\"%s\"\n", configDir.c_str());
        for (const auto& e : config.errors())
        {
            std::printf("CONFIG_ERROR %s\n", e.c_str());
        }
        std::fflush(stdout);
        return 2;
    }
    TurntableConfig cfg = config.turntable();
    std::printf(
        "CONFIG dir=\"%s\" summary=\"%s\" protocol=%s port=%d "
        "pelco_address=%d host=masked azimuth=[%.3f,%.3f] "
        "elevation=[%.3f,%.3f] use_speed_control=%d "
        "fine_speed=%.3f coarse_speed=%.3f sample_stale_ms=%d "
        "stop_timeout_ms=%d stop_retry_count=%d stop_retry_window_ms=%d\n",
        configDir.c_str(), config.summary().c_str(), cfg.protocol.c_str(),
        cfg.port, cfg.pelcoAddress, cfg.azimuthMin, cfg.azimuthMax,
        cfg.elevationMin, cfg.elevationMax, cfg.useSpeedControl ? 1 : 0,
        cfg.fineSpeed, cfg.coarseSpeed, cfg.sampleStaleMs, cfg.stopTimeoutMs,
        cfg.stopRetryCount, cfg.stopRetryWindowMs);
    for (const auto& w : config.warnings())
    {
        std::printf("CONFIG_WARN %s\n", w.c_str());
    }
    for (const auto& f : config.defaultedFields())
    {
        std::printf("CONFIG_DEFAULT %s\n", f.c_str());
    }
    if (cfg.useSpeedControl)
    {
        std::printf(
            "SPEED_NOTE use_speed_control=1：表 3.65 转速控制；"
            "位移 >1.0° 用 coarse_speed=%.3f °/s，否则 fine_speed=%.3f °/s；"
            "仅限本轮书面授权范围\n",
            cfg.coarseSpeed, cfg.fineSpeed);
    }
    else
    {
        std::printf(
            "SPEED_NOTE use_speed_control=0：表 3.5 设备默认速度；"
            "fine_speed/coarse_speed 不随命令发送\n");
    }
    std::fflush(stdout);

    if (cfg.protocol != "pelco_d_udp")
    {
        std::printf("ABORT protocol=%s 不是 pelco_d_udp\n",
                    cfg.protocol.c_str());
        return 2;
    }

    // ---- 2. 许可（可选）----
    TurntableAcceptancePermit permit;
    const bool havePermit = !permitPath.empty();
    if (havePermit)
    {
        std::string error;
        if (!loadPermitFile(permitPath, permit, error))
        {
            std::printf("PERMIT_PARSE_FAILED \"%s\"\n", error.c_str());
            return 2;
        }
        // 表 3.65（use_speed_control=true）在提供许可时按本地配置启用；
        // 速度值只来自配置，本工具没有命令行速度参数，也不会自动改配置。
        // 代码内的显式选择：只有提供许可时，本验收工具才把会话设为
        // MotionEnabled。config/ 没有这个键，生产装配也没有这条路径。
        cfg.sendPolicy = TurntableSendPolicy::MotionEnabled;
        std::printf(
            "PERMIT scope=\"%s\" op=absolute_position allow_stop_at_rest=%d "
            "az=[%.3f,%.3f] "
            "el=[%.3f,%.3f] max_displacement=%.3f max_executions=%d "
            "valid_from_ms=%lld valid_until_ms=%lld file=\"%s\"\n",
            permit.scopeId.c_str(),
            permit.allowStopFrameAtRest ? 1 : 0,
            permit.azimuthMinDeg,
            permit.azimuthMaxDeg, permit.elevationMinDeg,
            permit.elevationMaxDeg, permit.maxDisplacementDeg,
            permit.maxExecutions,
            static_cast<long long>(permit.validFromEpochMs),
            static_cast<long long>(permit.validUntilEpochMs),
            permitPath.c_str());
    }
    else
    {
        std::printf("PERMIT none：只读会话，控制帧恒为 0\n");
    }
    std::printf("SEND_POLICY %s\n",
                cfg.sendPolicy == TurntableSendPolicy::ReadOnly
                    ? "read_only"
                    : "motion_enabled(acceptance_permit)");
    std::fflush(stdout);

    auto transport = std::make_shared<LoggingCountingTransport>(
        makeUdpPelcoTransport());

    int exitCode = 0;
    bool initialized = false;
    bool normalExit  = false;
    TurntableDiagnostics finalDiag;

    {
        PelcoDTurntableController controller(cfg, transport);

        if (havePermit)
        {
            std::string error;
            if (!controller.enableAcceptancePermit(permit, error))
            {
                std::printf("PERMIT_REJECTED \"%s\"\n", error.c_str());
                std::printf(
                    "CONTROL_FRAMES sent=%d（未打开 socket，未发任何帧）\n",
                    transport->controlFrames.load());
                return 2;
            }
            std::printf("PERMIT_INSTALLED scope=\"%s\"\n",
                        permit.scopeId.c_str());
            std::fflush(stdout);
        }

        std::printf("INITIALIZE...\n");
        std::fflush(stdout);
        if (!controller.initialize())
        {
            std::printf("INIT_FAILED \"%s\"\n",
                        controller.lastErrorText().c_str());
            std::printf("CONTROL_FRAMES sent=%d\n",
                        transport->controlFrames.load());
            return 1;
        }
        initialized = true;
        std::printf("INIT_OK\n");
        printState(controller.state(), controller.diagnostics());

        MoveTrack track;
        StopWatch stopWatch;
        std::vector<double> arrivalAz;
        std::vector<double> arrivalEl;
        std::vector<double> arrivalTargetAz;
        std::vector<double> arrivalTargetEl;

        bool quit = false;
        bool interrupted = false;
        TurntableMotionState lastMotion = controller.state().motion;
        long long lastStatusMs = -1000;

        const long long observeDeadline =
            observeMs > 0 ? elapsedMs() + observeMs : -1;

        while (!quit)
        {
            if (g_interrupted)
            {
                interrupted = true;
                quit = true;
                std::printf(
                    "SIGNAL_INTERRUPT t=%lldms：进入停止/正常退出路径\n",
                    elapsedMs());
                std::fflush(stdout);
                break;
            }

            // ---- 输入（非阻塞，200 ms 切片）----
            // 观测模式完全不吃 stdin：即使 stdin 是 /dev/null 也不会
            // 提前退出，T0 的观测时长是确定的。
            std::string line;
            if (observeMs > 0)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(200));
                if (elapsedMs() >= observeDeadline)
                {
                    quit = true;
                    std::printf(
                        "OBSERVE_DONE t=%lldms requested_ms=%lld\n",
                        elapsedMs(), observeMs);
                    break;
                }
            }
            else
            {
                bool eof = false;
                const bool inputOk = readLineWithTimeout(200, line, eof);
                if (!inputOk && eof)
                {
                    quit = true;
                    std::printf("INPUT_EOF t=%lldms\n", elapsedMs());
                    break;
                }
            }
            const std::string cmd = trim(line);

            if (!cmd.empty())
            {
                std::printf("USER t=%lldms cmd=\"%s\"\n", elapsedMs(),
                            cmd.c_str());
                std::fflush(stdout);

                if (cmd == "quit" || cmd == "q")
                {
                    quit = true;
                }
                else if (cmd == "status")
                {
                    printState(controller.state(),
                               controller.diagnostics());
                    printDiagnostics(controller.diagnostics());
                }
                else if (cmd == "stop")
                {
                    const TurntableState before = controller.state();
                    const TurntableDiagnostics d0 = controller.diagnostics();
                    std::printf(
                        "STOP_REQUEST t=%lldms at_motion=%s cycle=%llu "
                        "az=%.4f el=%.4f fresh=%d txn=%llu stop_txn=%llu\n",
                        elapsedMs(), motionName(before.motion),
                        static_cast<unsigned long long>(
                            d0.sample.cycleId),
                        before.azimuth, before.elevation,
                        d0.sampleFresh ? 1 : 0,
                        static_cast<unsigned long long>(
                            d0.motionTransactionId),
                        static_cast<unsigned long long>(
                            d0.stopFactsTransactionId));
                    std::fflush(stdout);

                    const std::uint64_t motionTxnBefore =
                        d0.motionTransactionId;
                    const int controlBefore = transport->controlFrames.load();
                    controller.stop();
                    const TurntableDiagnostics d1 = controller.diagnostics();

                    // “停止帧已交付”与“根本不需要发帧”必须分开：静止时
                    // session 的 stop() 是无帧 no-op，不是交付失败。
                    const char* sendPath = "stop_frame_not_delivered";
                    if (cfg.sendPolicy == TurntableSendPolicy::ReadOnly)
                    {
                        sendPath = "no_frame(read_only)";
                    }
                    else if (d1.stopFrameDelivered)
                    {
                        sendPath = "stop_frame_delivered";
                    }
                    else if (transport->controlFrames.load() == controlBefore)
                    {
                        sendPath = "no_frame(no_motion_in_flight)";
                    }
                    std::printf(
                        "STOP_RESULT t=%lldms request_seq=%llu "
                        "completed_seq=%llu stop_delivered=%d "
                        "stop_stationary=%d uncertain=%d "
                        "stop_txn=%llu motion_txn_before=%llu "
                        "send_path=%s\n",
                        elapsedMs(),
                        static_cast<unsigned long long>(d1.stopRequestSeq),
                        static_cast<unsigned long long>(d1.stopCompletedSeq),
                        d1.stopFrameDelivered ? 1 : 0,
                        d1.stopObservedStationary ? 1 : 0,
                        d1.motionUncertain ? 1 : 0,
                        static_cast<unsigned long long>(
                            d1.stopFactsTransactionId),
                        static_cast<unsigned long long>(motionTxnBefore),
                        sendPath);

                    if (d1.stopFrameDelivered &&
                        !d1.stopObservedStationary &&
                        !d1.motionUncertain)
                    {
                        stopWatch.awaitingStationary = true;
                        stopWatch.stopTxn =
                            d1.stopFactsTransactionId;
                        stopWatch.requestedAtMs = elapsedMs();
                    }
                    if (track.active)
                    {
                        // 停止请求之后的反馈角不再作为“到位”判据。
                        track.stopRequested = true;
                    }
                    std::fflush(stdout);
                }
                else if (cmd == "stop_at_rest")
                {
                    // 验收 T1：在**静止**状态下实际发送一帧停止指令。
                    // 与 stop 的区别：stop 在静止时是无帧 no-op（C1），
                    // 本命令用于证明“许可允许时，停止帧确实能发出”。
                    const TurntableState before = controller.state();
                    const TurntableDiagnostics d0 = controller.diagnostics();
                    std::printf(
                        "STOP_AT_REST_REQUEST t=%lldms at_motion=%s "
                        "cycle=%llu az=%.4f el=%.4f fresh=%d "
                        "stop_delivered=%d\n",
                        elapsedMs(), motionName(before.motion),
                        static_cast<unsigned long long>(
                            d0.sample.cycleId),
                        before.azimuth, before.elevation,
                        d0.sampleFresh ? 1 : 0,
                        d0.stopFrameDelivered ? 1 : 0);
                    std::fflush(stdout);

                    const int controlBefore = transport->controlFrames.load();
                    std::string error;
                    const bool ok =
                        controller.sendAcceptanceStopFrameAtRest(error);
                    const TurntableDiagnostics d1 = controller.diagnostics();
                    const int controlAfter = transport->controlFrames.load();
                    std::printf(
                        "STOP_AT_REST_RESULT t=%lldms ok=%d "
                        "control_frames_added=%d stop_delivered=%d "
                        "stop_stationary=%d uncertain=%d stop_txn=%llu "
                        "msg=\"%s\"\n",
                        elapsedMs(), ok ? 1 : 0,
                        controlAfter - controlBefore,
                        d1.stopFrameDelivered ? 1 : 0,
                        d1.stopObservedStationary ? 1 : 0,
                        d1.motionUncertain ? 1 : 0,
                        static_cast<unsigned long long>(
                            d1.stopFactsTransactionId),
                        error.c_str());

                    if (ok && d1.stopFrameDelivered &&
                        !d1.stopObservedStationary &&
                        !d1.motionUncertain)
                    {
                        stopWatch.awaitingStationary = true;
                        stopWatch.stopTxn =
                            d1.stopFactsTransactionId;
                        stopWatch.requestedAtMs = elapsedMs();
                    }
                    if (track.active)
                    {
                        // 该停止帧之后的反馈角不再作为“到位”判据。
                        track.stopRequested = true;
                    }
                    std::fflush(stdout);
                }
                else if (cmd.rfind("move", 0) == 0)
                {
                    // move <az> <el> | move az <deg> | move el <deg>
                    std::vector<std::string> tokens;
                    std::size_t pos = 0;
                    while (pos < cmd.size())
                    {
                        const std::size_t sp = cmd.find(' ', pos);
                        const std::string tok =
                            cmd.substr(pos, sp == std::string::npos
                                                ? std::string::npos
                                                : sp - pos);
                        if (!tok.empty())
                        {
                            tokens.push_back(tok);
                        }
                        if (sp == std::string::npos)
                        {
                            break;
                        }
                        pos = sp + 1;
                    }

                    const TurntableState before = controller.state();
                    const TurntableDiagnostics d0 =
                        controller.diagnostics();
                    double az = 0.0;
                    double el = 0.0;
                    bool ok = false;
                    std::uint64_t refCycle = d0.sample.cycleId;
                    double refAz = d0.sample.azimuth;
                    double refEl = d0.sample.elevation;

                    if (tokens.size() == 3 && tokens[1] == "az")
                    {
                        ok = parseNumber(tokens[2], az);
                        el = refEl;
                        if (!d0.sampleFresh)
                        {
                            ok = false;
                            std::printf(
                                "MOVE_REJECTED reason=no_fresh_reference\n");
                        }
                    }
                    else if (tokens.size() == 3 && tokens[1] == "el")
                    {
                        ok = parseNumber(tokens[2], el);
                        az = refAz;
                        if (!d0.sampleFresh)
                        {
                            ok = false;
                            std::printf(
                                "MOVE_REJECTED reason=no_fresh_reference\n");
                        }
                    }
                    else if (tokens.size() == 3)
                    {
                        ok = parseNumber(tokens[1], az) &&
                             parseNumber(tokens[2], el);
                    }

                    if (!ok)
                    {
                        std::printf(
                            "MOVE_REJECTED reason=usage_or_nonfinite "
                            "（用法：move <az> <el> | move az <deg> | "
                            "move el <deg>）\n");
                    }
                    else
                    {
                        if (track.active && !track.outcomeReported)
                        {
                            std::printf(
                                "MOVE_OUTCOME_LOST seq=%llu "
                                "reason=superseded_before_observation\n",
                                static_cast<unsigned long long>(track.seq));
                        }

                        TurntableCommand command;
                        command.azimuthCommand   = az;
                        command.elevationCommand = el;

                        std::printf(
                            "MOVE_REQUEST t=%lldms az=%.4f el=%.4f "
                            "reference_cycle=%llu reference_az=%.4f "
                            "reference_el=%.4f reference_fresh=%d "
                            "at_motion=%s speed_control=%d\n",
                            elapsedMs(), az, el,
                            static_cast<unsigned long long>(refCycle),
                            refAz, refEl, d0.sampleFresh ? 1 : 0,
                            motionName(before.motion),
                            cfg.useSpeedControl ? 1 : 0);
                        std::fflush(stdout);

                        const bool accepted = controller.move(command);
                        // 用**本次调用分配**的序号，而不是诊断里的
                        // lastCommandSeq（后者可能还是上一条命令的结果）。
                        const std::uint64_t acceptedSeq =
                            controller.lastAcceptedCommandSeq();
                        const TurntableDiagnostics d1 =
                            controller.diagnostics();
                        std::printf(
                            "MOVE_ACCEPTED accepted=%d seq=%llu "
                            "（true 仅表示已受理，不表示交付或到位）\n",
                            accepted ? 1 : 0,
                            static_cast<unsigned long long>(acceptedSeq));
                        if (!accepted)
                        {
                            const TurntableDiagnostics d2 =
                                controller.diagnostics();
                            std::printf(
                                "MOVE_NOT_ACCEPTED seq=%llu outcome=%s "
                                "msg=\"%s\"\n",
                                static_cast<unsigned long long>(
                                    acceptedSeq),
                                outcomeName(d2.lastCommandOutcome),
                                d2.lastCommandMessage.c_str());
                        }
                        std::fflush(stdout);

                        if (accepted)
                        {
                            track.active          = true;
                            track.stopRequested   = false;
                            track.outcomeReported = false;
                            track.arrivalReported = false;
                            track.seq             = acceptedSeq;
                            track.targetAz        = az;
                            track.targetEl        = el;
                            track.refCycle        = refCycle;
                            track.refAz           = refAz;
                            track.refEl           = refEl;
                            track.acceptedAtMs    = elapsedMs();
                        }
                    }
                }
                else
                {
                    std::printf("UNKNOWN_COMMAND \"%s\"\n", cmd.c_str());
                }
                std::fflush(stdout);
            }

            // ---- 被动观测 ----
            const TurntableState state   = controller.state();
            const TurntableDiagnostics d = controller.diagnostics();

            if (state.motion != lastMotion)
            {
                std::printf("MOTION_CHANGE t=%lldms %s -> %s az=%.4f "
                            "el=%.4f cycle=%llu\n",
                            elapsedMs(), motionName(lastMotion),
                            motionName(state.motion), state.azimuth,
                            state.elevation,
                            static_cast<unsigned long long>(
                                d.sample.cycleId));
                lastMotion = state.motion;
                std::fflush(stdout);
            }

            if (track.active && !track.outcomeReported &&
                d.lastCommandSeq == track.seq &&
                d.lastCommandOutcome != TurntableCommandOutcome::None)
            {
                std::printf(
                    "CMD_RESULT seq=%llu outcome=%s msg=\"%s\" "
                    "delivered_is_arrival=0\n",
                    static_cast<unsigned long long>(track.seq),
                    outcomeName(d.lastCommandOutcome),
                    d.lastCommandMessage.c_str());
                track.outcomeReported = true;
                std::fflush(stdout);
                if (d.lastCommandOutcome ==
                    TurntableCommandOutcome::Failed)
                {
                    track.active = false;
                }
            }

            if (track.active && !track.stopRequested &&
                !track.arrivalReported && d.sampleFresh &&
                d.sample.valid &&
                state.motion == TurntableMotionState::STABLE &&
                azDelta(d.sample.azimuth, track.targetAz) <=
                    cfg.arrivalToleranceDeg &&
                std::fabs(d.sample.elevation - track.targetEl) <=
                    cfg.arrivalToleranceDeg)
            {
                std::printf(
                    "ARRIVAL_OBSERVED seq=%llu cycle=%llu az=%.4f "
                    "el=%.4f target=[%.4f,%.4f] elapsed_ms=%lld "
                    "feedback_only=1（设备反馈稳定，不等于独立实测静止，"
                    "也不等于绝对精度）\n",
                    static_cast<unsigned long long>(track.seq),
                    static_cast<unsigned long long>(d.sample.cycleId),
                    d.sample.azimuth, d.sample.elevation, track.targetAz,
                    track.targetEl,
                    elapsedMs() - track.acceptedAtMs);
                track.arrivalReported = true;
                arrivalAz.push_back(d.sample.azimuth);
                arrivalEl.push_back(d.sample.elevation);
                arrivalTargetAz.push_back(track.targetAz);
                arrivalTargetEl.push_back(track.targetEl);
                std::fflush(stdout);
            }

            if (track.active && state.motion == TurntableMotionState::ERROR &&
                !track.stopRequested)
            {
                std::printf(
                    "MOTION_ERROR seq=%llu latched=%d uncertain=%d "
                    "first_fault=%d \"%s\"\n",
                    static_cast<unsigned long long>(track.seq),
                    d.errorLatched ? 1 : 0, d.motionUncertain ? 1 : 0,
                    d.firstFault.code, d.firstFault.message.c_str());
                track.active = false;
                std::fflush(stdout);
            }

            if (stopWatch.awaitingStationary)
            {
                if (d.stopObservedStationary &&
                    d.stopFactsTransactionId == stopWatch.stopTxn)
                {
                    std::printf(
                        "STOP_OBSERVED_STATIONARY stop_txn=%llu "
                        "cycle=%llu az=%.4f el=%.4f elapsed_ms=%lld "
                        "（系统观测到反馈角在观测窗口内不变；"
                        "不是设备 ACK，也不是独立实测静止）\n",
                        static_cast<unsigned long long>(
                            stopWatch.stopTxn),
                        static_cast<unsigned long long>(
                            d.sample.cycleId),
                        d.sample.azimuth, d.sample.elevation,
                        elapsedMs() - stopWatch.requestedAtMs);
                    stopWatch.awaitingStationary = false;
                    std::fflush(stdout);
                }
                else if (d.motionUncertain ||
                         d.firstFault.code ==
                             aircraft::data::kErrTurntableStopNotObserved)
                {
                    std::printf(
                        "STOP_NOT_OBSERVED stop_txn=%llu uncertain=%d "
                        "first_fault=%d \"%s\"\n",
                        static_cast<unsigned long long>(
                            stopWatch.stopTxn),
                        d.motionUncertain ? 1 : 0, d.firstFault.code,
                        d.firstFault.message.c_str());
                    stopWatch.awaitingStationary = false;
                    std::fflush(stdout);
                }
            }

            // 周期性状态行（1 s），避免把 50 ms 轮询刷成日志墙。
            if (elapsedMs() - lastStatusMs >= 1000)
            {
                printState(state, d);
                lastStatusMs = elapsedMs();
            }
        }

        // ---- 正常退出：先停止，再销毁（stop + join + close）----
        std::printf("EXIT_REQUEST t=%lldms reason=%s\n", elapsedMs(),
                    interrupted ? "signal_interrupt"
                                : (quit ? "user_or_observe_done"
                                        : "unknown"));
        std::fflush(stdout);
        const TurntableDiagnostics beforeStop = controller.diagnostics();
        controller.stop();
        const long long stopWaitStart = elapsedMs();
        const long long stopWaitDeadline =
            stopWaitStart + cfg.stopTimeoutMs + 200;
        TurntableDiagnostics afterStop = controller.diagnostics();
        // 停止帧交付后，在不超过配置观测窗口的时间里等待
        // “观测到反馈角不变”。退出前把结果写清楚，不把
        // “已交付”当成“已静止”。
        while (afterStop.stopFrameDelivered &&
               !afterStop.stopObservedStationary &&
               !afterStop.motionUncertain &&
               elapsedMs() < stopWaitDeadline)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));
            afterStop = controller.diagnostics();
        }
        std::printf(
            "EXIT_STOP stop_req=%llu stop_done=%llu delivered=%d "
            "stationary=%d uncertain=%d waited_ms=%lld\n",
            static_cast<unsigned long long>(afterStop.stopRequestSeq),
            static_cast<unsigned long long>(afterStop.stopCompletedSeq),
            afterStop.stopFrameDelivered ? 1 : 0,
            afterStop.stopObservedStationary ? 1 : 0,
            afterStop.motionUncertain ? 1 : 0,
            elapsedMs() - stopWaitStart);
        finalDiag = afterStop;

        // 重复定位统计（只有人工逐条下发时才存在；不含任何自动循环）。
        // 按目标分组：往返于 A/B 两点时，混合统计会伪造“重复性”。
        if (arrivalAz.size() >= 2)
        {
            auto stats = [](const std::vector<double>& v, double& mean,
                            double& sd) {
                double sum = 0.0;
                for (double x : v) { sum += x; }
                mean = sum / static_cast<double>(v.size());
                double sq = 0.0;
                for (double x : v)
                {
                    sq += (x - mean) * (x - mean);
                }
                sd = std::sqrt(sq / static_cast<double>(v.size()));
            };
            std::vector<bool> used(arrivalAz.size(), false);
            for (std::size_t i = 0; i < arrivalAz.size(); ++i)
            {
                if (used[i])
                {
                    continue;
                }
                std::vector<double> azs;
                std::vector<double> els;
                for (std::size_t j = i; j < arrivalAz.size(); ++j)
                {
                    if (!used[j] &&
                        std::fabs(arrivalTargetAz[j] -
                                  arrivalTargetAz[i]) <= 1e-9 &&
                        std::fabs(arrivalTargetEl[j] -
                                  arrivalTargetEl[i]) <= 1e-9)
                    {
                        used[j] = true;
                        azs.push_back(arrivalAz[j]);
                        els.push_back(arrivalEl[j]);
                    }
                }
                if (azs.size() < 2)
                {
                    continue;
                }
                double meanAz = 0.0, sdAz = 0.0, meanEl = 0.0, sdEl = 0.0;
                stats(azs, meanAz, sdAz);
                stats(els, meanEl, sdEl);
                std::printf(
                    "REPEAT n=%zu target_az=%.4f target_el=%.4f "
                    "az_mean=%.4f az_std=%.4f el_mean=%.4f "
                    "el_std=%.4f device_feedback_only=1 "
                    "absolute_accuracy=NOT_MEASURED "
                    "arcmin_acceptance=NOT_MEASURED\n",
                    azs.size(), arrivalTargetAz[i], arrivalTargetEl[i],
                    meanAz, sdAz, meanEl, sdEl);
            }
        }

        (void)beforeStop;
        normalExit = true;
    }

    std::printf(
        "FINAL initialized=%d normal_exit=%d control_frames=%d "
        "query_frames=%d unknown_frames=%d stop_delivered=%d "
        "stop_stationary=%d uncertain=%d latched=%d first_fault=%d\n",
        initialized ? 1 : 0, normalExit ? 1 : 0,
        transport->controlFrames.load(), transport->queryFrames.load(),
        transport->unknownFrames.load(),
        finalDiag.stopFrameDelivered ? 1 : 0,
        finalDiag.stopObservedStationary ? 1 : 0,
        finalDiag.motionUncertain ? 1 : 0,
        finalDiag.errorLatched ? 1 : 0, finalDiag.firstFault.code);

    const bool readOnly =
        cfg.sendPolicy == TurntableSendPolicy::ReadOnly;
    if (readOnly && transport->controlFrames.load() != 0)
    {
        std::printf(
            "SELF_CHECK_FAILED read_only_session_sent_control_frames=%d\n",
            transport->controlFrames.load());
        exitCode = 3;
    }
    std::printf("EXIT_CODE %d\n", exitCode);
    std::fflush(stdout);
    return exitCode;
}
