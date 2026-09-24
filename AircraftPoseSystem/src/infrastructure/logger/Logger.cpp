// ============================================================================
//  src/infrastructure/logger/Logger.cpp
// ============================================================================

#include "infrastructure/logger/Logger.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>

#include "infrastructure/FileUtil.h"

namespace aircraft
{
namespace infrastructure
{

namespace
{

// 递归建目录的 makeDirectories() 原在此处（另有两份副本分别在
// persistence/FileMatchStatsStore.cpp 与 recorder/Recorder.cpp）。
// 三份**都漏掉了分隔符**，见 infrastructure/FileUtil.h 的文件头说明；
// 现已合并为该头文件中的唯一实现。

std::string dateStamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm           tmv {};
    ::localtime_r(&now, &tmv);
    char buf[16] = {0};
    std::strftime(buf, sizeof(buf), "%Y%m%d", &tmv);
    return std::string(buf);
}

}  // namespace

// ---------------------------------------------------------------------------

const char* logLevelName(LogLevel level)
{
    switch (level)
    {
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::INFO:  return "INFO";
    case LogLevel::WARN:  return "WARN";
    case LogLevel::ERROR: return "ERROR";
    case LogLevel::OFF:   return "OFF";
    }
    // 不可达：switch 已穷尽 5 个枚举值（-Wswitch 会在此处报缺失）。
    return "?";
}

bool parseLogLevel(const std::string& text, LogLevel& out)
{
    std::string upper;
    upper.reserve(text.size());
    for (const char c : text)
    {
        upper.push_back(
            static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }

    if (upper == "DEBUG") { out = LogLevel::DEBUG; return true; }
    if (upper == "INFO")  { out = LogLevel::INFO;  return true; }
    if (upper == "WARN" || upper == "WARNING") { out = LogLevel::WARN; return true; }
    if (upper == "ERROR") { out = LogLevel::ERROR; return true; }
    if (upper == "OFF" || upper == "NONE")     { out = LogLevel::OFF;   return true; }
    return false;
}

bool isValidLogLevel(int level)
{
    return level >= static_cast<int>(LogLevel::DEBUG)
        && level <= static_cast<int>(LogLevel::OFF);
}

// ---------------------------------------------------------------------------

Logger::~Logger()
{
    // 不在此处打日志（对象正在析构）。
    close();
}

bool Logger::initialize(const std::string& logDir, int level)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!isValidLogLevel(level))
    {
        // 越界值由 ConfigManager 在加载阶段拦截（ENG-10 §5.3），
        // 走到这里说明调用方跳过了校验 —— 拒绝而不是静默钳位：
        // 钳位会让"配置写错了"变成一件看不见的事。
        return false;
    }

    logDir_ = logDir;
    level_  = static_cast<LogLevel>(level);

    if (!fileutil::makeDirectories(logDir_))
    {
        return false;
    }

    filePath_ = fileutil::joinPath(logDir_, "aps_" + dateStamp() + ".log");
    file_.open(filePath_, std::ios::out | std::ios::app);
    if (!file_.is_open())
    {
        filePath_.clear();
        return false;
    }

    lines_ = 0;
    return true;
}

void Logger::setLevel(int level)
{
    if (!isValidLogLevel(level))
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    level_ = static_cast<LogLevel>(level);
}

LogLevel Logger::level() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return level_;
}

uint64_t Logger::lineCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lines_;
}

bool Logger::shouldLog(LogLevel level) const
{
    // 调用方须已持锁（本函数不改状态，仅读 level_）。
    if (level_ == LogLevel::OFF)
    {
        return false;
    }
    return static_cast<int>(level) >= static_cast<int>(level_);
}

void Logger::log(LogLevel level, const std::string& module, const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!shouldLog(level))
    {
        return;
    }

    // 级别定宽 5：`[INFO ]` / `[ERROR]` —— 定宽使各行的模块名与正文
    // 起始列对齐，现场用 less/记事本翻日志时不必逐行寻找正文起点。
    std::ostringstream line;
    line << wallClockText() << " [" << logLevelName(level) << "] ["
         << (module.empty() ? "?" : module) << "] " << message;

    ++lines_;

    // stdout 与文件各写一份。
    //
    // ⚠ **每行都 flush**（2026-09-23 实测后修改）。原先这里只写 '\n'，
    //    理由是"endl 每次强制 flush，逐帧日志下会成为性能项，需即时可见的
    //    关键节点由调用方显式 flush()"。该理由有两个漏洞，实测都发生了：
    //
    //    1. **没有任何调用方会去 flush()。** 现场最需要日志的时刻恰恰是
    //       进程非正常结束（崩溃、被 kill、被 timeout 终止）—— 而那时
    //       缓冲区里的最后几十行永远丢掉了。实测：以 SIGTERM 结束进程后，
    //       `logs/aps_YYYYMMDD.log` 是 **0 字节**，尽管程序已正常跑过
    //       整个启动流程并打印了 8 行 INFO。一个"事后才有用"的文件
    //       却在事件发生时就丢了内容，这使日志在最需要它的场合失效。
    //    2. 省下的开销被高估了。flush 是一次 write 系统调用（约 1~2 µs），
    //       而 INFO 级别的行频是"每次状态迁移一行"（整个测量 60 秒内
    //       不过几十行）。即使在 DEBUG 的逐帧日志下，2 µs 相对一帧的
    //       图像处理（毫秒级）也可以忽略 —— 若真的成为瓶颈，
    //       正确的应对是把级别调回 INFO，而不是丢掉日志。
    //
    //    FILE* 的 stdout 在**重定向到文件**时是全缓冲（4 KB），只有在
    //    终端里才是行缓冲 —— 这一点也实测到了：`> out.txt` 时最后的
    //    启动结论全部滞留在缓冲区里，看起来像是"日志在预览之后就不打了"。
    std::cout << line.str() << '\n';
    std::cout.flush();
    if (file_.is_open())
    {
        file_ << line.str() << '\n';
        file_.flush();
    }
}

void Logger::debug(const std::string& module, const std::string& message)
{
    log(LogLevel::DEBUG, module, message);
}

void Logger::info(const std::string& module, const std::string& message)
{
    log(LogLevel::INFO, module, message);
}

void Logger::warn(const std::string& module, const std::string& message)
{
    log(LogLevel::WARN, module, message);
}

void Logger::error(const std::string& module, const std::string& message)
{
    log(LogLevel::ERROR, module, message);
}

void Logger::flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout.flush();
    if (file_.is_open())
    {
        file_.flush();
    }
}

void Logger::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open())
    {
        file_.flush();
        file_.close();
    }
}

std::string Logger::wallClockText()
{
    const auto now = std::chrono::system_clock::now();
    const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now.time_since_epoch())
                         .count()
                   % 1000;

    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm           tmv {};
    ::localtime_r(&tt, &tmv);

    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);

    char out[40] = {0};
    std::snprintf(out, sizeof(out), "%s.%03d", buf, static_cast<int>(ms));
    return std::string(out);
}

}  // namespace infrastructure
}  // namespace aircraft
