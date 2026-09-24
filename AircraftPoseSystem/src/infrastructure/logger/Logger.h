#pragma once

// ============================================================================
//  src/infrastructure/logger/Logger.h
//
//  依据：ENG-01 §11（infrastructure/logger）、ENG-03 §12.7（libinfrastructure
//        含 Logger）、SYS-04 §6.5（IF-FILE-05 日志接口）、
//        ENG-02 §15（创建顺序：ConfigManager → Logger → …）
//
//  ⚠ 级别枚举是本文件**自定**的：SYS-04 §6.5 只冻结了
//    "日志级别由 `SystemConfig::logLevel` 控制"，未冻结取值含义；
//    ENG-09 也不含 LogLevel 条目。故此处定义 0..4 并与
//    SystemConfig::logLevel 的注释（"取值与 infrastructure/logger 的
//    级别枚举一致"）对齐。已登记为偏离（README §6）。
//
//  ⚠ 两套时间不要混用（ENG-09 §2.5）：
//    测量链的时间戳一律 CLOCK_MONOTONIC；而日志行的时间戳用**墙钟**，
//    因为现场排障时需要把日志与录像、其他系统日志对齐 —— 单调时钟
//    的读数在系统重启后归零，无法用于跨机比对。
//    反过来说，日志里的时间**不可**用于任何时延计算。
//
//  ⚠ 本类不是单例。理由与 ENG-10 §5.1 对 `ConfigManager::instance()`
//    的禁令同源：单例会让任意模块隐式依赖 infrastructure 层。
//    V2.1 第一阶段只有 app 打日志（11.md §九 的启动行），
//    各模块如何取到 Logger（构造注入 vs 全局宏）尚无冻结结论，
//    待裁决（见 裁决汇总.md）。
// ============================================================================

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace aircraft
{
namespace infrastructure
{

/// 日志级别。数值越小越详细，与 `SystemConfig::logLevel` 逐值对应。
enum class LogLevel
{
    DEBUG = 0,  ///< 逐帧/逐步细节，仅在排障时开启
    INFO  = 1,  ///< 常规运行事件（启动、设备状态、状态机迁移）
    WARN  = 2,  ///< 可继续但需关注（降级、使用了默认配置值）
    ERROR = 3,  ///< 操作失败，测量流程中断
    OFF   = 4   ///< 关闭全部输出
};

/// 级别名（"DEBUG" / "INFO" / …）。用于日志前缀与配置文件解析。
const char* logLevelName(LogLevel level);

/// 解析级别名（大小写不敏感）。无法识别时返回 false，**不修改** out ——
/// 让调用方决定是启动失败还是取默认值（ENG-10 §5.3）。
bool parseLogLevel(const std::string& text, LogLevel& out);

/// 级别取值是否在 [0,4] 内。供 ConfigManager 做越界校验（ENG-10 §5.3）。
bool isValidLogLevel(int level);

/// 进程级日志器（**按对象使用，非单例**，见文件头说明）。
///
/// 输出目标（SYS-04 §6.5"每模块独立日志前缀，统一写入 logs/"）：
///   - 标准输出：便于在前台直接观察（11.md §九 的 M1 验证方式）；
///   - `<logDir>/aps_YYYYMMDD.log`：追加写。
///
/// 行格式：`2026-09-23 14:05:01.123 [INFO ] [app] AircraftPoseSystem V2.1 start`
///         墙钟时间 + 级别（定宽 5 便于肉眼对齐）+ 模块名 + 正文。
///
/// 线程安全：log() 全程持锁。测量线程将来也要写日志（SYS-09 §12 的
/// measurement_xxx/log.txt 即取自同一批行），无锁会让日志交错成碎片。
class Logger
{
public:
    Logger() = default;
    ~Logger();
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;

    /// 打开日志文件。logDir 不存在时创建（含多级）。
    ///
    /// @param logDir 目录，取自 `SystemConfig::logDir`。
    /// @param level  级别，取自 `SystemConfig::logLevel`。
    /// @return 目录创建或文件打开失败时返回 false。**调用方须视为启动失败**
    ///         （ENG-10 §5.3 对配置缺失/越界一律失败的同一原则：
    ///         交付现场若日志写不出去，故障将无迹可查）。
    bool initialize(const std::string& logDir, int level);

    void setLevel(int level);
    LogLevel level() const;
    bool ready() const { return file_.is_open(); }

    /// 本次运行实际打开的日志文件路径。未 initialize 时为空串。
    /// 用途：启动时把路径打进日志本身（现场据此找到文件）。
    std::string filePath() const { return filePath_; }

    /// 已写出的行数（含未写入文件的 stdout 行）。
    uint64_t lineCount() const;

    void log(LogLevel level, const std::string& module, const std::string& message);

    // 便捷包装。module 为模块名（"app" / "device" / "algorithm" …）。
    void debug(const std::string& module, const std::string& message);
    void info(const std::string& module, const std::string& message);
    void warn(const std::string& module, const std::string& message);
    void error(const std::string& module, const std::string& message);

    /// 刷新文件缓冲。测量流程的关键节点（开始/结束/失败）后调用，
    /// 使崩溃时最后几行不留在缓冲区里。
    void flush();

    /// 关闭文件。重复调用无副作用。
    void close();

    /// 当前墙钟时刻，格式 "YYYY-MM-DD HH:MM:SS.mmm"。
    /// 独立成静态函数，供 Recorder 写 result.json 的 `written_at` 复用，
    /// 避免两处各写一份 strftime。
    static std::string wallClockText();

private:
    bool shouldLog(LogLevel level) const;

    mutable std::mutex mutex_;
    std::ofstream      file_;
    std::string        filePath_;
    std::string        logDir_;
    LogLevel           level_ = LogLevel::INFO;
    uint64_t           lines_ = 0;
};

}  // namespace infrastructure
}  // namespace aircraft
