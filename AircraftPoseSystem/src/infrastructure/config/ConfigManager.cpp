// ============================================================================
//  src/infrastructure/config/ConfigManager.cpp
//
//  加载 7 个 yaml。三级失败处理见头文件说明（ENG-10 §5.3）。
//
//  ⚠ 用 OpenCV 的 cv::FileStorage 而不是自己写 YAML 解析器：
//    本工程已硬依赖 OpenCV（libdata 的 PUBLIC 依赖），而标定文件与
//    特征库（TargetModelManager）也走同一条路径。再引入一个 YAML 库
//    只会多一份版本约束（现场一体机离线部署，依赖越少越好）。
//    代价：FileStorage 的 YAML 子集较窄（见下方 kYamlHeader 说明），
//    故配置文件一律按 OpenCV 自己写出的风格手写。
//
//  ⚠ 取值越界的判据来源逐条注明。凡冻结文档未给出数值的（如
//    measurement.yaml 的门槛值），本文件只做"合法性"校验（非负、
//    区间有序、非全零），不校验"合理性"——后者属于标定工作，
//    在代码里编一组"看起来合理"的阈值比留空更危险。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "infrastructure/config/ConfigManager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "infrastructure/logger/Logger.h"

namespace aircraft
{
namespace infrastructure
{

namespace
{

// ---- 7 个配置文件名（ENG-01 §3.2，顺序即加载顺序）----
const char* const kFileSystem      = "system.yaml";
const char* const kFileCamera      = "camera.yaml";
const char* const kFileOpticalRig  = "optical_rig.yaml";
const char* const kFileTurntable   = "turntable.yaml";
const char* const kFileTrigger     = "trigger.yaml";
const char* const kFileMeasurement = "measurement.yaml";
const char* const kFileValidation  = "validation.yaml";

// ---- SYS-10 §3.2 / §3.3 冻结的转台机械参数 ----
constexpr double kAzimuthLimitDeg   = 180.0;  ///< 水平 360° → ±180°
constexpr double kElevationLimitDeg = 60.0;   ///< 俯仰 -60° ~ +60°
constexpr double kAxisSpeedMinDegS  = 1.2;    ///< 两轴最低速度
constexpr double kAxisSpeedMaxDegS  = 30.0;   ///< 取俯仰轴上限（两轴共用的可行上限）

// ---- 焦距（CameraChannel::focalLength，单位 m）----
// 25 / 50 / 100 mm 三档，允许 ±30% 的批次差异；上界设在 0.5 m
// 是为了拦住"把 100 写成 100.0（mm 当 m）"这类单位错误：
// 100.0 远超 0.5，直接判为越界并给出提示。
constexpr double kFocalMinM = 0.001;
constexpr double kFocalMaxM = 0.5;

// ---- 相机角色名（ENG-09 §4.1 冻结的三个取值）----
// 同时接受 "CAM25" 与 "25" 两种写法：前者与枚举名逐字一致（推荐），
// 后者在现场手写配置时更省事。两者都映射到同一个枚举，不会产生歧义。
bool parseRole(const std::string& text, data::CameraRole& out)
{
    if (text == "CAM25" || text == "25")  { out = data::CameraRole::CAM25;  return true; }
    if (text == "CAM50" || text == "50")  { out = data::CameraRole::CAM50;  return true; }
    if (text == "CAM100" || text == "100") { out = data::CameraRole::CAM100; return true; }
    return false;
}

/// 字段读取器：统一实现 ENG-10 §5.3 的"字段缺失 → 默认值 + 记录字段名"。
///
/// 为什么把"缺失"与"取默认值"绑在一个函数里：分开写时很容易出现
/// `double v = node.empty() ? def : (double)node;` 这样忘了登记的写法，
/// 而漏登记的后果是"配置少填一项"重新变成静默事件 —— 正是 §5.3 要消除的。
class FieldReader
{
public:
    FieldReader(std::string group,
                std::vector<std::string>& defaults,
                std::vector<std::string>& warnings,
                std::vector<std::string>& errors)
        : group_(std::move(group))
        , defaults_(defaults)
        , warnings_(warnings)
        , errors_(errors)
    {
    }

    double real(const cv::FileNode& node, const char* key, double fallback)
    {
        if (missing(node, key, /*warnOnType=*/true))
        {
            return fallback;
        }
        return node.real();
    }

    int integer(const cv::FileNode& node, const char* key, int fallback)
    {
        if (missing(node, key, /*warnOnType=*/true))
        {
            return fallback;
        }
        return static_cast<int>(node.real());
    }

    /// 纳秒时长（超时、同步容差）。
    ///
    /// ⚠ 这里有一个**实测过的静默截断陷阱**，是本函数存在的唯一理由：
    ///   `cv::FileStorage` 的整数节点是 **32 位**的，而本工程的超时值
    ///   动辄 6e10（60 s）。若在 yaml 里写成朴素整数
    ///       task_timeout_ns: 60000000000
    ///   读回来是 **-129542144**（已实测），既不报错也不为空 ——
    ///   后续"非零即为合法"的校验会放它过去，超时判据随之完全失效。
    ///   故：负值一律判为**启动失败**并提示改用科学计数法
    ///   （正确写法 `6.0e10`，此时节点是 real 而非 int，不受 32 位限制）。
    uint64_t nanoseconds(const cv::FileNode& node, const char* key, uint64_t fallback)
    {
        if (missing(node, key, /*warnOnType=*/true))
        {
            return fallback;
        }
        const double v = node.real();
        if (!(v >= 0.0))
        {
            errors_.push_back(fullName(key) + " 读得负值 " + std::to_string(v) +
                               "：疑似 32 位整数截断，请改用科学计数法"
                               "（如 6.0e10 表示 60 s）");
            return fallback;
        }
        return static_cast<uint64_t>(v);
    }

    std::string text(const cv::FileNode& node, const char* key,
                     const std::string& fallback)
    {
        if (missing(node, key, /*warnOnType=*/false))
        {
            return fallback;
        }
        if (!node.isString())
        {
            warnings_.push_back(fullName(key) + "：应为字符串，已取默认值");
            defaults_.push_back(fullName(key));
            return fallback;
        }
        return node.string();
    }

    /// 布尔：同时接受 `1/0` 与 `"true"/"false"`。
    /// 不强制单一写法的原因：OpenCV 自己写出的是 1/0，而人手写配置时
    /// 习惯写 true/false —— 两种都要能用，且都不得静默取默认值。
    bool boolean(const cv::FileNode& node, const char* key, bool fallback)
    {
        if (missing(node, key, /*warnOnType=*/false))
        {
            return fallback;
        }
        if (node.isString())
        {
            const std::string s = node.string();
            if (s == "true" || s == "True" || s == "TRUE")   { return true; }
            if (s == "false" || s == "False" || s == "FALSE") { return false; }
            warnings_.push_back(fullName(key) + "：无法识别的布尔字面量 \"" + s +
                                "\"，已取默认值");
            defaults_.push_back(fullName(key));
            return fallback;
        }
        return node.real() != 0.0;
    }

    /// 固定长度数组（如 distance_band_edges 的 3 个边界）。
    /// 返回实际读到的个数；少于 expected 时补 fallback 并登记缺失。
    int reals(const cv::FileNode& node, const char* key, double* out, int expected,
              const double* fallback)
    {
        if (missing(node, key, /*warnOnType=*/false))
        {
            for (int i = 0; i < expected; ++i)
            {
                out[i] = fallback[i];
            }
            return 0;
        }
        if (!node.isSeq())
        {
            warnings_.push_back(fullName(key) + "：应为序列，已取默认值");
            defaults_.push_back(fullName(key));
            for (int i = 0; i < expected; ++i)
            {
                out[i] = fallback[i];
            }
            return 0;
        }

        int i = 0;
        for (auto it = node.begin(); it != node.end() && i < expected; ++it, ++i)
        {
            out[i] = (*it).real();
        }
        if (i < expected)
        {
            warnings_.push_back(fullName(key) + "：元素个数不足（" +
                                std::to_string(i) + " < " + std::to_string(expected) +
                                "），不足部分取默认值");
            defaults_.push_back(fullName(key));
            for (int k = i; k < expected; ++k)
            {
                out[k] = fallback[k];
            }
        }
        return i;
    }

    const std::vector<std::string>& warnings() const { return warnings_; }

private:
    std::string fullName(const char* key) const
    {
        return group_.empty() ? std::string(key) : group_ + "." + key;
    }

    bool missing(const cv::FileNode& node, const char* key, bool warnOnType)
    {
        if (node.empty() || node.isNone())
        {
            defaults_.push_back(fullName(key));
            return true;
        }
        if (warnOnType && (node.isMap() || node.isSeq()))
        {
            warnings_.push_back(fullName(key) + "：类型不符（期望标量），已取默认值");
            defaults_.push_back(fullName(key));
            return true;
        }
        return false;
    }

    std::string                group_;
    std::vector<std::string>&  defaults_;   // 命中默认值的字段全名
    std::vector<std::string>&  warnings_;   // 可疑但可继续
    std::vector<std::string>&  errors_;     // 越界（调用方据此判启动失败）
};

/// 打开一个 yaml。失败时把原因写入 errors 并返回 false。
///
/// ⚠ 文件缺失与解析失败在 ENG-10 §5.3 里同属"文件缺失 → 启动失败"
///   一类（都无法用默认值兜住），故合并处理。
///
/// ⚠ 实测过的第二个陷阱：`cv::FileStorage` 要求 **`%YAML:1.0` 必须是
///   文件的第一行**，指令之前不能有任何注释或空行 —— 否则不报"格式错误"
///   而是抛出 `Input file is invalid`，且**不带行号**，现场改配置的人
///   极难自行定位。故此处在该异常上追加一条明确的提示。
bool openYaml(const std::string& path, cv::FileStorage& fs, std::string& error)
{
    try
    {
        fs.open(path, cv::FileStorage::READ);
    }
    catch (const cv::Exception& e)
    {
        error = "解析失败：" + path +
                "（若为 \"Input file is invalid\"：请检查 %YAML:1.0 是否在"
                "文件首行——该指令之前不能有任何注释或空行）：" + e.what();
        return false;
    }
    if (!fs.isOpened())
    {
        error = "文件缺失或无法打开：" + path;
        return false;
    }
    return true;
}

std::string joinDir(const std::string& dir, const char* file)
{
    if (dir.empty())
    {
        return std::string(file);
    }
    if (dir.back() == '/')
    {
        return dir + file;
    }
    return dir + "/" + file;
}

bool insideRange(double v, double lo, double hi)
{
    return v >= lo && v <= hi;
}

bool strictlyIncreasing(const double* v, int n)
{
    for (int i = 1; i < n; ++i)
    {
        if (!(v[i] > v[i - 1]))
        {
            return false;
        }
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------

void ConfigManager::reset()
{
    system_     = data::SystemConfig{};
    opticalRig_ = data::OpticalRigConfig{};
    turntable_  = data::TurntableConfig{};
    trigger_    = data::TriggerConfig{};
    measurement_ = data::MeasurementConfig{};
    validation_  = data::ValidationConfig{};
    for (data::CameraConfig& c : cameras_)
    {
        c = data::CameraConfig{};
    }
    channels_.clear();
    calibrationSynthetic_ = false;
    configFiles_.clear();
    defaultedFields_.clear();
    warnings_.clear();
    errors_.clear();
    loaded_ = false;
}

const data::CameraConfig& ConfigManager::camera(data::CameraRole role) const
{
    for (const data::CameraConfig& c : cameras_)
    {
        if (c.role == role)
        {
            return c;
        }
    }
    static const data::CameraConfig kFallback;
    return kFallback;
}

std::string ConfigManager::summary() const
{
    std::ostringstream os;
    os << configDir_ << "：" << configFiles_.size() << " 文件 / "
       << channels_.size() << " 相机 / " << defaultedFields_.size()
       << " 项取默认值 / " << warnings_.size() << " 项告警";
    return os.str();
}

bool ConfigManager::load(const std::string& configDir)
{
    reset();
    configDir_ = configDir.empty() ? std::string("config") : configDir;

    const std::string pSystem      = joinDir(configDir_, kFileSystem);
    const std::string pCamera      = joinDir(configDir_, kFileCamera);
    const std::string pOpticalRig  = joinDir(configDir_, kFileOpticalRig);
    const std::string pTurntable   = joinDir(configDir_, kFileTurntable);
    const std::string pTrigger     = joinDir(configDir_, kFileTrigger);
    const std::string pMeasurement = joinDir(configDir_, kFileMeasurement);
    const std::string pValidation  = joinDir(configDir_, kFileValidation);

    configFiles_ = {pSystem, pCamera, pOpticalRig, pTurntable,
                    pTrigger, pMeasurement, pValidation};

    // =====================================================================
    //  1 system.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pSystem, fs, err))
        {
            errors_.push_back(err);
            return false;   // 系统级配置缺失：连日志目录都不知道，无可继续
        }

        const cv::FileNode g = fs["system"];
        FieldReader r("system", defaultedFields_, warnings_, errors_);

        // 相对路径相对于进程当前目录（现场用绝对路径更稳妥，见 yaml 内注释）。
        system_.logDir    = r.text(g["log_dir"], "log_dir", "logs");
        system_.outputDir = r.text(g["output_dir"], "output_dir", "output");
        system_.modelDir  = r.text(g["model_dir"], "model_dir", "models");
        system_.logLevel  = r.integer(g["log_level"], "log_level", 0);

        if (system_.logDir.empty())    { errors_.push_back("system.log_dir 为空"); }
        if (system_.outputDir.empty()) { errors_.push_back("system.output_dir 为空"); }
        if (system_.modelDir.empty())  { errors_.push_back("system.model_dir 为空"); }
        if (!isValidLogLevel(system_.logLevel))
        {
            errors_.push_back("system.log_level 越界：" +
                              std::to_string(system_.logLevel) +
                              "（允许 0=DEBUG 1=INFO 2=WARN 3=ERROR 4=OFF）");
        }
    }

    // =====================================================================
    //  2 camera.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pCamera, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode list = fs["cameras"];
        if (!list.isSeq() || list.size() != 3)
        {
            errors_.push_back("camera.cameras 必须是恰好 3 个条目的序列（实际 " +
                              std::to_string(static_cast<int>(list.size())) + "）");
            return false;
        }

        int index = 0;
        for (auto it = list.begin(); it != list.end(); ++it, ++index)
        {
            const cv::FileNode n = *it;
            FieldReader r("camera.cameras[" + std::to_string(index) + "]",
                          defaultedFields_, warnings_, errors_);

            data::CameraConfig c;
            c.cameraId = r.text(n["id"], "id", "");
            const std::string roleText = r.text(n["role"], "role", "");
            c.width  = r.integer(n["width"], "width", 0);
            c.height = r.integer(n["height"], "height", 0);
            c.exposureTime = r.real(n["exposure_time"], "exposure_time", 0.0);
            c.gain         = r.real(n["gain"], "gain", 0.0);

            const std::string ctx = "camera.cameras[" + std::to_string(index) + "]";

            // ---- backend（011-A1 新增）-------------------------------------
            // ⚠ **缺该键 = 配置错误、启动失败**，不做隐式默认（ENG-09 V2.3 §6.1）。
            // 理由：若按"检测到 SDK 就用真实后端"来默认，则现场 SDK 装好的
            // 那一刻，虚拟配置会**静默变成**真实采集，而操作者以为在跑仿真；
            // 反之 SDK 缺失时又静默退化为虚拟，使"没有真图"这件事只在测量
            // 结果上体现。显式声明把这两种静默切换都消除掉。
            // 故此处用 errors_（⇒ 启动失败）而**不是** defaults_（⇒ 取默认值）。
            {
                const cv::FileNode nb = n["backend"];
                if (nb.empty() || nb.isNone())
                {
                    errors_.push_back(
                        ctx + ".backend 缺失：必须显式声明 \"virtual\" 或 "
                        "\"imv\"（不设默认值——隐式默认会让虚拟配置在 SDK "
                        "装好后静默变成真实采集）");
                }
                else if (!nb.isString())
                {
                    errors_.push_back(ctx + ".backend 应为字符串 \"virtual\" 或 "
                                      "\"imv\"");
                }
                else
                {
                    c.backend = nb.string();
                    if (c.backend != "virtual" && c.backend != "imv")
                    {
                        errors_.push_back(ctx + ".backend 取值非法：" + c.backend +
                                          "（允许 \"virtual\" / \"imv\"）");
                    }
                }
            }

            // ---- serial（011-A1 新增）---------------------------------------
            // 设备**序列号**（期望值），来自设备标签。空 = 未绑定。
            // ⚠ 不在缺键时登记 defaults_：该键对 `backend: virtual` 本来就是
            // 无意义的（虚拟后端没有设备身份），把它记成"命中默认值"会让
            // 每次虚拟启动都多一条噪声提示，而提示该指向真问题时才有价值。
            // ⚠ 但**类型错了要报错**：`serial: 12345` 会被读成数字而静默丢弃。
            {
                const cv::FileNode ns = n["serial"];
                if (!ns.empty() && !ns.isNone())
                {
                    if (!ns.isString())
                    {
                        errors_.push_back(ctx + ".serial 应为字符串（设备标签上的"
                                          "序列号；不要写成数字，前导零会丢失）");
                    }
                    else
                    {
                        c.serialNumber = ns.string();
                    }
                }
            }

            // ---- trigger_mode（011-A1 改：bool → 三值字符串）----------------
            // ⚠ 旧版是 `1 = 硬触发，0 = 软触发`（`bool`）。本版改为
            // `software | hardware | free_run`，**旧数字值显式拒绝**并给迁移
            // 提示：让 `1` 静默变成"某一种模式"会把一次**配置未迁移**伪装成
            // 配置正确 —— 而它正好落在一个本项目最警惕的形态上（设备按一种
            // 模式跑、配置说另一种）。三值的必要性见 CameraTriggerMode.h。
            {
                const cv::FileNode nt = n["trigger_mode"];
                if (nt.empty() || nt.isNone())
                {
                    errors_.push_back(
                        ctx + ".trigger_mode 缺失：必须显式声明 software / "
                        "hardware / free_run（旧版数字 0/1 已不再接受）");
                }
                else if (!nt.isString())
                {
                    errors_.push_back(
                        ctx + ".trigger_mode 是数字（读到 " +
                        std::to_string(static_cast<int>(nt.real())) +
                        "）：本版已改为三值字符串。旧值 1 = 硬触发、0 = 软触发，"
                        "请改写为 hardware / software（旧 1 的另一半可能本来是"
                        "自由运行，故不自动映射）");
                }
                else
                {
                    const std::string tm = nt.string();
                    if (tm == "software")
                    {
                        c.triggerMode = data::CameraTriggerMode::Software;
                    }
                    else if (tm == "hardware")
                    {
                        c.triggerMode = data::CameraTriggerMode::Hardware;
                    }
                    else if (tm == "free_run")
                    {
                        c.triggerMode = data::CameraTriggerMode::FreeRun;
                    }
                    else
                    {
                        errors_.push_back(
                            ctx + ".trigger_mode 取值非法：" + tm +
                            "（允许 software / hardware / free_run）");
                    }
                }
            }

            if (!parseRole(roleText, c.role))
            {
                errors_.push_back(ctx + ".role 非法：" + roleText +
                                  "（允许 CAM25 / CAM50 / CAM100）");
            }
            if (c.cameraId.empty())
            {
                errors_.push_back(ctx + ".id 为空");
            }
            // 真实后端必须绑定序列号：三台相机是同型号，不绑定就只能
            // "取第 0 个"，而三路随机互换之后**没有任何错误**，只表现为
            // 角度系统性偏差。在此处拦下＝在**碰到硬件之前**失败。
            // ⚠ 后端自身的打开流程里还有一道同样的检查（`openDevice()`）——
            // 那一道不是重复，是"最后一道防线"：配置可以被绕过（测试直接
            // 构造 CameraConfig），而**打开设备**这一步绕不过去。
            if (c.backend == "imv" && c.serialNumber.empty())
            {
                errors_.push_back(
                    ctx + "：backend = \"imv\" 但未绑定 serial —— 三台相机同型号，"
                    "不绑定序列号就只能按序号打开，会让三路随机互换且不报错。"
                    "请填写该相机标签上的序列号。");
            }
            if (c.width <= 0 || c.height <= 0)
            {
                errors_.push_back(ctx + "：width/height 必须为正（实际 " +
                                  std::to_string(c.width) + "x" +
                                  std::to_string(c.height) + "）");
            }
            // 曝光时间单位是 s（ENG-09 §6.1）：>0 且不超过一帧的合理上限。
            // 上界 1 s 只为拦住"把毫秒当秒写"（5 ms 写成 5.0）。
            if (!(c.exposureTime > 0.0) || c.exposureTime > 1.0)
            {
                errors_.push_back(ctx + ".exposure_time 越界：" +
                                  std::to_string(c.exposureTime) +
                                  "（单位 s，允许 (0, 1]）");
            }
            if (c.gain < 0.0)
            {
                errors_.push_back(ctx + ".gain 为负：" + std::to_string(c.gain));
            }
            // 重复的 role / id 会让"三相机"体系退化为"两台对一台"而无人察觉
            for (int k = 0; k < index; ++k)
            {
                if (cameras_[k].role == c.role && !roleText.empty())
                {
                    errors_.push_back(ctx + ".role 与前面的条目重复：" + roleText);
                }
                if (!c.cameraId.empty() && cameras_[k].cameraId == c.cameraId)
                {
                    errors_.push_back(ctx + ".id 重复：" + c.cameraId);
                }
            }

            cameras_[index] = c;

            // ---- CameraChannel（供 OpticalRig::initialize）----
            data::CameraChannel ch;
            ch.cameraId = c.cameraId;
            ch.role     = c.role;
            ch.enabled  = true;
            const double focalM = r.real(n["focal_length"], "focal_length", 0.0);
            ch.focalLength      = focalM;

            if (!insideRange(focalM, kFocalMinM, kFocalMaxM))
            {
                errors_.push_back(
                    ctx + ".focal_length 越界：" + std::to_string(focalM) +
                    "（单位 **米**，25/50/100 mm 应写作 0.025 / 0.05 / 0.1）");
            }

            channels_.push_back(ch);
        }
    }

    // =====================================================================
    //  3 optical_rig.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pOpticalRig, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode g = fs["optical_rig"];
        FieldReader r("optical_rig", defaultedFields_, warnings_, errors_);

        opticalRig_.calibrationDir = r.text(g["calibration_dir"], "calibration_dir", "");
        opticalRig_.calibrationId  = r.text(g["calibration_id"], "calibration_id", "");
        const std::string mode = r.text(g["calibration_mode"], "calibration_mode", "file");

        if (mode == "synthetic")
        {
            calibrationSynthetic_ = true;
            warnings_.push_back(
                "optical_rig.calibration_mode = \"synthetic\"：标定文件缺失时"
                "改用合成内参，**结果不具测量意义**（仅用于 M1 合成闭环）");
        }
        else if (mode != "file")
        {
            errors_.push_back("optical_rig.calibration_mode 非法：" + mode +
                              "（允许 file / synthetic）");
        }

        if (opticalRig_.calibrationDir.empty())
        {
            errors_.push_back("optical_rig.calibration_dir 为空");
        }
        if (opticalRig_.calibrationId.empty())
        {
            errors_.push_back("optical_rig.calibration_id 为空");
        }
        else if (opticalRig_.calibrationId == "latest")
        {
            // OpticalRigConfig.h 冻结："不得使用 latest" —— 结果包必须可追溯到
            // 一份确定的标定，而 latest 会随目录内容变化。
            errors_.push_back("optical_rig.calibration_id 不得为 \"latest\""
                              "（OpticalRigConfig.h：结果包须可追溯）");
        }
    }

    // =====================================================================
    //  4 turntable.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pTurntable, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode g = fs["turntable"];
        FieldReader r("turntable", defaultedFields_, warnings_, errors_);

        turntable_.protocol = r.text(g["protocol"], "protocol", "");
        turntable_.azimuthMin   = r.real(g["azimuth_min"], "azimuth_min", 0.0);
        turntable_.azimuthMax   = r.real(g["azimuth_max"], "azimuth_max", 0.0);
        turntable_.elevationMin = r.real(g["elevation_min"], "elevation_min", 0.0);
        turntable_.elevationMax = r.real(g["elevation_max"], "elevation_max", 0.0);
        turntable_.coarseSpeed  = r.real(g["coarse_speed"], "coarse_speed", 0.0);
        turntable_.fineSpeed    = r.real(g["fine_speed"], "fine_speed", 0.0);
        turntable_.centerThreshold =
            r.real(g["center_threshold"], "center_threshold", 50.0);

        // 协议白名单（TurntableConfig.h 注释冻结的取值集合）。
        // 用白名单而不是"非空即可"：字符串协议名的拼写错误
        // （"sdk " / "SDK"）在别处只会表现为"转台没反应"。
        static const char* const kProtocols[] = {"sdk", "pekod", "rs485", "network"};
        if (std::find_if(std::begin(kProtocols), std::end(kProtocols),
                         [&](const char* p) { return turntable_.protocol == p; }) ==
            std::end(kProtocols))
        {
            errors_.push_back("turntable.protocol 非法：" + turntable_.protocol +
                              "（允许 sdk / pekod / rs485 / network）");
        }
        if (!(turntable_.azimuthMin < turntable_.azimuthMax))
        {
            errors_.push_back("turntable：azimuth_min 必须小于 azimuth_max");
        }
        if (!(turntable_.elevationMin < turntable_.elevationMax))
        {
            errors_.push_back("turntable：elevation_min 必须小于 elevation_max");
        }
        // 机械行程（SYS-10 §3.2，冻结）：水平 360°、俯仰 ±60°
        if (turntable_.azimuthMin < -kAzimuthLimitDeg ||
            turntable_.azimuthMax > kAzimuthLimitDeg)
        {
            errors_.push_back("turntable：方位范围超出机械行程 ±180°（SYS-10 §3.2）");
        }
        if (turntable_.elevationMin < -kElevationLimitDeg ||
            turntable_.elevationMax > kElevationLimitDeg)
        {
            errors_.push_back("turntable：俯仰范围超出机械行程 ±60°（SYS-10 §3.2）");
        }
        // 速度（SYS-10 §3.3，冻结）：两轴 1.2 ~ 60 / 30 °/s。
        // 共用上限取俯仰的 30 °/s —— coarseSpeed 是单值，必须两轴都可行。
        if (!insideRange(turntable_.coarseSpeed, kAxisSpeedMinDegS, kAxisSpeedMaxDegS))
        {
            errors_.push_back(
                "turntable.coarse_speed 越界：" + std::to_string(turntable_.coarseSpeed) +
                "（SYS-10 §3.3：两轴 1.2~60/30 °/s，共用上限取 30）");
        }
        if (!(turntable_.fineSpeed > 0.0) ||
            turntable_.fineSpeed > turntable_.coarseSpeed)
        {
            errors_.push_back("turntable.fine_speed 必须 ∈ (0, coarse_speed]（实际 " +
                              std::to_string(turntable_.fineSpeed) + "）");
        }
        if (!(turntable_.centerThreshold > 0.0))
        {
            // 阈值 <=0 会让对准判据恒真（ENG-09 §6.3）
            errors_.push_back("turntable.center_threshold 必须为正");
        }
    }

    // =====================================================================
    //  5 trigger.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pTrigger, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode g = fs["trigger"];
        FieldReader r("trigger", defaultedFields_, warnings_, errors_);

        trigger_.source = r.text(g["source"], "source", "");
        trigger_.periodMs = r.real(g["period_ms"], "period_ms", 0.0);
        trigger_.syncToleranceNs =
            r.nanoseconds(g["sync_tolerance_ns"], "sync_tolerance_ns", 0);

        static const char* const kSources[] = {"hardware", "virtual"};
        if (std::find_if(std::begin(kSources), std::end(kSources),
                         [&](const char* s) { return trigger_.source == s; }) ==
            std::end(kSources))
        {
            errors_.push_back("trigger.source 非法：" + trigger_.source +
                              "（允许 hardware / virtual）");
        }
        if (!(trigger_.periodMs > 0.0))
        {
            errors_.push_back("trigger.period_ms 必须为正（周期为 0 即无触发）");
        }
        if (trigger_.syncToleranceNs == 0)
        {
            // 容差为 0 时"三帧时间差 ≤ 0"几乎必然不成立，同步校验恒失败。
            // SYS-04 §412 把该值的来源列为 OPEN-SW-04（待硬件电气参数），
            // 但为 0 一定是错的，故判越界。
            errors_.push_back("trigger.sync_tolerance_ns 必须为正"
                              "（SYS-04 §412 OPEN-SW-04 未定值，但 0 恒不成立）");
        }
    }

    // =====================================================================
    //  6 measurement.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pMeasurement, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode g = fs["measurement"];
        FieldReader r("measurement", defaultedFields_, warnings_, errors_);

        measurement_.w1 = r.real(g["w1"], "w1", 0.0);
        measurement_.w2 = r.real(g["w2"], "w2", 0.0);
        measurement_.w3 = r.real(g["w3"], "w3", 0.0);
        measurement_.w4 = r.real(g["w4"], "w4", 0.0);
        for (double w : {measurement_.w1, measurement_.w2,
                         measurement_.w3, measurement_.w4})
        {
            if (w < 0.0)
            {
                errors_.push_back("measurement：权重不得为负");
                break;
            }
        }
        if (measurement_.w1 == 0.0 && measurement_.w2 == 0.0 &&
            measurement_.w3 == 0.0 && measurement_.w4 == 0.0)
        {
            // ENG-10 §5.3 举的例子：w1~w4 全为 0 → 启动失败。
            // 全 0 会让评分恒为 0、选择退化为"永远选第一个候选"。
            errors_.push_back("measurement：w1~w4 全为 0（ENG-10 §5.3 判为越界）");
        }

        measurement_.minSharpness      = r.real(g["min_sharpness"], "min_sharpness", 0.0);
        measurement_.minTargetPixelSize =
            r.real(g["min_target_pixel_size"], "min_target_pixel_size", 0.0);
        measurement_.minFeatureCount =
            r.integer(g["min_feature_count"], "min_feature_count", 0);
        measurement_.minMatchRatio = r.real(g["min_match_ratio"], "min_match_ratio", 0.0);
        for (double v : {measurement_.minSharpness, measurement_.minTargetPixelSize,
                         measurement_.minMatchRatio})
        {
            if (v < 0.0)
            {
                errors_.push_back("measurement：门槛值不得为负");
                break;
            }
        }
        if (measurement_.minFeatureCount < 0)
        {
            errors_.push_back("measurement.min_feature_count 不得为负");
        }

        measurement_.captureFrameCount =
            r.integer(g["capture_frame_count"], "capture_frame_count", 5);
        if (!insideRange(measurement_.captureFrameCount, 5, 10))
        {
            // ENG-09 §6.5 冻结的约定范围 5~10（随机误差按 1/√N 平均）
            errors_.push_back("measurement.capture_frame_count 越界：" +
                              std::to_string(measurement_.captureFrameCount) +
                              "（ENG-09 §6.5 约定 5~10）");
        }

        measurement_.taskTimeoutNs =
            r.nanoseconds(g["task_timeout_ns"], "task_timeout_ns",
                          measurement_.taskTimeoutNs);
        measurement_.searchTimeoutNs =
            r.nanoseconds(g["search_timeout_ns"], "search_timeout_ns",
                          measurement_.searchTimeoutNs);
        measurement_.targetFoundTimeoutNs =
            r.nanoseconds(g["target_found_timeout_ns"], "target_found_timeout_ns",
                          measurement_.targetFoundTimeoutNs);
        measurement_.alignTimeoutNs =
            r.nanoseconds(g["align_timeout_ns"], "align_timeout_ns",
                          measurement_.alignTimeoutNs);
        measurement_.stabilizeTimeoutNs =
            r.nanoseconds(g["stabilize_timeout_ns"], "stabilize_timeout_ns",
                          measurement_.stabilizeTimeoutNs);
        measurement_.selectTimeoutNs =
            r.nanoseconds(g["select_timeout_ns"], "select_timeout_ns",
                          measurement_.selectTimeoutNs);
        measurement_.captureTimeoutNs =
            r.nanoseconds(g["capture_timeout_ns"], "capture_timeout_ns",
                          measurement_.captureTimeoutNs);

        // ---- 取帧预算（ENG-09 V2.3 §6.5；SYS-04 V2.4 §6.1）----
        //  ⚠ 这两个量是**上限**而不是承诺值：`capture()` 时它们与实际等待处
        //    的"距状态期限的剩余"三者**取最小**（MultiCameraManager::capture）。
        //    故调大它们不会延长任何一次等待。
        measurement_.grabTimeoutMs =
            r.integer(g["grab_timeout_ms"], "grab_timeout_ms",
                      measurement_.grabTimeoutMs);
        if (measurement_.grabTimeoutMs <= 0)
        {
            // 0 **不是**"不等待"的意思：`IMV_GetFrame` 对 timeoutMS = 0 的
            // 语义在 SDK 中未文档化（ENG-09 V2.3 §2.5 核验表），项目**不定义**
            // 它，后端实现会明确拒绝 0（InvalidArgument）。负数同样非法。
            errors_.push_back(
                "measurement.grab_timeout_ms 必须为正（0 的语义 SDK 未文档化、"
                "项目不定义，后端会拒绝）");
        }

        measurement_.grabGroupBudgetNs =
            r.nanoseconds(g["grab_group_budget_ns"], "grab_group_budget_ns",
                          measurement_.grabGroupBudgetNs);
        if (measurement_.grabGroupBudgetNs == 0)
        {
            // 组预算为 0 ⇒ 每路都判"预算耗尽"，一次 SDK 调用都不会发生，
            // 表现为"三路全部取帧失败"而实际是配置把预算配没了。
            errors_.push_back("measurement.grab_group_budget_ns 为 0");
        }

        // ---- 取帧预算与状态时限的**余量事实**（只警告，不改冻结值）----
        //  capture_frame_count 帧 × 3 路 × grab_timeout_ms 若已占满
        //  capture_timeout_ns，则该状态**没有余量**，而复制、格式转换、评分
        //  都不受取帧等待参数约束 ⇒ CAPTURE 在真实相机上**可能被时限截断**。
        //  ⚠ 这里**不判启动失败、也不自行放宽冻结值**：时限体系的取值余量属
        //    《待裁决问题汇总》Q-D2，本批只把事实测出来、让它可见。
        if (measurement_.grabTimeoutMs > 0 && measurement_.captureTimeoutNs > 0 &&
            measurement_.captureFrameCount > 0)
        {
            const uint64_t needTotalMs =
                static_cast<uint64_t>(measurement_.grabTimeoutMs) * 3ULL *
                static_cast<uint64_t>(measurement_.captureFrameCount);
            const uint64_t captureTimeoutMs = measurement_.captureTimeoutNs / 1000000ULL;
            if (needTotalMs >= captureTimeoutMs)
            {
                warnings_.push_back(
                    "measurement：取帧预算无余量 —— capture_frame_count(" +
                    std::to_string(measurement_.captureFrameCount) + ") × 3 路 × "
                    "grab_timeout_ms(" +
                    std::to_string(measurement_.grabTimeoutMs) + ") = " +
                    std::to_string(needTotalMs) + " ms ≥ capture_timeout_ns(" +
                    std::to_string(captureTimeoutMs) +
                    " ms)，且未计入复制/格式转换/评分 ⇒ CAPTURE 在真实相机上"
                    "可能被时限截断（取值余量见《待裁决问题汇总》Q-D2）");
            }
        }

        measurement_.solveTimeoutNs =
            r.nanoseconds(g["solve_timeout_ns"], "solve_timeout_ns",
                          measurement_.solveTimeoutNs);
        measurement_.validateTimeoutNs =
            r.nanoseconds(g["validate_timeout_ns"], "validate_timeout_ns",
                          measurement_.validateTimeoutNs);
        measurement_.saveTimeoutNs =
            r.nanoseconds(g["save_timeout_ns"], "save_timeout_ns",
                          measurement_.saveTimeoutNs);

        if (measurement_.taskTimeoutNs == 0)
        {
            // ENG-10 §5.3 举的例子：taskTimeoutNs = 0 → 启动失败
            errors_.push_back("measurement.task_timeout_ns 为 0（ENG-10 §5.3 判为越界）");
        }

        // 三级时限的包含关系（SYS-08 §7.1〔引用无效·依据待裁决·见 Q-D2〕，冻结）：
        //   T_task > T_state > T_device
        // 某个状态超时 ≥ T_task 时该状态的超时永远不会触发 ——
        // 任务会先撞 T_task 并以 9001 结束，于是该状态专属错误码
        // （如 2003 对准重试耗尽）永远无法产生，故障定位信息丢失。
        struct TimeoutItem
        {
            const char* name;
            uint64_t    value;
        };
        const TimeoutItem stateTimeouts[] = {
            {"search_timeout_ns", measurement_.searchTimeoutNs},
            {"target_found_timeout_ns", measurement_.targetFoundTimeoutNs},
            {"align_timeout_ns", measurement_.alignTimeoutNs},
            {"stabilize_timeout_ns", measurement_.stabilizeTimeoutNs},
            {"select_timeout_ns", measurement_.selectTimeoutNs},
            {"capture_timeout_ns", measurement_.captureTimeoutNs},
            {"solve_timeout_ns", measurement_.solveTimeoutNs},
            {"validate_timeout_ns", measurement_.validateTimeoutNs},
            {"save_timeout_ns", measurement_.saveTimeoutNs},
        };
        for (const TimeoutItem& item : stateTimeouts)
        {
            if (item.value == 0)
            {
                errors_.push_back(std::string("measurement.") + item.name + " 为 0");
            }
            else if (item.value >= measurement_.taskTimeoutNs)
            {
                errors_.push_back(std::string("measurement.") + item.name +
                                  " 不小于 task_timeout_ns（违反 SYS-08 §7.1 的"
                                  " T_task > T_state，该状态超时将永不触发）");
            }
        }

        measurement_.maxAlignAttempts =
            r.integer(g["max_align_attempts"], "max_align_attempts", 8);
        measurement_.maxSolveAttempts =
            r.integer(g["max_solve_attempts"], "max_solve_attempts", 2);
        measurement_.maxValidateAttempts =
            r.integer(g["max_validate_attempts"], "max_validate_attempts", 2);
        measurement_.maxSelectAttempts =
            r.integer(g["max_select_attempts"], "max_select_attempts", 3);
        measurement_.maxCaptureAttempts =
            r.integer(g["max_capture_attempts"], "max_capture_attempts", 3);
        measurement_.maxSaveAttempts =
            r.integer(g["max_save_attempts"], "max_save_attempts", 3);
        measurement_.maxRollbackTotal =
            r.integer(g["max_rollback_total"], "max_rollback_total", 4);
        measurement_.maxRollbackPerEdge =
            r.integer(g["max_rollback_per_edge"], "max_rollback_per_edge", 2);

        const int attemptCounts[] = {
            measurement_.maxAlignAttempts,  measurement_.maxSolveAttempts,
            measurement_.maxValidateAttempts, measurement_.maxSelectAttempts,
            measurement_.maxCaptureAttempts, measurement_.maxSaveAttempts};
        for (int v : attemptCounts)
        {
            if (v < 1)
            {
                // 尝试次数为 0 表示该状态一次也不执行 —— 流程直接卡死，
                // 而错误码会报成"重试耗尽"这种与实际原因无关的说法。
                errors_.push_back("measurement：各状态的最大尝试次数必须 ≥ 1");
                break;
            }
        }
        if (measurement_.maxRollbackTotal < 0 || measurement_.maxRollbackPerEdge < 1)
        {
            errors_.push_back("measurement：max_rollback_total ≥ 0 且 "
                              "max_rollback_per_edge ≥ 1（SYS-08 §7.4）");
        }

        measurement_.sigmaA = r.real(g["sigma_a"], "sigma_a", 0.0);
        measurement_.sigmaB = r.real(g["sigma_b"], "sigma_b", 0.0);
        measurement_.sigmaC = r.real(g["sigma_c"], "sigma_c", 0.0);
        measurement_.sigmaPxFallback =
            r.real(g["sigma_px_fallback"], "sigma_px_fallback", 0.5);
        if (!(measurement_.sigmaPxFallback > 0.0))
        {
            // 回退值 ≤0 会让未标定时的 E 恒为 0，评分静默退化为 Q/F/M
            errors_.push_back("measurement.sigma_px_fallback 必须为正（ENG-09 §6.5）");
        }
        if (measurement_.sigmaA < 0.0 || measurement_.sigmaB < 0.0 ||
            measurement_.sigmaC < 0.0)
        {
            errors_.push_back("measurement：sigma_a/b/c 不得为负");
        }
        if (measurement_.sigmaA == 0.0 && measurement_.sigmaB == 0.0 &&
            measurement_.sigmaC == 0.0)
        {
            // 不是错误（未标定的合法状态），但必须留下痕迹：
            // 此时 E 恒为 0、评分只看 Q/F/M（MeasurementConfig.h 的说明）。
            warnings_.push_back(
                "measurement.sigma_a/b/c 全为 0：E 项未生效（未标定），"
                "评分退化为 Q/F/M —— 结果包中的 predictedErrorCalibrated 将为 false");
        }

        measurement_.nRef = r.real(g["n_ref"], "n_ref", 100.0);
        if (!(measurement_.nRef > 0.0))
        {
            errors_.push_back("measurement.n_ref 必须为正（F = min(1, nDetect/nRef)）");
        }

        measurement_.matchStatsMinSamples =
            r.integer(g["match_stats_min_samples"], "match_stats_min_samples", 10);
        measurement_.matchStatsColdStartPrior =
            r.real(g["match_stats_cold_start_prior"], "match_stats_cold_start_prior",
                   0.5);
        if (measurement_.matchStatsMinSamples < 1)
        {
            // ENG-10 §5.3 举的例子：N_min = 0 → 启动失败（收缩估计器退化）
            errors_.push_back("measurement.match_stats_min_samples 必须 ≥ 1"
                              "（ENG-10 §5.3 判为越界）");
        }
        if (!insideRange(measurement_.matchStatsColdStartPrior, 0.0, 1.0))
        {
            errors_.push_back("measurement.match_stats_cold_start_prior 必须 ∈ [0,1]");
        }

        const double kDefaultDistanceEdges[3] = {80.0, 150.0, 220.0};
        const double kDefaultIllumEdges[2]    = {0.0, 0.0};
        r.reals(g["distance_band_edges"], "distance_band_edges",
                measurement_.distanceBandEdges, 3, kDefaultDistanceEdges);
        r.reals(g["illum_band_edges"], "illum_band_edges",
                measurement_.illumBandEdges, 2, kDefaultIllumEdges);

        if (!strictlyIncreasing(measurement_.distanceBandEdges, 3))
        {
            // 边界必须严格递增：相等会让某个带为空，倒序会让 bandOf 的
            // 比较链（< e[0] / < e[1] / < e[2]）产出无意义的索引。
            errors_.push_back("measurement.distance_band_edges 必须严格递增"
                              "（ENG-10 §4.2 的 4 个距离带）");
        }
        if (measurement_.illumBandEdges[0] < 0.0 ||
            measurement_.illumBandEdges[0] > measurement_.illumBandEdges[1])
        {
            errors_.push_back("measurement.illum_band_edges 必须非负且不减");
        }
        if (measurement_.illumBandEdges[0] == 0.0 &&
            measurement_.illumBandEdges[1] == 0.0)
        {
            // 与 sigma 全 0 同理：不是错误，但必须可见。
            // ENG-09 §6.5 要求这两条边界与曝光设置一同标定。
            warnings_.push_back(
                "measurement.illum_band_edges 均为 0：光照分带未标定，"
                "所有光照条件落入同一桶，M_hist 的光照维度暂不可用");
        }
    }

    // =====================================================================
    //  7 validation.yaml
    // =====================================================================
    {
        cv::FileStorage fs;
        std::string     err;
        if (!openYaml(pValidation, fs, err))
        {
            errors_.push_back(err);
            return false;
        }

        const cv::FileNode g = fs["validation"];
        FieldReader r("validation", defaultedFields_, warnings_, errors_);

        validation_.maxReprojectionError =
            r.real(g["max_reprojection_error"], "max_reprojection_error", 0.0);
        validation_.minInlierRatio =
            r.real(g["min_inlier_ratio"], "min_inlier_ratio", 0.0);
        validation_.minConfidence = r.real(g["min_confidence"], "min_confidence", 0.0);
        validation_.yawMin = r.real(g["yaw_min"], "yaw_min", 0.0);
        validation_.yawMax = r.real(g["yaw_max"], "yaw_max", 0.0);

        if (!(validation_.maxReprojectionError > 0.0))
        {
            // 上限为 0 时判据 `error <= 0` 对任何实际解算都恒假
            errors_.push_back("validation.max_reprojection_error 必须为正（单位 pixel）");
        }
        if (!insideRange(validation_.minInlierRatio, 0.0, 1.0))
        {
            errors_.push_back("validation.min_inlier_ratio 必须 ∈ [0,1]");
        }
        if (!insideRange(validation_.minConfidence, 0.0, 1.0))
        {
            errors_.push_back("validation.min_confidence 必须 ∈ [0,1]");
        }
        if (!(validation_.yawMin < validation_.yawMax))
        {
            errors_.push_back("validation：yaw_min 必须小于 yaw_max");
        }
    }

    if (!errors_.empty())
    {
        return false;
    }

    loaded_ = true;
    return true;
}

}  // namespace infrastructure
}  // namespace aircraft
