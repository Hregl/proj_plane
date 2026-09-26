// ============================================================================
//  src/app/SystemInitializer.cpp
//
//  本文件是 009 的主体：把 ApplicationContext 里的对象按 ENG-02 §15 的顺序
//  造出来、初始化、互相接上。
//
//  ---- 三条贯穿全文件的写法约定 ----
//
//  1. **每一步都返回 bool 并写明失败点**，不抛异常。
//     ENG-10 §5.3 的三档处理（文件缺失→启动失败 / 字段缺失→默认值+记录 /
//     越界→启动失败）全部是返回值语义；一旦某处改用异常，调用链上就会出现
//     "有些失败要 catch、有些要查返回值"的双轨制，而漏 catch 的那条
//     会直接终止进程且不留日志。
//
//  2. **告警必须落到 warnings_ 与日志两处**，不得只在其中一处。
//     只写日志的告警在窗口里看不见（操作者不会去翻日志）；
//     只进界面warnings的告警在事后离线排查时找不到（现场只留日志文件）。
//     静默的告警等于没有告警 —— 这是 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕"降级必须可见"的同一要求。
//
//  3. **失败时的清理分层**：内存靠 unique_ptr，**设备资源靠显式回滚**。
//
//     ⚠ 本约定在 011-A1 被**收窄**（原文为"失败时的清理靠 unique_ptr，
//       不写手工回滚"，理由是"手工回滚的代码几乎不可能被测到"）。
//       该理由对**内存**成立 —— 本类确实不 `new` 任何东西，全部经
//       make_unique 存入 ctx_，`initialize()` 中途返回 false 时由
//       ApplicationContext 的析构统一收拾。
//       但它对**设备资源**不成立：`unique_ptr` 的析构**不会**替你调用
//       `IMV_StopGrabbing` / `IMV_Close` / `IMV_DestroyHandle` ——
//       真实后端持有的句柄与已建立的流是 SDK 侧的状态，
//       不还回去就是"进程退了、相机还开着"。
//       ∴ 启动被判失败时由 `rollbackDevices()` 逐路显式
//       `stop()` → `close()`（逆序），而"这条路测不到"这件事由
//       §4.4 的用例解决（替身记录底层 open/close 次数），
//       不靠"不写它"来回避。
//       内存那一半保持原样：仍然不写手工 free/delete。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include "app/SystemInitializer.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>

#include <unistd.h>

#include "algorithm/pipeline/PosePipeline.h"
#include "data/ErrorInfo.h"
#include "data/MonotonicClock.h"
#include "device/camera/ICameraBackend.h"
#include "device/camera/ImvCameraBackend.h"
#include "device/camera/VirtualCameraBackend.h"
#include "infrastructure/FileUtil.h"

namespace aircraft
{
namespace app
{

namespace
{

/// 目录可写性检查：建目录 + **真的写一个文件**。
///
/// 为什么不能只靠 `access(path, W_OK)`：
///   · root 下 access 恒返回可写（CAP_DAC_OVERRIDE），而实际可能写在
///     只读挂载上 —— 现场一体机常把根文件系统挂成只读，日志目录单独挂载；
///   · 更根本的是"可写"的唯一可信证据就是写成功了一次。
/// 检查会在目标目录留下并立即删除一个临时文件；这一步的代价是微秒级，
/// 换来的是"跑到第一次落盘时才发现写不了"变成"启动时就报出来"。
bool ensureWritableDir(const std::string& path, std::string& error)
{
    if (path.empty())
    {
        error = "路径为空";
        return false;
    }
    if (!infrastructure::fileutil::makeDirectories(path))
    {
        error = "无法创建目录：" + path;
        return false;
    }

    const std::string probe = infrastructure::fileutil::joinPath(path, ".aps_write_test");
    {
        std::ofstream out(probe, std::ios::out | std::ios::trunc);
        if (!out.is_open())
        {
            error = "目录不可写：" + path;
            return false;
        }
        out << "aps\n";
        out.flush();
        if (!out.good())
        {
            error = "目录不可写（写入失败）：" + path;
            out.close();
            ::unlink(probe.c_str());
            return false;
        }
    }
    ::unlink(probe.c_str());
    return true;
}

/// 状态名。与 application/ui/infrastructure 中已有的同名函数逐字一致 ——
/// 这是**第五份**副本（前四份见 infrastructure/recorder/Recorder.cpp 的说明）。
/// app 是全工程唯一同时依赖四个模块的地方，故它可以不含这一份；
/// 但 app 需要的是"给 operator 看的中文状态描述"，与那几个英文枚举名
/// 用途不同，故此处不重复实现英文名，只用 ui::UiText 的中文文本。
std::string noticeText(const std::vector<std::string>& notices)
{
    if (notices.empty())
    {
        return std::string();
    }
    std::string out = notices.front();
    if (notices.size() > 1)
    {
        out += "（另有 " + std::to_string(notices.size() - 1) + " 条，见日志）";
    }
    return out;
}

/// 角色名（"CAM25"…）。形状照抄仓内既有各处（`MeasurementController.cpp:74`、
/// `PreviewManager.cpp:25`、`Recorder.cpp:150`），**不新造公共接口** ——
/// 这些地方各自持有副本是本仓的既有约定（理由见 Recorder.cpp 的说明）。
const char* roleText(data::CameraRole role)
{
    switch (role)
    {
    case data::CameraRole::CAM25:  return "CAM25";
    case data::CameraRole::CAM50:  return "CAM50";
    case data::CameraRole::CAM100: return "CAM100";
    }
    return "CAM??";
}

/// 把一个 OperationResult 压成"状态 + 首因/清理因"一行。
///
/// ⚠ 不复用 `data::opStatusName` **不足以**表达这件事：那句只给状态名，
///    而 §2.2 修正 4 要求清理失败**同时带操作名与返回码**（同为 −119，
///    `IMV_GetFrame` 超时与 `IMV_ReleaseFrame` 超时是完全不同的故障）。
///    故此处组合使用仓内既有的 `data::opStatusName` / `data::sdkCallName`
///    （`data/OpStatus.h:172`／`:203`），**不新写一份枚举名表** ——
///    本仓已有五份状态名副本的教训（见上方 noticeText 的说明）。
///
/// ⚠ 首因与清理因**分列**，不合写成一句：两者同现时的处置不同（§2.2 修正 4），
///    合写会让"主操作失败"与"清理失败"看起来是同一件事。
std::string resultText(const data::OperationResult& r)
{
    std::string out = data::opStatusName(r.status);
    if (r.status == data::OpStatus::Unset)
    {
        // Unset 是**缺陷指示**（漏赋值），不是一种失败；打印时点名，
        // 免得日志里一个孤零零的 "Unset" 被当成"没做"。
        out += "（未赋值——缺陷）";
    }
    if (r.sdkError)
    {
        out += std::string("，失败调用 ") + data::sdkCallName(r.sdkError->call) +
               "（码 " + std::to_string(r.sdkError->code) + "）";
    }
    if (r.cleanupError)
    {
        out += std::string("，清理失败 ") + data::sdkCallName(r.cleanupError->call) +
               "（码 " + std::to_string(r.cleanupError->code) + "）";
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------

SystemInitializer::SystemInitializer(ApplicationContext& ctx)
    : ctx_(ctx)
{
}

SystemInitializer::~SystemInitializer() = default;

// ---------------------------------------------------------------------------

void SystemInitializer::log(const std::string& module, const std::string& message)
{
    if (ctx_.logger && ctx_.logger->ready())
    {
        ctx_.logger->info(module, message);
        return;
    }
    // Logger 未就绪（配置或日志目录本身出了问题）时退化为 stderr：
    // 启动期的信息恰恰是最需要留下的，而此刻还没有文件可写。
    std::fprintf(stderr, "[INFO ] [%s] %s\n", module.c_str(), message.c_str());
}

void SystemInitializer::warn(const std::string& message)
{
    warnings_.push_back(message);
    if (ctx_.logger && ctx_.logger->ready())
    {
        ctx_.logger->warn("app", message);
    }
    else
    {
        std::fprintf(stderr, "[WARN ] [app] %s\n", message.c_str());
    }
}

// ---------------------------------------------------------------------------
//  步骤 1~2：配置与日志
// ---------------------------------------------------------------------------

bool SystemInitializer::loadConfigAndLogger()
{
    ctx_.config = std::make_unique<infrastructure::ConfigManager>();

    if (!ctx_.config->load(ctx_.configDir))
    {
        std::ostringstream os;
        os << "配置加载失败（" << ctx_.config->configDir() << "）：";
        const auto& errs = ctx_.config->errors();
        for (std::size_t i = 0; i < errs.size(); ++i)
        {
            os << (i == 0 ? "" : "；") << errs[i];
        }
        errorText_ = os.str();
        // 9005：启动期的配置错误。它发生在状态机启动之前，不会有
        // result.json，故这个码是它唯一的机读痕迹（见 startupError() 说明）。
        startupError_ = data::ErrorInfo{data::kErrSystemConfig,
                                        errorText_,
                                        data::monotonicNowNs()};
        std::fprintf(stderr, "[ERROR] %s\n", errorText_.c_str());
        return false;
    }

    // 以 ConfigManager 实际使用的目录为准（它可能把相对路径规范化了）。
    ctx_.configDir = ctx_.config->configDir();

    // 字段缺失 → 取默认值并**逐条记录**（ENG-10 §5.3 的第二档）。
    for (const std::string& field : ctx_.config->defaultedFields())
    {
        warn("配置字段缺失，已取默认值：" + field);
    }
    for (const std::string& w : ctx_.config->warnings())
    {
        warn("配置告警：" + w);
    }

    const data::SystemConfig& sys = ctx_.config->system();

    // ---- 日志 ----
    // 顺序：配置 → 日志（ENG-02 §15）。log_dir / log_level 只有读完
    // 配置才知道，故 10.md §八 的"Logger 先于 ConfigManager"无法成立。
    ctx_.logger = std::make_unique<infrastructure::Logger>();
    if (!ctx_.logger->initialize(sys.logDir, sys.logLevel))
    {
        errorText_ = "日志初始化失败：无法创建或写入 log_dir=" + sys.logDir;
        std::fprintf(stderr, "[ERROR] %s\n", errorText_.c_str());
        return false;
    }

    log("app", "=== AircraftPoseSystem V2.1 启动 ===");
    log("app", ctx_.config->summary());

    return true;
}

// ---------------------------------------------------------------------------
//  步骤 3：标定 + 光机
// ---------------------------------------------------------------------------

bool SystemInitializer::buildOptical()
{
    const data::OpticalRigConfig& orc = ctx_.config->opticalRig();

    // calibration_mode 的两种取值在这里分叉（009 引入的装载策略，
    // 不在冻结的 OpticalRigConfig 中，理由见 ConfigManager.h）：
    //   "file"      —— 从 calibrationDir 读标定文件；缺失即启动失败
    //                  （ENG-10 §5.3 的第一档）。
    //   "synthetic" —— 用 loadDefaults(w, h) 生成默认矩阵。这是 M1
    //                  （11.md §六）明列的 mock 之一：M1 的目标是打通
    //                  链路，而非验证标定精度，而真实标定要等相机到货。
    ctx_.calibrationMode = ctx_.config->calibrationSynthetic() ? "synthetic" : "file";

    ctx_.calibration = std::make_unique<optical::CalibrationManager>();

    if (ctx_.calibrationMode == "synthetic")
    {
        const data::CameraConfig& cam = ctx_.config->camera(data::CameraRole::CAM25);
        if (!ctx_.calibration->loadDefaults(cam.width, cam.height))
        {
            errorText_ = "合成标定生成失败（calibration_mode=synthetic，"
                         "图像尺寸取自 CAM25 配置）";
            return false;
        }
        warn("calibration_mode=synthetic：使用**默认矩阵标定**，"
             "本次运行的姿态精度不具参考意义（M1 打通链路专用）");
    }
    else
    {
        if (!ctx_.calibration->load(orc.calibrationDir))
        {
            errorText_ = "标定加载失败（calibration_mode=file）：" +
                         ctx_.calibration->lastErrorText() +
                         "（目录 " + orc.calibrationDir + "）";
            return false;
        }
    }

    for (const std::string& w : ctx_.calibration->lastWarnings())
    {
        warn("标定告警：" + w);
    }

    // ---- 光机刚体 ----
    ctx_.rig = std::make_unique<optical::OpticalRig>();
    if (!ctx_.rig->initialize(ctx_.config->channels()))
    {
        errorText_ = "OpticalRig 初始化失败：通道配置为空或不合法";
        return false;
    }
    if (!ctx_.rig->setCalibration(ctx_.calibration->calibration()))
    {
        errorText_ = "OpticalRig 注入标定失败（标定与通道不匹配）";
        return false;
    }

    log("app", "标定：" + ctx_.calibration->calibrationId() +
                   "（模式 " + ctx_.calibrationMode + "）");

    summary_.calibrationLoaded = ctx_.calibration->loaded();
    summary_.enabledChannels   = 0;
    for (const data::CameraChannel& ch : ctx_.rig->cameras())
    {
        if (ch.enabled)
        {
            ++summary_.enabledChannels;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
//  步骤 4~5：设备（ENG-08 §3 的虚拟/真实切换点）
// ---------------------------------------------------------------------------

std::shared_ptr<device::ICameraBackend> SystemInitializer::makeBackend(
    const data::CameraConfig& config, std::string& error)
{
    const std::string where = "（通道 " + config.cameraId + "）";

    if (config.backend == "virtual")
    {
        return std::make_shared<device::VirtualCameraBackend>(config);
    }

    if (config.backend == "imv")
    {
        // ⚠ 先问"本次构建有没有 SDK"，而不是构造一个注定初始化失败的后端：
        //   两者都以启动失败告终，但**消息完全不同** —— "本次构建未接入 SDK，
        //   请用 -DIMV_SDK_ROOT=… 重新配置"是操作者能立刻执行的动作，
        //   而"初始化失败：NotImplemented"会把人引向设备与线缆。
        //   这正是 011-A0 裁定"两种不可用必须给不同文本"的同一条理由。
        std::shared_ptr<device::IImvApi> api = device::makeRealImvApi();
        if (!api)
        {
            error = "通道 " + config.cameraId +
                    " 配置为真实后端（backend: imv），但**本次构建未接入 SDK**。"
                    "请用 -DIMV_SDK_ROOT=<SDK 根目录> 重新配置后重建；"
                    "若本轮只想跑软件闭环，请把该通道的 backend 改为 \"virtual\""
                    "（本程序**不会**替你静默切换）。";
            return nullptr;
        }
        return std::make_shared<device::ImvCameraBackend>(config, api);
    }

    // 取值合法性已在 ConfigManager 拦过一次；此处仍如实失败，因为
    // CameraConfig 可以被**绕过配置加载**直接构造（测试、将来的其它入口），
    // 而"装配一个 backend 字段为空的后端"没有合理的兜底动作。
    error = "通道 " + config.cameraId + " 的 backend 取值非法：\"" +
            config.backend + "\"（允许 \"virtual\" / \"imv\"）" + where;
    return nullptr;
}

bool SystemInitializer::buildDevices()
{
    // ┌────────────────────────────────────────────────────────────────────┐
    // │ 显式装配点（ENG-08 §3、ENG-10 §5.1；011-A1 改为按配置逐个选择）。  │
    // │                                                                    │
    // │ **不**"检测到 SDK 就切真实"，**不**在真实相机打不开时静默换虚拟。  │
    // │ 每一路由 camera.yaml 的 `backend` 键决定，缺键即配置错误（启动前   │
    // │ 已由 ConfigManager 拦下）。ENG-08 §3 的过渡形态"CAM25 真实 +      │
    // │ CAM50/100 虚拟"因此就是三行各自独立的配置，不需要额外机制。        │
    // └────────────────────────────────────────────────────────────────────┘
    const data::CameraConfig& c25  = ctx_.config->camera(data::CameraRole::CAM25);
    const data::CameraConfig& c50  = ctx_.config->camera(data::CameraRole::CAM50);
    const data::CameraConfig& c100 = ctx_.config->camera(data::CameraRole::CAM100);

    std::string backendError;
    ctx_.backend25 = makeBackend(c25, backendError);
    if (!ctx_.backend25)
    {
        errorText_ = "CAM25 后端装配失败：" + backendError;
        return false;
    }
    ctx_.backend50 = makeBackend(c50, backendError);
    if (!ctx_.backend50)
    {
        errorText_ = "CAM50 后端装配失败：" + backendError;
        return false;
    }
    ctx_.backend100 = makeBackend(c100, backendError);
    if (!ctx_.backend100)
    {
        errorText_ = "CAM100 后端装配失败：" + backendError;
        return false;
    }

    ctx_.cameras = std::make_unique<device::MultiCameraManager>(
        ctx_.backend25, ctx_.backend50, ctx_.backend100);

    // ---- 初始化与启动：**只有这一处**调用 initialize() ----------------------
    //
    // ⚠ 上一版在此处**逐个** `b->initialize()`，紧接着 `initializeAll()` 又
    //   各自初始化一次 —— 同一个后端被初始化两遍。虚拟后端下这看不出问题，
    //    而真实后端下第二遍会重复枚举设备、重复 `IMV_Open`，句柄与打开状态
    //    的归属随之失去唯一责任人（"谁负责关"由"谁先成功"决定，而那是
    //    运行期才知道的事）。故本批起 `initializeAll()` 是**唯一**的初始化
    //    责任方，装配层只负责**造**后端。
    if (!ctx_.cameras->initializeAll())
    {
        errorText_ = "MultiCameraManager::initializeAll 失败（" +
                     ctx_.cameras->lastError().message + "）";
        rollbackDevices();
        return false;
    }

    // ---- 启动失败边界：**显式请求的真实后端打不开 ⇒ 整次启动失败** --------
    //
    // ⚠ 这一条不能用 `initializeAll()` 的返回值代替：它的判据是**合计**
    //    ≥2 路（§7.5 的降级下限）。"CAM25 真实 + CAM50/100 虚拟"这一形态下，
    //    真实那一路打不开时合计恰好是 2 ⇒ 达标 ⇒ 启动会被判成功，而
    //    "真实接入成功"这件事**并不成立** —— 操作者以为在跑真实采集，
    //    实际三路全是虚拟图。故逐路核对**配置为 imv 的那几路**。
    // ⚠ 迭代的是 (配置, 后端) **成对**的槽位，而不是只有配置（2026-09-26 改）：
    //    要如实说出"这一路为什么没就绪"，必须拿得到**该路后端自己**的
    //    说明（`lastErrorText()`）—— 例如按序列号匹配失败时，那里有
    //    完整的枚举结果（型号／序列号／厂商／设备键），正是操作者查
    //    接错线序所需的全部信息。只迭代配置就拿不到它。
    struct Slot
    {
        const data::CameraConfig*                      cfg;
        const std::shared_ptr<device::ICameraBackend>* backend;
    };
    const Slot slots[] = {{&c25, &ctx_.backend25},
                          {&c50, &ctx_.backend50},
                          {&c100, &ctx_.backend100}};
    for (const Slot& slot : slots)
    {
        const data::CameraConfig* c = slot.cfg;
        if (c->backend != "imv")
        {
            continue;
        }
        if (!ctx_.cameras->channelAvailable(c->role))
        {
            // ⚠ 走到这里时**管理器**的 `lastError()` 通常是**空**的：
            //    `initializeAll()` 开头就把它清空，而
            //    `availableCameraCount() >= 2` 又不触发它写 1001 —— 即
            //    "一路真实相机没打开"这件事在管理器里**不产生**错误文本
            //    （那一路的失败是瞬态、可降级的，见 `shouldDisableChannel`）。
            //
            //    ⚠ 上一版据此写的是"（无附加说明——单路初始化失败的明细见
            //    各后端自身日志）"，**那是一个假的指路牌**：后端并不写
            //    那种日志，实测（`backend: imv` + 本机不存在的序列号）
            //    进程级输出里除了"未能就绪"什么都没有 —— 而后端手里明明
            //    有枚举结果，`计划 §3.2` 要求"把枚举到的型号／序列号全列进
            //    错误信息"。故本批把**后端自己的说明**接到这里来。
            //
            // 取值顺序：后端自身的说明 → 管理器的错误文本 → "后端自身未给出
            // 说明"。最后一条如实说"没有"，且**不再指向任何不存在的日志**。
            const std::string backendDetail = (*slot.backend)->lastErrorText();
            const std::string managerDetail = ctx_.cameras->lastError().message;
            const std::string detail =
                !backendDetail.empty()
                    ? backendDetail
                    : (!managerDetail.empty() ? managerDetail
                                              : std::string("（后端自身未给出说明）"));
            errorText_ = "真实相机接入失败：通道 " + c->cameraId +
                         "（backend: imv，目标序列号 \"" + c->serialNumber +
                         "\"）未能就绪。设备层报告：" + detail +
                         "。本程序**不会**用其它通道或虚拟后端来让本次启动"
                         "看起来成功。";
            rollbackDevices();
            return false;
        }
    }

    if (!ctx_.cameras->startAll())
    {
        errorText_ = "MultiCameraManager::startAll 失败（" +
                     ctx_.cameras->lastError().message + "）";
        rollbackDevices();
        return false;
    }

    // ---- 触发 ----
    ctx_.trigger =
        std::make_unique<device::VirtualTriggerController>(ctx_.config->trigger());
    if (!ctx_.trigger->initialize())
    {
        errorText_ = "触发控制器初始化失败：" + ctx_.trigger->lastErrorText();
        return false;
    }
    if (!ctx_.trigger->enable())
    {
        errorText_ = "触发控制器使能失败：" + ctx_.trigger->lastErrorText();
        return false;
    }
    if (!ctx_.trigger->lastErrorText().empty())
    {
        // VirtualTriggerController 会把"配置要求 hardware 而实际是虚拟触发"
        // 这类不一致记在这里。它不致命，但必须说出来 ——
        // 否则这类不一致只会表现为"同步精度莫名偏低"。
        warn("触发：" + ctx_.trigger->lastErrorText());
    }

    // ---- 转台 ----
    ctx_.turntable =
        std::make_unique<device::VirtualTurntable>(ctx_.config->turntable());
    if (!ctx_.turntable->initialize())
    {
        errorText_ = "转台初始化失败（检查 turntable.yaml 的行程限位与速度）";
        return false;
    }

    // 装配摘要的两个数由**实际对象与设备回报**推出，不写常数（§3.4）：
    //   cameraCount      = 真的造出来的后端数（少一路就是 2，不是 3）
    //   cameraReadyCount = 初始化成功且未被禁用的路数
    // ⚠ 两者都要如实：只报前者会把"三路都装配了"说成"三路都能用"。
    summary_.cameraCount = 0;
    for (const auto& b : {ctx_.backend25, ctx_.backend50, ctx_.backend100})
    {
        if (b)
        {
            ++summary_.cameraCount;
        }
    }
    summary_.cameraReadyCount = ctx_.cameras->availableCameraCount();

    // 预览与测量共用同一套取帧预算（装配点是唯一的注入处，ENG-10 §5.2）。
    const data::MeasurementConfig& mcfg = ctx_.config->measurement();
    ctx_.cameras->setGrabBudget(mcfg.grabTimeoutMs, mcfg.grabGroupBudgetNs);

    logAssemblySummary();

    log("app", "设备：触发已使能、转台已就绪；取帧预算 " +
                   std::to_string(mcfg.grabTimeoutMs) + " ms/路，组预算 " +
                   std::to_string(mcfg.grabGroupBudgetNs / 1000000ULL) + " ms");
    return true;
}

// ---------------------------------------------------------------------------
//  装配摘要与回滚（011-A1，§3.4）
// ---------------------------------------------------------------------------

void SystemInitializer::rollbackDevices()
{
    // 顺序＝建立顺序的**逆序**：先转台/触发（本批无资源需归还），
    // 再相机逐路 stop() → close()。
    //
    // ⚠ 为什么每一步都要求"幂等、允许未初始化时调用"：本函数在**启动失败**
    //    时被调用，而失败点可能在启动序列的任何一步 —— 有的后端已 start、
    //    有的只 initialize 过、有的压根没造出来。"先判断它到过哪一步"
    //    会让回滚逻辑与启动序列耦合，故转而要求被调方自己吃下这些情形
    //    （`stop()`／`close()` 的契约已如此冻结）。
    //
    // ⚠ 逆序不是形式：`close()` 关设备、销毁句柄，之后 `stop()` 已无处可停。
    //    反过来（先 close 再 stop）在真实后端上会向一个已销毁的句柄停流。
    const std::shared_ptr<device::ICameraBackend> order[] = {
        ctx_.backend100, ctx_.backend50, ctx_.backend25};

    for (const auto& b : order)
    {
        if (!b)
        {
            continue;
        }
        b->stop();    // 停流；未 start 时无副作用
        const data::OperationResult r = b->close();
        if (!r.ok())
        {
            // 回滚本身失败**不覆盖**启动失败的原因（首因优先，§2.2 修正 4）：
            // errorText_ 已由调用方写好。这里只如实记一条，因为
            // "资源没还回去"是操作者需要知道的事实。
            warn("回滚：" + resultText(r) +
                 "（清理失败的原码见设备层日志）");
        }
    }

    // 先停流再销毁管理器：`stopAll()` 语义不变（只停流、允许重复调用），
    // 上面已逐路 stop 过，这里不再重复调 —— 重复调用虽安全，但会让
    // "回滚到底跑了哪些步骤"变得难以从日志读出来。
    ctx_.cameras.reset();
}

void SystemInitializer::logAssemblySummary()
{
    // 一行一路，四件事分开写（§3.4）：
    //   角色 / 逻辑 cameraId / 后端类型 / 设备身份（型号＋序列号或"未取得"）
    //   / **配置里的目标序列号单独一栏** / 可用性
    //
    // ⚠ 目标序列号必须独立成栏，**不得**与设备回报的序列号合成一格：
    //    本批的核心修正之一就是"型号匹配不能替代设备身份匹配"，
    //    若把"配置想要谁"与"实际连上谁"印成同一个字段，
    //    一份"配置写 A、连上 B"的错误装配会看起来完全正常。
    struct Row
    {
        data::CameraRole                role;
        const data::CameraConfig*       cfg;
        const std::shared_ptr<device::ICameraBackend>* backend;
    };
    const Row rows[] = {
        {data::CameraRole::CAM25,  &ctx_.config->camera(data::CameraRole::CAM25),  &ctx_.backend25},
        {data::CameraRole::CAM50,  &ctx_.config->camera(data::CameraRole::CAM50),  &ctx_.backend50},
        {data::CameraRole::CAM100, &ctx_.config->camera(data::CameraRole::CAM100), &ctx_.backend100},
    };

    log("app", "装配摘要：");
    for (const Row& r : rows)
    {
        const std::shared_ptr<device::ICameraBackend>& b = *r.backend;

        if (!b)
        {
            log("app", "  · " + std::string(roleText(r.role)) + "（" +
                           r.cfg->cameraId + "）：未装配"
                           "（backend=\"" + r.cfg->backend + "\"）");
            continue;
        }

        // 后端类型由**实际动态类型**得出，不由配置声明抄一遍 ——
        // 抄配置的话，装配代码一旦接错（把 imv 建成虚拟），摘要会跟着一起错。
        const char* type = (std::dynamic_pointer_cast<device::ImvCameraBackend>(b))
                               ? "真实（ImvCameraBackend）"
                               : "虚拟（VirtualCameraBackend）";

        const data::DeviceIdentity id = b->deviceIdentity();
        const std::string identity =
            id.queried
                ? ("型号 \"" + (id.modelName.empty() ? std::string("未取得")
                                                     : id.modelName) +
                   "\"、序列号 \"" +
                   (id.serialNumber ? *id.serialNumber : std::string("未取得")) +
                   "\"")
                : std::string("未查询（本后端不连设备）");

        log("app", "  · " + std::string(roleText(r.role)) + "（" +
                       r.cfg->cameraId + "）：" + type + "；设备身份 " + identity +
                       "；配置目标序列号 \"" +
                       (r.cfg->serialNumber.empty() ? std::string("（未绑定）")
                                                    : r.cfg->serialNumber) +
                       "\"；可用性 " +
                       (ctx_.cameras && ctx_.cameras->channelAvailable(r.role)
                            ? "就绪"
                            : "不可用"));
    }
}

// ---------------------------------------------------------------------------
//  步骤 6：预览
// ---------------------------------------------------------------------------

bool SystemInitializer::buildPreview()
{
    preview::PreviewConfig pcfg;
    pcfg.defaultCamera = data::CameraRole::CAM25;
    pcfg.mode          = preview::PreviewMode::AUTO;   // SYS-08 §8

    ctx_.preview = std::make_unique<preview::PreviewManager>(pcfg, ctx_.rig.get());

    for (const std::string& n : ctx_.preview->configNotices())
    {
        warn("预览配置：" + n);
    }

    // 消费者线程（见 ApplicationContext.h 中 previewWorker 的说明）。
    ctx_.previewWorker = std::make_unique<preview::PreviewWorker>(*ctx_.preview);
    if (!ctx_.previewWorker->start())
    {
        errorText_ = "预览线程启动失败（std::thread 创建失败，"
                     "通常是进程句柄/内存不足）";
        return false;
    }

    log("app", "预览：AUTO 模式，队列容量 " +
                   std::to_string(ctx_.preview->queue().capacity()));
    return true;
}

// ---------------------------------------------------------------------------
//  步骤 7~9：机型库、统计表、算法链
// ---------------------------------------------------------------------------

bool SystemInitializer::buildAlgorithm()
{
    const data::SystemConfig& sys = ctx_.config->system();

    // ---- 机型库 ----
    // 缺失**不构成启动失败**：特征库是后续交付物（ENG-08 §5）。
    // 但必须让"本次跑不出姿态"这件事从启动第一秒就可见 ——
    // 否则操作者会走完整个 60 秒流程才在 POSE_SOLVE 拿到一个 5001，
    // 而真正的原因在启动时就写在日志里了。
    ctx_.models = std::make_unique<algorithm::TargetModelManager>();
    if (ctx_.models->loadModel(sys.modelDir))
    {
        ctx_.modelsAvailable = true;
        log("app", "机型库：" + ctx_.models->modelId() + " / " +
                       ctx_.models->modelVersion() + "（" + sys.modelDir + "）");
    }
    else
    {
        ctx_.modelsAvailable = false;
        warn("机型库加载失败或缺失（model_dir=" + sys.modelDir +
             "）：测量流程可以启动，但 POSE_SOLVE 无法完成，"
             "本次运行的姿态结果不可用");
    }

    // ---- 历史匹配统计 ----
    // 路径固定在 runtime/ 下。**SystemConfig 中没有该字段**（冻结文档
    // 未定义），故此处按 ENG-10 §4 的约定写死相对路径，并登记待裁决
    // （是否在 SystemConfig 中补 matchStatsPath）。
    const std::string statsPath =
        infrastructure::fileutil::joinPath("runtime", "match_stats.yaml");

    ctx_.stats = std::make_unique<infrastructure::FileMatchStatsStore>(
        statsPath, ctx_.config->measurement());

    // load() 返回 false 有两种含义，必须分开对待（见该 .cpp）：
    //   · 文件不存在 → 返回 true（冷启动是正常的）；
    //   · 解析失败 → 返回 false，且已把现场文件改名保留。
    if (!ctx_.stats->load())
    {
        warn("历史匹配统计解析失败：" + ctx_.stats->lastErrorText());
    }
    else if (ctx_.stats->loadedFromFile())
    {
        log("app", "历史匹配统计：已载入 " +
                       std::to_string(ctx_.stats->modelCount()) + " 个机型的记录");
    }
    else
    {
        log("app", "历史匹配统计：冷启动（" + statsPath +
                       " 不存在，属正常首次运行）");
    }
    summary_.statsLoaded = ctx_.stats->loadedFromFile();

    // ---- 算法链 ----
    // ENG-10 §5.2：全部依赖在**构造时一次性注入**，之后不可变。
    // 这里传 rig->calibration() 的**值**（PosePipeline 按值持有，
    // 见其头文件），故标定在进程内被重新装载也不会影响已注入的链条 ——
    // 这正是 §5.2 约束 2 要的效果。
    //
    // ⚠ 尺度估计阶段在 M1 用 **MockTargetScaleEstimator**（11.md §十一
    //    "Algorithm 暂时 Mock"）。这不是图省事，而是**实测到的硬约束**：
    //
    //    PinholeScaleEstimator 的 Z = f_x·L/l 需要一个"目标真实尺寸 L"，
    //    而 L 来自机器型库（`TargetModelManager::loadModel` 取 points3d
    //    包围盒的最长边）。M1 阶段 models/ 目录不存在（特征库与三维点表
    //    属后续交付物），于是 L = 0，该阶段返回 false。
    //    而 SYS-08 §5.3 的 TARGET_FOUND 一旦算不出 TargetOffset 就没有
    //    经批准的恢复路径（见 MeasurementStrategy.cpp 中该分支的长注释），
    //    后果是**状态机在 TARGET_FOUND 就地 FAILED** —— 实测就是这样：
    //    `SEARCH → TARGET_FOUND → FAILED`，用时 26 ms，
    //    后面 9 个状态（ALIGN / STABILIZE / MEASURE_SELECT / CAPTURE /
    //    POSE_SOLVE / VALIDATE / SAVE）一个都走不到。
    //
    //    注入 MockTargetScaleEstimator 后，该阶段不再依赖机型库，
    //    状态机得以推进到 ALIGN 及其后 —— 这正是 11.md §十 第 5 条
    //    要验证的"IDLE → SEARCH → TARGET_FOUND → ALIGN → …"。
    //
    //    ⚠ 距离取 120 m 是**任意的**，唯一要求是落在 40~300 m 的
    //    有效量程内、且落在某个距离带（ENG-10 §4.2 的四档）里，
    //    使 M_hist 的分桶有意义。它不代表任何真实目标距离。
    //    已登记待裁决：M1/M2 是否应提供一个最小的合成机型库
    //    （points3d.yaml + model.yaml + 三份 feature*.bin），
    //    使 POSE_SOLVE 也能在无实物交付的前提下跑通。
    const bool mockScale = !ctx_.modelsAvailable;
    if (mockScale)
    {
        ctx_.mockScale = std::make_unique<algorithm::MockTargetScaleEstimator>(
            120.0 /* m，任意，见上 */, 1.0 /* confidence */);

        algorithm::PosePipeline::Stages stages;
        stages.scale = ctx_.mockScale.get();
        // 其余阶段留 nullptr → Pipeline 为每一个自建默认实现
        //（见 PosePipeline 全注入构造的说明）。故此处**只**替换尺度阶段，
        // 检测仍是 MockTargetDetector、特征与 PnP 仍是真实实现 ——
        // 与 008/009 已验收的算法链相比，只有这一个阶段不同。
        ctx_.pipeline = std::make_unique<algorithm::PosePipeline>(
            ctx_.config->measurement(), ctx_.config->validation(),
            *ctx_.models, ctx_.rig->calibration(), stages, ctx_.stats.get());
    }
    else
    {
        ctx_.pipeline = std::make_unique<algorithm::PosePipeline>(
            ctx_.config->measurement(), ctx_.config->validation(),
            *ctx_.models, ctx_.rig->calibration(), ctx_.stats.get());
    }

    log("app", std::string("算法链：") +
                   (mockScale
                        ? "MockTargetDetector / **MockTargetScaleEstimator** / "
                          "SiftFeatureExtractor / DescriptorFeatureMatcher / "
                          "CvPnPPoseEstimator"
                        : "MockTargetDetector / PinholeScaleEstimator / "
                          "SiftFeatureExtractor / DescriptorFeatureMatcher / "
                          "CvPnPPoseEstimator"));
    warn("算法链使用的是**桩检测器**（MockTargetDetector）："
         "它按图像的亮块给出检测框，不是真实的 YOLO 目标检测。"
         "该链路只用于打通 M1 闭环，检测与位姿精度均不具参考意义。");
    return true;
}

// ---------------------------------------------------------------------------
//  步骤 10~12：落盘、适配器、测量总控
// ---------------------------------------------------------------------------

bool SystemInitializer::buildApplication()
{
    // ---- 落盘 ----
    ctx_.recorder = std::make_unique<infrastructure::Recorder>(
        ctx_.config->system(), ctx_.configDir,
        ctx_.calibration->calibrationId(),
        ctx_.models->modelId().empty() ? std::string("unknown")
                                       : ctx_.models->modelId(),
        ctx_.models->modelVersion().empty() ? std::string("unknown")
                                            : ctx_.models->modelVersion());

    // ---- 适配器 ----
    // 顺序约束：controller 的构造需要 sink 的地址，sink 需要 recorder 的引用。
    ctx_.recorderSink = std::make_unique<RecorderSinkAdapter>(*ctx_.recorder);

    // ---- 测量总控 ----
    // 构造参数全部是协作者，不含任何配置读盘 —— 配置已在前面读入并注入。
    //
    // ⚠ `ctx_.modelsAvailable` 是本装配点**独有**的知识（它在 buildAlgorithm()
    // 里由机型库的加载结果置位，见本文件机型库一节）。控制器的
    // `IPosePipeline::solvePose` 只回一个 bool（IF-SW-02 冻结），机型库缺失
    // 与真正的 PnP 失败在那里不可区分，故必须由这里把该事实注入 ——
    // 否则"没装机型库"这条根因只能以 9004 兜底码出现，现场就要自己去猜。
    ctx_.controller = std::make_unique<application::MeasurementController>(
        *ctx_.cameras, *ctx_.turntable, *ctx_.pipeline, *ctx_.rig,
        ctx_.config->measurement(), ctx_.config->turntable(),
        ctx_.preview.get(), ctx_.trigger.get(), ctx_.recorderSink.get(),
        ctx_.modelsAvailable);

    for (const std::string& n : ctx_.controller->notices())
    {
        warn("测量控制器：" + n);
    }

    log("app", "装配完成：MeasurementController 已就绪，等待 startMeasurement()");
    return true;
}

// ---------------------------------------------------------------------------
//  启动自检
// ---------------------------------------------------------------------------

void SystemInitializer::selfCheck()
{
    const data::SystemConfig& sys = ctx_.config->system();

    // ---- 可写目录（写目标）----
    std::string why;
    if (!ensureWritableDir(sys.logDir, why))
    {
        warn("日志目录不可写：" + why);
    }
    if (!ensureWritableDir(sys.outputDir, why))
    {
        // 结果落盘失败不会让测量流程失败（SAVE 只是记录 notice），
        // 故不是致命项；但必须在这里说出来 —— 否则操作者会一直等到
        // 第一次测量完成才发现结果包没生成。
        warn("结果输出目录不可写：" + why +
             "：测量仍可进行，但结果包无法落盘");
    }
    else
    {
        log("app", "输出目录就绪：" + sys.outputDir);
    }

    // ---- 标定与相机分辨率是否自洽 ----
    // 这是实测中最容易静默出错的一处：合成标定按 CAM25 的尺寸生成内参
    // （cx, cy 在图像中心），若 CAM50/100 的分辨率与之不同，PnP 会用
    // 一组"中心偏了半个视场"的内参去解另外两路的图像 —— 结果能算出来、
    // 不会报错，只是误差大到没有意义。
    if (ctx_.rig)
    {
        for (const data::CameraChannel& ch : ctx_.rig->cameras())
        {
            const data::CameraConfig& cfg = ctx_.config->camera(ch.role);
            const data::CameraCalibration cal =
                ctx_.calibration->getCalibration(ch.role);
            if (cal.imageWidth > 0 && cal.imageHeight > 0 &&
                (cal.imageWidth != cfg.width || cal.imageHeight != cfg.height))
            {
                warn("相机 " + cfg.cameraId + " 的配置分辨率 " +
                     std::to_string(cfg.width) + "x" + std::to_string(cfg.height) +
                     " 与标定分辨率 " + std::to_string(cal.imageWidth) + "x" +
                     std::to_string(cal.imageHeight) +
                     " 不一致：内参（含主点）将错配，姿态结果不可用");
            }
        }
    }

    // ---- 降级起点 ----
    // 启动时不应处于降级态；若可用相机数不是 3，说明设备层已经出了问题，
    // 而 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求这必须在第一秒就可见。
    summary_.degradedAtStart = false;
    if (ctx_.cameras && ctx_.cameras->degraded())
    {
        summary_.degradedAtStart = true;
        warn("启动时即有相机不可用：本次运行的测量精度将下降");
    }
}

// ---------------------------------------------------------------------------
//  initialize
// ---------------------------------------------------------------------------

bool SystemInitializer::initialize(const std::string& configDir)
{
    ctx_.configDir = configDir;

    // 顺序即 ENG-02 §15（见 ApplicationContext.h 的逐项对照）。
    if (!loadConfigAndLogger())
    {
        return false;
    }
    if (!buildOptical())
    {
        return false;
    }
    if (!buildDevices())
    {
        return false;
    }
    if (!buildPreview())
    {
        return false;
    }
    if (!buildAlgorithm())
    {
        return false;
    }
    if (!buildApplication())
    {
        return false;
    }

    selfCheck();

    // 界面侧的告警汇总：启动有告警时，第一条必须能让操作者知道
    // "有话要说"，否则它们只会停在日志里。
    if (!warnings_.empty())
    {
        log("app", "启动完成，共 " + std::to_string(warnings_.size()) +
                       " 条告警（见 WARN 行）");
    }
    else
    {
        log("app", "启动完成，无告警");
    }

    ready_ = true;
    return true;
}

// ---------------------------------------------------------------------------
//  运行期
// ---------------------------------------------------------------------------

bool SystemInitializer::pumpIdlePreview(uint64_t nowNs)
{
    if (!ctx_.cameras || !ctx_.preview)
    {
        return false;
    }

    // 空闲态：状态机不推进任何采集动作，此处的采集**只**为预览服务。
    // 显示源由 AUTO 映射给出（IDLE → CAM25，见 PreviewManager
    // mapStateToCamera），submitFrom 会据此挑出那一路。
    //
    // ⚠ 期限（011-A1，§3.2）：预览路径**没有**状态机时限可借，故用
    //    "本次预览采集"的期限＝现在 + 组预算。两条纪律：
    //      · 只用 `nowNs`（本拍的时刻，由调用方注入）不用真实钟 ——
    //        否则测试注入的假时间会被墙钟判过期（与控制器同一条理由）；
    //      · 期限**算在这里**、每拍重算，不由管理器缓存（§2.2 修正 3）。
    const uint64_t budgetNs = ctx_.config->measurement().grabGroupBudgetNs;
    const uint64_t deadlineNs =
        (nowNs > (std::numeric_limits<uint64_t>::max)() - budgetNs)
            ? (std::numeric_limits<uint64_t>::max)()
            : nowNs + budgetNs;

    data::MultiCameraFrame frame;
    if (!ctx_.cameras->capture(frame, deadlineNs))
    {
        return false;
    }
    (void)nowNs;   // 帧自带时间戳（虚拟后端按采集时刻填写）

    if (!ctx_.preview->submitFrom(frame))
    {
        return false;
    }
    ++summary_.idleFrames;
    return true;
}

bool SystemInitializer::tick(uint64_t nowNs)
{
    if (!ready_)
    {
        return false;
    }
    ++summary_.ticks;

    // 1) 推进状态机。返回值 = 本次是否改变了状态。
    const bool stateAdvanced = ctx_.controller->tick(nowNs);

    // 2) 空闲态补帧（见文件头说明）。规则两条：
    //    ① 空闲与否按**推进后**的状态判定 —— 本拍刚转入终态时，
    //       画面正是操作者最需要看到的那一帧；
    //    ② **转入终态当拍统一跳过空闲补帧**（R04 收尾；判据是 `stateAdvanced`
    //       即"本拍状态是否推进"，**不是**"本拍有没有采集"）：这类当拍
    //       已采集的分支（SEARCH / ALIGN / CAPTURE）再补一次就是同一拍
    //       两次 capture()。下一拍状态不再推进，空闲预览自然恢复。
    //
    //       ⚠ 上一版这里写的理由是"`stateAdvanced` 为真意味着这一拍内
    //          控制器已经跑过活动态采集并提交了帧" —— **该推断不成立**：
    //          `POSE_SOLVE` / `VALIDATE` / `SAVE` 会推进状态却**不采集**
    //          （它们消费上一拍留下的帧）。规则本身不变（按状态推进判定
    //          是对的，见 .h 的说明），只是不能拿"已经采集过"当它的理由。
    //
    //    ⚠ ② 挡的是一条**真实可达**的边界，不是假想：
    //    "活动态采集成功、当拍转入终态"就发生在 ALIGN ——
    //    采集成功 → 检测 → 对准命令越程（2002）→ 能力边界不重试 →
    //    当拍 FAILED。去掉这一条，那一拍会采集两次。
    const data::MeasurementState st = ctx_.controller->state();
    const bool idle = (st == data::MeasurementState::IDLE ||
                       st == data::MeasurementState::COMPLETE ||
                       st == data::MeasurementState::FAILED);

    bool idleSubmitted = false;
    if (idle)
    {
        // 空闲态下 controller 不推进状态，故由本类负责让预览的 AUTO
        // 映射跟上（它自己只在状态迁移时通知）。
        // ⚠ setMeasurementState() 与补帧**分开**判断：AUTO 映射必须跟上
        //   推进后的状态（否则失败画面会被映射到旧焦段），而补帧要看
        //   本拍有没有推进。
        ctx_.preview->setMeasurementState(st);
        if (!stateAdvanced)
        {
            idleSubmitted = pumpIdlePreview(nowNs);
        }
    }

    // 3) 状态变化 → 界面必须刷新（含进入终止态那一次）。
    const int stateValue  = static_cast<int>(st);
    const bool stateChanged = (stateValue != lastState_);
    lastState_ = stateValue;

    // 4) 落盘结果上报一次。用"已上报"标记而不是比较字符串：
    //    -- 同一次结果会持续存在于 recorder 里，若不记住就会每拍都报一遍。
    bool saveReported = false;
    if ((st == data::MeasurementState::COMPLETE ||
         st == data::MeasurementState::FAILED) &&
        !lastSaveReported_)
    {
        lastSaveReported_ = true;
        saveReported      = true;

        if (ctx_.recorderSink->lastErrorText().empty() &&
            !ctx_.recorderSink->lastPackageDir().empty())
        {
            log("app", "结果包：" + ctx_.recorderSink->lastPackageDir());
        }
        else if (!ctx_.recorderSink->lastErrorText().empty())
        {
            warn("结果包问题：" + ctx_.recorderSink->lastErrorText());
        }
    }
    else if (st == data::MeasurementState::IDLE ||
             st == data::MeasurementState::SEARCH)
    {
        // 新任务开始：允许下一次终止态再次上报。
        lastSaveReported_ = false;
    }

    return stateAdvanced || stateChanged || idleSubmitted || saveReported;
}

// ---------------------------------------------------------------------------

bool SystemInitializer::startMeasurement()
{
    if (!ready_)
    {
        return false;
    }
    lastSaveReported_ = false;

    const data::MeasurementState before = ctx_.controller->state();
    ctx_.controller->startMeasurement();
    const data::MeasurementState after = ctx_.controller->state();

    log("app", std::string("收到测量启动请求：") +
                   (before == after ? "状态未改变（可能已在测量中）"
                                    : "状态机已启动"));

    if (after == data::MeasurementState::FAILED)
    {
        const data::ErrorInfo e = ctx_.controller->lastError();
        warn("测量未能启动：" + std::string(data::errorCodeName(e.code)) +
             " " + e.message);
        return false;
    }
    return true;
}

void SystemInitializer::stopMeasurement()
{
    if (ready_)
    {
        ctx_.controller->stopMeasurement();
        log("app", "收到测量停止请求（已同时发出转台停止命令）");
    }
}

// ---------------------------------------------------------------------------

ui::MeasurementView SystemInitializer::view() const
{
    ui::MeasurementView v;
    if (!ready_)
    {
        return v;   // valid = false → 界面显示占位
    }

    v.valid      = true;
    v.state      = ctx_.controller->state();
    v.degraded   = ctx_.controller->degraded();
    v.availableCameras = ctx_.controller->availableCameraCount();
    v.selectedCamera   = ctx_.controller->selectedCamera();

    // ---- 姿态 ----
    // 只在**解算成功**时才把数字交给界面。失败时 poseResult 里是一组
    // 零值，显示出来就是"目标姿态为 0°"—— 那是一个看起来完全正常的
    // 读数，而真相是"没算出来"。二者必须可区分。
    const data::ShipPoseResult pose = ctx_.controller->poseResult();
    if (pose.success)
    {
        v.hasPose          = true;
        v.yaw              = pose.yaw;
        v.pitch            = pose.pitch;
        v.roll             = pose.roll;
        v.reprojectionError = pose.reprojectionError;
    }

    // ---- 验证 ----
    if (v.state == data::MeasurementState::SAVE ||
        v.state == data::MeasurementState::COMPLETE)
    {
        const data::PoseValidationResult val = ctx_.controller->validationResult();
        v.hasValidation = true;
        v.confidence    = val.confidence;
        v.inlierRatio   = val.inlierRatio;
    }

    // ---- 转台 ----
    if (ctx_.turntable)
    {
        v.hasTurntable = true;
        v.turntable    = ctx_.turntable->state();
    }

    // ---- 单行文字 ----
    // 优先级：失败原因 > 测量控制器 notice > 状态描述。
    // 失败原因排第一，因为它决定了操作者接下来要做什么；
    // 而它们中的任何一个都不该被"就绪"这类常规描述盖掉。
    const data::ErrorInfo err = ctx_.controller->lastError();
    if (err.code != 0)
    {
        v.message = QStringLiteral("%1：%2")
                        .arg(QString::fromUtf8(data::errorCodeName(err.code)),
                             QString::fromStdString(err.message));
    }
    else if (!err.message.empty())
    {
        // ⚠ code == 0 且**已有 message** 是一个真实存在的组合：SYS-08 的
        //    若干失败路径（如 TARGET_FOUND 无恢复路径）没有对应的登记错误码，
        //    MeasurementStrategy 只能以 code=0 上报。
        //    此时**绝不能**走 errorCodeName(0) —— 它返回的字面量是 "OK"，
        //    界面会显示"OK：<失败原因>"，即在一条失败记录前面盖一个
        //    "正常"的章。两个字段各自都对，合起来是一句谎话。
        //    故此处只显示原因，不加码名；码缺失这件事由 README §6 的
        //    待裁决行跟踪（应否为这类失败补登记错误码）。
        v.message = QString::fromStdString(err.message);
    }
    else
    {
        const std::vector<std::string> notices = ctx_.controller->notices();
        if (!notices.empty())
        {
            v.message = QString::fromStdString(noticeText(notices));
        }
        else if (!ctx_.modelsAvailable)
        {
            v.message = QStringLiteral("特征库缺失：姿态解算不可用（见日志 WARN）");
        }
        else if (v.state == data::MeasurementState::IDLE)
        {
            v.message = QStringLiteral("就绪");
        }
        else if (v.state == data::MeasurementState::COMPLETE)
        {
            v.message = QStringLiteral("测量完成");
        }
    }

    // ---- 结果包 ----
    if (ctx_.recorderSink)
    {
        const std::string dir = ctx_.recorderSink->lastPackageDir();
        if (!dir.empty())
        {
            v.record = QString::fromStdString(dir);
        }
    }

    return v;
}

}  // namespace app
}  // namespace aircraft
