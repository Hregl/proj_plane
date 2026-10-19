// ============================================================================
//  tests/integration/SystemInitializerTest.cpp
//
//  依据：ENG-01 §14（app = 全工程唯一装配点）
//        ENG-02 §15（冻结的对象创建顺序）、§16（生命周期）
//        SYS-08 §8（AUTO 显示源映射）、§9（状态机运行于 Application 线程）
//        SYS-08 §7.7（能力边界不重试 → 2002）
//        R04 收尾（2026-09-24 评审裁定）：空闲补帧的两条规则
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本文件证明的是 **app 层**（src/app）的两件事，此前无人验证：           │
//  │   A. 逐状态的**采集归属** —— 每一拍到底谁在抓帧、抓了几次；           │
//  │   B. 空闲补帧的**边界** —— "活动态采集成功、当拍转入终态" 那一拍，     │
//  │      不得再补一帧预览（`SystemInitializer::tick` 的 `!stateAdvanced`）。│
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ---- 与 MeasurementFlowTest 的分工 ----
//
//  MeasurementFlowTest 用**桩算法链**证明状态机的收敛性（SYS-08 §10）；
//  本文件相反：**一个桩都不注入**，走 `SystemInitializer::initialize()` 的
//  真实装配（真实 7 个 yaml、真实标定装载、真实 VirtualCameraBackend /
//  VirtualTurntable、真实 PosePipeline 的桩检测器）。差别是本质的：
//  前者能证明"状态机在给定协作下必然终止"，后者能证明"装配出来的这套东西
//  真的会采帧、真的会走 ALIGN、真的会在越程时停"。
//
//  ---- 为什么可以不新增任何生产 API ----
//
//  `ApplicationContext` 是**调用方构造**、以引用交给 `SystemInitializer`
//  （main.cpp:98-99），故测试自建一个 ctx，再用 ctx 里**真实设备**的
//  **既有公开 API**（`VirtualCameraBackend::setTargetPixelOffset`、
//  `VirtualTurntable::setInitialAngles`）就能摆出所需场景。
//  本批**不新增注入槽、不新增生产 API**（评审 2026-09-24 的裁定）。
//
//  ---- 观测手段（全部是生产侧的既成事实，不新增探针）----
//
//  · 每拍采集次数 → 三路 `VirtualCameraBackend::frameCount()` 的逐拍增量。
//    `MultiCameraManager::capture()` 对每个可用通道各 grab 一次，
//    故"一路的增量"就是"capture() 被调用的次数"。
//  · 空闲补帧 → `SystemInitializer::summary().idleFrames` / `.ticks`。
//  · 预览侧 → `PreviewQueue::pushedCount()`。**不得 pop()**：装配真的起了
//    `PreviewWorker` 消费者线程，pop() 会与它抢队列；计数器不受消费影响。
//
//  ---- 环境约束（照做，不是风格）----
//
//  · CWD：`system.yaml` 的 log_dir / output_dir / model_dir 与写死的
//    `runtime/match_stats.yaml` 都是**相对进程 CWD** 的。故本套件建一个
//    临时目录、chdir 进去、把仓库 `config/` 的 7 个 yaml 拷进 `config/`
//    —— **不重写任何 yaml**（路径本就是相对的），仓库目录不被污染。
//  · 临时目录里**没有** `models/`、`calibration/`，这正是要的：
//    modelsAvailable=false ⇒ 注入 MockTargetScaleEstimator（否则状态机在
//    TARGET_FOUND 就地 FAILED，见 SystemInitializer::buildAlgorithm 的说明）；
//    calibration_mode="synthetic" ⇒ 合成内参可用，ALIGN 的 4001 前提成立。
//  · 时间基准**一律用真实单调时钟**，不自造时间戳：`startMeasurement()`
//    内部取的就是 `data::monotonicNowNs()`，虚拟转台默认也用真实时钟。
//    混用基准会让 deadlineExceeded 的判定失真。
//  · ⚠ 本机 GoogleTest 未安装 ⇒ CMake 的测试目标不生成，实跑走
//    `/tmp/gtshim/build_tests.sh`（见 README §6）。
// ============================================================================

// ============================================================================
//  ⚠ 本文件引用的 SYS-08 §7.x 经核实为悬空／撞号引用（2026-09-26 复核），依据待裁决，见《待裁决问题汇总》Q-D2 与《SYS-08-§7引用勘误.md》；正文引用仅描述现行行为，不作为冻结依据。
// ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#include "algorithm/model/TargetModelManager.h"
#include "app/ApplicationContext.h"
#include "app/SystemInitializer.h"
#include "data/CameraConfig.h"
#include "data/CameraRole.h"
#include "data/DeviceIdentity.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"
#include "data/MonotonicClock.h"
#include "data/SyntheticTargetGeometry.h"
#include "data/TargetModel.h"
#include "data/TurntableConfig.h"
#include "device/camera/ICameraBackend.h"
#include "device/camera/SyntheticTargetScene.h"
#include "device/camera/VirtualCameraBackend.h"
#include "device/turntable/ITurntableController.h"
#include "device/turntable/PelcoDTurntableController.h"
#include "device/turntable/VirtualTurntable.h"
#include "infrastructure/FileUtil.h"
#include "infrastructure/config/ConfigManager.h"
#include "preview/PreviewQueue.h"

#ifndef APS_SOURCE_DIR
#error "APS_SOURCE_DIR 未定义：本套件需要它定位仓库的 config/ 目录（CMake: target_compile_definitions；shim: DEFS 里的 -D）。"
#endif

using aircraft::app::ApplicationContext;
using aircraft::app::SystemInitializer;
using aircraft::data::MeasurementState;
using aircraft::device::VirtualCameraBackend;
using aircraft::device::VirtualTurntable;
using S = aircraft::data::MeasurementState;

namespace
{

// ===========================================================================
//  状态名（仅打印用）
//
//  ⚠ 生产侧**没有**状态名函数（`grep -rn stateName src/` 零命中），
//    本文件不发明一个生产 API 去满足打印需求 —— 打印只服务人读的复查记录。
// ===========================================================================

const char* stateName(MeasurementState s)
{
    switch (s)
    {
    case S::IDLE:           return "IDLE";
    case S::SEARCH:         return "SEARCH";
    case S::TARGET_FOUND:   return "TARGET_FOUND";
    case S::ALIGN:          return "ALIGN";
    case S::STABILIZE:      return "STABILIZE";
    case S::MEASURE_SELECT: return "MEASURE_SELECT";
    case S::CAPTURE:        return "CAPTURE";
    case S::POSE_SOLVE:     return "POSE_SOLVE";
    case S::VALIDATE:       return "VALIDATE";
    case S::SAVE:           return "SAVE";
    case S::COMPLETE:       return "COMPLETE";
    case S::FAILED:         return "FAILED";
    }
    return "?";
}

// ===========================================================================
//  测试环境：临时工作目录
// ===========================================================================

const char* const kConfigFiles[] = {
    "camera.yaml", "measurement.yaml", "optical_rig.yaml", "system.yaml",
    "trigger.yaml", "turntable.yaml", "validation.yaml",
};

bool copyFile(const std::string& from, const std::string& to)
{
    std::ifstream in(from, std::ios::binary);
    if (!in)
    {
        return false;
    }
    std::ofstream out(to, std::ios::binary);
    if (!out)
    {
        return false;
    }
    out << in.rdbuf();
    return out.good();
}

bool& prepared()
{
    static bool p = false;
    return p;
}

/// 由仓库 `camera.yaml` 得到一份**三路全虚拟**的底本（定义见文件下方）。
///
/// ⚠ 前置声明而不是把定义挪到这里：定义依赖 `readFile` / `replaceOnce` /
///    `repoYaml`，全都在下方几百行处；而那三个助手是**通用**的，把它们一起
///    上移会让"负例改造"这组逻辑与"测试环境准备"挤在一起。声明一行即可。
std::string allVirtualCameraYaml();

/// 建临时工作目录、chdir 进去、把仓库 `config/` 的 7 个 yaml 拷进 `config/`。
///
/// **幂等**：CWD 是进程级状态，而本套件的两个用例同处一个进程
/// （真实 GTest 与本工程的垫片都是如此）。用一套"套件级环境"而不是逐用例
/// 各建一次，是为了让两个场景跑在**同一套已装配路径**上 ——
/// 若逐用例 chdir 到新目录，就要求每个用例都自己重新 initialize()，
/// 而"同一个进程里两次真实装配"本身不是本文件要证明的事。
///
/// ⚠ 不用 `SetUpTestSuite`：本工程的垫片（/tmp/gtshim/gtest/gtest.h）
///   只实现了 TEST / TEST_F 与 Test::SetUp/TearDown，没有套件级钩子。
///   显式调用一次可同时被两种运行方式支持。
bool prepareWorkspace(std::string& err)
{
    if (prepared())
    {
        return true;
    }

    char        tmpl[] = "/tmp/aps_sysinit_XXXXXX";
    char* const dir    = ::mkdtemp(tmpl);
    if (dir == nullptr)
    {
        err = "mkdtemp 失败";
        std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
        return false;
    }

    const std::string tmp(dir);
    if (::chdir(tmp.c_str()) != 0)
    {
        err = "chdir 失败：" + tmp;
        std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
        return false;
    }

    const std::string cfgDir = tmp + "/config";
    if (!aircraft::infrastructure::fileutil::makeDirectories(cfgDir))
    {
        err = "创建目录失败：" + cfgDir;
        std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
        return false;
    }

    for (const char* const f : kConfigFiles)
    {
        // ⚠ APS_SOURCE_DIR 必须由构建系统传入**已展开的绝对路径**
        //   （CMake: ${PROJECT_SOURCE_DIR}；shim: "$ROOT" 需在命令里展开）。
        const std::string src = std::string(APS_SOURCE_DIR) + "/config/" + f;

        // ⚠ camera.yaml **不是原样拷贝**：本套件的工作目录夹具（`Harness`、
        //   `countsOf`）按"三路都是 `VirtualCameraBackend`"建立，而仓库配置
        //   自 2026-09-28 起承载**现场接线的真实选择**（CAM25 = `imv` +
        //   实际序列号）。原样拷进来会有两个后果，且都不报在"配置"上：
        //     ① `countsOf` 的 `static_pointer_cast<VirtualCameraBackend>`
        //        对真实后端得到**空指针**，随后解引用 —— 失败点是**别处**；
        //     ② 用例会去打开物理设备，于是同一份代码在有设备与没设备的
        //        机器上表现不同。
        //   故此处显式写入三路全虚拟的底本：测试环境必须与现场接线无关。
        if (std::string(f) == "camera.yaml")
        {
            const std::string virtualYaml = allVirtualCameraYaml();
            std::ofstream out(cfgDir + "/" + f, std::ios::binary);
            out << virtualYaml;
            if (virtualYaml.empty() || !out.good())
            {
                err = "写入三路全虚拟的 camera.yaml 失败（未能定位三路记录？）：" +
                      cfgDir + "/" + f;
                std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
                return false;
            }
            continue;
        }

        if (!copyFile(src, cfgDir + "/" + f))
        {
            err = "拷贝失败（APS_SOURCE_DIR=" + std::string(APS_SOURCE_DIR) + "）：" + src;
            std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
            return false;
        }
    }

    std::fprintf(stderr,
                 "\n[SystemInitializerTest] 临时工作目录：%s —— 仓库 config/ 的 7 个 yaml "
                 "已拷入（camera.yaml **已改写为三路全虚拟**，其余未重写取值）；"
                 "logs/ output/ runtime/ 均落在该目录，仓库目录不被污染\n",
                 tmp.c_str());

    prepared() = true;
    return true;
}

// ===========================================================================
//  逐拍账
// ===========================================================================

struct ChannelCounts
{
    uint64_t c25  = 0;
    uint64_t c50  = 0;
    uint64_t c100 = 0;
};

/// 一拍的完整账目。字段名与断言直接对应，便于断言失败时人读定位。
struct TickAccount
{
    MeasurementState start = S::IDLE;   ///< 该拍**开始**时的状态（断言的定位依据）
    MeasurementState end   = S::IDLE;   ///< 该拍**结束**时的状态
    bool     advanced      = false;     ///< 该拍状态是否发生了迁移
    uint64_t c25  = 0;                  ///< 各路 frameCount 增量 = capture() 次数
    uint64_t c50  = 0;
    uint64_t c100 = 0;
    uint64_t idleFrames = 0;            ///< summary().idleFrames 增量（空闲补帧）
    uint64_t pushed     = 0;            ///< 预览队列 pushedCount 增量
};

void printAccount(const TickAccount& a, int index, const char* tag = "")
{
    std::fprintf(stderr,
                 "  [%2d] %-14s -> %-14s  Δcam25=%llu Δcam50=%llu Δcam100=%llu"
                 "  ΔidleFrames=%llu Δpushed=%llu%s\n",
                 index, stateName(a.start), stateName(a.end),
                 static_cast<unsigned long long>(a.c25),
                 static_cast<unsigned long long>(a.c50),
                 static_cast<unsigned long long>(a.c100),
                 static_cast<unsigned long long>(a.idleFrames),
                 static_cast<unsigned long long>(a.pushed),
                 a.advanced ? "  (状态迁移)" : tag);
}

// ===========================================================================
//  夹具：真实装配 + 逐拍推进
// ===========================================================================

/// 装配结果。声明顺序 = 构造顺序，析构逆序 ⇒ init 先于 ctx 销毁
/// （`SystemInitializer` 持有 `ApplicationContext&`）。
struct Harness
{
    ApplicationContext                      ctx;
    std::unique_ptr<SystemInitializer>      init;

    /// 用真实 7 个 yaml 完整初始化。**不预注入任何桩** ——
    /// `initialize()` 无条件覆盖每个 `ctx.X`，预注入的对象必被销毁。
    ///
    /// @param configDir 配置目录；默认即场景 A/B/D 共用的 "config"。
    ///        仿真场景（场景 S）传入由仓库 yaml 改写出的独立目录 ——
    ///        它不能覆盖工作目录里的 "config"，否则场景 A 的
    ///        "models/ 不存在 ⇒ POSE_SOLVE 失败" 前提会被静默改掉。
    bool build(const std::string& configDir = "config")
    {
        init = std::make_unique<SystemInitializer>(ctx);
        return init->initialize(configDir) && init->ready() && ctx.controller != nullptr;
    }
};

ChannelCounts countsOf(const Harness& h)
{
    ChannelCounts c;
    c.c25  = std::static_pointer_cast<VirtualCameraBackend>(h.ctx.backend25)->frameCount();
    c.c50  = std::static_pointer_cast<VirtualCameraBackend>(h.ctx.backend50)->frameCount();
    c.c100 = std::static_pointer_cast<VirtualCameraBackend>(h.ctx.backend100)->frameCount();
    return c;
}

/// 推进一步并记账。时刻用**真实单调时钟**（与生产路径同一个来源）。
TickAccount step(Harness& h)
{
    const ChannelCounts before = countsOf(h);
    const uint64_t      idleBefore = h.init->summary().idleFrames;
    const uint64_t      pushBefore = h.ctx.preview->queue().pushedCount();

    TickAccount a;
    a.start = h.ctx.controller->state();

    h.init->tick(aircraft::data::monotonicNowNs());

    const ChannelCounts after = countsOf(h);
    a.end      = h.ctx.controller->state();
    a.advanced = (a.end != a.start);
    a.c25      = after.c25 - before.c25;
    a.c50      = after.c50 - before.c50;
    a.c100     = after.c100 - before.c100;
    a.idleFrames = h.init->summary().idleFrames - idleBefore;
    a.pushed     = h.ctx.preview->queue().pushedCount() - pushBefore;
    return a;
}

/// 逐拍推进到 `S::FAILED`（或 `S::COMPLETE`），返回全部账目。
/// 上限 200 拍是**测试的护栏**，不是生产约束：机型库缺失时 POSE_SOLVE
/// 必然失败，本函数断言的是"必然终止"这一形态，具体拍数不作承诺。
std::vector<TickAccount> runUntilTerminal(Harness& h, int firstIndex = 0)
{
    std::vector<TickAccount> accounts;
    for (int i = 0; i < 200; ++i)
    {
        const TickAccount a = step(h);
        printAccount(a, firstIndex + i);
        accounts.push_back(a);
        if (a.end == S::FAILED || a.end == S::COMPLETE)
        {
            break;
        }
    }
    return accounts;
}

/// 第一拍"开始时处于状态 s"的账目；未出现则返回 nullptr。
const TickAccount* firstStartingIn(const std::vector<TickAccount>& v, MeasurementState s)
{
    for (const TickAccount& a : v)
    {
        if (a.start == s)
        {
            return &a;
        }
    }
    return nullptr;
}

// ===========================================================================
//  场景 A：逐状态采集归属
// ===========================================================================

TEST(SystemInitializerTest, 场景A_逐状态采集归属与空闲补帧)
{
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    Harness h;
    ASSERT_TRUE(h.build()) << "真实装配失败：" << h.init->errorText();

    std::fprintf(stderr,
                 "\n[场景A] 真实装配完成：相机数=%d 启用通道=%d 标定=%d 机型库=%d "
                 "降级=%d；逐拍账如下（Δ 为相对上一拍的增量）\n",
                 h.init->summary().cameraCount, h.init->summary().enabledChannels,
                 static_cast<int>(h.init->summary().calibrationLoaded),
                 static_cast<int>(h.init->summary().modelsLoaded),
                 static_cast<int>(h.init->summary().degradedAtStart));

    // ---- ① IDLE 拍：app 的空闲补帧必须真的在跑（R04 的正向） ----
    {
        const TickAccount a = step(h);
        printAccount(a, 0);

        EXPECT_EQ(a.start, S::IDLE);
        EXPECT_EQ(a.end, S::IDLE) << "未启动测量时状态机不得自行前进";
        EXPECT_EQ(a.c25, 1u)  << "空闲态应补采一帧";
        EXPECT_EQ(a.c50, 1u);
        EXPECT_EQ(a.c100, 1u);
        EXPECT_EQ(a.idleFrames, 1u) << "summary().idleFrames 未递增";
        EXPECT_EQ(a.pushed, 1u) << "补采的帧没有进预览队列";
    }

    const int captureFrameCount = h.ctx.config->measurement().captureFrameCount;

    ASSERT_TRUE(h.init->startMeasurement()) << "测量启动失败："
                                            << h.ctx.controller->lastError().message;

    const std::vector<TickAccount> accounts = runUntilTerminal(h, 1);

    const bool terminal = !accounts.empty() && accounts.back().end == S::FAILED;
    ASSERT_TRUE(terminal)
        << "真实装配下测量未在 200 拍内终止于 FAILED。"
           "机型库缺失（models/ 为空）时 POSE_SOLVE 必失败 —— 这是本场景的"
           "既定前提，见 SystemInitializer::buildAlgorithm 的说明。";

    // ---- ② 逐状态的采集归属 ----
    //
    // 断言按"该拍**开始**时的状态"定位，不按绝对拍号：
    // SEARCH / STABILIZE 都可能跨多拍，用拍号定位会把测试写死在一次运行上。
    struct Expectation
    {
        MeasurementState state;
        uint64_t         delta25;   ///< 该路增量（三路相同，见 capture() 的语义）
        const char*      why;
    };

    const Expectation expectations[] = {
        {S::SEARCH,         1, "stepSearch：一次三相机采集 + CAM25 检测"},
        {S::TARGET_FOUND,   0, "复用 SEARCH 的那一帧（lastFrame_），无 acquire()"},
        {S::ALIGN,          1, "stepAlign：重新检测（§5.4）"},
        {S::STABILIZE,      1, "stepStabilize：稳定窗口内每拍采一帧"},
        {S::MEASURE_SELECT, 1, "stepMeasureSelect：重新采集后评分"},
        {S::CAPTURE, static_cast<uint64_t>(captureFrameCount),
                            "stepCapture：一拍内连采 captureFrameCount 帧"},
        {S::POSE_SOLVE,     0, "stepPoseSolve 无 acquire()"},
    };

    for (const Expectation& e : expectations)
    {
        const TickAccount* a = firstStartingIn(accounts, e.state);
        ASSERT_NE(a, nullptr)
            << "真实装配下没有出现 " << stateName(e.state) << " 拍";
        EXPECT_EQ(a->c25, e.delta25)
            << stateName(e.state) << " 拍 ΔCAM25 不符（" << e.why << "）";
        EXPECT_EQ(a->c50, e.delta25)
            << stateName(e.state) << " 拍 ΔCAM50 不符（" << e.why << "）";
        EXPECT_EQ(a->c100, e.delta25)
            << stateName(e.state) << " 拍 ΔCAM100 不符（" << e.why << "）";
        EXPECT_EQ(a->idleFrames, 0u)
            << stateName(e.state) << " 拍不应发生空闲补帧（活动态由控制器采集）";
    }

    // CAPTURE 那一拍若同时补了预览帧，采集增量会是 captureFrameCount + 1 ——
    // 上面的 EXPECT_EQ 已经挡住"多采"，这里的 `pushed` 断言挡住"多推"。
    //
    // ⚠ 预览队列有**两个**生产者，不能用它区分是谁推的：
    //   · MeasurementController 每次 acquire() 都通知一次预览（活动态路径）；
    //   · SystemInitializer::pumpIdlePreview 空闲态推一帧。
    //   故 `pushed == 采集次数` 是正常状态；空闲补帧的**唯一**判据是
    //   summary().idleFrames（它只由 app 的空闲路径递增）。
    {
        const TickAccount* cap = firstStartingIn(accounts, S::CAPTURE);
        ASSERT_NE(cap, nullptr);
        EXPECT_EQ(cap->pushed, static_cast<uint64_t>(captureFrameCount))
            << "CAPTURE 拍的预览推送数应等于采集帧数（控制器逐帧通知），"
               "多出来的那一次就是空闲补帧";
    }

    // ---- ③ 转入终态那一拍：不补帧（R04 的边界，在本场景的弱形式） ----
    //
    // 本场景的终态由 POSE_SOLVE 的失败带来（该拍本身不采集），
    // 故增量为 0；"当拍采集成功、当拍转终态"的强形式见场景 B。
    {
        const TickAccount& term = accounts.back();
        std::fprintf(stderr,
                     "[场景A] 终止拍：%s -> %s，终态码=%d（%s）\n",
                     stateName(term.start), stateName(term.end),
                     h.ctx.controller->lastError().code,
                     h.ctx.controller->lastError().message.c_str());

        EXPECT_EQ(h.ctx.controller->state(), S::FAILED);
        EXPECT_NE(h.ctx.controller->lastError().code, 0)
            << "FAILED 必须携带非零错误码（ENG-09 §5.27）";
        EXPECT_EQ(term.idleFrames, 0u)
            << "转入终态当拍不得补空闲帧（R04：下一拍才恢复）";

        // 下一拍：空闲预览恢复。这条与上一条成对，缺任一条都会让
        // `!stateAdvanced` 的两种坏写法（永不补帧 / 总是补帧）之一逃过。
        const TickAccount next = step(h);
        printAccount(next, static_cast<int>(accounts.size()) + 1);
        EXPECT_EQ(next.start, S::FAILED);
        EXPECT_EQ(next.c25, 1u) << "终态下一拍应恢复空闲补帧";
        EXPECT_EQ(next.idleFrames, 1u);
    }
}

// ===========================================================================
//  场景 B：ALIGN 越程 —— 当拍采集成功、当拍 FAILED
// ===========================================================================

TEST(SystemInitializerTest, 场景B_ALIGN越程当拍FAILED且当拍不补帧)
{
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    Harness h;
    ASSERT_TRUE(h.build()) << "真实装配失败：" << h.init->errorText();

    const aircraft::data::TurntableConfig& tcfg = h.ctx.config->turntable();

    // ---- 摆场景（全部用既有公开 API）----
    //
    // ① 方位角摆到行程上限。必须**在 initialize() 之后**设：VirtualTurntable
    //    的 initialize() 会把角度置为行程中点。
    // ② CAM50 的目标向右偏移，超过居中阈值。ALIGN 用的就是 CAM50
    //    （PreviewManager::mapStateToCamera 的 ALIGN → CAM50）。
    ASSERT_GT(tcfg.azimuthMax, tcfg.azimuthMin) << "转台行程未配置，越程判据无法成立";
    const double offsetPx = tcfg.centerThreshold + 50.0;

    // ApplicationContext::turntable 自 C-22 起是接口指针；
    // 默认协议 "sdk" 仍装配 VirtualTurntable，这里校验这一前提。
    auto* virtualTurntable =
        dynamic_cast<VirtualTurntable*>(h.ctx.turntable.get());
    ASSERT_NE(virtualTurntable, nullptr)
        << "默认配置应装配 VirtualTurntable（protocol=sdk）";
    virtualTurntable->setInitialAngles(tcfg.azimuthMax, 0.0);
    auto cam50 = std::static_pointer_cast<VirtualCameraBackend>(h.ctx.backend50);
    ASSERT_NE(cam50, nullptr);
    cam50->setTargetPixelOffset(offsetPx, 0.0);

    std::fprintf(stderr,
                 "\n[场景B] 起始方位=%g°（上限 %g°），CAM50 目标偏移 %g px"
                 "（居中阈值 %g px）；逐拍账如下\n",
                 tcfg.azimuthMax, tcfg.azimuthMax, offsetPx, tcfg.centerThreshold);

    ASSERT_TRUE(h.init->startMeasurement()) << "测量启动失败："
                                            << h.ctx.controller->lastError().message;

    const std::vector<TickAccount> accounts = runUntilTerminal(h, 1);

    ASSERT_FALSE(accounts.empty());
    ASSERT_EQ(h.ctx.controller->state(), S::FAILED)
        << "ALIGN 越程未使测量终止。命令的符号约定见 AlignmentController.cpp"
           " 的说明（方位：目标在中心右侧 ⇒ 方位角增大），"
           "若实测方向相反，需按评审预案改用负向起始角。";

    // ---- 关键断言：终止发生在**从 ALIGN 出发的那一拍**，且该拍采集成功 ----
    const TickAccount& term = accounts.back();
    ASSERT_EQ(term.start, S::ALIGN)
        << "终止拍起始于 " << stateName(term.start) << "，不是 ALIGN";
    EXPECT_TRUE(term.advanced);

    EXPECT_EQ(term.c25, 1u)
        << "越程当拍应已完成一次采集（acquire 成功才可能走到命令计算）";
    EXPECT_EQ(term.c50, 1u);
    EXPECT_EQ(term.c100, 1u);

    // 这就是 R04 的边界：本拍已经采集成功、并当拍转入终态，
    // 若 `tick()` 少了 `!stateAdvanced`，此处会是 2（多补一帧）。
    EXPECT_EQ(term.idleFrames, 0u) << "转入终态当拍不得补空闲帧";

    // 本拍的这一次推送来自控制器自己的 acquire()（活动态路径），
    // **不是**空闲补帧 —— 空闲补帧的判据是上面那一条 idleFrames。
    // 去掉 `!stateAdvanced` 后这里会是 2，与 c25 一起构成两道独立的拦截。
    EXPECT_EQ(term.pushed, 1u);

    const aircraft::data::ErrorInfo e = h.ctx.controller->lastError();
    EXPECT_EQ(e.code, aircraft::data::kErrTurntableOverTravel)
        << "终态码应为 2002（能力边界），实为 " << e.code << "（"
        << aircraft::data::errorCodeName(e.code) << "）：" << e.message;
    std::fprintf(stderr, "[场景B] 终态 2002 详情：%s\n", e.message.c_str());

    // ---- 下一拍：空闲预览恢复（"转入终态当拍不补帧，下一拍恢复"） ----
    const TickAccount next = step(h);
    printAccount(next, static_cast<int>(accounts.size()) + 1);
    EXPECT_EQ(next.start, S::FAILED);
    EXPECT_EQ(next.c25, 1u) << "终态下一拍应恢复空闲补帧";
    EXPECT_EQ(next.c50, 1u);
    EXPECT_EQ(next.c100, 1u);
    EXPECT_EQ(next.idleFrames, 1u);
}

// ===========================================================================
//  场景 C：取帧预算的两个配置键（011-A1 §3）
//
//  ---- 这一组证明了什么 ----
//
//  `grab_timeout_ms` / `grab_group_budget_ns` 是 `capture()` 里"三者取小"
//  的两个直接来源（见 MultiCameraManager::capture）。若它们只被
//  `MeasurementConfig` 默认值填充、而 yaml 里的键根本没被解析，症状是
//  **静默**的：改配置文件不生效，一切照默认值跑，没有任何一处报错。
//  故这里断言的不是"结构体字段存在"，而是"**yaml 里的字面值真的进来了**"。
//
//  ---- 为什么不走 SystemInitializer ----
//
//  配置加载是 **infrastructure** 层的事，`ConfigManager::load(dir)` 本身就
//  接受目录参数 ⇒ **不需要 chdir、不需要 7 个文件之外的任何东西**，
//  故本组用绝对路径直接加载，与场景 A/B 的临时工作目录互不干扰
//  （加载失败与成功都不改变进程 CWD）。
// ===========================================================================

/// 把一个 yaml 原文里的 `from` 换成 `to`，并**断言确实换到了**。
///
/// ⚠ 换不到就返回空串（调用方据此判失败），**不**静默返回原文：
///    若将来 yaml 里的写法变了，这里的替换会变成空操作，而"改坏的配置"
///    仍是好配置 ⇒ 负例会变成"加载居然成功了"这种看不出原因的失败。
///    宁可在此处显式报"未命中"。
std::string replaceOnce(const std::string& text, const std::string& from,
                        const std::string& to)
{
    const std::size_t pos = text.find(from);
    if (pos == std::string::npos)
    {
        return std::string();
    }
    std::string out = text;
    out.replace(pos, from.size(), to);
    return out;
}

std::string readFile(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        return std::string();
    }
    return std::string((std::istreambuf_iterator<char>(in)),
                       std::istreambuf_iterator<char>());
}

/// 建一个临时目录、把仓库 `config/` 的 7 个 yaml 拷进去，再用 `overrideText`
/// 覆盖其中的 `overrideFile`。返回目录路径（空串 = 失败）。
///
/// ⚠ **不 chdir**：`ConfigManager::load()` 收目录参数，故负例不需要把整个
///    进程搬进临时目录 —— 那会与场景 A/B 的工作目录语义纠缠在一起。
///    `SystemInitializer::initialize(dir)` 同样收目录，而 yaml 里的相对路径
///    （log_dir / output_dir / model_dir）按**进程 CWD** 解析 —— 那是本套件
///    的临时工作目录（`prepareWorkspace`），故不必再 chdir 一次。
std::string makeConfigVariantForFile(const std::string& overrideFile,
                                     const std::string& overrideText,
                                     std::string&       err,
                                     const std::string& secondFile = std::string(),
                                     const std::string& secondText = std::string())
{
    char        tmpl[] = "/tmp/aps_cfg_XXXXXX";
    char* const dir    = ::mkdtemp(tmpl);
    if (dir == nullptr)
    {
        err = "mkdtemp 失败";
        return std::string();
    }
    const std::string tmp = dir;
    for (const char* const f : kConfigFiles)
    {
        const std::string src = std::string(APS_SOURCE_DIR) + "/config/" + f;
        const std::string dst = tmp + "/" + f;
        if (f == overrideFile)
        {
            std::ofstream out(dst, std::ios::binary);
            out << overrideText;
            if (!out.good())
            {
                err = "写 " + overrideFile + " 失败：" + dst;
                return std::string();
            }
            continue;
        }
        if (!secondFile.empty() && f == secondFile)
        {
            // 转台用例不能带上仓库 camera.yaml 里的现场接线（CAM25=imv）：
            // 那会让启动在相机阶段就失败，永远走不到转台装配。
            std::ofstream out(dst, std::ios::binary);
            out << secondText;
            if (!out.good())
            {
                err = "写 " + secondFile + " 失败：" + dst;
                return std::string();
            }
            continue;
        }
        if (!copyFile(src, dst))
        {
            err = "拷贝失败：" + src;
            return std::string();
        }
    }
    return tmp;
}

std::string makeConfigVariant(const std::string& measurement, std::string& err)
{
    return makeConfigVariantForFile("measurement.yaml", measurement, err);
}

/// 仓库 `config/` 下某个 yaml 的原文（负例的替换底本）。
std::string repoYaml(const char* name)
{
    return readFile(std::string(APS_SOURCE_DIR) + "/config/" + name);
}

/// 仓库 measurement.yaml 的原文（负例的替换底本）。
std::string repoMeasurementYaml()
{
    return repoYaml("measurement.yaml");
}

/// 把**某一通道记录内**的一整行替换成 `line`（`line` 含缩进；空串 = 删掉该行）。
///
/// ⚠ 为什么必须有"记录作用域"，不能对整份 yaml 做"第一处匹配"
///   （2026-09-28 实机接入后暴露）：仓库的 `camera.yaml` 里 CAM25 已改为
///   `backend: "imv"`，于是
///   `replaceOnce(repoYaml("camera.yaml"), "backend: \"virtual\"", "imv")`
///   命中的是 **CAM50** —— 用例的意图是"把 cam25 改成真实后端"，实际改的是
///   cam50，而断言仍然通过（它只看整体启动成败），并且**真的去开了物理设备**
///   （实测输出："已枚举到 1 台设备"）。文本位置与"第几路"之间本来就没有
///   约定，只是因为历史上三路同形而**看起来**成立。
///   这类"改错了对象但仍然通过"的用例比没有用例更坏：它占着一条断言，
///   却证明的是另一件事。
///
/// @return 空串 = 锚点或键未命中（调用方据此判失败，不静默返回原文）。
std::string replaceInCameraRecord(const std::string& text,
                                  const std::string& cameraId,
                                  const std::string& key,
                                  const std::string& line)
{
    const std::string anchor = "id: \"" + cameraId + "\"";
    const std::size_t begin  = text.find(anchor);
    if (begin == std::string::npos)
    {
        return std::string();
    }
    // 列表项标记 "   - " 即下一条记录的开始（本文件的三条记录同缩进）。
    const std::size_t next = text.find("\n   -", begin);
    const std::size_t end  = (next == std::string::npos) ? text.size() : next;

    const std::string record = text.substr(begin, end - begin);
    const std::size_t k      = record.find(key + ":");
    if (k == std::string::npos)
    {
        return std::string();
    }
    const std::size_t lineBeg = record.rfind('\n', k);
    std::size_t       lineEnd = record.find('\n', k);
    if (lineEnd == std::string::npos)
    {
        lineEnd = record.size();
    }
    std::string replaced = record;
    replaced.replace(lineBeg, lineEnd - lineBeg,
                     line.empty() ? std::string() : ("\n" + line));

    std::string out = text;
    out.replace(begin, record.size(), replaced);
    return out;
}

/// 由仓库 `camera.yaml` 得到一份**三路全虚拟**的底本。
///
/// ⚠ 用例必须**显式**把三路压回 `virtual`，不能直接拿仓库配置当"虚拟底本"：
///   仓库配置在 2026-09-28 之后承载的是**现场接线的真实选择**（CAM25 = `imv`
///   + 实际序列号）。直接用它跑的用例会去打开物理设备 —— 于是同一份代码
///   在有设备与没设备的机器上表现不同，且"设备未连接"会让用例以**与它要
///   证明的事无关**的原因失败（实测：`IMV_StartGrabbing 返回 -118`）。
///   测试要的底本是一个**与现场接线无关**的确定状态，本函数给的就是它。
std::string allVirtualCameraYaml()
{
    std::string text = repoYaml("camera.yaml");
    const char* const ids[] = {"cam25", "cam50", "cam100"};
    for (const char* const id : ids)
    {
        const std::string a = replaceInCameraRecord(text, id, "backend",
                                                    "      backend: \"virtual\"");
        if (a.empty()) { return std::string(); }
        const std::string b = replaceInCameraRecord(a, id, "serial",
                                                    "      serial: \"\"");
        if (b.empty()) { return std::string(); }
        text = b;
    }
    return text;
}

TEST(SystemInitializerTest, 场景C_取帧预算键被读到)
{
    // ---- 判别式用例：两个键都改成"既非仓库现值、也非结构体默认值"的数 ----
    //
    // ⚠ 为什么不能直接断仓库现值：
    //   · `grab_group_budget_ns` 的仓库现值（3.0e8）与 `MeasurementConfig`
    //     的默认值**完全相同** ⇒ "解析了"与"压根没解析、全程吃默认值"
    //     两种情形给出同一个断言结果，这正是一个看不出差别的用例；
    //   · `grab_timeout_ms` 的仓库现值自 2026-09-28 起是 500（实机实测后
    //     由 100 改，见 config/measurement.yaml），已**不再**等于默认值 100
    //     —— 但它仍取第三个数 250 来断：若只把文字从 500 改成 500，
    //     "解析器把 grab_timeout_ms 写死成 500"这种实现同样能通过。
    //     取一个既非现值、也非默认值的数，三种来源才互不相同。
    std::string text = replaceOnce(repoMeasurementYaml(), "grab_timeout_ms: 500",
                                   "grab_timeout_ms: 250");
    ASSERT_FALSE(text.empty()) << "未命中 grab_timeout_ms 那一行，本用例的底本已失效";
    text = replaceOnce(text, "grab_group_budget_ns: 3.0e8", "grab_group_budget_ns: 4.0e8");
    ASSERT_FALSE(text.empty()) << "未命中 grab_group_budget_ns 那一行，本用例的底本已失效";

    std::string err;
    const std::string dir = makeConfigVariant(text, err);
    ASSERT_FALSE(dir.empty()) << err;

    aircraft::infrastructure::ConfigManager cfg;
    ASSERT_TRUE(cfg.load(dir)) << "改动后的配置本应加载成功";

    const aircraft::data::MeasurementConfig& m = cfg.measurement();
    EXPECT_EQ(m.grabTimeoutMs, 250) << "grab_timeout_ms 未按 yaml 的字面值解析";
    EXPECT_EQ(m.grabGroupBudgetNs, 400000000ULL)
        << "grab_group_budget_ns（4.0e8）未按 yaml 的字面值解析";
}

TEST(SystemInitializerTest, 场景C_仓库现值的零余量事实必须可见)
{
    // 本用例**不**用仓库现值证明"解析了"（那是上一条用例的事）；它证明的是
    // 另一件事：**当前这份配置的余量事实不得静默**。
    // 5 帧 × 3 路 × 500 ms = 7500 ms ≥ capture_timeout_ns(1500 ms)，
    // 且未计入复制/格式转换/评分 ⇒ CAPTURE 在真实相机上可能被时限截断。
    // ⚠ 2026-09-28：`grab_timeout_ms` 由 100 改 500 后，这项**从"恰好占满"
    //    变成"超出 5 倍"**（原 5 × 3 × 100 = 1500 ms **等于**上限）。余量问题
    //    因此更严重，告警必须继续出现 —— 若某次改动把这条告警弄没了，
    //    本用例就是唯一会响的地方。
    // ⚠ 只警告、不判启动失败 —— 取值余量属《待裁决问题汇总》Q-D2，
    //   本批不自行放宽冻结值。
    aircraft::infrastructure::ConfigManager cfg;
    ASSERT_TRUE(cfg.load(std::string(APS_SOURCE_DIR) + "/config"))
        << "仓库 config/ 加载失败";

    EXPECT_EQ(cfg.measurement().grabTimeoutMs, 500);
    EXPECT_EQ(cfg.measurement().grabGroupBudgetNs, 300000000ULL);

    bool sawWarning = false;
    for (const std::string& w : cfg.warnings())
    {
        if (w.find("取帧预算无余量") != std::string::npos)
        {
            sawWarning = true;
        }
    }
    EXPECT_TRUE(sawWarning) << "取帧预算已无余量，却没有给出告警（该事实不得静默）";
}

TEST(SystemInitializerTest, 场景C_取帧预算余量充足时不告警)
{
    // ---- 正对照：告警是**有条件**的，不是一句永远都印的话 ----
    //
    // 只把 capture_timeout_ns 放宽到 10 s（另两项不动，
    // 5 × 3 × 500 = 7500 ms < 10000 ms ⇒ 有余量），告警必须消失。
    // 缺了这条用例，把告警写成"无条件 push"也能让上一条用例通过。
    // ⚠ 放宽到的这个数**跟着 grab_timeout_ms 走**：2026-09-28 由 100 改成
    //    500 后，原来够用的 3.0 s 变成不够（7500 ms ≥ 3000 ms），本用例
    //    当场转红 —— 这正是"正对照必须真的留出余量"在起作用，不是误报。
    //    10 s 仍 < task_timeout_ns(60 s)，配置合法性不受影响。
    const std::string text = replaceOnce(repoMeasurementYaml(),
                                         "capture_timeout_ns: 1.5e9",
                                         "capture_timeout_ns: 1.0e10");
    ASSERT_FALSE(text.empty()) << "未命中 capture_timeout_ns 那一行，本用例的底本已失效";

    std::string err;
    const std::string dir = makeConfigVariant(text, err);
    ASSERT_FALSE(dir.empty()) << err;

    aircraft::infrastructure::ConfigManager cfg;
    ASSERT_TRUE(cfg.load(dir)) << "余量充足的那份配置本应加载成功";

    for (const std::string& w : cfg.warnings())
    {
        EXPECT_EQ(w.find("取帧预算无余量"), std::string::npos)
            << "余量充足却报了余量告警：" << w;
    }
    // 该项放宽后仍须读回放宽后的值（防止"替换了却没生效"被误判成通过）。
    EXPECT_EQ(cfg.measurement().captureTimeoutNs, 10000000000ULL);
}

TEST(SystemInitializerTest, 场景C_grab_timeout_ms为零即启动失败)
{
    // `IMV_GetFrame` 对 timeoutMS = 0 的语义在 SDK 中**未文档化**
    // （ENG-09 V2.3 §2.5），项目不定义它、后端会拒绝 0 ⇒ 配置层必须拦。
    // 若此处只是"取默认值"或"静默接受"，故障会推迟到第一次取帧才出现，
    // 且表现为"相机取不到帧"这种把人引向设备侧的假象。
    const std::string text =
        replaceOnce(repoMeasurementYaml(), "grab_timeout_ms: 500", "grab_timeout_ms: 0");
    ASSERT_FALSE(text.empty()) << "未命中 grab_timeout_ms 那一行，本用例的底本已失效";

    std::string err;
    const std::string dir = makeConfigVariant(text, err);
    ASSERT_FALSE(dir.empty()) << err;

    aircraft::infrastructure::ConfigManager cfg;
    EXPECT_FALSE(cfg.load(dir)) << "grab_timeout_ms = 0 应判为取值越界、启动失败";

    bool named = false;
    for (const std::string& e : cfg.errors())
    {
        if (e.find("grab_timeout_ms") != std::string::npos)
        {
            named = true;
        }
    }
    EXPECT_TRUE(named) << "失败原因未点名 grab_timeout_ms";
}

TEST(SystemInitializerTest, 场景C_grab_group_budget_ns为零即启动失败)
{
    // 组预算为 0 ⇒ 每路都判"预算耗尽"、一次 SDK 调用都不发生，
    // 表现为"三路全部取帧失败"，而实际是配置把预算配没了。
    const std::string text = replaceOnce(repoMeasurementYaml(),
                                         "grab_group_budget_ns: 3.0e8",
                                         "grab_group_budget_ns: 0");
    ASSERT_FALSE(text.empty()) << "未命中 grab_group_budget_ns 那一行，本用例的底本已失效";

    std::string err;
    const std::string dir = makeConfigVariant(text, err);
    ASSERT_FALSE(dir.empty()) << err;

    aircraft::infrastructure::ConfigManager cfg;
    EXPECT_FALSE(cfg.load(dir)) << "grab_group_budget_ns = 0 应判为取值越界、启动失败";

    bool named = false;
    for (const std::string& e : cfg.errors())
    {
        if (e.find("grab_group_budget_ns") != std::string::npos)
        {
            named = true;
        }
    }
    EXPECT_TRUE(named) << "失败原因未点名 grab_group_budget_ns";
}

// ===========================================================================
//  场景 D：显式装配、启动失败边界与生命周期归属（011-A1 §4.4）
//
//  ⚠ 与场景 A/B 的关键差别：本组**故意让启动失败**，故每个用例自建
//    `ApplicationContext` 与 `SystemInitializer`，**不**复用 `Harness`
//    （它的 `build()` 要求 `ready()` 为真）。失败路径上 `ctx.controller`
//    等成员保持为空，正是要断言的事。
// ===========================================================================

/// 生命周期序列拼成一行（断言失败时人能一眼看出多/少了哪一次调用）。
std::string joinLog(const std::vector<std::string>& log)
{
    std::string s;
    for (const std::string& e : log)
    {
        if (!s.empty())
        {
            s += " -> ";
        }
        s += e;
    }
    return s.empty() ? std::string("（空）") : s;
}

/// 一路后端的生命周期序列。
///
/// ⚠ `isVirtual == false` 与"虚拟后端一个调用都没收到"必须分开表达：
///    前者才是"这一路压根不是虚拟后端"（有 SDK 的构建下真的造出了
///    `ImvCameraBackend`，它没有这份记录），后者只在"造出来了但一次没调"
///    时才成立。混成一个空序列会让"没记录"看起来像"没调用"。
struct BackendLog
{
    bool                     isVirtual = false;
    std::vector<std::string> entries;
};

BackendLog backendLogOf(const std::shared_ptr<aircraft::device::ICameraBackend>& b)
{
    BackendLog r;
    const std::shared_ptr<VirtualCameraBackend> vb =
        std::dynamic_pointer_cast<VirtualCameraBackend>(b);
    if (vb)
    {
        r.isVirtual = true;
        r.entries   = vb->lifecycleLog();
    }
    return r;
}

/// cam25 显式配置为真实后端的配置目录（其余两路仍为虚拟）。
///
/// `serial` 必须一并给出：`backend: "imv"` 而 serial 为空会在
/// **ConfigManager** 处被更早拦下（"按序列号绑定设备"那条规则），
/// 而本组要测的是**装配层**的失败边界，不是配置校验。
/// 换言之：这份配置本身是合法的，失败必须来自"设备打不开"。
///
/// ⚠ 序列号用**本机不存在的测试值**，且底本来自 `allVirtualCameraYaml()`：
///   这两个条件各管一半 —— 前者保证用例不会去开现场那台相机（本机若真接了
///   设备，用它自己的序列号就会"意外成功"，而本组用例断言的是**失败**），
///   后者保证"三路里只有 cam25 是 imv"这件事由本用例自己决定，而不是
///   取决于仓库配置当前怎么接线。
std::string imvCam25ConfigDir(std::string& err)
{
    const std::string base = allVirtualCameraYaml();
    if (base.empty())
    {
        err = "未能在仓库 camera.yaml 里定位三路记录（id/backend/serial 写法已变），"
              "本用例的底本已失效";
        return std::string();
    }
    std::string text = replaceInCameraRecord(base, "cam25", "backend",
                                             "      backend: \"imv\"");
    if (text.empty())
    {
        err = "未命中 cam25 的 backend 行，本用例的底本已失效";
        return std::string();
    }
    text = replaceInCameraRecord(text, "cam25", "serial",
                                 "      serial: \"SN-CAM25-TEST-0001\"");
    if (text.empty())
    {
        err = "未命中 cam25 的 serial 行，本用例的底本已失效";
        return std::string();
    }
    return makeConfigVariantForFile("camera.yaml", text, err);
}

TEST(SystemInitializerTest, 场景D_全虚拟启动时每个后端恰好初始化一次)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    Harness h;
    ASSERT_TRUE(h.build()) << "全虚拟配置本应启动成功：" << h.init->errorText();

    struct Row
    {
        const char*                                    name;
        std::shared_ptr<aircraft::device::ICameraBackend> b;
    };
    const Row rows[] = {{"CAM25", h.ctx.backend25},
                        {"CAM50", h.ctx.backend50},
                        {"CAM100", h.ctx.backend100}};

    for (const Row& r : rows)
    {
        const BackendLog l = backendLogOf(r.b);

        // ---- 正对照：三路都**确实是**虚拟后端 ----
        // 缺了它，"日志里恰好一次 initialize"会在"这一路根本不是虚拟后端、
        // 压根没有日志"时也成立 —— 而"按配置装配"正是本用例要保的东西。
        ASSERT_TRUE(l.isVirtual)
            << r.name << " 在 camera.yaml 里配置为 virtual，却不是 VirtualCameraBackend";

        // ---- 判别式断言：`initialize` **恰好一次** ----
        // 本批从装配层修掉的缺陷是"同一个后端被初始化两遍"
        // （`buildDevices()` 先逐路 `initialize()`，`initializeAll()` 又各自
        //  初始化一次）。虚拟后端上这完全幂等、终态相同、除本记录外没有
        //  任何痕迹 —— 故只有调用序列能把"修好了"与"缺陷回来了"分开。
        EXPECT_EQ(joinLog(l.entries), "initialize -> start")
            << r.name << " 的生命周期序列不是「initialize -> start」："
                        "`initialize` 出现两次即双重初始化回来了";
    }

    // 装配摘要的数由实际对象推出（不写常数）：三路都造出来了。
    EXPECT_EQ(h.init->summary().cameraCount, 3);
}

TEST(SystemInitializerTest, 场景D_显式请求真实后端不可用即启动失败且不静默换虚拟)
{
    std::string err;
    const std::string dir = imvCam25ConfigDir(err);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    EXPECT_FALSE(init.initialize(dir)) << "真实相机打不开却判了启动成功";
    EXPECT_FALSE(init.ready());

    const std::string& e = init.errorText();
    EXPECT_NE(e.find("cam25"), std::string::npos) << "失败原因未点名通道：" << e;
    EXPECT_NE(e.find("imv"), std::string::npos)
        << "失败原因未说明该通道声明的后端是 imv：" << e;

    // 两条失败文本**都可能**出现，且两者都必须让操作者知道下一步该做什么：
    //   · "本次构建未接入 SDK…请用 -DIMV_SDK_ROOT=… 重新配置" —— 动作在**构建**；
    //   · "真实相机接入失败：通道 cam25…未能就绪" —— 动作在**设备与配置**。
    // 只断"失败"而不断"哪一类失败"，这条用例就分不出这两种截然不同的处境。
    //
    // ⚠ 走哪一条**不取决于**本文件是否被垫片编译，而取决于链接进来的
    //    `libdevice_camera.a` 里 `makeRealImvApi()` 是哪个定义（该函数在
    //    `ImvApiReal.cpp` 里由 `APS_HAVE_IMVSDK` 二选一）—— 本机
    //    `third_party/imvsdk/` 存在、库按"已接入 SDK"构建，故实测走的是
    //    **边界那条**（下面的打印把它记在运行记录里，不靠推断）。
    const bool namedSdkMissing = e.find("未接入 SDK") != std::string::npos;
    const bool namedAccessFail = e.find("接入失败") != std::string::npos;
    EXPECT_TRUE(namedSdkMissing || namedAccessFail)
        << "失败原因既没说「未接入 SDK」、也没说「接入失败」，操作者无法据此"
           "判断该改构建还是该查设备："
        << e;

    if (namedAccessFail)
    {
        // 后端**自己知道的原因**必须真的出现在用户可见的文本里
        // （011-A1 §3.2：按序列号匹配不到 ⇒ "把枚举到的型号／序列号
        // 全列进错误信息"；§4.4 的"未取得序列号如实记"）。
        //
        // ⚠ 上一版这里只有"（无附加说明——单路初始化失败的明细见各后端
        //    自身日志）"，而**后端并不写那种日志**：实机失败路径实测
        //    （进程级运行，`backend: imv` + 本机不存在的序列号）除
        //    "未能就绪"之外什么都没有。操作者据此分不清"线序接错"
        //    与"相机没上电"，而枚举结果就在后端手里。故本条断言钉住
        //    "后端 → 装配层 → 用户"这条通路确实接通了。
        EXPECT_NE(e.find("未找到序列号为"), std::string::npos)
            << "后端给出了匹配失败的原因，装配层却没把它带进用户可见文本："
            << e;
        EXPECT_EQ(e.find("无附加说明"), std::string::npos)
            << "失败文本仍在指向一个并不存在的「各后端自身日志」：" << e;

        std::fprintf(stderr,
                     "  〔失败分支〕本次走的是「启动失败边界」：后端**造得出来**，"
                     "初始化拿不到设备后由 buildDevices 判失败。文本：%s\n",
                     e.c_str());
    }
    else
    {
        std::fprintf(stderr,
                     "  〔失败分支〕本次走的是「makeBackend 直接失败」"
                     "（构建未接入 SDK）。文本：%s\n",
                     e.c_str());
    }

    // ---- 不部分启动 ----
    // 设备没就绪就不得把管理器与控制器交出去：`initialize()` 返回 false 之后
    // 任何 `ctx.X` 都不得处于"半装配"状态。
    EXPECT_EQ(ctx.cameras, nullptr) << "启动已判失败，却留下了半装配的相机管理器";
    EXPECT_EQ(ctx.controller, nullptr);

    // ---- 不静默换虚拟（本用例的核心）----
    // 这一路显式声明了 imv，打不开就必须**失败**。若被悄悄换成虚拟后端，
    // 本次启动会看起来成功，而三路图全是合成的 —— 这是本批最要防的失效形态。
    // 垫片与有 SDK 两种构建下 `ctx.backend25` 都非空，故两处断言都成立。
    ASSERT_TRUE(ctx.backend25 != nullptr)
        << "真实后端被造出来过（或压根没造），两种情形都要能看出它**不是**虚拟后端";
    EXPECT_EQ(std::dynamic_pointer_cast<VirtualCameraBackend>(ctx.backend25), nullptr)
        << "显式请求的真实后端不可用时被静默换成了虚拟后端";

    // ---- 设备身份不得从配置复制 ----
    // 没有读到设备时，报告里必须是"未取得"，而**不得**把配置里的目标序列号
    // 抄成实际身份 —— 抄了的话"配置写 A、实际连 B"这类错装配在摘要里
    // 会看起来完全正常（§2.3）。
    const aircraft::data::DeviceIdentity id = ctx.backend25->deviceIdentity();
    EXPECT_FALSE(id.serialNumber.has_value() &&
                 *id.serialNumber == "SN-CAM25-TEST-0001")
        << "后端把配置里的目标序列号当成了实际读到的设备身份";
}

TEST(SystemInitializerTest, 场景D_缺backend键即启动失败)
{
    // 缺键**不得**取默认值：一份没写 backend 的配置若被静默当成 virtual，
    // 那么 SDK 装好、真实相机接上之后，它仍会跑虚拟图而看起来一切正常。
    // 这与"检测到 SDK 就自动切真实"是同一个问题的两面。
    //
    // ⚠ 底本必须先压回"三路全虚拟"再删 cam25 的键：否则在"仓库里 cam25 已是
    //   imv"的今天，第一处 `backend: "virtual"` 是 cam50 的那一行 ——
    //   用例会变成"删掉 cam50 的 backend"，断言照样通过，而它声称在测的
    //   "cam25 缺键"一次也没被真的构造出来。
    const std::string base = allVirtualCameraYaml();
    ASSERT_FALSE(base.empty()) << "未能在仓库 camera.yaml 里定位三路记录";
    std::string text = replaceInCameraRecord(base, "cam25", "backend", "");
    ASSERT_FALSE(text.empty())
        << "未命中 cam25 的 backend 行（缩进或写法已变），本用例的底本已失效";
    EXPECT_EQ(text.find("backend: \"imv\""), std::string::npos)
        << "底本压回全虚拟后仍残留 imv —— 删键用例可能改错了对象";

    std::string err;
    const std::string dir = makeConfigVariantForFile("camera.yaml", text, err);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    EXPECT_FALSE(init.initialize(dir)) << "缺 backend 键却判了启动成功";
    EXPECT_FALSE(init.ready());
    EXPECT_NE(init.errorText().find("backend"), std::string::npos)
        << "失败原因未点名 backend 键：" << init.errorText();

    // 措辞要能把"**没写**这个键"与"写了但写错了"分开（011-A0 的同一条纪律：
    // 两种不可用必须给不同文本）。只断"提到了 backend"，则"缺键"被降格成
    // "类型不对"也照样通过 —— 而前者要改的是配置缺项，后者要改的是取值。
    EXPECT_NE(init.errorText().find("backend 缺失"), std::string::npos)
        << "失败原因未写明是**缺键**：" << init.errorText();
    EXPECT_EQ(ctx.controller, nullptr);
}

TEST(SystemInitializerTest, 场景D_启动失败时已建立资源逐路回滚)
{
    std::string err;
    const std::string dir = imvCam25ConfigDir(err);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);
    ASSERT_FALSE(init.initialize(dir)) << "本用例的前提是启动失败";

    // ---- 回滚要证明的两件事（§3.4）----
    //  ① **停流先于关设备**：两种顺序的终态相同（`READY` vs `UNKNOWN`，
    //     且都幂等），只有调用序列能把它们分开。反过来做（先 `close()`
    //     再 `stop()`）在真实后端上是向一个已销毁的句柄停流。
    //  ② 走到过 `initialize` 的那几路**必须终以 `close`**：
    //     "资源没还回去"不该只体现在一条 warn 里。
    //
    // ⚠ **跨通道的逆序不在本用例里**：`rollbackDevices()` 按
    //    {CAM100, CAM50, CAM25} 逐路停关，但每个后端只能看到**自己**的调用，
    //    "谁先谁后"没有逐对象证据。那一条由该函数的数组字面量固定，
    //    本用例**不声称覆盖**它（不拿"三路各自有序"冒充"跨路有序"）。
    struct Row
    {
        const char*                                    name;
        std::shared_ptr<aircraft::device::ICameraBackend> b;
    };
    const Row rows[] = {{"CAM25", ctx.backend25},
                        {"CAM50", ctx.backend50},
                        {"CAM100", ctx.backend100}};

    int checked = 0;
    for (const Row& r : rows)
    {
        const BackendLog l = backendLogOf(r.b);
        if (!l.isVirtual)
        {
            // 非虚拟后端（有 SDK 的构建下 cam25 就是真实后端）没有这份记录。
            continue;
        }
        ++checked;
        EXPECT_EQ(joinLog(l.entries), "initialize -> stop -> close")
            << r.name << " 的回滚序列不是「initialize -> stop -> close」"
                        "（先 stop 再 close）";
    }

    if (checked == 0)
    {
        std::fprintf(stderr,
                     "  〔本构建下不可达〕没有一路虚拟后端走到过 initialize ⇒ "
                     "rollbackDevices() 未产生可断言的记录；该路径的证据须来自"
                     "后端已装配的那一种构建\n");
    }
    else
    {
        std::fprintf(stderr, "  〔回滚证据〕%d 路虚拟后端记录了 stop -> close 的先后\n",
                     checked);
    }
}

// ===========================================================================
//  场景 D（011-A1 九项缺口 §1）：真实通道**取流启动**失败的失败边界
//
//  为什么不能用真后端来造这个场景：本机没有相机，"`IMV_StartGrabbing` 返回
//  非 0"这条分支在无设备条件下不可复现。故用**注入槽**换上一个可观测的替身，
//  但走的是**真实** `SystemInitializer::initialize()`（不是绕过装配层）。
// ===========================================================================

/// 装配替身：`initialize()` 成功、**`start()` 失败**（带区分性文本）。
///
/// ⚠ 记录**分两层**，这是本组用例的核心判据：
///   · 包装方法调用次数（`stopCalls`／`closeCalls`）—— 允许大于 1
///     （`rollbackDevices()` 与拥有者析构各调一次 `close()` 是**正常**的，
///      真实后端本身也幂等）；
///   · **底层资源**归还次数（`resourceReleases`）—— 必须**恰好一次**。
///     重复释放一个句柄会让 SDK 内部计数错乱，而错误码可能仍是 0，
///     是无声的破坏。把两者混成一个计数，就分不出"多调了一次幂等包装"
///     与"真的释放了两次"。
class StartFailingBackend : public aircraft::device::ICameraBackend
{
public:
    explicit StartFailingBackend(std::string startError)
        : startError_(std::move(startError))
    {
    }

    ~StartFailingBackend() override
    {
        // 兜底关闭（与真实后端的纪律一致）：拥有者正常关闭时不会走到这里。
        (void)close();
    }

    int  initializeCalls  = 0;
    int  startCalls       = 0;
    int  stopCalls        = 0;
    int  closeCalls       = 0;
    int  resourceReleases = 0;   ///< 底层资源**真的**归还了几次（必须恰好 1）
    bool released         = false;

    aircraft::data::OperationResult initialize() override
    {
        ++initializeCalls;
        return ok();
    }

    aircraft::data::OperationResult start() override
    {
        ++startCalls;
        // 取流启动失败：**设备打开了但没在出图** —— 与"打不开设备"是两种
        // 故障，处置不同（一个查线序/占用，一个查带宽/触发/取流权限）。
        lastErrorText_ = startError_;
        return aircraft::data::OperationResult{
            aircraft::data::OpStatus::SdkError,
            aircraft::data::SdkFailure{
                aircraft::data::SdkCall::ImvStartGrabbing, -118},
            std::nullopt};
    }

    aircraft::data::OperationResult stop() override
    {
        ++stopCalls;
        return ok();
    }

    aircraft::data::OperationResult close() override
    {
        ++closeCalls;
        if (!released)
        {
            released = true;
            ++resourceReleases;
        }
        return ok();
    }

    aircraft::data::TriggerModeState triggerModeState() const override
    {
        return aircraft::data::TriggerModeState{};
    }
    aircraft::data::OperationResult setTriggerMode(
        aircraft::data::CameraTriggerMode) override
    {
        return ok();
    }
    aircraft::data::DeviceIdentity deviceIdentity() const override
    {
        return aircraft::data::DeviceIdentity{};
    }
    std::string lastErrorText() const override { return lastErrorText_; }

    aircraft::data::OperationResult triggerSoftware() override { return ok(); }

    aircraft::data::GrabResult grab(aircraft::data::ImageFrame&,
                                   uint32_t) override
    {
        return aircraft::data::GrabResult{
            {aircraft::data::OpStatus::NotStarted, std::nullopt, std::nullopt}};
    }

private:
    static aircraft::data::OperationResult ok()
    {
        return aircraft::data::OperationResult{
            aircraft::data::OpStatus::Ok, std::nullopt, std::nullopt};
    }

    std::string startError_;
    std::string lastErrorText_;
};

TEST(SystemInitializerTest, 场景D_真实CAM25取流启动失败即整次启动失败)
{
    // §1 的主用例：CAM25 配置为 `imv`，替身 `initialize()` 成功、
    // **`start()` 失败**，另两路虚拟成功。
    //
    // ⚠ 这正是此前会**以整次启动成功收场**的形态：`initializeAll()` 与
    //    `startAll()` 的判据都是**合计** ≥2 路，CAM50/100 成功时 CAM25
    //    取流起不来恰好凑够 2 ⇒ `started_ = true; return true` ⇒ 操作者
    //    以为在跑真实的 25mm 采集，而这一路从未交付过一帧。
    std::string err;
    // ⚠ 先落工作目录：本用例**不**经 `Harness`（要自己注入工厂），
    //   而 `system.yaml` 的 log_dir／output_dir 是按**进程 CWD** 解析的
    //   —— 少了这一步，日志与输出会落进仓库目录。
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    const std::string dir = imvCam25ConfigDir(err);
    ASSERT_FALSE(dir.empty()) << err;

    auto cam25 = std::make_shared<StartFailingBackend>(
        "取流启动失败：IMV_StartGrabbing 返回 -118（设备未连接）");

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    // 注入槽（§1 裁决）：**只换 CAM25 那一路**。另两路仍要造出可用的后端
    // —— 返回空会被判装配失败，**不回落**内置工厂，故这里自己造虚拟后端。
    ctx.backendFactory =
        [cam25](const aircraft::data::CameraConfig& cfg, std::string&) {
            if (cfg.role == aircraft::data::CameraRole::CAM25)
            {
                return std::static_pointer_cast<aircraft::device::ICameraBackend>(
                    cam25);
            }
            return std::static_pointer_cast<aircraft::device::ICameraBackend>(
                std::make_shared<VirtualCameraBackend>(cfg));
        };

    EXPECT_FALSE(init.initialize(dir))
        << "CAM25 取流启动失败，整次启动却判了成功";
    EXPECT_FALSE(init.ready());

    const std::string e = init.errorText();
    EXPECT_NE(e.find("cam25"), std::string::npos) << "失败原因未点名通道：" << e;
    // ⚠ 阶段名必须是"取流启动"而**不是**"设备打开"：两者要查的东西不同，
    //    共用一句会让现场去查接错线序，而根因在 `IMV_StartGrabbing`。
    EXPECT_NE(e.find("取流启动"), std::string::npos)
        << "没有说清失败发生在哪个阶段：" << e;
    EXPECT_EQ(e.find("设备打开"), std::string::npos)
        << "设备**是**打开成功的，文本不得把阶段搞错：" << e;
    EXPECT_NE(e.find("IMV_StartGrabbing 返回 -118"), std::string::npos)
        << "后端自己给出的原因没有传到用户可见文本：" << e;

    // ---- 装配层确实用了注入的那个对象，且只初始化过一次 ----
    EXPECT_EQ(ctx.backend25, cam25) << "注入的后端不是实际被装配的对象";
    EXPECT_EQ(cam25->initializeCalls, 1) << "同一个后端被初始化了两遍";
    EXPECT_EQ(cam25->startCalls, 1);

    // ---- 回滚：两层分开看 ----
    EXPECT_EQ(cam25->resourceReleases, 1)
        << "底层资源必须归还，且**恰好一次**（重复释放是无声的破坏）";
    EXPECT_GE(cam25->stopCalls, 1) << "回滚必须先停流再关设备";

    std::fprintf(stderr,
                 "  〔§1 证据〕CAM25 替身：initialize=%d start=%d stop=%d "
                 "close=%d 底层释放=%d；文本：%s\n",
                 cam25->initializeCalls, cam25->startCalls, cam25->stopCalls,
                 cam25->closeCalls, cam25->resourceReleases, e.c_str());
}

TEST(SystemInitializerTest, 场景D_注入工厂返回空即装配失败且不回落内置工厂)
{
    // §1 裁决：注入槽一旦设置，**返回空就是结论**，不得回落内置工厂。
    //
    // ⚠ 为什么这条必须单独测：回落的表现不是崩溃，而是"启动成功、跑的是
    //    内置路径造出来的另一批对象" —— 测试断言的替身与实际跑的对象不是
    //    同一个，于是**在另一条路径上通过**，然后被当成证据。
    //
    // ⚠ 配置故意用**原本能正常启动**的全虚拟配置：若发生了回落，
    //    这次启动会成功、三个后端都非空 ⇒ 本用例当场转红。
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    ctx.backendFactory = [](const aircraft::data::CameraConfig&,
                            std::string& error) {
        error = "注入槽按用例要求返回空（本次**不得**回落内置工厂）";
        return std::shared_ptr<aircraft::device::ICameraBackend>{};
    };

    EXPECT_FALSE(init.initialize("config")) << "注入工厂返回空却被判成装配成功";
    EXPECT_FALSE(init.ready());

    EXPECT_EQ(ctx.backend25, nullptr) << "回落了内置工厂：这一路被造出来了";
    EXPECT_EQ(ctx.backend50, nullptr);
    EXPECT_EQ(ctx.backend100, nullptr);
    EXPECT_EQ(ctx.cameras, nullptr) << "启动已判失败，却留下了半装配的相机管理器";
    EXPECT_EQ(ctx.controller, nullptr);

    const std::string e = init.errorText();
    EXPECT_NE(e.find("CAM25"), std::string::npos) << e;
    EXPECT_NE(e.find("注入槽按用例要求返回空"), std::string::npos)
        << "注入方给出的说明必须传到用户可见文本：" << e;
}

TEST(SystemInitializerTest, 场景D_注入工厂返回空但不给说明时文本仍可执行)
{
    // §1 裁决的边角：注入方返回空却**没给说明**时，文本不得留一句空的
    // 拼接结果（"CAM25 后端装配失败：" 后面什么都没有）——
    // 指路牌必须是可执行的。
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    ctx.backendFactory = [](const aircraft::data::CameraConfig&,
                            std::string&) {
        return std::shared_ptr<aircraft::device::ICameraBackend>{};
    };

    EXPECT_FALSE(init.initialize("config"));

    const std::string e = init.errorText();
    EXPECT_NE(e.find("未给出错误说明"), std::string::npos)
        << "空说明被如实写成了「没有」，而不是留一句空拼接：" << e;
    EXPECT_NE(e.find("cam25"), std::string::npos)
        << "至少要说清是哪一路：" << e;
}

// ===========================================================================
//  场景 T：转台装配（C-22）
// ===========================================================================

TEST(SystemInitializerTest, 场景T_未实现协议显式失败不回落虚拟后端)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    for (const char* protocol : {"rs485", "network"})
    {
        std::string text = repoYaml("turntable.yaml");
        text = replaceOnce(text, "protocol: \"sdk\"",
                           std::string("protocol: \"") + protocol + "\"");
        ASSERT_FALSE(text.empty()) << "未命中 turntable.yaml 的 protocol 行";

        const std::string virtualCamera = allVirtualCameraYaml();
        ASSERT_FALSE(virtualCamera.empty());
        const std::string dir = makeConfigVariantForFile(
            "turntable.yaml", text, err, "camera.yaml", virtualCamera);
        ASSERT_FALSE(dir.empty()) << err;

        ApplicationContext ctx;
        SystemInitializer  init(ctx);

        EXPECT_FALSE(init.initialize(dir)) << protocol << " 被静默接受了";
        EXPECT_TRUE(ctx.turntable == nullptr)
            << protocol << " 下造出了转台对象 —— 可能是回落虚拟后端";
        EXPECT_NE(init.errorText().find("拒绝回落虚拟后端"), std::string::npos)
            << init.errorText();
        EXPECT_NE(init.errorText().find(protocol), std::string::npos)
            << "失败原因未点名协议：" << init.errorText();
    }
}

TEST(SystemInitializerTest, 场景T_pelco_d_udp装配真实后端并如实报告连不上)
{
    // 不需要真的有转台：127.0.0.1 上没有 Pelco-D 设备时，
    // 真实后端会构造、打开 UDP、查询并失败。这里证明的是
    // "protocol=pelco_d_udp 真的走到了真实类"，而不是回落虚拟件。
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    std::string text = repoYaml("turntable.yaml");
    text = replaceOnce(text, "protocol: \"sdk\"", "protocol: \"pelco_d_udp\"");
    ASSERT_FALSE(text.empty());
    text = replaceOnce(text, "center_threshold: 50.0",
                       "center_threshold: 50.0\n"
                       "   host: \"127.0.0.1\"\n"
                       "   port: 9\n"
                       "   pelco_address: 1");
    ASSERT_FALSE(text.empty()) << "未命中 center_threshold 行";

    const std::string virtualCamera = allVirtualCameraYaml();
    ASSERT_FALSE(virtualCamera.empty());
    const std::string dir = makeConfigVariantForFile(
        "turntable.yaml", text, err, "camera.yaml", virtualCamera);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    EXPECT_FALSE(init.initialize(dir)) << "本机没有转台，启动不应成功";
    ASSERT_TRUE(ctx.turntable != nullptr) << "真实后端没有被构造出来";
    EXPECT_TRUE(dynamic_cast<VirtualTurntable*>(ctx.turntable.get()) == nullptr)
        << "protocol=pelco_d_udp 却装配了虚拟转台";
    EXPECT_NE(dynamic_cast<aircraft::device::PelcoDTurntableController*>(
                  ctx.turntable.get()),
              nullptr)
        << "protocol=pelco_d_udp 没有装配 PelcoDTurntableController";

    const std::string e = init.errorText();
    EXPECT_NE(e.find("转台初始化失败"), std::string::npos) << e;
    // 本机无设备时，真实链路可能回 ICMP 端口不可达（"UDP 报文失败"），
    // 也可能只是超时（"查询…超时"）。两者都证明查询真的发出去了。
    EXPECT_TRUE(e.find("UDP") != std::string::npos ||
                e.find("查询") != std::string::npos)
        << "失败原因没有说明查询/UDP 阶段：" << e;
}

TEST(SystemInitializerTest, 场景T_pekod旧取值给出迁移提示)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    std::string text = repoYaml("turntable.yaml");
    text = replaceOnce(text, "protocol: \"sdk\"", "protocol: \"pekod\"");
    ASSERT_FALSE(text.empty()) << "未命中 turntable.yaml 的 protocol 行";

    const std::string dir = makeConfigVariantForFile("turntable.yaml", text, err);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);
    EXPECT_FALSE(init.initialize(dir)) << "旧协议值居然被接受了";

    const std::string e = init.errorText();
    EXPECT_NE(e.find("已移除"), std::string::npos) << e;
    EXPECT_NE(e.find("pelco_d_udp"), std::string::npos)
        << "迁移提示没给出新取值：" << e;
}

TEST(SystemInitializerTest, 场景T_广播地址被配置层拒绝)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    std::string text = repoYaml("turntable.yaml");
    text = replaceOnce(text, "protocol: \"sdk\"", "protocol: \"pelco_d_udp\"");
    ASSERT_FALSE(text.empty());
    text = replaceOnce(text, "center_threshold: 50.0",
                       "center_threshold: 50.0\n"
                       "   host: \"255.255.255.255\"\n"
                       "   port: 6666\n"
                       "   pelco_address: 1");
    ASSERT_FALSE(text.empty());

    const std::string dir = makeConfigVariantForFile("turntable.yaml", text, err);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);
    EXPECT_FALSE(init.initialize(dir)) << "广播地址被当成了合法转台地址";

    const std::string e = init.errorText();
    EXPECT_NE(e.find("单播"), std::string::npos) << e;
    EXPECT_TRUE(ctx.turntable == nullptr) << "配置层已拒绝，却仍进入了转台装配";
}

TEST(SystemInitializerTest, 场景T_注入转台工厂返回空即失败且不回落)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    ctx.turntableFactory =
        [](const aircraft::data::TurntableConfig&, std::string& error) {
            error = "注入槽按用例要求返回空（本次不得回落内置工厂）";
            return std::unique_ptr<aircraft::device::ITurntableController>{};
        };

    EXPECT_FALSE(init.initialize("config"));
    EXPECT_TRUE(ctx.turntable == nullptr) << "回落了内置工厂";
    const std::string e = init.errorText();
    EXPECT_NE(e.find("注入槽按用例要求返回空"), std::string::npos) << e;
}

namespace
{
struct CountingTurntable : aircraft::device::ITurntableController
{
    bool initialize() override { return true; }
    bool move(const aircraft::data::TurntableCommand&) override { return true; }
    aircraft::data::TurntableState state() override { return {}; }
    void stop() override { ++stopCalls; }
    int stopCalls = 0;
};

/// shutdown 用例的转台替身：计数放在共享状态里，对象被
/// `shutdownDevices()` 释放后仍可读取"停了几次"。
struct ShutdownTrackingTurntable : aircraft::device::ITurntableController
{
    std::shared_ptr<int> stopCalls;

    bool initialize() override { return true; }
    bool move(const aircraft::data::TurntableCommand&) override { return true; }
    aircraft::data::TurntableState state() override { return {}; }
    void stop() override
    {
        if (stopCalls)
        {
            ++(*stopCalls);
        }
    }
};

}  // namespace

TEST(SystemInitializerTest, 场景T_注入工厂优先于协议且对象被实际装配)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    std::string text = repoYaml("turntable.yaml");
    // 即使协议是"本批未实现"的 rs485，显式工厂也应优先生效 ——
    // 这是装配槽的既有语义，不在协议分支里被悄悄绕过。
    text = replaceOnce(text, "protocol: \"sdk\"", "protocol: \"rs485\"");
    ASSERT_FALSE(text.empty());

    const std::string virtualCamera = allVirtualCameraYaml();
    ASSERT_FALSE(virtualCamera.empty());
    const std::string dir = makeConfigVariantForFile(
        "turntable.yaml", text, err, "camera.yaml", virtualCamera);
    ASSERT_FALSE(dir.empty()) << err;

    ApplicationContext ctx;
    SystemInitializer  init(ctx);

    ctx.turntableFactory =
        [](const aircraft::data::TurntableConfig&, std::string&) {
            return std::unique_ptr<aircraft::device::ITurntableController>(
                new CountingTurntable());
        };

    EXPECT_TRUE(init.initialize(dir)) << init.errorText();
    ASSERT_TRUE(ctx.turntable != nullptr);
    EXPECT_NE(dynamic_cast<CountingTurntable*>(ctx.turntable.get()), nullptr)
        << "装配的不是注入工厂造出的对象";
}

TEST(SystemInitializerTest, 场景A3_shutdown停止转台释放触发与相机且幂等)
{
    std::string err;
    ASSERT_TRUE(prepareWorkspace(err)) << err;

    Harness h;
    auto stopCalls = std::make_shared<int>(0);
    h.ctx.turntableFactory =
        [stopCalls](const aircraft::data::TurntableConfig&,
                    std::string&) -> std::unique_ptr<
            aircraft::device::ITurntableController> {
            auto t = std::make_unique<ShutdownTrackingTurntable>();
            t->stopCalls = stopCalls;
            return t;
        };

    ASSERT_TRUE(h.build()) << "全虚拟配置本应启动成功：" << h.init->errorText();
    ASSERT_NE(h.ctx.turntable, nullptr);
    ASSERT_NE(h.ctx.trigger, nullptr);
    EXPECT_TRUE(h.ctx.trigger->enabled());

    // 启动后的生命周期证据：三路都已 initialize → start。
    const std::shared_ptr<aircraft::device::ICameraBackend> rows[] = {
        h.ctx.backend25, h.ctx.backend50, h.ctx.backend100};
    for (const auto& b : rows)
    {
        ASSERT_EQ(joinLog(backendLogOf(b).entries), "initialize -> start");
    }

    h.init->shutdown();

    // 1. 关闭入口：ready_ 不再为真，tick 不再推进。
    EXPECT_FALSE(h.init->ready());

    // 2. 转台：停止一次后释放（析构 join worker）。
    EXPECT_EQ(*stopCalls, 1);
    EXPECT_TRUE(h.ctx.turntable == nullptr);

    // 3. 触发器：disable 后释放。
    EXPECT_TRUE(h.ctx.trigger == nullptr);

    // 4. 相机：每路都 stop → close（这是 closeBackends 的可见结果）。
    for (const auto& b : rows)
    {
        EXPECT_EQ(joinLog(backendLogOf(b).entries),
                  "initialize -> start -> stop -> close");
    }

    // 5. 幂等：重复调用不再停转台、不再重复 stop/close。
    h.init->shutdown();
    EXPECT_EQ(*stopCalls, 1);
    for (const auto& b : rows)
    {
        EXPECT_EQ(joinLog(backendLogOf(b).entries),
                  "initialize -> start -> stop -> close");
    }

    // 6. 析构兜底也不得再触发一次（Harness 负责销毁 init 与 ctx）。
}

// ===========================================================================
//  场景 S：run_mode=simulation —— 同一合成场景贯穿生产链
//
//  ---- 这组用例证明什么 ----
//
//  批次 B 的验收句是："用同一合成场景证明'已知几何/相机/目标位姿 →
//  模拟图像 → 实际检测/匹配/PnP → 与独立真值比较 → 应用自动落盘'整条链
//  成立"。故本用例**不注入任何桩**、不直接给算法喂二维点：
//    · 图像：`SyntheticTargetScene` 按真值位姿投影出直角 L 形目标；
//    · 检测/匹配/PnP：生产链原样运行（CAD 结构点定位从图像里找角点）；
//    · 落盘：走 `SystemInitializer` 装配出的 `RecorderSinkAdapter` / Recorder；
//    · 比较：读回 result.json 里的 `truth.error`，与场景真值（场景固有事实）
//      对照 —— 真值只用于渲染与验收，**从不**进入求解器。
//
//  ---- 为什么要改写配置 ----
//
//  四处改写全部来自仿真场景自身的物理/装配前提，而不是给算法放宽判据：
//    · camera.yaml：合成标定按 CAM25 的尺寸生成内参并供三路共用
//      （`CalibrationManager::loadDefaults`），三路分辨率不一致会让
//      主点与图像错配 —— 故统一为 1280×1024；
//    · system.yaml：`run_mode=simulation`，且 log/output/model 指向本用例
//      专用路径。**不能**复用工作目录的 "models"：场景 A 依赖
//      "models/ 不存在 ⇒ 机型库不可用 ⇒ POSE_SOLVE 失败"这一前提，
//      在这里造出 models/ 会把那个前提静默改掉；
//    · measurement.yaml：`min_feature_count` 8 → 4。PnP 的门槛是
//      `max(4, min_feature_count)`，而合成几何只有 6 个 CAD 点 ——
//      8 会让本可解算的六点解被自己的配置拒掉。仓库值不动
//      （生产模型的点数尚未标定）；
//    · validation.yaml：yaw 区间 [−120°, −60°]。默认真值在本工程的
//      坐标约定与单位标定下分解为 yaw ≈ −90°（机头朝相机：
//      R = Rx(−5°)·Ry(90°)·Rz(1.2°)）。`±5°` 是"侧向目标"的物理范围，
//      与"迎头目标"不矛盾；这里改的是**场景的物理可达范围**，重投影/
//      内点/置信度三项精度判据仍取仓库值。
// ===========================================================================

namespace
{

/// 把文本写到 `path`；失败返回 false（调用方必须断言，否则后续用例会在
/// "文件根本没写出来"的前提下得到一个与被测代码无关的失败）。
bool writeTextAt(const std::string& path, const std::string& text)
{
    std::ofstream out(path, std::ios::binary);
    if (!out)
    {
        return false;
    }
    out << text;
    return out.good();
}

/// 写一份由 `data::synthetic` 几何生成的机型库（SYS-12 §7 的目录布局）。
///
/// @param scaled 负例用：把点表整体放大 2 倍（model_id 不变）。
///        这模拟"画面与模型不同源"——两者都能被各自加载，只有解算的
///        重投影对不上。**不是**注入最终位姿，也不是让求解器读真值。
/// @param perturbIndex 负例用：把该下标的点沿 x 移动 0.25 m（model_id 不变）。
///        用于证明同源校验不是只看包围盒尺度，而是逐点比内容。
bool writeSyntheticModelDir(const std::string& dir,
                            bool               scaled,
                            std::string&       err,
                            int                perturbIndex = -1)
{
    if (!aircraft::infrastructure::fileutil::makeDirectories(dir))
    {
        err = "创建模型目录失败：" + dir;
        return false;
    }

    std::vector<aircraft::data::ModelPoint3D> points =
        aircraft::data::synthetic::modelPoints();
    if (scaled)
    {
        for (aircraft::data::ModelPoint3D& p : points)
        {
            p.position *= 2.0f;
        }
    }
    if (perturbIndex >= 0
        && perturbIndex < static_cast<int>(points.size()))
    {
        points[static_cast<std::size_t>(perturbIndex)].position.x += 0.25f;
    }

    // model.yaml：模型身份与版本的**权威来源**（SYS-04 §6.3）。
    // model_id 必须与 `data::synthetic::modelId()` 一致，否则
    // `SystemInitializer::buildAlgorithm` 会拒绝启动（同源校验）。
    std::ostringstream meta;
    meta << "%YAML:1.0\n"
         << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
         << "version: 1\n"
         << "model_source: \"data/SyntheticTargetGeometry.h\"\n"
         << "generated_at: \"2026-10-20T00:00:00\"\n"
         << "feature_algorithm_version: \"none\"\n";
    if (!writeTextAt(dir + "/model.yaml", meta.str()))
    {
        err = "写 model.yaml 失败：" + dir;
        return false;
    }

    // points3d.yaml：点表。数值**逐点取自** `outline()`，不在此处另写一份。
    std::ostringstream pts;
    pts << "%YAML:1.0\n"
        << "model_id: \"" << aircraft::data::synthetic::modelId() << "\"\n"
        << "version: 1\n"
        << "points:\n";
    pts.setf(std::ios::fixed);
    pts.precision(6);
    for (const aircraft::data::ModelPoint3D& p : points)
    {
        pts << "  - { id: " << p.id << ", x: " << p.position.x
            << ", y: " << p.position.y << ", z: " << p.position.z
            << ", type: " << p.featureType << " }\n";
    }
    if (!writeTextAt(dir + "/points3d.yaml", pts.str()))
    {
        err = "写 points3d.yaml 失败：" + dir;
        return false;
    }

    // 三个焦段的特征库：空库（N=0、D=0）是"尚无纹理特征"的合法状态，
    // 也正是本场景要走的路径 —— B 类 CAD 结构点单独支撑 PnP。
    aircraft::data::TargetModel model;
    model.modelId = aircraft::data::synthetic::modelId();
    model.points3d = points;

    const aircraft::data::CameraRole roles[3] = {
        aircraft::data::CameraRole::CAM25, aircraft::data::CameraRole::CAM50,
        aircraft::data::CameraRole::CAM100};
    const char* const names[3] = {
        "feature25.bin", "feature50.bin", "feature100.bin"};
    for (int i = 0; i < 3; ++i)
    {
        if (!aircraft::algorithm::TargetModelManager::saveFeatureLibrary(
                dir + "/" + names[i], roles[i], model))
        {
            err = std::string("写特征库失败：") + names[i];
            return false;
        }
    }
    return true;
}

/// 由仓库 7 个 yaml 改写出一份 `run_mode=simulation` 的配置目录。
/// @param modelDirName system.yaml 的 model_dir（相对进程 CWD）。
/// @return 目录路径；空串 = 失败（原因写 `err`）。
std::string makeSimulationConfigDir(const std::string& modelDirName,
                                    std::string&       err)
{
    char        tmpl[] = "/tmp/aps_sim_cfg_XXXXXX";
    char* const dir    = ::mkdtemp(tmpl);
    if (dir == nullptr)
    {
        err = "mkdtemp 失败";
        return std::string();
    }
    const std::string tmp = dir;

    for (const char* const f : kConfigFiles)
    {
        std::string text;

        if (std::string(f) == "camera.yaml")
        {
            text = allVirtualCameraYaml();
            if (text.empty())
            {
                err = "未能定位 camera.yaml 的三路记录";
                return std::string();
            }
            // 统一 1280×1024（CAM50/CAM100 已是该值；只改 CAM25）。
            text = replaceInCameraRecord(text, "cam25", "width",
                                         "      width: 1280");
            if (text.empty())
            {
                err = "camera.yaml 的 cam25.width 未命中";
                return std::string();
            }
            const std::string withHeight = replaceInCameraRecord(
                text, "cam25", "height", "      height: 1024");
            if (withHeight.empty())
            {
                err = "camera.yaml 的 cam25.height 未命中";
                return std::string();
            }
            text = withHeight;
        }
        else if (std::string(f) == "system.yaml")
        {
            text = repoYaml("system.yaml");
            const std::pair<const char*, std::string> edits[] = {
                {"log_dir: \"logs\"",           "log_dir: \"sim_logs\""},
                {"output_dir: \"output\"",      "output_dir: \"sim_output\""},
                {"model_dir: \"models\"",
                 "model_dir: \"" + modelDirName + "\""},
                {"run_mode: \"hardware_readonly\"",
                 "run_mode: \"simulation\""},
            };
            for (const auto& e : edits)
            {
                text = replaceOnce(text, e.first, e.second);
                if (text.empty())
                {
                    err = std::string("system.yaml 未命中：") + e.first;
                    return std::string();
                }
            }
        }
        else if (std::string(f) == "measurement.yaml")
        {
            text = replaceOnce(repoYaml("measurement.yaml"),
                               "min_feature_count: 8",
                               "min_feature_count: 4");
            if (text.empty())
            {
                err = "measurement.yaml 未命中 min_feature_count";
                return std::string();
            }
        }
        else if (std::string(f) == "validation.yaml")
        {
            text = repoYaml("validation.yaml");
            text = replaceOnce(text, "yaw_min: -5.0", "yaw_min: -120.0");
            if (text.empty())
            {
                err = "validation.yaml 未命中 yaw_min";
                return std::string();
            }
            text = replaceOnce(text, "yaw_max: 5.0", "yaw_max: -60.0");
            if (text.empty())
            {
                err = "validation.yaml 未命中 yaw_max";
                return std::string();
            }
        }
        else
        {
            const std::string src = std::string(APS_SOURCE_DIR) + "/config/" + f;
            text = readFile(src);
            if (text.empty())
            {
                err = "读取失败：" + src;
                return std::string();
            }
        }

        if (!writeTextAt(tmp + "/" + f, text))
        {
            err = std::string("写 ") + f + " 失败：" + tmp;
            return std::string();
        }
    }
    return tmp;
}

/// 读回 result.json 里 `aircraftToShip.rotation` 与真值逐元素比较。
void expectRotationNear(const cv::FileNode& rotNode,
                        const cv::Matx33d&  expected,
                        const char*         tag)
{
    ASSERT_TRUE(rotNode.isSeq()) << tag << "：rotation 不是数组";
    ASSERT_EQ(rotNode.size(), 3u);
    int r = 0;
    for (const cv::FileNode& row : rotNode)
    {
        ASSERT_EQ(row.size(), 3u) << tag << "：rotation 行不是 3 元";
        for (int c = 0; c < 3; ++c)
        {
            double v = 0.0;
            row[c] >> v;
            EXPECT_NEAR(v, expected(r, c), 1e-6)
                << tag << "：truth rotation[" << r << "][" << c << "] 与场景真值不符";
        }
        ++r;
    }
}

}  // namespace

// ============================================================================
//  夹具事实：六个合成点确实非共面。
//
//  为什么要有这条用例：注释里的“非共面”曾用“深度 39.5~40.5 m 不同”
//  来支撑，但倾斜平板上的点同样有不同深度，那不是判据。这里直接从
//  `outline()` 取实际坐标做 SVD，钉住实测值；坐标一旦改动而忘记复核，
//  这条用例先红。
//
//  判据（不是从 Z 极差推断）：
//    · σ3/σ1 不可忽略：本夹具实测 7.5455e-2；
//    · 到最佳拟合平面的最大距离 max|d| 不可忽略：实测 0.4429 m。
//  阈值取“明显大于数值噪声、又留出复核余量”的量级，只用于本夹具；
//  它们不是 PnP 或定位器的判据，也不由此反推任何验收阈值。
// ============================================================================
TEST(SystemInitializerTest, 场景S_合成靶标六点非共面性可复核)
{
    const std::vector<cv::Point3f> pts = aircraft::data::synthetic::outline();
    ASSERT_EQ(pts.size(), 6u);

    cv::Mat centered(static_cast<int>(pts.size()), 3, CV_64F);
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        centered.at<double>(static_cast<int>(i), 0) = pts[i].x;
        centered.at<double>(static_cast<int>(i), 1) = pts[i].y;
        centered.at<double>(static_cast<int>(i), 2) = pts[i].z;
    }
    // ⚠ 不能用 `cv::mean(Mat)`：单通道时它返回**全体元素**的均值，
    //   不是逐列均值。这里要的是质心，必须按列求平均。
    cv::Mat colMeans;
    cv::reduce(centered, colMeans, 0, cv::REDUCE_AVG);
    ASSERT_EQ(colMeans.rows, 1);
    ASSERT_EQ(colMeans.cols, 3);
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        for (int c = 0; c < 3; ++c)
        {
            centered.at<double>(static_cast<int>(i), c) -=
                colMeans.at<double>(0, c);
        }
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(centered, w, u, vt, cv::SVD::FULL_UV);
    ASSERT_EQ(w.total(), 3u);
    const double s1 = w.at<double>(0);
    const double s3 = w.at<double>(2);
    ASSERT_GT(s1, 0.0);

    const double ratio = s3 / s1;
    EXPECT_NEAR(s1, 8.546060674, 1e-6);
    EXPECT_NEAR(s3, 0.644843763, 1e-6);
    EXPECT_NEAR(ratio, 7.545509e-02, 1e-6)
        << "σ3/σ1 变化：六点分布已不再是登记的非共面臂标，须重新复核";

    // 最佳拟合平面过质心；Vt 第 3 行即法向。
    const cv::Vec3d normal(vt.at<double>(2, 0),
                           vt.at<double>(2, 1),
                           vt.at<double>(2, 2));
    double maxAbsDist = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const cv::Vec3d p(centered.at<double>(static_cast<int>(i), 0),
                          centered.at<double>(static_cast<int>(i), 1),
                          centered.at<double>(static_cast<int>(i), 2));
        maxAbsDist = std::max(maxAbsDist, std::abs(p.dot(normal)));
    }
    EXPECT_NEAR(maxAbsDist, 0.442892, 1e-6)
        << "到拟合平面的最大距离变化：非共面结论须重新复核";
    EXPECT_GT(maxAbsDist, 1e-2)
        << "六点近似共面：不得再称“具有深度变化的靶标”，更不得称为非共面";
}

TEST(SystemInitializerTest, 场景S_仿真运行模式走通生产链并落盘真值对照)
{
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    std::string modelErr;
    ASSERT_TRUE(writeSyntheticModelDir("sim_models", false, modelErr))
        << modelErr;

    std::string cfgErr;
    const std::string simDir = makeSimulationConfigDir("sim_models", cfgErr);
    ASSERT_FALSE(simDir.empty()) << cfgErr;

    Harness h;
    ASSERT_TRUE(h.build(simDir)) << "仿真配置启动失败：" << h.init->errorText();

    // 装配形态：三路虚拟相机 + 虚拟转台 + 虚拟触发，无降级。
    EXPECT_EQ(h.init->summary().cameraCount, 3);
    EXPECT_EQ(h.init->summary().cameraReadyCount, 3);
    EXPECT_FALSE(h.init->summary().degradedAtStart);
    ASSERT_TRUE(h.ctx.recorder != nullptr);

    std::fprintf(stderr,
                 "\n[场景S] 仿真装配完成：机型库=%d 标定=%d；"
                 "开始 startMeasurement() → 逐拍推进\n",
                 static_cast<int>(h.init->summary().modelsLoaded),
                 static_cast<int>(h.init->summary().calibrationLoaded));

    ASSERT_TRUE(h.init->startMeasurement())
        << "测量启动失败：" << h.ctx.controller->lastError().message;

    const std::vector<TickAccount> accounts = runUntilTerminal(h, 1);
    ASSERT_FALSE(accounts.empty());

    // ---- 终态：必须是 COMPLETE，不能是"跑完但失败" ----
    const aircraft::data::ErrorInfo lastError = h.ctx.controller->lastError();
    ASSERT_EQ(h.ctx.controller->state(), S::COMPLETE)
        << "仿真链未走到 COMPLETE；末次错误码 " << lastError.code << "（"
        << aircraft::data::errorCodeName(lastError.code) << "）："
        << lastError.message;

    // ---- 落盘：结果包必须由 SAVE 自动产出 ----
    const std::string packageDir = h.ctx.recorder->lastPackageDir();
    ASSERT_FALSE(packageDir.empty())
        << "COMPLETE 但没有结果包：" << h.ctx.recorder->lastErrorText();
    const std::string resultPath = h.ctx.recorder->lastResultPath();
    std::fprintf(stderr, "[场景S] 结果包：%s\n", packageDir.c_str());

    cv::FileStorage fs(resultPath, cv::FileStorage::READ);
    ASSERT_TRUE(fs.isOpened()) << "result.json 不是合法 json：" << resultPath;

    // ---- 运行形态元数据（B2 的验收字段）----
    std::string state;
    std::string runMode;
    std::string modelType;
    std::string modelId;
    std::string featureVersion;
    std::string calibrationId;
    fs["state"] >> state;
    fs["run_mode"] >> runMode;
    fs["model_type"] >> modelType;
    fs["model_id"] >> modelId;
    fs["feature_version"] >> featureVersion;
    fs["calibration_id"] >> calibrationId;

    EXPECT_EQ(state, "COMPLETE");
    EXPECT_EQ(runMode, "simulation")
        << "结果包必须记录本次运行的形态，而不是仓库默认值";
    EXPECT_EQ(modelType, "synthetic");
    EXPECT_EQ(modelId, aircraft::data::synthetic::modelId());
    EXPECT_EQ(featureVersion, "1");
    // 结果包必须记录**实际使用**的标定标识。配置里的 `calibration_id:
    // synthetic-m1` 是"标定文件存在时应加载的 id"；本场景的标定文件不存在，
    // CalibrationManager 按 calibration_mode=synthetic 改用 loadDefaults
    // 的合成内参，其标识是 SYNTHETIC-NO-CALIBRATION。若包内写成
    // synthetic-m1，就等于声称用了一份从未加载的标定。
    EXPECT_EQ(calibrationId, h.ctx.calibration->calibrationId())
        << "结果包的 calibration_id 与实际装载的标定不一致";
    EXPECT_EQ(calibrationId, "SYNTHETIC-NO-CALIBRATION")
        << "仿真包必须如实标明用的是合成内参";

    // ---- 实际标定参数：标识相同不等于参数相同 --------------------
    //
    // 配置里写的是 `synthetic-m1`，实际装载的是默认合成内参。只断言
    // id 字符串无法区分“渲染与求解用了同一份参数”与“两者只是同名”。
    // 这里逐项核对包内参数，并与 `ctx_.rig->calibration()`（求解器使用
    // 的同一对象）比对；再用同一份标定独立重投影，确认渲染几何与参数
    // 自洽。三路相机均为 1280x1024、45° 水平视场、零畸变、单位外参。
    const cv::FileNode calibParams = fs["calibration_parameters"];
    ASSERT_TRUE(calibParams.isSeq()) << "结果包缺少 calibration_parameters";
    ASSERT_EQ(calibParams.size(), 4u)
        << "calibration_parameters 必须含 CAM25/CAM50/CAM100/RIG 四项";
    std::vector<std::string> calibText;
    for (const cv::FileNode& n : calibParams)
    {
        std::string s;
        n >> s;
        calibText.push_back(s);
    }

    const aircraft::data::OpticalRigCalibration actualCalib =
        h.ctx.rig->calibration();
    const double expectedFocal =
        (1280.0 / 2.0) / std::tan(22.5 * CV_PI / 180.0);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9f", expectedFocal);
    const std::string focalText(buf);

    const aircraft::data::CameraCalibration actualCams[3] = {
        actualCalib.cam25, actualCalib.cam50, actualCalib.cam100};
    const char* const calibRoles[3] = {"CAM25", "CAM50", "CAM100"};
    for (int i = 0; i < 3; ++i)
    {
        const std::string& text = calibText[static_cast<std::size_t>(i)];
        EXPECT_EQ(text.compare(0, std::string(calibRoles[i]).size(),
                               calibRoles[i]),
                  0)
            << "第 " << i << " 项不是 " << calibRoles[i] << "："
            << text;
        EXPECT_NE(text.find("id=SYNTHETIC-NO-CALIBRATION"), std::string::npos)
            << "参数文本必须带实际装载的标定标识：" << text;
        EXPECT_NE(text.find("fx=" + focalText), std::string::npos)
            << "包内 fx 不是默认合成内参的反推值：" << text;
        EXPECT_NE(text.find("image=1280x1024"), std::string::npos)
            << "包内图像尺寸与相机配置不符：" << text;
        EXPECT_NE(text.find("cameraToRig{R=[1.000000000"), std::string::npos)
            << "默认合成外参应为单位阵，包内不是：" << text;

        // 与求解器实际持有的数据逐项比对（不是比对配置文本）。
        const cv::Mat& k = actualCams[i].cameraMatrix;
        ASSERT_EQ(k.rows, 3);
        ASSERT_EQ(k.cols, 3);
        EXPECT_NEAR(k.at<double>(0, 0), expectedFocal, 1e-9);
        EXPECT_NEAR(k.at<double>(1, 1), expectedFocal, 1e-9);
        EXPECT_NEAR(k.at<double>(0, 2), 640.0, 1e-9);
        EXPECT_NEAR(k.at<double>(1, 2), 512.0, 1e-9);
        EXPECT_EQ(actualCams[i].imageWidth, 1280);
        EXPECT_EQ(actualCams[i].imageHeight, 1024);
        EXPECT_EQ(actualCams[i].cameraToRig.rotation, cv::Matx33d::eye());
        EXPECT_EQ(actualCams[i].cameraToRig.translation, cv::Vec3d(0, 0, 0));
    }
    EXPECT_NE(calibText[3].find("RIG: id=SYNTHETIC-NO-CALIBRATION"),
              std::string::npos);
    EXPECT_NE(calibText[3].find("rigToShip{R=[1.000000000"),
              std::string::npos);
    EXPECT_EQ(actualCalib.rigToShip.rotation, cv::Matx33d::eye());
    EXPECT_EQ(actualCalib.rigToShip.translation, cv::Vec3d(0, 0, 0));

    // 渲染侧自洽：用**同一份**实际标定重建场景，把六个点投影回去，
    // 应落回设计位置。这检查的是“参数 → 画面”的几何关系，不读取
    // 求解结果，也不复用生产误差函数。
    {
        aircraft::device::SyntheticTargetScene scene(
            actualCalib, aircraft::data::CameraRole::CAM100,
            aircraft::device::SyntheticTargetScene::
                defaultTruthAircraftToShip());
        ASSERT_TRUE(scene.valid());
        const aircraft::data::Transform& a2c = scene.aircraftToCamera();
        const cv::Point2d design[6] = {
            {480, 400}, {800, 400}, {800, 624},
            {620, 624}, {620, 500}, {480, 500}};
        const std::vector<cv::Point3f> pts =
            aircraft::data::synthetic::outline();
        ASSERT_EQ(pts.size(), 6u);
        const cv::Matx33d K = actualCalib.cam100.cameraMatrix;
        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            const cv::Vec3d p(pts[i].x, pts[i].y, pts[i].z);
            const cv::Vec3d pc = a2c.rotation * p + a2c.translation;
            ASSERT_GT(pc[2], 0.0);
            const double u = K(0, 0) * pc[0] / pc[2] + K(0, 2);
            const double v = K(1, 1) * pc[1] / pc[2] + K(1, 2);
            EXPECT_NEAR(u, design[i].x, 0.5)
                << "第 " << i + 1 << " 点投影 u 与设计位置不符";
            EXPECT_NEAR(v, design[i].y, 0.5)
                << "第 " << i + 1 << " 点投影 v 与设计位置不符";
        }
    }

    std::string realism;
    fs["component_realism"] >> realism;
    EXPECT_NE(realism.find("camera=virtual"), std::string::npos)
        << "component_realism 未如实声明虚拟相机：" << realism;
    EXPECT_NE(realism.find("detector=mock"), std::string::npos)
        << "component_realism 未如实声明桩检测器：" << realism;

    // ---- 每路相机的 backend / real_or_virtual ----
    const cv::FileNode cameras = fs["cameras"];
    ASSERT_TRUE(cameras.isSeq()) << "cameras 未写出";
    ASSERT_EQ(cameras.size(), 3u);
    const char* const expectedRoles[3] = {"CAM25", "CAM50", "CAM100"};
    int cameraIndex = 0;
    for (const cv::FileNode& n : cameras)
    {
        std::string role;
        std::string backend;
        std::string realOrVirtual;
        n["role"] >> role;
        n["backend"] >> backend;
        n["real_or_virtual"] >> realOrVirtual;
        EXPECT_EQ(role, expectedRoles[cameraIndex]);
        EXPECT_EQ(backend, "virtual") << "仿真包的 backend 必须如实写";
        EXPECT_EQ(realOrVirtual, "virtual");
        ++cameraIndex;
    }

    // ---- 独立真值与误差（真值只在这里被比较，不进入求解器）----
    const cv::FileNode truth = fs["truth"];
    ASSERT_FALSE(truth.empty()) << "仿真包必须写出 truth 块";
    std::string truthSource;
    truth["source"] >> truthSource;
    EXPECT_EQ(truthSource, aircraft::data::synthetic::truthSource());

    const aircraft::data::Transform expectedTruth =
        aircraft::device::SyntheticTargetScene::defaultTruthAircraftToShip();
    expectRotationNear(truth["aircraftToShip"]["rotation"],
                       expectedTruth.rotation, "truth");

    const cv::FileNode truthT =
        truth["aircraftToShip"]["translation"];
    ASSERT_TRUE(truthT.isSeq());
    ASSERT_EQ(truthT.size(), 3u);
    for (int i = 0; i < 3; ++i)
    {
        double v = 0.0;
        truthT[i] >> v;
        EXPECT_NEAR(v, expectedTruth.translation[i], 1e-6)
            << "truth translation[" << i << "] 与场景真值不符";
    }

    const cv::FileNode truthErr = truth["error"];
    ASSERT_FALSE(truthErr.empty()) << "成功包必须写出真值误差";
    double rotErrDeg = -1.0;
    double transErrM = -1.0;
    truthErr["rotation_deg"] >> rotErrDeg;
    truthErr["translation_m"] >> transErrM;

    // ---- 独立复算误差：不信任包内 `truth.error` 的算法 ----
    //
    // 只断言 "误差 ≤ 阈值" 挡不住"恒写 0"：那样 EXPECT_LE 照样通过。
    // 这里从包里读回**求解出的姿态**，在测试侧用与生产不同的代码路径
    // （trace 公式直接写在用例里）重算，再与包内数值比对。两者一致，
    // 才说明包内误差确实来自这次求解与真值的比较。
    {
        cv::Matx33d solvedR;
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                double v = 0.0;
                fs["aircraftToShip"]["rotation"][r][c] >> v;
                solvedR(r, c) = v;
            }
        }
        cv::Vec3d solvedT(0.0, 0.0, 0.0);
        for (int i = 0; i < 3; ++i)
        {
            double v = 0.0;
            fs["aircraftToShip"]["translation"][i] >> v;
            solvedT[i] = v;
        }

        const cv::Matx33d dr = expectedTruth.rotation.t() * solvedR;
        double cc = (dr(0, 0) + dr(1, 1) + dr(2, 2) - 1.0) * 0.5;
        cc = std::max(-1.0, std::min(1.0, cc));
        const double independentRotDeg = std::acos(cc) * 180.0 / CV_PI;
        const cv::Vec3d dt = solvedT - expectedTruth.translation;
        const double independentTransM =
            std::sqrt(dt[0] * dt[0] + dt[1] * dt[1] + dt[2] * dt[2]);

        EXPECT_NEAR(rotErrDeg, independentRotDeg, 1e-3)
            << "包内旋转误差与独立复算不一致";
        EXPECT_NEAR(transErrM, independentTransM, 1e-3)
            << "包内平移误差与独立复算不一致";
    }

    const double reprojectionForLog = [&fs]() {
        double e = -1.0;
        fs["reprojection_error"] >> e;
        return e;
    }();
    std::fprintf(stderr,
                 "[场景S] 链路成功与精度分列：\n"
                 "  应用终态 COMPLETE → 状态机与自动保存通路跑通\n"
                 "  旋转误差 %.6f deg = %.3f 角分 → 不构成一角分精度证据\n"
                 "  平移误差 %.6f m = %.1f mm → 随本场景真值定义记录\n"
                 "  重投影 %.6f px → 当前对应集的图像拟合结果\n",
                 rotErrDeg, rotErrDeg * 60.0,
                 transErrM, transErrM * 1000.0,
                 reprojectionForLog);

    // ---- 容差：是“通路判别上限”，不是精度验收指标 ----
    //
    // SYS-15 的 1 角分精度验收不适用于本合成链：本场景的相机是针孔模型、
    // 目标是已知投影、检测器是桩，测得的误差不能外推到实机精度。因此这里
    // 0.5° / 0.5 m 的用途只有一个：区分“链在解算”与“链没解算却报了成功”。
    //
    // 依据（在定义场景时固定，**不是**从本次观测值 0.3216° / 0.1434 m 反推）：
    //   · 旋转：粗姿态与真值相差 1.2°（`defaultTruthAircraftToShip` 的 Rz）。
    //     若求解器把粗姿态原样回填，误差正好是 1.2°。0.5° 小于该分离量，
    //     故“抄回先验”不能通过；它不是“精度合格线”。
    //   · 平移：真值距离 40 m，粗姿态的 (0,0,40) 与真值相同。0.5 m 是
    //     量级判别（挡轴序/尺度错误），约 1.25% 的距离，同样不是精度线。
    //   · 两个数字都已如实记录在验收文档中；不得因本次观测更小就收紧，
    //     也不得因换场景后变大就放宽。
    EXPECT_GT(rotErrDeg, 0.0);
    EXPECT_LE(rotErrDeg, 0.5)
        << "解算结果与独立真值的旋转差超过 0.5°：不是一条在解算的链";
    EXPECT_GT(transErrM, 0.0);
    EXPECT_LE(transErrM, 0.5)
        << "解算结果与独立真值的平移差超过 0.5 m";

    // ---- 求解确实用了图像侧的 CAD 对应（不是空对应集上的"成功"）----
    int cadCount = 0;
    int matchCount = 0;
    int inputFeatureCount = 0;
    fs["statistics"]["cad_count"] >> cadCount;
    fs["statistics"]["match_count"] >> matchCount;
    fs["statistics"]["feature_count"] >> inputFeatureCount;
    EXPECT_GE(cadCount, 4)
        << "PnP 至少需要 4 个 CAD 对应，结果包里只有 " << cadCount;
    EXPECT_EQ(matchCount, cadCount)
        << "纯 B 类退化路径下，参与 PnP 的对应集必须与统计量一致（R08）";
    (void)inputFeatureCount;

    double reprojection = -1.0;
    double inlierRatio = -1.0;
    fs["reprojection_error"] >> reprojection;
    fs["validation"]["inlier_ratio"] >> inlierRatio;
    // ⚠ 这里的重投影误差是 PnP 对**当前对应集**的图像拟合残差（像素），
    // 与 `CadStructureLocator` 的“每条拟合棱线中位残差 ≤ 0.3 px”
    // 不是同一个指标。实测值略大于 0.3 px 不构成违反冻结判据，
    // 也不能反过来证明定位器合格。定位器判据另见 AlgorithmStageTest。
    EXPECT_LE(reprojection, 2.0) << "VALIDATE 判据内的重投影误差";
    EXPECT_GE(inlierRatio, 0.3) << "VALIDATE 判据内的内点比例";

    // ---- 成功包的失败摘要：first_error.code == 0，且没有失败记录 ----
    const cv::FileNode failure = fs["failure"];
    ASSERT_FALSE(failure.empty()) << "成功包也必须写出 failure 摘要";
    int firstCode = -1;
    failure["first_error"]["code"] >> firstCode;
    EXPECT_EQ(firstCode, 0)
        << "COMPLETE 包的 first_error.code 必须为 0（没有失败根因）";

    EXPECT_EQ(fs["missing_required_fields"].size(), 0u)
        << "必含字段有缺失，包不完整";
    EXPECT_EQ(fs["non_finite_fields"].size(), 0u)
        << "结果包含非有限值";

    // ---- 正常状态迁移历史必须保留（不是"因为成功就清空"）----
    cv::FileStorage ff(packageDir + "/failure.json", cv::FileStorage::READ);
    ASSERT_TRUE(ff.isOpened()) << "failure.json 不是合法 json";
    const cv::FileNode history = ff["history"];
    ASSERT_TRUE(history.isSeq());
    ASSERT_GT(history.size(), 0u)
        << "成功包也保留正常迁移历史（SYS-08 的可追溯要求）";

    bool sawSearch = false;
    bool sawSave = false;
    std::string previousTo;
    for (const cv::FileNode& hop : history)
    {
        std::string from;
        std::string to;
        int code = -1;
        hop["from"] >> from;
        hop["to"] >> to;
        hop["error"]["code"] >> code;
        EXPECT_EQ(code, 0)
            << "成功包的迁移历史里出现失败记录：" << from << " -> " << to;
        if (to == "SEARCH") { sawSearch = true; }
        if (to == "SAVE")   { sawSave = true; }
        previousTo = to;
    }

    EXPECT_TRUE(sawSearch) << "迁移历史缺少进入 SEARCH 的一条";
    EXPECT_TRUE(sawSave) << "迁移历史缺少进入 SAVE 的一条";
    EXPECT_EQ(previousTo, "COMPLETE")
        << "迁移历史的终点必须是 COMPLETE（正常历史不得被截断）";
}

TEST(SystemInitializerTest, 场景S_模型标识相同但点表不同源时拒绝启动)
{
    // 负例：图像侧按 `data::synthetic::outline()` 渲染，模型侧的
    // model_id **保持不变**，只改点表内容：
    //   · 变体一：整体放大 2 倍；
    //   · 变体二：只把一个点移动 0.25 m。
    //
    // 为什么要断言“拒绝启动”而不是“跑到 FAILED”：
    //   model_id 相同不等于几何内容相同。若只在启动时比 id，整体缩放
    //   在某些位姿下可能被估计出的平移吸收，PnP 并不会必然失败；
    //   届时即使 COMPLETE，得到的也是一个与真实尺度不符的姿态。
    //   故同源校验必须比**点表内容**，并在装配阶段给出明确原因。
    //   ⚠ 本用例只证明这两个选定反例被拦下，不宣称“所有错误模型都会
    //   使 PnP 失败”。
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    struct Variant
    {
        const char* dir;
        bool        scaled;
        int         perturbIndex;
    };
    const Variant variants[] = {
        {"sim_models_scaled", true, -1},
        {"sim_models_onept", false, 2},
    };

    for (const Variant& v : variants)
    {
        std::string modelErr;
        ASSERT_TRUE(writeSyntheticModelDir(v.dir, v.scaled, modelErr,
                                           v.perturbIndex))
            << v.dir << "：" << modelErr;

        std::string cfgErr;
        const std::string simDir = makeSimulationConfigDir(v.dir, cfgErr);
        ASSERT_FALSE(simDir.empty()) << cfgErr;

        Harness h;
        EXPECT_FALSE(h.build(simDir))
            << v.dir << "：model_id 相同但点表内容不同，启动必须被拒绝";
        const std::string errorText = h.init->errorText();
        EXPECT_NE(errorText.find("模型标识相同不等于点表内容相同"),
                  std::string::npos)
            << v.dir << "：拒绝原因必须点名“标识相同、内容不同”："
            << errorText;
        EXPECT_NE(errorText.find("不同源"), std::string::npos)
            << v.dir << "：拒绝原因必须说明不同源：" << errorText;

        // 具体失败阶段：同源校验在 buildAlgorithm()，早于 buildApplication()
        // 创建 Recorder。recorder 为空证明失败没有落成“跑起来以后再失败”，
        // 也没有依赖后段 PnP 偶然报错。
        EXPECT_EQ(h.ctx.recorder, nullptr)
            << v.dir << "：未通过同源校验却已创建落盘器，失败阶段不对";
        EXPECT_EQ(h.ctx.controller, nullptr)
            << v.dir << "：未通过同源校验却已创建测量控制器，失败阶段不对";
    }
}

TEST(SystemInitializerTest, 场景S_仿真模式拒绝混入真实部件)
{
    // B2 的一致性校验：`run_mode=simulation` 只允许纯虚拟部件。
    // 否则结果包会写 "simulation"，而实际采集用了真实相机/触发/转台 ——
    // 包的形态声明与采集条件不符，比没有 run_mode 更糟。
    //
    // 三个负例各自改写**一份**仿真配置（其余六份保持仿真形态），
    // 证明拒绝确实来自被改写的那一项，而不是配置目录本身不可用。
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    std::string modelErr;
    ASSERT_TRUE(writeSyntheticModelDir("sim_models_reject", false, modelErr))
        << modelErr;

    auto variantOf = [&](const char* file, const std::string& from,
                         const std::string& to, std::string& e) {
        const std::string dir = makeSimulationConfigDir("sim_models_reject", e);
        if (dir.empty())
        {
            return std::string();
        }
        const std::string path = dir + "/" + file;
        std::string text = readFile(path);
        if (text.empty())
        {
            e = std::string("读 ") + file + " 失败：" + path;
            return std::string();
        }
        text = replaceOnce(text, from, to);
        if (text.empty())
        {
            e = std::string(file) + " 未命中：" + from;
            return std::string();
        }
        if (!writeTextAt(path, text))
        {
            e = std::string("写 ") + file + " 失败：" + path;
            return std::string();
        }
        return dir;
    };

    // 1) 真实相机：把 CAM25 从 virtual 改回现场后端 "imv"。
    {
        std::string e;
        const std::string dir = makeSimulationConfigDir("sim_models_reject", e);
        ASSERT_FALSE(dir.empty()) << e;
        const std::string path = dir + "/camera.yaml";
        std::string text = readFile(path);
        ASSERT_FALSE(text.empty()) << "读 camera.yaml 失败";
        text = replaceInCameraRecord(text, "cam25", "backend",
                                     "      backend: \"imv\"");
        ASSERT_FALSE(text.empty()) << "camera.yaml 的 cam25.backend 未命中";
        // 真实后端必须先通过"按序列号绑定"这一关，否则启动会在配置校验
        // 阶段就失败，走不到 run_mode 的一致性校验。这里给一个**不会
        // 真的去连设备**的占位序列号（校验只要求非空）。
        text = replaceInCameraRecord(text, "cam25", "serial",
                                     "      serial: \"SN-SIM-REJECT\"");
        ASSERT_FALSE(text.empty()) << "camera.yaml 的 cam25.serial 未命中";
        ASSERT_TRUE(writeTextAt(path, text));

        Harness h;
        EXPECT_FALSE(h.build(dir)) << "仿真模式混入真实相机仍启动成功";
        EXPECT_NE(h.init->errorText().find("全部为虚拟后端"), std::string::npos)
            << h.init->errorText();
    }

    // 2) 硬触发：trigger.source 改为 hardware。
    {
        std::string e;
        const std::string dir =
            variantOf("trigger.yaml", "source: \"virtual\"",
                      "source: \"hardware\"", e);
        ASSERT_FALSE(dir.empty()) << e;

        Harness h;
        EXPECT_FALSE(h.build(dir)) << "仿真模式混入硬触发仍启动成功";
        EXPECT_NE(h.init->errorText().find("trigger.source"), std::string::npos)
            << h.init->errorText();
    }

    // 3) 真实转台：protocol 改为 pelco_d_udp。
    {
        std::string e;
        const std::string dir =
            makeSimulationConfigDir("sim_models_reject", e);
        ASSERT_FALSE(dir.empty()) << e;
        const std::string path = dir + "/turntable.yaml";
        std::string text = readFile(path);
        ASSERT_FALSE(text.empty()) << "读 turntable.yaml 失败";
        text = replaceOnce(text, "protocol: \"sdk\"",
                           "protocol: \"pelco_d_udp\"");
        ASSERT_FALSE(text.empty()) << "turntable.yaml 的 protocol 未命中";
        // pelco_d_udp 的配置校验要求给出 host；补一个本机占位地址，
        // 使启动走到 run_mode 校验而不是先被"缺 host"拦下。
        text = replaceOnce(text, "center_threshold: 50.0",
                           "center_threshold: 50.0\n"
                           "   host: \"127.0.0.1\"\n"
                           "   port: 9\n"
                           "   pelco_address: 1");
        ASSERT_FALSE(text.empty()) << "turntable.yaml 的 center_threshold 未命中";
        ASSERT_TRUE(writeTextAt(path, text));

        Harness h;
        EXPECT_FALSE(h.build(dir)) << "仿真模式混入真实转台仍启动成功";
        EXPECT_NE(h.init->errorText().find("turntable.protocol"),
                  std::string::npos)
            << h.init->errorText();
    }
}

TEST(SystemInitializerTest, 场景S_机型库不是合成几何时拒绝启动)
{
    // 同源校验：仿真模式下加载的模型必须由 `SyntheticTargetGeometry.h`
    // 生成。只把 model_id 改掉、点表不动，证明拒绝来自标识校验本身，
    // 而不是模型文件读不出来。
    std::string envErr;
    ASSERT_TRUE(prepareWorkspace(envErr)) << envErr;

    std::string modelErr;
    ASSERT_TRUE(
        writeSyntheticModelDir("sim_models_badid", false, modelErr))
        << modelErr;

    const std::string modelPath = "sim_models_badid/model.yaml";
    std::string text = readFile(modelPath);
    ASSERT_FALSE(text.empty()) << "读 model.yaml 失败";
    text = replaceOnce(text,
                       std::string("model_id: \"") +
                           aircraft::data::synthetic::modelId() + "\"",
                       "model_id: \"production_aircraft\"");
    ASSERT_FALSE(text.empty()) << "model.yaml 的 model_id 未命中";
    ASSERT_TRUE(writeTextAt(modelPath, text));

    std::string cfgErr;
    const std::string simDir =
        makeSimulationConfigDir("sim_models_badid", cfgErr);
    ASSERT_FALSE(simDir.empty()) << cfgErr;

    Harness h;
    EXPECT_FALSE(h.build(simDir)) << "机型库不是合成几何仍启动成功";
    EXPECT_NE(h.init->errorText().find("画面与模型点表不同源"),
              std::string::npos)
        << h.init->errorText();
}

}  // namespace
