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

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "app/ApplicationContext.h"
#include "app/SystemInitializer.h"
#include "data/DeviceIdentity.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"
#include "data/MonotonicClock.h"
#include "data/TurntableConfig.h"
#include "device/camera/ICameraBackend.h"
#include "device/camera/VirtualCameraBackend.h"
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
        if (!copyFile(src, cfgDir + "/" + f))
        {
            err = "拷贝失败（APS_SOURCE_DIR=" + std::string(APS_SOURCE_DIR) + "）：" + src;
            std::fprintf(stderr, "[SystemInitializerTest] 环境准备失败：%s\n", err.c_str());
            return false;
        }
    }

    std::fprintf(stderr,
                 "\n[SystemInitializerTest] 临时工作目录：%s —— 仓库 config/ 的 7 个 yaml "
                 "已拷入（未重写任何取值）；logs/ output/ runtime/ 均落在该目录，"
                 "仓库目录不被污染\n",
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
    bool build()
    {
        init = std::make_unique<SystemInitializer>(ctx);
        return init->initialize("config") && init->ready() && ctx.controller != nullptr;
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

    h.ctx.turntable->setInitialAngles(tcfg.azimuthMax, 0.0);
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
                                     std::string&       err)
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

TEST(SystemInitializerTest, 场景C_取帧预算键被读到)
{
    // ---- 判别式用例：两个键都改成与结构体默认值**不同**的值 ----
    //
    // ⚠ 为什么不能直接断仓库现值：仓库现值（100 / 3.0e8）与
    //    `MeasurementConfig` 的默认值**完全相同** ⇒ "解析了"与
    //    "压根没解析、全程吃默认值"两种情形给出同一个断言结果，
    //    这正是一个看不出差别的用例。故先改成别的数再断。
    std::string text = replaceOnce(repoMeasurementYaml(), "grab_timeout_ms: 100",
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
    // 仓库现值（100 / 3.0e8）与默认值相同，故本用例**不**用它们证明解析；
    // 它证明的是另一件事：**当前这份配置的余量事实不得静默**。
    // 5 帧 × 3 路 × 100 ms = 1500 ms = capture_timeout_ns，且未计入
    // 复制/格式转换/评分 ⇒ CAPTURE 在真实相机上可能被时限截断。
    // ⚠ 只警告、不判启动失败 —— 取值余量属《待裁决问题汇总》Q-D2，
    //   本批不自行放宽冻结值。
    aircraft::infrastructure::ConfigManager cfg;
    ASSERT_TRUE(cfg.load(std::string(APS_SOURCE_DIR) + "/config"))
        << "仓库 config/ 加载失败";

    EXPECT_EQ(cfg.measurement().grabTimeoutMs, 100);
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
    // 只把 capture_timeout_ns 放宽到 3.0 s（另两项不动，
    // 5 × 3 × 100 = 1500 ms < 3000 ms ⇒ 有余量），告警必须消失。
    // 缺了这条用例，把告警写成"无条件 push"也能让上一条用例通过。
    const std::string text = replaceOnce(repoMeasurementYaml(),
                                         "capture_timeout_ns: 1.5e9",
                                         "capture_timeout_ns: 3.0e9");
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
    EXPECT_EQ(cfg.measurement().captureTimeoutNs, 3000000000ULL);
}

TEST(SystemInitializerTest, 场景C_grab_timeout_ms为零即启动失败)
{
    // `IMV_GetFrame` 对 timeoutMS = 0 的语义在 SDK 中**未文档化**
    // （ENG-09 V2.3 §2.5），项目不定义它、后端会拒绝 0 ⇒ 配置层必须拦。
    // 若此处只是"取默认值"或"静默接受"，故障会推迟到第一次取帧才出现，
    // 且表现为"相机取不到帧"这种把人引向设备侧的假象。
    const std::string text =
        replaceOnce(repoMeasurementYaml(), "grab_timeout_ms: 100", "grab_timeout_ms: 0");
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
std::string imvCam25ConfigDir(std::string& err)
{
    std::string text = replaceOnce(repoYaml("camera.yaml"),
                                   "backend: \"virtual\"", "backend: \"imv\"");
    if (text.empty())
    {
        err = "未命中 cam25 的 backend 行，本用例的底本已失效";
        return std::string();
    }
    text = replaceOnce(text, "serial: \"\"", "serial: \"SN-CAM25-TEST-0001\"");
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
    std::string text = replaceOnce(repoYaml("camera.yaml"),
                                   "      backend: \"virtual\"\n", "");
    ASSERT_FALSE(text.empty())
        << "未命中 cam25 的 backend 行（缩进或写法已变），本用例的底本已失效";

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

}  // namespace
