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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "app/ApplicationContext.h"
#include "app/SystemInitializer.h"
#include "data/ErrorInfo.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"
#include "data/MonotonicClock.h"
#include "data/TurntableConfig.h"
#include "device/camera/VirtualCameraBackend.h"
#include "infrastructure/FileUtil.h"
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

}  // namespace
