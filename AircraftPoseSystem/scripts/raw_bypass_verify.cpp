// ============================================================================
//  scripts/raw_bypass_verify.cpp
//
//  RAW 落盘与解码的**旁路实机验证工具**（011-A1 九项缺口 §4 之后的实机项；
//  用户 2026-09-28 裁决："RAW：旁路实机验证＋Mono10 垫片测试"）。
//
// ---------------------------------------------------------------------------
//  ⚠ 这是什么、不是什么
//
//  它**不是**一次测量结果包，而是**专项验证产物**：
//    只验证"真实相机交付的原始载荷 → Recorder → cam25.raw + result.json
//    元数据 → 离线解码"这一条链，**不经过**状态机、不解算姿态。
//    ∴ 包里的 `task_id` 以 `RAW_BYPASS_` 开头、`success=false`、
//      `calibration_id`/`model_id` 都带 `RAW_BYPASS_NO_*` 前缀，
//      没有任何姿态数值 —— 必须一眼看不出这是测量结果。
//
//  ⚠ 路径纪律（用户裁决："不要另写 RAW 写入器，也不要手工拼一份元数据"）：
//    本工具**只调用生产代码**：真实 `ImvCameraBackend` → `data::ImageFrame`
//    → 生产版 `Recorder::save()`。落盘、元数据、校验全部由 Recorder 完成，
//    工具自己**不写字节、不拼 JSON**（只读回来核对）。
//
// ---------------------------------------------------------------------------
//  编译（本工具**不进 CMake 目标**：它是验证工具，不该进交付物；且
//  GoogleTest 未安装时 tests/ 在 CMake 里被短路，本工具的验证对象是实机）
//
//    cd AircraftPoseSystem
//    g++ -std=c++17 -O0 -g -fPIC \
//        -DAPS_VERSION_MAJOR=2 -DAPS_VERSION_MINOR=1 \
//        -DAPS_VERSION_STRING='"2.1.0"' -DAPS_VERSION_STAGE='"v2.1-framework"' \
//        -I src -I third_party/imvsdk/include -I third_party/imvsdk/include/GenICam \
//        scripts/raw_bypass_verify.cpp -o /tmp/raw_bypass_verify \
//        -L build/lib -linfrastructure -ldevice_camera -ldevice_trigger -ldata \
//        -L build/imvsdk_runtime -lMVSDK \
//        -Wl,-rpath,$PWD/build/imvsdk_runtime \
//        $(pkg-config --cflags --libs opencv4)
//
//  运行（**必须从 AircraftPoseSystem/ 下**：config 与 output 都按相对路径解析）
//
//    DISPLAY= LD_LIBRARY_PATH=$PWD/build/imvsdk_runtime \
//      /tmp/raw_bypass_verify config /tmp/raw_bypass_out
//
//    ⚠ `LD_LIBRARY_PATH` 不能省：`-Wl,-rpath` 生成的是 RUNPATH，而 RUNPATH
//      **不传递给孙依赖**（ENG-09 记录的 SDK 三条硬约束第 ① 条），
//      只设 rpath 会在启动时才失败。
//
//  退出码：0 = 全部核对项通过；1 = 有核对项失败（逐项列在输出里）。
// ============================================================================

#include "data/CameraConfig.h"
#include "data/CameraRole.h"
#include "data/CameraTriggerMode.h"
#include "data/DeviceIdentity.h"
#include "data/ImageFrame.h"
#include "data/MeasurementRecord.h"
#include "data/OpStatus.h"
#include "data/PixelFormat.h"
#include "data/RawImagePayload.h"
#include "data/SystemConfig.h"
#include "device/camera/IImvApi.h"
#include "device/camera/ImvCameraBackend.h"
#include "infrastructure/config/ConfigManager.h"
#include "infrastructure/recorder/Recorder.h"

#include <opencv2/core.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{

using aircraft::data::OpStatus;
using aircraft::data::PixelFormat;

// ---------------------------------------------------------------------------
//  核对项的记账
// ---------------------------------------------------------------------------

/// 逐项记账：每条核对打印一行，最后汇总。
///
/// ⚠ 为什么要记账而不是"中途 return false"：本工具的产出是一份**证据**，
///   证据的价值在于"哪几条通过、哪几条没通过"同时可见。中途返回会让
///   后面几条根本没被检查，而报告上看起来像是"没出问题"。
struct Report
{
    int passed = 0;
    int failed = 0;

    void check(bool ok, const std::string& what, const std::string& detail = "")
    {
        if (ok) { ++passed; }
        else    { ++failed; }
        std::cout << (ok ? "  [通过] " : "  [**不通过**] ") << what;
        if (!detail.empty()) { std::cout << "\n           " << detail; }
        std::cout << "\n";
    }

    void note(const std::string& line) const
    {
        std::cout << "  [证据] " << line << "\n";
    }
};

/// 包内元数据里"三个长度字段"加实际文件长度，逐项记录。
struct LengthFacts
{
    uint64_t recomputed       = 0;   ///< 由**包内**宽高与格式用生产函数重算
    uint64_t expectedCompact  = 0;   ///< 包内 expected_compact_bytes
    uint64_t sdkPayload       = 0;   ///< 包内 sdk_payload_bytes
    uint64_t fileBytes        = 0;   ///< 实际 cam25.raw 的字节数
    bool     allFourEqual     = false;
};

// ---------------------------------------------------------------------------
//  小工具
// ---------------------------------------------------------------------------

const char* pixelFormatText(PixelFormat f)
{
    switch (f)
    {
    case PixelFormat::Mono8:        return "Mono8";
    case PixelFormat::Mono10:       return "Mono10";
    case PixelFormat::Mono12:       return "Mono12";
    case PixelFormat::Mono12Packed: return "Mono12Packed";
    case PixelFormat::BGR8:         return "BGR8";
    }
    return "未知";
}

const char* roleText(aircraft::data::CameraRole r)
{
    using aircraft::data::CameraRole;
    switch (r)
    {
    case CameraRole::CAM25:  return "CAM25";
    case CameraRole::CAM50:  return "CAM50";
    case CameraRole::CAM100: return "CAM100";
    }
    return "未知";
}

std::string sdkFailureText(const std::optional<aircraft::data::SdkFailure>& f)
{
    if (!f) { return "（无）"; }
    std::ostringstream os;
    os << aircraft::data::sdkCallName(f->call) << " 返回 " << f->code;
    return os.str();
}

/// FNV-1a 64 位。**不是**密码学散列，只用于"两处字节是否同一份"的指纹，
/// 便于人眼比对与贴进记录；真正的判据始终是下面的逐字节比较。
uint64_t fnv1a64(const std::vector<uint8_t>& bytes)
{
    uint64_t h = 1469598103934665603ULL;
    for (const uint8_t b : bytes)
    {
        h ^= static_cast<uint64_t>(b);
        h *= 1099511628211ULL;
    }
    return h;
}

std::vector<uint8_t> readFileBytes(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
}

std::string joinPath(const std::string& a, const std::string& b)
{
    if (a.empty()) { return b; }
    if (a.back() == '/') { return a + b; }
    return a + "/" + b;
}

// ---------------------------------------------------------------------------
//  带记账的 SDK 访问层（装饰器）
// ---------------------------------------------------------------------------
//
//  ⚠ 为什么用装饰器包住真实 API，而不是"直接调 SDK 再自己数"：
//    "释放恰好一次"、"取帧时交付缓冲的地址不是我们保存的地址"这两条
//    证据必须来自**后端真实走过的调用**，不能在工具里另起一套调用
//    （那测的就是工具自己的调用序列，与被验证的链路无关）。
//    装饰器对后端完全透明：它只是在每次真实调用前后记一笔。
class SpyApi : public aircraft::device::IImvApi
{
public:
    explicit SpyApi(std::shared_ptr<aircraft::device::IImvApi> inner)
        : inner_(std::move(inner)) {}

    // ---- 记账 ----
    int getFrameCalls     = 0;
    int releaseFrameCalls = 0;
    int triggerCalls      = 0;
    int startGrabbingCalls = 0;

    /// 最近一次 `getFrame` 交付的 **SDK 缓冲地址**。
    /// 与"我们保存的载荷地址"必须不同 —— 这是"复制发生在释放之前"的
    /// 可核对形式（同一地址只可能说明我们把 SDK 缓冲当成了自有数据）。
    const unsigned char* lastDeliveredBuffer = nullptr;

    /// `IMV_ReleaseFrame` 之后，那份缓冲**是否已被 SDK 复用**不作为判据
    /// （未文档化）；我们只如实记下"释放发生在何时"，由工具断言次序。
    int releaseCallsAtSnapshot = 0;

    void* deviceHandle() const { return handle_; }

    /// 独立读回一项特性（**不是**读我们自己的配置结构体）。
    int readbackEnum(const char* feature, std::string& out)
    {
        if (!handle_) { return -1; }
        return inner_->getEnumFeatureSymbol(handle_, feature, out);
    }

    int readbackDouble(const char* feature, double& out)
    {
        if (!handle_) { return -1; }
        return inner_->getDoubleFeatureValue(handle_, feature, out);
    }

    // ---- IImvApi ----
    int enumDevices(std::vector<aircraft::device::ImvDeviceEntry>& out) override
    {
        return inner_->enumDevices(out);
    }

    int createHandle(void*& handle, aircraft::device::ImvHandleMode mode,
                     void* identifier) override
    {
        const int r = inner_->createHandle(handle, mode, identifier);
        if (r == kOk) { created_ = handle; }
        return r;
    }

    int open(void* handle) override
    {
        const int r = inner_->open(handle);
        // 读回只在句柄打开之后才可能成功；`open` 是唯一能确定"打开了"的点。
        if (r == kOk) { handle_ = handle; }
        return r;
    }

    int close(void* handle) override
    {
        const int r = inner_->close(handle);
        if (handle_ == handle) { handle_ = nullptr; }
        return r;
    }

    int destroyHandle(void* handle) override
    {
        const int r = inner_->destroyHandle(handle);
        if (created_ == handle) { created_ = nullptr; }
        return r;
    }

    int getDeviceInfo(void* handle,
                      aircraft::device::ImvDeviceEntry& out) override
    {
        return inner_->getDeviceInfo(handle, out);
    }

    int setEnumFeatureSymbol(void* handle, const char* feature,
                            const char* symbol) override
    {
        return inner_->setEnumFeatureSymbol(handle, feature, symbol);
    }

    int getEnumFeatureSymbol(void* handle, const char* feature,
                             std::string& out) override
    {
        return inner_->getEnumFeatureSymbol(handle, feature, out);
    }

    int setIntFeatureValue(void* handle, const char* feature,
                           int64_t value) override
    {
        return inner_->setIntFeatureValue(handle, feature, value);
    }

    int getIntFeatureValue(void* handle, const char* feature,
                           int64_t& out) override
    {
        return inner_->getIntFeatureValue(handle, feature, out);
    }

    int setDoubleFeatureValue(void* handle, const char* feature,
                              double value) override
    {
        return inner_->setDoubleFeatureValue(handle, feature, value);
    }

    int getDoubleFeatureValue(void* handle, const char* feature,
                              double& out) override
    {
        return inner_->getDoubleFeatureValue(handle, feature, out);
    }

    int executeCommandFeature(void* handle, const char* feature) override
    {
        ++triggerCalls;
        return inner_->executeCommandFeature(handle, feature);
    }

    int startGrabbing(void* handle) override
    {
        ++startGrabbingCalls;
        return inner_->startGrabbing(handle);
    }

    int stopGrabbing(void* handle) override
    {
        return inner_->stopGrabbing(handle);
    }

    int getFrame(void* handle, aircraft::device::ImvFrameView& out,
                 unsigned int timeoutMs) override
    {
        ++getFrameCalls;
        const int r = inner_->getFrame(handle, out, timeoutMs);
        if (r == kOk) { lastDeliveredBuffer = out.data; }
        return r;
    }

    int releaseFrame(void* handle) override
    {
        ++releaseFrameCalls;
        return inner_->releaseFrame(handle);
    }

    static constexpr int kOk = 0;

private:
    std::shared_ptr<aircraft::device::IImvApi> inner_;
    void* handle_  = nullptr;   ///< 由 `open` 确立
    void* created_ = nullptr;   ///< 由 `createHandle` 确立
};

// ---------------------------------------------------------------------------
//  解码器：按**包内元数据**给出的几何把 cam25.raw 还原成二维图像
// ---------------------------------------------------------------------------

/// 一次解码的统计结果。
///
/// ⚠ 为什么要有"与显示图不一致的像素数"这一个量：
///   只报"解出来了"与"文件长度对"都不能说明**这个解码方式是对的** ——
///   对一份 24 MB 的缓冲，任何长度相符的读法都能跑完。
///   把"按某种位移/字节序解码后与交付的显示图不同的像素数"报出来，
///   就同时得到**正对照**（正确的那种必须为 0）与**判别力证据**
///   （错误的那几种必须大面积不为 0）。只有这样，
///   "显示图验证 >>2"才不是一句自证的话。
struct DecodeStat
{
    uint64_t mismatchVsDisplay = 0;   ///< 与显示图不一致的像素数
    uint16_t minValue          = 0;
    uint16_t maxValue          = 0;
    uint64_t above8BitCount    = 0;   ///< 值 > 255 的像素数
    double   meanValue         = 0.0;
};

DecodeStat decodeAgainst(const std::vector<uint8_t>& raw,
                         const cv::Mat&              display,
                         uint32_t                    width,
                         uint32_t                    height,
                         int                         shift,
                         bool                        bigEndian)
{
    DecodeStat st;
    const uint64_t pixels = static_cast<uint64_t>(width) * height;
    if (raw.size() < pixels * 2u || display.empty())
    {
        st.mismatchVsDisplay = ~static_cast<uint64_t>(0);   // 无法解码
        return st;
    }

    uint64_t sum = 0;
    uint16_t mn = 0xFFFFu;
    uint16_t mx = 0;
    for (uint64_t i = 0; i < pixels; ++i)
    {
        const uint8_t lo = raw[static_cast<std::size_t>(i) * 2u];
        const uint8_t hi = raw[static_cast<std::size_t>(i) * 2u + 1u];
        const uint16_t v = bigEndian
                               ? static_cast<uint16_t>((lo << 8) | hi)
                               : static_cast<uint16_t>(lo | (hi << 8));

        const int y = static_cast<int>(i / width);
        const int x = static_cast<int>(i % width);
        if ((static_cast<int>(v) >> shift) !=
            static_cast<int>(display.at<unsigned char>(y, x)))
        {
            ++st.mismatchVsDisplay;
        }
        if (v < mn) { mn = v; }
        if (v > mx) { mx = v; }
        if (v > 255u) { ++st.above8BitCount; }
        sum += v;
    }
    st.minValue = mn;
    st.maxValue = mx;
    st.meanValue = pixels ? static_cast<double>(sum) / static_cast<double>(pixels)
                          : 0.0;
    return st;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string configDir = (argc > 1) ? argv[1] : "config";
    const std::string outDir    = (argc > 2) ? argv[2] : "/tmp/raw_bypass_out";

    Report rep;

    std::cout << "============================================================\n";
    std::cout << " RAW 旁路实机验证（**专项验证产物**，不是一次测量结果包）\n";
    std::cout << " 链路：真实 ImvCameraBackend → ImageFrame → 生产版 "
                 "Recorder::save() → 离线解码\n";
    std::cout << "============================================================\n\n";

    // ---- 0. 配置：工作点从**随包发布的 yaml** 读，不在本文件里另写一份 ----
    std::cout << "── 0. 配置与工作点 ─────────────────────────────────────\n";
    aircraft::infrastructure::ConfigManager cfg;
    if (!cfg.load(configDir))
    {
        std::cout << "  config 加载失败（" << configDir << "）：\n";
        for (const std::string& e : cfg.errors()) { std::cout << "    " << e << "\n"; }
        return 1;
    }
    const aircraft::data::CameraConfig cam25 =
        cfg.camera(aircraft::data::CameraRole::CAM25);

    rep.note("配置目录：" + configDir);
    rep.note(std::string("代码版本：") + APS_VERSION_STRING + "-" +
             APS_VERSION_STAGE);
    rep.note("通道 " + cam25.cameraId + " / 角色 " + roleText(cam25.role) +
             " / 后端 " + cam25.backend + " / 序列号 " + cam25.serialNumber);
    {
        std::ostringstream os;
        os << "配置工作点：曝光 " << (cam25.exposureTime * 1000.0) << " ms"
           << "，增益 " << (cam25.gainRaw ? "gain_raw=" +
                                std::to_string(*cam25.gainRaw)
                                          : std::string("（未配置）"))
           << "，触发 " << aircraft::data::triggerModeName(cam25.triggerMode)
           << "，画幅 " << cam25.width << "x" << cam25.height
           << "，取帧时限 " << cfg.measurement().grabTimeoutMs << " ms";
        rep.note(os.str());
    }
    // 工作点核对：本次运行**必须**是验收工作点，否则这份证据不适用。
    rep.check(cam25.exposureTime == 0.05,
              "曝光 = 50 ms（当前验收工作点）",
              "实际 " + std::to_string(cam25.exposureTime * 1000.0) +
                  " ms —— 工作点变了就必须换一份记录，不得沿用本报告的结论");
    rep.check(cam25.gainRaw.has_value() && *cam25.gainRaw == 1.0,
              "gain_raw = 1（当前验收工作点）",
              cam25.gainRaw ? "实际 " + std::to_string(*cam25.gainRaw)
                            : std::string("未配置"));
    rep.check(cam25.backend == "imv", "该通道显式声明为真实后端（imv）",
              "实际 backend = " + cam25.backend);

    // ---- 1. 真实后端 + 记账装饰器 ----
    std::cout << "\n── 1. 真实 ImvCameraBackend ────────────────────────────\n";
    const std::shared_ptr<aircraft::device::IImvApi> real =
        aircraft::device::makeRealImvApi();
    if (!real)
    {
        std::cout << "  makeRealImvApi() 返回空：本构建未接入 SDK。\n"
                     "  本工具必须在带 SDK 的构建上运行。\n";
        return 1;
    }
    const std::shared_ptr<SpyApi> spy = std::make_shared<SpyApi>(real);
    aircraft::device::ImvCameraBackend backend(cam25, spy);

    const aircraft::data::OperationResult init = backend.initialize();
    rep.check(init.ok(), "backend->initialize() 成功",
              std::string("status=") + aircraft::data::opStatusName(init.status) +
                  "，sdkError=" + sdkFailureText(init.sdkError) +
                  "，清理=" + sdkFailureText(init.cleanupError) +
                  "，文本=" + backend.lastErrorText());
    if (!init.ok()) { return 1; }

    {
        const aircraft::data::DeviceIdentity id = backend.deviceIdentity();
        rep.note(std::string("设备身份：型号 ") +
                 (id.modelName.empty() ? "（未取得）" : id.modelName) +
                 "，序列号 " +
                 (id.serialNumber ? *id.serialNumber
                                  : std::string("（未取得）")) +
                 (id.queried ? "（已查询）" : "（未查询）"));
        rep.check(id.queried && id.serialNumber && !id.serialNumber->empty(),
                  "设备身份来自 SDK 查询（不是配置回抄）",
                  "实际序列号 " +
                      (id.serialNumber ? *id.serialNumber : std::string("（未取得）")));
        rep.check(id.serialNumber && *id.serialNumber == cam25.serialNumber,
                  "实际序列号与配置的绑定目标一致",
                  "配置期望 " + cam25.serialNumber);
    }

    // ---- 2. 格式/曝光/增益的**独立读回**（不读我们自己的结构体）----
    std::cout << "\n── 2. 设备侧读回（独立于本项目的数据结构）─────────────\n";
    {
        std::string fmt;
        const int rFmt = spy->readbackEnum("PixelFormat", fmt);
        rep.note("IMV_GetEnumFeatureSymbol(PixelFormat) 返回 " + std::to_string(rFmt) +
                 "，读回 \"" + fmt + "\"");
        rep.check(rFmt == 0 && fmt == "Mono10",
                  "设备侧 PixelFormat 读回 = Mono10",
                  "本机型只提供 Mono8 / Mono10 / Mono10Packed");

        double expUs = -1.0;
        const int rExp = spy->readbackDouble("ExposureTime", expUs);
        rep.note("IMV_GetDoubleFeatureValue(ExposureTime) 返回 " +
                 std::to_string(rExp) + "，读回 " + std::to_string(expUs) + " µs");
        rep.check(rExp == 0 && expUs == 50000.0,
                  "设备侧 ExposureTime 读回 = 50000 µs",
                  "实际 " + std::to_string(expUs) + " µs");

        double gain = -1.0;
        const int rGain = spy->readbackDouble("GainRaw", gain);
        rep.note("IMV_GetDoubleFeatureValue(GainRaw) 返回 " +
                 std::to_string(rGain) + "，读回 " + std::to_string(gain));
        rep.check(rGain == 0 && gain == 1.0,
                  "设备侧 GainRaw 读回 = 1（本项目未改变设备增益设定）",
                  "实际 " + std::to_string(gain));

        int64_t w = -1, h = -1;
        const int rW = spy->getIntFeatureValue(spy->deviceHandle(), "Width", w);
        const int rH = spy->getIntFeatureValue(spy->deviceHandle(), "Height", h);
        rep.note("Width/Height 读回 " + std::to_string(w) + "x" +
                 std::to_string(h) + "（返回 " + std::to_string(rW) + "/" +
                 std::to_string(rH) + "）");
    }

    // ---- 3. 启动采集 → 触发前置读回 → 发令 → 取帧 ----
    //         调用次序与 `MultiCameraManager::capture()` **逐句一致**：
    //         initialize → start → triggerModeState → (Software) triggerSoftware
    //         → grab。旁路的意义是绕开状态机，**不是**换一套调用方式。
    std::cout << "\n── 3. 取帧（次序同 MultiCameraManager::capture）─────────\n";
    const aircraft::data::OperationResult started = backend.start();
    rep.check(started.ok(), "backend->start() 成功",
              std::string("status=") + aircraft::data::opStatusName(started.status) +
                  "，文本=" + backend.lastErrorText());
    if (!started.ok()) { return 1; }

    {
        const aircraft::data::TriggerModeState tms = backend.triggerModeState();
        for (const auto& rb : tms.readbacks)
        {
            std::ostringstream os;
            os << "触发读回 " << (rb.feature.empty() ? "（未命名）" : rb.feature)
               << "：调用过=" << (rb.callAttempted ? "是" : "否")
               << "，返回空串=" << (rb.returnedEmpty ? "是" : "否")
               << "，失败=" << sdkFailureText(rb.failure)
               << "，取值=\"" << rb.value << "\"";
            rep.note(os.str());
        }
        rep.check(tms.reported.has_value(),
                  "三项触发前置特性读回完整（可合成实际生效的模式）",
                  "合成结果 = " +
                      (tms.reported
                           ? std::string(aircraft::data::triggerModeName(*tms.reported))
                           : std::string("未知")));
        rep.check(tms.reported.has_value() && tms.consistent,
                  "读回与请求一致（软件触发）");
    }

    const int releaseBeforeGrab = spy->releaseFrameCalls;
    {
        const aircraft::data::OperationResult trig = backend.triggerSoftware();
        rep.check(trig.ok(), "软件触发令发送成功",
                  std::string("status=") +
                      aircraft::data::opStatusName(trig.status) +
                      "，sdkError=" + sdkFailureText(trig.sdkError));
        if (!trig.ok()) { return 1; }
    }
    rep.check(spy->triggerCalls == 1, "本轮恰好发一次软件触发令",
              "实际 " + std::to_string(spy->triggerCalls) + " 次");

    aircraft::data::ImageFrame frame;
    const aircraft::data::GrabResult grabbed =
        backend.grab(frame, cfg.measurement().grabTimeoutMs);
    rep.check(grabbed.ok(), "backend->grab() 成功",
              std::string("status=") +
                  aircraft::data::opStatusName(grabbed.status) +
                  "，sdkError=" + sdkFailureText(grabbed.sdkError) +
                  "，清理=" + sdkFailureText(grabbed.cleanupError) +
                  "，诊断=" + grabbed.diagnosis);
    if (!grabbed.ok()) { return 1; }

    rep.note("GetFrame 调用 " + std::to_string(spy->getFrameCalls) +
             " 次，ReleaseFrame 调用 " +
             std::to_string(spy->releaseFrameCalls) + " 次（取帧前 " +
             std::to_string(releaseBeforeGrab) + " 次）");
    rep.check(spy->getFrameCalls == 1 && spy->releaseFrameCalls == 1,
              "取帧恰好一次、释放恰好一次",
              "取帧 " + std::to_string(spy->getFrameCalls) + " 次，释放 " +
                  std::to_string(spy->releaseFrameCalls) + " 次");
    rep.check(grabbed.frameStatusRaw.has_value() && *grabbed.frameStatusRaw == 0u,
              "SDK 帧状态原值 = 0 且**取得**（不是 nullopt）",
              grabbed.frameStatusRaw
                  ? "原值 " + std::to_string(*grabbed.frameStatusRaw)
                  : std::string("nullopt（未取得）"));

    // ---- 4. 自有载荷：地址与 SDK 交付缓冲不同，且释放已完成 ----
    std::cout << "\n── 4. 自有载荷（复制发生在释放之前）────────────────────\n";
    if (!frame.raw.bytes || frame.raw.bytes->empty())
    {
        std::cout << "  帧未携带原始载荷 —— 本工具无法验证 RAW 通路。\n";
        return 1;
    }
    rep.note("SDK 交付缓冲地址：" +
             [&] {
                 std::ostringstream os;
                 os << static_cast<const void*>(spy->lastDeliveredBuffer);
                 return os.str();
             }());
    rep.note("本项目保存的载荷地址：" +
             [&] {
                 std::ostringstream os;
                 os << static_cast<const void*>(frame.raw.bytes->data())
                    << "（长度 " << frame.raw.bytes->size() << " 字节）";
                 return os.str();
             }());
    rep.check(spy->lastDeliveredBuffer != frame.raw.bytes->data(),
              "保存的载荷**不是** SDK 交付缓冲那块内存（自有副本）",
              "同地址只可能说明我们把 SDK 缓冲当成了自有数据");
    rep.check(spy->releaseFrameCalls == 1,
              "载荷快照时帧**已释放**（释放次数已为 1）");
    rep.check(frame.raw.format == frame.captureFormat,
              "raw.format 与 captureFormat 一致（后端发布契约）",
              std::string("raw.format=") + pixelFormatText(frame.raw.format) +
                  "，captureFormat=" + pixelFormatText(frame.captureFormat));
    rep.check(frame.role == aircraft::data::CameraRole::CAM25,
              "帧自带 role = CAM25（与第 0 槽位一致，Recorder 会校验）",
              std::string("实际 ") + roleText(frame.role));

    {
        std::ostringstream os;
        os << "帧事实：frame_id=" << frame.frameId
           << "，timestamp_ns=" << frame.timestampNs
           << "，显示图 " << frame.image.cols << "x" << frame.image.rows
           << "（type=" << frame.image.type() << "）"
           << "，原始载荷 " << frame.raw.width << "x" << frame.raw.height
           << " / " << pixelFormatText(frame.raw.format)
           << " / valid_bits=" << frame.raw.validBits
           << " / sdk_payload_bytes=" << frame.raw.sdkPayloadBytes
           << " / expected_compact_bytes=" << frame.raw.expectedCompactBytes
           << " / compact_size_matches="
           << (frame.raw.compactSizeMatches ? "true" : "false")
           << " / sdk_pixel_format_code=" << frame.raw.sdkPixelFormatCode
           << " / padding_x=" << frame.raw.sdkPaddingX
           << " / padding_y=" << frame.raw.sdkPaddingY;
        rep.note(os.str());
    }

    // ---- 5. 交给**生产版 Recorder** 落盘 ----
    //         ⚠ 不跑状态机 ⇒ 没有任何终态是"真的"。取 COMPLETE 而非 FAILED：
    //           FAILED 会伪造一次故障，而"本次没有产出姿态"已由
    //           success=false 与 task_id 的 RAW_BYPASS_ 前缀如实表达。
    //           `calibration_id`/`model_id` 用 RAW_BYPASS_NO_* 前缀，
    //           使这份包**不可能**被误当成一次有效测量的结果。
    std::cout << "\n── 5. 生产版 Recorder::save() ─────────────────────────\n";
    aircraft::data::MeasurementRecord record;
    record.task.taskId = "RAW_BYPASS_" + cam25.serialNumber;
    record.task.state  = aircraft::data::MeasurementState::COMPLETE;
    record.task.result.success = false;   // 没有姿态结论，如实置假
    record.bestFrame.cam25     = std::move(frame);
    // cam50 / cam100 保持默认空帧 ⇒ 走"合法无帧占位"分支：
    // data_source=none、role 按槽位写。这也是本工具要顺带覆盖的一条分支。
    record.selectedCamera   = aircraft::data::CameraRole::CAM25;
    record.camerasAvailable = 1;      // 现场只接了 CAM25
    record.degraded         = true;   // 1 < 3 路，如实记
    record.calibrationId    = "RAW_BYPASS_NO_CALIBRATION";
    record.modelId          = "RAW_BYPASS_NO_MODEL";
    record.modelType        = "";     // C-003 的两类机型库尚未实施，留空
    record.softwareVersion  = std::string(APS_VERSION_STRING) + "-" +
                             APS_VERSION_STAGE;

    // 保存**之前**的原始载荷快照：落盘字节必须与它逐字节一致。
    const std::vector<uint8_t> beforeCopy = *record.bestFrame.cam25.raw.bytes;

    aircraft::data::SystemConfig sys;
    sys.outputDir = outDir;
    aircraft::infrastructure::Recorder recorder(sys, configDir,
                                                record.calibrationId,
                                                record.modelId, "RAW_BYPASS");
    const bool saved = recorder.save(record);
    rep.check(saved, "Recorder::save() 成功",
              "失败原因：" + recorder.lastErrorText());
    if (!saved) { return 1; }
    const std::string pkg = recorder.lastPackageDir();
    rep.note("结果包目录：" + pkg);

    // ---- 6. 核对：落盘字节 vs 保存前的自有载荷 ----
    std::cout << "\n── 6. 落盘字节 vs 保存前的自有载荷 ────────────────────\n";
    const std::string rawPath = joinPath(pkg, "cam25.raw");
    const std::vector<uint8_t> onDisk = readFileBytes(rawPath);
    {
        // ⚠ 两个长度都用十进制、只有散列用十六进制：混用进制会让
        //   "1770000 与 24576000 相等"这种误读发生（0x1770000 恰好
        //   就是 24576000）—— 一条给人看的证据行不能有两种进制。
        std::ostringstream os;
        os << "保存前载荷 " << beforeCopy.size() << " 字节 / FNV-1a 0x"
           << std::hex << std::uppercase << fnv1a64(beforeCopy) << std::dec
           << "；cam25.raw " << onDisk.size() << " 字节 / FNV-1a 0x"
           << std::hex << std::uppercase << fnv1a64(onDisk) << std::dec;
        rep.note(os.str());
    }
    rep.check(onDisk.size() == beforeCopy.size(),
              "cam25.raw 的文件长度 = 保存前载荷长度",
              "文件 " + std::to_string(onDisk.size()) + " 字节，载荷 " +
                  std::to_string(beforeCopy.size()) + " 字节");
    rep.check(onDisk == beforeCopy,
              "cam25.raw 与保存前的自有载荷**逐字节一致**",
              "（不是『长度相同即可』，是逐个字节比对）");
    rep.check(*record.bestFrame.cam25.raw.bytes == beforeCopy,
              "保存**不得**改动调用方的载荷（Recorder 只读）");

    // ---- 7. 核对：包内元数据 ----
    std::cout << "\n── 7. 包内元数据（从 result.json 读回）────────────────\n";
    cv::FileStorage fs(joinPath(pkg, "result.json"), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "  打不开 result.json —— 无法核对元数据。\n";
        return 1;
    }
    cv::FileNode f0 = fs["best_frame"]["frames"][0];

    // ⚠ 一律用 `int` 读：`cv::FileStorage` 的 `>>` 只重载了 int/float/double/
    //   string/Mat，**没有** `int64_t`。用它会在编译期就落到某个意外的
    //   重载上（或直接编不过）。本包这几个量（24.5 MB / 帧号）都装得下 int。
    std::string dataSource, pixelFormat, packing, alignment, byteOrder, role;
    int validBits = -1, pkgW = -1, pkgH = -1;
    int sdkPayload = -1, expectedCompact = -1, fileId = -1;
    f0["data_source"] >> dataSource;
    f0["role"] >> role;
    f0["pixel_format"] >> pixelFormat;
    f0["valid_bits"] >> validBits;
    f0["packing"] >> packing;
    f0["valid_bit_alignment"] >> alignment;
    f0["declared_byte_order"] >> byteOrder;
    f0["raw_image"]["width"] >> pkgW;
    f0["raw_image"]["height"] >> pkgH;
    f0["sdk_payload_bytes"] >> sdkPayload;
    f0["expected_compact_bytes"] >> expectedCompact;
    f0["frame_id"] >> fileId;

    rep.note("读回：role=\"" + role + "\"，data_source=\"" + dataSource +
             "\"，pixel_format=\"" + pixelFormat + "\"，valid_bits=" +
             std::to_string(validBits) + "，packing=\"" + packing +
             "\"，valid_bit_alignment=\"" + alignment +
             "\"，declared_byte_order=\"" + byteOrder + "\"");
    rep.check(dataSource == "raw", "data_source = \"raw\"（真身路径）");
    rep.check(role == "CAM25", "role = \"CAM25\"（按槽位写，不是帧自带的默认值）");
    rep.check(pixelFormat == "Mono10", "pixel_format = \"Mono10\"");
    rep.check(validBits == 10, "valid_bits = 10（解码位移由此推出：10−8 = 2）");
    rep.check(packing == "Unpacked", "packing = \"Unpacked\"");
    rep.check(alignment == "LsbZeroPadded", "valid_bit_alignment = \"LsbZeroPadded\"");
    rep.check(byteOrder == "LittleEndian", "declared_byte_order = \"LittleEndian\"");
    rep.check(fileId == static_cast<int>(record.bestFrame.cam25.frameId),
              "frame_id 与本次取到的帧一致",
              "包内 " + std::to_string(fileId) + "，帧 " +
                  std::to_string(record.bestFrame.cam25.frameId));
    rep.check(pkgW == static_cast<int>(record.bestFrame.cam25.raw.width) &&
                  pkgH == static_cast<int>(record.bestFrame.cam25.raw.height),
              "raw_image 宽高 = 设备回报的宽高",
              "包内 " + std::to_string(pkgW) + "x" + std::to_string(pkgH));
    rep.check(pkgW == 4096 && pkgH == 3000,
              "raw_image = 4096x3000（本机型满幅）",
              "实测 " + std::to_string(pkgW) + "x" + std::to_string(pkgH));

    // ---- 8. 核对：16 位容器与**四方长度一致** ----
    std::cout << "\n── 8. 16 位容器与长度一致性 ───────────────────────────\n";
    uint64_t recomputed = 0;
    const bool recomputeOk = aircraft::data::computeExpectedCompactBytes(
        static_cast<uint32_t>(pkgW), static_cast<uint32_t>(pkgH),
        PixelFormat::Mono10, recomputed);

    LengthFacts len;
    len.recomputed      = recomputeOk ? recomputed : 0;
    len.expectedCompact = static_cast<uint64_t>(expectedCompact < 0 ? 0 : expectedCompact);
    len.sdkPayload      = static_cast<uint64_t>(sdkPayload < 0 ? 0 : sdkPayload);
    len.fileBytes       = onDisk.size();
    len.allFourEqual    = (len.recomputed == len.expectedCompact) &&
                       (len.expectedCompact == len.sdkPayload) &&
                       (len.sdkPayload == len.fileBytes);
    {
        std::ostringstream os;
        os << "重算（包内几何 " << pkgW << "x" << pkgH << " / Mono10）= "
           << len.recomputed << "；expected_compact_bytes = "
           << len.expectedCompact << "；sdk_payload_bytes = " << len.sdkPayload
           << "；实际文件 = " << len.fileBytes;
        rep.note(os.str());
    }
    rep.check(recomputeOk, "按包内几何重算期望长度未溢出/未遇不支持格式");
    rep.check(len.recomputed == static_cast<uint64_t>(pkgW) *
                                    static_cast<uint64_t>(pkgH) * 2ull,
              "重算长度 = 宽 × 高 × 2 ⇒ **16 位容器**（不是 1 字节/像素）",
              "实际 " + std::to_string(len.recomputed) + " 字节");
    rep.check(len.allFourEqual,
              "重算长度 = expected_compact_bytes = sdk_payload_bytes = 文件长度",
              "四者不等即为长度契约破裂");

    // ---- 9. 解码与像素位置核对（正对照 + 判别力证据）----
    std::cout << "\n── 9. 离线解码与逐像素核对 ────────────────────────────\n";
    const cv::Mat& display = record.bestFrame.cam25.image;
    rep.check(!display.empty() && display.type() == CV_8UC1,
              "交付的显示图为单通道 8 位（CV_8UC1）",
              "type=" + std::to_string(display.type()));

    // 正确的一种：小端 16 位容器、位移 2。
    const DecodeStat good = decodeAgainst(onDisk, display,
                                          static_cast<uint32_t>(pkgW),
                                          static_cast<uint32_t>(pkgH),
                                          2, /*bigEndian=*/false);
    {
        std::ostringstream os;
        os << "按包内元数据（小端 16 位容器、位移 2）解码：" << pkgW << "x" << pkgH
           << " = " << (static_cast<uint64_t>(pkgW) * pkgH) << " 像素；"
           << "与显示图不一致 " << good.mismatchVsDisplay << " 个；"
           << "值域 [" << good.minValue << ", " << good.maxValue << "]，均值 "
           << good.meanValue << "；值 > 255 的像素 " << good.above8BitCount << " 个";
        rep.note(os.str());
    }
    rep.check(good.mismatchVsDisplay == 0,
              "**每一像素位置**都与交付的显示图一致（位移 2、小端）",
              "不一致 " + std::to_string(good.mismatchVsDisplay) + " 个像素");
    rep.check(good.above8BitCount > 0,
              "载荷里有值 > 255 的像素 ⇒ 内容真的是 10 位（不是按 8 位截断后存的）",
              "值 > 255 的像素 " + std::to_string(good.above8BitCount) + " 个");

    // 正对照：另外三种解码方式**必须**大面积不一致，否则上面的 0 不能算证据。
    const DecodeStat shift0 = decodeAgainst(onDisk, display,
                                            static_cast<uint32_t>(pkgW),
                                            static_cast<uint32_t>(pkgH),
                                            0, /*bigEndian=*/false);
    const DecodeStat shift4 = decodeAgainst(onDisk, display,
                                            static_cast<uint32_t>(pkgW),
                                            static_cast<uint32_t>(pkgH),
                                            4, /*bigEndian=*/false);
    const DecodeStat be     = decodeAgainst(onDisk, display,
                                            static_cast<uint32_t>(pkgW),
                                            static_cast<uint32_t>(pkgH),
                                            2, /*bigEndian=*/true);
    {
        std::ostringstream os;
        os << "正对照（必须大面积不一致，否则上面的 0 不构成证据）："
           << "位移 0 → 不一致 " << shift0.mismatchVsDisplay << "；"
           << "位移 4（照抄 Mono12）→ 不一致 " << shift4.mismatchVsDisplay << "；"
           << "大端 16 位容器 → 不一致 " << be.mismatchVsDisplay;
        rep.note(os.str());
    }
    rep.check(shift0.mismatchVsDisplay > 0 && shift4.mismatchVsDisplay > 0 &&
                  be.mismatchVsDisplay > 0,
              "误移 0 / 误移 4 / 误判字节序三种错法都会被这套核对抓住",
              "三者若有一个为 0，说明该维度在本帧数据上不可区分，"
              "本报告的对应结论不成立");

    // ---- 9bis 字节序：**不依赖显示图**的判据 ----
    //
    // ⚠ 上面那组"与显示图比对"证明不了字节序：显示图是**同一个后端**按
    //   **同一份声明**的字节序做出来的（`buildFrameFromView` 的显示转换），
    //   两者一致只说明"自洽"，不说明"声明符合设备"。
    //   本节补一条**只用原始字节与格式位宽**的判据：
    //     · 该帧的格式是 `Mono10`（设备侧读回，见第 2 节）⇒ 每个样本的
    //       **真值 ≤ 1023**（10 位有效位）；
    //     · 若设备实际按**相反**顺序存放，则我们按声明顺序组装出来的就不是
    //       那个真值，而是它的字节交换 —— 对 10 位样本，交换后绝大多数
    //       取值会**超过 1023**（低字节被当成了高位）。
    //   ∴ "按声明顺序组装 ⇒ 全部 ≤ 1023"且"按相反顺序组装 ⇒ 出现 > 1023"
    //     两条同时成立时，**声明顺序是被本帧数据支持的那一个**。
    //   ⚠ 这不是"已用第三方工具核实"：`IMV_PixelConvert` 对照路径（ENG-09
    //     附录 B 第 2 条记的对照项）**仍未跑**，故结论只能写到"与声明相符
    //     且有独立于显示图的判别证据"。
    const int kValidMax = (1 << 10) - 1;   // Mono10 ⇒ 真值上限 1023
    {
        std::ostringstream os;
        os << "字节序判据（不依赖显示图）：按声明的小端组装 ⇒ 最大值 "
           << good.maxValue << "（10 位上限 " << kValidMax << "）；"
           << "按大端组装 ⇒ 最大值 " << be.maxValue;
        rep.note(os.str());
    }
    rep.check(static_cast<int>(good.maxValue) <= kValidMax,
              "按**声明的小端**组装，每个样本都装得进 10 位（≤ 1023）",
              "最大值 " + std::to_string(good.maxValue));
    rep.check(static_cast<int>(be.maxValue) > kValidMax,
              "按**大端**组装会出现 > 1023 的样本 ⇒ 相反的顺序在本帧数据上可判别",
              "最大值 " + std::to_string(be.maxValue) + "（若 ≤ 1023，"
              "说明本帧数据对该判据不可判别，本节结论不成立）");

    // ---- 10. 无帧占位两路 ----
    std::cout << "\n── 10. cam50 / cam100 的合法无帧占位 ──────────────────\n";
    {
        std::string s1, s2, r1, r2;
        f0 = fs["best_frame"]["frames"][1];
        f0["data_source"] >> s1;
        f0["role"] >> r1;
        cv::FileNode f2 = fs["best_frame"]["frames"][2];
        f2["data_source"] >> s2;
        f2["role"] >> r2;
        rep.note("frames[1]：role=\"" + r1 + "\"，data_source=\"" + s1 +
                 "\"；frames[2]：role=\"" + r2 + "\"，data_source=\"" + s2 + "\"");
        rep.check(r1 == "CAM50" && r2 == "CAM100" && s1 == "none" && s2 == "none",
                  "另两路按槽位写角色、无数据（不因默认帧 role=CAM25 而误写）");
    }
    {
        std::ifstream in1(joinPath(pkg, "cam50.raw"));
        std::ifstream in2(joinPath(pkg, "cam100.raw"));
        rep.check(!in1.good() && !in2.good(),
                  "未生成 cam50.raw / cam100.raw（无数据即不落盘）");
    }

    // ---- 11. 收尾 ----
    const aircraft::data::OperationResult stopped = backend.stop();
    const aircraft::data::OperationResult closed  = backend.close();
    rep.check(stopped.ok(), "backend->stop() 成功",
              std::string("status=") +
                  aircraft::data::opStatusName(stopped.status));
    rep.check(closed.ok(), "backend->close() 成功",
              std::string("status=") +
                  aircraft::data::opStatusName(closed.status));

    // ---- 12. 结论 ----
    std::cout << "\n============================================================\n";
    std::cout << " 通过 " << rep.passed << " 项，不通过 " << rep.failed << " 项\n";
    std::cout << " 结果包：" << pkg << "\n";
    std::cout << " ⚠ 本目录是**专项验证产物**，不是一次测量结果包：\n"
                 "   task_id 以 RAW_BYPASS_ 开头、success=false、无姿态数值、\n"
                 "   calibration/model 均为 RAW_BYPASS_NO_* 占位。\n";
    std::cout << " 本工具**不覆盖**：应用正常测量流程自动触发 RAW 保存 ——\n"
                 "   那条路径要在状态机走到 SAVE 时才发生，而 SAVE 需要\n"
                 "   POSESOLVE 成功，后者受机型库缺失阻塞（未验证）。\n";
    std::cout << "============================================================\n";
    return rep.failed == 0 ? 0 : 1;
}
