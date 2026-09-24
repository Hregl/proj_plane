// ============================================================================
//  tests/integration/MeasurementFlowTest.cpp
//
//  SYS-08 §10「收敛性测试（新增，必测）」的 **7 个用例**逐条落地，
//  外加闭环正常路径与 IF-THR-05 / SYS-08 §8 的联调验证。
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本文件证明的是 SYS-08 §11 约束 9：                                    │
//  │   **单次测量任务在 T_task 内必然终止（成功或 FAILED）**。             │
//  │ 缺了它，"状态机有界"就只是文档上的一句话。                            │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ⚠ 测试手法：把**时钟**也一起注入（tick(nowNs) / startMeasurement(nowNs)），
//  故"60 s 任务在 60 s 终止"在毫秒级真实时间内即可验证，且判定是**精确**的
//  （注入的时刻没有抖动，不像真实时钟需要 ±1 s 的容差）。
//  这也是 RetryManager.h 与 MeasurementController.h 坚持"时刻由调用方传入"
//  的直接收益。
//
//  ⚠ 桩的构造原则：**目标位置只存在于图像里**。
//  VirtualCameras 按虚拟场景把目标画成一块亮矩形，
//  StubPipeline 再用阈值 + 连通域把它找回来 —— 偏差不通过任何共享变量传递，
//  与真实系统的信息流一致。若改成"桩之间直接传真值"，测出来的闭环就
//  少了一层（检测 → 偏差 → 角度）的验证。
// ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algorithm/pipeline/IPosePipeline.h"
#include "application/MeasurementController.h"
#include "data/CameraCalibration.h"
#include "data/CameraRole.h"
#include "data/DetectionResult.h"
#include "data/ErrorInfo.h"
#include "data/ImageFrame.h"
#include "data/ImageQuality.h"
#include "data/MeasurementConfig.h"
#include "data/MeasurementState.h"
#include "data/MeasurementTask.h"
#include "data/MultiCameraFrame.h"
#include "data/OpticalRigCalibration.h"
#include "data/PoseValidationResult.h"
#include "data/ShipPoseResult.h"
#include "data/TargetScaleEstimate.h"
#include "data/TurntableCommand.h"
#include "data/TurntableConfig.h"
#include "data/TurntableMotionState.h"
#include "data/TurntableState.h"
#include "device/camera/IMultiCameraManager.h"
#include "device/turntable/ITurntableController.h"
#include "optical/OpticalRig.h"
#include "preview/PreviewManager.h"

using aircraft::application::MeasurementController;
using aircraft::data::CameraRole;
using aircraft::data::MeasurementConfig;
using aircraft::data::MeasurementState;
using aircraft::data::TurntableConfig;
using S = aircraft::data::MeasurementState;

namespace
{

constexpr double kDegToRad = 0.01745329251994329576923690768489;

// ===========================================================================
//  虚拟场景
// ===========================================================================

/// 场景的真值：目标的方向（相对转台零位）与虚拟相机内参。
struct VirtualScene
{
    double azTarget = 20.0;   // deg
    double elTarget = -5.0;   // deg
    double fx = 2000.0;       // pixel
    double fy = 2000.0;       // pixel
    int    width = 2048;
    int    height = 1536;

    /// 可用相机数（§7.5 的三分支：3 正常 / 2 降级 / ≤1 失败）。
    /// 按 CAM25 → CAM50 → CAM100 的顺序取前 N 个。
    int channels = 3;

    /// 是否把目标画进图像。false 等价于"YOLO 恒返回空"。
    bool drawTarget = true;

    int targetWidth = 80;
    int targetHeight = 60;
};

// ===========================================================================
//  虚拟转台（device::ITurntableController）
// ===========================================================================

class VirtualTurntable : public aircraft::device::ITurntableController
{
public:
    /// 指令到位比例。1.0 = 完全到位；0.5 = 只走一半（几何收敛）；
    /// 2.0 = 过冲到两倍（等幅振荡，永不收敛）。
    double gain = 1.0;

    /// 最小可动角（deg）。|指令 - 当前| 小于它时机械不动、但如实回复"已到位"。
    ///
    /// ⚠ 这个参数是"闭环正确但永远收敛不了"的**物理**来源：
    /// 转台的分辨率/回程间隙使小于该值的调整作废，于是目标始终停在
    /// ±50 pixel 判据之外，ALIGN 的 8 次上限被真正耗尽（§7.3 / §7.7 的 2003）。
    /// 没有它就只能靠"桩返回失败"来造这个场景——那测的是错误处理，
    /// 不是收敛性。
    double minMoveDeg = 0.0;

    /// move() 是否返回失败（模拟 2001 转台通信失败）。
    bool failMove = false;

    /// 是否上报 ERROR 运动状态（§7.5：转台故障 → 直接 FAILED，不重试）。
    bool errorState = false;

    /// 每次 move() 后返回 MOVING 的次数。
    int movingTicks = 1;

    double azimuth = 0.0;
    double elevation = 0.0;
    int    moveCount = 0;

    /// 把转台**静置**在给定角度（同时写入目标角）。
    ///
    /// ⚠ 必须同时写 targetAz_/targetEl_：state() 在"无待执行指令"时会用
    /// target*_ 覆盖 azimuth/elevation（模拟机械到位）。只赋 azimuth 的话，
    /// 控制器第一次读 state() 就会把预置角度冲掉。
    void preset(double az, double el)
    {
        azimuth = targetAz_ = az;
        elevation = targetEl_ = el;
        pending_ = 0;
    }

    bool initialize() override { return true; }

    bool move(const aircraft::data::TurntableCommand& c) override
    {
        ++moveCount;
        if (failMove)
        {
            return false;
        }

        targetAz_ = azimuth + gain * (c.azimuthCommand - azimuth);
        targetEl_ = elevation + gain * (c.elevationCommand - elevation);

        if (std::fabs(targetAz_ - azimuth) < minMoveDeg)
        {
            targetAz_ = azimuth;   // 未达最小可动角：机械不动
        }
        if (std::fabs(targetEl_ - elevation) < minMoveDeg)
        {
            targetEl_ = elevation;
        }

        pending_ = movingTicks;
        return true;
    }

    aircraft::data::TurntableState state() override
    {
        aircraft::data::TurntableState s;

        if (errorState)
        {
            s.azimuth = azimuth;
            s.elevation = elevation;
            s.motion = aircraft::data::TurntableMotionState::ERROR;
            return s;
        }

        if (pending_ > 0)
        {
            --pending_;
            s.azimuth = azimuth;
            s.elevation = elevation;
            s.motion = aircraft::data::TurntableMotionState::MOVING;
            return s;
        }

        azimuth = targetAz_;
        elevation = targetEl_;
        s.azimuth = azimuth;
        s.elevation = elevation;
        s.motion = aircraft::data::TurntableMotionState::STABLE;
        return s;
    }

    void stop() override
    {
        pending_ = 0;
        targetAz_ = azimuth;
        targetEl_ = elevation;
    }

private:
    double targetAz_ = 0.0;
    double targetEl_ = 0.0;
    int    pending_ = 0;
};

// ===========================================================================
//  虚拟相机（device::IMultiCameraManager）
// ===========================================================================

class VirtualCameras : public aircraft::device::IMultiCameraManager
{
public:
    VirtualCameras(const VirtualScene& scene, const VirtualTurntable& turntable)
        : scene_(scene)
        , turntable_(turntable)
    {
    }

    /// 令接下来 N 次采集返回失败（模拟瞬时丢帧）。
    int failNextCaptures = 0;

    bool initializeAll() override { return true; }
    bool startAll() override { return true; }
    void stopAll() override {}

    bool capture(aircraft::data::MultiCameraFrame& frame) override
    {
        ++captureCount;

        if (failNextCaptures > 0)
        {
            --failNextCaptures;
            return false;
        }

        const CameraRole roles[3] = {CameraRole::CAM25, CameraRole::CAM50,
                                     CameraRole::CAM100};
        aircraft::data::ImageFrame* slots[3] = {&frame.cam25, &frame.cam50,
                                               &frame.cam100};

        // 每次采集一个**自增的采集序号**，三路共用（硬触发下三路是同一次曝光）。
        //
        // ⚠ 原实现把三路的 frameId 恒置为 1。那样"被解算的是**哪一次**采集"
        // 在测试里不可观测，C-02 的判据（最佳帧 vs 最后一次采集、空帧是否
        // 造成下标错位）都无从断言 —— 可测性必须先解决，否则只能写出一条
        // 恒真的测试（C02 §5）。
        const uint64_t acqId = ++acquisitionId_;

        for (int i = 0; i < 3; ++i)
        {
            if (i >= scene_.channels)
            {
                *slots[i] = aircraft::data::ImageFrame{};   // 空图 = 该路不可用
                continue;
            }
            *slots[i] = render(roles[i]);
            slots[i]->frameId = acqId;
        }

        if (capturePhase_)
        {
            ++phaseCaptureCount_;
            phaseAcquisitionIds_.push_back(acqId);

            // "这一次该路没有像素"：三路的 frameId 都已自增（它确实是第 N 次
            // 采集），只有指定焦段没有图像数据 —— 模拟 grab 成功但无像素。
            // 与 simulateFault 的区别：**不**禁用该通道，故下一次采集照常。
            if (static_cast<int>(phaseCaptureCount_) == blankOnPhaseCapture_)
            {
                switch (blankRole_)
                {
                case CameraRole::CAM50:  frame.cam50.image  = cv::Mat{}; break;
                case CameraRole::CAM100: frame.cam100.image = cv::Mat{}; break;
                case CameraRole::CAM25:  frame.cam25.image  = cv::Mat{}; break;
                }
            }
        }

        frame.triggerTimestamp = 1000000;

        // 曝光序号（裁决 D-C02-6 的第 5 个字段）。
        //
        // ⚠ 本桩**替身**的是 `device::IMultiCameraManager`，故它必须满足
        //    该字段对实现者的要求（同一组三帧共享一个值、随每次曝光单调递增），
        //    否则这里测出的记录会是一个"字段恒为 0"的记录，而 0 的含义是
        //    **未定** —— 用例就会变成恒真。
        //    真实生产者的推导（基准锁定 / 帧号回退 / 降级换参考路）由
        //    DeviceLayerTest 直接用 `MultiCameraManager` 验证；本文件只验证
        //    "这个值能一路进到记录里、且与解算的那一帧同源"。
        frame.exposureIndex = acqId;

        return true;
    }

    /// 打开 CAPTURE 阶段的采集计数（由 Harness 在进入 CAPTURE 时调用）。
    /// 每个 tick 只推进一个状态，故"此刻状态 == CAPTURE"意味着本 tick 内
    /// 会连续做完 CAPTURE 的全部采集。
    void armCapturePhase()
    {
        if (capturePhase_)
        {
            return;
        }
        capturePhase_ = true;
        phaseCaptureCount_ = 0;
        phaseAcquisitionIds_.clear();
    }

    /// 令 CAPTURE 阶段的**第 N 次**（1 基）采集的指定焦段无像素；-1 = 不注入。
    int        blankOnPhaseCapture_ = -1;
    CameraRole blankRole_ = CameraRole::CAM25;

    bool capturePhase_ = false;
    int  phaseCaptureCount_ = 0;

    /// CAPTURE 阶段各次采集的采集序号（依序）。
    std::vector<uint64_t> phaseAcquisitionIds_;

    int captureCount = 0;

private:
    aircraft::data::ImageFrame render(CameraRole role) const
    {
        aircraft::data::ImageFrame f;
        f.role = role;
        f.cameraId = "virtual";
        f.frameId = 1;
        f.timestampNs = 1000000;
        f.image = cv::Mat::zeros(scene_.height, scene_.width, CV_8UC1);

        if (!scene_.drawTarget)
        {
            return f;
        }

        // 目标相对光轴的角位移。俯仰一项取 (相机角 - 目标角)，因为
        // ENG-09 §2.4 冻结图像 Y **向下**为正：目标在光轴**上方**时
        // 应出现在画面顶部，即 pixelY < 0。
        const double dAz = scene_.azTarget - turntable_.azimuth;
        const double dEl = turntable_.elevation - scene_.elTarget;

        const double centreX = static_cast<double>(scene_.width) / 2.0;
        const double centreY = static_cast<double>(scene_.height) / 2.0;
        const double px = scene_.fx * std::tan(dAz * kDegToRad);
        const double py = scene_.fy * std::tan(dEl * kDegToRad);

        const int x = static_cast<int>(std::lround(
            centreX + px - scene_.targetWidth / 2.0));
        const int y = static_cast<int>(std::lround(
            centreY + py - scene_.targetHeight / 2.0));

        cv::rectangle(f.image, cv::Rect(x, y, scene_.targetWidth, scene_.targetHeight),
                      cv::Scalar(255), cv::FILLED);
        return f;
    }

    const VirtualScene&     scene_;
    const VirtualTurntable& turntable_;

    /// 采集序号发生器（本类**唯一**的采集身份来源）。
    uint64_t acquisitionId_ = 0;
};

// ===========================================================================
//  算法桩（algorithm::IPosePipeline）
// ===========================================================================

class StubPipeline : public aircraft::algorithm::IPosePipeline
{
public:
    /// 恒返回"未检出"（SYS-08 §10 用例 1 的"YOLO 恒返回空"）。
    bool detectReturnsEmpty = false;

    /// PnP 恒失败（用例 4）。
    bool solveFails = false;

    /// 验证恒不通过（用例 3）。
    bool validateFails = false;

    /// 验证**契约违背**（R05）：返回值与 out.valid 相反。
    /// 这是"坏掉的实现"的替身，用于逼出控制器的对账分支 —— 正确的实现下
    /// 该分支永不可达。详见 validate() 内的说明。
    bool validateLies = false;

    /// 通道评分恒返回候选集的第一个。
    /// 因为调用方传入的 allowed 已排除失败过的通道，故"次优"自然体现为
    /// 候选集顺序的下一个 —— 这正是 §7.3 升级规则第 2 条要的行为。
    bool selectFails = false;

    /// 每次 solvePose 收到的通道，按调用次序记录（用例 4 的判据）。
    std::vector<CameraRole> solveCameras;

    // ---- C-02：最佳帧数据链的观测点 ---------------------------------------

    /// selectBestFrame 应当返回的**视图**下标；-1 表示"视图最后一个"。
    ///
    /// ⚠ 默认 0 使"最佳帧 = 第 1 次采集"而"最后一次采集 = 第 5 次"，
    /// 二者天然不同 —— 这正是判据 1 需要的场景，且不需要任何
    /// "按帧号设定分数"的测试专用接口（C02 §5 方案 1）。
    int bestViewIndex = 0;

    /// 每次 selectBestFrame 收到的**视图帧数**，按调用次序。
    /// 判据 4 具备鉴别力的前提：若注入的空帧未生效，视图仍是 5 帧，
    /// viewToAcq 退化为恒等映射，"正确映射"与"错位下标"会给出同一答案。
    std::vector<std::size_t> viewSizes;

    /// 每次 solvePose 收到的帧的三路 frameId，按调用次序。
    /// 判据 1 / 2 / 4 都断言它 —— 它是"PnP 究竟解算了哪一次采集"的
    /// **唯一**可观测出口（控制器不对外暴露 bestFrame_）。
    std::vector<std::array<uint64_t, 3>> solvedFrameIds;

    int detectCalls = 0;
    int solveCalls = 0;

    /// 007 给 IF-SW-02 增加的纯虚方法（ENG-09 §8 允许的接口扩展，
    /// 见 IPosePipeline.h 的说明）：注入粗姿态。
    /// 桩只需记录调用，因为本文件的状态机判据不涉及 B 类定位。
    int    coarseAttitudeCalls = 0;
    double lastCoarseDistanceM = 0.0;

    void setCoarseAttitude(double azimuthDeg,
                           double elevationDeg,
                           double distanceM) override
    {
        (void)azimuthDeg;
        (void)elevationDeg;
        ++coarseAttitudeCalls;
        lastCoarseDistanceM = distanceM;
    }

    // ---- C-02 Step 5：统计量推送通道 --------------------------------------

    /// 控制器在构造里登记的观察者（判据 7 断言它在析构时被注销）。
    aircraft::algorithm::IPipelineObserver* observer = nullptr;

    /// 推送给观察者的次数。判据 8 断它 > 0 —— 若控制器忘了实现
    /// `onStatistics`（或接口连接断了），记录里的统计量会全为 0，
    /// 而**全为 0 是合法值**，只看记录是发现不了的。
    int statisticsPushes = 0;

    /// 是否真的推送。置 false 用于构造"算法没推送"的场景，
    /// 验证控制器能把它与"推送了全 0"区分开。
    bool publishStatistics = true;

    /// 推送的内容。默认各项**非零且互不相等** ——
    /// 全零（或各项相同）的载荷无法区分"字段串了"与"传对了"。
    aircraft::data::MeasurementStatistics payload = makePayload();

    static aircraft::data::MeasurementStatistics makePayload()
    {
        aircraft::data::MeasurementStatistics s;
        s.featureCount      = 140;
        s.matchCount        = 77;
        s.droppedByConflict = 9;
        s.cadCount          = 21;
        s.textureCount      = 56;
        s.spreadPx          = 812.5;
        return s;
    }

    void setStatisticsObserver(
        aircraft::algorithm::IPipelineObserver* o) override
    {
        observer = o;
    }

    /// 把统计量推给观察者（转发给控制器的 onStatistics）。
    void publish()
    {
        if (observer == nullptr || !publishStatistics)
        {
            return;
        }
        ++statisticsPushes;
        observer->onStatistics(payload);
    }

    bool detect(const aircraft::data::ImageFrame& frame,
                aircraft::data::DetectionResult& out) override
    {
        ++detectCalls;
        out = aircraft::data::DetectionResult{};
        out.sourceCamera = frame.role;

        // SYS-04 §4.2 / IF-SW-02：**未检出返回 false**，而不是 true + found=false。
        // 后者无法与"根本没跑检测"区分，而 SEARCH 的重试判据正依赖这个区分。
        if (detectReturnsEmpty || frame.image.empty())
        {
            out.found = false;
            return false;
        }

        // 从图像里把亮块找回来 —— 信息只经由图像传递。
        cv::Mat mask;
        cv::threshold(frame.image, mask, 128.0, 255.0, cv::THRESH_BINARY);

        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);
        if (points.empty())
        {
            out.found = false;
            return false;
        }

        out.found = true;
        out.bbox = cv::boundingRect(points);
        out.confidence = 0.95;
        return true;
    }

    bool estimateScale(const aircraft::data::ImageFrame&,
                       const aircraft::data::DetectionResult& detection,
                       aircraft::data::TargetScaleEstimate& out) override
    {
        // 只要求"能给出一个与像素尺寸有关的距离估计"，数值本身不影响本文件
        // 的任何判据（距离与焦段的对应关系是 007 的算法职责）。
        out.targetPixelSize =
            static_cast<double>(std::max(detection.bbox.width, detection.bbox.height));
        out.distance = 150.0;
        out.confidence = 0.9;
        return out.targetPixelSize > 0.0;
    }

    bool selectCamera(const aircraft::data::MultiCameraFrame&,
                      const std::vector<CameraRole>& allowed,
                      aircraft::data::MeasurementSelectionResult& out) override
    {
        if (selectFails || allowed.empty())
        {
            return false;
        }
        out.selectedCamera = allowed.front();
        out.score = 1.0;
        return true;
    }

    bool selectBestFrame(const std::vector<aircraft::data::ImageFrame>& frames,
                         int& bestIndex,
                         aircraft::data::ImageQuality& quality) override
    {
        if (frames.empty())
        {
            return false;
        }
        viewSizes.push_back(frames.size());
        bestIndex = (bestViewIndex < 0) ? static_cast<int>(frames.size()) - 1
                                        : bestViewIndex;
        quality.sharpness = 1.0;
        quality.exposure = 0.8;
        quality.contrast = 0.9;
        quality.featureCount = 120;
        quality.matchCount = 100;
        quality.matchRatio = 0.833;
        return true;
    }

    bool solvePose(const aircraft::data::MultiCameraFrame& frame,
                   CameraRole camera,
                   const aircraft::data::CameraCalibration& calib,
                   aircraft::data::ShipPoseResult& out) override
    {
        ++solveCalls;
        solveCameras.push_back(camera);
        solvedFrameIds.push_back(
            {frame.cam25.frameId, frame.cam50.frameId, frame.cam100.frameId});

        // ⚠ 推送点与真实 `PosePipeline` 一致：**匹配完成之后**、
        // 任何一条失败早退之前（真实链在"匹配成功但 PnP 未收敛"时同样推送，
        // 因为那正是最需要统计量来解释的一次）。
        // 桩没有独立的匹配阶段，故此处即"匹配已完成"。
        publish();

        // 标定必须可用：与本文件外的 AlignmentController 用例同一条要求。
        if (calib.cameraMatrix.empty() || solveFails)
        {
            out = aircraft::data::ShipPoseResult{};
            return false;
        }

        out = aircraft::data::ShipPoseResult{};
        out.success = true;
        out.yaw = 12.34;
        out.pitch = 1.10;
        out.roll = -0.45;
        out.reprojectionError = 0.42;
        return true;
    }

    bool validate(const aircraft::data::ShipPoseResult& pose,
                  aircraft::data::PoseValidationResult& out) override
    {
        out = aircraft::data::PoseValidationResult{};
        out.valid = !validateFails && pose.success;
        out.reprojectionError = pose.reprojectionError;
        out.inlierRatio = 0.91;
        out.confidence = 0.88;

        // R05（2026-09-24 审查报告）：返回真实的 bool ≡ out.valid，
        // 与真实验证器一致（PoseValidator.cpp 的每个返回点都满足该恒等）。
        //
        // ⚠ 原实现恒 `return true`，于是"不合格"只体现在 out.valid 上 ——
        //   桩与真实实现的这处语义差异，恰好把控制器那条断点藏了起来：
        //   真实路径取 `!validate(...)` 分支、桩永远不走，所以
        //   "不排除相机、不记录验证详情"的缺陷从未在任何测试里出现过。
        //   桩变得**忠实**，而不是被控制器迁就。
        //
        // validateLies：**故意自相矛盾**的注入，让返回值与 out.valid 相反。
        //
        // 为什么要造一个"坏掉的桩"：控制器的对账分支（契约违背 ⇒ 可见失败）
        // 在正确的实现下**永不可达**。本工程对同类断言的一贯做法是造一个
        // 只坏一处的替身把它逼出来（见 C-006 兜底码、C-02 评分器越界的用例）。
        // 而"真实实现坏掉"这件事本身在这里**无法用真实验证器表达** ——
        // 所以这个自相矛盾的桩是"本接口下真正无法执行的错误"唯一可测的替身。
        // 它同时是对契约文本的**负向固化**：若有人日后把契约改成
        // "bool = 执行是否成功"，本用例会立刻转红。
        if (validateLies)
        {
            return !out.valid;
        }
        return out.valid;
    }
};

// ===========================================================================
//  记录器桩（IF-THR-05）
// ===========================================================================

class StubRecorder : public aircraft::application::IRecorderSink
{
public:
    bool  succeed = true;
    int   saveCalls = 0;

    /// 最近一次落盘的记录（C-02 Step 5：入参由 MeasurementTask 改为
    /// MeasurementRecord）。测试据此断言"落盘的内容与解算的那一帧同源"，
    /// 而不只是断言 save() 被调用过 —— 后者在记录填错的实现下同样通过。
    aircraft::data::MeasurementRecord lastRecord;

    bool save(const aircraft::data::MeasurementRecord& record) override
    {
        ++saveCalls;
        lastRecord = record;
        return succeed;
    }
};

// ===========================================================================
//  测试夹具
// ===========================================================================

/// 每个 tick 之间的注入时间步长。默认 1 s —— 与 §7.3 的 2.5 s / 2 s 等
/// 状态超时同量级，使"每个尝试的耗时不超过 T_state"成为可检验的判据。
constexpr uint64_t kStepNs = 1000000000ULL;

class Harness
{
public:
    Harness()
        : cameras_(scene_, turntable_)
    {
    }

    void build(uint64_t stepNs = kStepNs)
    {
        stepNs_ = stepNs;

        rig_.initialize();   // 三通道 + 默认焦距（0.025/0.05/0.1 m）
        aircraft::data::OpticalRigCalibration cal;
        cal.cam25 = stubCalibration();
        cal.cam50 = stubCalibration();
        cal.cam100 = stubCalibration();
        EXPECT_TRUE(rig_.setCalibration(cal));

        controller_ = std::make_unique<MeasurementController>(
            cameras_, turntable_, pipeline_, rig_, config_, turntableConfig_,
            preview_, nullptr, useRecorder_ ? &recorder_ : nullptr,
            modelsAvailable_);
    }

    /// 启动并逐 tick 推进，直到终止态或达到 maxTicks。
    /// tick 注入的时刻依次为 startNs, startNs+step, startNs+2·step, …
    /// 返回值 = 实际执行的 tick 次数（-1 表示 maxTicks 用尽仍未终止）。
    int run(uint64_t startNs, int maxTicks)
    {
        controller_->startMeasurement(startNs);

        uint64_t now = startNs;
        observe(now);

        for (int i = 0; i < maxTicks; ++i)
        {
            if (finished())
            {
                return i;
            }

            // 每次 tick 只推进一个状态，故"此刻状态 == CAPTURE"意味着本 tick
            // 内会连续做完 CAPTURE 的全部采集。在此刻打开采集计数，
            // 就能精确指认"CAPTURE 的第几次采集"（C-02 判据 1 / 4 的前提）。
            if (controller_->state() == S::CAPTURE)
            {
                cameras_.armCapturePhase();
            }

            controller_->tick(now);
            observe(now);
            now += stepNs_;
        }
        return -1;
    }

    bool finished() const
    {
        const S s = controller_->state();
        return s == S::COMPLETE || s == S::FAILED;
    }

    /// 本次任务已经过的注入时间。
    uint64_t elapsedNs(uint64_t startNs) const { return now_ - startNs; }

    /// 状态序列中出现过的**回退边**次数。
    int backwardTransitions(S from, S to) const
    {
        int n = 0;
        for (std::size_t i = 1; i < states_.size(); ++i)
        {
            if (states_[i - 1] == from && states_[i] == to)
            {
                ++n;
            }
        }
        return n;
    }

    /// 状态序列中全部向后转换的次数（与 RetryManager::rollbackCount() 对账）。
    int totalBackwardTransitions() const
    {
        int n = 0;
        for (std::size_t i = 1; i < states_.size(); ++i)
        {
            if (StateMachineRank(states_[i]) < StateMachineRank(states_[i - 1])
                && states_[i] != S::FAILED && states_[i] != S::IDLE
                && states_[i - 1] != S::COMPLETE && states_[i - 1] != S::FAILED)
            {
                ++n;
            }
        }
        return n;
    }

    bool sawState(S state) const
    {
        for (S s : states_)
        {
            if (s == state)
            {
                return true;
            }
        }
        return false;
    }

    std::vector<S> states() const { return states_; }

    // ---- 各桩（公有，供用例按需改写）----
    VirtualScene      scene_;
    VirtualTurntable  turntable_;
    VirtualCameras    cameras_;
    StubPipeline      pipeline_;
    aircraft::optical::OpticalRig rig_;
    MeasurementConfig config_;
    TurntableConfig   turntableConfig_;
    StubRecorder      recorder_;
    aircraft::preview::PreviewManager* preview_ = nullptr;

    /// false 时以 recorder == nullptr 构造控制器（IF-THR-05 未挂载）。
    bool useRecorder_ = true;

    /// 装配点注入的"机型库是否可用"（裁决 C-006 §1.3）。
    /// 算法接口是纯 `bool`，`PosePipeline` 在机型库未加载时静默
    /// `return false`，控制器无从区分它与"真的 PnP 失败"—— 故由
    /// 唯一知道加载结果的装配点注入。false 时 `estimateScale` /
    /// `solvePose` 的失败应归 5001，而不是 9004。
    bool modelsAvailable_ = true;

    std::unique_ptr<MeasurementController> controller_;

private:
    static int StateMachineRank(S s)
    {
        switch (s)
        {
        case S::IDLE:           return 0;
        case S::SEARCH:         return 1;
        case S::TARGET_FOUND:   return 2;
        case S::ALIGN:          return 3;
        case S::STABILIZE:      return 4;
        case S::MEASURE_SELECT: return 5;
        case S::CAPTURE:        return 6;
        case S::POSE_SOLVE:     return 7;
        case S::VALIDATE:       return 8;
        case S::SAVE:           return 9;
        case S::COMPLETE:       return 10;
        case S::FAILED:         return -1;
        }
        return -1;
    }

    static aircraft::data::CameraCalibration stubCalibration()
    {
        aircraft::data::CameraCalibration c;
        c.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        c.cameraMatrix.at<double>(0, 0) = 2000.0;
        c.cameraMatrix.at<double>(1, 1) = 2000.0;
        c.imageWidth = 2048;
        c.imageHeight = 1536;
        return c;
    }

    /// 记录状态序列与"最后一次注入的时刻"。
    /// now_ 必须等于真正传给 tick() 的时刻：elapsedNs() 的判据
    /// （T_task = 60 s ±1 s、6 s 整点等）全部依赖这个等式。
    void observe(uint64_t now)
    {
        const S s = controller_->state();
        if (states_.empty() || states_.back() != s)
        {
            states_.push_back(s);
        }
        now_ = now;
    }

    uint64_t        stepNs_ = kStepNs;
    uint64_t        now_ = 0;
    std::vector<S>  states_;
};

/// 已配置的转台行程（±180° / −60°~+60°），使越程判定真正生效。
TurntableConfig configuredTurntable()
{
    TurntableConfig c;
    c.azimuthMin = -180.0;
    c.azimuthMax = 180.0;
    c.elevationMin = -60.0;
    c.elevationMax = 60.0;
    c.centerThreshold = 50.0;
    return c;
}

/// 把转台预设到"偏差恰好 (pixelX, pixelY)"的位置（用例 2 / 7 的构造方式）。
///
/// 由虚拟场景的投影式反解：pixelX = fx·tan(azTarget − az)  ⇒
/// az = azTarget − arctan(pixelX/fx)；俯仰一侧取 +（ENG-09 §2.4，Y 向下为正）。
void preloadOffset(Harness& h, double pixelX, double pixelY)
{
    h.turntable_.preset(
        h.scene_.azTarget - std::atan(pixelX / h.scene_.fx) / kDegToRad,
        h.scene_.elTarget + std::atan(pixelY / h.scene_.fy) / kDegToRad);
}

}  // namespace

// ===========================================================================
//  0 闭环正常路径（不属 §10 的 7 个用例，但它是 7 个用例的对照基线）
// ===========================================================================

TEST(MeasurementFlowTest, 正常路径应走完12状态并成功终止)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;   // 每次只走一半 → 几何收敛，需 4 次对准
    h.recorder_.succeed = true;
    h.build();

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    EXPECT_EQ(h.controller_->state(), S::COMPLETE);

    // §6 的正常流程必须**全部经过**，不能跳状态。
    //
    // ⚠ 这里列出的是 10 个状态：IDLE 不在序列内 —— 它是构造时的初始态，
    // startMeasurement() 的第一个动作就是 IDLE → SEARCH，故状态**序列**
    // 里永远看不到 IDLE。若把 IDLE 也写进期望表，由于
    // `MeasurementState::IDLE == 0`，数组尾部的零初始化会静默地追加一个
    // IDLE 需求，测试就在断言一件不可能发生的事（本文件曾如此）。
    const S expected[] = {S::SEARCH, S::TARGET_FOUND, S::ALIGN, S::STABILIZE,
                          S::MEASURE_SELECT, S::CAPTURE, S::POSE_SOLVE,
                          S::VALIDATE, S::SAVE, S::COMPLETE};
    for (S s : expected)
    {
        EXPECT_TRUE(h.sawState(s)) << "未经过状态 " << static_cast<int>(s);
    }
    EXPECT_FALSE(h.sawState(S::IDLE)) << "状态序列中不应出现 IDLE（它是构造初值）";

    // §5.4 的停止条件：对准应在 8 次上限内达成（此处 4 次）。
    EXPECT_LE(h.controller_->attempts(S::ALIGN), 8);
    EXPECT_EQ(h.controller_->lastError().code, 0);
    EXPECT_FALSE(h.controller_->degraded());
    EXPECT_EQ(h.controller_->rollbackCount(), 0);

    // IF-THR-05：结果包必须被交给记录器，且内容是本次任务。
    EXPECT_EQ(h.recorder_.saveCalls, 1);
    EXPECT_EQ(h.recorder_.lastRecord.task.state, S::COMPLETE);
    EXPECT_TRUE(h.recorder_.lastRecord.task.result.success);
    EXPECT_NEAR(h.recorder_.lastRecord.task.result.yaw, 12.34, 1e-9);
}

TEST(MeasurementFlowTest, 正常路径的每个尝试都不超过其状态超时)
{
    // §7.3 的"该状态最大耗时"列只有在"单次尝试 ≤ T_state"成立时才有意义。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // 对准动作：每次 = 1 tick 发出 + 1 tick 等待到位 = 2 s ≤ alignTimeoutNs(2.5 s)。
    const double perAttemptS =
        2.0 * static_cast<double>(kStepNs) / 1e9;
    EXPECT_LE(perAttemptS, 2.5);

    // 整条链路远在 T_task(60 s) 之内。
    EXPECT_LE(h.elapsedNs(0), h.config_.taskTimeoutNs);
}

TEST(MeasurementFlowTest, 未挂记录器时SAVE记notice而不伪装成落盘成功)
{
    // IF-THR-05 未挂载时（例如第一阶段尚未接入 Recorder），SAVE 不能变成
    // 永远失败 —— 那会把"没接记录器"报成 3 次落盘失败；也不能静默装作
    // 落盘成功 —— 那会让一条**没有留下任何证据**的测量被当作已完成。
    // 唯一诚实的处置：照常走完状态机，同时留下一条可见的 notice。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.useRecorder_ = false;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::COMPLETE);

    bool noticed = false;
    for (const std::string& n : h.controller_->notices())
    {
        if (n.find("未挂载") != std::string::npos)
        {
            noticed = true;
        }
    }
    EXPECT_TRUE(noticed) << "未挂记录器时必须留下可见的 notice，而不是静默通过";
}

TEST(MeasurementFlowTest, 落盘失败应在3次后终止且不进入COMPLETE)
{
    // §7.3：SAVE 上限 3，落盘写不进去没有更早的状态能解决。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.recorder_.succeed = false;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);

    // ⚠ `saveCalls` 是 4 而不是 3（裁决 C-007）：前 3 次是 SAVE 状态里的
    //   落盘尝试，第 4 次是**失败包**（failWith → saveFailurePackage）。
    //   两者语义不同，故不能合并计数 —— 真正回答"SAVE 有没有第 4 次尝试"
    //   的是下一行的 `attempts(S::SAVE)`。只留 saveCalls 会让"失败包写了一次"
    //   看起来像"SAVE 多试了一次"。
    EXPECT_EQ(h.recorder_.saveCalls, 4)
        << "3 次落盘尝试 + 1 次失败包（C-007）";
    EXPECT_EQ(h.controller_->attempts(S::SAVE), 3)
        << "失败包不得占用 SAVE 的尝试额度";
    EXPECT_FALSE(h.controller_->lastError().message.empty());

    // ⚠ 本用例里 recorder 恒失败，故第 4 次（失败包）也失败 —— 而它
    //   **不得**引发"失败→落盘失败→再失败"的循环。判据：状态仍停在
    //   FAILED、SAVE 仍只尝试过 3 次，且控制器没有新增 notice 以外的动作。
    //   递归一旦发生，这里的 attempts(S::SAVE) 会继续增长（新的失败会再
    //   触发一次恢复流程），断言即转红。
    EXPECT_EQ(h.controller_->state(), S::FAILED);
}

// ===========================================================================
//  SYS-08 §10 用例 1：任务时限
// ===========================================================================

TEST(MeasurementFlowTest, 用例1_任务时限_桩恒不检出在Ttask内以9001终止)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.pipeline_.detectReturnsEmpty = true;   // YOLO 恒返回空
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrTaskTimeout);

    // 判据：T_task = 60 s ± 1 s 内进入 FAILED。
    // 注入时钟下这个界是精确的：第一次 tick 时刻 ≥ 60 s 时立即终止。
    EXPECT_GE(h.elapsedNs(0), h.config_.taskTimeoutNs - 1);
    EXPECT_LE(h.elapsedNs(0), h.config_.taskTimeoutNs + 1);

    // SEARCH 不设次数上限，计数只反映"搜了多少次"。
    EXPECT_GT(h.controller_->attempts(S::SEARCH), 1);
    EXPECT_EQ(h.controller_->rollbackCount(), 0);
}

// ===========================================================================
//  SYS-08 §10 用例 2：对准上限
// ===========================================================================

TEST(MeasurementFlowTest, 用例2_对准上限_恒偏差100pixel时恰好8次后2003)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    // 转台最小可动角 5°：偏差 100 pixel 对应 2.86°，小于该值 → 机械不动。
    // 于是偏差恒定、永不进入 ±50 pixel 判据。
    h.turntable_.minMoveDeg = 5.0;
    preloadOffset(h, 100.0, 100.0);
    h.build();

    ASSERT_GE(h.run(0, 200), 0);

    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrAlignRetryExhausted);

    // 判据一：ALIGN **恰好**尝试 8 次。
    EXPECT_EQ(h.controller_->attempts(S::ALIGN), 8);
    EXPECT_EQ(h.turntable_.moveCount, 8) << "8 次尝试必须对应 8 条转台指令";

    // 判据二：总耗时 ≤ 20 s（§7.3 的 8 × 2.5 s 预算）。
    EXPECT_LE(h.elapsedNs(0), 20000000000ULL);

    // ALIGN 没有回退边：对准失败时退到更早的状态没有意义（§5.4）。
    EXPECT_EQ(h.controller_->rollbackCount(), 0);
}

// ===========================================================================
//  SYS-08 §10 用例 3：回退预算
// ===========================================================================

TEST(MeasurementFlowTest, 用例3_回退预算_VALIDATE恒不通过时的有界性)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.pipeline_.validateFails = true;
    h.build();

    ASSERT_GE(h.run(0, 400), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);

    // 判据一：总回退次数不超过 §7.4 的 4 次。
    EXPECT_LE(h.controller_->rollbackCount(), 4);

    // 判据二：同一回退边不超过 2 次。从状态序列独立数一遍，
    // 与 RetryManager 的计数对账 —— 两处不一致就说明"批准"与"记账"脱节。
    EXPECT_LE(h.backwardTransitions(S::VALIDATE, S::MEASURE_SELECT), 2);
    EXPECT_EQ(h.totalBackwardTransitions(), h.controller_->rollbackCount());

    // 判据三：以 FAILED 终止且错误可定位（消息必须点出状态名）。
    EXPECT_FALSE(h.controller_->lastError().message.empty());

    // 判据四（R05，2026-09-24 审查报告）：**验证详情必须留存**。
    //
    // 这正是原实现提前 return 丢掉的东西：`!validate(...)` 为真时直接
    // handleFailure 返回，于是 `validationResult_ = validation` 永不执行。
    // 上面三条判据在这个缺陷下**全都照旧成立**（回退次数只会更少，
    // 消息非空、码仍是 9004），所以没有本条时，缺陷只能被
    // `totalBackwardTransitions() == rollbackCount()` 间接撞到 ——
    // 那条信号说的是"计数对不上"，说不清是"验证详情丢了"。
    //
    // 本条的期望值来自**测试自己的桩**（0.42 / 0.91 / 0.88），
    // 不是读回实现成员，故无法伪装。
    EXPECT_FALSE(h.controller_->validationResult().valid);
    EXPECT_NEAR(h.controller_->validationResult().reprojectionError, 0.42, 1e-9);
    EXPECT_NEAR(h.controller_->validationResult().inlierRatio, 0.91, 1e-9);
    EXPECT_NEAR(h.controller_->validationResult().confidence, 0.88, 1e-9);

    // ⚠ 与 SYS-08 §10 的**一处已知偏离**，如实记录而不迁就。
    //
    // 文档期望本用例以 9002（回退预算耗尽）结束，实测不是。逐 tick 追踪
    // 得到的真实路径如下（tick 间隔 1 s，本文件顶部注释给出的注入时钟）：
    //
    //   t=16  VALIDATE#1 不通过 → 排除 CAM25 → 回退#1
    //   t=19  POSE_SOLVE#2 命中 §7.3 升级规则第 2 条 → switchToNextCamera()
    //                        把选中通道换成 CAM100（**CAM50 就此被排除**，
    //                        尽管它从未验证失败过）
    //   t=20  VALIDATE#2 不通过 → 排除 CAM100（本次实际使用的通道）→ 回退#2
    //   t=22  MEASURE_SELECT#3 发现候选集合**已空** → 状态内重试
    //   t=23  MEASURE_SELECT 的动作配额（§7.3 上限 3）耗尽 → FAILED，码 0
    //
    // 两个后果都与文档的算术不同，且都不是实现取舍：
    //   ① 三次排除只用了两次 VALIDATE 失败 —— 因为 switchToNextCamera()
    //      也会排除通道。§7.4 注释里"三台相机、每次排除一台、第 3 次无候选"
    //      这条推导默认了"只有 VALIDATE 会排除"，与 §7.3 的换机升级叠加后
    //      就不再成立。
    //   ② 于是先到者不是 §7.4 的同边预算（2 次），而是 MEASURE_SELECT
    //      自己的 §7.3 上限（3 次，已被"前进 1 次 + 回退 2 次"占满），
    //      故 9002 在本用例中**不可达**。
    //
    // 9002 的通路本身由用例 4（POSE_SOLVE 恒失败）覆盖并**确实到达**，
    // 所以"回退预算机制有效"这一点没有失去验证，失去的只是本用例的码值。
    // 该偏离已登记在 README §6。
    //
    // 断言写成"要么 9002、要么 9004"，是为了让这条偏离**可见而不脆弱**：
    // 将来若按裁决把两个上限的先后关系改正，本用例会在 9002 一侧继续通过。
    //
    // ⚠ C-006 把兜底侧从裸 0 收紧为 9004（kErrStateFailure）。原断言写的
    // `|| code == 0` 在 C-006 之后成了死码 —— 9004 不是 0，且 0 已不再被
    // 用作失败码。保留 `|| 0` 会让断言看起来更强，实际却是在等一个
    // 永远不会出现的值。
    const int code = h.controller_->lastError().code;
    EXPECT_TRUE(code == aircraft::data::kErrRollbackExhausted
                || code == aircraft::data::kErrStateFailure)
        << "实际码 " << code << "，消息：" << h.controller_->lastError().message;
}

// ---------------------------------------------------------------------------
//  R05 契约违背：validate() 的返回值与它写入的 out.valid 不一致
// ---------------------------------------------------------------------------

TEST(MeasurementFlowTest, R05_验证契约被违背时必须可见失败且详情仍留存)
{
    // 契约（本批裁决）：`validate()` 的 bool ≡ `out.valid` ≡ "是否通过全部判据"，
    // **不是**"验证过程是否执行成功"。真实 PoseValidator 的每个返回点都满足该恒等，
    // 故控制器的对账分支在正确实现下**永不可达** —— 桩的 `validateLies`
    // 就是"实现坏掉"的替身，也是本接口下唯一可测的错误形态。
    //
    // 要证三件事：
    //   ① 必须**失败**，不得静默按任一方继续；
    //   ② 失败必须**可见地指出不一致**（两个值都出现在消息里）；
    //   ③ 验证详情**仍然留存** —— 否则等于在新的通路上原样复现 R05 要修的缺陷
    //      （"验证过程出问题时，验证详情反而最不可得"）。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain  = 0.5;
    h.pipeline_.validateLies = true;
    h.build();

    ASSERT_GE(h.run(0, 400), 0) << "未在给定 tick 内终止";
    EXPECT_EQ(h.controller_->state(), S::FAILED);

    // 作"根因"来读，而不是读 lastError()：后续 tick 还会因回退/重试写入别的记录，
    // lastError() 会被后来的症状覆盖。分工与 C-007 一致 —— 首次失败记根因。
    const aircraft::data::FailureTrace& f = h.recorder_.lastRecord.failure;

    // ① 可见失败
    EXPECT_EQ(f.firstError.code, aircraft::data::kErrStateFailure)
        << "首次失败的码是 " << f.firstError.code << "，消息：" << f.firstError.message;

    // ② 消息点出不一致 —— 只说"验证失败"是不够的：那正是把契约违背
    //    与"判不合格"混为一谈的写法，也正是原实现读错返回值的那一步。
    EXPECT_NE(f.firstError.message.find("验证契约违背"), std::string::npos)
        << "消息未点出契约违背，实际：" << f.firstError.message;
    EXPECT_NE(f.firstError.message.find("out.valid"), std::string::npos)
        << "消息未给出被违背的那个字段，实际：" << f.firstError.message;

    // 它必须发生在 VALIDATE —— 若这条记录出现在别处，说明失败另有来源。
    EXPECT_EQ(f.firstFailedState, S::VALIDATE);

    // ③ 详情留存。桩的这次调用写入的是 out.valid = true（pose.success 为真、
    //    未置 validateFails），却返回 false —— 于是"权威值"与"返回值"
    //    正好相反，两条断言各自钉住一侧：消息说返回 false、详情说 out.valid 为真。
    EXPECT_TRUE(h.controller_->validationResult().valid)
        << "对账分支把 out.valid 一并丢了 —— 那正是 R05 要修的缺陷形态";
    EXPECT_NEAR(h.controller_->validationResult().reprojectionError, 0.42, 1e-9);
    EXPECT_NEAR(h.controller_->validationResult().inlierRatio, 0.91, 1e-9);
    EXPECT_NEAR(h.controller_->validationResult().confidence, 0.88, 1e-9);

    // ④ 因果对照：**同一条装配路径**，只把那个谎撤掉，任务应当正常走完。
    //
    //    这一条是"先红后绿"的内建版：它排除"H 组装配本身有问题"这一解释
    //    （否则上面的 FAILED 可能来自夹具，而不是来自被违背的契约），
    //    把失败的原因**唯一地**钉在 validateLies 上。
    //
    // ⚠ 本用例**不**断言 `rollbackCount() == 0`，虽然"契约违背不应被当成判不合格"
    //   会让人想这么写。实测该值非零，而这是**正确的**：
    //   `handleFailure(TRANSIENT)` 走的是本工程标准的瞬态恢复（回退到
    //   MEASURE_SELECT），`rollbackCount()` 数的是**回退迁移次数**，
    //   与"有没有排除通道"是两件事 —— 对账分支本身在 `handleFailure` 之后
    //   立即 return，**没有**调用 `excludeCamera`（处置动作留在
    //   `if (!validation.valid)` 分支里，见代码）。
    //   把它写成 0 是在断言一个设计从未承诺的性质；这条注释留在原处，
    //   免得后来者又想把它加回去。
    Harness c;
    c.turntableConfig_ = configuredTurntable();
    c.turntable_.gain  = 0.5;
    c.pipeline_.validateLies = false;   // 唯一的差异
    c.build();

    ASSERT_GE(c.run(0, 400), 0) << "对照组未在给定 tick 内终止";
    EXPECT_EQ(c.controller_->state(), S::COMPLETE)
        << "撤掉那个谎之后任务仍不成功 —— 那么上面的 FAILED 不能归因于契约违背";
}

// ===========================================================================
//  SYS-08 §10 用例 4：升级规则
// ===========================================================================

TEST(MeasurementFlowTest, 用例4_升级规则_PnP恒失败时第2次必须换输入)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.pipeline_.solveFails = true;
    h.build();

    ASSERT_GE(h.run(0, 400), 0);

    // 判据一（§7.3 升级规则本身，也是文档给本用例的唯一判据）：
    // 第 2 次尝试的输入与第 1 次**不相同**。
    ASSERT_GE(h.pipeline_.solveCameras.size(), 2u);
    EXPECT_NE(h.pipeline_.solveCameras[0], h.pipeline_.solveCameras[1])
        << "第 2 次必须换用次优相机（§7.3 / §5.8），否则只是重算同一个失败";

    // 判据二：POS_SOLVE 的尝试上限为 2（§7.3）。
    EXPECT_EQ(h.controller_->attempts(S::POSE_SOLVE), 2);

    // 判据三：必然终止于 FAILED，且回退有界。
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_LE(h.controller_->rollbackCount(), 4);
    EXPECT_LE(h.backwardTransitions(S::POSE_SOLVE, S::MEASURE_SELECT), 2);

    // ⚠ 本用例的终止码同样不是 9002，机制与用例 3 不同，如实记录：
    //
    //   t=17  POSE_SOLVE#2 换到 CAM50 后仍失败
    //   t=18  POSE_SOLVE 的两次机会用尽 → 回退#1 到 MEASURE_SELECT
    //   t=19  MEASURE_SELECT#2 → CAPTURE#2 → CAPTURE 要进 POSE_SOLVE
    //   t=20  POSE_SOLVE 上限已满 → **进入 POSE_SOLVE 的转换被拒**；
    //         而 CAPTURE 的这一次动作已被 §7.6 约束 2 记在 CAPTURE 头上
    //   t=21  CAPTURE 的第 3 次机会也这样用掉 → 回退#2（CAPTURE→MEASURE_SELECT）
    //   t=23  MEASURE_SELECT 的三次机会用尽 → FAILED，码 9004（C-006 前为裸 0）
    //
    // 也就是：**"回退次数"从未成为约束**，先到者始终是某个状态的 §7.3 次数上限。
    // 罪魁是 §7.6 约束 2（"所有状态进入时必须调用 beginAttempt()"）——
    // 在一个状态的机会已被用尽时，**连"转换进入它"都会被判为一次尝试**，
    // 于是被拒的转换仍消耗**发起方**的配额。§7.4 的 9002 因此在本用例中
    // 同样不可达。这与用例 3 是同一个文档级矛盾的两个侧面，已合并登记在
    // README §6：§7.3 与 §7.4 的"取先到者"在 §7.6 约束 2 之下无法同时成立，
    // 需要一条裁决指明 9002 何时才应可观测。
    // ⚠ C-006 把兜底侧从裸 0 收紧为 9004，理由同用例 3 的对应断言。
    const int code = h.controller_->lastError().code;
    EXPECT_TRUE(code == aircraft::data::kErrRollbackExhausted
                || code == aircraft::data::kErrStateFailure)
        << "实际码 " << code << "，消息：" << h.controller_->lastError().message;
    EXPECT_FALSE(h.controller_->lastError().message.empty());
}

// ===========================================================================
//  SYS-08 §10 用例 5：硬件故障不重试（降级继续）
// ===========================================================================

TEST(MeasurementFlowTest, 用例5_单台相机断连时降级继续且不消耗重试预算)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.scene_.channels = 2;   // CAM100 断连
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::COMPLETE)
        << "单台相机故障不得终止测量（§7.5：降级继续）";

    // 判据：degraded 可见 + 记录 cameras_available（§7.5）。
    EXPECT_TRUE(h.controller_->degraded());
    EXPECT_EQ(h.controller_->availableCameraCount(), 2);
    EXPECT_EQ(h.controller_->degradationNotice().code,
              aircraft::data::kErrCameraDegraded);

    // 判据：不进入重试计数 —— 除 ALIGN 外每个状态都只被进入过一次，
    // 且没有任何回退（降级不是失败，不该消耗 §7.4 的预算）。
    //
    // ⚠ ALIGN 被排除在外不是放宽要求：对准本身就是多次动作（本项目
    // 4 次收敛），§7.3 给它 8 次上限正是为此。真正要证明的是
    // "降级没有让任何一个**后续**状态被重复进入"。
    EXPECT_EQ(h.controller_->rollbackCount(), 0);
    EXPECT_EQ(h.controller_->attempts(S::SEARCH), 1);
    EXPECT_EQ(h.controller_->attempts(S::TARGET_FOUND), 1);
    EXPECT_EQ(h.controller_->attempts(S::STABILIZE), 1);
    EXPECT_EQ(h.controller_->attempts(S::MEASURE_SELECT), 1);
    EXPECT_EQ(h.controller_->attempts(S::CAPTURE), 1);
    EXPECT_EQ(h.controller_->attempts(S::POSE_SOLVE), 1);
    EXPECT_EQ(h.controller_->attempts(S::VALIDATE), 1);
    EXPECT_EQ(h.controller_->attempts(S::SAVE), 1);
    EXPECT_LE(h.controller_->attempts(S::ALIGN), 8);
}

// ===========================================================================
//  SYS-08 §10 用例 6：降级下限
// ===========================================================================

TEST(MeasurementFlowTest, 用例6_断开两台相机时立即1001且不消耗重试预算)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.scene_.channels = 1;   // 只剩 CAM25
    h.build();

    const int ticks = h.run(0, 200);

    // 判据：立即 FAILED（第一次采集即判定），code == 1001。
    ASSERT_GE(ticks, 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrCameraInsufficient);
    EXPECT_LE(ticks, 1) << "≤1 路相机应在首次采集后立即失败，而不是反复重试";

    // 判据：不消耗重试预算 —— SEARCH 只进入过一次，且无任何回退。
    EXPECT_LE(h.controller_->attempts(S::SEARCH), 1);
    EXPECT_LE(h.controller_->attempts(S::TARGET_FOUND), 0);
    EXPECT_EQ(h.controller_->rollbackCount(), 0);

    // 硬件故障不重试（§7.2）的量化体现：转台自始至终没有被指令过。
    EXPECT_EQ(h.turntable_.moveCount, 0);
}

// ===========================================================================
//  SYS-08 §10 用例 7：时限优先级
// ===========================================================================

TEST(MeasurementFlowTest, 用例7_时限与状态上限同时可达时报9001)
{
    // 构造：恒偏差 100 pixel（对准永不成功）+ ALIGN 上限 2 + T_task = 6 s。
    // 由用例 2 的时刻分析，ALIGN 的第 3 次尝试恰好落在 t = 6 s ——
    // 与 T_task 的到点**同一时刻**。§7.1 的 T_task 是硬保证，必须胜出。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.minMoveDeg = 5.0;
    h.config_.maxAlignAttempts = 2;
    h.config_.taskTimeoutNs = 6000000000ULL;   // 6 s
    preloadOffset(h, 100.0, 100.0);
    h.build();

    ASSERT_GE(h.run(0, 100), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code, aircraft::data::kErrTaskTimeout)
        << "两者同时成立时必须报 9001，而不是 ALIGN 的超限码 2003";
    EXPECT_EQ(h.elapsedNs(0), 6000000000ULL);
}

TEST(MeasurementFlowTest, 用例7对照_时限未到时由状态上限胜出报2003)
{
    // 与上一用例唯一的差别是 T_task 放宽到 30 s。若实现把"次数上限"的
    // 判定放在时限之前（或根本不查时限），两个用例会得到相同的码 ——
    // 那样 §10 用例 7 的"先到者生效"就没有被验证。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.minMoveDeg = 5.0;
    h.config_.maxAlignAttempts = 2;
    h.config_.taskTimeoutNs = 30000000000ULL;   // 30 s
    preloadOffset(h, 100.0, 100.0);
    h.build();

    ASSERT_GE(h.run(0, 100), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrAlignRetryExhausted);
    EXPECT_EQ(h.controller_->attempts(S::ALIGN), 2);
    EXPECT_LT(h.elapsedNs(0), h.config_.taskTimeoutNs);
}

// ===========================================================================
//  8 硬件故障的其余分支（§7.5 的转台行 + §7.2 的不重试）
// ===========================================================================

TEST(MeasurementFlowTest, 转台故障直接FAILED且不重试)
{
    // §7.5：转台无冗余，无法降级。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.errorState = true;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrTurntableComm);

    // 不重试的量化体现：ALIGN 只进入过一次（尝试 1 次即失败）。
    EXPECT_LE(h.controller_->attempts(S::ALIGN), 1);
    EXPECT_EQ(h.controller_->rollbackCount(), 0);
}

TEST(MeasurementFlowTest, 转台指令下发失败按瞬态重试)
{
    // ENG-09 §5.27：2001 是瞬态可重试（与"转台报故障状态"不同 ——
    // 通信失败可能只是偶发丢包）。转台重试仍受 ALIGN 的 8 次上限约束。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.failMove = true;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrAlignRetryExhausted)
        << "转台持续通信失败最终由 ALIGN 的次数上限收口（§7.3 / §7.7）";
    EXPECT_EQ(h.controller_->attempts(S::ALIGN), 8);
}

TEST(MeasurementFlowTest, 采集瞬时失败不消耗额外预算即可恢复)
{
    // §7.2 的瞬态：丢帧换一帧即可，属于"允许重试"的一类。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.cameras_.failNextCaptures = 1;   // 第一次采集失败
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::COMPLETE)
        << "一次丢帧不应把任务推成失败";

    // ⚠ 本行在 C-006 之前写的是 `== 0`，且当时**必然**通过 —— 那是巧合而非
    // 判据：`lastError_` 是"最近一次失败"的**事实记录**，成功恢复**不**清除
    // 它（清除会丢掉"这次任务曾丢过一帧"这一信息）。此前这一族的码全是裸 0
    // （0 的冻结语义是"未设置"、渲染为 "OK"），于是"记录了一次失败"与
    // "没有失败"在数值上恰好无法区分，断言才看起来成立。
    // C-006 把兜底码换成 9004 之后，这一行的**真实语义**才显形：
    // 任务成功（上一行已断言 COMPLETE），但过程里确实记下过一次失败。
    //
    // 本桩 `VirtualCameras` 未填写 `IMultiCameraManager::lastError()`
    // （这是桩的正当权利，见该接口的默认实现说明），故控制器按兜底码记录；
    // 真实设备会在这里给出 1001 或 3002。判据取"非 0"而非某个具体值，
    // 正是为了不把桩的沉默误当成设备的事实。
    EXPECT_TRUE(h.controller_->lastError().code
                    == aircraft::data::kErrStateFailure
                || h.controller_->lastError().code
                       == aircraft::data::kErrSyncOutOfTolerance)
        << "过程里丢过一帧，lastError_ 应留下非 0 的记录而不是被清成 0；"
           "实际码 " << h.controller_->lastError().code;
}

TEST(MeasurementFlowTest, 转台越程按能力边界立即FAILED报2002)
{
    // §7.7：转台越程是**能力边界**，重试 8 次也到不了，必须立刻停。
    Harness h;
    TurntableConfig tight = configuredTurntable();
    tight.azimuthMin = -1.0;
    tight.azimuthMax = 1.0;      // 行程 ±1°，目标在 20°
    h.turntableConfig_ = tight;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code,
              aircraft::data::kErrTurntableOverTravel);

    // 能力边界不重试：只尝试过 1 次就停（对比上一条用例的 8 次）。
    EXPECT_EQ(h.controller_->attempts(S::ALIGN), 1);
    EXPECT_EQ(h.turntable_.moveCount, 0) << "越程时不得向转台下发任何指令";
}

// ===========================================================================
//  9 人工取消（§7.3 的 9003）
// ===========================================================================

TEST(MeasurementFlowTest, 人工取消以9003终止并让转台停下)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.pipeline_.detectReturnsEmpty = true;   // 停在 SEARCH
    h.build();

    h.controller_->startMeasurement(0);
    uint64_t now = 0;
    for (int i = 0; i < 5; ++i)
    {
        h.controller_->tick(now);
        now += kStepNs;
    }
    ASSERT_EQ(h.controller_->state(), S::SEARCH);

    h.controller_->stopMeasurement();
    EXPECT_EQ(h.controller_->state(), S::FAILED);
    EXPECT_EQ(h.controller_->lastError().code, aircraft::data::kErrManualCancel);

    // 取消之后可以重新开始（FAILED → IDLE → SEARCH）。
    h.controller_->startMeasurement(now);
    EXPECT_EQ(h.controller_->state(), S::SEARCH);
    EXPECT_EQ(h.controller_->lastError().code, 0);
}

TEST(MeasurementFlowTest, 重复启动不打断进行中的任务)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.pipeline_.detectReturnsEmpty = true;
    h.build();

    h.controller_->startMeasurement(0);
    h.controller_->tick(0);
    h.controller_->tick(kStepNs);
    ASSERT_EQ(h.controller_->state(), S::SEARCH);

    h.controller_->startMeasurement(2 * kStepNs);   // 重复点击"开始"
    EXPECT_EQ(h.controller_->state(), S::SEARCH) << "进行中的任务不应被重启";
}

// ===========================================================================
//  10 SYS-08 §8：AUTO 模式下状态控制预览源
// ===========================================================================

TEST(MeasurementFlowTest, 状态变化应通知PreviewManager按SYS08_8切换显示源)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    aircraft::preview::PreviewManager preview({}, &h.rig_);
    h.preview_ = &preview;
    h.build();   // 重建控制器以带上 preview
    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0);
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // §8 的最后一次通知应停在终点状态；AUTO 源随状态走过 CAM25 → CAM50。
    EXPECT_EQ(preview.measurementState(), S::COMPLETE);
    EXPECT_EQ(preview.autoCamera(), h.controller_->selectedCamera())
        << "§8 的 MEASURE → Selected Camera 一行需要控制器把选中焦段一起告知";
}

// ===========================================================================
//  R04（2026-09-24 全仓审查报告）：测量期间的预览生产者
//
//  背景：MeasurementController 持有 preview_，却只调用 setAutoCamera 与
//  setMeasurementState，**从未调用 submitFrom**；而 SystemInitializer 的注释
//  声称"测量进行中 controller 每拍自己采集并提交预览"—— 该假设自写下来就
//  未实现。生产代码里唯一的 submitFrom 调用点在 SystemInitializer 的**空闲**
//  路径（pumpIdlePreview，只在 IDLE/COMPLETE/FAILED 执行）⇒ 测量期间画面空白。
//
//  ⚠ 本用例**不**走 submitFrom 的模块单测（那个早已存在于
//    tests/preview/PreviewLayerTest.cpp，而调用链照样是断的 —— 正是报告警惕的
//    "单测已有、链路仍断"形态）。它从控制器 tick() 出发、经 acquire() 才到达
//    预览，这才是本次修复的那条路径。
// ===========================================================================

TEST(MeasurementFlowTest, R04_测量期间预览必须有生产者)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    aircraft::preview::PreviewManager preview({}, &h.rig_);
    h.preview_ = &preview;
    h.build();   // 重建控制器以带上 preview

    // 自己驱动 tick 而不复用 h.run()，因为要**逐 tick 排空队列**：
    // 队列容量只有 3~5 且满载丢旧保新，CAPTURE 会连采 5 次，
    // 不逐 tick 排空的话前面的帧会被挤掉，"帧号严格递增"就只能在队尾验证。
    h.controller_->startMeasurement(0);
    uint64_t now = 0;

    std::vector<uint64_t> ids;
    std::vector<aircraft::data::CameraRole> roles;
    std::vector<aircraft::data::CameraRole> expectedRoles;
    std::vector<S> states;

    for (int i = 0; i < 200 && !h.finished(); ++i)
    {
        if (h.controller_->state() == S::CAPTURE)
        {
            h.cameras_.armCapturePhase();
        }

        // 提交发生在本 tick 的 acquire() 内，而显示源是由**上一次**迁移
        // （transitionTo → setMeasurementState）设定的 ⇒ 此刻读到的
        // displayCamera() 正是本 tick 提交时应写入的角色。
        const aircraft::data::CameraRole before = preview.displayCamera();
        const S stateBefore = h.controller_->state();

        h.controller_->tick(now);
        now += kStepNs;

        aircraft::data::PreviewFrame pf;
        while (preview.queue().pop(pf))
        {
            ids.push_back(pf.frame.frameId);
            roles.push_back(pf.frame.role);
            expectedRoles.push_back(before);
            states.push_back(stateBefore);
        }
    }

    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // 判据一（决定性的那条）：测量期间**确实有**帧进入预览队列。
    //   修复前这是本用例唯一会红的断言 —— 测试夹具里没有 SystemInitializer，
    //   空闲路径也不存在，故队列全程为空。
    ASSERT_FALSE(ids.empty())
        << "测量期间没有任何帧被提交给预览：预览的生产者只剩空闲路径（R04）";

    // 判据二：帧号**严格递增**。同一帧被重复投放会在这里被抓住 ——
    //   那正是"看起来有预览、其实是静止画面"的形态。
    for (std::size_t i = 1; i < ids.size(); ++i)
    {
        EXPECT_GT(ids[i], ids[i - 1])
            << "第 " << i << " 次提交的帧号 " << ids[i]
            << " 不大于前一次的 " << ids[i - 1] << "（重复投放或乱序）";
    }

    // 判据三：提交的那一路就是**当时的显示源**（submitFrom 的职责）。
    //   若控制器绕过 submitFrom 自己挑通道，这条会红。
    ASSERT_EQ(roles.size(), expectedRoles.size());
    for (std::size_t i = 0; i < roles.size(); ++i)
    {
        EXPECT_EQ(roles[i], expectedRoles[i])
            << "第 " << i << " 帧的角色与当时显示源不符";
    }

    // 判据四：覆盖的是**整条活动链**，不是只有 SEARCH。
    //   取出现过的状态集合大小 —— 只有 SEARCH 能提交的话这里会退化。
    std::vector<S> distinctStates = states;
    std::sort(distinctStates.begin(), distinctStates.end(),
              [](S a, S b) { return static_cast<int>(a) < static_cast<int>(b); });
    distinctStates.erase(std::unique(distinctStates.begin(), distinctStates.end()),
                         distinctStates.end());
    EXPECT_GE(distinctStates.size(), 3u)
        << "只在 " << distinctStates.size() << " 个状态里有提交，覆盖不完整";

    // 判据五：CAPTURE 的连采**全部投递**（不缓存、不延后），
    //   故帧号序列里必然出现同一个状态下的连续多次采集。
    EXPECT_GE(ids.size(), 5u);
}

TEST(MeasurementFlowTest, R04_未注入预览时不崩)
{
    // 负对照：preview_ == nullptr 是大量单测的常态（Harness 的默认值）。
    // submitPreview() 必须与 setAutoCamera 的判空同构，否则会新增一片崩溃面。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();   // preview_ 保持 nullptr

    ASSERT_EQ(h.preview_, nullptr);
    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0);
    EXPECT_EQ(h.controller_->state(), S::COMPLETE);
}

// ===========================================================================
//  R06（2026-09-24 全仓审查报告）：selectedScore 恒为 0
//
//  背景：成员 selection_ 声明了、也被 reset() 清过、也被 buildRecord() 读过，
//  但**从来没有被赋过真实值** —— 两处 selectCamera() 的结果都落在局部变量里。
//  于是记录里"选了哪台"对（selectedCamera 取的是成员）、"得分"恒 0，
//  两个字段都合法、都不报错（C-02 §1.5 的 D-C02-5）。
//
//  ⚠ 期望值 1.0 来自**测试自己的桩**（StubPipeline::selectCamera 写死
//    out.score = 1.0），不是读回实现成员 ⇒ 输入侧预言机，无法伪装。
// ===========================================================================

TEST(MeasurementFlowTest, R06_记录里的选中得分必须来自选择结论)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0);
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);
    ASSERT_EQ(h.recorder_.saveCalls, 1);

    // 判据一：得分不再恒 0。修复前这里是 0.0（selection_ 从未被赋值）。
    EXPECT_DOUBLE_EQ(h.recorder_.lastRecord.selectedScore, 1.0)
        << "selectedScore 恒为 0：选择结论落在了局部变量里（R06）";

    // 判据二：得分与角色**同源**。二者必须同时写回，
    //   否则会出现"新角色 + 旧得分"—— 两个值都合法、都不报错。
    EXPECT_EQ(h.recorder_.lastRecord.selectedCamera,
              h.controller_->selectedCamera());
    // ⚠ 不额外断言"记录得分 == 控制器成员得分" —— 那需要给
    //   MeasurementController 加一个公开访问器，而它的公开面受
    //   SYS-04 §4.5 冻结（本批不动）。上面的角色一致性加上"得分非 0"
    //   已足以锁定"两个字段同源"这一不变量：只改角色不改得分时，
    //   第 1 条断言（得分 1.0）就会红。
}

TEST(MeasurementFlowTest, R06_换机路径的得分也必须写回)
{
    // 上一条覆盖 stepMeasureSelect；本用例覆盖**另一个**调用点
    // switchToNextCamera（§7.3 升级规则第 2 条）。两处若只改一处，
    // 换机之后记录里的得分就会退回 0 —— 而"选了哪台"仍是对的，
    // 所以只看角色是发现不了的。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.pipeline_.validateFails = true;   // 驱动回退 → 换机
    h.build();

    const int ticks = h.run(0, 400);
    ASSERT_GE(ticks, 0);
    ASSERT_EQ(h.controller_->state(), S::FAILED);

    // 失败包同样要落盘（裁决 C-007），故 lastRecord 有效。
    ASSERT_EQ(h.recorder_.saveCalls, 1);

    EXPECT_DOUBLE_EQ(h.recorder_.lastRecord.selectedScore, 1.0)
        << "换机后得分退回 0：switchToNextCamera 的写回与角色脱节（R06）";
    EXPECT_EQ(h.recorder_.lastRecord.selectedCamera,
              h.controller_->selectedCamera());
}

// ===========================================================================
//  11 C-02：CAPTURE 的最佳帧数据链
//      （V2.1-C02_实施设计说明.md §4 的判据 1 / 2 / 4 / 6）
//
//  背景：SYS-08 §5.7 要求"多帧采集（5~10 frames），**选择**最佳帧"。
//  实现曾把 selectBestFrame() 的结果写入 bestFrameIndex_ 后**无人读取**，
//  PnP 用的是 lastFrame_（最后一次采集）—— 于是"评分最高的那一帧"与
//  "被解算的那一帧"可以是两张不同的图，而记录里的质量描述的是前者，
//  且不报错。这组用例锁定修复后的行为。
//
//  ⚠ 这三个用例都靠**桩的自然行为**制造场景（评分桩返回视图第 0 帧、
//  相机桩逐次自增采集序号），不引入任何"按帧号设定分数"的测试专用接口 ——
//  那种接口会让测试去迎合实现，而不是检验能力（C02 §5）。
// ===========================================================================

TEST(MeasurementFlowTest, C02判据1_被解算的必须是评分最高那一次采集)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    // 评分桩返回视图第 0 帧 ⇒ 最佳帧 = CAPTURE 的**第 1 次**采集，
    // 而最后一次采集是第 5 次。二者不同，判据才有鉴别力。
    h.pipeline_.bestViewIndex = 0;

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    ASSERT_EQ(h.cameras_.phaseAcquisitionIds_.size(), 5u)
        << "CAPTURE 应采集 captureFrameCount = 5 次";
    ASSERT_EQ(h.pipeline_.solvedFrameIds.size(), 1u);

    const uint64_t first = h.cameras_.phaseAcquisitionIds_.front();
    const uint64_t last  = h.cameras_.phaseAcquisitionIds_.back();
    const std::array<uint64_t, 3> solved = h.pipeline_.solvedFrameIds.front();

    EXPECT_NE(first, last) << "前提：第 1 次与第 5 次采集的帧号必须可区分";

    // 判据 1：PnP 的输入是**评分最高**那一次采集。
    EXPECT_EQ(solved[0], first)
        << "被解算的应是评分选中的那次采集，而不是最后一次采集";
    EXPECT_NE(solved[0], last)
        << "修复前 PnP 用的是 lastFrame_（最后一次采集）";

    // 判据 2：三路来自**同一次曝光**，否则结果包与 raw 对不上。
    EXPECT_EQ(solved[0], solved[1]) << "cam25 与 cam50 不是同一次采集";
    EXPECT_EQ(solved[1], solved[2]) << "cam50 与 cam100 不是同一次采集";
}

TEST(MeasurementFlowTest, C02判据4_选定焦段为空时索引映射不得错位)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    // CAPTURE 的第 2 次采集：CAM25 有帧号、无像素。
    h.cameras_.blankOnPhaseCapture_ = 2;
    h.cameras_.blankRole_ = CameraRole::CAM25;
    // 评分桩返回**视图最后一个** ⇒ 正确映射落到第 5 次采集；
    // 若误用"视图下标"直接索引采集数组，会落到第 4 次。
    h.pipeline_.bestViewIndex = -1;

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // 本用例的前提：测量通道确实是 CAM25（否则注入的空帧不在选定焦段上，
    // 视图不会缩短，用例就变成了恒真）。
    EXPECT_EQ(h.controller_->selectedCamera(), CameraRole::CAM25)
        << "前提：测量通道为 CAM25，空帧才落在选定焦段上";

    ASSERT_EQ(h.cameras_.phaseAcquisitionIds_.size(), 5u);
    ASSERT_EQ(h.pipeline_.viewSizes.size(), 1u);
    EXPECT_EQ(h.pipeline_.viewSizes.front(), 4u)
        << "空帧应只把它自己排除出评分视图（5 → 4），"
           "而不是让'第几次采集'与数组下标错位";

    ASSERT_EQ(h.pipeline_.solvedFrameIds.size(), 1u);
    const std::array<uint64_t, 3> solved = h.pipeline_.solvedFrameIds.front();

    // 判据 4：映射回**采集序号**，即第 5 次（视图最后一个）。
    EXPECT_EQ(solved[0], h.cameras_.phaseAcquisitionIds_.back())
        << "经 viewToAcq 映射后应指向第 5 次采集";
    EXPECT_NE(solved[0], h.cameras_.phaseAcquisitionIds_[3])
        << "若直接用视图下标索引采集数组，就会错位到第 4 次采集";
}

TEST(MeasurementFlowTest, C02判据6_采集帧数被夹到上界且可见)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.config_.captureFrameCount = 1000;   // 误配：远超 §5.7 的 5~10
    h.build();

    const int ticks = h.run(0, 400);
    ASSERT_GE(ticks, 0) << "未在 400 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // 每次采集留三路（约 15 MB），1000 帧 ≈ 15 GB —— 必须被夹住。
    EXPECT_EQ(h.cameras_.phaseAcquisitionIds_.size(), 20u)
        << "上界应为 20（C-02 §2.6），而不是原来的 100";

    bool noted = false;
    for (const std::string& n : h.controller_->notices())
    {
        if (n.find("夹取") != std::string::npos)
        {
            noted = true;
        }
    }
    EXPECT_TRUE(noted) << "夹取必须**可见**（既成事实不得静默）";
}

// ===========================================================================
//  C-02 Step 5：统计量的推送通道 + 记录的事实字段
//
//  裁决（2026-09-23）：**禁止**给 `IPosePipeline` 增加 `lastMatchResult()`
//  一类"内部状态查询"，改为算法在产生统计量时**推送**给观察者。
//  （同批的"死诊断接口清理"把这一整组已有的 6 个访问器也删掉了 ——
//  禁止新增与清理存量是同一条判据的两面。）
//
//  这组用例要证明的正是那个拒绝理由所指向的风险：
//  统计量"算出来了但没接到下游"时，记录里的字段全为 0 ——
//  而**全 0 是一个合法值**，只看记录是发现不了的。
//  因此判据必须落在"推送确实发生过"这件事上，而不只是记录的内容。
//
//  ⚠ 用例 9（未推送）与用例 8（已推送）是**成对**的：
//  单看任一个，"全 0"都能被解释成"算法没给"或"传丢了"；
//  成对之后，`statisticsReceived()` 是否区分得开才是判据。
// ===========================================================================

TEST(MeasurementFlowTest, C02判据8_统计量经推送通道进入记录)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    // 前提：算法真的推送过。若为 0，下面的断言变成"记录里恰好是默认值"，
    // 用例将失去鉴别力（控制器的成员本来就默认构造为全 0）。
    ASSERT_GT(h.pipeline_.statisticsPushes, 0)
        << "算法从未推送统计量 —— 本用例的前提不成立";
    ASSERT_EQ(h.recorder_.saveCalls, 1);

    EXPECT_TRUE(h.controller_->statisticsReceived())
        << "控制器没有把'收到统计量'这件事记下来";

    // 逐字段比对（载荷各项互不相等，故字段串位一定被抓住）。
    const aircraft::data::MeasurementStatistics& got =
        h.recorder_.lastRecord.statistics;
    const aircraft::data::MeasurementStatistics& want = h.pipeline_.payload;

    EXPECT_EQ(got.featureCount,      want.featureCount);
    EXPECT_EQ(got.matchCount,        want.matchCount);
    EXPECT_EQ(got.droppedByConflict, want.droppedByConflict);
    EXPECT_EQ(got.cadCount,          want.cadCount);
    EXPECT_EQ(got.textureCount,      want.textureCount);
    EXPECT_DOUBLE_EQ(got.spreadPx,   want.spreadPx);

    // 与记录里已有的"选择结果"对账：这两组量来自不同的算法阶段，
    // 不能是同一份数据的两个副本（C-04 的教训）。
    EXPECT_NE(h.recorder_.lastRecord.statistics.matchCount,
              h.recorder_.lastRecord.selectedQuality.matchCount)
        << "统计量的 matchCount 与 ImageQuality::matchCount **同名不同义**"
           "（后者是 RANSAC 内点），若相等说明其中一处填错了来源";
}

TEST(MeasurementFlowTest, C02判据9_算法未推送时记录可与推送全零区分)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.pipeline_.publishStatistics = false;   // 算法不推送
    h.build();

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    EXPECT_EQ(h.pipeline_.statisticsPushes, 0);
    EXPECT_FALSE(h.controller_->statisticsReceived())
        << "没有推送却报'已收到'—— 记录里的全 0 会被读成'算法给出的 0'";

    // 全 0 是**合法的**最小值（确实是算法给出的可能取值），
    // 故它本身不能作为"未收到"的证据；证据只能是上面那个标志。
    const aircraft::data::MeasurementStatistics& got =
        h.recorder_.lastRecord.statistics;
    EXPECT_EQ(got.featureCount, 0);
    EXPECT_EQ(got.matchCount, 0);
    EXPECT_DOUBLE_EQ(got.spreadPx, 0.0);

    // 反向对照：载荷本身非 0，故"记录里全 0"不可能是"推送内容本来就是 0"。
    EXPECT_NE(h.pipeline_.payload.matchCount, 0);
}

TEST(MeasurementFlowTest, C02判据10_控制器析构时必须注销观察者)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.build();

    ASSERT_EQ(h.pipeline_.observer, h.controller_.get())
        << "控制器应在构造时把自己登记为统计量观察者";

    // pipeline 的寿命长于 controller（Harness 的成员声明顺序即析构顺序），
    // 故若不注销，pipeline 会继续向一个已析构的对象推送 ——
    // 而那个地址此时可能已经住进了另一个对象。
    h.controller_.reset();

    EXPECT_EQ(h.pipeline_.observer, nullptr)
        << "控制器析构后 pipeline 仍持有它的地址（悬垂指针）";
}

TEST(MeasurementFlowTest, C02判据11_PnP未收敛时统计量仍须推送)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.pipeline_.solveFails = true;   // 匹配完成，但 PnP 不收敛
    h.build();

    const int ticks = h.run(0, 300);
    ASSERT_GE(ticks, 0) << "未在 300 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::FAILED);

    // 这正是最需要统计量来解释的一次失败（"检出够了却解不出来"）：
    // 推送点必须在**失败早退之前**，否则失败任务永远没有可解释的现场。
    EXPECT_GT(h.pipeline_.statisticsPushes, 0)
        << "推送点落在了 PnP 失败早退之后 —— 失败现场没有统计量";
    EXPECT_TRUE(h.controller_->statisticsReceived());
}

TEST(MeasurementFlowTest, D_C02_6_记录里的最佳帧带曝光序号)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    h.pipeline_.bestViewIndex = 0;   // 最佳帧 = 第 1 次采集（≠ 最后一次）

    const int ticks = h.run(0, 200);
    ASSERT_GE(ticks, 0) << "未在 200 个 tick 内终止";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    ASSERT_EQ(h.pipeline_.solvedFrameIds.size(), 1u);
    const std::array<uint64_t, 3> solved = h.pipeline_.solvedFrameIds.front();
    const aircraft::data::MultiCameraFrame& recorded =
        h.recorder_.lastRecord.bestFrame;

    // 记录里的 best_frame 必须**就是**被解算的那一组帧（C-02 判据 2
    // 在记录层面的复述）：结果包里的 raw 与 result.json 的位姿同源。
    EXPECT_EQ(recorded.cam25.frameId,  solved[0]);
    EXPECT_EQ(recorded.cam50.frameId,  solved[1]);
    EXPECT_EQ(recorded.cam100.frameId, solved[2]);

    // 曝光序号（裁决 D-C02-6）：0 是"未定"，绝不允许出现在记录里。
    EXPECT_NE(recorded.exposureIndex, 0u)
        << "最佳帧的曝光序号为 0（未定）—— 结果包无法回答"
           "'这几张 raw 是不是同一次曝光'";

    // 本桩内 exposureIndex 与采集序号同源（见 VirtualCameras::capture），
    // 故它应等于被解算那一次采集的帧号。
    EXPECT_EQ(recorded.exposureIndex, solved[0])
        << "曝光序号与解算的那一次采集对不上";
}

TEST(MeasurementFlowTest, D_C02_6_连续两次测量的曝光序号单调递增)
{
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain = 0.5;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);
    const uint64_t first = h.recorder_.lastRecord.bestFrame.exposureIndex;
    ASSERT_NE(first, 0u);

    // 同一次加电后的第二次测量：序号必须继续前进，不得按任务清零。
    // 若每次测量都从 1 开始，"第 N 次曝光"在跨任务比对时就是假的 ——
    // 而它恰恰是 raw 与设备日志对账的依据。
    // ⚠ 这里的 `t2 > 1` 不是形式检查，它锁定的是一个真实缺陷：
    //    `startMeasurement()` 曾把 `retry_.beginTask()` 排在"回到 IDLE"
    //    **之后**，而 `transition()` 会对目标状态上报一次尝试 ——
    //    该上报要查 T_task，此时窗口还是**上一次任务**的。于是上一次任务
    //    的 deadline 一过，COMPLETE → IDLE 这条唯一入口就被关死，
    //    每次点击都在同一步返回，**系统再也开不出第二次测量**。
    //    当时的现场：run() 返回 0（未推进任何 tick），state() 仍是 COMPLETE
    //    （看起来像"第二次测量瞬间完成"），而 notices 里只有一条
    //    "无法回到 IDLE：任务时限 T_task 已到，不再允许任何重试"。
    const int t2 = h.run(100000000000ULL, 200);
    ASSERT_GE(t2, 0) << "第二次测量未终止";
    ASSERT_GT(t2, 1) << "第二次测量没有真正推进（启动被拒）";
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);
    EXPECT_EQ(h.recorder_.saveCalls, 2) << "第二次测量没有落盘";

    const uint64_t second = h.recorder_.lastRecord.bestFrame.exposureIndex;
    EXPECT_GT(second, first)
        << "第二次测量的曝光序号（" << second << "）不大于第一次（"
        << first << "）";
}

// ===========================================================================
//  9 裁决 C-006：失败任务的终态错误码不得是 0
// ===========================================================================

TEST(MeasurementFlowTest, C006_全部失败通路的终态码都是登记码)
{
    // 裁决 C-006 的验收判据原文：
    //   "任何 FAILED 任务的 result.json / 日志中不再出现 `code = 0`"。
    //
    // ⚠ 这条判据为什么必须是**跨通路**的而不是逐点断言：
    //   0 的冻结语义是"未设置"，在 result.json 中被渲染成 "OK"
    //   （`errorCodeName(0) == "OK"`）。逐点改完并不等于全通路干净 ——
    //   只要还有**一条**路径把 0 交到 failWith()，现场就会出现一台
    //   "看起来成功了"的失败任务，而它的存在只能靠遍历所有通路发现。
    //
    // 本用例把几条**彼此成因完全不同**的失败通路各跑一遍，逐条检查
    // 终态码非 0 且 errorCodeName 认得它（不返回 "UNREGISTERED"）：
    //   · 落盘失败 3 次      —— SAVE 无登记码 → 9004
    //   · PnP 恒定不收敛      —— 机型库"已加载" → 9004（不是 5001）
    //   · 对准次数用尽        —— §7.7 有专属码 → 2003
    //   · 可用相机数 ≤ 1      —— §7.5 硬件故障 → 1001
    //   · T_task 到点         —— §7.1 硬保证 → 9001
    // 五条覆盖了"专属码 / 兜底码 / 设备段 / 任务级"四种来源。
    struct Case
    {
        const char* name;
        int  expectedCode;   ///< 0 表示"只要求非 0，不锁定具体值"
    };

    const Case cases[] = {
        {"落盘失败用尽", 0},
        {"PnP 恒定失败", 0},
        {"对准次数用尽", aircraft::data::kErrAlignRetryExhausted},
        {"相机数不足",   aircraft::data::kErrCameraInsufficient},
        {"任务时限到点", aircraft::data::kErrTaskTimeout},
    };

    for (const Case& c : cases)
    {
        Harness h;
        h.turntableConfig_ = configuredTurntable();
        h.turntable_.gain   = 0.5;

        const std::string which = c.name;
        if (which == "落盘失败用尽")
        {
            h.recorder_.succeed = false;
        }
        else if (which == "PnP 恒定失败")
        {
            h.pipeline_.solveFails = true;
        }
        else if (which == "对准次数用尽")
        {
            h.turntable_.minMoveDeg = 5.0;   // 每次移动都过头，永不进入 ±50
            preloadOffset(h, 100.0, 100.0);
        }
        else if (which == "相机数不足")
        {
            h.scene_.channels = 1;           // 只剩 CAM25
        }
        else if (which == "任务时限到点")
        {
            h.pipeline_.detectReturnsEmpty = true;
        }

        h.build();
        ASSERT_GE(h.run(0, 300), 0) << which << "：未在给定 tick 内终止";
        ASSERT_EQ(h.controller_->state(), S::FAILED) << which << "：未进入 FAILED";

        const int code = h.controller_->lastError().code;
        EXPECT_NE(code, 0)
            << which << "：终态码是 0（= \"未设置\"，在包中渲染为 \"OK\"），"
                       "这正是 C-006 要消除的形态。消息："
            << h.controller_->lastError().message;

        // 非 0 还不够：它必须是一个**已登记**的码，否则按名检索的自动化
        // 查询仍然失效（errorCodeName 对表外码返回 "UNREGISTERED"）。
        EXPECT_STRNE(aircraft::data::errorCodeName(code), "UNREGISTERED")
            << which << "：码 " << code << " 不在 ENG-09 §5.27 登记表内";

        if (c.expectedCode != 0)
        {
            EXPECT_EQ(code, c.expectedCode)
                << which << "：该通路有专属码，不应落到兜底码。消息："
                << h.controller_->lastError().message;
        }
    }
}

TEST(MeasurementFlowTest, C006_兜底码不得吞掉有专属码的通路)
{
    // 上一条用例只验了"非 0"。本用例验的是 C-006 的另一半要求：
    // **9004 只用于确实无法归类者**。若实现图省事把每条失败都写成 9004，
    // 上一条会全绿，而"按码区分故障类别"这一目的完全落空 ——
    // 那等于把 0 换成了另一个常数，没有换来任何分类能力。
    //
    // 判据：两条**成因完全不同**的通路必须给出**不同**的码，且都不是 9004。
    auto alignmentExhaustion = []() {
        Harness h;
        h.turntableConfig_ = configuredTurntable();
        h.turntable_.minMoveDeg = 5.0;
        preloadOffset(h, 100.0, 100.0);
        h.build();
        EXPECT_GE(h.run(0, 300), 0);
        return h.controller_->lastError().code;
    };
    auto cameraShortage = []() {
        Harness h;
        h.turntableConfig_ = configuredTurntable();
        h.scene_.channels = 1;
        h.build();
        EXPECT_GE(h.run(0, 300), 0);
        return h.controller_->lastError().code;
    };

    const int alignCode  = alignmentExhaustion();
    const int cameraCode = cameraShortage();

    EXPECT_EQ(alignCode,  aircraft::data::kErrAlignRetryExhausted);
    EXPECT_EQ(cameraCode, aircraft::data::kErrCameraInsufficient);
    EXPECT_NE(alignCode, cameraCode)
        << "两条成因不同的通路给出同一个码，等于没有分类能力";
}

// ===========================================================================
//  10 裁决 C-007：根因与用尽症状必须在同一条记录里并存
// ===========================================================================

TEST(MeasurementFlowTest, C007_根因与用尽症状并存于同一条记录)
{
    // C-007 的验收原文："一条错误信息里**同时**出现两者"。
    //
    // ⚠ 为什么"同时"是判据而不是"根因存在即可"：
    //   只记根因，现场不知道这次失败被耗到了哪一步（该不该调重试策略）；
    //   只记终点（这是修复前的现状），现场会照着终点的方向去修 ——
    //   而终点往往只是症状。典型：机型库没装 → 每次都解不出来 →
    //   回退预算用尽报 9002。只报 9002 会让人去调重试次数，
    //   而真正要修的是模型没加载。
    //
    // 链路固定为"机型库未加载"（装配点注入 modelsAvailable = false，
    // 见 C-006 §1.3）：算法接口是纯 bool，静默 return false 时控制器
    // 无从区分它与真的 PnP 失败，故由唯一知道加载结果的地方注入。
    auto runWithModelLibrary = [](bool available) {
        Harness h;
        h.turntableConfig_ = configuredTurntable();
        h.turntable_.gain   = 0.5;
        h.modelsAvailable_  = available;
        h.pipeline_.solveFails = true;   // PnP 恒不收敛
        h.build();
        EXPECT_GE(h.run(0, 300), 0) << "未在给定 tick 内终止";
        EXPECT_EQ(h.controller_->state(), S::FAILED);
        return h.recorder_.lastRecord.failure;
    };

    const aircraft::data::FailureTrace missing = runWithModelLibrary(false);

    // ---- 第一问：根因 ----
    EXPECT_EQ(missing.firstError.code, aircraft::data::kErrModelMissing)
        << "首次失败的码是 " << missing.firstError.code << "（"
        << aircraft::data::errorCodeName(missing.firstError.code)
        << "），应为 5001。消息：" << missing.firstError.message;
    EXPECT_EQ(missing.firstFailedState, S::POSE_SOLVE)
        << "根因所在状态不是 POSE_SOLVE —— 只给码不给状态时，"
           "同一个码在不同状态下的处置方向分不开";

    // ---- 第二问：终点是**另一个**码（症状）----
    EXPECT_NE(missing.finalError.code, 0)
        << "终态码是 0（= \"未设置\"，渲染为 \"OK\"），C-006 的验收判据未达成";
    EXPECT_NE(missing.finalError.code, missing.firstError.code)
        << "首末同码（" << missing.finalError.code
        << "）—— 这条链路里就没有'根因 vs 症状'的区分，"
           "C-007 要消除的正是'只报终点，把现场引向错误方向'";

    // ---- 第三问：过程可复原 ----
    ASSERT_FALSE(missing.history.empty())
        << "轨迹为空 —— 失败包只能回答'错了'，回答不了'走到哪一步'";
    EXPECT_EQ(missing.history.front().from, S::IDLE)
        << "轨迹没有从 IDLE 起头，中途的迁移被漏记过";
    for (std::size_t i = 1; i < missing.history.size(); ++i)
    {
        EXPECT_EQ(missing.history[i].from, missing.history[i - 1].to)
            << "第 " << i << " 条迁移与上一条不衔接";
    }
    EXPECT_EQ(missing.history.back().to, S::FAILED)
        << "轨迹的终点不是 FAILED";

    // 轨迹里必须有一条**带根因码**的迁移（失败引起的回退/终止），
    // 否则"哪一次迁移是失败引起的"只能靠相邻元素推断 ——
    // 那正是 StateTransition::error 存在的理由。
    bool carriesRootCause = false;
    for (const aircraft::data::StateTransition& t : missing.history)
    {
        if (t.error.code == aircraft::data::kErrModelMissing)
        {
            carriesRootCause = true;
        }
    }
    EXPECT_TRUE(carriesRootCause)
        << "轨迹里没有任何一条带着根因码 —— history 退化成了纯状态列表";

    // ---- 判据的另一半：机型库**可用**时同一条通路不得报 5001 ----
    //
    // ⚠ 少了这一半，"永远报 5001"也能让上面的断言全绿 ——
    //   那样注入的 modelsAvailable 就只是一个摆设：
    //   代码里读它，但结果与不读一样。
    const aircraft::data::FailureTrace present = runWithModelLibrary(true);
    EXPECT_EQ(present.firstError.code, aircraft::data::kErrStateFailure)
        << "机型库已加载而 PnP 仍失败时，码应是兜底 9004（真的无法归类），"
           "而不是 5001 —— 否则'模型缺失'这个码会吞掉所有解算失败。消息："
        << present.firstError.message;
    EXPECT_NE(present.firstError.code, missing.firstError.code)
        << "注入的机型库可用性没有影响判定结果";
}

TEST(MeasurementFlowTest, C007_成功任务有路径而无根因)
{
    // C-007 的验收里还有一句"不得用空轨迹冒充成功"：
    // 成功任务必然走过了若干状态，只是**没有错误**。若实现图省事
    // 对成功任务不填轨迹（或整块不填），结果包里"成功"与"失败但轨迹丢了"
    // 就长得一样了。
    Harness h;
    h.turntableConfig_ = configuredTurntable();
    h.turntable_.gain  = 0.5;
    h.build();

    ASSERT_GE(h.run(0, 200), 0);
    ASSERT_EQ(h.controller_->state(), S::COMPLETE);

    const aircraft::data::FailureTrace& f = h.recorder_.lastRecord.failure;

    EXPECT_EQ(f.firstError.code, 0) << "成功任务不该有一个'根因'";
    EXPECT_EQ(f.finalError.code, 0) << "成功任务的终态错误码为 0（= 未设置）";

    ASSERT_FALSE(f.history.empty()) << "成功任务的轨迹不得为空（不得用空轨迹冒充成功）";
    EXPECT_EQ(f.history.front().from, S::IDLE);
    EXPECT_EQ(f.history.back().to, S::COMPLETE);
    for (std::size_t i = 1; i < f.history.size(); ++i)
    {
        EXPECT_EQ(f.history[i].from, f.history[i - 1].to)
            << "第 " << i << " 条迁移与上一条不衔接";
    }

    // 正常前进的迁移一律无码（0）。若实现给每条都塞一个码，
    // "失败引起的迁移"这个判据就失效了（见 StateTransition.h）。
    for (const aircraft::data::StateTransition& t : f.history)
    {
        EXPECT_EQ(t.error.code, 0)
            << "成功任务的轨迹里出现了非零错误码："
            << aircraft::data::errorCodeName(t.error.code) << "（"
            << t.error.message << "）";
    }
}
