#pragma once

// ============================================================================
//  src/app/ApplicationContext.h
//
//  依据：ENG-01 §14（app 模块：QApplication / 依赖创建 / 系统启动）
//        ENG-02 §15（**冻结的对象创建顺序**）
//        ENG-02 §16（对象生命周期与所有权）
//        ENG-03 §12.9（可执行文件的冻结链接表）
//        ENG-10 §5.1（ConfigManager 不是单例）、§5.2（注入后不可变）
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │ 本类是**所有长生命周期对象的家**。成员按 ENG-02 §15 的顺序声明，      │
//  │ 由 SystemInitializer 按同一顺序创建。                                  │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  ---- 为什么全部是 unique_ptr（而不是直值成员）----
//
//  设备的构造函数**需要配置**：
//      VirtualCameraBackend(const data::CameraConfig&)
//      VirtualTriggerController(const data::TriggerConfig&)
//      VirtualTurntable(const data::TurntableConfig&)
//      PosePipeline(measurement, validation, models, rig, statsStore)
//      MeasurementController(cameras, turntable, pipeline, rig, mCfg, tCfg, ...)
//  而配置只有 `ConfigManager::load()` 之后才存在，即**构造之后**。
//
//  直值成员的替代方案是"先默认构造、再赋值"，但那要求每个类都有默认构造
//  与赋值，且会短暂存在一批"半初始化"的对象 —— 这正是 ENG-10 §5.2
//  "注入后不可变"想避免的状态（一个字段被改过两次，第一次的值没有任何
//  意义，却恰好是任务期间可能被读到的那一个）。unique_ptr 用
//  "存在即已完全构造"消掉了这个中间态。
//
//  10.md §四/§五 的草图是直值成员 + 无参构造（`ApplicationContext ctx;`），
//  它对这些真实构造函数**无法编译**（不是风格差异）。见 README §6。
//
//  ---- 声明顺序 = 构造顺序；析构按逆序，这不是形式问题 ----
//
//  C++ 按声明顺序构造、**逆序**析构。下面成员的声明顺序即 ENG-02 §15 的
//  创建顺序，于是析构时最后创建者最先销毁：controller 先于 preview 销毁
//  （controller 持有 preview 裸指针），preview 先于 rig 销毁
//  （PreviewManager 持有 rig 裸指针），pipeline 先于 stats 销毁
//  （pipeline 持有 stats 裸指针）……声明顺序写错会让"关闭程序时偶发崩溃"
//  变成一个找不到原因的缺陷 —— Release 下多半不崩、Debug 下偶尔崩，
//  且看起来与本次改动无关。
//
//  ⚠ 本头文件包含 application / device / preview / algorithm 的具体头。
//    这在 app 里是**应当的**：app 是全工程唯一的装配点（ENG-01 §14），
//    不认识这些类型就没有办法装配。其他任何模块这么做都会破坏 ENG-01 §18。
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "algorithm/model/TargetModelManager.h"
#include "algorithm/pipeline/PosePipeline.h"
#include "algorithm/scale/TargetScaleEstimator.h"
#include "application/MeasurementController.h"
#include "device/camera/ICameraBackend.h"
#include "device/camera/MultiCameraManager.h"
#include "device/trigger/VirtualTriggerController.h"
#include "data/SyntheticTargetGeometry.h"
#include "data/TurntableConfig.h"
#include "device/turntable/ITurntableController.h"
#include "infrastructure/config/ConfigManager.h"
#include "infrastructure/logger/Logger.h"
#include "infrastructure/persistence/FileMatchStatsStore.h"
#include "infrastructure/recorder/Recorder.h"
#include "optical/CalibrationManager.h"
#include "optical/OpticalRig.h"
#include "preview/PreviewManager.h"
#include "preview/PreviewWorker.h"

namespace aircraft
{
namespace app
{

/// 运行形态元数据（批次 B2）。
///
/// ⚠ 这些量**不是算法的输入**：它们只回答"本次运行为什么形态、图像是谁
///   渲染的、真值从哪来"。装配点（SystemInitializer）在构造适配器时填入。
///   `truthAircraftToShip` 只用于渲染与事后比较，**不得**进入求解器。
struct RunMetadata
{
    /// system.yaml 的 run_mode。
    std::string runMode = "hardware_readonly";

    /// 机型类型（C-003）。仿真为 "synthetic"；真实机型库尚未实施，
    /// 其余形态留空（如实表达"事实尚不存在"）。
    std::string modelType;

    /// 真值来源；空表示本次运行没有独立真值。
    std::string truthSource;

    /// 逐项声明非真实组件（相机/转台/触发/场景/检测器……）。
    std::string componentRealism;

    /// 每路相机的后端名，顺序 CAM25 / CAM50 / CAM100。
    std::vector<std::string> cameraBackends;

    /// 本次**实际装载**的标定参数文本，顺序 CAM25 / CAM50 / CAM100 / RIG。
    /// 由装配点从 `ctx_.rig->calibration()` 生成（渲染场景与求解器用的是
    /// 同一份对象），使结果包可脱离包外文件核对内参/外参。
    std::vector<std::string> calibrationParameters;

    /// 是否存在独立真值；false 时 `truthAircraftToShip` 无意义。
    bool hasTruth = false;

    /// 渲染场景使用的真值（aircraftToShip）。
    data::Transform truthAircraftToShip;
};

/// `application::IRecorderSink` → `infrastructure::Recorder` 的适配器。
///
/// ⚠ 这个十行的类存在的唯一原因是冻结文档之间的一处反向依赖，
///   完整说明见 infrastructure/recorder/Recorder.h 的文件头：
///   · `IRecorderSink` 定义在 application/MeasurementController.h；
///   · `Recorder` 的归属被 ENG-03 §12.7 冻结在 infrastructure；
///   · ENG-01 §17 的依赖方向是 application → infrastructure，反向禁止（§18）。
///   于是两者只能由 app 这个"全工程唯一装配点"（ENG-01 §14）桥接。
///
/// 为什么适配器放在 app 而不是把 sink 下沉到 data：C-21 把
/// `IMatchStatsStore` 下沉到 data，是因为它的**消费者**（MeasurementSelector，
/// 算法层）不能依赖 infrastructure；而 `IRecorderSink` 的消费者是
/// application，application **本来就依赖** data 与 infrastructure
/// （ENG-03 §12.6）。下沉它换不来任何分层收益，代价是把"落盘"这一
/// infrastructure 概念提前泄漏到最底层。已登记待裁决（见 README §6）。
class RecorderSinkAdapter : public application::IRecorderSink
{
public:
    /// @param recorder 由 ApplicationContext 持有，生命周期长于本对象
    ///                 （析构顺序见文件头说明）。
    /// @param metadata 运行形态元数据；默认值即"现场只读、无真值"，
    ///                 与批次 B 之前的行为逐字相同。
    explicit RecorderSinkAdapter(infrastructure::Recorder& recorder,
                                 RunMetadata metadata = {})
        : recorder_(recorder)
        , metadata_(std::move(metadata))
    {
    }

    bool save(const data::MeasurementRecord& record) override
    {
        // 补齐"这次测量所依据的外部事实"——它们不是测量过程的产物，
        // 控制器没有来源（见 MeasurementController::buildRecord()）。
        // 装配点是唯一同时知道标定 / 机型库 / 编译期版本的地方（ENG-01 §14）。
        //
        // ⚠ 按值拷贝：`record.bestFrame` 里的三个 cv::Mat 是**浅拷贝**
        // （引用计数），不复制像素，代价是三个原子操作。
        data::MeasurementRecord filled = record;

        // ---- 外部事实的合并（四类情形，统一按同一规则）-------------------
        //
        // 装配点与记录方都可能持有同一字段的值。规则只有一条：
        //   · 装配值空、记录有值            → 保留记录值；
        //   · 装配值有值、记录空            → 按装配权威补齐；
        //   · 两边都有且相同                → 一致；
        //   · 两边都有且不同                → 采用装配值，并逐条留下诊断。
        // 冲突时**不得**把两个来源的值拼成一条身份（例如 model_id 取记录、
        // model_type 取装配）。冲突不改写“包已发布”这一事实，但必须可见。
        std::vector<std::string> conflicts;
        mergeString(filled.calibrationId, recorder_.calibrationId(),
                    "calibration_id", conflicts);
        mergeString(filled.modelId, recorder_.modelId(),
                    "model_id", conflicts);

        // ---- 运行形态（批次 B2）-------------------------------------------
        //
        // ⚠ `modelType` 不再固定留空：仿真路径下装配点会给出 "synthetic"，
        //   而 Recorder 据此判定 `model_type` 这一项"缺不缺"。空串仍表示
        //   "类型事实不存在"（真实机型库尚未实施），不是遗漏。
        //
        // ⚠ 装配点没提供 modelType（非仿真路径）时保留记录值：
        //   `RecorderAdapter.LeavesModelTypeEmptyUntilC003Lands` 的正对照
        //   （"production" 必须活着穿过适配器）依赖这一点。
        mergeString(filled.runMode, metadata_.runMode, "run_mode", conflicts);
        mergeString(filled.modelType, metadata_.modelType, "model_type", conflicts);
        mergeString(filled.truthSource, metadata_.truthSource,
                    "truth_source", conflicts);
        mergeString(filled.componentRealism, metadata_.componentRealism,
                    "component_realism", conflicts);
        mergeList(filled.cameraBackends, metadata_.cameraBackends,
                  "camera_backends", conflicts);
        mergeList(filled.calibrationParameters, metadata_.calibrationParameters,
                  "calibration_parameters", conflicts);

        // ---- 身份一致性：model_id / model_type / 版本必须同源 -------------
        //
        // 合并后三者应来自同一次装配：`recorder_` 的 modelId 与
        // feature_version 在 SystemInitializer 中由**同一个模型对象**给出；
        // `metadata_.modelType` 只在仿真路径、且在装载模型 id 等于合成 id
        // 之后才被填入。这里再核一次，防止未来出现“id 是合成、类型是
        // production”这类拼接。不一致时**不猜**：清空 model_type，
        // 让 `missing_required_fields` 如实报缺，并留下诊断。
        if (!filled.modelType.empty() && !filled.modelId.empty()
            && filled.modelType == data::synthetic::modelType()
            && filled.modelId != data::synthetic::modelId())
        {
            conflicts.push_back(
                "model_type/model_id: 类型=synthetic 但 model_id=\"" +
                filled.modelId +
                "\"；拒绝拼装身份，model_type 置空（按必填缺失处理）");
            filled.modelType.clear();
        }

        // 真值比较只在**解算成功**时有意义。失败包仍写入真值本身
        // （场景的固有事实，供复盘），但不给出误差 —— 见 Recorder.cpp
        // 中 `truth.error` 的写出条件。
        if (metadata_.hasTruth)
        {
            filled.hasTruth = true;
            filled.truthAircraftToShip = metadata_.truthAircraftToShip;
            if (filled.task.result.success)
            {
                filled.truthRotationErrorDeg =
                    rotationErrorDeg(metadata_.truthAircraftToShip.rotation,
                                     filled.task.result.aircraftToShip.rotation);
                const cv::Vec3d d =
                    filled.task.result.aircraftToShip.translation
                    - metadata_.truthAircraftToShip.translation;
                filled.truthTranslationErrorM =
                    std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            }
        }

        // 软件版本：编译期宏（cmake/BuildOptions.cmake 注入）。
        // 这是全工程**第一处**真正读取 `APS_VERSION_*` 的地方 ——
        // 在此之前两个宏已定义但零引用（README §6 第 26 行），
        // 即 ENG-08 §16 的"版本基线可见"实际上没有实现。
        mergeString(filled.softwareVersion,
                    std::string(APS_VERSION_STRING) + "-"
                        + std::string(APS_VERSION_STAGE),
                    "software_version", conflicts);

        // ⚠ `modelType` 的补齐点就在这里（见上方"运行形态"一段），
        //   不挪到控制器里 —— 它读不到模型库。空串表示"类型事实不存在"：
        //   C-003 的 `production` 机型库尚未实施，`TargetModelManager`
        //   目前只有 modelId / modelVersion，没有类型概念；仿真路径的类型
        //   由装配点按 `data::synthetic::modelType()` 显式给出。

        filled.metadataConflicts = std::move(conflicts);
        return recorder_.save(filled);
    }

    /// 最近一次落盘的目录（供界面与日志显示）。失败时是已创建的部分目录。
    const std::string& lastPackageDir() const
    {
        return recorder_.lastPackageDir();
    }

    const std::string& lastErrorText() const { return recorder_.lastErrorText(); }

private:
    /// 合并一个字符串事实（规则见 save() 内注释）。
    ///
    /// `assemblyValue` 为空表示装配点没有这项事实（**不是**“事实为空”）。
    /// 两边冲突时采用装配值并记一条诊断；不抛异常、不阻断落盘。
    static void mergeString(std::string&              target,
                            const std::string&        assemblyValue,
                            const char*               field,
                            std::vector<std::string>& conflicts)
    {
        if (assemblyValue.empty() || target == assemblyValue)
        {
            return;
        }
        if (target.empty())
        {
            target = assemblyValue;
            return;
        }
        conflicts.push_back(std::string(field) + ": record=\"" + target
                            + "\" vs assembly=\"" + assemblyValue
                            + "\"; 采用装配值");
        target = assemblyValue;
    }

    /// 合并一组有序事实（如每路后端、每路标定参数）。
    ///
    /// 整组按一个单元处理：要么保留记录整组，要么采用装配整组；
    /// 不允许逐位置混拼（否则 CAM50 可能来自装配、CAM100 来自记录）。
    static void mergeList(std::vector<std::string>&       target,
                          const std::vector<std::string>& assemblyValue,
                          const char*                     field,
                          std::vector<std::string>&       conflicts)
    {
        if (assemblyValue.empty() || target == assemblyValue)
        {
            return;
        }
        if (target.empty())
        {
            target = assemblyValue;
            return;
        }
        conflicts.push_back(std::string(field) + ": record 与 assembly 均为非空"
                            "且整组不同；采用装配整组（不做逐位置拼接）");
        target = assemblyValue;
    }

    /// 两个旋转之差的角度，单位 deg：`angle(R_a^T · R_b)`。
    ///
    /// 用矩阵迹公式而不是欧拉角逐项相减：后者在 ±180° 附近会给出
    /// "两个几乎相同的姿态相差 360°"这类假误差，且依赖分解顺序。
    static double rotationErrorDeg(const cv::Matx33d& a,
                                   const cv::Matx33d& b)
    {
        const cv::Matx33d r = a.t() * b;
        double c = (r(0, 0) + r(1, 1) + r(2, 2) - 1.0) * 0.5;
        c = std::max(-1.0, std::min(1.0, c));
        return std::acos(c) * 180.0 / CV_PI;
    }

    infrastructure::Recorder& recorder_;
    RunMetadata               metadata_;
};

/// 装配结果的持有者。**构造不做任何 I/O、不读文件、不开设备**。
class ApplicationContext
{
public:
    ApplicationContext() = default;

    /// 关闭时**逐路归还设备资源**（011-A1，§3.4）。
    ///
    /// ⚠ 为什么 `= default` 不够：`unique_ptr`／`shared_ptr` 的析构只释放
    ///    **内存**，对 SDK 侧的"相机还开着 / 还在取流"没有任何作用 ——
    ///    `IMV_StopGrabbing` / `IMV_Close` / `IMV_DestroyHandle` 都不会被调用。
    ///    后端自身的析构确实会兜底调一次 `close()`（幂等，见 ICameraBackend.h），
    ///    但兜底是**防漏调**的保险，不能当成主路径：
    ///    主路径必须是拥有者在关闭时显式停流、关设备，顺序才可读、结果才可记。
    ///
    /// ⚠ 顺序：先 `stop()`（停流）再 `close()`（关设备 + 销毁句柄）——
    ///    反过来是向一个已销毁的句柄停流。三路**逆装配序**处理，
    ///    与 `SystemInitializer::rollbackDevices()` 一致。
    ///
    /// ⚠ 这里**不**经 `MultiCameraManager`：加一层 `closeAll()` 转发会让
    ///    "谁负责释放"更模糊，而本类已直接持有三个后端（§3.4 明确否掉了
    ///    `closeAll()`）。`stopAll()` 的语义仍是"只停流、允许重复调用"，
    ///    不因本析构而改变。
    ~ApplicationContext()
    {
        shutdownDevices();  // 析构兜底；主路径是 SystemInitializer::shutdown()
    }

    ApplicationContext(const ApplicationContext&)            = delete;
    ApplicationContext& operator=(const ApplicationContext&) = delete;

    /// **公开、幂等**的设备关闭契约（C-23 / A3）。
    ///
    /// 顺序：转台（stop → 析构 join worker）→ 触发器（disable → 析构）
    /// → 相机（stop → close）。与 `SystemInitializer::shutdown()` 的
    /// 设备段一致；后者在"转台"与"触发器"之间还插入了预览 worker 的
    /// stop/join，那是线程资源，不属于设备。
    ///
    /// ⚠ `closeBackends()` 是**私有且只管相机**的实现细节，不能当作
    ///   关闭契约：它不碰转台、不碰触发器。本函数才是。
    /// ⚠ 幂等：`stop()`/`disable()`/`close()` 的契约都允许未初始化时调用，
    ///    `unique_ptr::reset()` 对空指针无副作用；另有标志避免重复执行。
    void shutdownDevices()
    {
        if (devicesShutdown_)
        {
            return;
        }
        devicesShutdown_ = true;

        // 转台：停止（只读下为无帧 no-op）后销毁 —— 析构 join worker、
        // 关闭 socket。必须在相机之前：它持有独立的 worker 线程。
        if (turntable)
        {
            turntable->stop();
            turntable.reset();
        }

        // 触发器：先撤使能，再销毁。
        if (trigger)
        {
            trigger->disable();
            trigger.reset();
        }

        // 相机：先使诊断回调失效（防止关闭期间的回报访问已销毁的对象），
        // 再停流 → 关设备（内部逐路逆序，见 closeBackends）。
        if (cameras)
        {
            cameras->setDiagnosticSink({});
        }
        closeBackends();
    }

    // ========================================================================
    //  以下顺序即 ENG-02 §15 的冻结创建顺序，序号写在注释里以便逐条对照。
    //
    //  ⚠ §15 的清单是**粗粒度**的九项（ConfigManager → Logger → OpticalRig
    //    → MultiCameraManager → TriggerController → TurntableController →
    //    → PreviewManager → MeasurementController → MainWindow），
    //    未列出 CalibrationManager / TargetModelManager / FileMatchStatsStore
    //    / PosePipeline / Recorder —— 因为它是"对象"清单，而这几项是
    //    009 装配时才出现的**协作者**。它们的位置不是自由选择：
    //    · CalibrationManager 必须在 OpticalRig 之前（rig 的标定由它装载）；
    //    · TargetModelManager 与 FileMatchStatsStore 必须在 PosePipeline
    //      之前（pipeline 以裸指针/引用持有二者）；
    //    · Pipeline 必须在 MeasurementController 之前（controller 以引用持有）；
    //    · Recorder 必须在 RecorderSinkAdapter 之前，后者又必须在
    //      MeasurementController 之前（controller 以裸指针持有 sink）。
    //    因此每一处插入点都由依赖方向唯一确定，不存在第二种排法。
    // ========================================================================

    /// 1. 配置（ENG-02 §15 的第一项）。
    ///    ⚠ 排在 Logger 之前不是笔误：Logger 的 log_dir / log_level
    ///    只能读到这里才知道（ENG-02 §15 原文）。10.md §八 给的
    ///    "Logger 先于 ConfigManager"顺序在物理上无法成立
    ///    （见 README §6 的偏离行）。
    std::unique_ptr<infrastructure::ConfigManager> config;

    /// 2. 日志。
    std::unique_ptr<infrastructure::Logger> logger;

    /// 3. 光机刚体 + 标定装载。
    ///    ⚠ 标定必须先于 rig：`rig.setCalibration()` 的入参来自
    ///    CalibrationManager，而 OpticalRig 的默认标定是空的
    ///    （空标定不会被当作"可用的零标定"，见 MeasurementController.h）。
    std::unique_ptr<optical::CalibrationManager> calibration;
    std::unique_ptr<optical::OpticalRig>         rig;

    /// 4. 三相机。后端用 shared_ptr 持有 —— 这是 IMultiCameraManager 的
    ///    构造签名要求的（三个 shared_ptr<ICameraBackend>）。
    std::shared_ptr<device::ICameraBackend> backend25;
    std::shared_ptr<device::ICameraBackend> backend50;
    std::shared_ptr<device::ICameraBackend> backend100;
    std::unique_ptr<device::MultiCameraManager> cameras;

    /// 装配用后端工厂（**测试注入槽**，011-A1 九项缺口 §1 的裁决）。
    ///
    /// 签名：`(配置, 错误输出) → 后端`；返回空即**该路装配失败**，错误文本
    /// 由第二参数带出（与内置 `makeBackend` 完全同形，故装配层不必区分
    /// 两种来源）。
    ///
    /// ⚠ 为空 ⇒ 走 `SystemInitializer` 的内置装配（`virtual` → 虚拟后端，
    ///   `imv` → 真实后端）。**生产路径恒为空** —— 它不是配置项，没有
    ///   配置文件键，只能由进程内的调用方（测试）在 `initialize()` 之前设置。
    ///
    /// ⚠ **一旦设置，返回空就到此为止：不得回落内置工厂。** 理由：本槽的
    ///   全部用途是"把某一路换成可观测的替身"，而静默换回真实实现会让
    ///   测试断言的对象与实际跑的对象不是同一个 —— 那种失败比直接失败
    ///   危险得多（测试会在**另一条**路径上通过，然后被当成证据）。
    ///
    /// ⚠ 这是 **app 内部装配能力**，**不增加设备 ICD 接口**：`ICameraBackend`
    ///   的签名、`IMultiCameraManager` 的方法面都不因它而变；它只决定
    ///   "装配点用谁去造后端"。
    std::function<std::shared_ptr<device::ICameraBackend>(
        const data::CameraConfig&, std::string& error)>
        backendFactory;

    /// 转台装配槽（C-22）。为空 ⇒ 按 `TurntableConfig::protocol` 内置装配：
    /// `sdk` → VirtualTurntable；`pelco_d_udp` → PelcoDTurntableController；
    /// `rs485`/`network` → 启动失败（不得静默回落虚拟后端）。
    ///
    /// 与 `backendFactory` 同一约定：一旦设置，返回空即装配失败，
    /// 不回落内置工厂。生产路径恒为空。
    std::function<std::unique_ptr<device::ITurntableController>(
        const data::TurntableConfig&, std::string& error)>
        turntableFactory;

    /// 5. 触发与转台。
    std::unique_ptr<device::VirtualTriggerController> trigger;
    std::unique_ptr<device::ITurntableController>     turntable;

    /// 6. 预览（注入 rig 用于显示焦段与校验可用性）。
    std::unique_ptr<preview::PreviewManager> preview;

    /// 6b. 预览消费者线程。
    ///
    /// ⚠ 它**不在** ENG-02 §15 的清单里，但少了它整条预览链是死的：
    ///    PreviewManager 只做"入队（submitFrame）"与"按显示源过滤后发布
    ///    （workerDeliver / getFrame）"，真正把帧从队列搬到发布槽的
    ///    PreviewWorker 是一个**独立的线程对象**，必须由装配点创建并
    ///    start()。漏掉它不会有任何报错：界面只是永远显示"无图像"占位，
    ///    而"没有帧"与"没有消费者"看起来完全一样。
    ///    声明在 preview 之后，故析构时先于 preview 停止（它持有
    ///    PreviewManager&），这一顺序是必须的。
    std::unique_ptr<preview::PreviewWorker> previewWorker;

    /// 7. 算法。机型库按引用被 pipeline 持有，故必须排在 pipeline 之前。
    std::unique_ptr<algorithm::TargetModelManager> models;

    /// 7b. M1 的桩尺度估计器（仅在机型库缺失时创建，见
    ///     SystemInitializer::buildAlgorithm 的说明）。
    ///     pipeline 以裸指针持有它，故必须排在 pipeline **之前**声明、
    ///     之后析构 —— 顺序反了就是关闭程序时的偶发崩溃。
    std::unique_ptr<algorithm::MockTargetScaleEstimator> mockScale;

    /// 8. 历史匹配统计（C-21：接口在 data，实现在 infrastructure）。
    ///    由 pipeline 以裸指针持有，故同样必须先于 pipeline。
    std::unique_ptr<infrastructure::FileMatchStatsStore> stats;

    /// 9. 算法链（持有 models 与 stats 的裸指针）。
    std::unique_ptr<algorithm::PosePipeline> pipeline;

    /// 10. 落盘（持有 configDir 以写 config_snapshot/）。
    std::unique_ptr<infrastructure::Recorder> recorder;

    /// 11. 测量总控（持有 cameras / turntable / pipeline / rig / preview /
    ///     trigger / recorderSink 的裸指针或引用，故必须最后创建）。
    std::unique_ptr<application::MeasurementController> controller;

    /// 12. 落盘通道适配器。
    ///     ⚠ 排在 controller **之后**是编译期强制的：controller 的构造
    ///     需要它的地址，而它又需要 recorder。构造顺序由
    ///     SystemInitializer::initialize() 显式控制，不依赖本结构体的
    ///     成员顺序（这里只是为了析构时的相对次序正确）。
    std::unique_ptr<RecorderSinkAdapter> recorderSink;

    // ========================================================================
    //  运行期状态
    // ========================================================================

    /// 本次启动实际使用的配置目录（相对进程当前目录或绝对路径）。
    std::string configDir = "config";

    /// 标定策略："file" = 从 calibrationDir 读文件（交付形态）；
    /// "synthetic" = 用 loadDefaults 生成默认矩阵（M1 阶段，见 README §6）。
    /// 该键是 009 引入的**装载策略**，不在冻结的 OpticalRigConfig 中
    /// （理由见 ConfigManager.h）。
    std::string calibrationMode = "file";

    /// false 表示本次启动**没有**可用特征库（model_dir 缺失）。
    /// 不构成启动失败（特征库属后续交付物），但会使 POSE_SOLVE 无法完成，
    /// 故必须在日志与界面上如实说出来，而不是静默地跑完整个状态机后失败。
    bool modelsAvailable = false;

    /// 启动过程中的告警（不致命，但必须可见）。
    std::vector<std::string> warnings;

    /// `shutdownDevices()` 是否已执行（幂等标志）。
    bool devicesShutdown_ = false;

private:
    /// 逐路停流并关设备（析构调用，见上文析构的说明）。
    ///
    /// 幂等：`stop()`／`close()` 的契约各自保证"未 start／未 initialize／
    /// 已 close 时调用无副作用"，故这里**不判状态**、直接调 ——
    /// 判状态会让"到过哪一步"这个只有后端自己知道的事泄漏进装配层。
    ///
    /// ⚠ 关设备失败只写 stderr、**不外抛也不吞掉**：析构函数没有返回值，
    ///    但它**有能力**把"资源没还回去"这条事实说出来。吞掉才是错的
    ///    （§2.2 修正 4 的同一条理由：清理失败必须可见）。
    void closeBackends()
    {
        const std::shared_ptr<device::ICameraBackend> order[] = {
            backend100, backend50, backend25};

        for (const std::shared_ptr<device::ICameraBackend>& b : order)
        {
            if (!b)
            {
                continue;
            }
            b->stop();
            const data::OperationResult r = b->close();
            if (!r.ok())
            {
                // 无 `sdkError` 时不写"码 0"：0 是 `IMV_OK`，
                // 在这里会被读成"SDK 说成功了"，与"后端没给出调用信息"正相反。
                const std::string detail =
                    r.sdkError ? data::sdkFailureText(*r.sdkError)
                               : std::string("未提供 SDK 调用信息");
                std::fprintf(stderr, "[WARN ] [app] 关闭相机后端失败：%s（%s）\n",
                             data::opStatusName(r.status), detail.c_str());
            }
        }
    }
};

}  // namespace app
}  // namespace aircraft
