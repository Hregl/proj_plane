#pragma once

// ============================================================================
//  src/infrastructure/config/ConfigManager.h
//
//  依据：ENG-01 §3.2（config/ 下 7 个 yaml）、§11（infrastructure/config）
//        ENG-03 §12.7（libinfrastructure 含 ConfigManager）
//        ENG-10 §5.3（**配置加载失败的三级处理**，本类的核心行为约定）
//        ENG-09 §3.1 R4（配置结构体在 data/，由 ConfigManager 装载填充）
//        SYS-04 §6.5（IF-FILE-01 配置接口）
//
//  ⚠ 本类**不是单例**。ENG-10 §5.1 把 `ConfigManager::instance()`
//    直接列为禁止写法，理由：单例让任意层隐式依赖 infrastructure，
//    且单元测试无法在不落盘 yaml 的情况下构造算法对象。
//    故本类由 app 创建、按对象传递（ENG-02 §15 的创建顺序：
//    ConfigManager 是第一个被创建的对象 —— 因为 Logger 的目录来自它）。
//
//  ⚠ 与 10.md §八 的差异：工作流文档把 Logger 排在 ConfigManager 之前，
//    但那在物理上不成立 —— `Logger::initialize()` 需要
//    `SystemConfig::logDir` / `logLevel`，而这两个值只有加载完
//    system.yaml 才知道。冻结文档（ENG-02 §15 / ENG-04 §3）的顺序是
//    **ConfigManager → Logger → OpticalRig → …**，按 README §6 的
//    冲突规则以冻结文档为准。已登记为偏离。
//
//  ⚠ 三级失败处理（ENG-10 §5.3，冻结）：
//      | 文件缺失 | 启动失败，**不得**用默认值继续
//      | 字段缺失 | 用结构体默认值填充，并记录字段名（本类记入
//      |          defaultedFields()，由 app 写进日志）
//      | 取值越界 | 启动失败
//    本类的 load() 即该三级处理的实现：文件缺失或越界返回 false 并把
//    原因写进 errors()，字段缺失则**继续**但登记到 defaultedFields()。
//    "记录使用了默认值的字段名"是硬要求 —— 少填一个 w1 会让评分静默失效，
//    必须让它在日志里可见。
// ============================================================================

#include <string>
#include <vector>

#include "data/CameraChannel.h"
#include "data/CameraConfig.h"
#include "data/CameraRole.h"
#include "data/MeasurementConfig.h"
#include "data/OpticalRigConfig.h"
#include "data/SystemConfig.h"
#include "data/TriggerConfig.h"
#include "data/TurntableConfig.h"
#include "data/ValidationConfig.h"

namespace aircraft
{
namespace infrastructure
{

/// 7 个运行配置文件的加载器。
class ConfigManager
{
public:
    ConfigManager() = default;
    ConfigManager(const ConfigManager&)            = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    /// 加载 configDir 下的全部 7 个 yaml。
    ///
    /// @param configDir 目录（默认 "config"，相对进程当前目录）。
    /// @return 全部文件存在且取值合法时返回 true。
    ///         任一文件缺失、解析失败或取值越界则返回 false，原因见 errors()。
    ///
    /// 返回 false 时**对象处于未加载状态**，各 getter 返回结构体默认值 ——
    /// 调用方必须直接终止启动，不得继续使用（ENG-10 §5.3）。
    bool load(const std::string& configDir);

    bool loaded() const { return loaded_; }
    const std::string& configDir() const { return configDir_; }

    // ---- 冻结的配置结构体（ENG-09 §3.1 R4：注入到使用方，不再经本类）----

    const data::SystemConfig&    system() const { return system_; }
    const data::OpticalRigConfig& opticalRig() const { return opticalRig_; }
    const data::TurntableConfig& turntable() const { return turntable_; }
    const data::TriggerConfig&   trigger() const { return trigger_; }
    const data::MeasurementConfig& measurement() const { return measurement_; }
    const data::ValidationConfig&  validation() const { return validation_; }

    /// 指定焦段的相机配置。role 非法时返回静态默认对象（调用方不应依赖）。
    const data::CameraConfig& camera(data::CameraRole role) const;

    /// 三个相机通道（camera.yaml 的 3 个条目 + focal_length）。
    /// 直接喂给 `optical::OpticalRig::initialize(channels)`。
    ///
    /// ⚠ `CameraChannel::focalLength` 的单位是**米**（ENG-09 §2.3），
    ///   故 100 mm 镜头在 yaml 里写 `focal_length: 0.1`。
    ///   写成 100.0 不会有任何编译或运行错误，只会让 SYS-14 §10 的
    ///   距离估计式偏大 1000 倍 —— 因此 load() 对焦距做了量级校验。
    const std::vector<data::CameraChannel>& channels() const { return channels_; }

    /// `optical_rig.yaml` 的 `calibration_mode`：**加载策略**，不是领域配置。
    ///
    ///   false（"file"）      —— 标定文件缺失即启动失败（ENG-10 §5.3）。
    ///   true （"synthetic"） —— 退回 `CalibrationManager::loadDefaults()`。
    ///
    /// ⚠ 这一项**不在** `data::OpticalRigConfig` 里（ENG-09 §5 的冻结字段
    ///   表没有它），故本类单独持有。理由：它是"文件缺失时怎么办"的策略，
    ///   属于加载器的行为；若将来裁决把它并入冻结结构体，此处即改为读取
    ///   该结构体字段。synthetic 取值仅用于 M1 合成闭环
    ///   （11.md §六把"默认矩阵标定"列为显式 mock）——用合成标定得出的
    ///   姿态**不具测量意义**，只是把数据通路跑通。
    bool calibrationSynthetic() const { return calibrationSynthetic_; }

    // ---- 诊断 ----

    /// 加载期间因**字段缺失**而使用默认值的字段全名（"measurement.w1" 形式）。
    /// 由 app 逐条以 WARN 写入日志（ENG-10 §5.3 的"记录字段名"）。
    const std::vector<std::string>& defaultedFields() const { return defaultedFields_; }

    /// 加载期间出现的告警：字段缺失之外的可疑但可继续的情况
    /// （如 illumBandEdges 未标定、焦距量级可疑）。
    const std::vector<std::string>& warnings() const { return warnings_; }

    /// 失败原因（文件缺失 / 解析失败 / 取值越界）。load() 返回 false 时非空。
    const std::vector<std::string>& errors() const { return errors_; }

    /// 7 个配置文件的完整路径（顺序与 ENG-01 §3.2 一致）。
    /// 供 Recorder 写结果包的 config_snapshot/（SYS-04 §6.4）。
    const std::vector<std::string>& configFiles() const { return configFiles_; }

    /// 单行摘要，供启动日志打印（"config: 7 文件 / 3 相机 / 0 默认值 / 0 告警"）。
    std::string summary() const;

private:
    void reset();

    std::string configDir_;
    bool        loaded_ = false;

    data::SystemConfig     system_;
    data::OpticalRigConfig opticalRig_;
    data::TurntableConfig  turntable_;
    data::TriggerConfig    trigger_;
    data::MeasurementConfig measurement_;
    data::ValidationConfig  validation_;
    data::CameraConfig      cameras_[3];
    std::vector<data::CameraChannel> channels_;
    bool                    calibrationSynthetic_ = false;

    std::vector<std::string> configFiles_;
    std::vector<std::string> defaultedFields_;
    std::vector<std::string> warnings_;
    std::vector<std::string> errors_;
};

}  // namespace infrastructure
}  // namespace aircraft
