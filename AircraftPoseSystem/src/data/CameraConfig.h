#pragma once

// ============================================================================
//  src/data/CameraConfig.h
//
//  依据：ENG-09 §4.1、§6.1（类型定义冻结）、§2.2 / §2.3（单位）、裁决 C-17
//
//  裁决 C-17（职责划分，冻结）：
//      CameraConfig  = **运行时可调**参数（曝光、增益、触发模式）
//      CameraChannel = **静态描述 + 标定**（相机 ID、焦段角色、焦距、标定）
//
//  为什么必须分成两个类型（C-17 的理由，实现时不要合并）：
//  两者的**生命周期**根本不同。CameraConfig 从 cameras/*.yaml 读入后可在
//  运行中按场景调整（逆光时改曝光），且**不参与离线复现的判定**——
//  它只影响图像怎么拍出来的。CameraChannel 来自标定文件，一旦标定就固定，
//  且**直接决定测量结果是否正确**（焦距错了，角度就错了）。
//  若合并为一个类型，则"改曝光"与"改标定"会走同一个写回路径，
//  一次误操作就能把标定值覆盖掉，且没有任何机制能发现。
//
//  ⚠ cameraId 与 role 的关系（2026-09-26 更正，裁决 C-01 v1.7）：
//  **cameraId 不是设备序列号**，它是**通道逻辑名**（`cam25`），来自
//  camera.yaml 的 `id` 键；role 是焦段角色。两者**不是一一对应**的语义 ——
//  role 决定用哪份标定和哪份特征库，cameraId 是"这三路之一"的名字。
//
//  ⚠ 本行原文写的是"cameraId 是设备序列号（华睿 A7A20MU201 的唯一标识）"，
//  那是**错的**，且正是"逻辑 ID ↔ 硬件序列号绑定"这条缺陷的**根源**：
//    ① 它把**型号**（A7A20MU201 是三台相机共有的型号）当成了唯一标识 ——
//       而唯一标识是**序列号**，同型号设备靠序列号区分；
//    ② 它让"按 cameraId 打开设备"看起来成立，而 cameraId 实际是
//       `cam25` 这样的逻辑名，拿它去匹配设备只能匹配到"第 0 个"。
//  真实设备身份由**SDK 枚举回报**取得，承载于 `data::DeviceIdentity`
//  （见 DeviceIdentity.h），并由**配置中的目标序列号**（本结构体的
//  `serialNumber` 字段）与之比对 —— 两者都**不**与 cameraId 混用。
// ============================================================================

#include <optional>
#include <string>

#include "data/CameraRole.h"
#include "data/CameraTriggerMode.h"

namespace aircraft
{
namespace data
{

/// 单台相机的运行时配置（ENG-09 §6.1）。
struct CameraConfig
{
    /// **通道逻辑名**（`cam25`），取自 camera.yaml 的 `id`。
    /// ⚠ 它不是设备序列号 —— 见本文件头 2026-09-26 的更正说明。
    /// 它参与装配、日志与结果包的通道标识，**不参与设备匹配**。
    std::string cameraId;

    /// 设备**序列号**（期望值，来自 camera.yaml 的 `serial`）。
    /// 空串 = 未绑定：真实后端遇到空序列号会**明确失败**，
    /// 不会退化成"取第 0 个设备"（那会让三台同型号相机随机互换）。
    std::string serialNumber;

    /// 后端类型：`"virtual"` 或 `"imv"`（来自 camera.yaml 的 `backend`）。
    ///
    /// ⚠ 缺该键 = 配置错误、启动失败 —— **不做隐式默认**。
    /// 理由：若按"检测到 SDK 就用真实后端"来默认，则现场 SDK 装好
    /// 的那一刻，虚拟配置会**静默变成**真实采集，而操作者以为自己在跑
    /// 仿真；反之 SDK 缺失时又静默退化为虚拟，使"没有真图"这件事
    /// 只在测量结果上体现。显式声明把这两种静默切换都消除掉。
    std::string backend;

    /// 焦段角色，决定标定与特征库的选择。
    CameraRole role = CameraRole::CAM25;

    /// 图像宽度，单位 pixel。必须与标定时的分辨率一致 ——
    /// 分辨率改变会直接平移 CameraCalibration::cameraMatrix 的主点，
    /// 而内参不会自动失效（见 CameraCalibration.h 的说明）。
    int width = 0;

    /// 图像高度，单位 pixel。
    int height = 0;

    /// 曝光时间，单位 **s**（ENG-09 §6.1 注释冻洁为 s，不是 ms 也不是 us）。
    /// 注意与 ImageFrame::exposureTime 同单位；与 ImageQuality::exposure
    /// 不同 —— 后者是 [0,1] 的合理性评分，不是时间。
    double exposureTime = 0.0;  // s

    /// 增益，单位 **dB**（ENG-09 §6.1 冻结）。来自 camera.yaml 的 `gain`。
    ///
    /// ⚠ `optional` 而不是 `double`（2026-09-28，C-018）：必须能区分
    /// **"没配增益"**与**"配了 0.0 dB"** —— 后者是合法值，若两者同形，
    /// "配置写漏了"就会被静默当成"配了 0 dB"，而这正是本工程反复出现的
    /// "字段存在、数值正常、语义不成立"那一类。同一理由适用于 `gainRaw`。
    ///
    /// ⚠ 本字段**只对确实声明 dB 语义 `Gain` 节点的设备可用**。它**不是**
    /// `gainRaw` 的别名，两者必须分别对待（见 `gainRaw` 的说明）。
    std::optional<double> gain;  // dB

    /// 增益的**设备原生数值**，来自 camera.yaml 的 `gain_raw`，**无单位解释**。
    ///
    /// ⚠ 为什么需要它，而不把配置里的 dB 值直接写进设备（C-018，2026-09-28）：
    /// `gain = 6.0` 的含义是 **6 dB**。若把 6.0 原样写进设备的 `GainRaw`
    /// 节点，则"写 6、读回 6"只证明**写入成功**，**证明不了单位是 dB** ——
    /// 项目就此替设备假定了一个未经验证的换算（1 原生值 = 1 dB）。增益直接
    /// 改变像素灰度，灰度进入特征提取，最终表现为测量角偏差，且全程不报错。
    ///
    /// 实测依据（2026-09-28，A7A20MU201，序列号 `FD88772AAK00078`，
    /// 固件 `V1.000.00.0.R(20230529,252473)`，读自设备自带 GenICam XML
    /// 与 `IMV_*Feature*` 调用）：
    ///   · **无单位**：`GainRaw` 节点内**没有 `<Unit>` 子节点**；整份 XML 里
    ///     `<Unit>` 的全部取值只有 `C`／`Hz`／`us`。⚠ 措辞纪律："dB" 在 XML 中
    ///     确实出现 20 次，但**全部**属于 `DeEmphasisA`（以太网预加重的枚举项名
    ///     与说明，属**传输层**信号完整性设置），**没有一处**与 `Gain`／
    ///     `GainRaw` 关联 —— 不得简化成"XML 里没有 dB"，那句话是**假的**，
    ///     而假的理由正是排查时最该被看见的那类细节。
    ///   · **无换算依据**：XML 与 SDK 均未给出"原生值 → dB"的任何表达式或说明。
    ///   · `GainRaw` 为 `<Float Name="GainRaw" NameSpace="Standard">`，
    ///     `Representation Linear`、节点内静态 `<Max>32</Max>`、
    ///     `pMin` 指向 `GainRawMin`（`<Integer>`，`<Value>1</Value>`，
    ///     `Visibility Invisible`）⇒ 量程 1..32。
    ///     ⚠ 该节点的**步长没有声明**：`<Inc>1</Inc>` 属于 `GainRawMin`
    ///     这个 Integer 节点，**不属于** `GainRaw`；SDK 也没有取
    ///     Floating 步长的接口（只有 `IMV_GetIntFeatureInc`）。
    ///   · **没有 `Gain` 节点**：`IMV_GetDoubleFeatureValue("Gain")` 返回
    ///     **−110**（`IMV_ERROR_PROPERTY_TYPE`），`IMV_GetFeatureType("Gain")`
    ///     的**类型出参为 0**。⚠ 注意 `IMV_GetFeatureType` 的返回类型是
    ///     **`bool`**（`IMVApi.h:775`）而不是错误码：早期探针把它当错误码读并
    ///     据此写下"返回 1"，属**误读**，本节已按上述判据更正。
    /// ⇒ 单位与换算依据**都不明确**，故本字段表示**原生数值**，
    /// 项目不对它做任何单位解释，读回比对也按原生数值语义
    /// （见 `ImvCameraBackend::configureGain`）。**禁止**把 dB 值填进本字段。
    ///
    /// 范围**不由本项目裁定**：上表中的 1..32 是**该机型**的设备事实，
    /// 写进 `data` 层就等于把一台相机的量程冻进通用类型。越界由设备
    /// 自己拒绝（`IMV_INVALID_RANGE`），本项目只拒绝负值（不合理）。
    std::optional<double> gainRaw;

    /// 触发模式（三值，ENG-09 V2.3 §6.1）。
    ///
    /// ⚠ 由 `bool`（true = 硬触发）扩为枚举，yaml 取值
    /// `software|hardware|free_run`。**旧数字值显式拒绝**并给出迁移提示 ——
    /// 让 `1` 静默变成"某一种模式"会把一次配置未迁移伪装成配置正确。
    /// 硬触发失效时上层降级为软触发并置 kErrTriggerDegraded(3001)；
    /// 该降级的**决策点不在本结构体**（见 ICameraBackend::setTriggerMode
    /// 的说明），本字段只承载请求值。
    CameraTriggerMode triggerMode = CameraTriggerMode::FreeRun;
};

}  // namespace data
}  // namespace aircraft
