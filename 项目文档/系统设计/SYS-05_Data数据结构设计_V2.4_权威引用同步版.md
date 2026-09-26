# SYS-05_Data数据结构设计_V2.4（权威引用同步版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 软件系统的数据结构设计。

本版本基于：

* 多相机固定光机架构；
* PreviewManager；
* MeasurementRecord；
* Calibration Package；
* 三坐标体系；
* 闭环姿态测量流程；

进行升级。

---

# 1.1 V2.4 修订性质

V2.4 只做**一件事**：**再次同步当前的权威引用**。V2.3 的正文（`ImageFrame` 的三个新字段、
九字段未改的说明、§5.1 注等）**一字未动**，改的只是 §1.1 与 §5.1 注里"以哪一版为准"的
**现时声明**。

⚠ **类型与字段的唯一权威是 `ENG-09_类型与命名冻结表_V2.5_清理责任与异常安全版`**，
本文件是**设计层叙述**：字段名、取值、组合合法性、错误码一律以 ENG-09 V2.5 为准；
两者若不一致，**以 ENG-09 V2.5 为准**（见 ENG-09 §1.1「权威性规则」）。
∴ 本版**不重复** ENG-09 的契约条文细节，只登记"本文件涉及的部分变了什么"。

> ⚠ **为什么必须同批改这一句（留痕）**：本句是**当前裁决规则**，不是历史修订记录 ——
> V2.3 写的是"以 ENG-09 **V2.4** 为准"，而 ENG-09 已升 **V2.5**（V2.4 已归档）。
> 新旧语义冲突时，旧句会把读者**指回已归档的版本**，故同批改为 V2.5；
> 修订记录里 V2.2 / V2.3 两行的版本指向是**当时的事实**，**保留不动**。
> ⚠ 本句之所以**又一次**需要改：ENG-09 V2.5 改的是 §5.29 的**聚合规则**
> （"判定类别与实际调用历史分别记录"），而本文件 §5.1 的 `ImageFrame` 说明
> **正是**引用那一节的地方 —— 指针不更新，读者会照旧看到被删掉的旧结论。

⚠ **本版未改动任何既有字段的语义**：`image` / `frameId` / `timestampNs` /
`deviceTimestampNs` / `cameraId` / `role` / `exposureTime` / `gain` / `valid`
九个字段一字未改（其中 `gain` / `valid` 的删除属 SYS-04 V2.3 的 ICD 侧修正，
**不涉及本文件的 Data 层定义**）。

**V2.3 引入的改动清单（保留在册，本版未动）**：V2.3 只把 §1.1 与 §5.1 注里的
"以 ENG-09 **V2.3** 为准"改为 **V2.4**，正文与既有字段语义一字未动。

---

# 2 数据层设计原则

## 2.1 职责

Data层只负责：

* 数据定义；
* 数据传输；
* 数据序列化结构。

不负责：

* 设备控制；
* 算法计算；
* UI逻辑；
* 文件读写。

---

## 2.2 依赖原则

冻结：

```text id="1j6q8h"
Application

↓

Algorithm / Device / Optical

↓

Data

```

Data为公共基础层。

禁止：

```text id="x8d5pm"
Data

↓

Algorithm

```

禁止：

```text id="y0s3ae"
Data

↓

Device SDK
```

---

# 3 数据结构总体分类

系统数据分为：

```text id="9n5q6f"
1. 图像数据

2. 标定数据

3. 坐标数据

4. 测量数据

5. 姿态数据

6. 控制数据

7. 结果记录数据

```

---

# 4 相机基础数据

## 4.1 CameraRole

路径：

```text
src/data/CameraRole.h
```

定义：

```cpp
enum class CameraRole
{

    CAM25,

    CAM50,

    CAM100,

    UNKNOWN

};
```

---

用途：

标识：

* 镜头；
* 测量通道；
* 标定对象。

---

# 5 图像数据结构

## 5.1 ImageFrame

表示单相机采集帧。

```cpp
struct ImageFrame
{

cv::Mat image;


uint64_t frameId;


uint64_t timestampNs;


uint64_t deviceTimestampNs;


std::string cameraId;


CameraRole role;


double exposureTime;


double gain;


bool valid;


// ---- V2.2 新增（011-A1 数据契约批，裁决 C-015）----

RawImagePayload raw;

PixelFormat     captureFormat;

RawDataPolicy   rawPolicy;

};
```

> ⚠ **上表中 `gain` / `valid` 两字段的归属**：SYS-04 V2.3 曾以"代码无此二字段"为由
> 从 **ICD 的 `ImageFrame`** 中删除它们。本节是 **Data 层设计定义**，其字段来源是
> 裁决 C-01 的**并集冻结**（并集含 `exposureTime`）；两处描述的**权威以 ENG-09 V2.5 §5.5 为准**
> （§5.5「`ImageFrame`」在 V2.5 仍然存在，节号未变 —— V2.5 改的是 §5.29 的聚合规则，
> **不涉及 §5.5**）。
> 本节照录并集以保持"设计层定义"完整，**不据此声称代码中存在这两个字段**。

---

## 字段说明

### timestampNs

定义：

主机单调时钟。

用途：

* 同步判断；
* 帧排序。

禁止：

使用墙钟时间。

---

### deviceTimestampNs

定义：

SDK提供设备时间。

用途：

* 时钟偏移诊断。

若SDK不提供：

必须：

```cpp
deviceTimestampNs=0
```

禁止：

使用主机时间填充。

---

### raw

定义：

相机交付的**原始字节**及其全部解释信息（自有副本，见 `RawImagePayload`）。

用途：

* 保留未经改动的原始载荷；
* 记录该载荷自身的解释依据（格式、有效位、打包、字节序）。

禁止：

* 把 `image` 当作裸缓冲的**解码依据** —— 解码依据**只有** `raw` 自身的字段；
* 让 `raw.bytes` 悬挂在 SDK 缓冲区上（必须在归还 SDK 缓冲前完成复制）。

---

### captureFormat

定义：

本帧的**采集格式**（后端实际收到并解释的格式）。

用途：

* 与 `raw.format` 的一致性断言（不一致即**失败**，不是静默转换）。

⚠ **独立于 `raw` 是否为空而存在**。

---

### rawPolicy

定义：

原始载荷的**必要性**策略（由 `captureFormat` 决定）。取值 `RawOptional` / `RawRequired`。

用途：

* Recorder 据此与 `captureFormat` 一起决定"`raw` 为空"时是照常保存还是报契约错误。

⚠ **独立于 `raw` 是否为空而存在**。

**为什么 `captureFormat` / `rawPolicy` 必须独立于 `raw`**：
它们回答的是"**这一帧本来应该带什么**"，不是"**它实际带了什么**"。
若从 `raw` 反推（`raw` 空就当 8 位处理），则"真实 12 位帧丢了载荷"与
"虚拟 8U 帧本来就没有载荷"在数据上**完全同形** —— 前者必须**报错**，后者**照常保存**。

---

# 6 多相机同步数据

## 6.1 MultiCameraFrame

表示一次同步采集结果。

```cpp
struct MultiCameraFrame
{

ImageFrame cam25;


ImageFrame cam50;


ImageFrame cam100;


uint64_t triggerTimestamp;


uint64_t exposureIndex;


};
```

---

## exposureIndex

表示：

一次曝光事件编号。

用于：

三相机数据关联。

---

# 7 标定数据结构

## 7.1 CameraCalibration

表示单相机标定结果。

```cpp
struct CameraCalibration
{

cv::Mat cameraMatrix;


cv::Mat distortion;


int imageWidth;


int imageHeight;


std::string calibrationId;


};
```

---

包含：

* 内参；
* 畸变；
* 图像尺寸；
* 标定版本。

---

# 7.2 CameraRigTransform

表示：

Camera→Rig。

```cpp
struct CameraRigTransform
{

Transform transform;


CameraRole role;


std::string calibrationId;

};
```

---

# 7.3 RigShipTransform

表示：

Rig→Ship。

```cpp
struct RigShipTransform
{

Transform transform;


std::string calibrationId;

};
```

---

# 8 坐标转换数据

## 8.1 Transform

统一空间转换。

```cpp
struct Transform
{

cv::Mat rotation;


cv::Vec3d translation;


};
```

---

表示：

$$
T=[R|t]
$$

---

# 9 目标模型数据

## 9.1 TargetModel

表示目标三维模型。

```cpp
struct TargetModel
{

std::string modelId;


std::string modelType;


double realSizeM;


};
```

---

## modelType

示例：

```text
production

synthetic
```

---

release版本：

禁止加载：

```text
synthetic
```

模型。

---

# 10 特征统计数据

## 10.1 MeasurementStatistics

用于：

结果追溯。

```cpp
struct MeasurementStatistics
{

int featureCount;


int matchCount;


double matchRatio;


int cadCount;


int textureCount;


double spreadPx;

};
```

---

说明：

该结构只保存事实统计。

不包含：

* matcher逻辑；
* 算法状态。

---

# 11 图像质量数据

## 11.1 ImageQuality

```cpp
struct ImageQuality
{

double sharpness;


int featureCount;


int matchCount;


double matchRatio;

};
```

---

用途：

MeasurementSelector评分。

---

# 12 测量候选数据

## 12.1 MeasurementCandidate

```cpp
struct MeasurementCandidate
{

CameraRole camera;


double distanceEstimate;


ImageQuality quality;


double score;

};
```

---

用于：

镜头选择。

---

# 13 姿态结果数据

## 13.1 ShipPoseResult

```cpp
struct ShipPoseResult
{

bool success;


double yaw;


double pitch;


double roll;


Transform shipToAircraft;


double reprojectionError;


double confidence;

};
```

---

说明：

在线验证：

只判断：

* 几何一致性；
* 结果可信度。

不包含：

真实误差。

---

# 14 验证结果数据

## 14.1 PoseValidationResult

```cpp
struct PoseValidationResult
{

bool valid;


double reprojectionError;


double inlierRatio;


double confidence;


};
```

---

不包含：

```text
yawErrorArcmin
```

原因：

真实精度需要外部真值。

---

# 15 转台数据

## 15.1 TurntableCommand

```cpp
struct TurntableCommand
{

double azimuth;


double elevation;

};
```

---

## 15.2 TurntableState

```cpp
struct TurntableState
{

double azimuth;


double elevation;


bool moving;


bool error;

};
```

---

# 16 目标偏差数据

## 16.1 TargetOffset

```cpp
struct TargetOffset
{

double pixelX;


double pixelY;


bool centered;

};
```

---

用途：

视觉闭环控制。

---

# 17 错误数据

## 17.1 ErrorCode

```cpp
enum class ErrorCode
{

NONE,


CAMERA_ERROR,


TRIGGER_ERROR,


TURNTABLE_ERROR,


ALGORITHM_ERROR,


CONFIG_ERROR

};
```

---

## 17.2 ErrorInfo

```cpp
struct ErrorInfo
{

ErrorCode code;


std::string message;

};
```

---

# 18 失败追踪数据

## 18.1 FailureTrace

用于：

失败任务追溯。

```cpp
struct FailureTrace
{

MeasurementState firstFailedState;


ErrorInfo firstError;


MeasurementState finalFailedState;


ErrorInfo finalError;


std::vector<StateTransition> history;

};
```

---

包含：

* 首次失败；
* 最终失败；
* 状态轨迹。

---

# 19 测量记录数据

## 19.1 MeasurementRecord

用于：

最终结果包。

```cpp
struct MeasurementRecord
{

MeasurementTask task;


MultiCameraFrame frames;


TurntableState turntable;


CameraRole selectedCamera;


double selectedScore;


ImageQuality selectedQuality;


MeasurementStatistics statistics;


std::string modelType;


bool degraded;


int camerasAvailable;


FailureTrace failureTrace;

};
```

---

职责：

保存一次完整测量事实。

---

# 20 精度验收数据

## 20.1 AccuracyReport

用于离线验收。

```cpp
struct AccuracyReport
{

bool hasReference;


double referenceYaw;


double measuredYaw;


double yawErrorArcmin;


bool qualified;

};
```

---

说明：

仅用于：

* 标定实验；
* 精度验收。

不参与在线测量。

---

# 21 标定包数据

## CalibrationPackage

包含：

```text
CalibrationPackage

├── CameraCalibration

├── CameraRigTransform

├── RigShipTransform

└── calibrationId

```

---

# 22 数据版本管理

所有关键数据绑定：

```text
version

+

calibrationId

+

modelId

+

softwareVersion
```

---

# 23 数据流关系

完整数据链：

```text
Camera

↓

ImageFrame

↓

MultiCameraFrame

↓

MeasurementSelector

↓

PosePipeline

↓

ShipPoseResult

↓

MeasurementRecord

↓

Recorder

```

---

# 24 数据禁止事项

禁止：

## Data包含算法状态

错误：

```cpp
ImageFrame.match()
```

---

禁止：

## Data调用设备

错误：

```cpp
ImageFrame.capture()
```

---

禁止：

## Data依赖SDK

错误：

```cpp
ImvImageBuffer
```

---

# 25 设计冻结总结

| 类别   | 状态 |
| ---- | -- |
| 图像数据 | 冻结 |
| 同步数据 | 冻结 |
| 标定数据 | 冻结 |
| 坐标数据 | 冻结 |
| 姿态数据 | 冻结 |
| 统计数据 | 冻结 |
| 结果记录 | 冻结 |
| 精度报告 | 冻结 |

---

本文档作为 AircraftPoseSystem V2.4 多相机闭环测量系统 Data层设计基线。

---

# 26 修订记录

| 版本 | 日期 | 修订性质 | 备注 |
|---|---|---|---|
| V2.1 | — | 标定融合与闭环测量版 | 旧版已归档至 `archive/SYS-05_Data数据结构设计_V2.1_多相机闭环测量版.md` |
| **V2.2** | 2026-09-26 | **A1 数据契约版** | §5.1 `ImageFrame` 增 `raw` / `captureFormat` / `rawPolicy` 三字段；新增 §1.1 修订性质与三字段说明。类型权威为 ENG-09 V2.3，本版不重复其契约条文 |
| **V2.3** | 2026-09-26 | **权威引用同步版** | **只改当前权威声明**：§1.1 与 §5.1 注里"以 ENG-09 V2.3 为准"⇒ **V2.4**（ENG-09 已升版、V2.3 已入 `archive/`，旧句会把读者指回归档版本）。⚠ **正文与所有既有字段语义一字未动**；修订记录里 V2.2 行的"类型权威为 ENG-09 V2.3"作为**当时的事实保留**。依据＝**C-016**（011-A1 九项缺口定向修复批）的文档一致性要求 |
| **V2.4** | 2026-09-26 | **权威引用同步版** | **只改当前权威声明**：§1.1 与 §5.1 注里"以 ENG-09 V2.4 为准"⇒ **V2.5**（§1.1 三处指针 ＋ §5.5 一处，实测 **4 处**）。ENG-09 已升 **V2.5**（改的是 §5.29 的**聚合规则**：判定类别与实际调用历史分别记录），V2.4 已入 `archive/` ⇒ 旧句会把读者指回**已归档**版本，且会让读者照旧看到 §5.1 所引那条**已被删除**的旧结论。⚠ **键表、字段与所有正文一字未动**；修订记录里 V2.2／V2.3 两行的版本指向作为**当时的事实保留**。依据＝**C-01 v1.9**（011-A1 显示职责与清理确认批）的文档一致性要求 |

**V2.4 未改动**：§2~§25 的全部内容（含 V2.2 批新增的 §1.1 与 §5.1 三字段）——
包括 §22 数据版本管理、§23 数据流关系、§24 数据禁止事项、§25 设计冻结总结。
