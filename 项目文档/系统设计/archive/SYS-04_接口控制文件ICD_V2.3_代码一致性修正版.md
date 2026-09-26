# SYS-04_ICD接口控制文件_V2.3（代码一致性修正版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统模块接口控制规范。

本版本针对：

* 三相机固定光机结构；
* CalibrationManager；
* TargetModelManager；
* MeasurementRecord；
* FailureTrace；
* Recorder；
* 真实硬件接入；

进行接口补充。

---

# 1.1 V2.3 修订性质

V2.3 是**代码一致性修正版**。不改变软件架构、不新增接口、不改变代码。

V2.2 与代码基线的一致性经逐条审计（见
`AircraftPoseSystem/V2.1-SYS04不一致逐条对照.md`）：21 个接口声明点中
**12 处与代码不符**，其中 **5 处使用了代码中完全不存在的类型名**。

V2.3 按以下原则修正：

1. **ICD 描述真实存在的接口，不描述预期接口。**
   凡文档声明而代码不存在的实体，一律修正为代码实际形态 ——
   **是文档向代码收敛，不是把代码改向文档**；
2. **不新增接口。** 修正只做替换、改名、删除，不做扩充；
3. **不保留包装类型。** 同一实体在文档与代码中只允许存在一个名字。

⚠ 本次修正是**单向**的。修正后代码侧不产生任何改动；
`AircraftPoseSystem/src/` 与 `SYS-04 V2.2` 的不一致处以本版为准。

---

# 1.2 本版修正清单

| # | 节 | 修正内容 | 裁决 |
|---|---|---|---|
| 1 | 4.1 | 删 `gain` / `valid`（代码无此二字段） | 一致性 |
| 2 | 5.1 | 增 `lastError()`（代码有，文档漏列） | 一致性 |
| 3 | 9.1 | 删 `getCameraToRig()`；`getCameraCalibration` → `getCalibration`；`RigShipTransform` → `Transform` | 裁决 2/3 |
| 4 | 10.1 | `getModel()` → `get(CameraRole)`；`modelType()` → `modelVersion()` | 一致性 |
| 5 | 11.1 | `transform(Transform)` → `transform(CameraRole, const CameraPose&)` | 一致性 |
| 6 | 12.1 | `DetectionResult detect(ImageFrame&)` → `bool detect(const ImageFrame&, DetectionResult&)` | 一致性 |
| 7 | 12.2 | `ScaleEstimate` → `TargetScaleEstimate`；补 `CameraCalibration` / `targetRealSizeM` 入参 | 裁决 5 |
| 8 | 12.3 | `CameraRole select(vector<...>)` → `bool select(const vector<...>&, MeasurementSelectionResult&)` | 一致性 |
| 9 | 13.1 | `process(CalibrationPackage, …)` → 实际 8 方法接口；`CalibrationPackage` → `OpticalRigCalibration` | 裁决 1 |
| 10 | 15.1 | 删 `lastErrorText()` | 已裁决 |
| 11 | 16 | 补 `selectedCamera` / `selectedScore` / `selectedQuality` / `degraded` / `camerasAvailable` / `calibrationId` / `softwareVersion` | 一致性 |
| 12 | 18.1 | 补公开面（22 项） | 一致性 |
| 13 | 20.1 | 增 `PreviewFrame` 为 `ImageFrame` 显示层包装的说明 | 裁决 6 |
| 14 | 21 | `ErrorCode` → 标注为逻辑概念，当前实现为 `int` 段码 | 裁决 4 |
| 15 | 9.1 / 10.1 / 11.1 | 三个虚构接口类名收敛为具体类名（去 `I` 前缀） | 已裁决 |
| 16 | 26 | 新增「待处置项」；§26.2 已于本版内裁决并实施 | — |

**未改动**：6.1 `ICameraBackend`、7.1 `ITriggerController`、8.1 `ITurntableController`、
14.1 `IPipelineObserver`、17 `FailureTrace`、19 `AlignmentController`、23 禁止接口 ——
以上 7 节经审计与代码**逐字段/逐方法吻合**。

---

# 2 接口设计原则

## 2.1 分层原则

系统采用：

```text id="4j7m1x"
UI

↓

Application

↓

Algorithm

Device

Optical

↓

Data

```

---

# 2.2 依赖方向

冻结：

```text id="8f3n5m"
Application

↓

Device

Optical

Algorithm

↓

Data

```

---

禁止：

```text id="k7x2pz"
Algorithm

↓

Camera SDK
```

---

禁止：

```text id="m5q8vw"
UI

↓

Hardware
```

---

禁止：

```text id="p8n4qx"
Recorder

↓

Pipeline内部状态
```

---

# 3 接口分类

系统接口分为：

```text id="x5m7qc"
1. 数据接口

2. 设备接口

3. 标定接口

4. 坐标接口

5. 算法接口

6. 模型接口

7. 记录接口

8. 控制接口

```

---

# 4 Data公共接口

---

# 4.1 ImageFrame

方向：

```text id="n3q8mz"
Device

↓

Preview

Algorithm

Application
```

---

结构：

```cpp id="h6m2qx"
struct ImageFrame
{

cv::Mat image;


uint64_t frameId;


uint64_t timestampNs;


uint64_t deviceTimestampNs;


std::string cameraId;


CameraRole role;


double exposureTime;

};
```

---

⚠ V2.3 修正：删除 `double gain;` 与 `bool valid;` —— 代码
（`src/data/ImageFrame.h`）中无此二字段，全 `src/` 零命中。本版按代码实况收敛。

若后续真实相机接入（011-A1）确实需要携带增益，应在**同时**修改
`ImageFrame` 与 ICD 的同一个提交内加入，不得只在文档侧声明。

---

约束：

## frameId

必须：

单调递增。

---

## timestampNs

必须：

主机单调时间。

禁止：

墙钟。

---

## deviceTimestampNs

SDK支持：

填写设备时间。

SDK不支持：

必须：

```cpp id="r5m9qx"
0
```

禁止：

伪造。

---

# 5 MultiCamera接口

## 5.1 IMultiCameraManager

职责：

管理三相机。

---

接口：

```cpp id="q8n4mz"
class IMultiCameraManager          // 实现类：device::MultiCameraManager
{

public:


virtual bool initializeAll()=0;


virtual bool startAll()=0;


virtual void stopAll()=0;


virtual bool capture(
data::MultiCameraFrame&
frame
)=0;


virtual data::ErrorInfo lastError() const =0;


};
```

---

## 5.2 lastError

V2.3 新增（代码有、V2.2 漏列）。

必须提供的原因：`capture()` 只返回 `bool`，而它失败的原因至少有两类
**处置完全不同**的情形：

```text id="c5n8mq"
可用相机数不足   →  1001，硬件故障，不重试

三路时间戳超差   →  3002，瞬态，重采即可
```

上层若只凭 `false` 自己编一个码，等于把这两类混为一谈。

实现者**应当**填写本字段。未填写（`code == 0`）时上层按兜底码 9004 记录，
**不得**冒充某个具体码 —— 宁可少报一个具体码，也不要凭空造一个不成立的根因。

---

输出：

```text id="j4p8mq"
MultiCameraFrame
```

---

# 6 Camera接口

## 6.1 ICameraBackend

职责：

隔离SDK。

---

接口：

```cpp id="m7q2xz"
class ICameraBackend
{

public:

virtual bool initialize()=0;


virtual bool start()=0;


virtual void stop()=0;


virtual bool grab(
ImageFrame&
frame
)=0;


virtual bool setTriggerMode(
bool enable
)=0;


};
```

---

实现：

```text id="f8m3qw"
VirtualCameraBackend

ImvCameraBackend

```

---

# 7 Trigger接口

## 7.1 ITriggerController

接口：

```cpp id="w4m8py"
class ITriggerController
{

virtual bool initialize()=0;


virtual bool enable()=0;


virtual bool trigger()=0;


virtual void disable()=0;

};
```

---

数据流：

```text id="b6n3qw"
Trigger

↓

Camera Exposure

↓

ImageFrame

```

---

# 8 Turntable接口

## 8.1 ITurntableController

接口：

```cpp id="p9x4mq"
class ITurntableController
{

virtual bool initialize()=0;


virtual bool move(
const TurntableCommand&
)=0;


virtual TurntableState state()=0;


virtual void stop()=0;

};
```

---

输入：

```text id="z5m7qx"
TurntableCommand
```

---

输出：

```text id="n8q2mw"
TurntableState
```

---

# 9 标定接口

## 9.1 CalibrationManager

职责：

管理光机标定（`data::OpticalRigCalibration`）。

---

接口：

```cpp id="t6m8px"
class CalibrationManager
{

public:


virtual bool load(
const std::string& path
)=0;


virtual bool loadDefaults(
int imageWidth,
int imageHeight
)=0;


virtual CameraCalibration getCalibration(
CameraRole role
)=0;


virtual Transform getRigToShip()
=0;


virtual OpticalRigCalibration calibration()
=0;


virtual std::string calibrationId()
=0;


virtual bool loaded()
=0;


};
```

---

## 9.2 V2.3 修正说明

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `CameraRigTransform getCameraToRig(CameraRole)` | **删除** | 代码无此方法。相机→刚体变换内嵌于 `CameraCalibration`：`data::CameraCalibration` 含 `Transform cameraToRig;` 字段，故该量由 `getCalibration(role).cameraToRig` 获得，不需独立访问器 |
| `CameraCalibration getCameraCalibration(CameraRole)` | `getCalibration(CameraRole)` | 与代码一致 |
| `RigShipTransform getRigToShip()` | `Transform getRigToShip()` | `RigShipTransform` 在代码中零命中。**不保留包装类型** |
| — | 增 `loadDefaults` / `calibration` / `loaded` | 代码有、V2.2 漏列 |
| `CameraRigTransform` / `RigShipTransform` | 均为类型名 | 二者在代码中零命中，本版一并消除 |

**相机→刚体变换的取法（替代 `getCameraToRig`）**：

```cpp
data::CameraCalibration calib = calibration.getCalibration(role);

data::Transform cameraToRig = calib.cameraToRig;
```

⚠ `optical::CalibrationManager` 另提供 `lastWarnings()` / `lastErrorText()` 两个
诊断字符串访问器。**V2.3 不将其列入 ICD** —— 与 §15.1 删除 `lastErrorText()` 同一条判据：
失败原因经 `data::ErrorInfo` 上报，不以裸字符串跨层传递。

---

# 10 TargetModel接口

新增。

## 10.1 TargetModelManager

职责：

管理目标模型和特征库。

---

接口：

```cpp id="c4m7px"
class TargetModelManager
{

public:


virtual bool loadModel(
const std::string& path
)=0;


virtual TargetModel get(
CameraRole role
)=0;


virtual bool loaded()
=0;


virtual std::string modelId()
=0;


virtual std::string modelVersion()
=0;


virtual double targetRealSizeM()
=0;


};
```

---

## 10.2 V2.3 修正说明

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `TargetModel getModel()` | `get(CameraRole role)` | 与代码一致。**必须带入参**：三个焦段各有独立特征库（SYS-12 §8），无参接口无法表达"取哪一路" |
| `std::string modelType()` | `std::string modelVersion()` | 代码有 `modelVersion()` 无 `modelType()` |
| — | 增 `loaded` / `targetRealSizeM` | 代码有、V2.2 漏列 |

⚠ **`modelType` 在代码中的真实落点是 `data::MeasurementRecord::modelType` 字段**
（模型来源标记：production / synthetic），**不是** `TargetModelManager` 的方法。
V2.2 在此处的方法声明与记录字段同名，导致两处语义混淆，本版予以分离。

---

输出：

包括：

* modelId；
* modelVersion；
* 3D点；
* 特征库。

---

# 11 坐标转换接口

## 11.1 CoordinateTransformer

职责：

完成：

Camera→Rig→Ship。

---

接口：

```cpp id="m9q3xz"
class CoordinateTransformer
{

public:


virtual Transform cameraToRig(
CameraRole role
)=0;


virtual ShipPoseResult transform(
CameraRole role,
const CameraPose& cameraPose
)=0;


};
```

---

## 11.2 V2.3 修正说明

`transform` 的入参由 `(Transform cameraPose)` 改为 `(CameraRole role, const CameraPose& cameraPose)`：

* **多一个 `role`** —— 坐标链（相机→刚体→舰体）的起点由被测量的那一路相机决定，
  与 `cameraToRig(role)` 是同一参数，不能只凭一个位姿矩阵反推；
* **入参类型是 `CameraPose` 不是 `Transform`** —— `Transform`（`data::Transform`）
  是纯刚体变换矩阵，`CameraPose`（`data::CameraPose`）是带角色信息的相机位姿。

⚠ 实现类 `CoordinateTransformer` 的构造期为 `const CalibrationManager&` 注入，
`rigToShip` **每次向 `CalibrationManager` 取**而非持副本 —— 理由：标定可在运行期重载
（见 §9.1 `load`），持副本会让"重标定后仍用旧外参"变成无报错的错误结果。

---

# 12 Algorithm接口

---

# 12.1 TargetDetector

接口：

```cpp id="v8m4qx"
class TargetDetector
{

public:


virtual bool detect(
const ImageFrame& frame,
DetectionResult& out
)=0;


};
```

---

V2.3 修正：V2.2 写作 `DetectionResult detect(ImageFrame&)`。改为
`bool` 返回 + 出参 —— **未检出以 `false` 表达，而不是 `true` 加 `found=false`**，
编译器强制调用方处理失败分支。

---

# 12.2 TargetScaleEstimator

接口：

```cpp id="x3m7qp"
class TargetScaleEstimator
{

public:


virtual bool estimate(
const DetectionResult& detection,
const CameraCalibration& calibration,
double targetRealSizeM,
TargetScaleEstimate& out
)=0;


};
```

---

V2.3 修正两处：

1. `ScaleEstimate` → **`TargetScaleEstimate`**。`ScaleEstimate` 在代码中零命中
   （仅出现在 `TargetScaleEstimator.h` 的一行注释里作为 `8.md` 的引用），
   实际类型为 `data::TargetScaleEstimate`。**不保留包装类型**；
2. 入参由 1 个补为 4 个。尺度估计需要相机内参（`calibration`）与目标真实尺寸
   （`targetRealSizeM`）才能由像素尺度反推距离 —— V2.2 的单入参形态在几何上不闭合。

---

# 12.3 MeasurementSelector

接口：

```cpp id="k5n8mq"
class MeasurementSelector
{

public:


bool select(
const std::vector<MeasurementCandidate>& candidates,
MeasurementSelectionResult& out
) const;


};
```

---

## 12.3.1 为何不以 CameraRole 为返回值

V2.2 写作 `CameraRole select(std::vector<MeasurementCandidate>)`，
以 `CameraRole::UNKNOWN` 表示"无候选"。本版改为 `bool` + 出参，理由：

```text
ENG-09 §4.1 只冻结了三个角色：CAM25 / CAM50 / CAM100，没有 UNKNOWN。
```

若给 `CameraRole` 加 `UNKNOWN` 哨兵，它会经由**默认构造**悄悄出现在所有含该枚举的
结构体中（`ImageFrame.role`、`MeasurementCandidate.camera` …），
使"这台相机不存在"与"这台相机是 25mm"在内存中不可区分。

返回 `bool` 则编译器强制调用方处理失败分支：
`false` = 无候选可选（`MEASURE_SELECT` 状态按 SYS-08 §7.3 重试，最多 3 次；
用尽后按 §7.7 进入 `FAILED`）。

该偏离**已登记**，见 README §6 第 28 行。本版是将其同步进 ICD。

---

输入：

* 距离；
* 目标像素；
* 清晰度；
* 特征质量。

---

# 13 PosePipeline接口

## 13.1 主接口

**IF-SW-02**：本接口是 **application ↔ algorithm 的唯一调用面**。

```cpp id="h7m3qx"
class IPosePipeline          // 实现类：algorithm::PosePipeline（本接口唯一实现方）
{

public:


virtual ~IPosePipeline() = default;


virtual void setStatisticsObserver(
IPipelineObserver* observer
)=0;


virtual void setCoarseAttitude(
double azimuthDeg,
double elevationDeg,
double distanceM
)=0;


virtual bool detect(
const ImageFrame& frame,
DetectionResult& out
)=0;


virtual bool estimateScale(
const ImageFrame& frame,
const DetectionResult& detection,
TargetScaleEstimate& out
)=0;


virtual bool selectCamera(
const MultiCameraFrame& frame,
const std::vector<CameraRole>& allowed,
MeasurementSelectionResult& out
)=0;


virtual bool selectBestFrame(
const std::vector<ImageFrame>& frames,
int& bestIndex,
ImageQuality& quality
)=0;


virtual bool solvePose(
const MultiCameraFrame& frame,
CameraRole camera,
const CameraCalibration& calib,
ShipPoseResult& out
)=0;


virtual bool validate(
const ShipPoseResult& result,
PoseValidationResult& out
)=0;


};
```

---

## 13.2 V2.3 修正说明

V2.2 §13.1 声明：

```cpp
class PosePipeline
{

public:


bool process(
const MultiCameraFrame& frame,
const CalibrationPackage& calibration,
const TargetModel& model
);


};
```

该声明**两端均不成立**：`process` 在 `src/algorithm/` 零命中；
`CalibrationPackage` 在代码中零命中。本版按代码实况重写为上面的 8 方法接口。

**修正点**：

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `bool process(MultiCameraFrame, CalibrationPackage, TargetModel)` 单方法 | 8 个纯虚方法（`detect` / `estimateScale` / `selectCamera` / `selectBestFrame` / `solvePose` / `validate` + `setStatisticsObserver` / `setCoarseAttitude`） | 算法链是**分阶段可独立调用**的，非单入口。ENG-10 §2.2 的 A/B 两类特征都须进匹配，`MockPoseEstimator` 式的单入口无法承载内点统计 |
| `CalibrationPackage` | **删除该类型名** | 代码零命中。实际标定为 `data::OpticalRigCalibration`（含 `cam25` / `cam50` / `cam100` / `rigToShip`），**构造期注入**而非入参传递 |
| `TargetModel model` 入参 | `TargetModelManager` 构造期注入 | 三个焦段各有独立特征库，整份模型一次性传入无法表达"取哪一路"（同 §10.2） |

**标定与模型的注入方式**（对应 V2.2 那三个入参的去处）：

```text id="r3m9qx"
data::OpticalRigCalibration   →  PosePipeline 构造期注入

algorithm::TargetModelManager →  PosePipeline 构造期注入（裸指针）
```

原因：IF-SW-02 冻结的入参里**没有转台角度、也没有 `rigToShip`**，
而冻结签名承诺产出 `ShipPoseResult.aircraftToShip`（舰体系结果）——
舰体系变换不在任何入参里。三级坐标链（相机→刚体→舰体）由 `PosePipeline`
自行合成，代价是本文件内重复约 30 行 ZYX 分解。
该偏离已登记，见 README §6 第 38 行。

> ⚠ **本节的「IF-SW-02 冻结面」本身仍待处置**：README §6 第 29 行引用
> 「SYS-04 §4.2 冻结的调用面只有 detect / estimateScale / selectCamera /
> selectBestFrame / solvePose / validate」，但 **SYS-04 中不存在 §4.2**。
> 该问题不属本版范围，已独立立项。见 §26。

---

## 13.3 输出

```text id="q8m2px"
ShipPoseResult              ← solvePose 出参

PoseValidationResult        ← validate 出参

MeasurementStatistics       ← 经 IPipelineObserver::onStatistics 推送
```

⚠ `MeasurementStatistics` **不是本接口任何方法的返回值**，而是算法在**产生统计量的
那一刻**经 `IPipelineObserver::onStatistics()` 推送（见 §14）。这样设计是为了避免给
`IPosePipeline` 增加"查询内部状态"的访问器 —— 与 §26 登记的裁决同源。

---

# 14 Pipeline统计接口

## 14.1 IPipelineObserver

用于：

输出算法统计。

---

接口：

```cpp id="r4m8qz"
class IPipelineObserver
{

virtual void onStatistics(
const MeasurementStatistics&
)=0;


};
```

---

数据来源：

```text id="y7m2qx"
FeatureMatcher

↓

IPipelineObserver

↓

MeasurementRecord

```

---

禁止：

Recorder读取：

Pipeline内部变量。

---

# 15 Recorder接口

## 15.1 IRecorderSink

职责：

保存测量结果。

---

接口：

```cpp id="n6m8qx"
class IRecorderSink          // 实现方：infrastructure::Recorder（经 app::RecorderSinkAdapter 桥接）
{

public:


virtual ~IRecorderSink() = default;


virtual bool save(
const MeasurementRecord& record
)=0;


};
```

---

## 15.2 V2.3 修正说明

V2.2 声明 `virtual std::string lastErrorText() = 0;`。该**已删除**，理由三条：

1. **与同类接口形态一致** —— `ICameraBackend`（§6.1）同为设备侧落盘/采集接口，
   亦无 `lastErrorText()`，失败一律经 `data::ErrorInfo` 上报；
2. **`save()` 已返回 `bool`**，调用方需要的"成功/失败"已具备；
3. **失败原因不属本层** —— 落盘失败的具体原因在 infrastructure 层的 `Recorder`
   内部，跨层回传裸字符串等于把 `ErrorInfo`（错误码 + 消息 + 时间戳）的职责
   降级成一个无码可检索的串。本项目按错误码检索 `result.json` / 日志的自动化
   查询会因此失效。

⚠ **入参类型说明**：`save()` 的入参是 `data::MeasurementRecord`
（4 字段值传递的 `MeasurementTask` 不足以产出结果包，见 §16）。
该变更由裁决 C-002 交付，代码与本文档一致。

---

保存：

* 原始图像；
* result.json；
* calibration；
* config snapshot；
* 日志。

---

# 16 MeasurementRecord接口

一次测量结果。

```cpp id="z5q7mx"
struct MeasurementRecord
{

MeasurementTask task;


MultiCameraFrame bestFrame;


MeasurementStatistics statistics;


TurntableState turntable;


CameraRole selectedCamera = CameraRole::CAM25;


double selectedScore = 0.0;


ImageQuality selectedQuality;


FailureTrace failure;


bool degraded = false;


int camerasAvailable = 0;


std::string calibrationId;


std::string modelId;


std::string modelType;


std::string softwareVersion;


};
```

---

## 16.1 V2.3 修正说明

V2.2 只列出 6 项**概念**（`MeasurementTask` / `MultiCameraFrame` / `TurntableState` /
`MeasurementStatistics` / `TargetModel信息` / `FailureTrace`），本版补全为 14 字段实况。

V2.2 的概念项与字段的对应关系全部成立：

```text id="t7m4qx"
MeasurementTask         →  task

MultiCameraFrame        →  bestFrame

TurntableState          →  turntable

MeasurementStatistics   →  statistics

TargetModel信息          →  modelId + modelType

FailureTrace            →  failure
```

本版补入的字段及其承载要求：

| 字段 | 承载 |
|---|---|
| `selectedCamera` / `selectedScore` / `selectedQuality` | ENG-10 §4.1 选择结果三元组 |
| `degraded` / `camerasAvailable` | **SYS-08 §7.5「降级必须在 UI 可见」的承载字段**。V2.2 漏列这两项，会使该要求在 ICD 上找不到落点 |
| `calibrationId` / `modelId` / `modelType` / `softwareVersion` | 结果包的可追溯性 —— 记录必须能回答"这次测量用的是哪份标定、哪个机型库、哪个软件版本" |

⚠ `modelType` 为**模型来源标记**（production / synthetic），非管理器方法名
（见 §10.2）。SYS-02 要求 release 阶段禁止 synthetic 模型进入测量路径，
本字段是该约束在结果包上的唯一判据。

---

# 17 FailureTrace接口

用于：

失败追踪。

结构：

```cpp id="p4m7xz"
struct FailureTrace
{

MeasurementState firstFailedState;


ErrorInfo firstError;


MeasurementState finalFailedState;


ErrorInfo finalError;


std::vector<StateTransition>
history;

};
```

---

# 18 Application接口

## 18.1 MeasurementController

接口（公开面）：

```cpp id="h8m3qy"
class MeasurementController : public IPipelineObserver
{

public:


virtual ~MeasurementController() = default;


/---- 控制 ----/

void startMeasurement();


void startMeasurement(
uint64_t nowNs
);


void stopMeasurement();


bool tick(
uint64_t nowNs
);


/---- 状态 ----/

MeasurementState state() const;


ErrorInfo lastError() const;


bool degraded() const;


ErrorInfo degradationNotice() const;


int availableCameraCount() const;


uint64_t remainingNs(
uint64_t nowNs
) const;


int attempts(
MeasurementState state
) const;


int rollbackCount() const;


/---- 结果 ----/

CameraRole selectedCamera() const;


TargetOffset lastOffset() const;


ImageQuality bestQuality() const;


ShipPoseResult poseResult() const;


PoseValidationResult validationResult() const;


MeasurementTask task() const;


/---- 统计（IPipelineObserver）----/

void onStatistics(
const MeasurementStatistics& statistics
) override;


MeasurementStatistics statistics() const;


bool statisticsReceived() const;


std::vector<std::string> notices() const;


};
```

---

## 18.2 V2.3 修正说明

V2.2 只列 3 项（`startMeasurement` / `stopMeasurement` / `state`），
三者均在代码中存在，差异全在**漏列**。本版补全公开面。

⚠ **`poseResult()` / `validationResult()` 是结果读取，不是内部状态查询。**
它们返回控制器**已收口的结果包内容**（与 `MeasurementRecord` 同源），
不穿透到算法层私有成员 —— 与 §26 登记的被否决形态（给 `IPosePipeline` 增加
`lastMatchResult()` 之类的访问器）性质不同。

`degraded()` / `degradationNotice()` / `availableCameraCount()` / `notices()`
承载 SYS-08 §7.5 的降级可见性要求（与 §16 `degraded` / `camerasAvailable` 同源）。

---

# 19 AlignmentController接口

输入：

```text id="w6m9qx"
TargetOffset
```

---

输出：

```text id="q3m8px"
TurntableCommand
```

---

禁止：

Algorithm直接调用。

---

# 20 Preview接口

## 20.1 PreviewManager

接口：

```cpp id="x7m4qz"
class PreviewManager
{

public:


bool setCamera(
CameraRole role
);


void setMode(
PreviewMode mode
);


virtual bool getFrame(
PreviewFrame& frame
);


};
```

---

## 20.2 PreviewFrame

```cpp id="w4n7qx"
struct PreviewFrame
{

ImageFrame frame;


uint64_t displayTimestamp = 0;


};
```

**PreviewFrame 为 ImageFrame 的显示层包装数据。** 它不是与 `ImageFrame` 并列的
另一种图像类型，而是在图像之外附带一个**显示时间戳**（供预览刷新节奏判断）。

故 `getFrame()` 的"取一帧图像"与"取一个（图像 + 显示时间戳）二元组"**是包含关系，
不是类型冲突**。V2.2 写作 `getFrame(ImageFrame&)`；本版按代码实况写作
`PreviewFrame&`，并在此说明二者关系。

⚠ `getFrame` 的消费者是 **GUI 线程的定时重绘**（单消费者）。它记录"上次已交付的序号"，
**从两个线程调用会让先到者把帧吃掉**。窗口最小化后 Qt 仍派发定时器事件，
按可见性启停由 `showEvent` / `hideEvent` 负责。

---

# 21 错误接口

## ErrorInfo

```cpp id="m5q8xz"
struct ErrorInfo
{

int code = 0;


std::string message;


uint64_t timestampNs = 0;

};
```

---

## 21.1 关于 ErrorCode

**`ErrorCode` 是逻辑概念，当前实现为 `int` 段码。**

```text id="n8q5mx"
逻辑概念：  ErrorCode（一个具名的错误码类型）

当前实现：  int + 段常量

后续演进：  统一枚举化
```

V2.3 在此如实标注这一状态，**不将 `ErrorCode` 写作已存在的类型**。
代码中该类型零命中；错误码由 `int` 常量承载，按 ENG-09 §5.27 分段：

```text id="p3m7qx"
1000 ~ 1999   相机

2000 ~ 2999   转台

3000 ~ 3999   触发 / 同步

4000 ~ 4999   标定

5000 ~ 5999   模型与特征库

6000 ~ 6999   算法

9000 ~ 9999   系统级
```

⚠ `code == 0` 的含义是**"未设置"**，不是"成功"，也不得作为失败码使用。

**`ErrorCode` 枚举化属设计层改动，不属本次文档一致性修正范围** ——
本版只消除"文档把一个不存在的类型当作已冻结事实"这一处不一致。
枚举化时应作为独立裁决处理，并同步 `errorCodeName()` 的 `switch`。

---

## 21.2 错误必须携带

* **错误码** —— `code`，须落在上表段内；
* **状态** —— **由调用方所在的状态机上下文承载，不在本结构体内**。
  `FailureTrace`（§17）的 `firstFailedState` / `finalFailedState` 即该信息；
* **原因** —— `message`，须能回答"为什么"，不得只复述错误码名。

> V2.2 同节写作「错误码；状态；原因」三项而结构体只有两字段。本版补充说明第三项
> （状态）的实际承载位置 —— 它并非缺失，而是刻意不由 `ErrorInfo` 承担：
> 同一个错误码在不同状态下处置不同（SYS-08 §7.3 的重试策略按状态分支），
> 把状态塞进错误结构体会使两者的唯一性互相污染。

---

# 22 接口依赖关系

冻结：

```text id="w9m2qx"
Data

↑

Device

Optical

Algorithm

Application

UI

```

---

# 23 禁止接口

禁止：

## Algorithm控制设备

错误：

```text id="v4m7qx"
PnP

↓

Turntable
```

---

禁止：

## UI访问SDK

错误：

```text id="k8m3py"
MainWindow

↓

ImvSDK
```

---

禁止：

## Recorder访问私有状态

错误：

```text id="q6m8xz"
Recorder

↓

PosePipeline private
```

---

# 24 接口版本管理

接口修改必须：

* 更新ICD；
* 更新测试；
* 更新版本号。

当前：

```text id="m7q2xz"
ICD_V2.3
```

---

# 25 设计冻结总结

| 接口                 | 状态   | V2.3 变动 |
| ------------------ | ---- | ------- |
| Camera接口           | 冻结   | 未改（§6.1 逐方法吻合） |
| MultiCamera接口      | 冻结   | 补 `lastError()`（§5.2） |
| Trigger接口          | 冻结   | 未改 |
| Turntable接口        | 冻结   | 未改 |
| Calibration接口      | 冻结   | 类名 `ICalibrationManager` → `CalibrationManager`；删 `getCameraToRig`、改名、去包装类型（§9.2 / §26.2） |
| Coordinate接口       | 冻结   | 类名 `ICoordinateTransformer` → `CoordinateTransformer`；`transform` 补 `role` 入参（§11.2 / §26.2） |
| Algorithm接口        | 冻结   | 三个签名改为 `bool` + 出参（§12）；`ScaleEstimate` → `TargetScaleEstimate` |
| TargetModel接口      | 新增冻结 | 类名 `ITargetModelManager` → `TargetModelManager`；`getModel()` → `get(role)`、`modelType()` → `modelVersion()`（§10.2 / §26.2） |
| PosePipeline接口      | 冻结   | `process()` → 8 方法实况（§13.2） |
| Recorder接口         | 冻结   | 删 `lastErrorText()`（§15.2） |
| PipelineObserver接口 | 冻结   | 未改（逐字吻合） |
| MeasurementRecord   | 冻结   | 6 概念 → 14 字段（§16.1） |
| FailureTrace        | 冻结   | 未改（5 字段逐字段吻合） |
| Preview接口          | 冻结   | `getFrame` 出参 → `PreviewFrame`（§20） |
| 错误接口              | 冻结   | `ErrorCode` 标注为逻辑概念（§21） |

---

# 26 待处置项

本次代码一致性审计提出两项：

* **§26.1 IF-SW-02 冻结面缺失** —— 不属本版修正，独立立项；
* **§26.2 接口类的实现形态** —— 已在本版内裁决并实施。

## 26.1 IF-SW-02 冻结面缺失

`IF-SW-02` 与「SYS-04 §4.2」被代码、`README.md` §6、`V2.1-C01` 共 11 处
作为"冻结的算法层调用面"引用，但：

```text id="v2q8mx"
SYS-04 中不存在 §4.2

SYS-04 中不存在任何 IF-SW-xx 编号

IF-SW-02 在 项目文档/ 全目录零命中
```

READM 引用其冻结面为 `detect / estimateScale / selectCamera / selectBestFrame /
solvePose / validate` 六个方法，而这**六个方法名在 SYS-04 全文一次都未出现**
（V2.2 §13.1 写的是 `process`；V2.3 §13.1 已按代码实况重写为 8 方法）。

**性质**：接口能力缺失，不是接口文档错误。

**处置**：独立立项 —— `IF-SW-02_PipelineObserver接口一致性修正`。
需同时决定：冻结面是 6 方法还是 8 方法（差 `setStatisticsObserver` /
`setCoarseAttitude` 两个由后续裁决加入的合法扩展），以及 IF-SW 编号体系是否重建。

## 26.2 接口类的实现形态（✅ 已裁决，已实施）

§9.1 / §10.1 / §11.1 原声明的 `ICalibrationManager` / `ITargetModelManager` /
`ICoordinateTransformer` 三个抽象接口类在代码中**零命中** ——
代码提供的是具体类。即：**方法级一致，类型级为虚构**。

**裁决（2026-09-24）**：**不保留 `I` 前缀，按代码收敛为具体类名。**

```text id="y6m2qx"
ICalibrationManager     →  CalibrationManager

ITargetModelManager     →  TargetModelManager

ICoordinateTransformer  →  CoordinateTransformer
```

理由：C++ 的接口抽象**不必须** `I` 前缀。项目已进入工程实现阶段，ICD 要服务开发、
集成与测试；正文写 `ICalibrationManager` 而代码是 `CalibrationManager`，
会持续产生误解 —— 而这正是本版要消除的那类不一致。

⚠ **本节的收敛只针对这三个类。** 其余六个 `I` 前缀接口
（`ICameraBackend` / `IMultiCameraManager` / `ITriggerController` /
`ITurntableController` / `IPipelineObserver` / `IPosePipeline`）
**在代码中真实存在，一律保留原名**，不改。判据是"代码里有没有这个名字"，
不是"有没有 `I` 前缀"。

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统接口控制基线。

---

# 27 修订记录

| 版本 | 日期 | 修订内容 |
|---|---|---|
| V2.2 | — | 多相机标定融合与闭环测量版。新增 CalibrationManager / TargetModelManager / MeasurementRecord / FailureTrace / Recorder / 真实硬件接入的接口补充 |
| **V2.3** | **2026-09-24** | **代码一致性修正版**。按代码基线逐条修正 12 处接口声明不符，消除 5 个虚构类型名（`CalibrationPackage` / `CameraRigTransform` / `RigShipTransform` / `ErrorCode` / `ScaleEstimate`）与 3 个虚构接口类名（`ICalibrationManager` / `ITargetModelManager` / `ICoordinateTransformer`，收敛为具体类名）。**不改变软件架构、不新增接口、不改变代码**。审计依据：`AircraftPoseSystem/V2.1-SYS04不一致逐条对照.md` |

---

## 27.1 版本关系

| 文件 | 状态 |
|---|---|
| `SYS-04_接口控制文件ICD_V2.3_代码一致性修正版.md` | **唯一有效版本** |
| `archive/SYS-04_接口控制文件ICD_V2.2_多相机闭环测量版.md` | 已归档，仅作历史留存，**不再作为接口依据** |

⚠ 项目当前**无版本管理仓库**，故 V2.2 以移动至 `archive/` 的方式留存而非删除 ——
保留审计链。引用 SYS-04 时一律指向 V2.3。
