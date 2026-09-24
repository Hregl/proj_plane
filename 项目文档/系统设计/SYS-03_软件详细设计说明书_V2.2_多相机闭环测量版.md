# SYS-03_软件详细设计说明书_V2.2（三相机闭环测量与真实硬件接入版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 软件详细设计。

本版本针对：

* 三相机固定光机系统；
* 真实A7A20MU201接入；
* CalibrationManager；
* TargetModelManager；
* MeasurementRecord；
* Recorder；
* FailureTrace；
* 闭环测量流程；

进行详细设计补充。

---

# 2 软件总体分层

系统采用分层架构：

```text
UI Layer

↓

Application Layer

↓

Algorithm Layer

Device Layer

Optical Layer

↓

Data Layer

```

---

# 3 工程模块结构

```text
AircraftPoseSystem/

├── data/

├── device/

├── optical/

├── preview/

├── algorithm/

├── application/

├── infrastructure/

├── ui/

└── app/

```

---

# 4 Data模块详细设计

## 4.1 模块职责

Data模块负责：

* 公共数据结构；
* 模块间数据传递；
* 数据序列化定义。

---

禁止：

* 设备逻辑；
* 算法逻辑；
* UI逻辑。

---

# 5 核心数据结构

---

# 5.1 ImageFrame

用途：

表示单相机采集图像。

包含：

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

};
```

---

要求：

## frameId

必须单调递增。

---

## timestampNs

必须使用：

主机单调时间。

---

## deviceTimestampNs

SDK支持：

填写。

SDK不支持：

必须为：

```text
0
```

---

# 5.2 MultiCameraFrame

表示一次多相机采集。

包含：

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

用途：

* 同步验证；
* 测量输入。

---

# 5.3 CalibrationPackage

表示完整标定数据。

包含：

```text
Camera Intrinsic

Camera-Rig Transform

Rig-Ship Transform

Calibration ID
```

---

来源：

SYS-16。

---

# 5.4 MeasurementRecord

表示一次完整测量记录。

包含：

```text
MeasurementTask

MultiCameraFrame

TurntableState

SelectedCamera

MeasurementStatistics

TargetModel信息

FailureTrace

```

---

用途：

* 保存；
* 复现；
* 分析。

---

# 6 Device模块详细设计

---

# 6.1 Camera模块

结构：

```text
ICameraBackend

↓

ImvCameraBackend

VirtualCameraBackend

```

---

# 6.2 ImvCameraBackend

真实设备：

```text
A7A20MU201

↓

ImvSDK

↓

ImageFrame
```

---

职责：

* SDK初始化；
* 相机打开；
* 参数设置；
* 图像采集。

---

# 6.3 当前开发阶段

支持：

```text
CAM25

真实相机


CAM50

VirtualCamera


CAM100

VirtualCamera

```

---

最终：

```text
CAM25

CAM50

CAM100

```

均可真实接入。

---

# 7 Optical模块详细设计

---

# 7.1 OpticalRig

职责：

维护：

```text
CAM25→Rig

CAM50→Rig

CAM100→Rig

```

---

# 7.2 CalibrationManager

职责：

加载：

* 内参；
* 畸变；
* 外参；
* Calibration ID。

---

流程：

```text
Calibration Package

↓

CalibrationManager

↓

PosePipeline

```

---

# 7.3 CoordinateTransformer

完成：

```text
Camera

↓

Rig

↓

Ship

```

---

公式：

$$
T_{Aircraft}^{Ship}
=
T_{Rig}^{Ship}
\times
T_{Camera}^{Rig}
\times
T_{Aircraft}^{Camera}
$$

---

# 8 Preview模块详细设计

---

# 8.1 PreviewManager

职责：

实时显示管理。

支持：

* CAM25默认显示；
* 手动切换；
* 自动选择。

---

# 8.2 Preview数据流

```text
Camera

↓

ImageFrame

↓

PreviewQueue

↓

PreviewManager

↓

Qt

```

---

# 8.3 PreviewQueue

策略：

```text
丢旧保新
```

目的：

降低显示延迟。

---

# 9 Algorithm模块详细设计

---

# 9.1 PosePipeline

总体流程：

```text
Detection

↓

Scale

↓

MeasurementSelector

↓

Feature

↓

Matching

↓

PnP

↓

Validation

```

---

# 9.2 TargetDetector

输入：

ImageFrame。

输出：

DetectionResult。

---

# 9.3 TargetScaleEstimator

输入：

* DetectionResult；
* TargetModel。

输出：

* distanceEstimate；
* targetPixelSize。

---

# 9.4 MeasurementSelector

输入：

```text
CAM25

CAM50

CAM100

```

评价：

* 像素覆盖；
* 特征数量；
* 清晰度；
* 预测姿态误差。

输出：

selectedCamera。

---

# 9.5 TargetModelManager

职责：

管理：

* CAD模型；
* 三维点；
* 特征库。

目录：

```text
models/

aircraft/

├── model.yaml

├── points3d.yaml

├── feature25.bin

├── feature50.bin

└── feature100.bin

```

---

# 9.6 FeatureExtractor

支持：

## 自然纹理

* SIFT。

## CAD结构

* CAD Structure Locator。

---

# 9.7 FeatureMatcher

建立：

```text
2D Image Points

↓

3D Model Points

```

---

输出：

MatchResult。

---

# 9.8 PoseEstimator

输入：

* 2D点；
* 3D点；
* CameraCalibration。

输出：

Aircraft→Camera。

---

# 9.9 PoseValidator

验证：

* 重投影误差；
* 内点比例；
* confidence；
* 非有限值。

---

不判断：

真实Yaw误差。

---

# 10 Application模块详细设计

---

# 10.1 MeasurementController

负责：

* 测量任务；
* 状态推进；
* 模块调度。

---

# 10.2 状态流程

```text
IDLE

↓

SEARCH

↓

TARGET_FOUND

↓

ALIGN

↓

STABILIZE

↓

MEASURE_SELECT

↓

CAPTURE

↓

POSE_SOLVE

↓

VALIDATE

↓

SAVE

↓

COMPLETE

```

---

失败：

进入：

FAILED。

---

# 11 Recorder模块设计

---

# 11.1 Recorder职责

保存：

```text
MeasurementRecord

↓

measurement_xxx/
```

---

内容：

```text
cam25.raw

cam50.raw

cam100.raw

result.json

config_snapshot/

calibration/

log.txt

```

---

# 11.2 Recorder禁止行为

禁止：

直接读取：

* Pipeline私有变量；
* Algorithm内部状态。

---

只能接收：

MeasurementRecord。

---

# 12 FailureTrace设计

保存：

* 首次失败状态；
* 首次错误；
* 最终错误；
* 状态迁移历史。

---

来源：

StateMachine统一维护。

---

# 13 系统启动流程

```text
main.cpp

↓

ConfigManager

↓

CalibrationManager

↓

TargetModelManager

↓

Device初始化

↓

Preview启动

↓

MeasurementController启动

↓

MainWindow显示

```

---

# 14 多线程关系

线程：

```text
CameraWorker

TriggerWorker

SynchronizerWorker

PreviewWorker

AlgorithmWorker

TurntableWorker

RecorderWorker

```

---

通信：

* Queue；
* Signal/Slot；
* shared_ptr。

---

# 15 实时性设计

目标：

```text
Feature开始

↓

Yaw输出

≤100ms
```

---

优化：

* 单通道算法；
* ROI；
* 异步保存；
* Preview隔离。

---

# 16 配置管理

配置：

```text
config/

├── camera.yaml

├── optical_rig.yaml

├── measurement.yaml

├── validation.yaml

├── trigger.yaml

└── turntable.yaml

```

---

# 17 错误处理

统一：

```text
ErrorInfo

+

FailureTrace
```

---

覆盖：

* 相机异常；
* 标定异常；
* 模型异常；
* 算法异常；
* 保存异常。

---

# 18 测试要求

覆盖：

## 单元测试

* Data；
* StateMachine。

---

## 模块测试

* Device；
* Optical；
* Algorithm。

---

## 集成测试

* Preview；
* Recorder；
* Measurement流程。

---

## 硬件测试

* A7A20MU201；
* Trigger；
* Turntable。

---

# 19 当前开发阶段

当前：

```text
Framework Alpha

↓

Hardware Integration

```

目标：

完成：

```text
A7A20MU201

↓

ImvSDK

↓

ImageFrame

↓

Preview

↓

Recorder
```

---

# 20 设计冻结总结

| 模块           | 状态 |
| ------------ | -- |
| Data         | 冻结 |
| Device       | 冻结 |
| Optical      | 冻结 |
| Preview      | 冻结 |
| Algorithm    | 冻结 |
| Application  | 冻结 |
| Recorder     | 冻结 |
| FailureTrace | 冻结 |
| 真实硬件接入路线     | 冻结 |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统软件详细设计基线。
