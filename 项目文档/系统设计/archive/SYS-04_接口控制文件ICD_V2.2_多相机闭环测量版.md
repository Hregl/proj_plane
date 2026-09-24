# SYS-04_ICD接口控制文件_V2.2（多相机标定融合与闭环测量版）

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


double gain;


bool valid;

};
```

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
class IMultiCameraManager
{

public:


virtual bool initializeAll()=0;


virtual bool startAll()=0;


virtual void stopAll()=0;


virtual bool capture(
data::MultiCameraFrame&
frame
)=0;


};
```

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

## 9.1 ICalibrationManager

职责：

管理Calibration Package。

---

接口：

```cpp id="t6m8px"
class ICalibrationManager
{

public:


virtual bool load(
const std::string& path
)=0;


virtual CameraCalibration getCameraCalibration(
CameraRole role
)=0;


virtual CameraRigTransform getCameraToRig(
CameraRole role
)=0;


virtual RigShipTransform getRigToShip()
=0;


virtual std::string calibrationId()
=0;


};
```

---

# 10 TargetModel接口

新增。

## 10.1 ITargetModelManager

职责：

管理目标模型和特征库。

---

接口：

```cpp id="c4m7px"
class ITargetModelManager
{

public:


virtual bool loadModel(
const std::string& path
)=0;


virtual TargetModel getModel()
=0;


virtual std::string modelId()
=0;


virtual std::string modelType()
=0;


};
```

---

输出：

包括：

* modelId；
* modelType；
* 3D点；
* 特征库。

---

# 11 坐标转换接口

## 11.1 ICoordinateTransformer

职责：

完成：

Camera→Rig→Ship。

---

接口：

```cpp id="m9q3xz"
class ICoordinateTransformer
{

public:


virtual Transform cameraToRig(
CameraRole role
)=0;


virtual ShipPoseResult transform(
Transform cameraPose
)=0;


};
```

---

# 12 Algorithm接口

---

# 12.1 TargetDetector

接口：

```cpp id="v8m4qx"
DetectionResult detect(
ImageFrame&
);
```

---

# 12.2 TargetScaleEstimator

接口：

```cpp id="x3m7qp"
ScaleEstimate estimate(
DetectionResult&
);
```

---

# 12.3 MeasurementSelector

接口：

```cpp id="k5n8mq"
CameraRole select(
std::vector<MeasurementCandidate>
);
```

---

输入：

* 距离；
* 目标像素；
* 清晰度；
* 特征质量。

---

# 13 PosePipeline接口

## 13.1 主接口

```cpp id="h7m3qx"
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

---

输出：

```text id="q8m2px"
ShipPoseResult

MeasurementStatistics

PoseValidationResult

```

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
class IRecorderSink
{

public:


virtual bool save(
const MeasurementRecord&
record
)=0;


virtual std::string lastErrorText()
=0;


};
```

---

保存：

* 原始图像；
* result.json；
* calibration；
* config snapshot；
* 日志。

---

# 16 MeasurementRecord接口

一次测量结果：

包含：

```text id="z5q7mx"
MeasurementTask

+

MultiCameraFrame

+

TurntableState

+

MeasurementStatistics

+

TargetModel信息

+

FailureTrace

```

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

接口：

```cpp id="h8m3qy"
startMeasurement();


stopMeasurement();


MeasurementState state();

```

---

职责：

* 状态机；
* 任务管理；
* 流程调度。

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
setCamera(
CameraRole
);


setMode(
PreviewMode
);


getFrame(
ImageFrame&
);

```

---

# 21 错误接口

## ErrorInfo

```cpp id="m5q8xz"
struct ErrorInfo
{

ErrorCode code;


std::string message;

};
```

---

错误必须携带：

* 错误码；
* 状态；
* 原因。

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
ICD_V2.2
```

---

# 25 设计冻结总结

| 接口                 | 状态   |
| ------------------ | ---- |
| Camera接口           | 冻结   |
| MultiCamera接口      | 冻结   |
| Trigger接口          | 冻结   |
| Turntable接口        | 冻结   |
| Calibration接口      | 冻结   |
| Coordinate接口       | 冻结   |
| Algorithm接口        | 冻结   |
| TargetModel接口      | 新增冻结 |
| Recorder接口         | 冻结   |
| PipelineObserver接口 | 冻结   |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统接口控制基线。
