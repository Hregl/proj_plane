# SYS-05_Data数据结构设计_V2.1（标定融合与闭环测量版）

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

};
```

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

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统 Data层设计基线。
