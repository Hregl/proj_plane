# AircraftPoseSystem 核心类设计 V2.1（多相机闭环测量版）

## 1 文档目的

本文档定义AircraftPoseSystem V2.1工程核心C++类设计。

V2.1相比V2.0新增：

-   OpticalRig光机系统；
-   AlignmentController目标对准；
-   TurntableController转台控制；
-   TriggerController硬触发；
-   MeasurementSelector测量通道选择；
-   TargetModelManager目标模型管理；
-   PoseValidator姿态验证；
-   CoordinateTransformer坐标转换。

本文档作为：

-   C++实现；
-   单元测试；
-   模块集成；

的核心类基线。

------------------------------------------------------------------------

# 2 核心设计原则

## 2.1 分层设计

    Application

    ↓

    Service Interface

    ↓

    Device / Algorithm

    ↓

    Hardware / Runtime

------------------------------------------------------------------------

## 2.2 接口隔离

硬件：

    Interface

    ↓

    Backend

算法：

    Pipeline

    ↓

    Algorithm Module

------------------------------------------------------------------------

## 2.3 依赖注入

核心对象由上层创建。

禁止：

``` cpp
class Controller
{
    CameraManager camera_;
};
```

推荐：

``` cpp
Controller(
    IMultiCameraManager* camera
);
```

------------------------------------------------------------------------

# 3 系统核心对象关系

整体创建：

    main.cpp

    ↓

    ConfigManager

    ↓

    Logger

    ↓

    OpticalRig

    ↓

    MultiCameraManager

    ↓

    TriggerController

    ↓

    TurntableController

    ↓

    PreviewManager

    ↓

    MeasurementController

    ↓

    MainWindow

------------------------------------------------------------------------

# 4 Data核心类

# 4.1 ImageFrame

文件：

    data/ImageFrame.h

职责：

保存单相机图像。

核心成员：

``` cpp
cv::Mat image;

uint64_t frameId;

uint64_t timestampNs;        // 主机 CLOCK_MONOTONIC，ns

uint64_t deviceTimestampNs;  // 相机内部时间戳，ns；不可用则 0

std::string cameraId;

CameraRole role;

double exposureTime;         // s
```

**裁决 C-01：** 原清单缺 `cameraId`、`exposureTime`、`deviceTimestampNs`，与 SYS-05 §5.1、ENG-02 §4.1 三处不一致。以 ENG-09 §5.5 冻结版本为准。

------------------------------------------------------------------------

# 4.2 MultiCameraFrame

文件：

    data/MultiCameraFrame.h

职责：

保存三相机同步数据。

成员：

``` cpp
ImageFrame cam25;

ImageFrame cam50;

ImageFrame cam100;

uint64_t triggerTimestamp;
```

------------------------------------------------------------------------

# 4.3 ShipPoseResult

文件：

    data/ShipPoseResult.h

职责：

保存最终姿态。

成员：

``` cpp
bool success;

double yaw;

double pitch;

double roll;                 // deg

Transform aircraftToShip;

double reprojectionError;    // pixel
```

**裁决 C-06：** 原字段名 `shipToAircraft` 与坐标链方向相反。坐标链合成结果为 **aircraft→ship**，故改为 `aircraftToShip`（ENG-09 §2.1）。

------------------------------------------------------------------------

# 5 Optical核心类

# 5.1 OpticalRig

文件：

    optical/OpticalRig.h

职责：

描述固定三相机光机结构。

成员：

``` cpp
std::vector<CameraChannel> cameras_;

OpticalRigCalibration calibration_;
```

------------------------------------------------------------------------

接口：

``` cpp
CameraChannel getCamera(
CameraRole role
);

OpticalRigCalibration calibration();
```

------------------------------------------------------------------------

# 5.2 CameraChannel

文件：

    optical/CameraChannel.h

职责：

描述一个完整光学通道。

成员：

``` cpp
std::string cameraId;

CameraRole role;

double focalLength;

CameraCalibration calibration;
```

------------------------------------------------------------------------

# 5.3 CalibrationManager

文件：

    optical/CalibrationManager.h

职责：

管理：

-   内参；
-   相机到Rig；
-   Rig到Ship。

接口：

``` cpp
bool load();

CameraCalibration get(CameraRole role);
```

------------------------------------------------------------------------

# 5.4 CoordinateTransformer

文件：

    optical/CoordinateTransformer.h

职责：

坐标转换。

转换：

    Camera

    ↓

    Rig

    ↓

    Ship

------------------------------------------------------------------------

# 6 Device核心类

# 6.1 MultiCameraManager

文件：

    device/camera/MultiCameraManager.h

职责：

管理三相机。

接口：

``` cpp
bool initializeAll();

bool startAll();

void stopAll();

bool capture(
MultiCameraFrame&
);
```

------------------------------------------------------------------------

# 6.2 ICameraBackend

文件：

    device/camera/ICameraBackend.h

职责：

统一相机接口。

接口：

``` cpp
initialize();

start();

stop();

grab();

setTriggerMode();
```

------------------------------------------------------------------------

# 6.3 ImvCameraBackend

文件：

    device/camera/ImvCameraBackend.h

职责：

华睿SDK实现。

------------------------------------------------------------------------

# 6.4 VirtualCameraBackend

职责：

虚拟设备测试。

------------------------------------------------------------------------

# 7 Trigger核心类

# 7.1 ITriggerController

文件：

    device/trigger/ITriggerController.h

职责：

硬触发抽象。

接口：

``` cpp
initialize();

enable();

trigger();

disable();
```

------------------------------------------------------------------------

# 7.2 HardwareTriggerController

真实硬件实现。

------------------------------------------------------------------------

# 7.3 VirtualTriggerController

测试实现。

------------------------------------------------------------------------

# 8 Turntable核心类

# 8.1 ITurntableController

文件：

    device/turntable/ITurntableController.h

职责：

两轴转台抽象。

接口：

``` cpp
initialize();

move();

state();

stop();
```

------------------------------------------------------------------------

# 8.2 PekoTurntableController

职责：

真实转台协议适配。

------------------------------------------------------------------------

# 8.3 VirtualTurntable

职责：

转台仿真。

------------------------------------------------------------------------

# 9 Preview核心类

# 9.1 PreviewManager

文件：

    preview/PreviewManager.h

职责：

实时预览。

支持：

-   AUTO；
-   MANUAL。

------------------------------------------------------------------------

# 9.2 PreviewWorker

职责：

预览线程。

数据：

    PreviewQueue

    ↓

    PreviewManager

    ↓

    Qt

------------------------------------------------------------------------

# 10 Application核心类

# 10.1 MeasurementController

文件：

    application/MeasurementController.h

职责：

测量任务总控制。

管理：

-   StateMachine；
-   Strategy；
-   Algorithm；
-   Recorder。

------------------------------------------------------------------------

# 10.2 AlignmentController

文件：

    application/AlignmentController.h

职责：

视觉闭环控制转台。

输入：

    TargetOffset

输出：

    TurntableCommand

------------------------------------------------------------------------

# 10.3 MeasurementStrategy

职责：

管理：

-   测量流程；
-   相机选择；
-   异常恢复。

------------------------------------------------------------------------

# 10.4 StateMachine

状态：

    IDLE

    SEARCH

    TARGET_FOUND

    ALIGN

    STABILIZE

    MEASURE_SELECT

    CAPTURE

    POSE_SOLVE

    VALIDATE

    SAVE

    COMPLETE

    FAILED

------------------------------------------------------------------------

# 10.5 RetryManager

文件：

    application/RetryManager.h

职责：

**唯一持有重试计数器与任务时限。** 状态机只做状态转换，不做重试决策。

接口：

``` cpp
bool beginAttempt(
MeasurementState state,
uint64_t nowNs
);

bool beginRollback(
MeasurementState from,
MeasurementState to,
uint64_t nowNs
);

bool deadlineExceeded(
uint64_t nowNs
) const;

void reset();
```

**策略来源：** SYS-08 §7（接口全文见 §7.6，本文档不重复定义）。
**配置来源：** `MeasurementConfig`（ENG-09 §6.5）。

**禁止：**

    StateMachine

    ↓

    int retryCount_;

禁止在任何状态类内保存局部重试计数器——计数分散后无法施加全局回退预算（SYS-08 §7.4）。

------------------------------------------------------------------------

# 11 Algorithm核心类

# 11.1 TargetDetector

路径：

    algorithm/detection/

职责：

YOLO检测。

------------------------------------------------------------------------

# 11.2 TargetScaleEstimator

路径：

    algorithm/scale/

职责：

尺度估计。

------------------------------------------------------------------------

# 11.3 MeasurementSelector

路径：

    algorithm/selection/

职责：

选择最佳测量通道。

输入：

    CAM25

    CAM50

    CAM100

输出：

最佳Camera。

------------------------------------------------------------------------

# 11.4 TargetModelManager

路径：

    algorithm/model/

职责：

管理：

-   CAD模型；
-   特征库。

------------------------------------------------------------------------

# 11.5 FeatureExtractor

路径：

    algorithm/feature/

职责：

特征提取。

------------------------------------------------------------------------

# 11.6 FeatureMatcher

路径：

    algorithm/matcher/

职责：

建立2D-3D对应。

------------------------------------------------------------------------

# 11.7 PnPPoseEstimator

路径：

    algorithm/pose/

职责：

PnP求解。

------------------------------------------------------------------------

# 11.8 PoseValidator

路径：

    algorithm/validation/

职责：

结果验证。

------------------------------------------------------------------------

# 12 Pipeline核心类

文件：

    algorithm/pipeline/PosePipeline.h

流程：

    Feature

    ↓

    Matching

    ↓

    PnP

    ↓

    Coordinate Transform

    ↓

    Validation

------------------------------------------------------------------------

# 13 Worker类设计

## CameraWorker

实例：

    Camera25Worker

    Camera50Worker

    Camera100Worker

------------------------------------------------------------------------

## TriggerWorker

职责：

触发管理。

------------------------------------------------------------------------

## TurntableWorker

职责：

转台控制。

------------------------------------------------------------------------

## SynchronizerWorker

职责：

三相机同步。

------------------------------------------------------------------------

## MeasurementSelectorWorker

职责：

测量通道选择。

------------------------------------------------------------------------

## AlgorithmWorker

职责：

算法执行。

------------------------------------------------------------------------

## RecorderWorker

职责：

数据保存。

------------------------------------------------------------------------

# 14 UI核心类

## MainWindow

负责：

用户交互。

------------------------------------------------------------------------

## ImageViewer

负责：

图像显示。

------------------------------------------------------------------------

## TurntablePanel

显示：

-   方位；
-   俯仰；
-   状态。

------------------------------------------------------------------------

# 15 禁止设计

禁止：

## UI访问设备

    MainWindow

    ↓

    CameraBackend

------------------------------------------------------------------------

## Algorithm访问SDK

    PnP

    ↓

    ImvSDK

------------------------------------------------------------------------

## Data包含业务

    ImageFrame

    ↓

    startMeasurement()

------------------------------------------------------------------------

# 16 验收标准

满足：

1.  核心类职责明确；
2.  接口可替换；
3.  支持Mock测试；
4.  支持真实硬件；
5.  支持虚拟设备；
6.  支持离线部署。
