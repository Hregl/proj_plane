# AircraftPoseSystem C++类与文件规划 V2.1（多相机闭环测量版）

## 1 文档目的

本文档定义AircraftPoseSystem
V2.1工程中的C++类组织、文件职责以及核心对象关系。

V2.1相比V2.0新增：

-   OpticalRig光机系统；
-   AlignmentController；
-   TurntableController；
-   TriggerController；
-   MeasurementSelector；
-   TargetScaleEstimator；
-   TargetModelManager；
-   PoseValidator；
-   CoordinateTransformer。

本文档作为编码实现前的类级工程基线。

------------------------------------------------------------------------

# 2 C++设计原则

## 2.1 单一职责

每个类只负责一个核心功能。

例如：

正确：

    MultiCameraManager

    负责相机管理

错误：

    MultiCameraManager

    同时负责：

    相机

    算法

    UI

------------------------------------------------------------------------

## 2.2 接口与实现分离

采用：

    Interface

    ↓

    Implementation

例如：

    ITurntableController

    ↓

    PekoTurntableController

------------------------------------------------------------------------

## 2.3 依赖注入

核心模块由上层创建。

禁止：

类内部创建复杂对象。

推荐：

``` cpp
MeasurementController(
    IMultiCameraManager* camera,
    IPosePipeline* pipeline
);
```

------------------------------------------------------------------------

# 3 核心对象关系

整体：

    main.cpp

    ↓

    ConfigManager

    ↓

    OpticalRig

    ↓

    MultiCameraManager

    ↓

    PreviewManager

    ↓

    MeasurementController

    ↓

    MainWindow

------------------------------------------------------------------------

# 4 Data层类设计

## 4.1 ImageFrame

文件：

    src/data/ImageFrame.h

职责：

保存单相机图像。

成员：

``` cpp
cv::Mat image;

uint64_t frameId;

uint64_t timestampNs;        // 主机 CLOCK_MONOTONIC，ns

uint64_t deviceTimestampNs;  // 相机内部时间戳，ns；不可用则 0

std::string cameraId;

CameraRole role;

double exposureTime;         // s
```

**裁决 C-01：** 原清单缺 `deviceTimestampNs` 与 `exposureTime`，与 SYS-05 §5.1、ENG-04 §4.1 三处不一致。以 ENG-09 §5.5 冻结版本为准。

------------------------------------------------------------------------

## 4.2 MultiCameraFrame

文件：

    src/data/MultiCameraFrame.h

职责：

保存同步采集结果。

成员：

``` cpp
ImageFrame cam25;

ImageFrame cam50;

ImageFrame cam100;

uint64_t triggerTimestamp;
```

------------------------------------------------------------------------

## 4.3 ShipPoseResult

文件：

    src/data/ShipPoseResult.h

职责：

保存舰体坐标姿态。

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

## 4.4 TurntableCommand

文件：

    src/data/TurntableCommand.h

职责：

转台控制命令。

------------------------------------------------------------------------

# 5 Optical层类设计

# 5.1 OpticalRig

文件：

    src/optical/OpticalRig.h

职责：

描述固定三相机光机结构。

成员：

``` cpp
std::vector<CameraChannel> cameras_;

OpticalRigCalibration calibration_;
```

------------------------------------------------------------------------

# 5.2 CameraRole

文件：

    src/data/CameraRole.h

定义：

``` cpp
enum class CameraRole
{

CAM25,

CAM50,

CAM100

};
```

------------------------------------------------------------------------

# 5.3 CameraChannel

文件：

    src/data/CameraChannel.h

职责：

描述单个光学通道。**归属 data 层**（裁决 C-07）。

成员：

``` cpp
std::string cameraId;

CameraRole role;

double focalLength;   // m，100mm 镜头 = 0.1

CameraCalibration calibration;

bool enabled;
```

**裁决 C-03：** 原清单缺 `enabled`，与 SYS-05 §4.4 不一致。以 ENG-09 §5.4 为准，保留 `enabled`（三相机需支持单独禁用，用于调试与降级运行）。

**注意单位：** `focalLength` 单位为**米**，100mm 镜头写作 `0.1`。这是最容易出错的一处，代码中必须注释（ENG-09 §2.3）。

------------------------------------------------------------------------

# 5.4 CalibrationManager

文件：

    src/optical/CalibrationManager.h

职责：

管理：

-   内参；
-   外参；
-   Rig到Ship。

接口：

``` cpp
load();

getCalibration();
```

------------------------------------------------------------------------

# 5.5 CoordinateTransformer

文件：

    src/optical/CoordinateTransformer.h

职责：

坐标转换。

负责：

    Camera

    ↓

    Rig

    ↓

    Ship

------------------------------------------------------------------------

# 6 Device层类设计

# 6.1 MultiCameraManager

文件：

    src/device/camera/MultiCameraManager.h

职责：

管理三台相机。

接口：

``` cpp
initializeAll();

startAll();

stopAll();

capture();
```

------------------------------------------------------------------------

# 6.2 ICameraBackend

文件：

    src/device/camera/ICameraBackend.h

职责：

统一相机接口。

------------------------------------------------------------------------

# 6.3 ImvCameraBackend

文件：

    src/device/camera/ImvCameraBackend.h

职责：

华睿SDK适配。

------------------------------------------------------------------------

# 6.4 VirtualCameraBackend

文件：

    src/device/camera/VirtualCameraBackend.h

用途：

测试。

------------------------------------------------------------------------

# 7 Trigger模块类设计

# 7.1 ITriggerController

文件：

    src/device/trigger/ITriggerController.h

职责：

硬触发抽象。

------------------------------------------------------------------------

# 7.2 HardwareTriggerController

文件：

    HardwareTriggerController.h

真实设备。

------------------------------------------------------------------------

# 7.3 VirtualTriggerController

文件：

    VirtualTriggerController.h

测试模拟。

------------------------------------------------------------------------

# 8 Turntable模块类设计

# 8.1 ITurntableController

文件：

    src/device/turntable/ITurntableController.h

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

文件：

    PekoTurntableController.h

职责：

真实转台协议适配。

------------------------------------------------------------------------

# 8.3 VirtualTurntable

文件：

    VirtualTurntable.h

用途：

仿真测试。

------------------------------------------------------------------------

# 9 Preview层类设计

# 9.1 PreviewManager

文件：

    src/preview/PreviewManager.h

职责：

实时预览管理。

支持：

-   AUTO；
-   MANUAL。

------------------------------------------------------------------------

# 9.2 PreviewWorker

文件：

    src/preview/PreviewWorker.h

职责：

预览线程。

------------------------------------------------------------------------

# 10 Application层类设计

# 10.1 MeasurementController

文件：

    src/application/MeasurementController.h

职责：

测量总控制。

管理：

-   状态机；
-   设备；
-   算法。

------------------------------------------------------------------------

# 10.2 AlignmentController

文件：

    src/application/AlignmentController.h

职责：

视觉转台闭环。

输入：

    TargetOffset

输出：

    TurntableCommand

------------------------------------------------------------------------

# 10.3 MeasurementStrategy

文件：

    src/application/MeasurementStrategy.h

职责：

测量策略控制。

------------------------------------------------------------------------

# 10.4 StateMachine

文件：

    src/application/StateMachine.h

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

------------------------------------------------------------------------

# 11 Algorithm层类设计

# 11.1 TargetDetector

文件：

    algorithm/detection/TargetDetector.h

功能：

YOLO检测。

------------------------------------------------------------------------

# 11.2 TargetScaleEstimator

文件：

    algorithm/scale/TargetScaleEstimator.h

功能：

距离/尺度估计。

------------------------------------------------------------------------

# 11.3 MeasurementSelector

文件：

    algorithm/selection/MeasurementSelector.h

功能：

选择最佳测量相机。

------------------------------------------------------------------------

# 11.4 TargetModelManager

文件：

    algorithm/model/TargetModelManager.h

功能：

管理：

-   CAD模型；
-   特征库。

------------------------------------------------------------------------

# 11.5 FeatureExtractor

文件：

    algorithm/feature/FeatureExtractor.h

功能：

特征提取。

------------------------------------------------------------------------

# 11.6 FeatureMatcher

文件：

    algorithm/matcher/FeatureMatcher.h

功能：

建立2D-3D对应。

------------------------------------------------------------------------

# 11.7 PnPPoseEstimator

文件：

    algorithm/pose/PnPPoseEstimator.h

功能：

PnP姿态解算。

------------------------------------------------------------------------

# 11.8 PoseValidator

文件：

    algorithm/validation/PoseValidator.h

功能：

结果验证。

------------------------------------------------------------------------

# 12 Pipeline设计

文件：

    algorithm/pipeline/PosePipeline.h

流程：

    Detection

    ↓

    Selection

    ↓

    Feature

    ↓

    Matching

    ↓

    PnP

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

文件：

    TriggerWorker.h

------------------------------------------------------------------------

## TurntableWorker

文件：

    TurntableWorker.h

------------------------------------------------------------------------

## SynchronizerWorker

文件：

    SynchronizerWorker.h

------------------------------------------------------------------------

## AlgorithmWorker

文件：

    AlgorithmWorker.h

------------------------------------------------------------------------

## RecorderWorker

文件：

    RecorderWorker.h

------------------------------------------------------------------------

# 14 UI类设计

## MainWindow

    ui/MainWindow.h

------------------------------------------------------------------------

## ImageViewer

    ui/widgets/ImageViewer.h

------------------------------------------------------------------------

## TurntablePanel

    ui/widgets/TurntablePanel.h

显示：

-   方位；
-   俯仰；
-   状态。

------------------------------------------------------------------------

# 15 对象创建顺序

main.cpp：

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

# 16 禁止设计

禁止：

## UI创建设备

    MainWindow

    ↓

    CameraSDK

------------------------------------------------------------------------

## Algorithm调用设备

    PnP

    ↓

    CameraBackend

------------------------------------------------------------------------

## Data包含业务

    ImageFrame

    ↓

    startMeasurement()

------------------------------------------------------------------------

# 17 验收标准

满足：

1.  类职责清晰；
2.  接口可替换；
3.  支持Mock测试；
4.  支持虚拟设备；
5.  支持真实硬件；
6.  支持离线部署。
