# SYS-01_软件总体架构设计_V2.1（多相机闭环测量版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 软件总体架构。

本版本针对：

* 三相机固定光机结构；
* 锁焦三镜头测量体系；
* 光机标定体系；
* 视觉闭环对准；
* 转台控制接口；
* 舰体坐标姿态输出；

进行总体架构设计。

---

# 2 系统目标

系统实现：

```text id="h9p4qz"
目标发现

↓

视觉对准

↓

多相机采集

↓

最佳焦段选择

↓

目标特征匹配

↓

PnP姿态求解

↓

舰体坐标输出

```

---

# 3 总体架构设计原则

## 3.1 分层设计

系统采用：

```text id="7c4mvp"
UI层

↓

Application业务层

↓

Algorithm算法层

↓

Device设备层

Optical光机层

↓

Data数据层

```

---

## 3.2 依赖方向

冻结：

```text id="0f7m9x"
UI

↓

Application

↓

Algorithm / Device / Optical

↓

Data
```

---

禁止：

```text id="9q4wml"
Algorithm

↓

Camera SDK
```

---

禁止：

```text id="6r8jhz"
UI

↓

Hardware
```

---

# 4 系统逻辑架构

整体：

```text id="3y9qmv"
                  MainWindow

                       |

                       |

             MeasurementController


                       |

      ---------------------------------

      |               |              |

   Device          Optical       Algorithm


      |               |              |

 CameraManager   Calibration    PosePipeline


      |

 ----------------

 |       |        |

CAM25  CAM50   CAM100


```

---

# 5 软件模块划分

工程结构：

```text id="9z6x4k"
AircraftPoseSystem

├── data

├── device

├── optical

├── preview

├── algorithm

├── application

├── infrastructure

├── ui

└── app

```

---

# 6 Data数据层

## 6.1 职责

负责：

* 公共数据结构；
* 模块间数据传输；
* 数据定义。

---

核心数据：

包括：

* ImageFrame；
* MultiCameraFrame；
* CalibrationPackage；
* Transform；
* ShipPoseResult；
* MeasurementRecord。

---

# 7 Device设备层

## 7.1 职责

管理：

* 相机；
* 触发；
* 转台。

---

## 7.2 Camera架构

```text id="5v2x7j"
ICameraBackend

↓

ImvCameraBackend

↓

A7A20MU201

```

---

同时支持：

```text id="8m1qzf"
VirtualCameraBackend
```

用于：

* 软件开发；
* 测试。

---

## 7.3 MultiCameraManager

负责：

三通道管理：

```text id="4z7cmt"
CAM25

CAM50

CAM100
```

---

输出：

```text id="f7j2mq"
MultiCameraFrame
```

---

# 8 Optical光机层

## 8.1 职责

管理：

* 三相机固定关系；
* 标定参数；
* 坐标转换。

---

## 8.2 OpticalRig

维护：

```text id="2x9w7v"
Camera25→Rig

Camera50→Rig

Camera100→Rig

```

---

## 8.3 CalibrationManager

负责：

加载：

* 内参；
* 畸变；
* 外参；
* Calibration ID。

---

## 8.4 CoordinateTransformer

负责：

```text id="6k3z8q"
Camera

↓

Rig

↓

Ship

```

---

# 9 Preview预览层

## 9.1 职责

提供：

实时图像显示。

---

数据流：

```text id="w8m3xp"
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

## 9.2 Preview策略

默认：

```text id="9v6h4y"
CAM25
```

支持：

* 手动切换；
* 自动选择。

---

# 10 Algorithm算法层

## 10.1 职责

完成：

* 检测；
* 特征；
* 匹配；
* 姿态。

---

流程：

```text id="5p7n2d"
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

# 11 Application业务层

## 11.1 MeasurementController

负责：

系统测量流程。

管理：

* 状态机；
* 任务；
* 模块调度。

---

## 11.2 状态机

状态：

```text id="w7k3p9"
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

```

---

# 12 视觉闭环架构

流程：

```text id="z8m2ky"
目标偏差

↓

AlignmentController

↓

TurntableCommand

↓

Turntable

↓

目标居中

```

---

输入：

```cpp id="d5n8vx"
TargetOffset
```

输出：

```cpp id="m7q3zc"
TurntableCommand
```

---

# 13 多相机测量策略

三个镜头：

```text id="x5n7mv"
CAM25

CAM50

CAM100
```

不是固定距离切换。

采用：

目标像素驱动。

输入：

* 距离；
* 目标尺寸；
* 特征数量；
* 清晰度；
* 预测误差。

输出：

最佳测量通道。

---

# 14 标定架构

标定体系：

```text id="p6v9qw"
Camera Intrinsic

+

Camera→Rig

+

Rig→Ship

```

---

来源：

SYS-16。

---

在线运行：

冻结：

* K；
* D；
* 外参。

---

# 15 坐标架构

坐标链：

```text id="b4x7qa"
Aircraft

↓

Camera

↓

Rig

↓

Ship

```

---

最终输出：

```text id="q5w8mz"
Aircraft Pose in Ship Frame
```

---

# 16 数据记录架构

一次测量：

生成：

```text id="n7c3vp"
MeasurementRecord

↓

Recorder

↓

measurement_xxx/

```

---

包含：

* 原始图像；
* 姿态结果；
* 标定信息；
* 模型信息；
* 统计信息；
* 日志。

---

# 17 多线程架构

线程：

```text id="k8y2sp"
CameraWorker

TriggerWorker

TurntableWorker

SynchronizerWorker

AlgorithmWorker

PreviewWorker

RecorderWorker

```

---

线程通信：

采用：

* Queue；
* Signal/Slot；
* 共享数据对象。

---

# 18 实时性架构

目标：

```text id="j9w4mv"
Feature开始

↓

Yaw输出

≤100ms
```

---

策略：

* 三相机采集并行；
* Preview独立；
* 算法单通道执行；
* Recorder异步。

---

# 19 配置架构

配置：

```text id="m2h8xr"
config/

├── system.yaml

├── camera.yaml

├── optical_rig.yaml

├── trigger.yaml

├── turntable.yaml

├── measurement.yaml

└── validation.yaml

```

---

# 20 版本与追溯架构

一次结果绑定：

```text id="y5v8nk"
Software Version

+

Calibration ID

+

Model ID

+

Config Version
```

---

# 21 测试架构

测试分层：

```text id="c9m4hx"
Unit Test

↓

Module Test

↓

Integration Test

↓

Hardware Test

↓

System Acceptance
```

---

# 22 部署架构

部署目录：

```text id="r6k2pz"
deploy/

├── bin

├── lib

├── config

├── models

└── calibration

```

---

# 23 禁止架构

禁止：

## UI直连硬件

错误：

```text id="p3m7cz"
MainWindow

↓

CameraSDK
```

---

禁止：

## 算法直连设备

错误：

```text id="v8x4qm"
PnP

↓

Camera
```

---

禁止：

## 在线修改标定

错误：

```text id="w2n6ks"
Measurement

↓

Optimize Intrinsic
```

---

# 24 设计冻结总结

| 模块          | 状态 |
| ----------- | -- |
| 总体分层        | 冻结 |
| Device      | 冻结 |
| Optical     | 冻结 |
| Algorithm   | 冻结 |
| Application | 冻结 |
| Preview     | 冻结 |
| Data        | 冻结 |
| 标定体系        | 冻结 |
| 坐标体系        | 冻结 |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统总体架构设计基线。
