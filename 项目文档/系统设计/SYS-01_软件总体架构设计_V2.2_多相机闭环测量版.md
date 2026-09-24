# SYS-01_软件总体架构设计_V2.2（多相机闭环测量版）

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
* **软件生命周期阶段（V2.2 新增，§24）；**
* **工程资产链（V2.2 新增，§25）；**

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
* OpticalRigCalibration；
* Transform；
* ShipPoseResult；
* MeasurementRecord。

---

⚠ V2.2 修正：V2.1 此处写作 `CalibrationPackage`。该类型名在代码中**零命中**，
实际类型为 `data::OpticalRigCalibration`（含 `cam25` / `cam50` / `cam100` / `rigToShip`）。
本次按 SYS-04 ICD V2.3 的同一条裁决（文档向代码收敛，减少虚构类型）同步修正。

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

# 24 软件生命周期阶段

系统按四个阶段推进。**每个阶段有独立的完成判据，不得跳阶段。**

```text id="lp4m8x"
Phase 1  Framework Alpha

↓

Phase 2  Hardware Integration

↓

Phase 3  Measurement Beta

↓

Phase 4  Release

```

---

## 24.1 Phase 1 — Framework Alpha

目标：**架构与软件链路成立，全部外部输入为虚拟件。**

特征：

* 三相机全部为 `VirtualCameraBackend`；
* 无任何厂商 SDK 依赖；
* 无实机、无转台、无触发硬件；
* 全链路可离线跑通：采集 → 检测 → 特征 → 匹配 → PnP → 校验 → 落盘。

完成判据：

```text id="fa6n2q"
无 SDK 时工程可配置、可构建、可运行

连续测量闭环跑通，结果包可落盘并可重新读出

Unit / Module / Integration 三层测试全绿
```

状态：**✅ 已完成**（对应 M1 达成；10 个套件、242 用例、0 失败）。

---

## 24.2 Phase 2 — Hardware Integration

目标：**用真实硬件替换虚拟件，逐件替换、逐件验证。**

特征：

* 相机由 `VirtualCameraBackend` 换为 `ImvCameraBackend`（A7A20MU201）；
* 转台、触发按同一方式接入；
* **替换顺序：先单件，后组合** —— 不得一次性换掉全部虚拟件。

完成判据：

```text id="hi7m3p"
真实相机连续采集运行

真实转台闭环对准运行

触发 / 同步判据在真实时间戳下成立
```

状态：**进行中**。当前处于 `011-A0`（SDK 环境准备，✅ 通过）→ `011-A1`（真实相机接入）。

⚠ **本阶段的失败模式与 Phase 1 不同**：Phase 1 的错误几乎都会以编译失败或断言失败暴露；
本阶段的典型错误是"**链路通了但没有数据**"—— 例如 SDK 链路打通而设备枚举未通、
枚举通了而取流参数不匹配、取到图而时间戳语义不对。
故本阶段每一项都必须有**可观测的验收判据**，不能以"程序没报错"代替。

---

## 24.3 Phase 3 — Measurement Beta

目标：**测量精度达标，进入验收量级。**

特征：

* 三相机全部真实；
* 使用 **Production 模型**（非 synthetic）；
* 使用真实标定（非 `loadDefaults` 合成标定）。

完成判据：

```text id="mb5q9r"
Yaw ≤ 1 角分（SYS-15 §4 误差预算的验收指标）

σ_px < 0.3 pixel（SYS-15 §4.5 硬性要求）

CadStructureLocator 残差 ≤ 0.3 pixel
且与人工标注结构点偏差 ≤ 0.5 pixel（ENG-08 §10 任务 A）
```

⚠ **本阶段最大的未闭环环节**是 CAD 结构点的亚像素定位 —— 该项从未实测。
误差预算合成结果为 0.97 角分，距 1 角分指标仅 **3% 余量**，
故该环节一旦超出预期，指标即失守。

---

## 24.4 Phase 4 — Release

目标：**离线交付，现场可部署可复现。**

特征：

* 按 SYS-18 的部署形态交付（离线包，无网络依赖）；
* 结果包携带完整追溯链（软件版本 / 标定 ID / 模型 ID / 配置版本，见 §20）。

完成判据：

```text id="rl8n4k"
release 构建拒绝 synthetic 模型进入测量路径

现场解压后可直接启动，无需人工设置环境变量

结果包可由第三方独立复核
```

---

## 24.5 阶段与虚拟件的对应

```text id="pv2m7z"
               CAM25        CAM50        CAM100       转台      触发
Phase 1        Virtual      Virtual      Virtual      Virtual   Virtual
Phase 2        Real         Virtual      Virtual      Real      Real  ← 逐件替换
Phase 3        Real         Real         Real         Real      Real
Phase 4        Real         Real         Real         Real      Real
```

⚠ **Phase 2 只允许 CAM25 先转真实**，CAM50 / CAM100 保持虚拟。
理由：三路同时接入时，任何一路失败都无法区分是"该路硬件问题"还是"多路协同问题"。
详见 SYS-02 的阶段需求与 SYS-18 的部署阶段。

---

# 25 工程资产链

系统运行依赖**三类工程资产**。三者均为**交付物**，随版本一同冻结。

```text id="ac9m4v"
Calibration Package        标定资产

Target Model Package       目标模型资产

Configuration Package      配置资产

```

---

## 25.1 三类资产的归属

| 资产 | 内容 | 落点 | 权威文档 |
|---|---|---|---|
| **Calibration Package** | 三路内参 K、畸变 D、`Camera→Rig` 外参、`Rig→Ship` 外参、标定 ID | `deploy/calibration/` | SYS-16（标定实施）、SYS-11（光机刚体标定） |
| **Target Model Package** | 3D 结构点表 + 三焦段各自的描述子库 + 模型 ID / 版本 | `deploy/models/` | SYS-12（目标模型与特征库） |
| **Configuration Package** | `system.yaml` / `camera.yaml` / `optical_rig.yaml` / `trigger.yaml` / `turntable.yaml` / `measurement.yaml` / `validation.yaml` | `deploy/config/` | SYS-17（系统配置与参数管理） |

---

## 25.2 ⚠ 资产名 ≠ C++ 类型名

**这是本项目已实际发生过的一次混淆，须登记。**

```text id="an5q3m"
Calibration Package     ← 资产（交付物）名

data::OpticalRigCalibration   ← 该资产在内存中的 C++ 类型
```

`CalibrationPackage` 曾被当作一个 **C++ 类型**写进冻结文档
（SYS-01 §6.1、SYS-04 §13.1），而代码中该类型**零命中** ——
实际承载标定资产的类型是 `data::OpticalRigCalibration`。

2026-09-24 已修正：SYS-04 V2.3 与本文档 §6.1 均改为 `data::OpticalRigCalibration`。

⚠ **资产命名与类型命名今后必须分开**：本节的三类资产是"交付物"，
它们**不**对应任何名为 `CalibrationPackage` / `TargetModelPackage` /
`ConfigurationPackage` 的 C++ 类型。引用内存中的标定数据时，
一律使用真实类型名。

---

## 25.3 资产链的冻结与追溯

一次测量结果必须能回答"用的是哪一份资产"：

```text id="tr6w9n"
结果包
  ├── calibrationId     ← 指向 Calibration Package
  ├── modelId           ← 指向 Target Model Package
  ├── modelType         ← production / synthetic
  └── softwareVersion   ← 指向软件版本
```

⚠ `calibrationId` 与 `modelId` 由**装配点**注入并写入 `MeasurementRecord`
（见 SYS-04 V2.3 §16）。没有这两个字段，结果包在事后**无法判断**它来自哪一份标定或模型 ——
而"用错版本标定算出的姿态"在数值上完全正常，不会自报。

---

# 26 设计冻结总结

| 模块          | 状态 | V2.2 变动 |
| ----------- | -- | ------ |
| 总体分层        | 冻结 | 未改 |
| Device      | 冻结 | 未改 |
| Optical     | 冻结 | 未改 |
| Algorithm   | 冻结 | 未改 |
| Application | 冻结 | 未改 |
| Preview     | 冻结 | 未改 |
| Data        | 冻结 | `CalibrationPackage` → `OpticalRigCalibration`（§6.1） |
| 标定体系        | 冻结 | 未改 |
| 坐标体系        | 冻结 | 未改 |
| **软件生命周期阶段** | **新增冻结** | §24 |
| **工程资产链**    | **新增冻结** | §25 |

---

# 27 修订记录

| 版本 | 日期 | 修订内容 |
|---|---|---|
| V2.1 | — | 多相机闭环测量版 |
| **V2.2** | **2026-09-24** | **增量补充**：① 新增 §24 软件生命周期阶段（Framework Alpha / Hardware Integration / Measurement Beta / Release 四阶段及各自完成判据）；② 新增 §25 工程资产链（Calibration Package / Target Model Package / Configuration Package 三类资产及其归属，并登记「资产名 ≠ C++ 类型名」这一已发生的混淆）；③ 修正 §6.1 的 `CalibrationPackage` → `data::OpticalRigCalibration`（与 SYS-04 ICD V2.3 同一裁决）。**未改动其余章节，未改变软件架构。** |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统总体架构设计基线（V2.2）。
