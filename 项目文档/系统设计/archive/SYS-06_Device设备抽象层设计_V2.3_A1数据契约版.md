# SYS-06_Device设备抽象层设计_V2.3（A1 数据契约版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 多相机闭环测量系统设备抽象层设计。

本版本针对：

* A7A20MU201真实相机接入；
* ImvSDK适配；
* 三相机架构扩展；
* Virtual设备保留；
* 硬件触发；
* 转台控制接口；

进行补充设计。

---

# 1.1 V2.3 修订性质

V2.3 是 **011-A1 数据契约批**（裁决 **C-015**，2026-09-26）在 Device 层设计文档上的落地。
本版**动了既有冻结接口签名** —— `ICameraBackend` 由 **6 个**方法变为 **11 个**。

**接口面的唯一权威是 `SYS-04 V2.4 §6.1`**（ICD），本文件是**设计层叙述**：
两者若不一致，**以 SYS-04 V2.4 §6.1 为准**。
变量类型（`OpStatus` / `OperationResult` / `GrabResult` / `DeviceIdentity` /
`TriggerModeState` / `CameraTriggerMode`）的权威是 **ENG-09 V2.3 §5.28～§5.31**。

**本版改动清单**：

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 5.1 | `ICameraBackend` 由 6 方法扩为 11 方法（返回类型与形参一并更新） | **签名** |
| 2 | 6.2 | 采集流程补"软件触发发令由 `MultiCameraManager` 单点执行"的责任边界 | 新增 |
| 3 | 6.3 | `ImageFrame` 必填字段增 `raw` / `captureFormat` / `rawPolicy` | 字段 |
| 4 | 12.1 | `camera.yaml` 键表按实际配置修正（原表含 5 个不存在的键名） | 一致性修正 |

⚠ 本文件经检索**不含任何 `SYS-08 §7.x` 引用**，故本批的 §7.x 勘误（Q-D2）
**不涉及本文件**。

⚠ **本版不改变任何恢复决策**：断连不重连、超时不重试、降级策略一字未改
（属待裁决 Q-D1）。本版只把**结果分类**与**证据归属**补进接口，使"一次超时"
不再与"一次真实断连"在接口上同形 —— R09 的已知根因正是
"所有取帧失败被统一禁用"。

---

# 2 Device层总体职责

Device层负责：

* 相机设备管理；
* 图像采集；
* 触发控制；
* 转台通信；
* 硬件状态监测。

---

Device层不负责：

* 特征提取；
* PnP姿态求解；
* 标定计算；
* UI显示；
* 测量策略。

---

# 3 Device总体架构

```text id="5m8qzx"
Device Layer


├── Camera

│
├── Trigger

│
└── Turntable

```

---

# 4 Device分层设计

结构：

```text id="k7m3qx"
Interface

↓

Backend

↓

Hardware SDK

↓

Physical Device

```

---

示例：

```text id="w8m5qp"
ICameraBackend

↓

ImvCameraBackend

↓

ImvSDK

↓

A7A20MU201

```

---

# 5 Camera设备设计

---

# 5.1 ICameraBackend接口

职责：

统一相机访问。

接口：

```cpp id="r6m8qx"
class ICameraBackend
{

public:

virtual ~ICameraBackend()=default;


virtual data::OperationResult initialize()=0;


virtual data::OperationResult start()=0;


virtual data::OperationResult stop()=0;


virtual data::OperationResult close()=0;


virtual data::GrabResult grab(
data::ImageFrame& frame,

uint32_t timeoutMs
)=0;


virtual data::OperationResult setTriggerMode(
data::CameraTriggerMode mode
)=0;


virtual data::OperationResult triggerSoftware()=0;


virtual data::DeviceIdentity deviceIdentity() const =0;


virtual data::TriggerModeState triggerModeState() const =0;


virtual std::string lastErrorText() const =0;


};
```

⚠ **V2.3 修正（裁决 C-015）**：本接口由 **6 个**方法变为 **11 个**。
逐方法的契约、`@return` 的逐值语义、三级释放归属、桩的形态
**一律以 `SYS-04 V2.4 §6.1` 与其 §6.2 为准**，本节不重复。
三条必须在本层说明的**责任边界**：

1. **`grab()` 只取帧，绝不发令。** 软件触发的发令归 `MultiCameraManager`
   （**单执行者**）—— 后端不知道也不该知道"上层是否已经发过令"，
   后端再发一次会变成两拍。
2. **`stop()` 不等于 `close()`。** `stop()` 只停流（允许未 `start` 时调用、须无副作用、幂等）；
   `close()` 才关设备并销毁句柄（本版新增，幂等、允许未 `initialize()` 时调用）。
3. **`initialize()` 的失败出口必须自行 `close()`** —— 半初始化不许留在句柄上。

**既有 `@return` 语义的变更**：原接口用 `bool` 把"超时、断连、未 start"
混在一句话里，属**分类缺失**（使上层无法按原因处置）。本版改为逐值可读的
`OperationResult` / `GrabResult`。

---

# 5.2 VirtualCameraBackend

用途：

软件开发和测试。

支持：

```text id="p5m7qx"
CAM25

CAM50

CAM100
```

---

功能：

模拟：

* 图像输出；
* frameId；
* timestamp。

---

# 5.3 ImvCameraBackend

真实设备实现。

目标设备：

```text id="h7m2qx"
A7A20MU201
```

---

负责：

* ImvSDK初始化；
* 相机枚举；
* 参数设置；
* 图像采集。

---

# 6 ImvCameraBackend实现要求

---

# 6.1 初始化流程

流程：

```text id="n4m8qx"
Load SDK

↓

Enumerate Camera

↓

Open Device

↓

Read Parameters

↓

Ready

```

---

# 6.2 采集流程

流程：

```text id="x7m3qp"
Grab Image

↓

Convert Mono Image

↓

Fill ImageFrame

↓

Push Queue

```

**V2.3 补充的责任边界（单执行者）**：

```text id="tq4w8n"
MultiCameraManager::capture()

↓ （若该路模式读回为 Software）

ExecuteCommandFeature("TriggerSoftware")   恰好一次

↓

ICameraBackend::grab()                     恰好一次 —— 后端不发令

↓

Release SDK Buffer（复制必须先于释放）
```

⚠ 任一通道一次 `capture()` 内 `IMV_ExecuteCommandFeature` 与 `IMV_GetFrame`
的次数**都是 1**；**发令失败 ⇒ `grab()` 次数为 0**（不得"失败也照样去等一帧"）。
**"等待一帧"不等于"发出了一次软件触发"**。

⚠ **`Mono12` 的转换**：显示转换（`8U = 像素值 >> (validBits − 8)`）由后端实现，
**不委派 `IMV_PixelConvert`** —— 委派会把缩放／对齐规则藏进 SDK 内部而无法核实。
本批 `Mono12` 的依据是 **PFNC 2.4 §6.1.1**（格式码逐位相符），
实机核实的是"**本台设备的输出符合该约定**"。

---

# 6.3 ImageFrame字段要求

必须填写：

```text id="q5m8zx"
frameId

timestampNs

deviceTimestampNs

cameraId

CameraRole

image

raw

captureFormat

rawPolicy

```

⚠ **V2.3 新增三个字段**（依据 ENG-09 V2.3 §5.5）：
`raw`（原始载荷及其解释信息）、`captureFormat`（本帧采集格式）、
`rawPolicy`（原始载荷必要性策略）。
后两者**独立于 `raw` 是否为空** —— 它们回答"这一帧**本来应该带什么**"，
不是"它实际带了什么"。

⚠ **`image.type()` 永远不作为裸缓冲的解码依据**：裸缓冲的解码依据**只有**
`raw` 的 `format` / `validBits` / `packing` / `declaredByteOrder`。
`image` 与 `raw` **都不得悬挂在 SDK 缓冲区上** —— 两者都必须在
`IMV_ReleaseFrame` 之前完成复制。

---

# 6.4 时间戳要求

## timestampNs

来源：

主机单调时钟。

用途：

* 排序；
* 同步。

禁止：

使用系统墙钟。

---

## deviceTimestampNs

如果SDK提供：

填写。

如果SDK不提供：

必须：

```cpp id="m8q2xz"
0
```

禁止：

复制timestampNs。

---

# 7 SDK发现与编译设计

## 7.1 ImvSDK目录

采用：

外部SDK路径。

示例：

```text id="q8m3xy"
third_party/

└── imvsdk/

    ├── include/

    ├── lib/

    └── runtime/

```

---

# 7.2 CMake发现策略

优先：

```text id="n5m7qx"
IMV_SDK_ROOT

↓

环境变量

↓

默认路径
```

---

禁止：

仅依赖：

系统全局搜索。

---

# 7.3 SDK缺失状态

如果：

ImvSDK不存在。

要求：

VirtualCamera仍可：

* 编译；
* 运行；
* 测试。

---

真实相机模块：

独立禁用。

---

# 8 MultiCameraManager设计

## 8.1 职责

统一管理：

```text id="z4m8qx"
CAM25

CAM50

CAM100
```

---

# 8.2 当前开发阶段配置

当前硬件条件：

```text id="y6m3qp"
CAM25

真实A7A20MU201


CAM50

VirtualCamera


CAM100

VirtualCamera

```

---

# 8.3 最终配置

三台真实相机：

```text id="q3m7xz"
CAM25

CAM50

CAM100

```

---

# 9 Trigger设备设计

---

# 9.1 ITriggerController

接口：

```cpp id="m8q5zx"
class ITriggerController
{

virtual bool initialize()=0;


virtual bool enable()=0;


virtual bool trigger()=0;


virtual void disable()=0;

};
```

---

# 9.2 硬件触发

支持：

A7A20MU201：

* Hirose接口；
* 光耦输入；
* 外部触发。

---

# 9.3 Trigger工作流程

```text id="p7m4qx"
TriggerController

↓

Hardware Trigger

↓

Camera Exposure

↓

ImageFrame

```

---

# 10 MultiCamera同步设计

## 10.1 同步目标

保证：

三相机图像：

属于：

同一次曝光事件。

---

## 10.2 同步依据

包括：

* trigger事件；
* frameId；
* timestampNs；
* exposureIndex。

---

# 11 Turntable设备设计

---

# 11.1 ITurntableController

接口：

```cpp id="w5m8qx"
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

# 11.2 PekoTurntableController

负责：

* Peko_D协议；
* SDK调用；
* 网络接口；
* RS485接口。

---

# 11.3 VirtualTurntable

用途：

* 软件测试；
* 无硬件开发。

---

# 12 Device配置

配置文件：

```text id="m4q8zx"
config/

├── camera.yaml

├── trigger.yaml

└── turntable.yaml

```

---

# 12.1 camera.yaml

包含（**实际键名**，与 `config/camera.yaml` 一致）：

```yaml id="p8m2qx"
id:            # 通道的**逻辑名**（CameraChannel::cameraId），与焦段绑定
role:          # CAM25 / CAM50 / CAM100
backend:       # virtual | imv
serial:        # 设备标签上的序列号（期望值）；backend=imv 时必须非空
width:         # 分辨率
height:
exposure_time: # 单位 **秒**
gain:          # 单位 dB
trigger_mode:  # software | hardware | free_run
focal_length:  # 单位 **米**
```

⚠ **V2.3 一致性修正**：原表写的 `camera_id` / `camera_role` / `resolution` /
`pixel_format` / `backend_type` 五个键名**在配置文件与 `CameraConfig` 中都不存在**，
现按实际键名修正。权威键表见 **SYS-17 V1.1 §5**。

注意：

`backend` / `serial` 属于：

**系统装配配置**（逐通道**显式声明**，缺键即启动失败，不设隐式默认）。

它进入 `CameraConfig`（ENG-09 V2.3 §6.1），但**不进入 Data 层的帧数据** ——
后端类型由装配摘要的**独立一栏**表达，**不得**填进 `ImageFrame` 或
`DeviceIdentity.modelName`（ENG-09 V2.3 §5.30 第 3 条）。

---

# 13 Device生命周期

## 初始化

```text id="z7m5qx"
ApplicationContext

↓

Device initialize

↓

Ready

```

---

## 启动

```text id="h8m3qp"
start()

↓

采集

↓

运行
```

---

## 停止

```text id="q6m8xz"
stop()

↓

Release SDK

↓

Close Device

```

---

# 14 异常处理

## 14.1 相机异常

包括：

* SDK断开；
* 无图像；
* 超时。

输出：

```text id="m7q3xz"
ErrorInfo
```

---

# 14.2 Trigger异常

包括：

* 无触发；
* 同步超差。

---

# 14.3 转台异常

包括：

* 通信失败；
* 超限；
* 停止失败。

---

# 15 Device与其他模块关系

允许：

```text id="q5m8yx"
Application

↓

Device

↓

Data
```

---

禁止：

```text id="x8m3qp"
Device

↓

Algorithm
```

---

禁止：

```text id="w7m2zx"
Device

↓

UI
```

---

# 16 测试要求

## VirtualCamera

验证：

* frameId递增；
* timestamp递增；
* 图像输出。

---

## ImvCameraBackend

验证：

* SDK加载；
* 相机连接；
* 单帧采集；
* 连续采集。

---

## MultiCameraManager

验证：

* 三通道管理；
* 异常恢复；
* 同步接口。

---

# 17 当前开发路线

## Phase 1

软件框架：

```text id="h4m8qx"
VirtualCamera

↓

Preview

```

---

## Phase 2

真实单相机：

```text id="n7m5qx"
A7A20MU201

↓

ImvSDK

↓

Preview

```

---

## Phase 3

三相机扩展：

```text id="z6m8qx"
CAM25

CAM50

CAM100

```

---

# 18 设计冻结总结

| 项目                 | 状态 |
| ------------------ | -- |
| Camera接口           | 冻结 |
| ImvCameraBackend   | 冻结 |
| VirtualCamera      | 保留 |
| MultiCameraManager | 冻结 |
| Trigger接口          | 冻结 |
| Turntable接口        | 冻结 |
| SDK隔离              | 冻结 |
| 真实硬件接入路线           | 冻结 |

---

本文档作为 AircraftPoseSystem V2.3 多相机闭环测量系统 Device设备抽象层设计基线。

---

# 19 修订记录

| 版本 | 日期 | 修订性质 | 备注 |
|---|---|---|---|
| V2.2 | — | 真实硬件接入版 | 旧版已归档至 `archive/SYS-06_Device设备抽象层设计_V2.2_多相机闭环测量版.md` |
| **V2.3** | 2026-09-26 | **A1 数据契约版** | §5.1 接口扩为 11 方法；§6.2 补单执行者边界；§6.3 增三字段；§12.1 键表按实际配置修正。接口权威为 SYS-04 V2.4 §6.1，类型权威为 ENG-09 V2.3 |

**V2.3 未改动**：§2~§4、§7~§11、§13~§18 的全部内容 —— 包括 §7 SDK 发现与编译设计、
§8 `MultiCameraManager`、§9 `ITriggerController`、§11 转台、§13 生命周期、
§14 异常处理、§15 模块关系、§16 测试要求、§17 开发路线、§18 设计冻结总结。
