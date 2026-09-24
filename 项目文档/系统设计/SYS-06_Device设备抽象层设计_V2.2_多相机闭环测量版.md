# SYS-06_Device设备抽象层设计_V2.2（真实硬件接入版）

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


virtual bool initialize()=0;


virtual bool start()=0;


virtual void stop()=0;


virtual bool grab(
data::ImageFrame& frame
)=0;


virtual bool setTriggerMode(
bool enable
)=0;


};
```

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

```

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

包含：

```yaml id="p8m2qx"
camera_id:

camera_role:

resolution:

pixel_format:

trigger_mode:

backend_type:
```

---

注意：

backend_type属于：

系统装配配置。

不进入Data层。

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

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统 Device设备抽象层设计基线。
