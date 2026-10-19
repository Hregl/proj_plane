# SYS-10_转台控制与目标对准设计_V1.0（视觉闭环控制版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 中转台控制与目标视觉对准设计。

目标：

实现：

```text id="u7n4pc"
目标检测

↓

目标偏差计算

↓

视觉闭环控制

↓

转台运动

↓

目标居中

```

完成：

* 方位轴控制；
* 俯仰轴控制；
* 视觉闭环对准；
* 转台状态监测；
* 控制接口适配。

---

# 2 系统功能概述

转台系统用于：

* 初始目标搜索；
* 目标居中；
* 姿态测量辅助；
* 视觉跟踪。

---

系统最终目标：

目标中心保持：

```text id="k4m8qx"
±50 pixel
```

范围内。

---

# 3 转台硬件需求

## 3.1 运动范围

目标：

### 方位轴

```text id="h9m3xw"
360°
```

---

### 俯仰轴

```text id="p5k7vz"
-60° ~ +60°
```

---

# 3.2 运动速度

参考指标：

## 方位

```text id="j7n2qs"
1.2°/s ~ 60°/s
```

---

## 俯仰

```text id="w4m8pk"
1.2°/s ~ 30°/s
```

---

# 3.3 控制精度

目标：

```text id="z8q3mv"
±0.2°
```

---

# 4 转台软件架构

结构：

```text id="m5k9qx"
AlignmentController

        ↓

TurntableWorker

        ↓

ITurntableController

        ↓

Turntable Backend

        ↓

Hardware

```

---

# 5 ITurntableController接口

## 5.1 职责

屏蔽：

* SDK；
* 网络协议；
* RS485协议。

---

接口：

```cpp id="v7m3kp"
class ITurntableController
{

public:


virtual ~ITurntableController()=default;


virtual bool initialize()=0;


virtual bool move(
const data::TurntableCommand&
command
)=0;


virtual data::TurntableState state()=0;


virtual void stop()=0;


};
```

---

# 6 转台后端实现

## 6.1 PekoTurntableController

真实设备实现。

负责：

* SDK调用；
* Peko_D协议；
* 网络通信；
* RS485通信。

---

## 6.2 VirtualTurntable

用于：

* 软件测试；
* 无硬件开发。

模拟：

* 方位变化；
* 俯仰变化；
* 运动状态。

---

# 7 AlignmentController设计

## 7.1 功能

根据目标图像偏差生成转台控制量。

---

输入：

```cpp id="q3k8mz"
TargetOffset
```

包含：

```cpp id="x8m2vp"
pixelX

pixelY

centered
```

---

输出：

```cpp id="m6p9qv"
TurntableCommand
```

包含：

```cpp id="n4x7cz"
azimuth

elevation
```

---

# 8 视觉闭环控制流程

完整流程：

```text id="g5n8mq"
Camera

↓

TargetDetector

↓

TargetOffset

↓

AlignmentController

↓

TurntableCommand

↓

Turntable

↓

Camera

```

---

形成闭环。

---

# 9 像素误差到角度转换

## 9.1 输入

目标中心：

```text id="b7m2qp"
(cx,cy)
```

图像中心：

```text id="q8n3mv"
(cx0,cy0)
```

---

偏差：

$$
\Delta x=c_x-c_{x0}
$$

$$
\Delta y=c_y-c_{y0}
$$

---

# 9.2 角度转换

根据当前焦距：

$$
\theta_x=
atan(
\frac{\Delta x \times pixelSize}{f}
)
$$

$$
\theta_y=
atan(
\frac{\Delta y \times pixelSize}{f}
)
$$

---

输出：

```text id="r6m8pk"
方位修正角

俯仰修正角
```

---

# 10 控制策略

## 10.1 粗对准

大偏差：

采用：

快速转动。

---

特点：

* 高速度；
* 低精度。

---

## 10.2 精对准

小偏差：

采用：

低速调整。

---

特点：

* 低抖动；
* 高稳定。

---

# 11 控制模式

## 11.1 Search模式

用于：

目标搜索。

特点：

* 扫描；
* 大范围运动。

---

## 11.2 Alignment模式

用于：

目标居中。

特点：

* 闭环控制；
* 小范围调整。

---

## 11.3 Stabilize模式

用于：

测量前稳定。

检查：

* 转台停止；
* 图像稳定。

---

# 12 停止判据

目标满足：

```text id="h8m3qv"
|pixelX| ≤50

|

pixelY| ≤50
```

---

同时：

转台：

* 速度接近0；
* 无误差状态。

---

# 13 转台状态管理

TurntableState：

包含：

```cpp id="y4m7qp"
struct TurntableState
{

double azimuth;


double elevation;


bool moving;


bool error;

};
```

---

# 14 异常处理

## 14.1 通信异常

包括：

* SDK断开；
* 网络异常；
* RS485失败。

处理：

进入：

FAILED。

---

## 14.2 超限异常

检测：

* 方位超限；
* 俯仰超限。

---

## 14.3 控制失败

记录：

* ErrorCode；
* ErrorInfo；
* FailureTrace。

---

# 15 与状态机关系

状态：

```text id="m3q8vz"
TARGET_FOUND

↓

ALIGN

↓

STABILIZE

```

调用：

AlignmentController。

---

# 16 与视觉算法关系

Algorithm输出：

```text id="p7k2xm"
TargetOffset
```

---

Application负责：

调用：

```text id="q6m8px"
AlignmentController
```

---

禁止：

Algorithm直接控制转台。

错误：

```text id="u5n8qm"
PnP

↓

Turntable
```

---

# 17 多线程关系

转台独立线程：

```text id="c8m4qp"
TurntableWorker
```

---

输入：

```text id="r7n2mv"
TurntableCommand
```

---

输出：

```text id="h5m8qx"
TurntableState
```

---

# 18 性能要求

闭环目标：

* 快速发现目标；
* 快速居中；
* 稳定后测量。

---

要求：

对准过程：

不影响：

* Preview；
* Algorithm；
* Recorder。

---

# 19 测试要求

## VirtualTurntable测试

验证：

* 指令执行；
* 状态变化；
* 限位。

---

## AlignmentController测试

验证：

输入：

不同pixel偏差。

输出：

正确控制方向。

---

## Hardware测试

验证：

* SDK；
* 网络；
* RS485；
* 实际运动。

---

# 20 设计冻结总结

| 项目        | 状态        |
| --------- | --------- |
| 转台接口      | 冻结        |
| 视觉闭环      | 冻结        |
| 目标居中      | ±50 pixel |
| 控制线程      | 独立        |
| 真实协议      | SDK适配     |
| Virtual设备 | 保留        |
| 算法禁止直控转台  | 冻结        |

---

本文档作为 AircraftPoseSystem V2.1 视觉闭环转台控制设计基线。
