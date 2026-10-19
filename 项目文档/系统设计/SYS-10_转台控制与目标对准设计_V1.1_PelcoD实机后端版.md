# SYS-10_转台控制与目标对准设计_V1.1（PelcoD实机后端版）

> **V1.1 相比 V1.0 的变更（2026-10-19，Pelco-D/UDP 转台后端接入批，裁决 C-22）**
>
> 1. **§4 / §5.1 / §6.1**：`PekoTurntableController` → **`PelcoDTurntableController`**；
>    协议名改为 **Pelco-D**，传输改为**以太网 UDP**。§4 架构图补出后端内部的
>    I/O 线程；§5.1 补出“网络协议 = Pelco-D over UDP”，接口方法面不变。
> 2. **新增 §6.3～§6.6**：真实后端的帧格式、角度映射、速度模式、到位判据、
>    失败策略与禁止指令。帧结构的可追溯依据见
>    `AircraftPoseSystem/evidence/转台_PelcoD_v1/`（厂商 PDF 不入库）。
> 3. **§13**：把 V1.0 的旧 `TurntableState`（`bool moving`/`bool error`）改为
>    ENG-09 裁决 C-04 冻结的 `motion` 枚举，消除文档与代码的旧冲突。
> 4. **§17**：明确 `TurntableWorker` 的职责由**后端内部 I/O 线程**承担，
>    本批不新增独立 Worker 类；`state()` 只读快照。
> 5. **§18**：补充“`state()` 不做 I/O”的 tick 约束。
> 6. **§20**：新增版本与修订记录。
>
> **本版不改**：`ITurntableController` 的方法面、`TurntableCommand`、
> `TurntableMotionState` 的枚举值、ALIGN/STABILIZE 的状态机语义。
> PH-01 的阶段归属仍待裁决；本批是“实机后端接入验证批”，不声称阶段完成。

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

TurntableWorker（逻辑职责）

        ↓

ITurntableController

        ↓

Turntable Backend

   ┌────┴────┐
   │ I/O 线程 │  ← 真实后端内部：轮询角度、收发指令
   └────┬────┘
        ↓

Hardware

```

⚠ **V1.1 实现约定**：`TurntableWorker` 在代码中**没有独立类**；其“输入
`TurntableCommand` / 输出 `TurntableState`”的职责由 `PelcoDTurntableController`
内部的 I/O 线程承担。`state()` 只返回快照，调用方仍按 §17 的输入/输出语义使用。

---

# 5 ITurntableController接口

## 5.1 职责

屏蔽：

* SDK；
* 网络协议（本批为 **Pelco-D over UDP**）；
* RS485协议（本批未实现）。

⚠ 接口方法面在 V1.1 **不变**：`initialize()` / `move()` / `state()` / `stop()`。
`move()` 的返回值仍只表示“命令已被受理”，不表示到位；是否到位只看
`state().motion`。

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

## 6.1 PelcoDTurntableController

真实设备实现（裁决 C-22：原 `PekoTurntableController` 改名）。

负责：

* **Pelco-D 协议**（标准 Pelco-D + 厂商自定义扩展）；
* **以太网 UDP** 通信（本批）；
* SDK 调用（本批不需要厂商 SDK）；
* RS485 通信（本批未实现）。

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

## 6.3 Pelco-D 帧格式（UDP）

7 字节帧：

```text
0xFF | 地址 | Data1 | Data2 | Data3 | Data4 | 校验码
```

校验码 = `(地址 + Data1 + Data2 + Data3 + Data4) & 0xFF`，**不含起始位**。

⚠ **指令码位置不是统一的**：

* **标准 Pelco-D** 指令码放 **Data2**，Data1 = `0x00`：
  `0x51`/`0x53` 查询水平/垂直角，`0x4B`/`0x4D` 绝对定位，`0x00` 停止；
* **厂商自定义** 指令码放 **Data1**：
  `0xE0` 工作模式、`0xC5` 到位反馈、`0xE1` 实时角度等。

角度编码：×100 取整，高字节在前。水平为**无符号 0～360°**，
垂直为**有符号 16 位**。

## 6.4 角度、速度与到位判据

**水平映射**（设备 0～360 ↔ 软件 −180～180）：

```text
设备→软件：az = raw/100; sw = (az > 180) ? az - 360 : az
软件→设备：d = fmod(sw, 360); if (d < 0) d += 360; raw = round(d*100)
```

⚠ 软件零点 = 设备零点；本批不做零位偏移。`180.00` 与 `−180.00` 是同一物理点，
设备读回为 `+180`。

**垂直**：有符号，直接使用，无环绕。

**速度**：设备单位为 **r/min**，指令值放大 10 倍。换算
`rpm10 = round(degPerSec * 10 / 6)`（1 r/min = 6 °/s）。

* `use_speed_control=false`（默认）：表 3.5，使用设备内部默认速度；
* `use_speed_control=true`：表 3.65，指令携带转速。

不做自动回退：设备拒绝（`EE EE EE EE`）即报失败并记录。

**到位/STABLE** 必须同时满足：

1. 两轴 `|current − target| ≤ arrival_tolerance_deg`；
2. 连续 `stable_read_count` 个轮询周期，两轴读数变化都
   `≤ stable_threshold_deg`；
3. 每个周期两轴都有有效回复。

⚠ **不得**用 `0xC5` 到位回传判断实际位置（它回的是指令目标角），
**不得**用 `0xDD` 的“刹车”标志判断稳定（实测角度仍在变化时也可能报刹车）。
本批也不启用 `0xE1` 实时角度流。

## 6.5 失败与重试

| 事件 | 处置 |
|---|---|
| 初始化查询超时/`FF FF FF FF` | 重试 2 次；仍失败 ⇒ 初始化失败，2001 |
| 移动回显完全一致 | 接受，置 MOVING，启动看门狗 |
| `FF FF FF FF` | 重试；用尽 ⇒ `move()` 失败，2001 |
| `EE EE EE EE` | **不重试**，`move()` 失败，2001 |
| 单次轮询一轴丢包 | 下一周期再查，不立即 ERROR |
| 连续 `comm_failure_threshold` 周期无有效回复 | 置 ERROR（2001）并锁存 |
| MOVING 超看门狗 | 置 ERROR，尽力 stop（2001） |
| 本地超行程 | **不发帧**，2002，置 ERROR |
| 部分轴接受、部分失败 | 立即尽力 stop，返回 false（2001） |
| WorkMode 非 `0x00` | 初始化失败，不自动清故障/逃逸限位 |

看门狗：`预期行程时间 × 1.5 + 5 s`，上限 600 s。`use_speed_control=false` 时
按 12 °/s 保守估计。

## 6.6 禁止发送的指令

* `0xC9` 自检；
* `0xDE` 重启；
* `0xBD` 转圈/方向控制；
* 扫描、预置位、限位逃逸类指令。

这些指令可能在现场造成**非预期的大范围运动**，本批不实现、不发送。

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

⚠ **V1.1 更正**：V1.0 本节的 `bool moving` / `bool error` 与裁决 C-04 冻结的
`TurntableState` 不一致，现按冻结定义更正。

`TurntableState`（ENG-09 §4.1，裁决 C-04）：

```cpp
struct TurntableState
{
    double azimuth;                  // deg
    double elevation;                // deg
    TurntableMotionState motion;     // IDLE | MOVING | STABLE | ERROR
};
```

`TurntableMotionState` 的取值：

| 取值 | 含义 |
|---|---|
| `IDLE` | 未收到运动指令 |
| `MOVING` | 已受理运动指令，未满足到位判据 |
| `STABLE` | 按 §6.4 的到位判据判定到位且读数稳定 |
| `ERROR` | 通信中断、超时、超行程或设备拒绝，锁存到下一次成功动作 |

`moving` 不再单独存在：`motion != IDLE && motion != STABLE` 与旧 `moving`
语义等价（C-04）。`state().motion` 是判“是否到位”的唯一依据，
`move()` 的返回值只表示命令是否被受理。

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

⚠ **V1.1**：`TurntableWorker` 在代码中**没有独立类**（见 §4）。
它的职责由真实后端 `PelcoDTurntableController` 内部的 **I/O 线程**承担：

* I/O 线程按 `poll_interval_ms`（默认 50 ms）轮询两轴角度，更新快照；
* 逻辑输入仍是 `TurntableCommand`（经 `move()` 交给 I/O 线程），
  逻辑输出仍是 `TurntableState`（`state()` 返回快照）；
* 串行化：`move()` / `stop()` / I/O 线程的轮询共用 `ioMutex_`，
  避免运动回显被轮询抢走；
* `state()` **只读快照，不做 I/O、不拿 `ioMutex_`**。

各调用的阻塞上限：

| 调用 | I/O | 最坏阻塞 | 调用频率 |
|---|---|---|---|
| `initialize()` | 是 | 约 3 次查询 × 50 ms × 3 尝试 | 启动一次 |
| `move()` | 是（等回显） | `reply_timeout_ms` 的 3 倍（默认 150 ms） | ALIGN 下发时 |
| `state()` | 否 | < 1 ms | 每 16 ms tick |
| `stop()` | 仅在有未完成动作/ERROR 时发送 | < 1 ms | 幂等 |
| I/O 线程 `pollOnce()` | 是 | 约 100 ms（两轴都超时） | 默认 20 Hz |

线程生命周期：`initialize()` 成功后才启动 I/O 线程；析构时置
`running_ = false`、通知条件变量、`join`，必要时发停止帧，再关闭 socket。
目标总耗时 < 1 s。

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

⚠ **V1.1 tick 约束**：

* 主循环 tick 为 **16 ms**（`main.cpp`），ALIGN / STABILIZE 两个状态
  **每个 tick 都会调用 `state()`**；
* 因此 `state()` **不得做任何 I/O**（不得阻塞等待回包、不得拿 I/O 锁）；
  它只拷贝一个已发布的快照。所有网络收发都在后端 I/O 线程内完成；
* `move()` 只在状态机下发新指令时调用，不进入每 tick 路径；
* 若把查询放进 `state()`，一次 50 ms 超时会让主循环卡 3 个 tick，
  并连带拖慢 Preview / Algorithm / Recorder —— 本版明确禁止。

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
| 控制线程      | 后端内部 I/O 线程（无独立 Worker 类） |
| 真实协议      | Pelco-D over UDP（SDK / RS485 未实现） |
| Virtual设备 | 保留        |
| 算法禁止直控转台  | 冻结        |

---

## 20.1 版本与修订记录

| 版本 | 日期 | 变更 |
|---|---|---|
| V1.0 | — | 初版基线 |
| V1.1 | 2026-10-19 | 裁决 C-22：后端改名 `PelcoDTurntableController`、协议明确为 Pelco-D/UDP；新增 §6.3～§6.6（帧格式、角度/速度/到位判据、失败策略、禁止指令）；§13 改用 C-04 的 `motion` 枚举；§17 改为后端内部 I/O 线程；§18 增加 `state()` 不做 I/O 的 tick 约束 |

⚠ **阶段口径**：本批为“实机后端接入验证批”，**不声称**第一阶段或第二阶段完成；
PH-01 的阶段归属仍待裁决。

---

本文档作为 AircraftPoseSystem V2.1 视觉闭环转台控制设计基线。
