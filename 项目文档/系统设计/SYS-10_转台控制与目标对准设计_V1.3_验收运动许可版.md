# SYS-10_转台控制与目标对准设计_V1.3（验收运动许可版）

> **V1.3 相比 V1.2 的变更（2026-10-21，2026-10-22 补正，转台实机验收入口与控制验证批，裁决 C-24）**
>
> 1. **§6.5**：新增**验收运动许可**（`TurntableAcceptancePermit`）门禁。许可
>    只在**独立验收工具**中、于 `initialize()` 之前由代码安装；它不是配置键，
>    生产装配没有调用路径。许可限定设备（host/port/pelco_address）、允许的
>    操作（本版仅绝对定位）、两轴角度范围、单次位移上限、执行次数与有效期。
>    未安装、未生效、过期、次数用尽、目标越界或位移超限 ⇒ **2007，不发运动帧**。
> 2. **§6.5**：许可**只管新的运动**。`stop()`、退出清理与故障收口**不受**
>    许可状态或有效期影响，仍遵守 C-23 的停止预算与三事实区分。例外是验收
>    T1 的**静止停止帧**：它是主动改变线上流量的操作，须许可显式授权
>    （`allowStopFrameAtRest`），但不消耗运动次数、不读有效期。
> 3. **§6.5 / §13**：安装许可**不改变** `WorkModeTrust`，也不把 `EchoOnly`/
>    `Unknown` 改写成“协议已确认为正常”；`ReplyNonZero`/`NoReply` 在
>    `initialize()` 仍失败，许可不能绕过。许可同样**不绕过**已锁存错误、
>    `motionUncertain` 或过期/无效样本。
> 4. **§19**：新增许可门禁的 fake transport 测试判据与变异检查记录。
>
> ⚠ **`sendPolicy` 契约不变**：生产配置仍固定 `ReadOnly`，**没有任何配置键**
> 可以打开运动。`MotionEnabled` 仍只在 fake transport 单元测试与**显式安装
> 许可**的验收工具会话中出现；验收工具不得调用 `setTestMotionGate(true)`。
>
> ⚠ **本版不授权实机运动**：许可机制只规定“软件在什么条件下才可能发帧”。
> 实际现场执行仍须另行书面授权，并按执行卡限定轴、角度、次数与停止方式。
> **软件门禁已验证 ≠ 实机运动已验证**。

> **V1.2 相比 V1.1 的变更（2026-10-19，转台可信化批，裁决 C-23）**
>
> 1. **§6.4**：样本必须**两轴同轮**；任一轮轴缺失/地址错/非有限 ⇒ 整条样本
>    `valid=false`，不得把新旧轴值拼接。新增 `sample_stale_ms` 新鲜度判据：
>    超时 ⇒ 诊断 stale、`motion=ERROR`，上层不得采集。
> 2. **§6.5**：重试与停止语义统一——`stop_retry_count` 是**首次失败后的重发
>    次数**（总次数 ≤ `1+count`），全部尝试须在 `stop_retry_window_ms` 内；
>    失败锁存 **2005** 并置 `motionUncertain`，**不自动重试**；停止帧已交付
>    但 `stop_timeout_ms` 内未观察到静止 ⇒ **2006**。新增错误码 2004～2008。
> 3. **§12**：停止判据明确为“**规定停止处理 + 观测到的静止**”，不是急停，
>    也不是协议 ACK。三件事分开表述：帧已交付 / 观察到静止 / 协议确认
>    （本批未证明，禁止使用）。
> 4. **§13**：`STABLE` 是**本轮样本**满足判据的结论，偏离、单轴失效或过期
>    即撤销；`WorkMode` 不是运动状态证据。
> 5. **§17**：`move()` 只表示**命令已受理**，不阻塞调用线程等回显；
>    worker 是唯一 I/O 所有者；`stop()` 是屏障，取消其之前的未执行命令；
>    接收切片 ≤10 ms，切片超时不等于事务超时。
> 6. **§19**：补充 C1–C4 反例对应的测试判据与变异检查要求。
>
> **本版不改**：`ITurntableController` 的四个方法签名、`TurntableCommand`、
> `TurntableMotionState` 枚举、ALIGN/STABILIZE 的状态机语义。
> ⚠ **软件修复已验证 ≠ 实机运动已验证**：本版描述的行为由 fake transport
> 单元/集成测试验证；真实运动、真实停止与丢包**尚未验证**，需另行书面授权。

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

**样本有效性（C4 / C2，V1.2 新增）**：

* 一次轮询周期**同时**查询两轴，产出一条 `TurntableSample`：
  `{azimuth, elevation, sampleTimestampNs, cycleId, valid, source}`。
* 任一轴缺失、回复地址与 `pelco_address` 不符、角度非有限 ⇒ **整条样本
  `valid=false`**；不得把新方位角与上一轮的俯仰角拼在一起发布。
* 每次轮询（无论成功与否）递增 `cycleId`；`state()` 与诊断快照发布的是
  **同一轮次**的结果。
* 样本超过 `sample_stale_ms`（默认 500）未更新，或时间戳非正/回退 ⇒
  诊断 `sampleFresh=false`，`state().motion` 对上层报 **ERROR**；上层不得
  把过期读数当作可采集状态。
* `STABLE` **不是锁存值**：已到位后读数偏离超过 `arrival_tolerance_deg`、
  任一轴失效或样本过期 ⇒ **撤销** `STABLE`；重新满足 §6.4 三条判据才可再次
  置位。

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
| 命令/角度非有限、配置非法 | **不发任何字节**，2004 |
| 只读模式（`ReadOnly`） | **所有路径**（初始化、轮询、失联、错误锁存、退出、`stop()`）控制帧恒为 0；`stop()` 为无帧 no-op |
| 停止帧在窗口内未交付 | 锁存 **2005**，置 `motionUncertain`；**不自动重试** |
| 停止帧已交付、观测窗口内未静止 | 锁存 **2006**，置 `motionUncertain`；不重发 |
| 运动门禁拒绝（只读模式、无有效验收许可） | **2007**，不发运动帧 |
| 验收许可未安装/未生效/过期/次数用尽 | 新运动 **2007**，不发运动帧；`stop()` 与退出清理**不受影响** |
| 许可目标超出两轴范围或单次位移超限 | 新运动 **2007**，不发运动帧 |
| 许可与只读策略并存 | `ReadOnly` **优先**；许可不能把只读会话变成可运动会话 |
| 许可试图绕过异常 WorkMode/锁存/过期样本 | **拒绝**（2007）；`WorkModeTrust` 保持实际值 |
| 运动看门狗到期 | **2008**，先请求停止、再锁存；首因保留 2008 |

**停止重试（V1.2 统一策略）**：

* `stop_retry_count`（默认 2，范围 0～5）表示**首次失败后的重发次数**，
  总发送次数最多 `1 + stop_retry_count`；
* 全部尝试必须在 `stop_retry_window_ms`（默认 2000）内完成，退避
  `stop_backoff_ms`（默认 200）；
* 窗口耗尽仍无法交付 ⇒ 2005 + `motionUncertain=true`。**不再自动重发**；
  只有显式 `stop()` 或成功的 `initialize()` 可以重新武装；
* 停止帧已交付 ≠ 已观察到静止：后者由连续新鲜样本在 `stop_timeout_ms`
  （默认 3000）内满足静止判据得出；
* **不自动清除** `motionUncertain`：下一次 `startMeasurement()` 不得仅因读到
  角度或模式就清锁存。

**验收运动许可（V1.3 新增，C-24）**：

* 类型为 `data::TurntableAcceptancePermit`（ENG-09 §5.35），由独立验收工具
  在 `initialize()` **之前**通过 `enableAcceptancePermit()` 安装；每个会话
  最多安装一次，不可替换。
* 许可至少限定：设备 `host`/`port`/`pelco_address`（必须与配置逐项一致）、
  允许的操作（本版仅绝对定位）、方位/俯仰目标范围、相对**当前新鲜样本**的
  单次位移上限、最大执行次数、有效期 `[valid_from, valid_until)`（墙钟）。
* 执行次数在**进入发送阶段**时扣减；被拒绝的命令不扣减。次数用尽只拒绝
  **新运动**，不锁死会话，也不影响停止与退出。
* `move()==true` 仍只表示**已受理**；许可通过不等于交付、到位或精度达标。
* 许可不是 `sendPolicy`、不是 YAML 键、不是 `WorkMode` 证据，也不得调用
  `setTestMotionGate(true)` 代替。生产装配与 `config/` 没有任何安装路径。
* 过期后**禁止新运动**，但已经开始的运动必须仍可 `stop()`；停止预算、首因
  保留与 `stopFrameDelivered`/`stopObservedStationary` 的区分不变。
* **静止停止帧（T1 专用）**：`stop()` 在静止时是无帧 no-op，不能用来证明
  “停止帧能发出”。许可另设 `allowStopFrameAtRest`，由验收工具的
  `stop_at_rest` 命令经 `sendAcceptanceStopFrameAtRest()` 在**静止**状态
  实际发送一帧 `0x00`。该操作只在以下条件全部满足时执行：已安装许可且
  标志为 1、非只读、当前静止（同轮新鲜有效双轴样本、无待观测停止、无锁存
  故障/不确定）、本次会话尚未交付过停止帧。它**不消耗**运动次数、**不读**
  有效期（属于停止/安全收口，不是新运动），执行后由既有停止观测流程给出
  `stopObservedStationary`；不自动重复补帧。T1 必须是本操作，不得先运动
  再以 `stop()` 记作 T1。

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

## 12.1 视觉对准的停止判据

目标满足：

```text id="h8m3qv"
|pixelX| ≤50 且 |pixelY| ≤50
```

同时转台 `state().motion == STABLE`（§6.4 的样本判据）且无锁存错误。
`STABLE` 只由**本轮新鲜、两轴同轮的样本**得出；过期/无效样本不得维持它。

## 12.2 停止动作的判据（V1.2 新增）

**停止**指“发起规定的停止处理，并记录已知结果”，不是急停，也不是协议 ACK。
三件事必须分开表述，字段名不得混用：

| 事实 | 判据 | 字段 |
|---|---|---|
| 停止帧已交给传输层 | `send()` 对停止帧成功 | `stopFrameDelivered` |
| 在规定观测窗口内观察到静止 | 连续新鲜样本满足静止判据 | `stopObservedStationary` |
| 协议层面的制动/停止确认 | 需要设备明确反馈 | **本批未证明，禁止使用** |

无法观察到静止时，任务仍明确失败，同时保留 `motionUncertain=true`。
**不得**写成“已让机构停住”，也不得把 `stopFrameDelivered` 当作“已停止”。

读取路径：`ITurntableDiagnostics::diagnostics()`（并列接口，不改变
`ITurntableController` 的四个方法）；结果由 `MeasurementController` 写入
`MeasurementRecord.stopRequested / stopFrameDelivered / stopObservedStationary /
motionUncertain` 并随结果包落盘。

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

**V1.2 补充（C-23）**：

* `STABLE` 是**本轮样本**满足 §6.4 判据的结论，不是锁存值；偏离、单轴失效
  或样本过期即撤销。
* 样本过期、无效或时间戳异常时，`state().motion` 对上层报 **ERROR**，
  即使内部保存的最后一个状态曾是 `STABLE`。
* `WorkMode` **不是** `MOVING`/`IDLE` 的证据。它只有四类可信度：
  `Unknown` / `EchoOnly`（回复与查询逐字节相同）/ `ReplyNonZero` /
  `NoReply`；**不设** `ConfirmedNormal`。本批没有可放行运动的协议证据，
  WorkMode 不构成运动许可。
* `stopFrameDelivered` / `stopObservedStationary` / `motionUncertain` 是
  **并列的诊断事实**，不进入 `TurntableMotionState` 枚举；`motion` 的语义
  保持 C-04 冻结的四个值。

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
* **单所有者 worker（V1.2 选定方案，不是唯一方案）**：worker 是唯一 I/O
  所有者；`move()` 在锁下写入“最新运动命令”单槽（带单调序号）后立即返回，
  **不在调用线程等待回显**；发送失败、无回显、被新命令覆盖、被停止取消，
  都通过命令序号与诊断快照给出明确终结结果（`None/Delivered/Superseded/
  Cancelled/Failed`）；
* `stop()` 分配更高的停止序号、清空待执行运动槽，并等待 worker 处理到该
  屏障；屏障之前的旧命令必须作废，停止后**不得恢复执行**；
* 接收切成 **≤10 ms** 的片段；**单个切片超时只是继续轮询**，不得被当成整个
  事务超时；
* `state()` **只读快照，不做 I/O、不拿 I/O 锁**。

各调用的阻塞上限：

| 调用 | I/O | 最坏阻塞 | 调用频率 |
|---|---|---|---|
| `initialize()` | 是 | 约 3 次查询 × 50 ms × 3 尝试 | 启动一次 |
| `move()` | 否（仅入队） | 锁竞争量级（**不等回显**） | ALIGN 下发时 |
| `state()` | 否 | < 1 ms | 每 16 ms tick |
| `stop()` | 等待 worker 到达停止屏障 | `stop_timeout_ms`（默认 3000 ms） | 幂等 |
| I/O 线程 `pollOnce()` | 是 | 约 100 ms（两轴都超时） | 默认 20 Hz |

⚠ **停止时延上界**只在“一次发送 + 一个接收切片 + 实测调度/锁竞争”的条件下
成立；本版**不预先声称** `< 1 ms`，实测值另行登记。

线程生命周期：`initialize()` 成功后才启动 I/O 线程；关闭时先停止新的命令
入口，再 `stop()`（请求停止并等待屏障），随后 `join` worker，最后关闭
socket。目标总耗时 < 1 s。

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

## 真实后端（fake transport）测试（V1.2 新增，C-23）

不连接真实 UDP 地址，以可注入传输桩验证：

| 反例 | 判据 |
|---|---|
| C1 只读失联却发停止帧 | `ReadOnly` 在正常、失联、错误锁存、退出四条路径上控制帧计数恒为 0；失联后的失败收口（上层 `stop()`）同样为 0 |
| C2 到位后偏离仍保持 STABLE | 偏离、单轴失效、样本过期三种序列均**撤销** `STABLE`；上层快照报 ERROR 或不采集 |
| C3 错地址回复仍初始化成功 | 工作模式、水平角、俯仰角三类回复分别以错地址注入，均不能完成事务，不能更新样本 |
| C4 两轴跨轮拼接 | 单轴缺失/错地址时整条样本 `valid=false`，不得把旧轴值配新值；`cycleId` 递增 |

另需覆盖：NaN/Inf 与非法配置返回 2004 且发送计数为 0；停止 2005/2006；
重试总次数 ≤ `1+stop_retry_count`；显式 `stop()` / 成功重新初始化才重新武装；
看门狗 2008；锁存后保留首因。

⚠ **变异检查**：每项修复须先改回错误实现、确认**预期断言**变红，再用编辑
恢复（禁止用文件复制恢复）。记录“哪条断言因哪个变异变红”。

## 验收运动许可测试（V1.3 新增，C-24）

不连接真实 UDP 地址，以 fake transport 验证：

| 反例 | 判据 |
|---|---|
| 只把 `sendPolicy` 设为 `MotionEnabled`、未装许可 | `move()` 拒绝，控制帧 0 |
| 许可设备与配置不一致 | 安装失败，**打开 socket 之前**返回；控制帧 0 |
| 许可过期/未生效/次数用尽 | 新运动 2007、控制帧不增加；已开始的运动仍可停止，停止帧计数正确 |
| 目标越界或位移超过许可 | 2007、控制帧 0；边界内对照命令仍可受理 |
| 许可试图绕过只读/锁存/过期样本 | 仍拒绝；`WorkModeTrust` 不被改写 |
| 停止被误纳入许可门禁 | 过期后停止仍**必须**发出（该变异必须使断言变红） |
| 无许可 / 未授权 `allow_stop_at_rest` | `sendAcceptanceStopFrameAtRest()` 拒绝，停止帧 0 |
| 只读会话 + 已装许可 + 已授权静止停止 | 仍拒绝，控制帧 0（只读优先） |
| 静止且已授权 | 恰好发出 1 帧 `0x00`，`stopFrameDelivered=true`；轮询观测静止后再次请求**不再补帧** |
| 运动中请求静止停止 | 拒绝；应改用 `stop()`，不得借此多发一帧 |
| 样本过期 / 故障锁存 | 拒绝，零帧 |
| 许可过期后请求静止停止 | 允许（停止/安全收口，不是新运动），仍只发 1 帧 |

另需覆盖：许可缺少必需键/重复键/非有限值在**工具解析阶段**失败且不发帧；
`max_executions` 在发送阶段扣减，被拒绝命令不扣减；`allow_stop_at_rest`
是**必需键**（缺失即解析失败），静止停止帧不扣减执行次数。

## Hardware测试

验证：

* SDK；
* 网络；
* RS485；
* 实际运动。

⚠ **本批未执行真实运动/停止**：Hardware 的“实际运动”仍是未验证项，需另行
书面授权；只读实机会话的控制帧必须为 0。

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
| 样本有效性（两轴同轮/新鲜度） | 冻结（C-23） |
| 停止三事实（交付/观测/ACK） | 冻结（C-23；ACK 未证明，禁用） |
| 只读发送门禁 | 冻结（C-23；生产配置不可打开运动） |
| 验收运动许可 | 冻结（C-24；仅独立验收工具安装，非配置键；只管新运动） |

---

## 20.1 版本与修订记录

| 版本 | 日期 | 变更 |
|---|---|---|
| V1.0 | — | 初版基线 |
| V1.1 | 2026-10-19 | 裁决 C-22：后端改名 `PelcoDTurntableController`、协议明确为 Pelco-D/UDP；新增 §6.3～§6.6（帧格式、角度/速度/到位判据、失败策略、禁止指令）；§13 改用 C-04 的 `motion` 枚举；§17 改为后端内部 I/O 线程；§18 增加 `state()` 不做 I/O 的 tick 约束 |
| V1.2 | 2026-10-19 | 裁决 C-23：§6.4 样本两轴同轮 + `sample_stale_ms` + STABLE 撤销；§6.5 统一停止重试与 2004～2008；§12 停止三事实分离；§13 WorkMode 四态、STABLE 非锁存；§17 `move()` 只受理、worker 单所有者、停止屏障、≤10 ms 切片；§19 补 C1–C4 与变异检查 |
| V1.3 | 2026-10-21（2026-10-22 补正） | 裁决 C-24：§6.5 新增验收运动许可（设备/操作/范围/位移/次数/有效期；只拒新运动，不拦停止）；**补正**：T1 静止停止帧须 `allowStopFrameAtRest` 显式授权，`stop()` 静止时仍无帧，该帧不消耗次数、不读有效期；§13 许可不改写 WorkMode 可信度；§19 补许可与静止停止帧测试/变异判据；§20 冻结总结补登。⚠ 生产只读门禁与 `sendPolicy` 契约不变；本版不构成实机运动授权 |

⚠ **阶段口径**：本批为“转台可信化批”，**不声称**第一阶段或第二阶段完成；
PH-01 的阶段归属仍待裁决。**软件修复已验证**与**实机运动尚未验证**分列，
不得互相替代。

---

本文档作为 AircraftPoseSystem V2.1 视觉闭环转台控制设计基线。
