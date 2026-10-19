# SYS-09_多线程并发设计_V2.2（转台单所有者线程版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 多线程并发架构设计。

目标：

实现：

* 三相机并行采集；
* 硬触发同步；
* Preview实时显示；
* 算法异步计算；
* 转台独立控制；
* 数据异步保存；

满足：

```text id="t9x6pz"
实时性

+

稳定性

+

可扩展性
```

---

# 1.1 V2.2 修订性质

V2.2 只同步**转台 worker** 的并发契约（裁决 **C-23**），其余线程模型
（Camera／Trigger／Synchronizer／Preview／Algorithm／Recorder）**一字未动**。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | §10 | 明确**单所有者 I/O worker**：`move()` 只把命令写入单槽并立即返回，所有协议收发只在 worker 线程发生；新命令覆盖旧命令时给出**终结结果**（`Superseded`／`Cancelled`／`Failed`） | 语义澄清 |
| 2 | §10 | 新增**停止屏障**与 ≤10 ms 接收切片：`stop()` 分配更高的停止序号，worker 在发送前、轴切换前、每个接收切片后检查；停止屏障之前的旧命令作废 | 新增 |
| 3 | §16 | 关闭顺序按 **SYS-06 V2.8 §13** 与 `SystemInitializer::shutdown()` 的**实际实现**对齐：任务收口 → 转台 stop/join → 预览 stop/join → 触发器 → 相机 | 一致性修正 |

⚠ **不宣称“单所有者 worker 是唯一可行方案”**：它是本工程的**选定方案**，
理由是协议会话的收发状态（回显匹配、事务序号、停止优先级）只能有一个所有者，
否则需要额外的锁层级；这不排除其他实现，只是本实现的取舍。
⚠ 时延上界**不预先声称**：切片 ≤10 ms 只约束“单次接收等待”的粒度，
不构成端到端停止时延保证；实测值以 SYS-10 V1.2 §17 登记为准。
⚠ 本版**不新增**任何线程，也不改变既有线程的创建/销毁责任。

---

# 2 多线程设计原则

## 2.1 线程隔离

不同任务独立线程：

避免：

* UI阻塞；
* 算法阻塞；
* 硬件等待阻塞。

---

## 2.2 数据驱动

线程之间：

不直接调用内部函数。

采用：

* Queue；
* Signal/Slot；
* shared_ptr数据对象。

---

## 2.3 生命周期统一管理

所有线程：

由：

```text id="h7m4cx"
ApplicationContext

↓

ThreadManager
```

创建和释放。

---

# 3 总体线程架构

系统线程：

```text id="n4k8vx"
                Main Thread

                    |

                    |

               MainWindow


================================================


Camera25Worker

Camera50Worker

Camera100Worker


          |

          |

   MultiCameraManager


          |

          |

 SynchronizerWorker


          |

          |

  MultiCameraFrame


          |

          |

 AlgorithmWorker


          |

          |

 PoseResult


          |

          |

 RecorderWorker



------------------------------------------------


PreviewWorker


------------------------------------------------


TriggerWorker


------------------------------------------------


TurntableWorker

```

---

# 4 主线程（UI Thread）

## 4.1 职责

负责：

* Qt事件循环；
* 用户操作；
* 界面刷新。

---

不负责：

* 图像采集；
* 算法计算；
* 文件保存。

---

# 5 CameraWorker设计

---

# 5.1 Camera25Worker

职责：

管理：

25mm相机。

输出：

```text id="a6v9qp"
ImageFrame(CAM25)
```

---

用途：

* 搜索；
* Preview；
* 近距离测量。

---

# 5.2 Camera50Worker

职责：

管理：

50mm相机。

输出：

```text id="k7x2pm"
ImageFrame(CAM50)
```

---

用途：

中距离测量。

---

# 5.3 Camera100Worker

职责：

管理：

100mm相机。

输出：

```text id="m5q8vz"
ImageFrame(CAM100)
```

---

用途：

远距离高精测量。

---

# 6 TriggerWorker设计

## 6.1 职责

管理：

硬件触发。

流程：

```text id="w4n9pc"
TriggerWorker

↓

TriggerController

↓

Camera Exposure

↓

Frame

```

---

## 6.2 要求

必须保证：

* 触发顺序；
* 触发周期；
* 触发状态。

---

# 7 SynchronizerWorker设计

## 7.1 职责

组合三相机帧。

输入：

```text id="v8m2yh"
CAM25 Frame

CAM50 Frame

CAM100 Frame
```

---

输出：

```text id="r7x5nc"
MultiCameraFrame
```

---

# 7.2 同步判据

检查：

* frameId；
* timestampNs；
* exposureIndex。

---

# 8 PreviewWorker设计

## 8.1 职责

实时显示。

数据：

```text id="j8q2vx"
Camera

↓

PreviewQueue

↓

PreviewWorker

↓

PreviewManager

↓

Qt
```

---

## 8.2 特点

允许：

丢帧。

策略：

```text id="m6p8yz"
丢旧保新
```

---

# 9 AlgorithmWorker设计

## 9.1 职责

执行：

完整姿态算法。

流程：

```text id="x3n7mc"
MultiCameraFrame

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

输出：

```text id="q9m4vx"
ShipPoseResult
```

---

# 10 TurntableWorker设计

## 10.1 职责

独立控制转台。

输入：

```text id="w8k3pq"
TurntableCommand
```

---

输出：

```text id="z6y2mt"
TurntableState
```

---

流程：

```text id="a3k7px"
AlignmentController

↓

TurntableWorker

↓

ITurntableController

```

---

## 10.2 单所有者 I/O 与命令受理（V2.2 新增，C-23）

* **唯一的协议收发所有者**是转台 worker（真实后端内部线程）。
  `move()` **不在调用线程等待回显**：它在 `commandMutex_` 下把命令写入
  **单槽**（带单调命令序号）后立即返回；`move()==true` 只表示**命令已受理**，
  不表示已发送、更不表示已到位。
* 发送、回显匹配、轮询、停止全部只在 worker 线程发生。调用方（UI 线程、
  `AlignmentController`）**不得**直接调用传输层的收发。
* 每条命令有唯一的**终结结果**：`Delivered`（已交给传输层）、
  `Superseded`（被更新的运动命令覆盖）、`Cancelled`（被停止屏障取消）、
  `Failed`（发送失败/事务失败）。终结结果进入
  `TurntableDiagnostics`（§5.34），调用方**不得**用“没有回显”反推“没有执行”。
* 被覆盖或被取消的旧命令**不得**在停止之后恢复执行。

## 10.3 停止屏障与接收切片（V2.2 新增，C-23）

* `stop()` 分配**更高的停止序号** `stopSeq`，清空待执行运动槽，然后等待
  worker 处理到该停止（上限 `stop_timeout_ms`）。
* worker 在以下位置检查 `stopSeq`：**每次发送前**、**每轴切换前**、
  **每个接收切片后**。屏障之前接受的旧命令一律作废。
* 后端/传输层的单次接收等待切成 **≤10 ms** 的片段；
  **单个切片超时只表示“本片没有数据”，继续等待整个事务的预算**，
  不得把一次切片超时当作事务失败，也不得让停止等待被一次长 `receive()` 拖住。
* ⚠ 以上是**粒度约束**，不是端到端时延承诺；实测条件与数值见
  SYS-10 V1.2 §17。worker 的终止/`join()` 是对象生命周期的一部分：
  在 join 完成前不能销毁传输对象、sink 或 `Logger`。

---

# 11 RecorderWorker设计

## 11.1 职责

异步保存。

输入：

```text id="g5p8wx"
MeasurementRecord
```

---

保存：

* 原图；
* 结果；
* 标定；
* 日志；
* 配置。

---

# 12 Queue设计

---

# 12.1 PreviewQueue

用途：

实时显示。

参数：

建议：

```text id="k4x8vz"
capacity=3~5
```

---

满载：

删除旧帧。

---

# 12.2 MeasurementQueue

用途：

正式测量。

要求：

* 不随意丢弃；
* 保证数据完整。

---

# 12.3 AlgorithmQueue

用途：

算法任务。

输入：

```text id="p8m3qw"
MultiCameraFrame
```

---

# 12.4 RecorderQueue

用途：

保存。

特点：

异步。

---

# 13 数据所有权设计

## 13.1 图像

采用：

```cpp id="y6q2mr"
shared_ptr<ImageFrame>
```

原因：

减少大图复制。

---

## 13.2 多相机数据

采用：

```cpp id="s7m9cx"
shared_ptr<MultiCameraFrame>
```

---

## 13.3 姿态结果

采用：

值传递。

---

# 14 QThread实现规范

采用：

```text id="f8m2zp"
QObject Worker

+

QThread
```

---

禁止：

```cpp id="c5v7mq"
class Worker:

public QThread
```

---

原因：

* 生命周期清晰；
* 易测试；
* 信号槽明确。

---

# 15 线程启动流程

启动：

```text id="p4n8wy"
main.cpp

↓

ApplicationContext

↓

Create Worker

↓

moveToThread()

↓

connect()

↓

thread.start()

```

---

# 16 线程关闭流程

关闭（V2.2 按 `SystemInitializer::shutdown()` 的实际顺序对齐；
设备段的权威描述见 **SYS-06 V2.8 §13**）：

```text id="k3v7mh"
Stop Measurement（任务收口，幂等；内部请求一次转台停止）

↓

Stop Turntable（stop() → 销毁 → join worker / 关闭 socket）

↓

Stop Preview（stop() → join）

↓

Stop Trigger（disable → 销毁）

↓

Stop Camera（诊断回调失效 → stop → close）

↓

thread.wait()（其余 worker）

↓

Release

```

⚠ **顺序理由**：转台必须先于相机/触发器销毁，因为它的 worker 持有
独立的 socket 与 sink；预览 worker 必须先于 `PreviewManager` 与相机销毁，
否则线程可能访问已释放对象。`stop()` 在只读策略下是**无帧 no-op**，
但仍是关闭流程的必需一步（它同时负责锁存/清除状态并终止 worker 循环）。
⚠ 关闭责任方是 `SystemInitializer::shutdown()`（公开、幂等）；
`ApplicationContext::shutdownDevices()` 只负责设备段，不能替代它。

---

# 17 异常处理

## 17.1 相机异常

流程：

```text id="m8q5nx"
CameraWorker

↓

DeviceManager

↓

Application

```

---

## 17.2 算法异常

流程：

```text id="w2p6zr"
AlgorithmWorker

↓

ErrorInfo

↓

StateMachine

```

---

## 17.3 保存异常

流程：

```text id="q7m9vx"
RecorderWorker

↓

ErrorInfo

↓

MeasurementRecord
```

---

# 18 实时性设计

目标：

```text id="j5x9mq"
Feature开始

↓

Yaw输出

≤100ms
```

---

优化：

## 采集

三相机独立线程。

---

## 预览

独立流水线。

---

## 算法

只处理：

selectedCamera。

---

## 保存

后台执行。

---

# 19 线程性能监控

记录：

* 线程运行时间；
* Queue长度；
* 丢帧数量；
* 算法耗时。

---

输出：

```json id="m8q3yz"
thread_status.json
```

---

# 20 测试要求

测试：

## CameraWorker

验证：

* 连续采集；
* 时间戳递增。

---

## SynchronizerWorker

验证：

* 同步误差；
* 异常帧处理。

---

## AlgorithmWorker

验证：

* 输入输出；
* 超时。

---

## RecorderWorker

验证：

* 异步保存；
* 大数据量。

---

# 21 设计冻结总结

| 线程                 | 职责 |
| ------------------ | -- |
| MainThread         | UI |
| CameraWorker       | 采集 |
| TriggerWorker      | 触发 |
| SynchronizerWorker | 同步 |
| PreviewWorker      | 显示 |
| AlgorithmWorker    | 姿态 |
| TurntableWorker    | 控制 |
| RecorderWorker     | 保存 |

---

# 22 修订记录（V2.2 新增）

| 版本 | 日期 | 修订性质 | 备注 |
|---|---|---|---|
| V2.0 | — | 多相机闭环测量版 | 旧版已归档至 `archive/` |
| V2.1 | — | 三相机闭环测量版 | 旧版已归档至 `archive/SYS-09_多线程并发设计_V2.1_多相机闭环测量版.md` |
| **V2.2** | 2026-10-19 | **转台单所有者线程版** | ① 新增 §1.1 修订性质；② §10 增 §10.2（单所有者 I/O、`move()` 仅受理、命令终结结果）、§10.3（停止屏障、≤10 ms 接收切片、join 生命周期）；③ §16 关闭顺序按 `SystemInitializer::shutdown()` 实际实现对齐。其余线程模型未动。依据＝**C-23**（转台可信化批） |

**V2.2 未改动**：§2~§9、§11~§15、§17~§21 的全部内容；不新增线程，
不改变既有线程的创建/销毁责任。

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统多线程并发设计基线。
