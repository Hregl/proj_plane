# SYS-09_多线程并发设计_V2.1（三相机闭环测量版）

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

关闭：

```text id="k3v7mh"
Stop Measurement

↓

Stop Algorithm

↓

Stop Synchronizer

↓

Stop Preview

↓

Stop Turntable

↓

Stop Trigger

↓

Stop Camera

↓

thread.wait()

↓

Release

```

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

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统多线程并发设计基线。
