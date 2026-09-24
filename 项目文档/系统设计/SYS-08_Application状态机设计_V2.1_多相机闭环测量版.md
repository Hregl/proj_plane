# SYS-08_Application状态机设计_V2.1（三相机闭环测量版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 应用层状态机设计。

状态机负责：

* 测量流程控制；
* 设备调度；
* 算法流程管理；
* 异常恢复；
* 结果保存。

---

# 2 状态机设计原则

## 2.1 设计目标

保证：

```text id="6t7mnp"
任务启动

↓

目标搜索

↓

视觉对准

↓

采集

↓

姿态计算

↓

结果验证

↓

保存

↓

完成
```

完整闭环。

---

## 2.2 状态机职责

负责：

* 状态迁移；
* 状态进入/退出动作；
* 错误处理；
* 流程恢复。

不负责：

* 算法计算；
* 相机采集实现；
* 转台底层控制。

---

# 3 状态总体定义

系统状态：

```cpp id="w9h4mc"
enum class MeasurementState
{

IDLE,


SEARCH,


TARGET_FOUND,


ALIGN,


STABILIZE,


MEASURE_SELECT,


CAPTURE,


POSE_SOLVE,


VALIDATE,


SAVE,


COMPLETE,


FAILED

};
```

---

# 4 状态迁移总图

```text id="z5v8mp"
                 IDLE

                   |

                   ↓

               SEARCH

                   |

                   ↓

            TARGET_FOUND

                   |

                   ↓

                ALIGN

                   |

                   ↓

             STABILIZE

                   |

                   ↓

          MEASURE_SELECT

                   |

                   ↓

              CAPTURE

                   |

                   ↓

            POSE_SOLVE

                   |

                   ↓

              VALIDATE

                   |

                   ↓

                SAVE

                   |

                   ↓

             COMPLETE


异常：

任意状态

    ↓

 FAILED

```

---

# 5 IDLE状态

## 5.1 状态说明

系统空闲状态。

---

## 5.2 进入条件

系统启动完成：

* 配置加载；
* 标定加载；
* 设备初始化。

---

## 5.3 可执行动作

允许：

* 开始测量；
* 查询状态。

---

# 6 SEARCH状态

## 6.1 状态说明

目标搜索阶段。

---

## 6.2 输入

来自：

Preview/Camera。

---

## 6.3 执行动作

包括：

* CAM25优先搜索；
* 图像检测；
* 目标定位。

---

## 6.4 成功条件

检测到：

```text id="q3h7mv"
Target Found
```

迁移：

```text id="8m2x4p"
SEARCH

↓

TARGET_FOUND
```

---

## 6.5 失败处理

超时：

进入：

FAILED。

---

# 7 TARGET_FOUND状态

## 7.1 状态说明

目标已检测。

---

## 7.2 执行动作

执行：

* 尺度估计；
* 目标质量评估。

---

## 7.3 输出

产生：

```text id="7k5zsm"
TargetOffset

ScaleEstimate
```

---

## 7.4 失败处理

如果：

目标质量不足：

进入：

FAILED。

---

# 8 ALIGN状态

## 8.1 状态说明

视觉闭环对准阶段。

---

## 8.2 输入

```text id="3y8m1n"
TargetOffset
```

---

## 8.3 输出

```text id="m6c2wp"
TurntableCommand
```

---

流程：

```text id="a5p7kx"
目标偏差

↓

AlignmentController

↓

转台控制

↓

目标移动

```

---

## 8.4 完成条件

目标进入：

居中范围。

例如：

```text id="p2k9rm"
±50 pixel
```

---

# 9 STABILIZE状态

## 9.1 状态说明

等待：

* 转台停止；
* 图像稳定。

---

## 9.2 检查

包括：

* 转台状态；
* 图像变化；
* 时间稳定。

---

# 10 MEASURE_SELECT状态

## 10.1 状态说明

选择最佳测量通道。

---

## 10.2 输入

三个候选：

```text id="m7c3vq"
CAM25

CAM50

CAM100

```

---

## 10.3 评价指标

包括：

* 目标像素大小；
* 清晰度；
* 特征数量；
* 预测姿态误差。

---

## 10.4 输出

```text id="z8n5hp"
selectedCamera
```

---

# 11 CAPTURE状态

## 11.1 状态说明

执行正式测量采集。

---

## 11.2 动作

执行：

* 触发；
* 三相机采集；
* 时间同步。

---

## 11.3 输出

```text id="k7v3mt"
MultiCameraFrame
```

---

# 12 POSE_SOLVE状态

## 12.1 状态说明

执行姿态求解。

---

## 12.2 输入

包括：

* 图像；
* 模型；
* 标定参数。

---

## 12.3 算法流程

```text id="h3m7xq"
Feature

↓

Matching

↓

PnP

↓

Coordinate Transform

```

---

## 12.4 输出

```text id="w4n8py"
ShipPoseResult
```

---

# 13 VALIDATE状态

## 13.1 状态说明

验证姿态结果。

---

## 13.2 检查内容

包括：

* 重投影误差；
* 内点比例；
* confidence；
* 非有限值。

---

## 13.3 注意

不判断：

```text id="c8q2mv"
Yaw真实误差
```

原因：

真实值需要外部基准。

---

# 14 SAVE状态

## 14.1 状态说明

保存测量结果。

---

## 14.2 保存内容

生成：

```text id="s7h4mx"
MeasurementRecord
```

---

保存：

* 原始图像；
* 姿态结果；
* 标定ID；
* 模型ID；
* 统计信息；
* 日志。

---

# 15 COMPLETE状态

## 15.1 状态说明

一次测量成功完成。

---

保存：

* 最终结果；
* 状态记录。

---

等待：

下一次任务。

---

# 16 FAILED状态

## 16.1 状态说明

测量失败。

---

## 16.2 必须输出

包括：

* ErrorCode；
* ErrorInfo；
* FailureTrace。

---

## 16.3 失败记录

保存：

```text id="g5v8nx"
首次失败状态

↓

最终失败状态

↓

完整迁移轨迹
```

---

# 17 异常处理设计

## 17.1 相机异常

示例：

* 断连；
* 无帧；
* 超时。

处理：

进入：

FAILED。

---

## 17.2 标定异常

包括：

* 标定文件不存在；
* Calibration ID不匹配。

处理：

禁止测量。

---

## 17.3 算法异常

包括：

* 匹配失败；
* PnP失败；
* 验证失败。

记录：

根因。

---

# 18 状态迁移规则

所有迁移必须经过：

```text id="f9m3kx"
StateMachine::transition()
```

统一管理。

---

禁止：

模块直接修改状态。

错误：

```cpp id="x7q1hz"
algorithm.state=FAILED;
```

---

正确：

```text id="p4m8cs"
Algorithm

↓

ErrorInfo

↓

MeasurementController

↓

StateMachine
```

---

# 19 与线程关系

状态机运行于：

Application线程。

其他线程：

```text id="c6w9pz"
CameraWorker

AlgorithmWorker

RecorderWorker

TurntableWorker
```

通过：

* Queue；
* Signal/Slot；

通信。

---

# 20 测量任务生命周期

完整：

```text id="n4w7sy"
Create Task

↓

IDLE

↓

SEARCH

↓

TARGET_FOUND

↓

ALIGN

↓

STABILIZE

↓

MEASURE_SELECT

↓

CAPTURE

↓

POSE_SOLVE

↓

VALIDATE

↓

SAVE

↓

COMPLETE

```

---

# 21 状态机验收

测试：

## 正常流程

验证：

完整路径。

---

## 异常流程

验证：

* 相机失败；
* 算法失败；
* 标定失败。

---

## 重复测量

验证：

COMPLETE后：

可以重新：

IDLE。

---

# 22 设计冻结总结

| 状态             | 功能   |
| -------------- | ---- |
| IDLE           | 空闲   |
| SEARCH         | 搜索   |
| TARGET_FOUND   | 目标确认 |
| ALIGN          | 对准   |
| STABILIZE      | 稳定   |
| MEASURE_SELECT | 焦段选择 |
| CAPTURE        | 采集   |
| POSE_SOLVE     | 姿态求解 |
| VALIDATE       | 验证   |
| SAVE           | 保存   |
| COMPLETE       | 完成   |
| FAILED         | 失败   |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统 Application状态机设计基线。
