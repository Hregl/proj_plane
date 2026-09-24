# SYS-19_系统测试与验收设计_V1.1（多相机闭环测量阶段化验收版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统测试体系与验收规范。

目标：

建立覆盖：

* 软件工程；
* 设备接入；
* 光机标定；
* 算法验证；
* 姿态精度；
* 实时性能；
* 稳定运行；

的完整测试与验收体系。

---

# 2 测试总体原则

## 2.1 分阶段验收原则

系统采用：

```text id="9k7v3m"
Framework Alpha

↓

Hardware Integration

↓

Measurement Beta

↓

Release

```

阶段化验收。

---

## 2.2 数据可追溯原则

所有测试结果必须绑定：

```text id="m5q8xz"
Software Version

+

Config Version

+

Calibration ID

+

Model ID

```

---

## 2.3 可复现原则

测试必须保存：

* 输入数据；
* 配置；
* 标定；
* 模型；
* 输出结果。

---

# 3 测试体系结构

目录：

```text id="q7m3vx"
tests/

├── unit/

├── device/

├── optical/

├── algorithm/

├── integration/

├── hardware/

├── stability/

└── acceptance/

```

---

# 4 测试阶段划分

---

# 4.1 Phase 1：Framework Alpha

目标：

验证软件框架完整性。

环境：

```text id="k5n8qm"
UOS

+

Virtual Device

```

---

测试内容：

* CMake构建；
* Qt启动；
* VirtualCamera；
* Preview；
* 状态机；
* Recorder。

---

验收：

满足：

```text id="w8m2qp"
软件闭环运行
```

---

# 4.2 Phase 2：Hardware Integration

目标：

验证真实硬件链路。

环境：

```text id="h6m3px"
A7A20MU201

+

ImvSDK

+

UOS

```

---

测试内容：

* SDK加载；
* 相机枚举；
* 单帧采集；
* 连续采集；
* Preview显示；
* RAW保存。

---

验收：

完成：

```text id="q5m8xz"
A7A20MU201

↓

ImvSDK

↓

ImageFrame

↓

Preview

↓

Recorder
```

闭环。

---

# 4.3 Phase 3：Measurement Beta

目标：

验证真实测量能力。

环境：

```text id="x8m2qp"
三相机

+

Calibration Package

+

Production Model
```

---

测试内容：

* 三相机同步；
* 光机标定；
* MeasurementSelector；
* PnP；
* 坐标转换；
* 姿态输出。

---

# 4.4 Phase 4：Release

目标：

系统交付验收。

验证：

* 精度；
* 实时性；
* 稳定性；
* 部署能力。

---

# 5 单元测试

---

# 5.1 Data测试

验证：

* ImageFrame；
* MultiCameraFrame；
* Transform；
* ShipPoseResult；
* MeasurementRecord。

---

# 5.2 StateMachine测试

验证：

状态：

```text id="p8m4qx"
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

测试：

* 正常迁移；
* 非法迁移；
* 异常恢复。

---

# 6 Device测试

---

# 6.1 VirtualCamera测试

验证：

* frameId递增；
* timestamp递增；
* 图像输出。

---

# 6.2 ImvCameraBackend测试

Phase 2执行。

测试：

* SDK初始化；
* 相机连接；
* 参数读取；
* 图像采集。

---

# 6.3 MultiCameraManager测试

验证：

* 三通道管理；
* 启停；
* 异常处理。

---

# 7 Trigger测试

---

## 7.1 软件触发

验证：

* 触发流程；
* 图像关联。

---

## 7.2 硬件触发

验证：

* GPIO触发；
* 曝光同步；
* 时间一致性。

---

# 8 光机标定测试

---

# 8.1 内参标定测试

对象：

```text id="h4m8qx"
CAM25

CAM50

CAM100

```

---

验证：

* K矩阵；
* 畸变参数；
* 重投影误差。

---

# 8.2 Camera→Rig外参测试

验证：

```text id="j7m3qx"
CAM25→Rig

CAM50→Rig

CAM100→Rig

```

---

指标：

* 平移误差；
* 旋转误差；
* 重投影误差。

---

# 8.3 Rig→Ship测试

验证：

```text id="w6m3qp"
Rig

↓

Ship

```

---

检查：

* 坐标方向；
* 转换稳定性。

---

# 9 Preview测试

---

## 9.1 功能测试

验证：

* 程序启动显示；
* CAM25默认显示；
* 手动切换。

---

## 9.2 性能测试

记录：

* 刷新频率；
* 延迟；
* 丢帧数量。

---

# 10 Algorithm测试

---

# 10.1 TargetDetector测试

验证：

* 检测结果；
* 置信度；
* 推理时间。

---

# 10.2 MeasurementSelector测试

输入：

```text id="n5m8qx"
CAM25

CAM50

CAM100
```

---

验证：

* 评分；
* 镜头选择；
* 边界情况。

---

# 10.3 Feature测试

验证：

* 特征数量；
* 匹配数量；
* 稳定性。

---

# 10.4 PnP测试

输入：

* 2D点；
* 3D点；
* Calibration。

输出：

* 姿态；
* 重投影误差。

---

# 11 CAD辅助测试

---

# 11.1 CAD结构点测试

验证：

* CAD点加载；
* 结构点匹配。

---

# 11.2 CadLocatorBenchmark

验证：

* 亚像素定位；
* 残差；
* 可用性。

---

说明：

该测试用于算法能力验证。

不作为当前硬件接入阶段阻塞项。

---

# 12 坐标转换测试

验证：

完整链路：

```text id="f7m3qx"
Aircraft

↓

Camera

↓

Rig

↓

Ship

```

---

检查：

* 旋转矩阵合法；
* 平移连续；
* 坐标闭合误差。

---

# 13 实时性测试

目标：

```text id="x6m2qp"
Feature开始

↓

Yaw输出

≤100ms
```

---

记录：

| 阶段         | 时间 |
| ---------- | -- |
| Feature    | ms |
| Matching   | ms |
| PnP        | ms |
| Validation | ms |
| Total      | ms |

---

# 14 姿态精度验收

---

# 14.1 在线验证

检查：

* 重投影误差；
* 内点比例；
* confidence；
* 非有限值。

---

# 14.2 离线精度验收

需要：

外部真值。

输入：

```text id="z5m8qx"
Reference Pose

+

Measured Pose
```

---

计算：

$$
Error=
|Pose_{measure}-Pose_{reference}|
$$

---

目标：

```text id="m8q3px"
Yaw≤1角分
```

---

# 15 距离覆盖测试

测试距离：

```text id="p7m3qx"
40m

80m

120m

150m

220m

300m
```

---

验证：

* 镜头覆盖；
* MeasurementSelector；
* 姿态稳定性。

---

# 16 三镜头测试

---

## CAM25

范围：

```text id="r5m8qx"
45~120m
```

---

## CAM50

范围：

```text id="n7m3qx"
100~220m
```

---

## CAM100

范围：

```text id="h8m3qx"
180~320m
```

---

验证：

* 切换连续；
* 标定一致；
* 输出一致。

---

# 17 转台闭环测试

验证：

流程：

```text id="q6m8px"
TargetOffset

↓

AlignmentController

↓

TurntableCommand

↓

Turntable

↓

Target Center
```

---

指标：

目标：

```text id="w5m8qx"
±50 pixel
```

---

# 18 稳定性测试

---

# 18.1 长时间运行

建议：

8小时。

检查：

* CPU；
* 内存；
* 线程；
* 文件。

---

# 18.2 连续测量

建议：

1000次。

统计：

* 成功率；
* 失败原因；
* 平均耗时。

---

# 19 数据包验收

每次测量生成：

```text id="k8m3qx"
measurement_xxx/

├── cam25.raw

├── cam50.raw

├── cam100.raw

├── result.json

├── config_snapshot/

├── calibration/

└── log.txt

```

---

检查：

* 文件完整；
* 参数一致；
* 可复现。

---

# 20 错误追踪验收

失败任务必须包含：

* ErrorCode；
* ErrorInfo；
* FailureTrace。

---

检查：

首次失败原因是否保留。

---

# 21 发布验收流程

流程：

```text id="p4m8qx"
开发测试

↓

集成测试

↓

硬件测试

↓

精度测试

↓

稳定性测试

↓

发布验收

```

---

# 22 测试报告输出

目录：

```text id="m7q3qx"
test_report/

├── unit.xml

├── performance.csv

├── precision.json

├── stability.md

└── summary.md

```

---

# 23 验收标准总结

| 项目      | 指标     |
| ------- | ------ |
| 工程编译    | 通过     |
| 真实相机    | 支持     |
| 三相机架构   | 支持     |
| 标定体系    | 有效     |
| Preview | 实时     |
| 实时性     | ≤100ms |
| Yaw精度   | ≤1角分   |
| 稳定运行    | 满足要求   |

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统测试与验收设计基线。


