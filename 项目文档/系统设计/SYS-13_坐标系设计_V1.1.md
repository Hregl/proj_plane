# SYS-13_坐标系设计_V1.1（三相机固定光机坐标版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 多相机视觉测量系统中的坐标体系设计。

目标：

建立从：

```text id="x7j4ab"
Camera Coordinate

        ↓

OpticalRig Coordinate

        ↓

Ship Coordinate

        ↓

Aircraft Coordinate

```

的完整空间转换链。

本文档用于约束：

* 光机标定；
* 外参管理；
* PnP姿态输出；
* 舰体坐标转换；
* 多相机数据融合。

---

# 2 坐标体系总体架构

系统采用四级坐标体系：

```text id="4s9w2n"
Level 1

Camera Frame


↓

Level 2

OpticalRig Frame


↓

Level 3

Ship Frame


↓

Level 4

Aircraft Frame

```

---

# 3 Camera坐标系

## 3.1 定义

每个相机具有独立坐标系。

包括：

```text id="r3p7fz"
CAM25

CAM50

CAM100
```

---

# 3.2 坐标定义

采用标准相机坐标：

* 原点：光心；
* Z轴：光轴方向；
* X轴：图像水平方向；
* Y轴：图像竖直方向。

---

# 3.3 相机坐标转换

每个相机：

具有：

$$
T_{Camera}^{Rig}
$$

表示：

相机坐标到光机坐标的转换。

---

# 4 OpticalRig坐标系

## 4.1 定义

OpticalRig为三相机固定结构参考坐标。

作用：

统一：

* 三相机空间关系；
* 光机安装关系。

---

# 4.2 坐标原点

定义：

推荐采用：

光机机械安装基准中心。

要求：

安装完成后：

保持固定。

---

# 4.3 坐标方向

冻结：

* X轴；
* Y轴；
* Z轴。

所有：

* Camera→Rig；
* Rig→Ship；

转换必须使用该坐标。

---

# 5 Camera→Rig转换

## 5.1 转换关系

对于第i个相机：

$$
P_{Rig}
=
T_{Camera_i}^{Rig}
P_{Camera_i}
$$

其中：

$$
i=
25,50,100
$$

---

# 5.2 参数来源

来自：

SYS-16：

《相机标定与光机外参实施设计》。

输出：

```text id="qf6k0d"
CAM25_to_Rig

CAM50_to_Rig

CAM100_to_Rig

```

---

# 5.3 数据组成

每个转换：

包含：

旋转矩阵：

$$
R
$$

和平移：

$$
t
$$

即：

$$
T=
[R|t]
$$

---

# 6 Ship坐标系

## 6.1 定义

Ship Frame为舰体参考坐标。

用于：

最终姿态输出。

---

# 6.2 坐标原点

根据舰体安装基准确定。

要求：

* 固定；
* 可测量；
* 可复现。

---

# 6.3 坐标方向

冻结：

* X轴；
* Y轴；
* Z轴。

定义必须与舰载系统总体坐标一致。

---

# 7 Rig→Ship转换

## 7.1 定义

建立：

```text id="2pmqvq"
OpticalRig

↓

Ship

```

关系。

---

## 7.2 数学表达

$$
P_{Ship}
=
T_{Rig}^{Ship}
P_{Rig}
$$

---

## 7.3 参数来源

来自：

* 光机安装测量；
* 外部基准测量；
* 专用标定流程。

---

# 8 Camera→Ship完整链路

最终：

$$
T_{Camera}^{Ship}
=
T_{Rig}^{Ship}
\times
T_{Camera}^{Rig}
$$

---

数据流：

```text id="q4q8ra"
Image

↓

Camera Coordinate

↓

OpticalRig Coordinate

↓

Ship Coordinate

```

---

# 9 Aircraft坐标系

## 9.1 定义

Aircraft Frame表示目标飞机自身坐标。

用于：

描述：

* 飞机姿态；
* 目标模型；
* 3D特征点。

---

# 9.2 来源

来自：

目标CAD模型。

包含：

* 3D模型坐标；
* 特征点坐标；
* 结构点坐标。

---

# 10 PnP姿态关系

## 10.1 输入

PnP输入：

```text id="w4n6ub"
2D Image Points

+

3D Aircraft Points

+

Camera Calibration

```

---

# 10.2 输出

PnP得到：

$$
T_{Aircraft}^{Camera}
$$

表示：

飞机坐标到相机坐标。

---

# 10.3 转换到舰体坐标

最终：

$$
T_{Aircraft}^{Ship}
=
T_{Rig}^{Ship}
\times
T_{Camera}^{Rig}
\times
T_{Aircraft}^{Camera}
$$

---

# 11 多相机联合测量

## 11.1 三相机关系

三个相机：

分别观测目标。

转换：

统一到：

```text id="x9sd7w"
OpticalRig Frame
```

---

## 11.2 联合结果

所有测量：

必须转换到同一：

```text id="w2l5fa"
Ship Frame
```

进行比较。

---

# 12 坐标变换数据管理

目录：

```text id="v5c3b1"
calibration/

├── CAM25/

│   └── camera_to_rig.yaml


├── CAM50/

│   └── camera_to_rig.yaml


├── CAM100/

│   └── camera_to_rig.yaml


└── rig_to_ship.yaml

```

---

# 13 坐标版本管理

每组坐标参数：

绑定：

```text id="a7q8l3"
calibration_id
```

包含：

* 标定时间；
* 光机版本；
* 软件版本。

---

禁止：

不同Calibration ID混合使用。

---

# 14 坐标一致性验证

## 14.1 多相机验证

同一目标点：

分别通过：

CAM25/CAM50/CAM100

转换到Rig。

比较：

$$
\Delta P
$$

---

## 14.2 舰体系验证

验证：

Rig→Ship转换：

* 方向；
* 平移；
* 稳定性。

---

# 15 坐标误差来源

主要包括：

## 相机内参误差

影响：

* 光线投影；
* PnP。

---

## Camera→Rig误差

影响：

* 多相机融合。

---

## Rig→Ship误差

影响：

* 最终舰体姿态。

---

# 16 与标定系统关系

关系：

```text id="7j2h5w"
SYS-16

↓

Calibration Package

↓

SYS-13 Coordinate

↓

PosePipeline

```

---

# 17 与算法系统关系

算法只使用：

```text id="wq3p1m"
Camera Calibration

+

Coordinate Transform

+

Target Model

```

禁止：

算法模块直接访问：

* 相机设备；
* SDK；
* 转台接口。

---

# 18 与转台系统关系

转台坐标：

作为辅助信息输入：

用于：

* 粗姿态；
* 搜索；
* 对准。

不替代：

PnP姿态结果。

---

# 19 坐标冻结规则

以下变化必须重新确认：

* 相机安装位置变化；
* 镜头拆装；
* 光机结构变化；
* 舰体安装基准变化。

---

# 20 设计冻结总结

| 项目           | 状态               |
| ------------ | ---------------- |
| Camera坐标     | 冻结               |
| OpticalRig坐标 | 冻结               |
| Ship坐标       | 冻结               |
| Aircraft坐标   | 冻结               |
| Camera→Rig   | 标定获得             |
| Rig→Ship     | 标定获得             |
| PnP输出        | Aircraft→Camera  |
| 最终输出         | Aircraft→Ship    |
| 坐标版本         | Calibration ID管理 |

---

本文档作为 AircraftPoseSystem V2.1 三相机固定光机系统坐标体系设计基线。
