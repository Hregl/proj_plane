# SYS-12_目标模型与特征库设计_V1.1（三相机PnP测量版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 中目标模型与特征库设计。

目标：

建立面向：

* 飞机目标检测；
* CAD辅助定位；
* 自然纹理匹配；
* PnP姿态求解；
* 三焦段测量；

的目标模型数据体系。

---

# 2 设计原则

## 2.1 模型职责

目标模型负责：

* 三维几何描述；
* 特征点定义；
* 多焦段特征库管理；
* 2D-3D对应关系建立。

---

不负责：

* 相机采集；
* 标定计算；
* 转台控制。

---

# 3 模型体系架构

系统目标模型：

```text id="4j8s9x"
TargetModel

├── Geometry Model

├── CAD Structure Points

├── Natural Feature Library

└── Camera Specific Feature Library

```

---

# 4 模型目录结构

冻结：

```text id="n4q7fz"
models/

└── aircraft/


    ├── model.yaml


    ├── points3d.yaml


    ├── cad_points.yaml


    ├── feature25.bin


    ├── feature50.bin


    ├── feature100.bin


    └── metadata.yaml

```

---

# 5 模型类型

## 5.1 production

真实交付模型。

包含：

* 真实目标CAD；
* 实际尺寸；
* 三维点；
* 特征库。

用于：

正式测量。

---

## 5.2 synthetic

合成测试模型。

用途：

* 软件开发；
* 单元测试；
* 算法验证。

---

release版本：

禁止：

```text id="y6m2kp"
synthetic
```

模型参与正式测量。

---

# 6 TargetModel数据结构

```cpp id="8x4v9m"
struct TargetModel
{

std::string modelId;


std::string modelType;


double realSizeM;


};
```

---

字段：

## modelId

模型唯一编号。

---

## modelType

取值：

```text id="2k8p5m"
production

synthetic

```

---

## realSizeM

目标实际尺寸。

用于：

* 尺度估计；
* 粗姿态。

---

# 7 三维几何模型

## 7.1 points3d.yaml

保存：

目标三维特征点。

示例：

```yaml
points:

  - id: 1

    x: 0.0

    y: 0.0

    z: 0.0

```

---

用途：

PnP输入：

```text id="8z1q5k"
3D Object Points
```

---

# 8 CAD结构点设计

## 8.1 CAD Structure Points

来源：

目标CAD模型。

用于：

* 高精定位；
* 结构约束；
* 辅助PnP。

---

## 8.2 数据内容

包含：

```text id="h3m8qy"
point_id

3D position

feature_type

visibility

```

---

# 9 自然纹理特征库

## 9.1 目的

解决：

* CAD点不足；
* 纹理丰富区域定位。

---

来源：

真实图像：

提取：

* SIFT；
* 描述子；
* 特征位置。

---

# 10 三焦段特征库设计

三个镜头分别维护：

```text id="6v3z4h"
feature25.bin

feature50.bin

feature100.bin

```

---

原因：

不同焦段：

* 视场不同；
* 分辨率不同；
* 特征尺度不同。

---

禁止：

不同镜头混用同一特征库。

---

# 11 Feature Library数据

包含：

* KeyPoint；
* Descriptor；
* 对应3D点。

---

逻辑：

```text id="g7p2xw"
2D Feature

↓

Descriptor Matching

↓

3D Point

```

---

# 12 模型加载流程

流程：

```text id="z5k8mq"
启动

↓

读取model.yaml

↓

加载points3d

↓

加载CAD点

↓

加载feature库

↓

Model Ready

```

---

# 13 TargetModelManager设计

职责：

管理：

* 模型生命周期；
* 特征库加载；
* 版本检查。

---

接口：

```cpp id="m4x8qz"
bool load(
const std::string& path
);


TargetModel model();

```

---

# 14 多焦段模型选择

根据：

MeasurementSelector输出：

```text id="p9n3wy"
CameraRole
```

选择：

对应特征库。

---

例如：

CAM25：

```text id="w4k8zs"
feature25.bin
```

---

CAM50：

```text id="d6q2my"
feature50.bin
```

---

CAM100：

```text id="r8m5kx"
feature100.bin
```

---

# 15 PnP数据链

完整流程：

```text id="x7n3mp"
Image

↓

Feature Extraction

↓

Feature Matching

↓

2D Points

+

3D Points

↓

PnP

↓

Aircraft Pose

```

---

# 16 CAD与自然纹理融合策略

系统支持：

两类特征：

```text id="z9m4qx"
A类：

自然纹理Feature


B类：

CAD Structure Feature

```

---

匹配结果：

统一进入：

```text id="h6p2vz"
Correspondence Set
```

---

# 17 匹配统计

输出：

```cpp id="n8q4km"
MeasurementStatistics
```

包括：

```text
featureCount

matchCount

matchRatio

cadCount

textureCount

spreadPx

```

---

# 18 模型版本管理

每个模型绑定：

```text id="v5q9px"
modelId

+

modelVersion

+

softwareVersion

```

---

特征库变化：

必须更新：

modelVersion。

---

# 19 模型验证

加载时检查：

## 文件完整性

检查：

* model.yaml；
* points3d；
* feature库。

---

## 尺寸一致性

检查：

realSizeM。

---

## 特征数量

检查：

feature数量是否满足要求。

---

# 20 与算法模块关系

数据流：

```text id="h7m3qp"
TargetModelManager

↓

PosePipeline

↓

PnP

```

---

Algorithm只读取：

* 模型数据；
* 特征数据。

---

# 21 与标定关系

模型坐标：

属于：

Aircraft Frame。

相机标定：

属于：

Camera Frame。

二者通过：

PnP建立：

```text id="w3k6mz"
Aircraft

↓

Camera

```

关系。

---

# 22 与数据层关系

依赖：

```text id="x5n7qy"
TargetModel

↓

Data

```

---

禁止：

模型直接访问：

* Camera SDK；
* Device。

---

# 23 测试要求

## 模型加载测试

验证：

* 文件存在；
* 参数正确。

---

## 特征库测试

验证：

* 三焦段独立；
* 加载成功。

---

## PnP测试

验证：

* 2D-3D对应正确；
* 姿态可恢复。

---

# 24 当前开发阶段策略

当前：

允许：

```text id="m7q2xz"
synthetic model

+

VirtualCamera

```

用于：

软件闭环。

---

真实阶段：

替换：

```text id="k8m4py"
production model

+

真实图像

```

---

# 25 设计冻结总结

| 项目           | 状态   |
| ------------ | ---- |
| 目标模型         | 冻结   |
| CAD点         | 冻结   |
| 自然纹理         | 冻结   |
| 三焦段特征库       | 冻结   |
| synthetic隔离  | 冻结   |
| production模型 | 交付要求 |
| PnP输入        | 冻结   |

---

本文档作为 AircraftPoseSystem V2.1 三相机目标模型与特征库设计基线。
