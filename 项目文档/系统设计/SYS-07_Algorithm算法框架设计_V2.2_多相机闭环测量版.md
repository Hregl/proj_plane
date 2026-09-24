# SYS-07_Algorithm算法框架设计_V2.2（三相机标定融合与闭环姿态测量版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 中算法系统架构设计。

本版本针对：

* 三相机固定光机结构；
* 锁焦三镜头体系；
* MeasurementSelector像素驱动选焦；
* TargetModelManager模型管理；
* CAD辅助定位；
* 自然纹理匹配；
* PnP姿态求解；
* 标定参数融合；
* 闭环测量；

进行算法架构补充。

---

# 2 Algorithm层职责

Algorithm层负责：

* 目标检测；
* 距离与尺度估计；
* 测量通道选择；
* 特征提取；
* 特征匹配；
* 2D-3D对应建立；
* PnP姿态求解；
* 坐标转换；
* 姿态结果验证。

---

Algorithm层不负责：

* 相机采集；
* SDK调用；
* 转台控制；
* UI显示；
* 数据保存。

---

# 3 算法总体流程

完整算法链：

```text id="h8q3mv"
ImageFrame

↓

Target Detection

↓

Scale Estimation

↓

Measurement Selector

↓

Feature Extraction

↓

Feature Matching

↓

PnP Pose Solve

↓

Coordinate Transform

↓

Pose Validation

↓

Ship Pose Result

```

---

# 4 算法模块结构

```text id="r6m2xp"
algorithm/

├── detection/

├── scale/

├── selection/

├── model/

├── feature/

├── matcher/

├── pose/

├── validation/

└── pipeline/

```

---

# 5 TargetDetector目标检测

## 5.1 功能

从输入图像中检测目标区域。

---

输入：

```text id="m5q8nx"
ImageFrame
```

---

输出：

```cpp id="q7m3pz"
DetectionResult
```

---

结构：

```cpp
struct DetectionResult
{

bool found;


int x;

int y;

int width;

int height;


double confidence;

};
```

---

# 5.2 实现方式

支持：

* YOLO；
* 传统视觉检测；
* 模板检测。

接口：

```text id="y8m4qx"
TargetDetector

↓

Implementation

```

---

# 6 TargetScaleEstimator尺度估计

## 6.1 功能

估计：

* 目标距离；
* 目标像素尺寸。

---

输入：

```text id="w4m8pz"
DetectionResult

+

TargetModel
```

---

输出：

```cpp
struct ScaleEstimate
{

double distanceEstimate;


double targetPixelSize;


double confidence;

};
```

---

# 6.2 用途

用于：

* MeasurementSelector；
* 粗定位；
* 镜头选择。

不作为：

最终姿态结果。

---

# 7 MeasurementSelector测量通道选择

## 7.1 设计目标

三镜头：

```text id="p8m2xz"
CAM25

CAM50

CAM100
```

固定安装。

运行时：

选择最佳测量通道。

---

# 7.2 输入数据

```cpp
struct TargetMeasurementFeature
{

double distanceEstimate;


double targetPixelSize;


int featureCount;


double sharpness;


double predictedPoseError;

};
```

---

# 7.3 评分模型

$$
Score=
w_1P+w_2F+w_3S+w_4E
$$

---

## P：像素覆盖评分

目标：

```text id="g5m8qx"
1200~3000 pixel
```

---

## F：特征数量评分

依据：

* SIFT数量；
* CAD点数量；
* 匹配数量。

---

## S：清晰度评分

依据：

* Laplacian；
* 图像锐度；
* 特征响应。

---

## E：姿态误差预测评分

依据：

* 重投影误差；
* PnP稳定性。

---

# 8 TargetModelManager目标模型管理

## 8.1 功能

管理：

* CAD模型；
* 三维点；
* 特征库。

---

目录：

```text id="j7m3qx"
models/

└── aircraft/

    ├── model.yaml

    ├── points3d.yaml

    ├── cad_points.yaml

    ├── feature25.bin

    ├── feature50.bin

    └── feature100.bin

```

---

# 8.2 模型类型

支持：

```text id="q9m3px"
production

synthetic

```

---

release阶段：

禁止：

```text id="w6m2qz"
synthetic
```

模型进入正式测量。

---

# 9 FeatureExtractor特征提取

## 9.1 特征类型

系统支持：

两类特征：

```text id="m8q5xz"
A类：

自然纹理特征


B类：

CAD结构特征

```

---

# 9.2 自然纹理路径

采用：

* SIFT；
* Descriptor。

输出：

```text id="n7m3qx"
2D KeyPoint

+

Descriptor

```

---

# 9.3 CAD结构路径

来源：

CAD模型。

输出：

```text id="r4m8px"
2D Structure Points

+

3D CAD Points

```

---

# 10 CadStructureLocator

## 10.1 功能

基于：

CAD结构约束。

用于：

* 高精定位；
* 辅助PnP。

---

# 10.2 当前状态

CadStructureLocator：

属于：

增强测量路径。

基础测量路径：

仍需：

自然纹理特征。

---

# 10.3 亚像素验证

通过：

```text id="x5m8qp"
CadLocatorBenchmark
```

验证：

* 定位误差；
* 可用性；
* 稳定性。

---

# 11 FeatureMatcher匹配

## 11.1 功能

建立：

```text id="z8m4qx"
2D Image Points

↓

3D Model Points
```

---

# 11.2 输出

```cpp
struct MatchResult
{

int matchCount;


};
```

---

# 11.3 统计输出

通过：

```text id="q6m8xz"
IPipelineObserver
```

输出：

```cpp
MeasurementStatistics
```

---

统计：

* featureCount；
* matchCount；
* matchRatio；
* cadCount；
* textureCount；
* spreadPx。

---

# 12 PnPPoseEstimator

## 12.1 功能

计算：

目标姿态。

---

输入：

```text id="v7m3qx"
2D Points

+

3D Points

+

CameraCalibration
```

---

输出：

```text id="p4m8xz"
Aircraft→Camera Transform
```

---

# 12.2 求解流程

```text id="k8m3qp"
Correspondence

↓

RANSAC

↓

PnP

↓

Reprojection Error

↓

Pose Result

```

---

# 13 坐标转换

PnP输出：

```text id="n5m8qx"
Aircraft

↓

Camera

```

---

转换：

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

# 14 PoseValidator结果验证

## 14.1 职责

判断：

结果是否自洽。

---

检查：

* 重投影误差；
* 内点比例；
* confidence；
* 非有限值。

---

不包含：

```text id="x9m3qw"
Yaw真实误差
```

原因：

真实精度需要外部真值。

---

# 15 PosePipeline

## 15.1 功能

组织完整算法流程。

---

接口：

```cpp
bool process(
const MultiCameraFrame& frame
);
```

---

输入：

* 图像；
* 标定；
* 模型。

---

输出：

```text id="h6m2qx"
ShipPoseResult

+

MeasurementStatistics

+

PoseValidationResult

```

---

# 16 三相机算法策略

## 16.1 三相机采集

输入：

```text id="q7m4pz"
CAM25

CAM50

CAM100
```

---

## 16.2 单通道深度计算

流程：

```text id="y8m3qx"
MultiCameraFrame

↓

MeasurementSelector

↓

Selected Camera

↓

完整PosePipeline
```

---

不对三个镜头同时执行完整PnP。

---

# 17 算法实时性设计

目标：

```text id="p5m8qx"
Feature开始

↓

Yaw输出

≤100ms
```

---

优化：

## Detection

限制ROI。

---

## Feature

控制数量。

---

## Matching

使用候选过滤。

---

## PnP

RANSAC优化。

---

# 18 算法误差控制

主要来源：

## 特征误差

控制：

* 亚像素优化；
* 高质量特征。

---

## 匹配误差

控制：

* RANSAC；
* 重投影过滤。

---

## 标定误差

控制：

* SYS-16标定流程。

---

# 19 算法测试要求

## Unit测试

验证：

* Detector；
* Selector；
* Matcher；
* PnP。

---

## Golden测试

固定：

* 图像；
* 模型；
* 标定。

---

## 性能测试

记录：

* Detection时间；
* Matching时间；
* PnP时间；
* 总时间。

---

# 20 模块依赖关系

允许：

```text id="z6m3qx"
Algorithm

↓

Data

↓

OpenCV
```

---

禁止：

```text id="x4m8qp"
Algorithm

↓

Camera SDK
```

---

禁止：

```text id="q5m7xz"
Algorithm

↓

Qt
```

---

# 21 设计冻结总结

| 项目                  | 状态         |
| ------------------- | ---------- |
| 算法流程                | 冻结         |
| MeasurementSelector | 冻结         |
| 三焦段选择               | 冻结         |
| TargetModelManager  | 冻结         |
| CAD辅助定位             | 增强路径       |
| 自然纹理匹配              | 基础路径       |
| PnP输入               | 冻结         |
| 坐标输出                | Ship Frame |
| 在线验证                | 自洽性        |
| 精度验收                | 外部真值       |

---

本文档作为 AircraftPoseSystem V2.1 三相机闭环姿态测量算法框架设计基线。
