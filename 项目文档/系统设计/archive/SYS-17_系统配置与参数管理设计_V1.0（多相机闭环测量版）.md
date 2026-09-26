# SYS-17_系统配置与参数管理设计_V1.0（多相机闭环测量版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统配置管理体系。

目标：

统一管理：

* 系统运行参数；
* 相机参数；
* 光机标定参数；
* 测量策略参数；
* 转台参数；
* 验证参数；
* 日志与存储参数。

确保：

* 参数可追溯；
* 配置可复现；
* 版本可管理。

---

# 2 配置管理原则

## 2.1 配置与代码分离

系统运行参数：

不写入代码。

采用：

```text id="c7m2vx"
Executable

+

config/

+

calibration/

+

models/

```

---

## 2.2 配置只读原则

运行阶段：

加载配置。

禁止：

运行过程中：

* 修改配置文件；
* 动态改变标定参数。

---

## 2.3 配置版本绑定

一次测量结果必须关联：

```text id="q8m4zy"
Software Version

+

Config Version

+

Calibration ID

+

Model ID
```

---

# 3 配置目录结构

冻结：

```text id="r5n8mq"
config/

├── system.yaml

├── camera.yaml

├── optical_rig.yaml

├── trigger.yaml

├── turntable.yaml

├── measurement.yaml

├── validation.yaml

└── logging.yaml

```

---

# 4 system.yaml

## 4.1 功能

系统级配置。

示例：

```yaml
system:

  name:
    AircraftPoseSystem


  version:
    V2.1


  mode:
    normal

```

---

字段：

| 字段      | 说明   |
| ------- | ---- |
| name    | 系统名称 |
| version | 软件版本 |
| mode    | 运行模式 |

---

# 5 camera.yaml

## 5.1 功能

管理相机基础参数。

示例：

```yaml
camera:

  count: 3


  channels:

    - id: CAM25

      role: CAM25


    - id: CAM50

      role: CAM50


    - id: CAM100

      role: CAM100

```

---

# 5.2 相机参数

包含：

* 分辨率；
* Pixel Format；
* 曝光；
* 增益；
* 触发模式。

---

示例：

```yaml
resolution:

  width:4096

  height:3000


pixel_format:

  Mono12

```

---

# 6 optical_rig.yaml

## 6.1 功能

管理光机结构配置。

---

示例：

```yaml
optical_rig:

  calibration_id:

    CAL_20260923_V01


  calibration_mode:

    file

```

---

包含：

* 光机版本；
* 标定ID；
* 坐标关系。

---

# 7 trigger.yaml

## 7.1 功能

管理触发配置。

示例：

```yaml
trigger:

  mode:

    hardware


  sync_tolerance_ns:

    1000000

```

---

参数：

| 参数                | 说明   |
| ----------------- | ---- |
| mode              | 触发模式 |
| sync_tolerance_ns | 同步容差 |

---

# 8 turntable.yaml

## 8.1 功能

管理转台参数。

示例：

```yaml
turntable:

  interface:

    pekod


  azimuth_limit:

    360


  elevation_limit:

    60

```

---

包含：

* 通信方式；
* 轴限制；
* 速度限制。

---

# 9 measurement.yaml

## 9.1 功能

管理测量策略。

示例：

```yaml
measurement:

  default_camera:

    CAM25


  pixel_target_min:

    1200


  pixel_target_max:

    3000

```

---

包含：

* 焦段选择参数；
* 特征数量阈值；
* 测量模式。

---

# 10 validation.yaml

## 10.1 功能

管理结果验证参数。

示例：

```yaml
validation:

  max_reprojection_error:

    2.0


  min_inlier_ratio:

    0.5


  min_confidence:

    0.8

```

---

注意：

不包含：

```text id="r3m7qy"
yawErrorArcmin
```

原因：

真实角度误差需要外部真值。

---

# 11 logging.yaml

## 11.1 功能

管理日志。

示例：

```yaml
logging:

  level:

    INFO


  directory:

    logs/

```

---

# 12 ConfigManager设计

## 12.1 职责

负责：

* 配置加载；
* 参数校验；
* 配置快照。

---

接口：

```cpp
class ConfigManager
{

bool load(
std::string path
);


bool validate();


Config get();

};
```

---

# 13 配置校验

启动阶段检查：

## 系统配置

检查：

* 版本；
* 模式。

---

## 相机配置

检查：

* 通道数量；
* CameraRole。

---

## 标定配置

检查：

* calibration_id；
* 文件存在。

---

## 模型配置

检查：

* model_id；
* model_type。

---

# 14 配置快照

每次测量保存：

```text id="n6p3kw"
measurement_xxx/

└── config_snapshot/

    ├── system.yaml

    ├── camera.yaml

    ├── measurement.yaml

    ├── validation.yaml

    └── ...
```

---

目的：

保证：

历史结果可复现。

---

# 15 配置与标定关系

关系：

```text id="w8m2pc"
config

↓

calibration_id

↓

Calibration Package

```

---

禁止：

配置引用：

不存在的标定版本。

---

# 16 配置与模型关系

关系：

```text id="q3m7vx"
config

↓

model_id

↓

TargetModelManager

```

---

禁止：

release加载：

synthetic模型。

---

# 17 配置加载流程

启动：

```text id="h7m2qx"
main.cpp

↓

ConfigManager

↓

参数校验

↓

ApplicationContext

↓

模块初始化

```

---

# 18 配置异常处理

## 文件不存在

结果：

启动失败。

---

## 参数非法

结果：

输出：

ErrorInfo。

---

## 版本不匹配

结果：

拒绝运行。

---

# 19 与其他模块关系

```text id="x9m4qp"
SYS-17

↓

SYS-05 Data

↓

SYS-06 Device

↓

SYS-07 Algorithm

↓

SYS-16 Calibration

```

---

# 20 设计冻结总结

| 配置     | 状态 |
| ------ | -- |
| 系统配置   | 冻结 |
| 相机配置   | 冻结 |
| 标定配置   | 冻结 |
| 测量配置   | 冻结 |
| 转台配置   | 冻结 |
| 验证配置   | 冻结 |
| 日志配置   | 冻结 |
| 配置版本追溯 | 冻结 |

---

本文档作为 AircraftPoseSystem V2.1 系统配置与参数管理设计基线。
