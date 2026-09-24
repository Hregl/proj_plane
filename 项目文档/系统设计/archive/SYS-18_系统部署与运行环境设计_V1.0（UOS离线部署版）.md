# SYS-18_系统部署与运行环境设计_V1.0（UOS离线部署版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统部署环境、软件安装、运行配置和离线部署规范。

目标：

保证系统能够在目标工控机环境中：

* 独立运行；
* 稳定启动；
* 参数可恢复；
* 数据可追溯。

---

# 2 部署原则

## 2.1 离线部署原则

系统面向：

* 舰载环境；
* 工业现场；
* 无公网环境。

部署包必须包含：

* 可执行程序；
* 依赖库；
* 配置文件；
* 标定文件；
* 模型文件。

---

## 2.2 环境一致性原则

开发环境与部署环境保持：

* 操作系统一致；
* 编译器一致；
* 依赖版本一致。

---

# 3 软件运行环境

## 3.1 操作系统

支持：

```text id="0a4w6m"
UOS
```

---

## 3.2 编译环境

要求：

* C++17；
* CMake；
* GCC。

---

## 3.3 图形环境

要求：

* Qt运行环境；
* GPU显示支持。

---

# 4 部署目录结构

冻结：

```text id="n8m5q2"
AircraftPoseSystem/

├── bin/

│   └── AircraftPoseSystem


├── lib/


├── config/


├── calibration/


├── models/


├── logs/


├── data/


└── version/

```

---

# 5 bin目录

包含：

主程序：

```text id="c6v9m3"
AircraftPoseSystem
```

---

启动：

```bash id="y2p8nk"
./AircraftPoseSystem
```

---

# 6 lib目录

保存：

运行依赖：

包括：

* Qt库；
* OpenCV库；
* SDK动态库。

---

目录：

```text id="f7q3ma"
lib/

├── Qt

├── OpenCV

└── ImvSDK

```

---

# 7 config目录

保存：

系统配置。

结构：

```text id="q6m8vx"
config/

├── system.yaml

├── camera.yaml

├── optical_rig.yaml

├── measurement.yaml

├── validation.yaml

├── trigger.yaml

└── turntable.yaml

```

---

# 8 calibration目录

保存：

标定数据。

结构：

```text id="w4m9py"
calibration/

├── CAM25/

├── CAM50/

├── CAM100/

└── rig_to_ship.yaml

```

---

包含：

* 内参；
* 畸变；
* Camera→Rig；
* Rig→Ship。

---

# 9 models目录

保存：

目标模型。

结构：

```text id="h5q7mx"
models/

└── aircraft/

    ├── model.yaml

    ├── points3d.yaml

    ├── feature25.bin

    ├── feature50.bin

    └── feature100.bin

```

---

# 10 版本管理

目录：

```text id="p3n8mq"
version/

├── software_version.txt

├── calibration_version.txt

└── model_version.txt

```

---

# 11 启动流程

系统启动：

```text id="m8x2qz"
main.cpp

↓

Logger初始化

↓

Config加载

↓

Calibration加载

↓

Model加载

↓

Device初始化

↓

Preview启动

↓

Application启动

↓

MainWindow显示

```

---

# 12 环境检查

启动前检查：

## 软件版本

检查：

* 软件版本；
* 配置版本。

---

## 标定检查

检查：

* Calibration ID；
* 文件完整。

---

## 模型检查

检查：

* Model ID；
* 特征库。

---

## 设备检查

检查：

* 相机连接；
* 转台连接。

---

# 13 离线部署流程

## 13.1 开发机

完成：

```text id="d9m3kf"
编译

↓

测试

↓

打包

```

---

## 13.2 部署机

复制：

```text id="q7x3mv"
AircraftPoseSystem/
```

---

执行：

```bash id="a5n8qk"
./AircraftPoseSystem
```

---

# 14 部署包内容

必须包含：

## 程序

* executable；
* dynamic libraries。

---

## 配置

* yaml文件。

---

## 标定

* calibration package。

---

## 模型

* target model。

---

## 文档

包含：

* 版本说明；
* 使用说明；
* 更新记录。

---

# 15 SDK部署

## 15.1 ImvSDK

真实相机部署时：

必须包含：

* SDK动态库；
* 配置；
* 运行环境。

---

## 15.2 SDK版本绑定

记录：

```text id="p4m8zx"
ImvSDK Version
```

---

禁止：

运行环境自动升级SDK。

---

# 16 日志管理

日志目录：

```text id="s8q3my"
logs/

├── system.log

├── device.log

├── algorithm.log

└── error.log

```

---

记录：

* 启动信息；
* 设备状态；
* 测量过程；
* 错误信息。

---

# 17 数据存储

测量数据：

```text id="q2m7px"
data/

└── measurement_xxx/

```

---

包含：

* 原始图像；
* result.json；
* 配置快照；
* 标定信息。

---

# 18 故障恢复

## 软件异常

保存：

* 日志；
* 错误码。

---

## 设备异常

执行：

* 重连；
* 停止任务。

---

## 配置异常

禁止启动。

---

# 19 部署测试

## 19.1 启动测试

验证：

* 程序启动；
* 界面显示。

---

## 19.2 设备测试

验证：

* 相机连接；
* 图像采集。

---

## 19.3 测量测试

验证：

* 状态机流程；
* 结果保存。

---

# 20 版本发布流程

流程：

```text id="r7m4qx"
开发版本

↓

测试版本

↓

发布版本

↓

部署包

```

---

发布包绑定：

```text id="k9n2mv"
Code Commit

+

Config

+

Calibration

+

Model

+

Test Report

```

---

# 21 设计冻结总结

| 项目    | 状态 |
| ----- | -- |
| 部署目录  | 冻结 |
| UOS环境 | 冻结 |
| 离线部署  | 冻结 |
| 配置管理  | 冻结 |
| 标定管理  | 冻结 |
| 模型管理  | 冻结 |
| 日志管理  | 冻结 |
| 版本追溯  | 冻结 |

---

本文档作为 AircraftPoseSystem V2.1 系统部署与运行环境设计基线。
