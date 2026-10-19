# SYS-18_系统部署与运行环境设计_V1.1（部署阶段补充版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统部署环境、软件安装、运行配置和离线部署规范。

**V1.1 补充**：新增 §3 部署阶段，区分**开发部署**（全虚拟 / CAM25 单真实相机）
与**最终部署**（三真实相机 + Calibration Package + Production Model）。
V1.0 只描述了最终部署形态，开发部署在文档中没有任何位置。**部署规范本身未变。**

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

# 3 部署阶段

系统按阶段推进 —— 阶段定义与完成判据见 SYS-01 §24，各阶段的需求范围见 SYS-02 §5。
**本文档规定的部署形态按阶段不同。**

```text id="d5n1ka"

开发部署                    最终部署

（Phase 1 / Phase 2）        （Phase 4 交付）

↓                           ↓

全虚拟相机                   三真实相机

或 CAM25 单真实相机           + 真实转台

                             + 硬件外触发

                             + Calibration Package

                             + Production Model

```

---

## 3.1 开发部署

用于 Phase 1 / Phase 2 的开发与调试。

| 项 | 形态 |
| --- | --- |
| 相机 | `VirtualCameraBackend`（全虚拟），**或** CAM25 单真实相机 + 其余虚拟 |
| 转台 | `VirtualTurntable` |
| 触发 | `VirtualTriggerController` |
| 标定 | **无真实标定**（合成标定，ID 为 `SYNTHETIC-NO-CALIBRATION`） |
| 模型 | **无真实机型库**（该交付物在 Phase 1 / Phase 2 尚不存在） |
| 运行形态 | 开发机原地运行；**离线部署包尚不适用** |

⚠ **开发部署下 §13 的四项环境检查必然不全通过** —— 标定、模型、设备都会报缺失。

这是**如实反映现实**，不是缺陷。∴ §13 的检查在开发部署下应作为
**诊断输出**读取，**不得当作启动门槛**；把开发部署做成"检查全绿才算正常"
会让这套检查本身失去意义 —— 一按就红的门槛，最后只会被绕过。

---

## 3.2 最终部署

用于 Phase 4 交付。

| 项 | 形态 |
| --- | --- |
| 相机 | **三真实相机**（CAM25 / CAM50 / CAM100，A7A20MU201） |
| 转台 | 真实转台（Peko_D 协议） |
| 触发 | 硬件外触发 |
| 标定 | **Calibration Package** 就位（§9 `calibration/`） |
| 模型 | **Production Model** 就位（§10 `models/`） |
| 运行形态 | 离线部署包，现场解压即启动（§14 / §15） |

⚠ **最终部署禁止加载 `synthetic model`**（SYS-02 §13.2）。

该约束在部署侧的表现是：§13 的**模型检查必须能区分"生产模型"与"synthetic model"**，
不能只检查文件是否存在 —— 只查存在性的话，一份合成模型同样能让检查全绿。

---

## 3.3 两阶段之间

从开发部署到最终部署是**逐件替换**，不是一次切换
（SYS-01 §24.2：**先单件，后组合**）。

∴ 中间形态（CAM25 真实 + 转台 / 触发 / 其余相机虚拟）是**合法的开发部署形态**，
但**不得作为交付形态**：它既没有真实标定，也没有生产模型。

---

## 3.4 ⚠ 当前代码事实

`SystemInitializer::buildDevices()` 当前装配的是**全虚拟**：

| 位置 | 当前装配 | 真实后端的状态 |
| --- | --- | --- |
| 三路相机 | `VirtualCameraBackend` ×3 | `ImvCameraBackend`：**诚实桩**（SDK 适配未实现） |
| 触发 | `VirtualTriggerController` | `HardwareTriggerController`：**诚实桩** |
| 转台 | `VirtualTurntable` | `PekoTurntableController`：**诚实桩** |

∴ 当前既不是 §3.1 的第二种形态（CAM25 单真实相机），也不是 §3.2 的最终部署，
而是 **§3.1 的第一种形态（全虚拟）**。

详见 SYS-02 V2.2 §5.4。

---

# 4 软件运行环境

## 4.1 操作系统

支持：

```text id="0a4w6m"
UOS
```

---

## 4.2 编译环境

要求：

* C++17；
* CMake；
* GCC。

---

## 4.3 图形环境

要求：

* Qt运行环境；
* GPU显示支持。

---

# 5 部署目录结构

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

# 6 bin目录

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

# 7 lib目录

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

# 8 config目录

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

# 9 calibration目录

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

# 10 models目录

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

# 11 版本管理

目录：

```text id="p3n8mq"
version/

├── software_version.txt

├── calibration_version.txt

└── model_version.txt

```

---

# 12 启动流程

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

# 13 环境检查

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

# 14 离线部署流程

## 14.1 开发机

完成：

```text id="d9m3kf"
编译

↓

测试

↓

打包

```

---

## 14.2 部署机

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

# 15 部署包内容

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

# 16 SDK部署

## 16.1 ImvSDK

真实相机部署时：

必须包含：

* SDK动态库；
* 配置；
* 运行环境。

---

## 16.2 SDK版本绑定

记录：

```text id="p4m8zx"
ImvSDK Version
```

---

禁止：

运行环境自动升级SDK。

---

# 17 日志管理

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

# 18 数据存储

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

# 19 故障恢复

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

# 20 部署测试

## 20.1 启动测试

验证：

* 程序启动；
* 界面显示。

---

## 20.2 设备测试

验证：

* 相机连接；
* 图像采集。

---

## 20.3 测量测试

验证：

* 状态机流程；
* 结果保存。

---

# 21 版本发布流程

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

# 22 设计冻结总结

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

---

# 23 修订记录

| 版本 | 日期 | 性质 | 变动 |
| --- | --- | --- | --- |
| V1.0 | — | 基线 | UOS 离线部署版初版 |
| **V1.1** | 2026-09-24 | **增量补充** | 新增 §3 部署阶段；原 §3~§21 顺延为 §4~§22 |

---

## 23.1 本版变动说明

**部署规范本身未变。** 本版补的是**此前整份文档缺失的一个维度**：

实测 V1.0 全文对「虚拟件 / 阶段 / 开发部署 / 生产模型」**零命中** ——
即整份文档默认的就是**最终部署形态**（三真实相机 + 真实标定 + 生产模型），
而 Phase 1 / Phase 2 实际跑的**全虚拟**形态在部署文档里**没有任何位置**。

后果与本项目反复出现的缺陷形态同类：**文档描述的是未来的样子**，
于是 §13 的环境检查在开发部署下一按就红，而文档没有一处说明"这是正常的"。

本版新增 §3 部署阶段，规定：

* **§3.1 开发部署** —— 全虚拟，或 CAM25 单真实相机；标定与模型均无真实件；
* **§3.2 最终部署** —— 三真实相机 + 真实转台 + 硬触发 + Calibration Package + Production Model；
* **§3.3 两阶段之间** —— 逐件替换，中间形态合法但不得作为交付形态；
* **§3.4 当前代码事实** —— 当前为**全虚拟**，且三个真实后端**均为诚实桩**。

---

## 23.2 版本关系

* 有效版本：`SYS-18_系统部署与运行环境设计_V1.1（部署阶段补充版）.md`（本文件）
* 归档版本：`archive/SYS-18_系统部署与运行环境设计_V1.0（UOS离线部署版）.md`

项目无版本管理仓库，旧版**归档保留、不删除**，以保留审计链。
