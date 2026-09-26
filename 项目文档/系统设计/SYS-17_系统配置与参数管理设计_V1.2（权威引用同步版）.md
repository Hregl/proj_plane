# SYS-17_系统配置与参数管理设计_V1.2（权威引用同步版）

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

# 1.1 V1.2 修订性质

V1.2 只做**一件事**：**同步当前的权威引用**。V1.1 的全部正文（`camera.yaml`／
`measurement.yaml` 两张键表、§5.3／§5.4／§9.2 的新增小节）**一字未动**，
改的只是"以哪一版为准"的**现时声明**。

⚠ **本节（§5 / §9）的键表是本工程的权威键表**；
键的**类型、默认值、校验规则**的权威是 **ENG-09 V2.4 §6.1（`CameraConfig`）
与 §6.5（`MeasurementConfig`）**，本文件只登记"有哪些键、含义与失败边界"。

> ⚠ **为什么必须同批改这一句（留痕）**：本句是**当前裁决规则**，不是历史修订记录 ——
> 旧文写"权威是 ENG-09 **V2.3** §6.1 / §6.5"，而 ENG-09 已升 **V2.4**（V2.3 已入 `archive/`）。
> 两版在配置键上若有出入，旧句会把读者**指回归档版本**。故同批改为 V2.4
> （§6.1 与 §6.5 两个节号在 V2.4 中原位保留，已逐节核对）；
> 修订记录里 V1.1 那一行的"权威为 ENG-09 V2.3"是**当时的事实**，**保留不动**。

**V1.1 引入的改动清单（保留在册，本版未动）**：

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 5.1 / 5.2 | `camera.yaml` 键表按**实际配置文件**重写（原示例含不存在的键名与结构） | 一致性修正 |
| 2 | 5.3 | **新增小节**：`backend` / `serial` 的失败边界与"不隐式默认"纪律 | 新增 |
| 3 | 5.4 | **新增小节**：`trigger_mode` 由旧数字值改三值字符串的**迁移规则** | **兼容性变化** |
| 4 | 9.2 | **新增小节**：`measurement.yaml` 新增 `grab_timeout_ms` / `grab_group_budget_ns` | 新增 |

⚠ **本版未改动**：§2~§4、§6~§8、§10~§19 的全部内容。配置的**加载流程、
只读原则、版本绑定、快照、校验**一律照旧。

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

管理**逐通道**的相机参数。

⚠ **V1.1 修正**：原示例写的是 `camera.count` / `camera.channels[].id` 一批
**在配置文件与 `CameraConfig` 中都不存在**的键名与层级，现按**实际配置文件**重写。

示例（与 `config/camera.yaml` 一致）：

```yaml
---
cameras:
   -
      id: "cam25"            # 通道的**逻辑名**
      role: "CAM25"          # CAM25 / CAM50 / CAM100
      backend: "virtual"     # virtual | imv
      serial: ""             # 设备标签上的序列号（期望值）
      width: 1280
      height: 1024
      exposure_time: 0.005   # 单位 **秒**
      gain: 6.0              # 单位 dB
      trigger_mode: software # software | hardware | free_run
      focal_length: 0.025    # 单位 **米**
   -
      id: "cam50"
      role: "CAM50"
      # …
```

## 5.1.1 键表

| 键 | 含义 | 单位／取值 | 缺失或非法时 |
|---|---|---|---|
| `id` | 通道的**逻辑名**（`CameraChannel::cameraId`），与焦段绑定 | 字符串 | **启动失败** |
| `role` | 焦段角色 | `CAM25` / `CAM50` / `CAM100` | **启动失败** |
| `backend` | 后端类型 | `virtual` / `imv` | **启动失败**（见 §5.3） |
| `serial` | 设备标签上的序列号（**期望值**，用于绑定设备） | 字符串；`backend=imv` 时**必须非空** | **启动失败** |
| `width` / `height` | 分辨率 | 像素 | 启动失败 |
| `exposure_time` | 曝光 | **秒**（不是 ms/µs） | 启动失败 |
| `gain` | 增益 | dB | 启动失败 |
| `trigger_mode` | 触发模式 | `software` / `hardware` / `free_run` | **启动失败**（见 §5.4） |
| `focal_length` | 焦距 | **米**（量级校验 `(0.001, 0.5]`） | 启动失败 |

⚠ **`id` 是逻辑名，不是设备序列号。** 设备身份在独立的 `serial` 键里。
若把逻辑名当序列号，`serial` 键就无处安放，设备身份也就无法与通道解耦
（换一台同型号相机要改 `id`，而 `id` 还被预览／落盘／界面引用）。

⚠ **单位陷阱两处**（写错不会有任何编译或运行错误，只会静默算错）：
`focal_length` 写成 `25.0` 会让距离估计式偏大 1000 倍进而**选错焦段**；
`exposure_time` 写成 ms 会让曝光差 1000 倍。

---

# 5.2 相机参数

包含：

* 分辨率；
* 曝光；
* 增益；
* 触发模式；
* 焦距。

（**键名与取值见 §5.1.1 键表**；原示例中的 `pixel_format` 键当前配置中不存在，
采集格式由后端在运行时确定并写入 `ImageFrame.captureFormat`，
**不作为配置项**。）

---

# 5.3 `backend` 与 `serial` 的失败边界（V1.1 新增）

**逐通道显式声明，缺键即配置错误、启动失败。**

⚠ **不设默认值，也不因"检测到 SDK"自动切换** —— 隐式默认会让一份虚拟配置
在 SDK 装好后**静默**变成真实采集；同理，**不得**在真实相机打开失败后
静默换虚拟。

**`backend` 的两种取值**：

| 取值 | 含义 |
|---|---|
| `virtual` | 使用 `VirtualCameraBackend`（软件开发与测试）；`serial` 留空 |
| `imv` | 使用 `ImvCameraBackend`（真实设备）；`serial` **必须非空** |

⚠ **显式请求 `imv` 而打开失败 ⇒ 启动失败、明确报告接入失败**；
**不能**靠另外两路虚拟相机把这次真实模式判为启动成功。
（三台相机同型号，按型号或"第 0 个设备"打开会在换了线序之后静默接错焦段，
故按 `serial` **精确匹配**；匹配到 0 个或**多个**都明确失败。）

---

# 5.4 `trigger_mode` 的迁移规则（V1.1 新增）

**旧版是数字值**（`1` = 硬触发、`0` = 软触发），**本版起是三值字符串**。

⚠ **旧数字值显式拒绝并给迁移提示**，**不自动映射**：
旧 `1` 的语义是"开触发"，而开触发之后还分软件源与硬件源，
自动映射到任一边都可能把另一边的意图**悄悄改掉**。

| 新取值 | 含义 | 本批状态 |
|---|---|---|
| `software` | 软触发 | **本批 CAM25 采用** |
| `hardware` | 硬触发 | 归 **PH-01**，未实施 |
| `free_run` | 自由运行 | 可表达；真实后端返回 `NotImplemented`，**不静默当自由运行** |

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
* 测量模式；
* **取帧预算**（见 §9.2）。

---

## 9.2 取帧预算键（V1.1 新增）

| 键 | 含义 | 默认值 | 校验 |
|---|---|---|---|
| `grab_timeout_ms` | **单次**取帧等待上限（ms），传给后端 → SDK 的 `IMV_GetFrame` | `100` | **必须为正**：`0` 的语义 SDK 未文档化、项目不定义，后端**拒绝 0** |
| `grab_group_budget_ns` | **一次 `capture()`（三路一轮）** 的总预算（ns） | `3.0e8`（300 ms） | 为正 |

⚠ **两者都只是上限，不是承诺**：实际等待取
"`grab_timeout_ms` / 组剩余 / 距状态期限的剩余"三者中**最小者**，
故调大它们**不会**延长任何一次等待。

⚠ **三路合计超过组预算时**，后面的路以"**预算耗尽**"记 `Timeout`，
且**既不发软件触发令也不取帧** —— 不发令是硬要求：否则会留下一个
收不回帧的**悬空触发**。

⚠ **这两个值与既有的时限体系零余量**（`capture_frame_count 5 × 300 ms = 1.5 s`
＝ `capture_timeout_ns`，复制／转换／评分未计入）⇒ CAPTURE 在真实相机上
**可能被时限截断**。该事实已登记为待裁决项 **Q-D2**，本版**只如实记录、不自行改冻结值**。

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

本文档作为 AircraftPoseSystem V1.2 系统配置与参数管理设计基线。

---

# 21 修订记录

| 版本 | 日期 | 修订性质 | 备注 |
|---|---|---|---|
| V1.0 | — | 多相机闭环测量版 | 旧版已归档至 `archive/SYS-17_系统配置与参数管理设计_V1.0（多相机闭环测量版）.md` |
| **V1.1** | 2026-09-26 | **A1 数据契约版** | §5.1 键表按实际配置重写；新增 §5.3（`backend`/`serial` 失败边界）、§5.4（`trigger_mode` 迁移）、§9.2（取帧预算键）。键的类型与校验权威为 ENG-09 V2.3 §6.1 / §6.5 |
| **V1.2** | 2026-09-26 | **权威引用同步版** | **只改当前权威声明**：§1.1 里"权威是 ENG-09 V2.3 §6.1 / §6.5"⇒ **V2.4**（旧版已入 `archive/`，此前已逐节核对 §6.1 与 §6.5 在 V2.4 中原位保留）。⚠ **键表与全部正文一字未动**；修订记录里 V1.1 行的版本指向作为**当时的事实保留**。依据＝**C-016**（011-A1 九项缺口定向修复批）的文档一致性要求 |

**V1.2 未改动**：§2~§20 的全部内容（含 V1.1 批新增的 §5.3／§5.4／§9.2）—— 包括配置只读原则、
版本绑定、目录结构、`system.yaml` / `optical_rig.yaml` / `trigger.yaml` /
`turntable.yaml` / `validation.yaml` / `logging.yaml`、`ConfigManager` 设计、
配置校验、配置快照、配置加载流程、配置异常处理、设计冻结总结。
