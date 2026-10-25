# SYS-17_系统配置与参数管理设计_V1.8（许可安装主体同步版）

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

# 1.1 V1.4 修订性质

V1.4 做**两件事**：**增益键的实质修正** ＋ **权威引用的同步**。配置的**加载流程、
只读原则、版本绑定、快照、校验**一律照旧。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 5.1 / 5.1.1 / **5.5（新增）** | 增益由**单个** `gain`（dB）改为**两个互斥键** `gain`（dB）／`gain_raw`（**设备原生值**），**两键都不给也合法**；示例按现状（CAM25 接实机）重写 | **兼容性变化** |
| 2 | 1.1 / 9.2 | 权威引用同步：**ENG-09 V2.5 → V2.6**；§9.2 的**当前取值**如实登记（`grab_timeout_ms: 500`） | 指针更新 ＋ 现状登记 |

⚠ **本节（§5 / §9）的键表是本工程的权威键表**；
键的**类型、默认值、校验规则**的权威是 **ENG-09 V2.11 §6.1（`CameraConfig`）
与 §6.5（`MeasurementConfig`）**（V2.7～V2.10 均未改这两节；V2.8 只改 §6.3 转台字段，
V2.9／V2.10 只补事务归属与验收许可类型），本文件只登记"有哪些键、含义与失败边界"。

> ⚠ **为什么必须同批改这一句（留痕）**：本句是**当前裁决规则**，不是历史修订记录 ——
> V1.3 写的是"权威是 ENG-09 **V2.5** §6.1 / §6.5"，而 ENG-09 已升 **V2.6**（V2.5 已入 `archive/`）。
> ⚠ 这一版**不是只动指针**：ENG-09 V2.6 **实质改动了 §6.1** —— 增益字段由
> `double gain` 改为 `std::optional<double> gain`，并**新增** `std::optional<double> gainRaw`；
> 若只改版本号而不改键表，读者会被指向一个**已经作废的增益契约**
> （"缺键即启动失败"在 V2.6 起不再成立）。故 §5.1／§5.1.1／§5.5 与本节同日修改。
> §6.5 在 V2.6 中**原位未动**（仍是 `MeasurementConfig`），**已逐节核对**。
> 修订记录里 V1.1／V1.2／V1.3 三行的版本指向是**当时的事实**，**保留不动**。

**V1.3 的留痕（保留在册，本次未动其字）**：

> V1.3 只做**一件事**：再次同步当前的权威引用。V1.2 的全部正文（`camera.yaml`／
> `measurement.yaml` 两张键表、§5.3／§5.4／§9.2 的新增小节）**一字未动**，
> 改的只是"以哪一版为准"的**现时声明**。
> ⚠ ENG-09 V2.5 改的是 §5.29 的**聚合规则**（判定类别与实际调用历史分别记录），
> **不涉及 §6.1／§6.5** ⇒ 两张键表**无实质冲突**，那一版只动指针。

**V1.1 引入的改动清单（保留在册，本版未动）**：

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 5.1 / 5.2 | `camera.yaml` 键表按**实际配置文件**重写（原示例含不存在的键名与结构） | 一致性修正 |
| 2 | 5.3 | **新增小节**：`backend` / `serial` 的失败边界与"不隐式默认"纪律 | 新增 |
| 3 | 5.4 | **新增小节**：`trigger_mode` 由旧数字值改三值字符串的**迁移规则** | **兼容性变化** |
| 4 | 9.2 | **新增小节**：`measurement.yaml` 新增 `grab_timeout_ms` / `grab_group_budget_ns` | 新增 |

⚠ **本版未改动**：§2~§4、§6~§8、§10~§19 的全部内容（含 §5.3／§5.4 两节原文）。
§9.2 的**键表与默认值**未动，只补了一段"当前配置的实际取值"。配置的**加载流程、
只读原则、版本绑定、快照、校验**一律照旧。

---

# 1.2 V1.5 修订性质

V1.5 只改**转台配置**：把 §8 的旧示例（`interface: pekod`、键名与
`TurntableConfig` 不符）换成**合法的键表**，并登记 Pelco-D/UDP 后端新增的
10 个通信键。其余配置（`system.yaml` / `camera.yaml` / `optical_rig.yaml` /
`trigger.yaml` / `measurement.yaml` / `validation.yaml` / `logging.yaml`）
**一字未动**。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | §8 | 旧示例与 `TurntableConfig` 不符（`interface` 不存在、单位为 360/60 的斜杠写法不合法），按实际 `config/turntable.yaml` 重写 | 一致性修正 |
| 2 | §8.2（新增） | `turntable.yaml` **合法键表**、默认值、校验规则 | 新增 |
| 3 | §8.3（新增） | `protocol` 取值与 `pelco_d_udp` 的通信键说明 | 新增 |
| 4 | §13 | 转台配置校验清单补上协议、端点与网络键 | 新增 |
| 5 | §21 | 新增 V1.5 修订记录 | 记录 |

⚠ **权威**：键的**类型、默认值、校验**以 **ENG-09 V2.11 §6.3（`TurntableConfig`，
裁决 C-22／C-23）** 为准；本节只登记“有哪些键、含义与失败边界”。

⚠ **仓库默认配置仍为 `protocol: "sdk"`**；`pelco_d_udp` 的实机 `host`
**不得写入版本库**，实机配置放仓库根 `验收证据/`（gitignored），
以 `argv[1]` 指定配置目录启动。详见 §8.3。

---

# 1.3 V1.6 修订性质

V1.6 只改**转台配置**：登记裁决 **C-23** 新增的 5 个停止/新鲜度键，
并明确 `sendPolicy` **没有配置键**。其余配置文件**一字未动**。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | §8.2 | 键表新增 `sample_stale_ms`、`stop_timeout_ms`、`stop_retry_count`、`stop_retry_window_ms`、`stop_backoff_ms`（默认值与校验） | 新增 |
| 2 | §8.3 | 明确只读发送门禁**不可由配置打开**：`sendPolicy` 不是 YAML 键，配置层固定构造 `ReadOnly` | 新增 |
| 3 | §13 | 转台配置校验清单补上 5 个新键的范围检查 | 新增 |
| 4 | §21 | 新增 V1.6 修订记录 | 记录 |

⚠ **权威**：5 个键的类型、默认值与取值范围以 **ENG-09 V2.11 §6.3** 为准。

⚠ **本节不得被理解为“新增了运动开关”**：C-23 的只读门禁是**代码结构性保证**，
不是配置项。YAML 中出现 `sendPolicy`／`motion_enabled`／`allow_motion` 之类键时，
解析器**不读取**它们（它们不在 §8.2 键表内），`sendPolicy` 仍固定为 `ReadOnly`。
⚠ 当前解析器不会因未登记键而拒绝启动（§8.2 已如实登记）；这些键打不开门禁，
但也不产生报错。

---

# 1.4 V1.7 修订性质

V1.7 只补**验收运动许可**的配置边界说明（裁决 **C-24**）：
`TurntableAcceptancePermit` **不是配置结构体、没有 YAML 键**，只由独立
验收工具在代码中安装。其余键表、默认值与校验**一字未动**。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 开篇 | 新增本修订性质 | 说明 |
| 2 | §8.3 | 明确许可不是配置键、生产装配无安装路径，验收工具的配置文件副本要求 | 新增 |
| 3 | §21 | 新增 V1.7 修订记录 | 记录 |

⚠ **本节不得被理解为“新增了运动配置”**：`sendPolicy` 仍固定 `ReadOnly`，
许可只在独立工具的进程内安装；把 `turntable.yaml` 写满也打不开生产运动。

# 1.5 V1.8 修订性质

V1.8 只同步**验收运动许可的安装主体**（裁决 **C-24**）：§8.3 把“只由独立
验收工具安装”改为“独立验收工具，或主程序显式 `--permit <file>` 的验收
入口”。许可仍不是配置键；默认生产启动（不带 `--permit`）仍没有安装路径。
键表、默认值与校验**一字未动**。

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 开篇 | 新增本修订性质 | 说明 |
| 2 | §8.3 | 许可安装主体补入主程序显式 `--permit <file>` 入口；明确默认生产启动仍无路径 | 一致性修正 |
| 3 | §21 | 新增 V1.8 修订记录 | 记录 |

⚠ **本节不得被理解为“新增了运动配置”**：`sendPolicy` 仍固定 `ReadOnly`，
`--permit` 是显式操作员动作而非 YAML 键；把 `turntable.yaml` 写满也打不开
默认生产启动的运动门禁。

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
⚠ **V1.4 再修**：V1.1 的示例虽键名正确，但取值仍是虚拟阶段的**占位值**
（`1280×1024` / `0.005 s` / `gain: 6.0`），与现场配置已不一致；本版按
**当前 `config/camera.yaml`** 重写示例（CAM25 已接实机 A7A20MU201，
CAM50／CAM100 仍为虚拟占位）。

示例（与当前 `config/camera.yaml` 一致）：

```yaml
---
cameras:
   -
      id: "cam25"            # 通道的**逻辑名**
      role: "CAM25"          # CAM25 / CAM50 / CAM100
      backend: "imv"         # virtual | imv
      serial: "FD88772AAK00078"   # 设备标签上的序列号（期望值）
      width: 4096
      height: 3000
      exposure_time: 0.05    # 单位 **秒**（＝ 50 ms，当前验收工作点）
      gain_raw: 1            # 增益的**设备原生值**（本机型无 dB 语义的 Gain 节点）
      trigger_mode: software # software | hardware | free_run
      focal_length: 0.025    # 单位 **米**
   -
      id: "cam50"            # 虚拟通道：serial 留空，且**不写任何增益键**
      role: "CAM50"
      backend: "virtual"
      serial: ""
      width: 1280            # 占位值（无设备可查询）
      height: 1024
      exposure_time: 0.003
      trigger_mode: software
      focal_length: 0.05
   -
      id: "cam100"
      role: "CAM100"
      # …与 cam50 同形，focal_length: 0.1
```

⚠ **示例里的 `4096×3000` / `0.05` / `gain_raw: 1` 是"当前验收工作点"的实测值，
不是定档的测量参数**（换目标、换光源、换镜头都要重测）；`cam50`／`cam100`
的取值是**虚拟占位值**，不得据它们判断实机链路已验收。

## 5.1.1 键表

| 键 | 含义 | 单位／取值 | 缺失或非法时 |
|---|---|---|---|
| `id` | 通道的**逻辑名**（`CameraChannel::cameraId`），与焦段绑定 | 字符串 | **启动失败** |
| `role` | 焦段角色 | `CAM25` / `CAM50` / `CAM100` | **启动失败** |
| `backend` | 后端类型 | `virtual` / `imv` | **启动失败**（见 §5.3） |
| `serial` | 设备标签上的序列号（**期望值**，用于绑定设备） | 字符串；`backend=imv` 时**必须非空** | **启动失败** |
| `width` / `height` | 分辨率 | 像素 | 启动失败 |
| `exposure_time` | 曝光 | **秒**（不是 ms/µs） | 启动失败 |
| `gain` | 增益，单位 **dB** | 浮点；**只对确实声明 dB 语义 `Gain` 节点**的设备可用 | **可缺失**（＝本项目不写该特性）；负值 ⇒ 启动失败；与 `gain_raw` **同时出现** ⇒ 启动失败（见 §5.5） |
| `gain_raw` | 增益的**设备原生数值** | 浮点，**本项目不做单位解释**，量程由设备裁决 | **可缺失**；负值 ⇒ 启动失败；与 `gain` **同时出现** ⇒ 启动失败（见 §5.5） |
| `trigger_mode` | 触发模式 | `software` / `hardware` / `free_run` | **启动失败**（见 §5.4） |
| `focal_length` | 焦距 | **米**（量级校验 `(0.001, 0.5]`） | 启动失败 |

⚠ **`id` 是逻辑名，不是设备序列号。** 设备身份在独立的 `serial` 键里。
若把逻辑名当序列号，`serial` 键就无处安放，设备身份也就无法与通道解耦
（换一台同型号相机要改 `id`，而 `id` 还被预览／落盘／界面引用）。

⚠ **单位陷阱三处**（写错不会有任何编译或运行错误，只会静默算错）：
`focal_length` 写成 `25.0` 会让距离估计式偏大 1000 倍进而**选错焦段**；
`exposure_time` 写成 ms 会让曝光差 1000 倍；
`gain`／`gain_raw` **量纲不同**（dB vs 设备原生值），互填不会报错但**语义已经变了**
—— 见 §5.5。

⚠ **增益是"两键互斥、且可都不给"的唯一一处**：其余键缺失即启动失败，
增益键不是（缺＝本项目不写该特性）。**这不是"增益可省"的宽松，是语义上的区分**，
详见 §5.5。

---

# 5.2 相机参数

包含：

* 分辨率；
* 曝光；
* 增益（**两个互斥的键**，见 §5.5）；
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

# 5.5 增益键的选择与迁移规则（V1.4 新增）

`camera.yaml` 里增益有**两个互斥的键**，**两个都不给也合法**：

| 键 | 单位 | 可用的设备 | 不给时 |
|---|---|---|---|
| `gain` | **dB** | 只对**确实声明 dB 语义 `Gain` 节点**的设备 | 本项目不写该特性 |
| `gain_raw` | **设备原生值**（本项目不解释单位） | 任何提供 `GainRaw` 的设备 | 本项目不写该特性 |

**三条硬规则**：

1. **同时给出即启动失败**。两者单位不同，同时存在时"实际写了多少增益"**没有唯一答案**
   —— 与其猜一个，不如拒绝这份配置。
2. **两个都不给是合法状态**，含义是"**本项目不写该特性，设备保持其当前值**"。
   这与"配 0 dB"是**两件事**：后者是**主动写入 0**。装配摘要会区分这两者，
   故**不得**为了"把配置写全"而随手补一个 0。
3. **负值在本地拒绝**；取值范围**不由本项目判定**，一律由**设备裁决**
   （越界返回设备错误码，本项目如实上报）。设备量程随机型变化，
   本地写死一个范围会把"设备不支持这个值"说成"配置非法"，掩盖真实原因。

⚠ **迁移时不得把 dB 数值照抄进 `gain_raw`**：`gain = 6.0` 的含义是 **6 dB**；
把 `6.0` 原样填进 `gain_raw` 是**换了一个量纲**，写进去的可能是完全不同的增益。
反过来也要说清：**"写 6、读回 6"只证明写入成功，证明不了单位是 dB**
—— 读回值一致与单位语义是两件事，后者要查设备描述文件。

⚠ **本工程当前机型（A7A20MU201，序列号 `FD88772AAK00078`）只能用 `gain_raw`**：
实测**没有 `Gain` 节点**（`IMV_GetDoubleFeatureValue("Gain")` 返回 **−110**），
只有 `GainRaw`（Float，量程 1..32，节点内**没有 `<Unit>` 子节点**，且 SDK 未提供步长读取）
⇒ 其含义只能按**原生数值**理解。
⚠ **不得把这段简化成"XML 里没有 dB"——那句话是假的**：整份 XML 里"dB"确实出现过，
但它属于 `DeEmphasisA`（以太网预加重，**传输层**信号完整性设置）的枚举说明，
与该机型的增益**无关**。
⚠ 该结论**只对本次实机与本次固件成立**；**标定／换相机时必须重新实测，不得沿用本段**。

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

⚠ **V1.5 更正**：V1.4 的示例（`interface: pekod`、`azimuth_limit: 360`、
`elevation_limit: 60`）与实际 `TurntableConfig` **不符** —— 既没有
`interface` 键，也没有 `azimuth_limit` / `elevation_limit` 键；这些键在
`ConfigManager` 中会被忽略或触发告警。现按实际配置文件重写。

示例（与仓库 `config/turntable.yaml` 一致；默认协议仍为 `sdk`）：

```yaml
%YAML:1.0
---
turntable:
  protocol: "sdk"          # "sdk" | "pelco_d_udp" | "rs485" | "network"
  azimuth_min: -180.0      # deg
  azimuth_max: 180.0       # deg
  elevation_min: -60.0     # deg
  elevation_max: 60.0      # deg
  coarse_speed: 5.0        # deg/s
  fine_speed: 1.2          # deg/s
  center_threshold: 50.0   # pixel
```

---

包含：

* 通信方式；
* 轴限制；
* 速度限制。

---

## 8.2 合法键表

| YAML 键 | 类型 | 默认 | 校验 | 适用协议 |
|---|---|---|---|---|
| `protocol` | string | `"sdk"` | 仅 `{"sdk","pelco_d_udp","rs485","network"}`；`"pekod"` 已移除，报错并提示改为 `pelco_d_udp` | 全部 |
| `azimuth_min` | double | `-180.0` | 必须 `< azimuth_max`，且在 ±180 内 | 全部 |
| `azimuth_max` | double | `180.0` | 同上 | 全部 |
| `elevation_min` | double | `-60.0` | 必须 `< elevation_max`，且在 ±60 内 | 全部 |
| `elevation_max` | double | `60.0` | 同上 | 全部 |
| `coarse_speed` | double | — | 1.2～30 deg/s，且 `>= fine_speed` | 全部 |
| `fine_speed` | double | — | 1.2～30 deg/s，且 `<= coarse_speed` | 全部 |
| `center_threshold` | double | `50.0` | `> 0`，单位 pixel | 全部 |
| `host` | string | `""` | **非空**；`inet_pton(AF_INET)` 可解析；拒绝 `0.0.0.0`、组播、广播；允许 `127.0.0.1` | 仅 `pelco_d_udp` |
| `port` | int | `6666` | 1～65535 | 仅 `pelco_d_udp` |
| `pelco_address` | int | `1` | 1～255；**Pelco-D 设备地址，不是 IP 地址** | 仅 `pelco_d_udp` |
| `reply_timeout_ms` | int | `50` | 10～1000 | 仅 `pelco_d_udp` |
| `poll_interval_ms` | int | `50` | 20～1000 | 仅 `pelco_d_udp` |
| `stable_read_count` | int | `3` | 2～20 | 仅 `pelco_d_udp` |
| `stable_threshold_deg` | double | `0.02` | 0.001～1.0 | 仅 `pelco_d_udp` |
| `arrival_tolerance_deg` | double | `0.2` | 0.02～1.0；与 SYS-10 §3.3 的 ±0.2° 对齐 | 仅 `pelco_d_udp` |
| `comm_failure_threshold` | int | `5` | 1～20（50 ms × 5 = 250 ms） | 仅 `pelco_d_udp` |
| `use_speed_control` | bool | `false` | 布尔；`false` = 设备默认速度（表 3.5），`true` = 带转速定位（表 3.65） | 仅 `pelco_d_udp` |
| `sample_stale_ms` | int | `500` | 50～5000；样本超过此时限未更新 ⇒ stale／`ERROR`，上层不得采集（C-23） | 仅 `pelco_d_udp` |
| `stop_timeout_ms` | int | `3000` | 100～30000；停止帧交付后等待“观测到静止”的窗口（C-23） | 仅 `pelco_d_udp` |
| `stop_retry_count` | int | `2` | 0～5；**首次失败后的重发次数**，总发送次数 ≤ `1 + 本值`（C-23） | 仅 `pelco_d_udp` |
| `stop_retry_window_ms` | int | `2000` | 100～10000；全部停止发送尝试须在此窗口内完成（C-23） | 仅 `pelco_d_udp` |
| `stop_backoff_ms` | int | `200` | 0～1000；停止重发间隔（C-23） | 仅 `pelco_d_udp` |

⚠ **网络键的读取策略**：只有 `protocol == "pelco_d_udp"` 时才读取并校验上表
后 15 个键；其他协议下不读取（不产生“取默认值”的记录）。若其他协议下显式
出现这些键，给 warning：“仅 pelco_d_udp 使用”。

⚠ **没有 `send_policy` 键**（C-23）：只读发送门禁由代码固定构造为
`TurntableSendPolicy::ReadOnly`。任何 `send_policy`／`motion_enabled`／
`allow_motion` 之类的键都**不在这张合法键表里**，ConfigManager 不读取它们，
`sendPolicy` 仍为 `ReadOnly` —— **配置无法打开运动门禁**。
⚠ **如实登记现状**：当前解析器对未登记的键**没有统一的拒绝机制**，这类键会被
静默忽略，不会导致启动失败。它们打不开门禁，但也不会报错；因此**不得**把
“配置里写了运动开关”当作“运动已启用”的证据。若将来需要“未知键即失败”，
须另立裁决并补全所有配置段的键白名单。

## 8.3 `protocol` 取值与失败边界

| 取值 | 装配结果 | 本批状态 |
|---|---|---|
| `"sdk"` | `VirtualTurntable` | 可用，**仓库默认** |
| `"pelco_d_udp"` | `PelcoDTurntableController`（真实后端） | 已实现，需填 `host` |
| `"rs485"` | — | 已登记、**本批未实现**：启动失败，**不回落虚拟后端** |
| `"network"` | — | 已登记、**本批未实现**：启动失败，**不回落虚拟后端** |

⚠ **不得静默回落**：除 `"sdk"` 外的协议若装配失败，必须报错并终止启动；
不得为了让程序跑起来而退回 `VirtualTurntable`。

⚠ **实机地址不入库**：仓库 `config/turntable.yaml` 中 `protocol` 保持
`"sdk"`，`host` 不出现。实机验证时把配置目录副本放
`<仓库根>/验收证据/turntable_pelcod_v1/config/`（gitignored），
以该目录作为 `argv[1]` 启动应用。

⚠ **验收运动许可不是配置键（V1.7／V1.8，C-24）**：`TurntableAcceptancePermit`
不进入 `TurntableConfig`，也不在 §8.2 键表中；`acceptance_permit`、
`allow_motion`、`max_executions` 之类 YAML 键都**不会**被读取，写它们
打不开运动。许可的安装主体只有**独立验收工具 `turntable_acceptance`**，
或**主程序显式 `--permit <file>` 的验收入口**；两者都在打开 socket 之前
由代码安装到真实后端。默认生产应用（不带 `--permit`）**没有安装路径**；
`--permit` 是显式操作员动作，不是配置键。许可文件的设备三项必须与该次
启动的配置一致，不匹配即拒绝安装、控制帧为 0。许可文件另含
`allow_stop_at_rest`（T1 静止停止帧授权），同样是**许可文件键**而非 YAML
配置键；缺失即解析失败，`0` 时静止停止帧被拒绝。验收工具同样以配置目录
副本作为 `--config` 参数，原始载荷按证据约定存放，不进入版本库。

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

⚠ **当前配置的实际取值（V1.4 登记现状，键表默认值不变）**：
本工程当前 `config/measurement.yaml` 把 `grab_timeout_ms` 设为 **`500`**，
`grab_group_budget_ns` 保持 **`3.0e8`**。上表"默认值"列写的是**代码默认**（`100`），
**当前配置不是默认值** —— 引用本表时不要把它当成"实际生效值"。
改值的依据是实机实测：A7A20MU201 满幅 4096×3000 / `Mono10`、50 ms 曝光下
**单帧约 141 ms** > 旧的 100 ms，旧值下每一轮取帧都记 `Timeout`。
⚠ 该 141 ms **只适用于该配置与该测量路径**，**不可外推成持续采集／预览／保存的帧率**；
取值来源与限定见 `config/measurement.yaml` 同键注释。

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

## 转台配置（V1.5 新增）

检查：

* `protocol` 在合法取值内；`"pekod"` 给出迁移提示；
* 行程：`azimuth_min < azimuth_max`、`elevation_min < elevation_max`，且在冻结范围内；
* 速度：`1.2 <= fine_speed <= coarse_speed <= 30`（deg/s）；
* `center_threshold > 0`；
* 仅 `pelco_d_udp`：`host` 非空且可解析为 IPv4 单播地址（非 `0.0.0.0`、
  非组播、非广播）；`port` 1～65535；`pelco_address` 1～255；
  `reply_timeout_ms` 10～1000；`poll_interval_ms` 20～1000；
  `stable_read_count` 2～20；`stable_threshold_deg` 0.001～1.0；
  `arrival_tolerance_deg` 0.02～1.0；`comm_failure_threshold` 1～20；
  `use_speed_control` 为布尔；
* 仅 `pelco_d_udp`（V1.6 新增，C-23）：`sample_stale_ms` 50～5000；
  `stop_timeout_ms` 100～30000；`stop_retry_count` 0～5；
  `stop_retry_window_ms` 100～10000；`stop_backoff_ms` 0～1000。
* **不存在** `send_policy` 键：`sendPolicy` 由代码固定为 `ReadOnly`，
  配置校验不提供任何打开运动门禁的路径。

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

本文档作为 AircraftPoseSystem V1.8 系统配置与参数管理设计基线。

---

# 21 修订记录

| 版本 | 日期 | 修订性质 | 备注 |
|---|---|---|---|
| V1.0 | — | 多相机闭环测量版 | 旧版已归档至 `archive/SYS-17_系统配置与参数管理设计_V1.0（多相机闭环测量版）.md` |
| **V1.1** | 2026-09-26 | **A1 数据契约版** | §5.1 键表按实际配置重写；新增 §5.3（`backend`/`serial` 失败边界）、§5.4（`trigger_mode` 迁移）、§9.2（取帧预算键）。键的类型与校验权威为 ENG-09 V2.3 §6.1 / §6.5 |
| **V1.2** | 2026-09-26 | **权威引用同步版** | **只改当前权威声明**：§1.1 里"权威是 ENG-09 V2.3 §6.1 / §6.5"⇒ **V2.4**（旧版已入 `archive/`，此前已逐节核对 §6.1 与 §6.5 在 V2.4 中原位保留）。⚠ **键表与全部正文一字未动**；修订记录里 V1.1 行的版本指向作为**当时的事实保留**。依据＝**C-016**（011-A1 九项缺口定向修复批）的文档一致性要求 |
| **V1.3** | 2026-09-26 | **权威引用同步版** | **只改当前权威声明**：§1.1 里"权威是 ENG-09 **V2.4** §6.1 / §6.5"⇒ **V2.5**（实测 **1 处**；§6.1 与 §6.5 两个节号在 V2.5 中原位保留，已逐节核对）。ENG-09 V2.5 改的是 §5.29 的**聚合规则**，**不涉及**这两节 ⇒ 两张键表**无实质冲突**。⚠ **键表与全部正文一字未动**；修订记录里 V1.1／V1.2 两行的版本指向作为**当时的事实保留**。依据＝**C-01 v1.9**（011-A1 显示职责与清理确认批）的文档一致性要求 |
| **V1.4** | 2026-09-28 | **增益双键与权威引用同步版** | ① §5.1.1 键表：`gain` 由"缺失即启动失败"改为**可缺失**，**新增 `gain_raw`**（设备原生值），两键**互斥**；② §5.1 示例按**当前 `config/camera.yaml`** 重写（CAM25 接实机 A7A20MU201，原示例还是虚拟阶段的占位值）；③ **新增 §5.5**（增益键的选择与迁移规则，含"dB 数值不得照抄进 `gain_raw`"与本机型实测结论）；④ §9.2 补**当前配置实际取值**（`grab_timeout_ms: 500`，键表默认值不变）；⑤ §1.1 权威声明 **ENG-09 V2.5 ⇒ V2.6**（实测 **1 处**）。⚠ 本版**不是只动指针**：ENG-09 V2.6 **实质改了 §6.1**（`gain` 改 `optional` ＋ 新增 `gainRaw`），键表必须同批跟上。依据＝**C-01 v2.0**（2026-09-28，011-A1 实机接入与 RAW 旁路验证批） |
| **V1.5** | 2026-10-19 | **转台通信键表版** | ① §8.1 旧示例（`interface: pekod`、`azimuth_limit`/`elevation_limit`）与 `TurntableConfig` 不符，按实际 `config/turntable.yaml` 重写；② **新增 §8.2** 合法键表（含 `pelco_d_udp` 的 10 个通信键、默认值与校验）；③ **新增 §8.3** `protocol` 取值、失败边界与"实机地址不入库"；④ §13 补充转台配置校验清单。其余配置与正文未动。依据＝**C-22**（Pelco-D/UDP 转台后端接入批，ENG-09 V2.7） |
| **V1.6** | 2026-10-19 | **转台停止与新鲜度键版** | ① **新增 §1.3**；② §8.2 键表新增 5 个键：`sample_stale_ms`、`stop_timeout_ms`、`stop_retry_count`、`stop_retry_window_ms`、`stop_backoff_ms`（默认值与校验同 ENG-09 V2.8 §6.3）；③ §8.2／§8.3 明确**没有 `send_policy` 键**、只读门禁不可由配置打开，并如实登记"未登记键当前不拒绝启动"；④ §13 补 5 个新键的范围校验。其余配置未动。依据＝**C-23**（转台可信化批，ENG-09 V2.8） |
| **V1.7** | 2026-10-21（2026-10-22 补正） | **验收运动许可非配置键版** | ① **新增 §1.4**；② §8.3 明确 `TurntableAcceptancePermit` 不是配置键、生产装配无安装路径，许可只由独立验收工具 `--permit` 安装；**补正**：`allow_stop_at_rest` 是许可文件键而非 YAML 键，缺失即解析失败；③ 修订记录。**键表、默认值与校验一字未动**。依据＝**C-24**（转台实机验收入口与控制验证批，ENG-09 V2.10） |
| **V1.8** | 2026-10-26 | **许可安装主体同步版** | ① **新增 §1.5**；② §8.3 把许可安装主体同步为“独立验收工具，或主程序显式 `--permit <file>` 的验收入口”，并明确默认生产启动（不带 `--permit`）仍无安装路径；③ 修订记录。**键表、默认值与校验一字未动**。依据＝**C-24**（ENG-09 V2.11、SYS-04 V2.10） |

⚠ V1.7 已移入 `archive/SYS-17_系统配置与参数管理设计_V1.7_验收运动许可非配置键版.md`，
仅作历史留存；引用本文件时一律指向 **V1.8**。

**V1.4 未改动**：§2~§4、§6~§8、§10~§20 的全部内容（含 V1.1 批新增的 §5.3／§5.4 与
§9.2 的键表本体）—— 包括配置只读原则、版本绑定、目录结构、
`system.yaml` / `optical_rig.yaml` / `trigger.yaml` /
`turntable.yaml` / `validation.yaml` / `logging.yaml`、`ConfigManager` 设计、
配置校验、配置快照、配置加载流程、配置异常处理、设计冻结总结。

**V1.5 未改动**：§2~§7、§9~§12、§14~§20 的全部内容；V1.5 只改 §8（转台配置）
与 §13 的转台校验项。V1.4 行中“§8 未动”的描述作为**当时的事实保留**。

**V1.6 未改动**：§2~§7、§9~§12、§14~§20 的全部内容；V1.6 只改 §1（新增 §1.3）、
§8.2／§8.3 的转台键与门禁说明、§13 的转台校验项。V1.5 行中“其余配置与正文未动”
作为**当时的事实保留**。
