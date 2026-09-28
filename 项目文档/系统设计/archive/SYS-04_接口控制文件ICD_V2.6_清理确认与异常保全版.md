# SYS-04_ICD接口控制文件_V2.6（清理确认与异常保全版）

---

# 1 文档目的

本文档定义 AircraftPoseSystem V2.1 系统模块接口控制规范。

本版本针对：

* 三相机固定光机结构；
* CalibrationManager；
* TargetModelManager；
* MeasurementRecord；
* FailureTrace；
* Recorder；
* 真实硬件接入；

进行接口补充。

---

# 1.1 V2.3 修订性质

V2.3 是**代码一致性修正版**。不改变软件架构、不新增接口、不改变代码。

V2.2 与代码基线的一致性经逐条审计（见
`AircraftPoseSystem/V2.1-SYS04不一致逐条对照.md`）：21 个接口声明点中
**12 处与代码不符**，其中 **5 处使用了代码中完全不存在的类型名**。

V2.3 按以下原则修正：

1. **ICD 描述真实存在的接口，不描述预期接口。**
   凡文档声明而代码不存在的实体，一律修正为代码实际形态 ——
   **是文档向代码收敛，不是把代码改向文档**；
2. **不新增接口。** 修正只做替换、改名、删除，不做扩充；
3. **不保留包装类型。** 同一实体在文档与代码中只允许存在一个名字。

⚠ 本次修正是**单向**的。修正后代码侧不产生任何改动；
`AircraftPoseSystem/src/` 与 `SYS-04 V2.2` 的不一致处以本版为准。

---

# 1.2 本版修正清单

| # | 节 | 修正内容 | 裁决 |
|---|---|---|---|
| 1 | 4.1 | 删 `gain` / `valid`（代码无此二字段） | 一致性 |
| 2 | 5.1 | 增 `lastError()`（代码有，文档漏列） | 一致性 |
| 3 | 9.1 | 删 `getCameraToRig()`；`getCameraCalibration` → `getCalibration`；`RigShipTransform` → `Transform` | 裁决 2/3 |
| 4 | 10.1 | `getModel()` → `get(CameraRole)`；`modelType()` → `modelVersion()` | 一致性 |
| 5 | 11.1 | `transform(Transform)` → `transform(CameraRole, const CameraPose&)` | 一致性 |
| 6 | 12.1 | `DetectionResult detect(ImageFrame&)` → `bool detect(const ImageFrame&, DetectionResult&)` | 一致性 |
| 7 | 12.2 | `ScaleEstimate` → `TargetScaleEstimate`；补 `CameraCalibration` / `targetRealSizeM` 入参 | 裁决 5 |
| 8 | 12.3 | `CameraRole select(vector<...>)` → `bool select(const vector<...>&, MeasurementSelectionResult&)` | 一致性 |
| 9 | 13.1 | `process(CalibrationPackage, …)` → 实际 8 方法接口；`CalibrationPackage` → `OpticalRigCalibration` | 裁决 1 |
| 10 | 15.1 | 删 `lastErrorText()` | 已裁决 |
| 11 | 16 | 补 `selectedCamera` / `selectedScore` / `selectedQuality` / `degraded` / `camerasAvailable` / `calibrationId` / `softwareVersion` | 一致性 |
| 12 | 18.1 | 补公开面（22 项） | 一致性 |
| 13 | 20.1 | 增 `PreviewFrame` 为 `ImageFrame` 显示层包装的说明 | 裁决 6 |
| 14 | 21 | `ErrorCode` → 标注为逻辑概念，当前实现为 `int` 段码 | 裁决 4 |
| 15 | 9.1 / 10.1 / 11.1 | 三个虚构接口类名收敛为具体类名（去 `I` 前缀） | 已裁决 |
| 16 | 26 | 新增「待处置项」；§26.2 已于本版内裁决并实施 | — |

**未改动**：6.1 `ICameraBackend`、7.1 `ITriggerController`、8.1 `ITurntableController`、
14.1 `IPipelineObserver`、17 `FailureTrace`、19 `AlignmentController`、23 禁止接口 ——
以上 7 节经审计与代码**逐字段/逐方法吻合**。

---

# 1.3 V2.4 修订性质

V2.4 是 **011-A1 数据契约批**（裁决 **C-015**，2026-09-26）的 ICD 侧落地。

⚠ **与 V2.3 的性质相反，必须说清楚**：V2.3 是"**文档向代码收敛**"（不新增接口、不改变代码）；
V2.4 是"**代码按裁决扩充、文档同步冻结**" —— 它**动了既有冻结签名**，是本文件第一条这样的修订。
∴ 本版的读者不是"了解现状"，而是"了解**新的**契约面"。

**本版要解决的实际问题**（不是抽象的一致性）：`capture()`／`grab()` 只有一个 `bool`，
于是**所有**取帧失败在接口上同形 —— 一次超时与一次真实断连无法区分，
而 R09 的已知根因正是"**所有取帧失败被统一禁用**"。本版把结果分类、
每路证据、原始载荷与设备身份四件事补进接口面。

---

# 1.4 本版修正清单

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 4.1 | `ImageFrame` 增 `raw` / `captureFormat` / `rawPolicy` 三字段；并**收紧** `image` 的语义边界（只描述 8U 显示图） | **签名/字段** |
| 2 | 5.1 | `capture(...)` 增 `uint64_t deadlineNs` 形参；新增 `lastCaptureRound()`（**纯虚**） | **签名** |
| 3 | 5.3 | **新增小节**：`lastCaptureRound()` 的契约（每轮覆盖、每路证据保留、聚合规则） | 新增 |
| 4 | 6.1 | `ICameraBackend` 五方法返回 `OperationResult`、`grab` 增 `timeoutMs` 并返回 `GrabResult`；新增 `close()` / `triggerSoftware()` / `deviceIdentity()` / `triggerModeState()` / `lastErrorText()`；`setTriggerMode` 入参由 `bool` 改三值枚举 | **签名** |
| 5 | 6.2 | **新增小节**：V2.4 对 6.1 的说明（三级释放归属、诊断出口、模式读回、桩的形态） | 新增 |
| 6 | 15.2 | 撤回 V2.3 删除 `IRecorderSink::lastErrorText()` 的**理由 ①**（其前提已被本版推翻）；**删除结论不变** | 修正 |
| 7 | 21.1 | 错误码段新增 1003 / 1004 / 1005 / 1006 / 1007（语义以 ENG-09 V2.4 §5.29 为准） | 新增 |
| 8 | 12.3 / 16 / 18.1 / 21.2 | `SYS-08 §7.x` 引用**就地划掉并标注失效**（依据待裁决 Q-D2），**行为与字段一字未改** | **引用勘误** |
| 9 | 27.1 | 版本关系更新（V2.3 归档） | — |

**未改动**：7.1 `ITriggerController`（仍是硬触发与 3001 降级的抽象，
**软件触发的发令不在本接口** —— 见 6.2）、8.1 `ITurntableController`、
14.1 `IPipelineObserver`、17 `FailureTrace`、19 `AlignmentController`、23 禁止接口、
以及 §12.1 / §12.2 / §13.1 / §20.1 的方法级声明。

---

# 1.5 V2.5 修订性质

V2.5 是 **011-A1 九项缺口定向修复批**（2026-09-26，配套裁决 **C-01 v1.8**）的 ICD 侧落地。

⚠ **本版与 V2.4 的性质不同，必须说清楚**：V2.4 改的是**签名与字段**（结果分类、每路证据、
原始载荷、设备身份进入接口面）；**V2.5 一个字都没改签名，改的全是"被调用时发生了什么"** ——
即**每个方法在失败、超期、异常三种压力下的行为边界与清理责任**。

为什么这类内容必须进 ICD，而不能只写在实现里：九项缺口里有 **4 项（P1）**的根因**不在一行代码判错**，
而在**调用方与被调方对"这一步失败意味着什么"的理解不一致**：

```text
取流已启动？      装配层检查在 startAll() 之前     → 一次启动失败被报成整次启动成功（缺口 §1）
帧状态什么意思？   接口上没有"帧不可交付"这一档   → 非零帧状态被推成 NoFrame 或干脆忽略（缺口 §2）
期限还够吗？       一个读数被三处复用、0 被当成"无期限" → 过期任务重新拿到一份预算（缺口 §3）
清理失败算谁输？   七处出口丢弃清理结果、覆写首因     → 返回值与错误文本指向两个不同原因（缺口 §5）
```

⇒ 这些是**契约层面的空白**：接口签名两边都对，**行为两边都合理但不一样**。
∴ 本版把"**失败时谁负责什么**"写成逐条责任，并明确标注**哪些是新的调用责任（订阅方须改）**、
哪些只是把既有实现的行为写实。

**本版不新增任何错误码，不改变 §5.1 / §6.1 的任何一个签名**
（V2.4 的接口面逐字保留），**不关闭** Q-D1／Q-D2／R09 的任何登记项。

---

# 1.6 本版修正清单

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 5.4 | **新增小节**：`capture()` 的调用责任 —— 三个期限检查点、触发前置检查（"不发令、不取帧"）、失败时的输出帧纪律 | 新增／**调用责任** |
| 2 | 5.3 | **V2.5 补充**：每轮记录的"**清旧留新**"规则（哪些本轮事实必须保留、哪些只是上一轮的陈旧文本必须清） | 补充 |
| 3 | 6.2 第 3 条 | **V2.5 补充**：七处初始化失败出口**一律合并**清理结果、**首因不得被覆盖**；`closeDevice()` 不再写 `lastErrorText_`，独立 `close()` 写自己的文本 | **调用责任** |
| 4 | 6.2 第 5 条 | **V2.5 补充**：模式读回的三分支处置（不完整／不一致／一致），前两种**不发令、不取帧**且**不永久禁用通道** | **调用责任** |
| 5 | 6.2 第 9 条 | **V2.5 补充**：**失败交付** —— `CorruptFrame` 与空图**一律不交付**；失败时输出帧**不被触碰**（含不写部分字段） | **调用责任** |
| 6 | 6.2 第 10 条 | **新增**：期限传递（`timeoutMs` 的三个检查点、各自措辞、"无期限"与"已到期"必须分开） | 新增 |
| 7 | 6.2 第 11 条 | **新增**：取帧的**异常安全**（守卫持有释放责任、守卫不抛、异常路径诊断走 `lastErrorText()`、**本批不把异常转成状态码**） | 新增 |
| 8 | 27.1 | 版本关系更新（V2.4 归档）；全文件对 ENG-09 的**版本指向**由 V2.3 改指 **V2.4** | 引用勘误 |

**未改动**：§5.1 / §6.1 的全部方法签名与类型（逐字沿用 V2.4）；
§4.1 `ImageFrame`、§5.2 `lastError`、§6.2 第 1／2／4／6／7／8 条的**结论**；
§7.1 起的全部后续接口；错误码表（无新增码）；§1.2 / §1.4 的历史清单（保留为历史）。

⚠ **须改的调用方**：装配层（`SystemInitializer` 一侧的取流启动复检）、
`MultiCameraManager`（三个检查点、读回三分支、空图判据、sink）、
`ImvCameraBackend`（帧状态、七处出口、异常守卫）。**三者都在本批内改完**。

---

# 1.7 V2.6 修订性质

V2.6 是 **011-A1 清理责任与异常安全批**（2026-09-26，配套裁决 **C-01 v1.9**）的 ICD 侧落地。

⚠ **本版与 V2.5 同属"改行为、不改签名"**：§5.1 / §6.1 的方法签名与类型**逐字沿用**。
改的是 V2.5 **没写完的那一半** —— 三条**同一件事、两种处置**的残留：

```text
释放未确认算谁输？  V2.5 只把"已有主失败"那条写进 cleanupError   → 帧检查通过时禁用通道、
                    ⇒ 同一个"释放未获确认"在两条路径上拿到相反处置      帧已损坏时却继续可用
清理抛异常算谁抛？  V2.5 写了"守卫不抛"，但没写三步各自独立保护    → 清理/诊断一步抛出，
                    ⇒ 原异常可能被二次异常顶掉                       原异常连同类型与文本永久丢失
判定本地＝没调用？  V2.3 起的"本地判定 ⇒ sdkError 必须为 nullopt" → 触发读回不一致（调用成功、
                    ⇒ 与 §5.4／§6.2 的读回不一致条款直接矛盾          本地比对得出）在文档上"必须为空"
```

⇒ 本版给出三条**统一规则**：① `cleanupError` 是"**资源还回去了没有**"的**唯一**字段
（两个入口都写）⇒ **释放未获确认 ⇒ 停止该路后续采集**；② 异常出口**三步各自独立保护**、
末尾 `throw;` **无条件执行**；③ **判定类别与实际调用历史分别记录**。

**本版不新增任何错误码，不改变 §5.1 / §6.1 的任何一个签名**
（V2.4／V2.5 的接口面逐字保留），**不关闭** Q-D1／Q-D2／R09 的任何登记项。
R09 仍为"**部分修复**"（本版把"释放未获确认"的处置统一了，但**自动恢复**仍缺资源恢复依据）。

---

# 1.8 本版修正清单

| # | 节 | 变更 | 类别 |
|---|---|---|---|
| 1 | 6.2 第 3 条 | **V2.6 补充**：`cleanupError` 的**两个入口都写**（主操作成功而清理失败时**同时**写 `sdkError` 与 `cleanupError`），并说明"首因"与"资源归属"是两个问题 | **调用责任** |
| 2 | 6.2 第 11 条 | **V2.6 改写**：异常出口**三步各自独立保护**（描述原异常／显式清理／写诊断），末尾 `throw;` **无条件执行**；`cleanup()` **`noexcept`**、释放抛出时**不重试**并如实返回"调用抛出异常、无返回码" | **调用责任** |
| 3 | 5.1 | **V2.6 补充**：`IMultiCameraManager` 消费侧的第一判据改为 **`cleanupError` 有值 ⇒ 停止该路后续采集**（措辞只谈资源、**不改写首因**、自动恢复须另有依据） | **调用责任** |
| 4 | 5.4 | **V2.6 补充**：触发读回不一致（`ContractViolation`）**不得**把 `sdkError` 写成空 —— 读回调用**确实成功**，如实记 `{该调用, IMV_OK}` | **调用责任** |
| 5 | 6.2 第 5 条 | **V2.6 补充**：同上，落在后端的读回路径上 | 补充 |
| 6 | 26.3 第 4 条 | **V2.6 已实施**：`data::OpStatus.h` 增 `sdkFailureCodeText()`／`sdkFailureText()` 两个助手作**唯一**渲染处，全仓 **10 处**打印改走它（其中 2 处手写措辞分支删除）；⚠ 本条先写成"当前实现符合要求、只登记"，**该结论当时是错的**，同批更正（见 26.3 第 4 条） | **显示责任** |
| 7 | 27.1 | 版本关系更新（V2.5 归档）；全文件**正文**对 ENG-09 的**版本指向**由 V2.4 改指 **V2.5**（实测 **11 处**：§4.1 ×2、§5.3 ×2、§6.1 ×1、§6.2 ×4、§21.1 ×1、§26.3 ×1）；⚠ §1.2／§1.4 的**历史修正清单**里的版本号是**当年的事实**，**保留不动**（§1.4 第 7 行仍写 V2.4）—— 「**正文的现时指针改、历史清单的字不改**」是本次的口径 | 引用勘误 |

**未改动**：§5.1 / §6.1 的全部方法签名与类型；
§5.3 的清旧留新规则、§5.4 的三个期限检查点与触发前置三分支、§6.2 第 10 条（期限传递）；
§6.2 第 1／2／4／6／7／8／9 条的**结论**；§7.1 起的全部后续接口；
错误码表（无新增码）；§1.2 / §1.4 / §1.6（V2.5）的历史清单（保留为历史）。

⚠ **须改的调用方**：`ImvCameraBackend`（`mergeCleanup` 的提升分支也写 `cleanupError`、
异常出口三步各自保护、`cleanup()` `noexcept`）、
`MultiCameraManager`（`cleanupError` 优先的通道处置）。**两者都在本批内改完**。

---

# 2 接口设计原则

## 2.1 分层原则

系统采用：

```text id="4j7m1x"
UI

↓

Application

↓

Algorithm

Device

Optical

↓

Data

```

---

# 2.2 依赖方向

冻结：

```text id="8f3n5m"
Application

↓

Device

Optical

Algorithm

↓

Data

```

---

禁止：

```text id="k7x2pz"
Algorithm

↓

Camera SDK
```

---

禁止：

```text id="m5q8vw"
UI

↓

Hardware
```

---

禁止：

```text id="p8n4qx"
Recorder

↓

Pipeline内部状态
```

---

# 3 接口分类

系统接口分为：

```text id="x5m7qc"
1. 数据接口

2. 设备接口

3. 标定接口

4. 坐标接口

5. 算法接口

6. 模型接口

7. 记录接口

8. 控制接口

```

---

# 4 Data公共接口

---

# 4.1 ImageFrame

方向：

```text id="n3q8mz"
Device

↓

Preview

Algorithm

Application
```

---

结构：

```cpp id="h6m2qx"
struct ImageFrame
{

cv::Mat image;


uint64_t frameId;


uint64_t timestampNs;


uint64_t deviceTimestampNs;


std::string cameraId;


CameraRole role;


double exposureTime;


// ---- V2.4 新增（裁决 C-015）----

RawImagePayload raw;

PixelFormat captureFormat;

RawDataPolicy rawPolicy;

};
```

---

⚠ V2.3 修正：删除 `double gain;` 与 `bool valid;` —— 代码
（`src/data/ImageFrame.h`）中无此二字段，全 `src/` 零命中。本版按代码实况收敛。

若后续真实相机接入（011-A1）确实需要携带增益，应在**同时**修改
`ImageFrame` 与 ICD 的同一个提交内加入，不得只在文档侧声明。

---

⚠ **V2.4 修正（011-A1 数据契约批，裁决 C-015）**：新增上述**三个**字段
（类型定义以 **ENG-09 V2.5 §5.28** 为准，本文件不重复冻结字段表）：

* **`raw`** —— 相机交付的**原始字节**及其全部解释信息。⚠ 它是**自有副本**
  （`IMV_ReleaseFrame` 之前完成复制），**不悬挂在 SDK 缓冲区上**；
  `image` **同样**不得悬挂在 SDK 缓冲区上；
* **`captureFormat`** / **`rawPolicy`** —— 本帧的**采集格式**与**原始载荷必要性策略**。

⚠ **为什么后两个字段必须独立于 `raw` 存在**：它们回答"**这一帧本来应该带什么**"，
不是"它实际带了什么"。若从 `raw` 反推（`raw` 空就当 8 位处理），
则"真实 12 位帧丢了载荷"与"虚拟 8U 帧本来就没有载荷"在数据上**完全同形** ——
前者必须**报错**、后者**照常保存**。分开之后，Recorder 只看帧自身携带的字段即可分流。

⚠ **`image` 的语义边界（本版收紧）**：`image` 是 **8U 显示图**，
`image.type()` / `width` / `height` **只描述它**，**永远不作为裸缓冲的解码依据**。
原始载荷的解码依据**只有** `raw` 自身的
`pixel_format` / `valid_bits` / `packing` / `declared_byte_order`（ENG-09 V2.5 §5.28）。
V2.3 及更早的 `Recorder` 注释曾写"width / height / type 是 cam\*.raw 裸缓冲的解码依据" ——
**那句已作废**。

---

⚠ **`ImageFrame` 变大的后果（二进制兼容性）**：`raw` 使 `ImageFrame` 体积增大，
`MultiCameraFrame` 随之增大 ⇒ **静态库不得混用旧 `libdata.a`**，必须全量重编。
这是本版唯一的**二进制布局**变更。

---

约束：

## frameId

必须：

单调递增。

---

## timestampNs

必须：

主机单调时间。

禁止：

墙钟。

---

## deviceTimestampNs

SDK支持：

填写设备时间。

SDK不支持：

必须：

```cpp id="r5m9qx"
0
```

禁止：

伪造。

---

# 5 MultiCamera接口

## 5.1 IMultiCameraManager

职责：

管理三相机。

---

接口：

```cpp id="q8n4mz"
class IMultiCameraManager          // 实现类：device::MultiCameraManager
{

public:


virtual bool initializeAll()=0;


virtual bool startAll()=0;


virtual void stopAll()=0;


virtual bool capture(
data::MultiCameraFrame&
frame,

uint64_t deadlineNs
)=0;


virtual data::CaptureRound lastCaptureRound() const =0;


virtual data::ErrorInfo lastError() const =0;


};
```

⚠ **V2.5 补充（调用责任，签名未变）**：`capture()` 的**行为边界**由 **§5.4** 定义 ——
三个期限检查点、触发前置检查的三分支处置、以及"失败时输出帧不被触碰"。
⚠ 其中**最容易被误解的一条**：`capture() == false` **不蕴含**"本轮一次 SDK 调用都没有"
（可能是"**已发令、未取帧**"），也不蕴含"所有通道都没拿到帧"。
**逐路事实只看 §5.3 的记录**，不得从这一个 `bool` 反推。

⚠ **V2.6 补充（消费侧的第一判据：`cleanupError`，不是主状态）**：
`MultiCameraManager` 决定"这一路下一轮还采不采"时，**先看 `cleanupError` 有没有值**：

1. **有值 ⇒ 停止该路后续采集**（`channelAvailable == false`），
   并在该路记录里写明**释放未获确认**（含调用名与码）；
   ⚠ 措辞**只谈资源**（未归还的缓冲会被 SDK 内部缓存复用、污染后续帧），
   **不得**写成"SDK 错误导致禁用（分类未明确）" —— 后者会让人去查**设备**，
   而真正该查的是**缓冲归属**；
2. **主失败与清理诊断一并保留**：`status` 与 `sdkError` **一字不动**，
   现场同时看到"本来因为什么失败"与"资源没还回去"；
3. **这一条必须在按状态分类之前判**，且**只看 `cleanupError`** ——
   若按"主状态是否属于禁用集合"来判，同一个"释放未获确认"会因为
   "帧本身好不好"而拿到两种处置（§6.2 第 3 条的 V2.6 补充）。
   ⚠ 注意：**释放未获确认**与**主失败本就被判禁用**（`SdkError`／`Disconnected`）
   两件事可以同时成立；此时以"**释放未获确认**"为准（它是**资源**事实，
   与"设备为什么失败"无关），但**两条信息都要留下**；
4. **自动恢复不在本版**：重新启用须先有**资源恢复依据**（R09 仍为"部分修复"）。

---

## 5.2 lastError

V2.3 新增（代码有、V2.2 漏列）。

必须提供的原因：`capture()` 只返回 `bool`，而它失败的原因至少有两类
**处置完全不同**的情形：

```text id="c5n8mq"
可用相机数不足   →  1001，硬件故障，不重试

三路时间戳超差   →  3002，瞬态，重采即可
```

上层若只凭 `false` 自己编一个码，等于把这两类混为一谈。

⚠ **V2.4 补充**：`lastError()` 表达的是"**最近一次整体失败的原因**"，
它**不能**代替**每路**的证据（"两路成功、一条超时"时它只能说一件事）——
每路结果走 §5.3 的 `lastCaptureRound()`；而 SDK 层的**设备身份诊断文本**
走 §6.2 第 8 条的 `ICameraBackend::lastErrorText()`。三者层次不同，**不得混用**。

实现者**应当**填写本字段。未填写（`code == 0`）时上层按兜底码 9004 记录，
**不得**冒充某个具体码 —— 宁可少报一个具体码，也不要凭空造一个不成立的根因。

---

输出：

```text id="j4p8mq"
MultiCameraFrame
```

---

## 5.3 lastCaptureRound（V2.4 新增）

**为什么要有它**：`capture()` 返回 `bool`，而 `lastError()` 只有一个 `ErrorInfo` ——
两者都表达不了"**三路各自发生了什么**"。一次取帧里"两路成功、一条超时"是本批最常见的失败形态，
而在只有 `bool` 的接口上，那条超时的证据**无处存放**，上层只能凭经验反推
（现行代码就是靠"该通道的图是否非空"来反推，于是**一次超时会被记成"相机断连"**）。

**契约（类型定义以 ENG-09 V2.5 §5.32 为准）：**

```cpp id="w4t7mq"
data::CaptureRound lastCaptureRound() const;
```

1. **每轮 `capture()` 覆盖一次** —— 它表达的是"**最近一轮**发生了什么"，不是历史累计；
2. **成功时每路结果仍须保留** —— "两路成功、一条超时"那条超时的证据**必须可读**；
3. **`capturedCount` 的判据 = "本轮成功交付的有效帧数"**（该路 `status == Ok` **且**
   交付的图合法）。⚠ **不承诺**它等于最终写盘的文件数 —— 落盘还受 Recorder 的分支影响，
   两者不一致是**正常**的；
4. **聚合按严重度取最大者**，且 **`attempted == false` 的本地超时同样参与聚合**
   （预算耗尽时没有调用 SDK，却确实产生了一次本地 `Timeout`；只挑"尝试过且失败"会把它整条漏掉）；
5. **`startedNs` / `finishedNs` 是实测耗时** —— 把"一次 GUI 回调里 `capture()` 实际占用了多久"
   变成**可测事实**。⚠ 本版**不承诺**"组预算 = GUI 线程硬上限"：一次回调里会连采多组，
   复制、转换、发令都不受 `GetFrame` 等待参数约束。

⚠ **本方法是纯虚，不给默认实现**：本仓有两个 `IMultiCameraManager` 的测试替身，
默认实现会让"**漏迁移**"静默通过并报出 0 通道；纯虚把遗漏变成**编译错误**。

---

**V2.5 补充：每轮记录"清旧留新"。**
本记录**只描述最近一轮**（第 1 条），故每轮开始时必须**清除上一轮的陈旧错误信息**，
而**本次实际取得的事实一律保留**。两者不得混为一谈：

| 情形 | 本轮记录里应当存在什么 |
|---|---|
| 正常取帧 | `frameStatusRaw = 0`（**不是 `nullopt`**）、实际调用诊断、实际触发读回；**该路错误文本为空** |
| 入口预算不足（未取帧） | 本轮**确实没有取得**读回／触发／帧状态 ⇒ 这些字段为空／`nullopt`，**不得**填上一轮的值 |
| 读回后预算不足（未发令） | **保留本次读回结果** |
| 发令后预算不足（未取帧） | **保留本次读回与触发结果**、`attempted = true`、帧状态**未取得**（`nullopt`） |

⚠ 判据是"**本次是否真的取到了这个事实**"，不是"这个字段有没有值"：
"帧状态为 0"（正常）与"帧状态未取得"（`nullopt`）**必须可区分**（ENG-09 V2.5 §5.32）。
⚠ **不得**把 `ICameraBackend::lastErrorText()` 读进本轮记录充数 —— 它是"最近一次失败文本"，
可能来自**更早**的一轮，混进来就等于用一个陈旧原因解释一次新的失败。

---

## 5.4 capture() 的调用责任（V2.5 新增）

本节定义 `capture()` 在**压力条件下**的行为边界。签名与 V2.4 逐字相同（§5.1），
**新增的是调用责任**，不是接口面。

**1 期限（`deadlineNs`）必须在三个检查点各自重读时钟、各自处置，措辞互不混用。**
⚠ 缺口原形：实现只在入口读一次钟算出剩余量，此后**模式读回（真实后端是三次阻塞 SDK 读）、
软件触发、`grab()` 复用同一份读数** —— 等到取帧时这份预算**早已过期**，
而失效的预算原本在控制器侧会被兜底逻辑**重新注入一份**（故 V2.5 同时要求把
"无期限"与"已到期"分开，见 §6.2 第 10 条）。
⚠ 实施中的实测结论（**如实登记，供后续版本核对**）：该"重新注入"在**当前控制器里可达**，
且已被端到端用例与定向变异证实 —— **结论方向与本节初稿相反，以本段为准**。可达路径如下：
`tick(nowNs)` 的入口判定读的是**形参** `nowNs`（`main.cpp` 传入的快照），而 `stepCapture()` 每一轮
判停/取期限时**重新读一次注入时钟**（`MeasurementController::nowNs()`），二者**不是同一次读数**：
只要任务期限落在这两次读钟**之间**（入口为 `T_task − ε` 通过检查、CAPTURE 首轮重读已是 `T_task + ε`，
此刻状态级期限尚未到），本次调用就会带着**已到期**的任务走进 `acquireDeadlineNs()`。
⇒ 控制器侧「已到期 ⇒ 期限＝`nowNs`」这一支**不是防御性分支**，它是**缺陷本体**；
若把它退回"忽略 `taskRemaining == 0`"的旧逻辑，这一拍会退化为状态预算／组预算、
**重新获得一份预算**并发起本不该发起的采集。
∴ 本版**必须**为它写"改前红、改后绿"的用例：
`tests/integration/MeasurementFlowTest.cpp` 的 `A1_44_同一拍内跨过任务期限时一次采集都不发起`
（注入时钟前置量 `+1 ns` 构造上述场景；断言该拍**一次采集都不发起**、无"已采满"提示、
最终整次以 `kErrTaskTimeout` 收尾）＋ 定向变异 **M18**（忠实复现修复前形态：停用 `== 0` 分支
**并**给第三分支加 `taskRemaining > 0 &&`）⇒ 上述 4 条断言**全部转红**。
⚠ 其余四个 `acquireDeadlineNs()` 调用点传入的都是本拍形参，被入口判定挡住，
∴ **只有** `stepCapture()` 的逐轮重读这一条路能让"已到期"走到本函数 —— 这正是本版把
"无期限"与"已到期"分成两个显式分支、且不允许合并的原因。

| 检查点 | 条件（**重读时钟**后的剩余） | 软件触发次数 | `grab` 次数 | 该路记录 |
|---|---|---|---|---|
| **入口**（连模式读回都不执行） | < 1 ms | **0** | **0** | `Timeout` ＋ `sdkError == nullopt` ＋ 文本"预算耗尽" |
| **读回后、发令前** | < 1 ms | **0** | **0** | `Timeout` ＋ 文本"**发令前预算耗尽**" |
| **发令返回后、取帧前** | < 1 ms | **1** | **0** | `attempted = true`、**保留触发结果**、`Timeout` ＋ 文本"**已发令、未取帧（取帧前预算耗尽）**" |

⚠ 第三行**不得**写成"既未发令也未取帧" —— 那会把**已经发生**的一次发令抹掉，
让下层据此误判"设备没收到令"。
⚠ 每个检查点的 `timeoutMs` 用**该点重读后的读数**重算（`min(单次上限, 组剩余, 距 `deadlineNs` 剩余)`），
**不得**复用入口那一次的读数。

**2 触发前置检查：读回不能确认前置条件时，"不发令、不取帧"。**
模式读回（§6.2 第 4／5 条）是发令的**前置条件**，三种情形处置如下：

| 情形 | 本路结果 | 后续动作 |
|---|---|---|
| 读回不完整（`reported` 未知） | `NotStarted`／应用码 1004 ＋ 文本"**本轮无法确认触发前置条件**" | **不发令、不调用 `grab()`** |
| 读回完整但与请求不一致 | `ContractViolation`／应用码 1006 ＋ 记录**请求值与实际值** | **不发令、不调用 `grab()`** |
| 读回完整且一致 | 按**实际回读模式**执行 | **仅 `Software` 发软件触发（恰好一次）** |

⚠ 两种失败**都不永久禁用通道**：下一轮允许重新读回、重新判定。
"读不到"是**本轮**的事实，不是设备的永久属性。
⚠ "未知"**不得**写成"设备已停止取流" —— 前者是"我们没确认"，后者是"设备状态如此"，
**我们不得声称知道没有证据的事**。
⚠ 被调方有义务让这个判断**做得出来**：读回记录须带**特性名**、须能区分
"调用失败"与"调用成功但返回空串"（ENG-09 V2.5 §5.31）。

⚠ **V2.6 补充：上表第二行的 `sdkError` 不得为空。**
"不一致"这一行的**判定是本地做的**，但读回调用**确实成功返回了值** ⇒
`sdkError` **必须**如实记为 `{该读回调用, IMV_OK}`，并同时记录请求值与实际值。
⚠ **不得**因为"结论属本地判定"就把它写成空（"**本地判定不代表未调用过 SDK**"，
ENG-09 V2.5 §5.29）—— 抹掉一次真实调用，会让排查看不出"我们**读到过**什么、
只是读到的与要的不一样"，从而把一个**配置不符**问题误判成**设备没响应**。
⚠ 第三行（一致）**照样**保留实际的读回调用记录：一致也是**依据读回值**得出的结论。

**3 失败时输出帧不被触碰。**
`capture()` 整体失败、或任一路失败，**都不得**修改 `MultiCameraFrame` 的对应内容去"传诊断"。
诊断走 §5.3 的记录与 §6.2 的出口 —— **不为了传诊断而交付失败帧**。

---

# 6 Camera接口

## 6.1 ICameraBackend

职责：

隔离SDK。

---

接口：

```cpp id="m7q2xz"
class ICameraBackend
{

public:

virtual data::OperationResult initialize()=0;


virtual data::OperationResult start()=0;


virtual data::OperationResult stop()=0;


virtual data::OperationResult close()=0;


virtual data::GrabResult grab(
data::ImageFrame&
frame,

uint32_t timeoutMs
)=0;


virtual data::OperationResult setTriggerMode(
data::CameraTriggerMode mode
)=0;


virtual data::OperationResult triggerSoftware()=0;


virtual data::DeviceIdentity deviceIdentity() const =0;


virtual data::TriggerModeState triggerModeState() const =0;


virtual std::string lastErrorText() const =0;


};
```

⚠ **V2.4 修正（裁决 C-015）—— 这是本文件第一条动了既有冻结签名的修订。**
本接口由 **6 个**方法变为 **11 个**；三个方法**保留原名、改名返回类型与形参**
（`initialize` / `start` / `stop`）或**两者都改**（`grab` / `setTriggerMode`）。
类型定义以 **ENG-09 V2.5 §5.28～§5.31** 为准，本文件只冻结**接口面**。

---

## 6.2 V2.4／V2.5 对 6.1 的说明

**1 为什么返回类型由 `bool` 改为 `OperationResult`。**
`bool` 把"超时"、"断连"、"未启动"、"格式不支持"、"参数错误"、"SDK 内部错误"
**压成同一个值**，而上层对它们的处置**互不相同**（等待／重试／降级／终止）。
这不是抽象的一致性问题：R09 的已知根因就是"**所有取帧失败被统一禁用**"，
而"统一"之所以可能，正是因为接口上它们同形。

**2 诊断出口：`sdkError` 与 `cleanupError` 都在 `OperationResult` 里，且必须带操作名。**
⚠ 单独一个返回码不够：同是 `-119`，`IMV_GetFrame` 超时与 `IMV_ReleaseFrame` 超时
是**完全不同**的故障（前者"没等到图"，后者"资源没还回去"），现场动作相反。
⚠ **主操作成功而必要清理失败 ⇒ 整体返回失败且不交付帧** —— `ReleaseFrame` 失败
**不得**仍报 `Ok`（那是把"资源没还回去"说成成功）。
⚠ **V2.6 更正：上一句（"本地判定类错误不得携带 `sdkError`"）已废除。**
它把一个**调用历史**问题写成了**判定类别**的推论，而两者是**两个问题**：
判定属本地类，说的是"结论从哪里来"；`sdkError` 记的是"**此前确实调用过哪一次**"。
⇒ 冻结为：`sdkError` **按实际调用情况如实填写** ——
本次**没有**发起调用的本地失败（如 `timeoutMs == 0`）为 `nullopt`；
依据一次**成功**调用得出的本地结论（如第 5 条触发读回不一致）
**必须**带 `{该调用, IMV_OK}`。判据是"**这次调用发生了没有**"，
**不是**"结论从哪里来"。完整语义见 **ENG-09 V2.5 §5.29**。

**3 三级释放的归属**（本版新增 `close()` 的全部理由）：

| 层次 | 谁做 | 时机／幂等要求 |
|---|---|---|
| **停流** `IMV_StopGrabbing` | `stop()`（**语义不变**：允许未 start 时调用、须无副作用） | `stopAll()` 照旧"无论 available 与否都调" |
| **关设备 + 销毁句柄** `IMV_Close` / `IMV_DestroyHandle` | **`close()`**：幂等、允许在未 `initialize()` 或已 `close()` 时调用、无副作用 | 由调用方在"设备不再使用"时显式调用；**`stop()` 不等同于 `close()`** |
| **失败出口** | `initialize()`／`start()` **自己** | 失败时释放**它自己**已建立的半初始化资源，**不得**留下打开的句柄 |

⚠ 断言纪律：**看的是底层资源，不是包装调用次数** —— `close()` 这一层幂等包装**允许**
被拥有者与析构兜底各调一次；要防的是"**再次调用产生第二次底层销毁**"。

⚠ **V2.5 补充（清理失败与首因，缺口的直接来源）**：
"失败时释放自己"只有**释放**这一半是不够的，**释放的结果也必须被合并、首因也必须被保住**，
否则返回值与错误文本会指向**两个不同的原因**。三条硬规则：

1. **每一次失败出口都要合并清理结果**：把清理调用的结果按
   `OperationResult` 的既有语义合并 —— **主因占 `status`／`sdkError`，清理失败只进 `cleanupError`**。
   ⚠ **丢弃清理结果**（`(void)closeDevice();` 形态）与**用清理结果覆写主因**同样不可接受：
   前者让"资源没还回去"消失，后者让真实首因消失；
2. **首因文本由本次失败的局部快照给出，之后才追加清理信息**，
   形如 `"<本次首因>；清理失败：<调用名> 返回 <码>"`。⚠ 因此
   **`closeDevice()` 这类底层清理例程不得写诊断文本** ——
   否则一次**独立的** `close()`（与历史故障无关）会把旧故障混进本次文本；
3. **顶层独立 `close()` 失败时写自己的清理文本**，与任何历史故障无关。

⚠ 本版把这三条写成**责任**而非实现细节，是因为它们决定了失败报告**是否可信**：
一个"状态说 A、文本说 B"的失败报告，会让现场**按 B 去查一个不存在的问题**。

⚠ **V2.6 补充（上表三条的收口："资源还回去了没有"是一个独立事实）**：
上面第 1 条只说了"已有主失败 ⇒ 清理失败进 `cleanupError`"，**没有说清另一半** ——
**主操作成功而清理失败时也要进**。缺了这一半，同一个"释放未获确认"会在两条路径上
拿到**相反**的处置：调用方若按"主状态是否属于禁用集合"决定通道去留，
则**帧检查通过**时（状态被提升为 `SdkError` ⇒ 禁用）与**帧已损坏**时
（首因被保留 ⇒ 状态不在禁用集合里 ⇒ **继续可用**）会得到两种结果，
区别只在于"**帧本身好不好**"—— 而帧好不好与"**缓冲还回去了没有**"是**两个不相干的问题**。
⇒ 冻结为：

1. **两个入口都写 `cleanupError`**：清理失败一律进该字段（含"主操作成功"那一入口），
   它是"**资源还回去了没有**"的**唯一**字段。
   `sdkError` 回答"**失败的首因是什么**"、`cleanupError` 回答"**资源还回去了没有**" ——
   两者**不得只留一个**；
2. **`cleanupError` 有值 ⇒ 停止该路后续采集**（调用方视角的处置见 §5.1 的同一补充）。
   ⚠ 措辞**只谈资源**、**不改写首因**（`status` 与 `sdkError` 一字不动：
   现场要同时看到"本来因为什么失败"与"资源没还回去"）；
3. **释放未确认时不得盲目重试、不得自动恢复**：未知的是**资源状态**，
   故处置是"**停止该路后续采集**"而不是"判定这台相机坏了"；
   自动恢复须先有**资源恢复依据**（本版不做，R09 因此仍为"**部分修复**"）。

**4 软件触发的执行者只有一个：`MultiCameraManager`。**
`triggerSoftware()` 在本接口上存在，但**后端不得自行发令**：
每一路取帧**之前**若该路模式读回为 `Software`，由管理器调它**恰好一次**。
`grab()` **只取帧**（`IMV_GetFrame`），**绝不**发令 —— 后端不知道也不该知道"上层是否已经发过令"。
⚠ **发令失败 ⇒ 取帧次数为 0**（不得"失败也照样去等一帧"）；
**"等待一帧"不等于"发出了一次软件触发"**。
⚠ `ITriggerController`（§7.1）**不变**：它仍是**硬触发**与 `3001` 降级的抽象，
**不持有相机后端、不发软件触发令**。

**5 模式读回必须读到三项，不能只读一个开关。**
⚠ **只读 `TriggerMode == "On"` 分不出软件触发与硬件触发** —— 两者都是 `"On"`，
区别只在 `TriggerSource`。故选择器／开关／触发源三项都要读，
且**读不到要能表达"未知"**（`nullopt`），**不得**把"没读到"默认成某个合法模式
（那会让一次读失败伪装成"模式正确"）。映射表与读回规则见 ENG-09 V2.5 §5.31。

⚠ **V2.5 补充：读回的三分支处置，以及"未知"不得被回退成请求值。**
缺口原形：实现用 `reported ? reported : requested` 的三元表达式 —— **读不到就当作请求值**，
于是"一次读失败"被伪装成"模式正确"，随后的发令与取帧都建立在一个**没有证据的前提**上。
V2.5 删除该回退，按下表处置（与 §5.4 第 2 条同一张表，此处是**被调方**视角）：

| 情形 | 本路结果 | 后续动作 |
|---|---|---|
| 读回不完整、`reported` 未知 | `NotStarted`／1004 ＋ "**本轮无法确认触发前置条件**" | **不发令、不取帧** |
| 读回完整但与请求不一致 | `ContractViolation`／1006 ＋ **记录请求值与实际值** | **不发令、不取帧** |
| 读回完整且一致 | 按**实际回读模式**执行 | 仅 `Software` 发软件触发 |

⚠ 读回记录必须能回答"**是哪一项特性**读不出来、**是调用失败还是返回空串**"
（`TriggerSelector` / `TriggerMode` / `TriggerSource` 三项各自带特性名与失败现场，
ENG-09 V2.5 §5.31）—— 只有一个 `vector<SdkFailure>` 说不清这件事。
⚠ **读回调用的失败信息不得被丢弃**：接口装不下就补诊断字段，**不得**因为"反正没用上"而扔掉。

⚠ **V2.6 补充：不一致那一行（`ContractViolation`）的 `sdkError` 必须如实带调用。**
读回**调用确实成功**了（返回了值），"与请求不一致"是**本地比对**得出的结论。
按"**判定类别与实际调用历史分别记录**"（ENG-09 V2.5 §5.29），这条路径上
`sdkError` **必须**如实记 `{该读回调用, IMV_OK}` ——
⚠ **不得**因为"结论是本地算出来的"就把它写成空：
**本地判定不代表"未调用过 SDK"**，写成空是在抹掉一次**真实发生**的调用。
（V2.3 起那句"本地判定 ⇒ `sdkError` 必须为 `nullopt`"**已废除**，
它与本表这一行**直接矛盾** —— 本版按语义检索把同类处一并改掉。）
⚠ 反向同样成立：**"本地判定"也不等于"必须带 `sdkError`"** ——
没有调用过的本地失败（如 `timeoutMs == 0`）**仍然**是 `nullopt`，
判据是**这次调用发生了没有**，不是结论从哪里来。

**6 `deviceIdentity()` 与型号匹配。**
⚠ **型号匹配不能替代设备身份匹配**：打开相机按**序列号**精确匹配；
配置里序列号为空 ⇒ **明确失败**，**不取"第 0 个设备"**（那会让三台同型号相机随机互换）。
未取得身份就写"未取得"，**不得**从配置复制目标序列号去填实际身份。

**7 诚实桩的形态（不以 `bool` 退化）。**
未找到 SDK 与"找到 SDK 但适配未完成"是**两件不同的事**，必须给**不同文本**
（后者要写明"**是代码没写、别去查硬件和配置**"）；两者都返回 `NotImplemented`
且 `sdkError == nullopt`（**根本没调用 SDK**，符合 ENG-09 V2.5 §5.29 状态表的**第一行**："本次确实没有发起任何 SDK 调用"）。

**8 `lastErrorText()` —— 与 §15.2 删除的那个同名方法无关。**
本方法承载的是**设备身份诊断文本**（枚举到的型号／序列号、按序列号匹配的结果），
供装配摘要与失败报告打印；它**不是**错误码通道（错误码走 `OperationResult` 与 `ErrorInfo`）。
⚠ 设备层内 `lastErrorText()` 是**既有多处实例的共同形态**
（`PekoTurntableController` / `VirtualTriggerController` / `HardwareTriggerController` 均有），
本版把该形态提到接口上，是因为**只有后端拿得到那些设备侧文本**，
而装配层报"接入失败"时若只能贴一句"（无附加说明）"，失败报告就指向一个不存在的东西。

**9 `@return` 逐值语义。** 各 `OpStatus` 取值的完整含义、应用码与聚合严重度
见 **ENG-09 V2.5 §5.29 的状态全表**（本文件不重复）。⚠ 其中两条**不得违反**：
`status != Ok` 时 **`frame` 不被触碰**（不写任何字段，包括不写部分字段 ——
写半成品会诱导调用方误用）；**`Timeout` 不得覆盖 `CorruptFrame`**
（"没等到帧"与"调用成功但帧坏了"处置不同：前者查触发与链路，后者查格式与布局）。

⚠ **V2.5 补充一：失败交付 —— 什么算"可交付"，以及失败帧一律不交付。**
"接口没写 `frame`"只是**被调方**的纪律；**调用方**还须有对应的判据，
否则失败帧会从另一条路（判据只看 `status`／只看图非空）漏进流水线。三条：

1. **`CorruptFrame`（1005）与"空图"一律不交付**：SDK 调用成功但帧不满足可交付条件时，
   结论是"**这次没有可用的帧**"，不是"有帧但质量差"。⚠ 措辞只说"**SDK 调用成功，
   但帧不满足可交付条件**"，**不宣称知道具体损坏原因**（帧状态各位的含义未经文档化，
   我们不得替设备解释自己的状态位）；
2. **"后端报成功但交付空图"按契约违背处理**：调用方发现"`status == Ok` 却没有合法图"，
   应记 `ContractViolation`／1006，而**不得**沿用 `Ok`（否则一次空图在聚合里**完全消失**，
   整轮被报成正常，原因进不了任何失败说明）；
3. **调用诊断原样转发，不得补造**：调用方不得由"我调用了取帧"推出"所以一定有
   `{ImvGetFrame, 0}`" —— 后端**可能根本没调用 SDK**（虚拟／测试替身）。
   原来是 `nullopt` 就继续是 `nullopt`，`cleanupError`／本次诊断字段同样原样转发。

⚠ `capturedCount` 的判据**继续冻结为**"`status == Ok` **且**交付的图合法非空" ——
交付判据**不依赖**"某个实现当前恰好让两个条件等价"。

⚠ **V2.5 补充二：失败的传播面不得只靠一个 `bool`。**
`grab()` 返回成功而清理失败 ⇒ **整体失败且不交付帧**（第 2 条已述）；
本轮各路的调用诊断须**同时**进入两条消费路径 —— **整轮失败**的出口与**降级说明**的出口。
⚠ 现状是只有一个出口接到了现场信息，另一个出口（整轮失败）**接不到任何现场信息**，
现场只能凭状态码猜。**两条出口都要接**，否则"采集失败"与"采集失败但原因已知"没有区别。

**10 期限传递：`timeoutMs` 是"这一次调用"的上限，每次都必须重算。**
`grab(frame, timeoutMs)` 的参数语义是**单次调用的等待上限**，不是一个可以跨步骤复用的"剩余时长"。
∴ 调用方在**每一个**期限检查点（§5.4 第 1 条的三行）各重读一次时钟、各重算一次 `timeoutMs`。
⚠ 复用一个**过期**的读数是缺口原形：中间的模式读回（真实后端三次阻塞 SDK 读）
与软件触发发令都要花时间，等走到 `grab()` 时那份预算可能已经耗尽 ——
于是"按 200 ms 等待"实际等的是**一个早已不存在的额度**。
⚠ 到期后的处置见 §5.4 表（**不发令／不发令／已发令但不取帧**），措辞逐行不同。

⚠ **"无期限"与"已到期"必须分开**（本版新增的取值要求，方向由裁决确认）：
哨兵 `data::kNoDeadlineNs`（`UINT64_MAX`）**表示"无期限"，不参与取小、不得被算成已到期**；
其余取值 `≤ now` 表示**已到期**。两者在任何判据里都**不得合并成一个分支** ——
把"无期限"当"已到期"会**取消掉**本可允许的等待，把"已到期"当"无期限"则会让
一次已经超期的任务**重新获得一份预算**。该常量与 `RetryManager::kNoDeadline` **同值同义**
（定义与取值边界见 ENG-09 V2.5 §2.5）。
⚠ 两类误判的**当前可观测性**，实施时的实测结论如下（两者**都不许**被写成"已由用例覆盖"）：
· "**已到期**被算成无期限／被重新注入预算"：**可达，且已有实测证据** ——
  入口判定读的是**形参**、`stepCapture()` 每轮读的是**重新注入的钟**，期限可以落在两次读钟之间
  （完整论证与证据见 §5.4 第 1 条的同一段登记：用例 `A1_44_同一拍内跨过任务期限时一次采集都不发起`
  ＋ 变异 M18 ⇒ 4 条断言转红）。∴ 该方向**不是**防御性登记，而是本版修复的缺陷本体；
· "**无期限**被算成已到期"：**当前没有**任何一条端到端用例构造出"无期限"的任务
  （`data::kNoDeadlineNs` 只在 `RetryManager::beginTask` 的溢出分支上产生，
  现有测试配置走不到那里），∴ 该方向同样**未被实测**。
⇒ "无期限"这一方向只能靠**代码判据**守住（不得合并两分支、不得用 `0` 作哨兵、
`kNoDeadlineNs` 与 `RetryManager::kNoDeadline` 同值同义）；为它写端到端用例只会得到恒真的绿灯，
故本版**不写**，并在此如实登记"未实测"。

**11 取帧的异常安全：释放责任由守卫持有，且守卫不抛。**
`grab()` 在"`GetFrame` 成功"与"`ReleaseFrame`"之间还有**复制与图像分配**，
其中任何一次分配失败（`std::bad_alloc`）都会**跳过释放**，把 SDK 帧泄漏掉。三条：

1. **RAII 守卫持有"释放"责任**：正常出口由函数体**显式取回**释放结果并合并进返回值
   （`GetFrame` 成功而 `ReleaseFrame` 失败 ⇒ **整体失败、不交付帧**，与第 2 条一致）；
   **异常路径下守卫先释放**，再让异常继续上抛；
2. **守卫析构不得抛异常**，且**清理诊断的写入失败不得覆盖原异常**
   （析构内须全吞并置位）—— 一个"清理时又抛"的析构会在栈展开中直接终止进程，
   那比泄漏更难排查；
3. ⚠ **异常路径的诊断必须有能在栈展开之后读取的出口**：守卫若只把诊断写进一个局部结果对象，
   这份结果**随栈展开消失**，调用方拿不到。
   ∴ 异常路径把**本次异常文本与清理结果**留在**后端诊断状态**（由既有的
   `lastErrorText()` 读取）；**正常返回路径仍通过 `GrabResult` 传递**，两者不混用。

⚠ **V2.6 改写：上面第 1／2 条只说了"谁负责"，没说"另两步抛出时怎么办"。**
缺口原形：异常出口是**三步顺序执行的普通代码** ——
① 描述原异常、② 显式清理、③ 写诊断 —— 而**这三步每一步都要分配内存**。
任一步抛出（例如 ① 拼描述文本时 `bad_alloc`），函数尾部的 `throw;` **根本执行不到**，
调用方收到的是**二次异常**，**原异常连同它的类型与文本永久丢失** ——
而"保住原异常"正是这段代码存在的**唯一理由**（记录一次清理失败，不能拿一次真实故障去换）。

∴ **V2.6 冻结为三条**（缺一不可）：

1. **三步各自独立保护**：每步关在自己的保护里，**失败只损失那一步的产物**
   （描述失败 ⇒ 诊断文本退化为"描述本身构造失败"的占位句；清理失败 ⇒ 按第 3 条记；
   写诊断失败 ⇒ 这条诊断丢了，但帧已释放、原异常仍在）；
2. **末尾的 `throw;` 无条件执行** ⇒ **原异常必定原样上抛**。⚠ 判定标准不是"捕获到了某个
   `bad_alloc`"，而是"**捕获到的就是原来那一个**" —— 验收用例必须让原异常与二次异常
   **可区分**（原异常 `bad_alloc`、二次异常换一种类型或带可识别文本），
   只断言"抛了 `bad_alloc`"证明不了这件事；
3. **`cleanup()` 是 `noexcept` 且不重试**：释放调用本身抛出时，如实返回
   `{ImvReleaseFrame, kCallThrewCode}`（"**调用抛出异常、无返回码**"）。
   ⚠ 那次调用**是否已经把缓冲还回去了无法判断**，**盲目重试就是"一个帧释放两次"**
   （SDK 内部缓存计数错乱，而错误码可能仍是 0 —— **无声的破坏**）。
   ⚠ `kCallThrewCode` 是**标记值、不是码**：显示时必须走专用措辞，
   **不得**把 `-2147483648` 当 SDK 原码打印（本文件只把该措辞的责任点出来，
   取值与显示规则见 ENG-09 V2.5 §5.29）。

⚠ **三步各自保护 ≠ 只保护清理**：只保护第 ② 步的话，第 ① 步失败时**显式清理根本没跑**，
于是**析构兜底会运行** —— 那条路径上原异常侥幸不丢，但"释放是否恰好一次"变成
依赖析构的时序。⚠ 由此产生一条**记录纪律**：**不得**用"关掉析构兜底后用例全绿"
论证"兜底不可达" —— 全绿只说明**现有用例没覆盖那条路径**（登记结论只能写成
"**现有测试未覆盖**"，**不能**写成"不可达"；见 §26.3）。

⚠ **本版不把异常转成状态码**：`grab()` 的异常语义（抛还是记）**未经裁决**，
不在本批范围内 —— 本版只要求"**释放恰好一次、原异常不丢、诊断可读**"。

---

实现：

```text id="f8m3qw"
VirtualCameraBackend

ImvCameraBackend

```

---

# 7 Trigger接口

## 7.1 ITriggerController

接口：

```cpp id="w4m8py"
class ITriggerController
{

virtual bool initialize()=0;


virtual bool enable()=0;


virtual bool trigger()=0;


virtual void disable()=0;

};
```

---

数据流：

```text id="b6n3qw"
Trigger

↓

Camera Exposure

↓

ImageFrame

```

---

# 8 Turntable接口

## 8.1 ITurntableController

接口：

```cpp id="p9x4mq"
class ITurntableController
{

virtual bool initialize()=0;


virtual bool move(
const TurntableCommand&
)=0;


virtual TurntableState state()=0;


virtual void stop()=0;

};
```

---

输入：

```text id="z5m7qx"
TurntableCommand
```

---

输出：

```text id="n8q2mw"
TurntableState
```

---

# 9 标定接口

## 9.1 CalibrationManager

职责：

管理光机标定（`data::OpticalRigCalibration`）。

---

接口：

```cpp id="t6m8px"
class CalibrationManager
{

public:


virtual bool load(
const std::string& path
)=0;


virtual bool loadDefaults(
int imageWidth,
int imageHeight
)=0;


virtual CameraCalibration getCalibration(
CameraRole role
)=0;


virtual Transform getRigToShip()
=0;


virtual OpticalRigCalibration calibration()
=0;


virtual std::string calibrationId()
=0;


virtual bool loaded()
=0;


};
```

---

## 9.2 V2.3 修正说明

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `CameraRigTransform getCameraToRig(CameraRole)` | **删除** | 代码无此方法。相机→刚体变换内嵌于 `CameraCalibration`：`data::CameraCalibration` 含 `Transform cameraToRig;` 字段，故该量由 `getCalibration(role).cameraToRig` 获得，不需独立访问器 |
| `CameraCalibration getCameraCalibration(CameraRole)` | `getCalibration(CameraRole)` | 与代码一致 |
| `RigShipTransform getRigToShip()` | `Transform getRigToShip()` | `RigShipTransform` 在代码中零命中。**不保留包装类型** |
| — | 增 `loadDefaults` / `calibration` / `loaded` | 代码有、V2.2 漏列 |
| `CameraRigTransform` / `RigShipTransform` | 均为类型名 | 二者在代码中零命中，本版一并消除 |

**相机→刚体变换的取法（替代 `getCameraToRig`）**：

```cpp
data::CameraCalibration calib = calibration.getCalibration(role);

data::Transform cameraToRig = calib.cameraToRig;
```

⚠ `optical::CalibrationManager` 另提供 `lastWarnings()` / `lastErrorText()` 两个
诊断字符串访问器。**V2.3 不将其列入 ICD** —— 与 §15.1 删除 `lastErrorText()` 同一条判据：
失败原因经 `data::ErrorInfo` 上报，不以裸字符串跨层传递。

---

# 10 TargetModel接口

新增。

## 10.1 TargetModelManager

职责：

管理目标模型和特征库。

---

接口：

```cpp id="c4m7px"
class TargetModelManager
{

public:


virtual bool loadModel(
const std::string& path
)=0;


virtual TargetModel get(
CameraRole role
)=0;


virtual bool loaded()
=0;


virtual std::string modelId()
=0;


virtual std::string modelVersion()
=0;


virtual double targetRealSizeM()
=0;


};
```

---

## 10.2 V2.3 修正说明

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `TargetModel getModel()` | `get(CameraRole role)` | 与代码一致。**必须带入参**：三个焦段各有独立特征库（SYS-12 §8），无参接口无法表达"取哪一路" |
| `std::string modelType()` | `std::string modelVersion()` | 代码有 `modelVersion()` 无 `modelType()` |
| — | 增 `loaded` / `targetRealSizeM` | 代码有、V2.2 漏列 |

⚠ **`modelType` 在代码中的真实落点是 `data::MeasurementRecord::modelType` 字段**
（模型来源标记：production / synthetic），**不是** `TargetModelManager` 的方法。
V2.2 在此处的方法声明与记录字段同名，导致两处语义混淆，本版予以分离。

---

输出：

包括：

* modelId；
* modelVersion；
* 3D点；
* 特征库。

---

# 11 坐标转换接口

## 11.1 CoordinateTransformer

职责：

完成：

Camera→Rig→Ship。

---

接口：

```cpp id="m9q3xz"
class CoordinateTransformer
{

public:


virtual Transform cameraToRig(
CameraRole role
)=0;


virtual ShipPoseResult transform(
CameraRole role,
const CameraPose& cameraPose
)=0;


};
```

---

## 11.2 V2.3 修正说明

`transform` 的入参由 `(Transform cameraPose)` 改为 `(CameraRole role, const CameraPose& cameraPose)`：

* **多一个 `role`** —— 坐标链（相机→刚体→舰体）的起点由被测量的那一路相机决定，
  与 `cameraToRig(role)` 是同一参数，不能只凭一个位姿矩阵反推；
* **入参类型是 `CameraPose` 不是 `Transform`** —— `Transform`（`data::Transform`）
  是纯刚体变换矩阵，`CameraPose`（`data::CameraPose`）是带角色信息的相机位姿。

⚠ 实现类 `CoordinateTransformer` 的构造期为 `const CalibrationManager&` 注入，
`rigToShip` **每次向 `CalibrationManager` 取**而非持副本 —— 理由：标定可在运行期重载
（见 §9.1 `load`），持副本会让"重标定后仍用旧外参"变成无报错的错误结果。

---

# 12 Algorithm接口

---

# 12.1 TargetDetector

接口：

```cpp id="v8m4qx"
class TargetDetector
{

public:


virtual bool detect(
const ImageFrame& frame,
DetectionResult& out
)=0;


};
```

---

V2.3 修正：V2.2 写作 `DetectionResult detect(ImageFrame&)`。改为
`bool` 返回 + 出参 —— **未检出以 `false` 表达，而不是 `true` 加 `found=false`**，
编译器强制调用方处理失败分支。

---

# 12.2 TargetScaleEstimator

接口：

```cpp id="x3m7qp"
class TargetScaleEstimator
{

public:


virtual bool estimate(
const DetectionResult& detection,
const CameraCalibration& calibration,
double targetRealSizeM,
TargetScaleEstimate& out
)=0;


};
```

---

V2.3 修正两处：

1. `ScaleEstimate` → **`TargetScaleEstimate`**。`ScaleEstimate` 在代码中零命中
   （仅出现在 `TargetScaleEstimator.h` 的一行注释里作为 `8.md` 的引用），
   实际类型为 `data::TargetScaleEstimate`。**不保留包装类型**；
2. 入参由 1 个补为 4 个。尺度估计需要相机内参（`calibration`）与目标真实尺寸
   （`targetRealSizeM`）才能由像素尺度反推距离 —— V2.2 的单入参形态在几何上不闭合。

---

# 12.3 MeasurementSelector

接口：

```cpp id="k5n8mq"
class MeasurementSelector
{

public:


bool select(
const std::vector<MeasurementCandidate>& candidates,
MeasurementSelectionResult& out
) const;


};
```

---

## 12.3.1 为何不以 CameraRole 为返回值

V2.2 写作 `CameraRole select(std::vector<MeasurementCandidate>)`，
以 `CameraRole::UNKNOWN` 表示"无候选"。本版改为 `bool` + 出参，理由：

```text
ENG-09 §4.1 只冻结了三个角色：CAM25 / CAM50 / CAM100，没有 UNKNOWN。
```

若给 `CameraRole` 加 `UNKNOWN` 哨兵，它会经由**默认构造**悄悄出现在所有含该枚举的
结构体中（`ImageFrame.role`、`MeasurementCandidate.camera` …），
使"这台相机不存在"与"这台相机是 25mm"在内存中不可区分。

返回 `bool` 则编译器强制调用方处理失败分支：
`false` = 无候选可选（`MEASURE_SELECT` 状态按 ~~SYS-08 §7.3~~〔引用无效·依据待裁决·见 Q-D2〕 重试，最多 3 次；
用尽后按 ~~§7.7~~〔引用无效·依据待裁决·见 Q-D2〕 进入 `FAILED`）。
⚠ **次数（3 次）与「用尽即 FAILED」这两条行为本身不变**，失效的只是其来源所指的节号 ——
现行落点在 `MeasurementStrategy`（次数上限与升级）；依据待裁决，见《SYS-08-§7引用勘误.md》。

该偏离**已登记**，见 README §6 第 28 行。本版是将其同步进 ICD。

---

输入：

* 距离；
* 目标像素；
* 清晰度；
* 特征质量。

---

# 13 PosePipeline接口

## 13.1 主接口

**IF-SW-02**：本接口是 **application ↔ algorithm 的唯一调用面**。

```cpp id="h7m3qx"
class IPosePipeline          // 实现类：algorithm::PosePipeline（本接口唯一实现方）
{

public:


virtual ~IPosePipeline() = default;


virtual void setStatisticsObserver(
IPipelineObserver* observer
)=0;


virtual void setCoarseAttitude(
double azimuthDeg,
double elevationDeg,
double distanceM
)=0;


virtual bool detect(
const ImageFrame& frame,
DetectionResult& out
)=0;


virtual bool estimateScale(
const ImageFrame& frame,
const DetectionResult& detection,
TargetScaleEstimate& out
)=0;


virtual bool selectCamera(
const MultiCameraFrame& frame,
const std::vector<CameraRole>& allowed,
MeasurementSelectionResult& out
)=0;


virtual bool selectBestFrame(
const std::vector<ImageFrame>& frames,
int& bestIndex,
ImageQuality& quality
)=0;


virtual bool solvePose(
const MultiCameraFrame& frame,
CameraRole camera,
const CameraCalibration& calib,
ShipPoseResult& out
)=0;


virtual bool validate(
const ShipPoseResult& result,
PoseValidationResult& out
)=0;


};
```

---

## 13.2 V2.3 修正说明

V2.2 §13.1 声明：

```cpp
class PosePipeline
{

public:


bool process(
const MultiCameraFrame& frame,
const CalibrationPackage& calibration,
const TargetModel& model
);


};
```

该声明**两端均不成立**：`process` 在 `src/algorithm/` 零命中；
`CalibrationPackage` 在代码中零命中。本版按代码实况重写为上面的 8 方法接口。

**修正点**：

| V2.2 | V2.3 | 理由 |
|---|---|---|
| `bool process(MultiCameraFrame, CalibrationPackage, TargetModel)` 单方法 | 8 个纯虚方法（`detect` / `estimateScale` / `selectCamera` / `selectBestFrame` / `solvePose` / `validate` + `setStatisticsObserver` / `setCoarseAttitude`） | 算法链是**分阶段可独立调用**的，非单入口。ENG-10 §2.2 的 A/B 两类特征都须进匹配，`MockPoseEstimator` 式的单入口无法承载内点统计 |
| `CalibrationPackage` | **删除该类型名** | 代码零命中。实际标定为 `data::OpticalRigCalibration`（含 `cam25` / `cam50` / `cam100` / `rigToShip`），**构造期注入**而非入参传递 |
| `TargetModel model` 入参 | `TargetModelManager` 构造期注入 | 三个焦段各有独立特征库，整份模型一次性传入无法表达"取哪一路"（同 §10.2） |

**标定与模型的注入方式**（对应 V2.2 那三个入参的去处）：

```text id="r3m9qx"
data::OpticalRigCalibration   →  PosePipeline 构造期注入

algorithm::TargetModelManager →  PosePipeline 构造期注入（裸指针）
```

原因：IF-SW-02 冻结的入参里**没有转台角度、也没有 `rigToShip`**，
而冻结签名承诺产出 `ShipPoseResult.aircraftToShip`（舰体系结果）——
舰体系变换不在任何入参里。三级坐标链（相机→刚体→舰体）由 `PosePipeline`
自行合成，代价是本文件内重复约 30 行 ZYX 分解。
该偏离已登记，见 README §6 第 38 行。

> ⚠ **本节的「IF-SW-02 冻结面」本身仍待处置**：README §6 第 29 行引用
> 「SYS-04 §4.2 冻结的调用面只有 detect / estimateScale / selectCamera /
> selectBestFrame / solvePose / validate」，但 **SYS-04 中不存在 §4.2**。
> 该问题不属本版范围，已独立立项。见 §26。

---

## 13.3 输出

```text id="q8m2px"
ShipPoseResult              ← solvePose 出参

PoseValidationResult        ← validate 出参

MeasurementStatistics       ← 经 IPipelineObserver::onStatistics 推送
```

⚠ `MeasurementStatistics` **不是本接口任何方法的返回值**，而是算法在**产生统计量的
那一刻**经 `IPipelineObserver::onStatistics()` 推送（见 §14）。这样设计是为了避免给
`IPosePipeline` 增加"查询内部状态"的访问器 —— 与 §26 登记的裁决同源。

---

# 14 Pipeline统计接口

## 14.1 IPipelineObserver

用于：

输出算法统计。

---

接口：

```cpp id="r4m8qz"
class IPipelineObserver
{

virtual void onStatistics(
const MeasurementStatistics&
)=0;


};
```

---

数据来源：

```text id="y7m2qx"
FeatureMatcher

↓

IPipelineObserver

↓

MeasurementRecord

```

---

禁止：

Recorder读取：

Pipeline内部变量。

---

# 15 Recorder接口

## 15.1 IRecorderSink

职责：

保存测量结果。

---

接口：

```cpp id="n6m8qx"
class IRecorderSink          // 实现方：infrastructure::Recorder（经 app::RecorderSinkAdapter 桥接）
{

public:


virtual ~IRecorderSink() = default;


virtual bool save(
const MeasurementRecord& record
)=0;


};
```

---

## 15.2 V2.3 修正说明

V2.2 声明 `virtual std::string lastErrorText() = 0;`。该**已删除**，理由三条：

1. ~~**与同类接口形态一致** —— `ICameraBackend`（§6.1）同为设备侧落盘/采集接口，
   亦无 `lastErrorText()`，失败一律经 `data::ErrorInfo` 上报；~~
   ⚠ **V2.4 撤回本条**：其前提已被本版推翻 —— `ICameraBackend` 自 V2.4 起**有**
   `lastErrorText()`（§6.2 第 8 条）。⚠ **删除 `IRecorderSink::lastErrorText()` 的结论不变**，
   判据改为下面两条（原第 2、3 条）＋一条补充：`ICameraBackend` 的 `lastErrorText()`
   承载的是**设备身份诊断文本**（枚举结果与匹配情况），它**不是**错误码通道；
   而 `IRecorderSink` 是**跨层桥接面**（app → infrastructure），
   其失败原因由 `Recorder` 内部持有、且 `save()` 已返回 `bool`，
   回传裸串只会让"按错误码检索结果包与日志"的自动化查询失效。
2. **`save()` 已返回 `bool`**，调用方需要的"成功/失败"已具备；
3. **失败原因不属本层** —— 落盘失败的具体原因在 infrastructure 层的 `Recorder`
   内部，跨层回传裸字符串等于把 `ErrorInfo`（错误码 + 消息 + 时间戳）的职责
   降级成一个无码可检索的串。本项目按错误码检索 `result.json` / 日志的自动化
   查询会因此失效。

⚠ **入参类型说明**：`save()` 的入参是 `data::MeasurementRecord`
（4 字段值传递的 `MeasurementTask` 不足以产出结果包，见 §16）。
该变更由裁决 C-002 交付，代码与本文档一致。

---

保存：

* 原始图像；
* result.json；
* calibration；
* config snapshot；
* 日志。

---

# 16 MeasurementRecord接口

一次测量结果。

```cpp id="z5q7mx"
struct MeasurementRecord
{

MeasurementTask task;


MultiCameraFrame bestFrame;


MeasurementStatistics statistics;


TurntableState turntable;


CameraRole selectedCamera = CameraRole::CAM25;


double selectedScore = 0.0;


ImageQuality selectedQuality;


FailureTrace failure;


bool degraded = false;


int camerasAvailable = 0;


std::string calibrationId;


std::string modelId;


std::string modelType;


std::string softwareVersion;


};
```

---

## 16.1 V2.3 修正说明

V2.2 只列出 6 项**概念**（`MeasurementTask` / `MultiCameraFrame` / `TurntableState` /
`MeasurementStatistics` / `TargetModel信息` / `FailureTrace`），本版补全为 14 字段实况。

V2.2 的概念项与字段的对应关系全部成立：

```text id="t7m4qx"
MeasurementTask         →  task

MultiCameraFrame        →  bestFrame

TurntableState          →  turntable

MeasurementStatistics   →  statistics

TargetModel信息          →  modelId + modelType

FailureTrace            →  failure
```

本版补入的字段及其承载要求：

| 字段 | 承载 |
|---|---|
| `selectedCamera` / `selectedScore` / `selectedQuality` | ENG-10 §4.1 选择结果三元组 |
| `degraded` / `camerasAvailable` | **~~SYS-08 §7.5~~ 的承载字段** ⇒ ⚠ **该节号不存在**（**引用无效，依据待裁决**，见《SYS-08-§7引用勘误.md》**Q-D2**）。**但「降级必须在 UI 可见」这一要求在工程上仍被执行**：它的效力来源是实施期自造的工程约定 + 既有测试，**不是冻结文档**；且降级策略本身与 `SYS-08 §17.1` 冲突（**Q-D1**）。V2.2 漏列这两项会使该要求在 ICD 上找不到落点 —— **字段保留，理由改写** |
| `calibrationId` / `modelId` / `modelType` / `softwareVersion` | 结果包的可追溯性 —— 记录必须能回答"这次测量用的是哪份标定、哪个机型库、哪个软件版本" |

⚠ `modelType` 为**模型来源标记**（production / synthetic），非管理器方法名
（见 §10.2）。SYS-02 要求 release 阶段禁止 synthetic 模型进入测量路径，
本字段是该约束在结果包上的唯一判据。

---

# 17 FailureTrace接口

用于：

失败追踪。

结构：

```cpp id="p4m7xz"
struct FailureTrace
{

MeasurementState firstFailedState;


ErrorInfo firstError;


MeasurementState finalFailedState;


ErrorInfo finalError;


std::vector<StateTransition>
history;

};
```

---

# 18 Application接口

## 18.1 MeasurementController

接口（公开面）：

```cpp id="h8m3qy"
class MeasurementController : public IPipelineObserver
{

public:


virtual ~MeasurementController() = default;


/---- 控制 ----/

void startMeasurement();


void startMeasurement(
uint64_t nowNs
);


void stopMeasurement();


bool tick(
uint64_t nowNs
);


/---- 状态 ----/

MeasurementState state() const;


ErrorInfo lastError() const;


bool degraded() const;


ErrorInfo degradationNotice() const;


int availableCameraCount() const;


uint64_t remainingNs(
uint64_t nowNs
) const;


int attempts(
MeasurementState state
) const;


int rollbackCount() const;


/---- 结果 ----/

CameraRole selectedCamera() const;


TargetOffset lastOffset() const;


ImageQuality bestQuality() const;


ShipPoseResult poseResult() const;


PoseValidationResult validationResult() const;


MeasurementTask task() const;


/---- 统计（IPipelineObserver）----/

void onStatistics(
const MeasurementStatistics& statistics
) override;


MeasurementStatistics statistics() const;


bool statisticsReceived() const;


std::vector<std::string> notices() const;


};
```

---

## 18.2 V2.3 修正说明

V2.2 只列 3 项（`startMeasurement` / `stopMeasurement` / `state`），
三者均在代码中存在，差异全在**漏列**。本版补全公开面。

⚠ **`poseResult()` / `validationResult()` 是结果读取，不是内部状态查询。**
它们返回控制器**已收口的结果包内容**（与 `MeasurementRecord` 同源），
不穿透到算法层私有成员 —— 与 §26 登记的被否决形态（给 `IPosePipeline` 增加
`lastMatchResult()` 之类的访问器）性质不同。

`degraded()` / `degradationNotice()` / `availableCameraCount()` / `notices()`
承载 ~~SYS-08 §7.5~~〔引用无效·依据待裁决·见 Q-D2〕 的降级可见性要求（与 §16 `degraded` / `camerasAvailable` 同源）。⚠ 该节号**不存在**，见《SYS-08-§7引用勘误.md》；**行为不变**。

---

# 19 AlignmentController接口

输入：

```text id="w6m9qx"
TargetOffset
```

---

输出：

```text id="q3m8px"
TurntableCommand
```

---

禁止：

Algorithm直接调用。

---

# 20 Preview接口

## 20.1 PreviewManager

接口：

```cpp id="x7m4qz"
class PreviewManager
{

public:


bool setCamera(
CameraRole role
);


void setMode(
PreviewMode mode
);


virtual bool getFrame(
PreviewFrame& frame
);


};
```

---

## 20.2 PreviewFrame

```cpp id="w4n7qx"
struct PreviewFrame
{

ImageFrame frame;


uint64_t displayTimestamp = 0;


};
```

**PreviewFrame 为 ImageFrame 的显示层包装数据。** 它不是与 `ImageFrame` 并列的
另一种图像类型，而是在图像之外附带一个**显示时间戳**（供预览刷新节奏判断）。

故 `getFrame()` 的"取一帧图像"与"取一个（图像 + 显示时间戳）二元组"**是包含关系，
不是类型冲突**。V2.2 写作 `getFrame(ImageFrame&)`；本版按代码实况写作
`PreviewFrame&`，并在此说明二者关系。

⚠ `getFrame` 的消费者是 **GUI 线程的定时重绘**（单消费者）。它记录"上次已交付的序号"，
**从两个线程调用会让先到者把帧吃掉**。窗口最小化后 Qt 仍派发定时器事件，
按可见性启停由 `showEvent` / `hideEvent` 负责。

---

# 21 错误接口

## ErrorInfo

```cpp id="m5q8xz"
struct ErrorInfo
{

int code = 0;


std::string message;


uint64_t timestampNs = 0;

};
```

---

## 21.1 关于 ErrorCode

**`ErrorCode` 是逻辑概念，当前实现为 `int` 段码。**

```text id="n8q5mx"
逻辑概念：  ErrorCode（一个具名的错误码类型）

当前实现：  int + 段常量

后续演进：  统一枚举化
```

V2.3 在此如实标注这一状态，**不将 `ErrorCode` 写作已存在的类型**。
代码中该类型零命中；错误码由 `int` 常量承载，按 ENG-09 §5.27 分段：

```text id="p3m7qx"
1000 ~ 1999   相机

2000 ~ 2999   转台

3000 ~ 3999   触发 / 同步

4000 ~ 4999   标定

5000 ~ 5999   模型与特征库

6000 ~ 6999   算法

9000 ~ 9999   系统级
```

**V2.4 新增的相机段错误码（`011-A1`，裁决 C-015）：**

```text id="k9v3mx"
1003   相机取帧超时或无有效帧

1004   相机通道未就绪或未启动

1005   相机 SDK 错误或帧不可用

1006   取帧契约错误或参数非法（本地判定，未调用 SDK）

1007   请求的相机格式/模式本批未实现
```

⚠ **`1005` 与 `1006` 不得混用**：前者是"**相机给的帧有问题**"，
后者是"**我们自己的代码或配置有问题**" —— 现场动作完全不同。
语义的逐值对应关系（哪个 `OpStatus` 产出哪个码）见 ENG-09 V2.5 §5.29 的状态全表。

⚠ `code == 0` 的含义是**"未设置"**，不是"成功"，也不得作为失败码使用。

**`ErrorCode` 枚举化属设计层改动，不属本次文档一致性修正范围** ——
本版只消除"文档把一个不存在的类型当作已冻结事实"这一处不一致。
枚举化时应作为独立裁决处理，并同步 `errorCodeName()` 的 `switch`。

---

## 21.2 错误必须携带

* **错误码** —— `code`，须落在上表段内；
* **状态** —— **由调用方所在的状态机上下文承载，不在本结构体内**。
  `FailureTrace`（§17）的 `firstFailedState` / `finalFailedState` 即该信息；
* **原因** —— `message`，须能回答"为什么"，不得只复述错误码名。

> V2.2 同节写作「错误码；状态；原因」三项而结构体只有两字段。本版补充说明第三项
> （状态）的实际承载位置 —— 它并非缺失，而是刻意不由 `ErrorInfo` 承担：
> 同一个错误码在不同状态下处置不同（~~SYS-08 §7.3~~ ⇒ **引用无效，依据待裁决（Q-D2）**：
> `SYS-08 V2.1` 的 `§7.3` 是 TARGET_FOUND 状态的"输出"节，与本处所指的"次数表／重试策略"
> **内容完全不同**；现行重试策略的落点在代码 `MeasurementStrategy`／`RetryManager`，
> 依据待裁决，见《SYS-08-§7引用勘误.md》），
> 把状态塞进错误结构体会使两者的唯一性互相污染。

---

# 22 接口依赖关系

冻结：

```text id="w9m2qx"
Data

↑

Device

Optical

Algorithm

Application

UI

```

---

# 23 禁止接口

禁止：

## Algorithm控制设备

错误：

```text id="v4m7qx"
PnP

↓

Turntable
```

---

禁止：

## UI访问SDK

错误：

```text id="k8m3py"
MainWindow

↓

ImvSDK
```

---

禁止：

## Recorder访问私有状态

错误：

```text id="q6m8xz"
Recorder

↓

PosePipeline private
```

---

# 24 接口版本管理

接口修改必须：

* 更新ICD；
* 更新测试；
* 更新版本号。

当前：

```text id="m7q2xz"
ICD_V2.3
```

---

# 25 设计冻结总结

| 接口                 | 状态   | V2.3 变动 |
| ------------------ | ---- | ------- |
| Camera接口           | 冻结   | 未改（§6.1 逐方法吻合） |
| MultiCamera接口      | 冻结   | 补 `lastError()`（§5.2） |
| Trigger接口          | 冻结   | 未改 |
| Turntable接口        | 冻结   | 未改 |
| Calibration接口      | 冻结   | 类名 `ICalibrationManager` → `CalibrationManager`；删 `getCameraToRig`、改名、去包装类型（§9.2 / §26.2） |
| Coordinate接口       | 冻结   | 类名 `ICoordinateTransformer` → `CoordinateTransformer`；`transform` 补 `role` 入参（§11.2 / §26.2） |
| Algorithm接口        | 冻结   | 三个签名改为 `bool` + 出参（§12）；`ScaleEstimate` → `TargetScaleEstimate` |
| TargetModel接口      | 新增冻结 | 类名 `ITargetModelManager` → `TargetModelManager`；`getModel()` → `get(role)`、`modelType()` → `modelVersion()`（§10.2 / §26.2） |
| PosePipeline接口      | 冻结   | `process()` → 8 方法实况（§13.2） |
| Recorder接口         | 冻结   | 删 `lastErrorText()`（§15.2） |
| PipelineObserver接口 | 冻结   | 未改（逐字吻合） |
| MeasurementRecord   | 冻结   | 6 概念 → 14 字段（§16.1） |
| FailureTrace        | 冻结   | 未改（5 字段逐字段吻合） |
| Preview接口          | 冻结   | `getFrame` 出参 → `PreviewFrame`（§20） |
| 错误接口              | 冻结   | `ErrorCode` 标注为逻辑概念（§21） |

---

# 26 待处置项

本次代码一致性审计提出两项：

* **§26.1 IF-SW-02 冻结面缺失** —— 不属本版修正，独立立项；
* **§26.2 接口类的实现形态** —— 已在本版内裁决并实施。

## 26.1 IF-SW-02 冻结面缺失

`IF-SW-02` 与「SYS-04 §4.2」被代码、`README.md` §6、`V2.1-C01` 共 11 处
作为"冻结的算法层调用面"引用，但：

```text id="v2q8mx"
SYS-04 中不存在 §4.2

SYS-04 中不存在任何 IF-SW-xx 编号

IF-SW-02 在 项目文档/ 全目录零命中
```

READM 引用其冻结面为 `detect / estimateScale / selectCamera / selectBestFrame /
solvePose / validate` 六个方法，而这**六个方法名在 SYS-04 全文一次都未出现**
（V2.2 §13.1 写的是 `process`；V2.3 §13.1 已按代码实况重写为 8 方法）。

**性质**：接口能力缺失，不是接口文档错误。

**处置**：独立立项 —— `IF-SW-02_PipelineObserver接口一致性修正`。
需同时决定：冻结面是 6 方法还是 8 方法（差 `setStatisticsObserver` /
`setCoarseAttitude` 两个由后续裁决加入的合法扩展），以及 IF-SW 编号体系是否重建。

## 26.3 本版新登记的待处置项（V2.4）

**1 取帧失败的"降级策略"仍无冻结依据，且与 `SYS-08 §17.1` 冲突（待裁决）。**
本版把**分类**做清楚了（超时／无帧／未就绪／断连／SDK 错误／帧损坏分别可读），
但"**分类之后该怎么处置**"（等待／重试／降级继续／终止）**一字未改** ——
它属 **Q-D1**（《待裁决问题汇总》），**不得标为已批准**。
⚠ 本版的分类**不得**被用来论证"真实硬触发失败后可自动改软触发验收"之类的结论。

**2 时限体系的零余量（待裁决）。**
`capture_frame_count` / `grab_timeout_ms` / `capture_timeout_ns` / `grab_group_budget_ns`
这几个值**没有冻结依据**，且 `5 × 300 ms = 1.5 s` 与 `capture_timeout_ns` 零余量
（复制／格式转换／评分未计入）⇒ CAPTURE 在真实相机上**可能被时限截断**。
属 **Q-D2** 的附带问题，见 ENG-09 V2.5 §12.3 第 4 条。

**3 实机核实项（`-119` 即超时、`timeoutMS = 0`、`-114/-115/-116` 的断连语义、
`paddingX/Y` 语义、字节序声明、"Off 即自由运行"）。**
逐条在册于 **ENG-09 V2.5 §13 附录 A / §14 附录 B**；
⚠ 凡落在"**推断**"或"**未文档化**"行的实现，**一律只记"已实现／待实机核实"，不得记"已验证"**。

**4 `kCallThrewCode` 的显示措辞只有一个出口（V2.6 已实施）。**
`data::kCallThrewCode`（`INT32_MIN`）是"**调用抛出异常、无返回码**"的**标记值，不是 SDK 码**。
要求：它**只**能被翻译成措辞，**任何**打印路径都**不得**把这个数字当 SDK 原码显示 ——
`-2147483648` 会被现场读成一个真实错误码，从而白白去查一个不存在的码。

⚠ **本条在写进 V2.6 时先记成"当前实现符合要求、只登记" —— 该结论是错的，同批已更正。**
当时的措辞是"只在一处被翻译（`noteGrabException`），其余打印路径只走 `sdkCallName()`
与码值分列"。按语义检索核对后的事实是：**全仓 10 处**把 `SdkFailure::code` 写进人读文本，
其中**只有 1 处**（`noteGrabException`）带措辞分支，另外 9 处**都**是
`std::to_string(code)`／`%d` 直接打印 —— 也就是说，标记值本来就会在
`ImvCameraBackend` 的清理文本、`ChannelGrabRecord` 的记录文本、
`SystemInitializer` 的失败摘要、`ApplicationContext` 的关闭告警里被念成数字。
"分列"不是保护：**码值那一列正是出问题的位置**。

**本批的实施方式（不是登记）**：`data::OpStatus.h` 增两个 inline 助手
`data::sdkFailureCodeText()`（唯一渲染码或措辞的地方）与
`data::sdkFailureText()`（`<调用名> 返回 <码>` ／ `<调用名> 调用抛出异常、无返回码`），
**10 处全部改走它**，其中 `cleanupUnconfirmedText()` 与 `noteGrabException()` 里
各自手写的一份措辞分支**删除**（同一句话有两份实现，分叉的那一刻就会有一处重新开始念数字）。
**机器读的字段不受影响**：`OperationResult::sdkError->code` 仍原样保留 `INT32_MIN`，
读包方按同一份 `data` 层定义解释（见 ENG-09 V2.5 §5.29）。

**验证**：`DeviceLayerTest` 的 `ThrownReleaseMarkerNeverReachesHumanReadableText`
（经真实 `MultiCameraManager` 走"释放调用抛出"这条**真实可达**的路径 ——
`FrameLeaseGuard::cleanup()` 是 `noexcept` 的，把抛出如实记成 `{ImvReleaseFrame, kCallThrewCode}`）
逐条断言两个出口出现措辞、且都**不出现** `-2147483648`；
定向变异（把 `sdkFailureCodeText` 的抛出分支改为 `std::to_string`）令 **5 条**断言变红
（记录文本 2 条、通道停止原因 2 条、异常路径诊断 1 条）。

---

## 26.2 接口类的实现形态（✅ 已裁决，已实施）

§9.1 / §10.1 / §11.1 原声明的 `ICalibrationManager` / `ITargetModelManager` /
`ICoordinateTransformer` 三个抽象接口类在代码中**零命中** ——
代码提供的是具体类。即：**方法级一致，类型级为虚构**。

**裁决（2026-09-24）**：**不保留 `I` 前缀，按代码收敛为具体类名。**

```text id="y6m2qx"
ICalibrationManager     →  CalibrationManager

ITargetModelManager     →  TargetModelManager

ICoordinateTransformer  →  CoordinateTransformer
```

理由：C++ 的接口抽象**不必须** `I` 前缀。项目已进入工程实现阶段，ICD 要服务开发、
集成与测试；正文写 `ICalibrationManager` 而代码是 `CalibrationManager`，
会持续产生误解 —— 而这正是本版要消除的那类不一致。

⚠ **本节的收敛只针对这三个类。** 其余六个 `I` 前缀接口
（`ICameraBackend` / `IMultiCameraManager` / `ITriggerController` /
`ITurntableController` / `IPipelineObserver` / `IPosePipeline`）
**在代码中真实存在，一律保留原名**，不改。判据是"代码里有没有这个名字"，
不是"有没有 `I` 前缀"。

---

本文档作为 AircraftPoseSystem V2.1 多相机闭环测量系统接口控制基线。

---

# 27 修订记录

| 版本 | 日期 | 修订内容 |
|---|---|---|
| V2.2 | — | 多相机标定融合与闭环测量版。新增 CalibrationManager / TargetModelManager / MeasurementRecord / FailureTrace / Recorder / 真实硬件接入的接口补充 |
| **V2.3** | **2026-09-24** | **代码一致性修正版**。按代码基线逐条修正 12 处接口声明不符，消除 5 个虚构类型名（`CalibrationPackage` / `CameraRigTransform` / `RigShipTransform` / `ErrorCode` / `ScaleEstimate`）与 3 个虚构接口类名（`ICalibrationManager` / `ITargetModelManager` / `ICoordinateTransformer`，收敛为具体类名）。**不改变软件架构、不新增接口、不改变代码**。审计依据：`AircraftPoseSystem/V2.1-SYS04不一致逐条对照.md` |
| **V2.4** | **2026-09-26** | **取帧契约与诊断出口版**（011-A1 数据契约批，裁决 **C-015**）。⚠ **本文件第一条动了既有冻结签名的修订**：`ImageFrame` 增 `raw` / `captureFormat` / `rawPolicy`；`IMultiCameraManager::capture` 增 `deadlineNs`、新增纯虚 `lastCaptureRound()`；`ICameraBackend` 由 6 方法扩为 11 方法（五方法返回 `OperationResult`、`grab` 增 `timeoutMs` 并返回 `GrabResult`，新增 `close()` / `triggerSoftware()` / `deviceIdentity()` / `triggerModeState()` / `lastErrorText()`）；`setTriggerMode` 入参由 `bool` 改三值枚举；新增错误码 1003-1007。**撤回 §15.2 的理由 ①**（结论不变）。`SYS-08 §7.x` 引用就地划掉并标注失效（**Q-D2**，行为与字段一字未改）。详见 §1.3 / §1.4 |
| **V2.5** | **2026-09-26** | **失败边界与清理责任版**（011-A1 九项缺口定向修复批，配套裁决 **C-01 v1.8**）。⚠ **零签名变更**：§5.1 / §6.1 与 V2.4 逐字相同，改的是**调用责任** —— 新增 §5.4（`capture()` 的三个期限检查点、触发前置检查的三分支"不发令／不取帧"、失败时输出帧不被触碰）；§5.3 / §6.2 第 3／5／9 条补"清旧留新""清理结果合并与首因保留""读回三分支""失败交付"；§6.2 新增第 10 条（期限传递与"无期限 vs 已到期"）与第 11 条（取帧异常安全）。**不新增错误码**。全文件对 ENG-09 的版本指向改为 **V2.4**。详见 §1.5 / §1.6 |
| **V2.6** | **2026-09-26** | **清理确认与异常保全版**（011-A1 清理责任与异常安全批，配套裁决 **C-01 v1.9**）。⚠ **零签名变更**：§5.1 / §6.1 与 V2.4／V2.5 逐字相同，改的是 V2.5 **没写完的那一半** —— 三条"同一件事、两种处置"的残留：① `cleanupError` **两个入口都写** ⇒ 消费侧第一判据改为"`cleanupError` 有值即停止该路后续采集"（帧检查通过时禁用、帧已损坏时继续可用这对矛盾消除）；② 异常出口**三步各自独立保护**、末尾 `throw;` **无条件执行**（原异常不再可能被二次异常顶掉）；③ **判定类别与实际调用历史分别记录**（撤回 V2.3 起的"本地判定 ⇒ `sdkError` 必须为 `nullopt`"，它把触发读回不一致这条**确实调用过**的路径判成了"必须为空"）；④ §26.3 第 4 条由"登记"改为"已实施"（`kCallThrewCode` 显示措辞收敛到唯一助手）。**不新增错误码**。R09 仍为"**部分修复**"。全文件对 ENG-09 的版本指向改为 **V2.5**。详见 §1.7 / §1.8 |

---

## 27.1 版本关系

| 文件 | 状态 |
|---|---|
| `SYS-04_接口控制文件ICD_V2.6_清理确认与异常保全版.md` | **唯一有效版本** |
| `archive/SYS-04_接口控制文件ICD_V2.5_失败边界与清理责任版.md` | 已归档，仅作历史留存，**不再作为接口依据** |
| `archive/SYS-04_接口控制文件ICD_V2.4_取帧契约与诊断出口版.md` | 已归档，仅作历史留存，**不再作为接口依据** |
| `archive/SYS-04_接口控制文件ICD_V2.3_代码一致性修正版.md` | 已归档，仅作历史留存，**不再作为接口依据** |
| `archive/SYS-04_接口控制文件ICD_V2.2_多相机闭环测量版.md` | 已归档，仅作历史留存，**不再作为接口依据** |

⚠ **按"一文件一版"约定**，旧版以移动至 `archive/` 的方式留存而非删除 —— 保留审计链。
引用 SYS-04 时一律指向 **V2.6**。
⚠ V2.3 的 §4.1 / §5.1 / §6.1 描述的是**扩充前**的接口面，
按 V2.3 实现会与 V2.6 的契约不符（`bool` 返回值无法表达失败分类）；
⚠ **V2.4 的签名仍然有效，但其行为边界描述不完整** ——
按 V2.4 实现会把"已发令、未取帧""读回未知被回退成请求值""清理失败覆写首因"三条当成合规。
⚠ **V2.5 的签名与行为边界均仍有效，但清理责任的描述只写了一半** ——
按 V2.5 实现会出现"同一个『释放未获确认』在两条路径上拿到相反处置"
（帧检查通过 ⇒ 禁用通道；帧已损坏 ⇒ 继续可用），且异常出口只保护第 ② 步。
∴ **V2.5 及更早版本均不得再作接口依据**。

**与 ENG-09 的版本对应**：本文件管**接口面**，类型与字段的**来源**在 ENG-09。
当前有效版本为 **SYS-04 V2.6 ＋ ENG-09 V2.5**，两者**同批发布、须成对引用**。
⚠ 本版与 ENG-09 V2.5 是**同一批**：`cleanupError` 的两个入口、
`kCallThrewCode` 的显示助手、`SdkFailure` 的"判定类别 vs 调用历史"三条
在两边**逐条对应**；只引用一边会得到自相矛盾的两份契约（ENG-09 V2.5 §12.2 同有此警告）。
