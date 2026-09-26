# SYS-04_ICD接口控制文件_V2.4（取帧契约与诊断出口版）

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
| 7 | 21.1 | 错误码段新增 1003 / 1004 / 1005 / 1006 / 1007（语义以 ENG-09 V2.3 §5.29 为准） | 新增 |
| 8 | 12.3 / 16 / 18.1 / 21.2 | `SYS-08 §7.x` 引用**就地划掉并标注失效**（依据待裁决 Q-D2），**行为与字段一字未改** | **引用勘误** |
| 9 | 27.1 | 版本关系更新（V2.3 归档） | — |

**未改动**：7.1 `ITriggerController`（仍是硬触发与 3001 降级的抽象，
**软件触发的发令不在本接口** —— 见 6.2）、8.1 `ITurntableController`、
14.1 `IPipelineObserver`、17 `FailureTrace`、19 `AlignmentController`、23 禁止接口、
以及 §12.1 / §12.2 / §13.1 / §20.1 的方法级声明。

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
（类型定义以 **ENG-09 V2.3 §5.28** 为准，本文件不重复冻结字段表）：

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
`pixel_format` / `valid_bits` / `packing` / `declared_byte_order`（ENG-09 V2.3 §5.28）。
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

**契约（类型定义以 ENG-09 V2.3 §5.32 为准）：**

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
类型定义以 **ENG-09 V2.3 §5.28～§5.31** 为准，本文件只冻结**接口面**。

---

## 6.2 V2.4 对 6.1 的说明

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
⚠ **本地判定类错误（`ContractViolation` / `InvalidArgument`）不得携带 `sdkError`**：
它们**根本没调用 SDK**，伪造一个"调用过"的失败会让排查指向设备侧。

**3 三级释放的归属**（本版新增 `close()` 的全部理由）：

| 层次 | 谁做 | 时机／幂等要求 |
|---|---|---|
| **停流** `IMV_StopGrabbing` | `stop()`（**语义不变**：允许未 start 时调用、须无副作用） | `stopAll()` 照旧"无论 available 与否都调" |
| **关设备 + 销毁句柄** `IMV_Close` / `IMV_DestroyHandle` | **`close()`**：幂等、允许在未 `initialize()` 或已 `close()` 时调用、无副作用 | 由调用方在"设备不再使用"时显式调用；**`stop()` 不等同于 `close()`** |
| **失败出口** | `initialize()`／`start()` **自己** | 失败时释放**它自己**已建立的半初始化资源，**不得**留下打开的句柄 |

⚠ 断言纪律：**看的是底层资源，不是包装调用次数** —— `close()` 这一层幂等包装**允许**
被拥有者与析构兜底各调一次；要防的是"**再次调用产生第二次底层销毁**"。

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
（那会让一次读失败伪装成"模式正确"）。映射表与读回规则见 ENG-09 V2.3 §5.31。

**6 `deviceIdentity()` 与型号匹配。**
⚠ **型号匹配不能替代设备身份匹配**：打开相机按**序列号**精确匹配；
配置里序列号为空 ⇒ **明确失败**，**不取"第 0 个设备"**（那会让三台同型号相机随机互换）。
未取得身份就写"未取得"，**不得**从配置复制目标序列号去填实际身份。

**7 诚实桩的形态（不以 `bool` 退化）。**
未找到 SDK 与"找到 SDK 但适配未完成"是**两件不同的事**，必须给**不同文本**
（后者要写明"**是代码没写、别去查硬件和配置**"）；两者都返回 `NotImplemented`
且 `sdkError == nullopt`（**根本没调用 SDK**，符合 ENG-09 V2.3 §5.29 第一条语义）。

**8 `lastErrorText()` —— 与 §15.2 删除的那个同名方法无关。**
本方法承载的是**设备身份诊断文本**（枚举到的型号／序列号、按序列号匹配的结果），
供装配摘要与失败报告打印；它**不是**错误码通道（错误码走 `OperationResult` 与 `ErrorInfo`）。
⚠ 设备层内 `lastErrorText()` 是**既有多处实例的共同形态**
（`PekoTurntableController` / `VirtualTriggerController` / `HardwareTriggerController` 均有），
本版把该形态提到接口上，是因为**只有后端拿得到那些设备侧文本**，
而装配层报"接入失败"时若只能贴一句"（无附加说明）"，失败报告就指向一个不存在的东西。

**9 `@return` 逐值语义。** 各 `OpStatus` 取值的完整含义、应用码与聚合严重度
见 **ENG-09 V2.3 §5.29 的状态全表**（本文件不重复）。⚠ 其中两条**不得违反**：
`status != Ok` 时 **`frame` 不被触碰**（不写任何字段，包括不写部分字段 ——
写半成品会诱导调用方误用）；**`Timeout` 不得覆盖 `CorruptFrame`**
（"没等到帧"与"调用成功但帧坏了"处置不同：前者查触发与链路，后者查格式与布局）。

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
语义的逐值对应关系（哪个 `OpStatus` 产出哪个码）见 ENG-09 V2.3 §5.29 的状态全表。

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
属 **Q-D2** 的附带问题，见 ENG-09 V2.3 §12.3 第 4 条。

**3 实机核实项（`-119` 即超时、`timeoutMS = 0`、`-114/-115/-116` 的断连语义、
`paddingX/Y` 语义、字节序声明、"Off 即自由运行"）。**
逐条在册于 **ENG-09 V2.3 §13 附录 A / §14 附录 B**；
⚠ 凡落在"**推断**"或"**未文档化**"行的实现，**一律只记"已实现／待实机核实"，不得记"已验证"**。

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

---

## 27.1 版本关系

| 文件 | 状态 |
|---|---|
| `SYS-04_接口控制文件ICD_V2.4_取帧契约与诊断出口版.md` | **唯一有效版本** |
| `archive/SYS-04_接口控制文件ICD_V2.3_代码一致性修正版.md` | 已归档，仅作历史留存，**不再作为接口依据** |
| `archive/SYS-04_接口控制文件ICD_V2.2_多相机闭环测量版.md` | 已归档，仅作历史留存，**不再作为接口依据** |

⚠ **按"一文件一版"约定**，旧版以移动至 `archive/` 的方式留存而非删除 —— 保留审计链。
引用 SYS-04 时一律指向 **V2.4**。
⚠ V2.3 的 §4.1 / §5.1 / §6.1 描述的是**扩充前**的接口面，
按 V2.3 实现会与 V2.4 的契约不符（`bool` 返回值无法表达失败分类）——
**两版均不得再作接口依据**。
