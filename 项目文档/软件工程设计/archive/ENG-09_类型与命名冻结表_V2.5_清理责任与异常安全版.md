# AircraftPoseSystem 类型与命名冻结表 V2.5（清理责任与异常安全版）

## 1 文档目的

本文档定义 AircraftPoseSystem V2.1 唯一权威的数据类型、命名与单位基线。

编写背景：

V2.1 设计文档采用"每份文档自包含"的编写方式，同一类型在 SYS/ENG 多份文档中重复定义，已发生实际漂移：

- 同一 struct 在不同文档中字段不一致（`ImageFrame`、`CameraCalibration`、`CameraChannel`）；
- 同名类型含义相反（`TurntableState` 在同一版本中既是 struct 又是 enum）；
- 坐标变换方向不统一，姿态字段名与坐标链方向相反；
- 存在 5 个被引用但从未定义的类型（`TrackingResult`、`FeatureSet`、`FeatureDatabase`、`ErrorInfo`、`PoseResult`）；
- 角度与长度单位从未定义。

编码开工前必须收敛。本文档是收敛结果。

> **V2.5 相比 V2.4 的变更（2026-09-26，011-A1 清理责任与异常安全批）**
>
> **对既有小节做语义修订，不新增小节、不动任何既有节号**（四类）：
> ① **§5.29 废除"本地判定 ⇒ `sdkError` 必须为 `nullopt`"这条判据**，改为
> **"判定类别与实际调用历史分别记录"**（`Unset` 通道同步改写；1006 的"来源"列去掉"未调用 SDK"）；
> 新增 `data::kCallThrewCode`（**调用抛出异常、无返回码**，**不是** SDK 码，显示必须走专用措辞）；
> ② **§5.29 新增"清理责任"两条**：`cleanupError` 是"**资源还回去了没有**"的**唯一**字段
> （主操作成功时的清理失败**也**进它）、**释放未获确认 ⇒ 停止该路后续采集**（首因一字不动）；
> ③ **异常出口冻结为三步各自独立保护**（描述异常／显式清理／写诊断），末尾 `throw;` **无条件执行**；
> `FrameLeaseGuard::cleanup()` **`noexcept`** 且**不重试**；
> ④ **§5.28 结果包键集按数据来源分组**：`sdk_pixel_format_code`／`sdk_padding_x`／`sdk_padding_y`
> 与三个长度键**只在真身路径写出**（回落路径**缺键**胜过**写 0**）。
> ⚠ 本版**未新增任何错误码、未关闭 §12.3 的任何登记项**：Q-D1／Q-D2 原样在册，
> R09 仍为"**部分修复**"。配套裁决见 **C-01 v1.9**；
> 类型与字段的**来源**在本文档，**接口签名**在 **SYS-04 V2.6 §6.1／§5.1** 与 **SYS-06 V2.5 §5.1**。
> V2.4 关于**帧状态**、**触发读回三分支**、**`raw_image`**、**`kNoDeadlineNs`** 的四条结论**依然有效**。
> 详细修订记录见 §12。

> **V2.4 相比 V2.3 的变更（2026-09-26，011-A1 九项缺口定向修复批）**
>
> **对既有小节做语义修订，不新增小节、不动任何既有节号**（五类）：
> ① **§5.29 `GrabResult` 增两个诊断字段**（`frameStatusRaw`／`diagnosis`）并冻结其传递路径；
> ② **帧状态归类冻结**：`IMV_FrameInfo::status != 0` ⇒ **只**推出 `CorruptFrame`，
> **删除** V2.3 的"任意非零帧状态即可推出 `NoFrame`"表述；**"报 `Ok` 却交付空图" ⇒ `ContractViolation`**，
> 且**调用诊断原样转发、不补造**；
> ③ **§5.31 触发读回逐项化**（`TriggerFeatureReadback`）并给出三分支处置表，
> **删除**"读回为空 ⇒ 用请求值"的三元表达式；
> ④ **§5.28 结果包增 `raw_image`**（裸缓冲自己的宽高，**仅真身路径写出**），
> 落盘复检改为**四个长度事实互证**；
> ⑤ **§2.5 增"无期限 vs 已到期"**（`data::kNoDeadlineNs`）；**§5.32 增每轮"清旧留新"表与三个预算检查点**。
> ⚠ 本版**未新增任何错误码、未关闭 §12.3 的任何登记项**：Q-D1／Q-D2 原样在册，
> R09 仍为"**部分修复**"。配套裁决见 **C-01 v1.8**；
> 类型与字段的**来源**在本文档，**接口签名**在**当时**的 SYS-04 V2.5 §6.1／§5.1 与 SYS-06 V2.4 §5.1（SYS-04 现已升至 **V2.6**）。
> V2.3 的 SDK 事实核验结论（§13／§14）**依然有效**（V2.3 全文见 `archive/`），本版未回退其任何结论。
> 详细修订记录见 §12。

> **V2.3 相比 V2.2 的变更（2026-09-26，011-A1 数据契约批）**
>
> 新增四组类型与两处字段/枚举变更，**并更正本文档 §5.27 与 §9.2 中一批已失效的 `SYS-08 §7.x` 依据**：
> ① **新增 §5.28 `RawImagePayload` 及其配套枚举**（`PixelFormat`／`Packing`／`ByteOrder`／
> `BitAlignment`／`RawDataPolicy`）；② **新增 §5.29 取帧结果与诊断出口**（`OpStatus`／`SdkCall`／
> `SdkFailure`／`OperationResult`／`GrabResult`）；③ **新增 §5.30 `DeviceIdentity`**；
> ④ **新增 §5.31 `CameraTriggerMode`／`TriggerModeState`**；⑤ **§5.5 `ImageFrame` 增三个字段**
> （`raw`／`captureFormat`／`rawPolicy`）；⑥ **§4.1 的 `DeviceState` 增一个值 `DISCONNECTED`**；
> ⑦ **§6.1 `CameraConfig` 增 `serialNumber`／`backend`，`triggerMode` 由 `bool` 改三值枚举**；
> ⑧ **§5.27 增 5 个错误码**（1003／1004／1005／1006／1007）并**改写全部 `SYS-08 §7.x` 来源列**；
> ⑨ 新增 **§13 附录 A**／**§14 附录 B**，把本批用到的 **SDK 事实核验结论**与**样例核查结果**逐项冻结
> （每条**保留"文档支持／样例支持／推断／未文档化"的标注**，其中**有一条被更正**，留痕见 §14）。
> ⚠ **本版是本文档第一条动了既有冻结签名的配套变更**（配套裁决见 C-01 v1.7 的 **C-015**）：
> 类型与字段的**来源**在本文档，**接口签名**在当时的 SYS-04 V2.4／SYS-06 V2.3。
> ⚠ 其中两条结论**已被 V2.4 删除**（§5.29 的"非零帧状态即可推出 `NoFrame`"、§5.31 的三元读回），
> 引用 V2.3 的这两处时以 V2.4 为准。
> V2.2 的 E 单位换算修正**依然有效**（V2.2 全文见 `archive/`），本版未回退其任何结论。
> 详细修订记录见 §12。

> **V2.2 相比 V2.1 的变更（2026-09-24，R07 审查缺陷）**
>
> 只改一处：**§5.16 与 §9.2 中 E（`predictedError`）的算式补回弧度→角分换算因子**，
> 并把 §5.16 的"单位修正"论证由**错误的**"该式的输出就是角度"改为"该式输出为**弧度**"。
> §5.16 的字段表、§9.2 的其余结论、以及本文档其余全部章节**一字未改**。
> ⚠ C-14 当年把 E 的单位由 pixel 改判为角分，**结论是对的，但论证是错的** ——
> 它改的是**标签**，没有改**算式**，所以"标签对而数值错 3437.75 倍"。
> 配套修改见 ENG-10 V2.2 §2.1 / §3.2。详细修订记录见 §12。

### 1.1 权威性规则

1. 本文档与任何其他设计文档冲突时，**以本文档为准**；
2. 新增或修改任何跨模块类型，**必须先改本文档**，再改代码与其他文档；
3. SYS-05 保留"数据流与数据关系说明"职能；其 §3~§12 的类型定义**降级为历史版本**，以本文档 §5 取代；
4. 本文档冻结的字段名与枚举值，代码中不得就地改名；如需变更走 §8 流程。

---

## 2 命名与单位约定（冻结）

### 2.1 坐标变换方向

**冻结规则：**

变换量一律命名为 `<A>To<B>`，语义为**"把在 A 系中表达的坐标转换到 B 系"**。

即：

    p_B = AToB * p_A

**示例：**

    p_rig     = cameraToRig   * p_camera
    p_ship    = rigToShip     * p_rig
    p_camera  = aircraftToCamera * p_aircraft

**为什么必须冻结：**

SYS-13 §5 给出的坐标链为：

    T_ship^aircraft = T_ship^rig · T_rig^camera · T_camera^aircraft

按 SYS-13 自身约定（`T_X_Y` 表示 Y→X）该式自洽，合成结果方向为 **aircraft→ship**。但 SYS-05 §11.2 把同一结果字段命名为 `shipToAircraft`（ship→aircraft），§11.1 把 PnP 直接输出命名为 `cameraToAircraft`（camera→aircraft）。**两者互为逆变换，字段名与坐标链方向相反。**

本表裁决：保留 SYS-13 的链方向，**修正 SYS-05 的两个字段名**：

| 原字段名 | 冻结字段名 | 所在类型 |
|---|---|---|
| `cameraToAircraft` | `aircraftToCamera` | `CameraPose` |
| `shipToAircraft` | `aircraftToShip` | `ShipPoseResult` |

`cameraToRig`（`CameraCalibration`）与 `rigToShip`（`OpticalRigCalibration`）方向正确，**保持不变**。

### 2.2 角度单位

**冻结：所有角度字段一律为「度」（degree），非弧度。**

包括：

- `ShipPoseResult.yaw` / `pitch` / `roll`
- `TurntableState.azimuth` / `elevation`
- `TurntableCommand.azimuthCommand` / `elevationCommand`
- `TargetOffset.threshold` 相关配置项

换算基准：

    1 角分 = 1/60 度 = 0.0166667 度
    1 度 = 60 角分

**理由：** 验收指标以角分表述（Yaw ≤ 1 角分），转台指标以度表述（±0.2°）。若内部使用弧度，`if (yaw <= 1.0)` 这类比较极易误判"1 弧度"与"1 角分"。

**约定：** 头文件中每个角度字段必须带 `// deg` 注释。若后续需要弧度参与数学运算，在算法内部局部转换（`deg * CV_PI / 180.0`），不得改变结构体字段单位。

### 2.3 长度与距离单位

**冻结：所有长度、平移、距离一律为「米」（m）。**

包括：

- `Transform.translation`
- `TargetScaleEstimate.distance`
- `ModelPoint3D.position`
- `CameraChannel.focalLength`（**注意：为米，100mm 镜头写作 0.1**）

**理由：** 舰载场景距离量级为 40~300m，若平移向量用毫米、距离用米，同一条代码路径上会出现 1000 倍混用。统一为米可让 `distance` 与 `translation` 直接比较。

**装载转换：** 若使用 OpenCV 工具链生成的标定文件（平移单位为毫米），转换必须在 `CalibrationManager::load()` 内一次性完成，**不得泄漏到算法层**。

焦距字段特别提示：`focalLength` 单位是米而工程语言习惯说"100mm 镜头"，是最容易出错的一处，代码中必须注释。

### 2.4 像素坐标

`TargetOffset.pixelX` / `pixelY`：

- **语义**：目标中心相对**图像中心**的带符号像素偏差；
- **符号**：X 向右为正，Y 向下为正（与 OpenCV 图像坐标一致）；
- **单位**：pixel，`double`。

**理由：** SYS-10 §7 的停止判据 `abs(pixelX) <= threshold` 只有在"相对图像中心"时才有物理意义。若定义为"相对目标框中心"，该判据恒为 0。

`cv::Rect bbox`（`DetectionResult`）保持 OpenCV 约定：左上角原点，X 向右，Y 向下，单位像素。

### 2.5 时间戳

系统内**存在两个时间戳，语义不同，不得混用**：

| 字段 | 时钟源 | 用途 |
|---|---|---|
| `ImageFrame.timestampNs` | 主机单调时钟 `CLOCK_MONOTONIC`，采集线程在 `grab()` 返回后立即打点 | 同步校验、时延统计的唯一基准 |
| `ImageFrame.deviceTimestampNs` | 相机 SDK 上报的相机内部时间戳（无则置 0） | 仅用于诊断跨相机时钟偏移 |

`MultiCameraFrame.triggerTimestamp`：主机单调时钟纳秒，取该组三帧中 `timestampNs` 的最大值。

**禁止使用墙钟时间（`CLOCK_REALTIME`）做同步判断**——系统需长时间运行且支持离线部署，墙钟可能被 NTP 或人工校时跳变。

**V2.3 补充（011-A1 数据契约批，裁决 C-015）**：SDK 回报的帧时间戳（`IMV_FrameInfo::timeStamp`）
**只进** `deviceTimestampNs` 与诊断字段，**不得**替换主机单调时钟 ——
即 `timestampNs` 一律是**后端在 `grab()` 返回后自己打的主机单调钟点**，
与"设备说了什么时间"无关。⚠ 两者的**来源不同**这一点必须写进代码注释：
`timestampNs` 由**主机**生成，`deviceTimestampNs` 由**设备**生成（未取得则 0），
二者不可互相替代、不可比较。

**V2.4 补充（011-A1 九项缺口 §3，裁决 C-01 v1.8）："无期限"与"已到期"必须用两个不同的值表示。**

`data::kNoDeadlineNs`（与 `data::monotonicNowNs()` **同一头文件**）＝ `UINT64_MAX`，
语义为"**没有期限**"，与 `RetryManager::kNoDeadline` **同值同义**（后者改为引用前者，
其原有注释与理由保留）。冻结规则：

1. **无期限不参与取小**：`min(单次上限, 组剩余, 距 deadline 剩余)` 里，
   取值为 `kNoDeadlineNs` 的那一项**不参与比较**（不得被当成一个极大的有限值去比，
   也不得被算成"已到期"）。
2. **除哨兵外，`deadline ≤ now` 一律是"已到期"**：剩余量记 **0**，
   `remainingNs(budget, elapsed)` 在到期后返回 0。**0 不是"无期限"**。
3. ⚠ **为什么必须分开**：控制器曾用 `deadline == 0` 表示"无期限"，同时又用
   `taskRemaining > 0` 排除"剩余为 0"。于是 `nowNs == 0`（或任何一次真实到期）
   会掉进"既没被当成无期限、也没被当成已到期"的缝里，被兜底逻辑**重新注入一份预算** ——
   过期任务因此获得第二次机会，而"取帧超时"这件事在证据上消失。
   ⇒ 控制器改为显式三分支（`taskRemaining == kNoDeadlineNs` ⇒ 无期限；
   `== 0` ⇒ **已到期，期限＝`nowNs`**；其余 ⇒ `nowNs + taskRemaining` 取小），
   **不再有任何 `deadline == 0` 哨兵**。`IMultiCameraManager.h` 的接口注释同步写明
   这两个取值的含义。
   ⚠ **实施实测（如实登记，供后续版本核对）**：`== 0` 分支在**当前控制器里可达**，
   **结论方向与本节初稿相反，以本段为准**。`MeasurementController::tick(nowNs)` 的入口判定读的是
   **形参**（`main.cpp` 传入的快照），而 `stepCapture()` 每一轮判停/取期限时**重新读一次注入时钟**
   （`MeasurementController::nowNs()`）；期限只要落在这两次读钟**之间**（入口 `T_task − ε` 通过、
   CAPTURE 首轮重读已是 `T_task + ε`、状态级期限尚未到），本次调用就会带着**已到期**的任务
   走进 `acquireDeadlineNs()` —— 若退回旧的"忽略 `taskRemaining == 0`"逻辑，
   这一拍会退化为状态预算／组预算、**重新获得一份预算**并发起本不该发起的采集。
   ∴ 该分支**不是防御性保留**，而是本版修复的缺陷本体，**已写"改前红、改后绿"的用例**：
   `MeasurementFlowTest.cpp` 的 `A1_44_同一拍内跨过任务期限时一次采集都不发起`
   ＋ 定向变异 **M18**（忠实复现修复前形态：停用 `== 0` 分支 **并**给第三分支加 `taskRemaining > 0 &&`）
   ⇒ 4 条断言**全部转红**。⚠ 其余四个 `acquireDeadlineNs()` 调用点传入的都是本拍形参、
   被入口判定挡住，**只有** `stepCapture()` 的逐轮重读这一条路可达。
   反向误判（"无期限"被算成已到期）**仍未被实测**：现有测试无一条构造出无期限任务
   （`kNoDeadlineNs` 只在 `RetryManager::beginTask` 的溢出分支产生），该方向只由**代码判据**守住
   （不合并分支、不用 `0` 作哨兵）。

### 2.6 枚举命名

所有枚举值使用 `SCREAMING_CASE`（与既有 `MeasurementState` 保持一致）。

枚举一律使用 `enum class`，禁止裸 `enum`。

### 2.7 通用规则

- 结构体一律为**值类型**（POD 语义），不含业务方法与设备指针；
- 矩阵字段使用**固定尺寸类型**（`cv::Matx33d` 而非 `cv::Mat`），避免运行时尺寸错误；
- 标识字符串统一 `std::string`，禁止 `char*`；
- 跨线程传递的图像与帧数据用 `shared_ptr<const T>`（见 SYS-09 §14）。

---

## 3 模块归属规则（冻结）

### 3.1 规则

- **R1**：仅由数据组成的值类型、以及被两个及以上层引用的枚举，一律置于 `src/data/`；
- **R2**：`data` 不依赖任何其他模块，**包含不依赖 Qt**；仅允许依赖 OpenCV 基础类型（`cv::Mat`、`cv::Matx33d`、`cv::Vec3d`、`cv::Point2f/3f`、`cv::Rect`）；
- **R3**：接口类（`I*`）与实现类置于其所属层；算法层不得包含设备层头文件；
- **R4**：配置结构体（`*Config`）置于 `src/data/`，由 `infrastructure/ConfigManager` 负责装载填充，通过构造函数注入到使用方。

### 3.2 `CameraRole` 归属裁决（本次最重要的归属修正）

**裁决：`CameraRole` 下沉到 `src/data/CameraRole.h`。**

原规划为 `src/optical/CameraRole.h`（ENG-01 §6、ENG-02 §5.2），与依赖图冲突：

- `ImageFrame`（`src/data/`）含 `CameraRole role` 成员 → **data 依赖 optical**；
- `DetectionResult`（`src/data/`）含 `CameraRole sourceCamera` → 同上；
- `MeasurementCandidate`（`src/data/`）含 `CameraRole camera` → 同上；
- `MeasurementSelector`（algorithm）需要使用该枚举，而 algorithm 按 ENG-03 §12.5 只依赖 `data + OpenCV`，**不依赖 optical** → 该枚举不可达。

即：若 `CameraRole` 留在 optical，`data` 与 `algorithm` 两个模块**均无法编译**。

同理下沉的类型：`Transform`、`CoordinateFrame`、`MeasurementState`、`DeviceState`、`TurntableMotionState`。

### 3.3 `OpticalRig` 层级归属裁决

**裁决：`OpticalRig` 属于 `optical` 层，不是 Device 层设备抽象。**

SYS-06 把 `OpticalRig` 写在 Device 章节（§4），但其规划路径为 `optical/OpticalRig.h`，且 ENG-01 §17 / ENG-03 §2.2 的依赖图中 `optical` 位于 `device` 之下（device 依赖 optical）。两者不能同时成立。

冻结语义：

- `OpticalRig`（optical 层）= **静态描述**：三相机通道注册表 + 光机标定数据 + 查询接口，**不含任何设备控制能力**；
- 相机控制由 `device/camera/MultiCameraManager` 负责，它**依赖** `OpticalRig` 获取通道与标定信息；
- SYS-06 §4 的表述修订为"光学标定数据在 Device 层被消费"，`OpticalRig` 本身不在 device 层。

---

## 4 类型清单（按层，冻结）

### 4.1 `src/data/` — 值类型与共用枚举

| 文件 | 类型 | 说明 |
|---|---|---|
| `CameraRole.h` | `enum class CameraRole` | `CAM25` / `CAM50` / `CAM100` |
| `CoordinateFrame.h` | `enum class CoordinateFrame` | `CAMERA` / `OPTICAL_RIG` / `SHIP` / `AIRCRAFT` |
| `MeasurementState.h` | `enum class MeasurementState` | 12 个状态，见 §5.8 |
| `DeviceState.h` | `enum class DeviceState` | `UNKNOWN` / `INIT` / `READY` / `RUNNING` / `ERROR` / **`DISCONNECTED`**（V2.3 新增，见 §5.33） |
| `TurntableMotionState.h` | `enum class TurntableMotionState` | `IDLE` / `MOVING` / `STABLE` / `ERROR` |
| `Transform.h` | `struct Transform` | 见 §5.1 |
| `CameraCalibration.h` | `struct CameraCalibration` | 见 §5.2 |
| `OpticalRigCalibration.h` | `struct OpticalRigCalibration` | 见 §5.3 |
| `CameraChannel.h` | `struct CameraChannel` | 见 §5.4 |
| `ImageFrame.h` | `struct ImageFrame` | 见 §5.5 |
| `MultiCameraFrame.h` | `struct MultiCameraFrame` | 见 §5.6 |
| `PreviewFrame.h` | `struct PreviewFrame` | 见 §5.7 |
| `TurntableState.h` | `struct TurntableState` | 见 §5.9 |
| `TurntableCommand.h` | `struct TurntableCommand` | 见 §5.10 |
| `TargetOffset.h` | `struct TargetOffset` | 见 §5.11 |
| `DetectionResult.h` | `struct DetectionResult` | 见 §5.12 |
| `AlignmentResult.h` | `struct AlignmentResult` | 见 §5.13 |
| `TargetScaleEstimate.h` | `struct TargetScaleEstimate` | 见 §5.14 |
| `ImageQuality.h` | `struct ImageQuality` | 见 §5.15 |
| `MeasurementCandidate.h` | `struct MeasurementCandidate` | 见 §5.16 |
| `MeasurementSelectionResult.h` | `struct MeasurementSelectionResult` | 见 §5.17 |
| `ModelPoint3D.h` | `struct ModelPoint3D` | 见 §5.18 |
| `FeatureDescriptor.h` | `struct FeatureDescriptor` | 见 §5.19 |
| `TargetModel.h` | `struct TargetModel` | 见 §5.20 |
| `FeatureSet.h` | `struct FeatureSet` | 见 §5.21（**新增**，原未定义） |
| `FeatureCorrespondence.h` | `struct FeatureCorrespondence` | 见 §5.22 |
| `CameraPose.h` | `struct CameraPose` | 见 §5.23 |
| `ShipPoseResult.h` | `struct ShipPoseResult` | 见 §5.24 |
| `PoseValidationResult.h` | `struct PoseValidationResult` | 见 §5.25 |
| `MeasurementTask.h` | `struct MeasurementTask` | 见 §5.26 |
| `ErrorInfo.h` | `struct ErrorInfo` | 见 §5.27（**新增**，原未定义） |
| `PixelFormat.h` | `enum class PixelFormat`／`Packing`／`ByteOrder`／`BitAlignment` | 见 §5.28（**V2.3 新增**） |
| `RawImagePayload.h` | `struct RawImagePayload`／`enum class RawDataPolicy` | 见 §5.28（**V2.3 新增**） |
| `OpStatus.h` | `enum class OpStatus`／`SdkCall`、`struct SdkFailure`／`OperationResult`／`GrabResult` | 见 §5.29（**V2.3 新增**） |
| `DeviceIdentity.h` | `struct DeviceIdentity` | 见 §5.30（**V2.3 新增**） |
| `CameraTriggerMode.h` | `enum class CameraTriggerMode`／`struct TriggerModeState` | 见 §5.31（**V2.3 新增**） |
| `CaptureRound.h` | `struct ChannelGrabRecord`／`struct CaptureRound` | 见 §5.32（**V2.3 新增**） |
| `CameraConfig.h` 等 | `struct *Config` | 见 §6 |

**从 `data/` 移除：**

- `ShipFrame` — 与 `OpticalRigCalibration.rigToShip` 语义重复，**删除**。坐标系标识由 `CoordinateFrame` 枚举承担，外参由 `CalibrationManager` 承担。
- `PoseResult` — ENG-01 §5 曾列出 `PoseResult.h`，但全套文档未定义该类型，与 `ShipPoseResult` 重复，**删除**。

### 4.2 `src/optical/`

| 文件 | 类型 | 职责 |
|---|---|---|
| `OpticalRig.h` | `class OpticalRig` | 三相机通道注册表 + 标定持有，只读查询 |
| `CalibrationManager.h` | `class CalibrationManager` | 标定装载、版本校验、单位转换 |
| `CoordinateTransformer.h` | `class CoordinateTransformer` | Camera → Rig → Ship 坐标转换 |
| `CameraSynchronizer.h` | `class CameraSynchronizer` | 三帧同步组合为 `MultiCameraFrame` |

### 4.3 `src/device/`

| 文件 | 类型 |
|---|---|
| `camera/MultiCameraManager.h` | `class MultiCameraManager`（实现 `IMultiCameraManager`） |
| `camera/IMultiCameraManager.h` | `class IMultiCameraManager` |
| `camera/ICameraBackend.h` | `class ICameraBackend` |
| `camera/ImvCameraBackend.h` | `class ImvCameraBackend` |
| `camera/VirtualCameraBackend.h` | `class VirtualCameraBackend` |
| `turntable/ITurntableController.h` | `class ITurntableController` |
| `turntable/PekoTurntableController.h` | `class PekoTurntableController` |
| `turntable/VirtualTurntable.h` | `class VirtualTurntable` |
| `trigger/ITriggerController.h` | `class ITriggerController` |
| `trigger/HardwareTriggerController.h` | `class HardwareTriggerController` |
| `trigger/VirtualTriggerController.h` | `class VirtualTriggerController` |

### 4.4 其余层

`preview/`、`application/`、`algorithm/`、`infrastructure/`、`runtime/`、`ui/`、`app/` 的类清单沿用 ENG-01 §7~§14，**唯一变更**为 `algorithm/selection/MeasurementSelector` 需从 `data` 引入 `CameraRole` 与 `MeasurementCandidate`。

---

## 5 类型定义（冻结）

所有 `struct` 为定义即冻结，字段顺序即为内存顺序，禁止增删改字段名。

### 5.1 `Transform`

    struct Transform
    {
        cv::Matx33d rotation;     // 3x3 正交旋转矩阵，右手系
        cv::Vec3d   translation;  // m
    };

命名语义见 §2.1。`rotation` 使用固定尺寸类型，避免 `cv::Mat` 运行时尺寸不确定。

### 5.2 `CameraCalibration`

    struct CameraCalibration
    {
        cv::Mat      cameraMatrix;  // 3x3 CV_64F 内参
        cv::Mat      distortion;    // 1xN CV_64F 畸变系数
        Transform    cameraToRig;   // Camera → OpticalRig
        int          imageWidth;
        int          imageHeight;
    };

**裁决（C-02）：** SYS-11 §4.1 缺 `imageWidth` / `imageHeight`，**以 SYS-05 §4.2 为准，保留**。像素偏差归一化与 PnP 均需要。

### 5.3 `OpticalRigCalibration`

    struct OpticalRigCalibration
    {
        CameraCalibration cam25;
        CameraCalibration cam50;
        CameraCalibration cam100;
        Transform         rigToShip;   // OpticalRig → Ship
    };

### 5.4 `CameraChannel`

    struct CameraChannel
    {
        std::string       cameraId;
        CameraRole        role;
        double            focalLength;  // m，100mm 镜头 = 0.1
        CameraCalibration calibration;
        bool              enabled;
    };

**裁决（C-03）：** ENG-02 §5.3 缺 `enabled`，**以 SYS-05 §4.4 为准，保留**。三相机支持单独禁用（调试与降级运行需要）。

### 5.5 `ImageFrame`

    struct ImageFrame
    {
        cv::Mat     image;
        uint64_t    frameId;
        uint64_t    timestampNs;        // 主机 CLOCK_MONOTONIC，ns
        uint64_t    deviceTimestampNs;  // 相机内部时间戳，ns；不可用则 0
        std::string cameraId;
        CameraRole  role;
        double      exposureTime;       // s
        // ---- V2.3 新增（011-A1 数据契约批，裁决 C-015）----
        RawImagePayload raw;            // 相机交付的原始字节及其解释信息，见 §5.28
        PixelFormat     captureFormat;  // 本帧**采集格式**（独立于 raw 是否为空）
        RawDataPolicy   rawPolicy;      // 原始载荷的必要性策略（独立于 raw 是否为空）
    };

**V2.3 变更（裁决 C-015）：** 新增 `raw`／`captureFormat`／`rawPolicy` **三个**字段，
其余七个字段的语义**一字未改**。三条冻结规则：

1. **`captureFormat` 与 `rawPolicy` 必须独立于 `raw` 存在** —— 它们回答的是
   "**这一帧本来应该带什么**"，不是"**它实际带了什么**"。若从 `raw` 反推
   （`raw` 空就当 8 位处理），则"真实 12 位帧丢了载荷"与"虚拟 8U 帧本来就没有载荷"
   在数据上**完全同形**，而前者必须**报错**、后者**照常保存**。
2. **`image` 的语义边界收紧**：`width`／`height`／`image.type()` **只描述 8U 显示图**，
   **永远不作为裸缓冲的解码依据**；裸缓冲的解码依据**只有** `raw` 自身的
   `format`／`validBits`／`packing`／`declaredByteOrder`（§5.28）。
3. **`image` 与 `raw` 都不得悬挂在 SDK 缓冲区上**：两者都必须在 `IMV_ReleaseFrame`
   之前完成复制；`RawImagePayload::bytes` 是**自有副本**（§5.28 第 5 条）。
   ⚠ 不得在注释里写"`shared_ptr` 使浅拷贝天然安全"——安全来自
   **自有 + 生命周期正确 + 发布后不变**三件事，不是来自智能指针本身。

**裁决（C-01）：** 该类型在三份文档中字段不一致：

| 出处 | 字段 |
|---|---|
| SYS-05 §5.1 | image, frameId, timestampNs, cameraId, role, exposureTime |
| ENG-02 §4.1 | image, frameId, timestampNs, cameraId, role |
| ENG-04 §4.1 | image, frameId, timestampNs, role |

冻结为上表并集，**新增 `deviceTimestampNs`**（见 §2.5）。

### 5.6 `MultiCameraFrame`

    struct MultiCameraFrame
    {
        ImageFrame cam25;
        ImageFrame cam50;
        ImageFrame cam100;
        uint64_t   triggerTimestamp;  // 主机 CLOCK_MONOTONIC，ns
    };

三处文档定义一致，直接沿用。

### 5.7 `PreviewFrame`

    struct PreviewFrame
    {
        ImageFrame frame;
        uint64_t   displayTimestamp;
    };

### 5.8 `MeasurementState`

    enum class MeasurementState
    {
        IDLE,
        SEARCH,
        TARGET_FOUND,
        ALIGN,
        STABILIZE,
        MEASURE_SELECT,
        CAPTURE,
        POSE_SOLVE,
        VALIDATE,
        SAVE,
        COMPLETE,
        FAILED
    };

**裁决（C-13）：** SYS-05 §12 在 `MeasurementTask` 中使用该类型，但定义在 SYS-08 §4。冻结归属 `src/data/MeasurementState.h`，状态语义与转换规则仍由 SYS-08 定义。

### 5.9 `TurntableState` 与 `TurntableMotionState`

    enum class TurntableMotionState
    {
        IDLE,
        MOVING,
        STABLE,
        ERROR
    };

    struct TurntableState
    {
        double               azimuth;    // deg
        double               elevation;  // deg
        TurntableMotionState motion;
    };

**裁决（C-04，本次最严重的命名冲突）：** 同一版本中 `TurntableState` 被定义为两个互不相容的类型：

- SYS-05 §6.1：`struct TurntableState { azimuth; elevation; moving; }`
- SYS-10 §10：`enum class TurntableState { IDLE; MOVING; STABLE; ERROR; }`

而 `ITurntableController::state()` 的返回类型正是它（SYS-06 §10.1）。两者不可能同名。

冻结方案：

- **结构体保留名称 `TurntableState`**（它是跨模块传递的数据）；
- **枚举更名为 `TurntableMotionState`**；
- 结构体中的 `bool moving` **删除**，由 `motion` 承担（`moving` 与 `motion != IDLE && motion != STABLE` 完全等价，保留两者会产生不一致状态）；
- 枚举补充 `ERROR`，使转台错误可随状态一起回传，无需独立错误通道。

### 5.10 `TurntableCommand`

    struct TurntableCommand
    {
        double azimuthCommand;    // deg
        double elevationCommand;  // deg
    };

### 5.11 `TargetOffset`

    struct TargetOffset
    {
        double pixelX;    // 相对图像中心，右为正
        double pixelY;    // 相对图像中心，下为正
        bool   centered;
    };

**裁决（C-11）：** SYS-05 §6.3 与 SYS-10 §7 字段一致，但**均未定义参考原点**。冻结为"相对图像中心"，理由见 §2.4。`centered` 由 `abs(pixelX) <= threshold && abs(pixelY) <= threshold` 计算，`threshold` 来自 `TurntableConfig`，默认 50。

### 5.12 `DetectionResult`

    struct DetectionResult
    {
        bool       found;
        cv::Rect   bbox;          // 左上原点，像素
        double     confidence;
        CameraRole sourceCamera;
    };

### 5.13 `AlignmentResult`

    struct AlignmentResult
    {
        bool         success;
        TargetOffset offset;
        int          iterations;
        double       finalErrorPixel;
    };

### 5.14 `TargetScaleEstimate`

    struct TargetScaleEstimate
    {
        double distance;        // m，仅用于通道选择
        double targetPixelSize; // pixel
        double confidence;
    };

该值**不作为最终输出**（SYS-02 FR-004 约束），只允许被 `MeasurementSelector` 消费。

### 5.15 `ImageQuality`

    struct ImageQuality
    {
        double sharpness;     // 清晰度（Laplacian 方差或梯度能量）
        double exposure;      // 曝光合理性 [0,1]，1 = 最佳
        double contrast;      // 对比度 [0,1]
        int    featureCount;
        int    matchCount;
        double matchRatio;    // 内点/总数，[0,1]
    };

**裁决（C-12）：** SYS-05 §9.1 只有 `sharpness`，但 SYS-07 §6.2 与 SYS-14 §7 的评分模型把"图像质量 Q"定义为**清晰度 + 曝光 + 对比度**三项，字段缺失导致 Q 无法计算。冻结补入 `exposure` 与 `contrast`。

### 5.16 `MeasurementCandidate`

    struct MeasurementCandidate
    {
        CameraRole          camera;
        TargetScaleEstimate scale;
        ImageQuality        quality;

        // ---- 评分四个分项（均为归一化后的值，ENG-10 §3.6）----
        double qNorm;              // Q，[0,1]
        double fNorm;              // F，[0,1]
        double mNorm;              // M = M_hist，[0,1]
        double eNorm;              // E，[0,1]

        // ---- E 的三个输入（ENG-10 §3.2）----
        double featureSpreadPx;    // W，特征点展布宽度，pixel
        int    nDetect;            // 本帧提取到的关键点数
        double nEst;               // M_hist × nDetect
        double sigmaPxEst;         // 预测的单点定位标准差，pixel

        double predictedError;     // E，角分（**单位已修正**，见下）
        bool   predictedErrorCalibrated;  // a,b,c 是否已标定（ENG-10 §3.4）

        double score;
    };

**裁决（C-14）：** SYS-07 §6.2 的评分式为 `Score = w1*Q + w2*F + w3*M - w4*E`，但 SYS-05 §9.2 的 `MeasurementCandidate` **没有 E 的载体字段**。冻结补入 `predictedError`。

> ⚠ **V2.2 勘误（不改上述裁决记录）：** 该处所引 **"SYS-05 §9.2" 节号有误**。
> SYS-05 的 §9 是"目标模型数据"，**只有 §9.1，没有 §9.2**；
> `MeasurementCandidate` 实际在 **SYS-05 §12.1**。
> C-14 的裁决文本按"历史引用不改号"的纪律**原样保留**，仅在此加注。
> 另注：SYS-05 §12.1 的类型定义已按 §1.1 第 3 条降级为历史版本，不影响本条冻结。

**单位修正（原注释为 `// 单位 pixel`，错误）：** E 是**角度**量（最终要与之比较的是 Yaw 误差 1 角分），冻结为**角分（arcminute）**。以 pixel 为单位的量是它的输入 `sigmaPxEst`，两者不可混用。

⚠ **V2.2 修正 —— 原论证是错的，但结论对（R07）：** C-14 当年写的理由是
"其计算式 `σ_θ = σ_px·√12/(W·√N)` **的输出就是角度**"。这句话**不成立**：
`σ_px / W` 是一个**比值**（分子分母同为 pixel），故该式输出的是**弧度**，不是角度、
更不是角分。C-14 因此**只改了标签、没改算式** —— 于是"标签写着角分、数值是弧度"，
`predictedError` 被低估 **3437.7468 倍**（`180/π×60`）。

正确表述为：

    σ_θ[弧度] = σ_px · √12 / ( W · √N )
    E[角分]   = σ_θ[弧度] × 3437.7468

**C-14 的结论（E 的单位是角分）依然正确**，因为紧随其后的 ENG-10 §2.1 表格
**自始就按角分给出数值**且一直正确 —— 错的是算式与它的论证。详见 ENG-10 V2.2 §2.1 / §3.2。

**四分项必须分别保留（新增）：** 原定义只留 `score`，导致"选择逻辑可解释"（SYS-14 §20 约束 3）无法兑现。**`score` 是加权和，不可逆推回分项**，故四个归一化分项与 `weights` 都必须记录，见 SYS-14 §19。

**新增字段来源：** `qNorm/fNorm/mNorm/eNorm`、`featureSpreadPx`、`nDetect`、`nEst`、`sigmaPxEst`、`predictedErrorCalibrated` 均引自 **ENG-10 §3**。

`predictedError` 的计算方法已冻结于 **ENG-10 §3.2**；历史匹配成功率的存储与生命周期冻结于 **ENG-10 §4**（原 §9 待办至此结项）。

### 5.17 `MeasurementSelectionResult`

    struct MeasurementSelectionResult
    {
        CameraRole selectedCamera;
        double     score;
    };

### 5.18 `ModelPoint3D`

    struct ModelPoint3D
    {
        int          id;
        cv::Point3f  position;      // m，Aircraft Frame
        std::string  featureType;   // "cad" | "texture"
    };

### 5.19 `FeatureDescriptor`

    struct FeatureDescriptor
    {
        int     featureId;
        int     point3dIndex;   // 索引 TargetModel::points3d
        cv::Mat descriptor;     // 1 x D，CV_32F
    };

### 5.20 `TargetModel`

    struct TargetModel
    {
        std::string                 modelId;
        std::vector<ModelPoint3D>   points3d;
        std::vector<FeatureDescriptor> features;
    };

**裁决（C-08）：** SYS-05 §10.1 与 SYS-12 §5.2/§9 对同一概念给出两套互斥建模：

| 出处 | 建模 |
|---|---|
| SYS-05 §10.1 | `points3d: vector<cv::Point3f>` + `descriptors: cv::Mat`（扁平，无关联） |
| SYS-12 §5.2/§9 | `ModelPoint3D` 结构 + `FeatureDescriptor` 结构（显式关联） |

**以 SYS-12 为准。** SYS-05 的扁平方案丢失了"哪个描述子对应哪个三维点"的关联，而该关联正是 PnP 2D-3D 对应的唯一依据，不可用。

冻结为 `FeatureDescriptor.point3dIndex` + `TargetModel.points3d` 的索引关联，同时消除 `FeatureDescriptor` 内嵌 `point3d` 与 `ModelPoint3D.position` 的重复存储。

### 5.21 `FeatureSet`

    struct FeatureSet
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat                   descriptors;   // N x D，CV_32F
    };

**裁决（C-09）：** SYS-07 §8 声明 `FeatureExtraction` 输出 `FeatureSet`，但全套文档从未定义该类型。冻结补定义。

### 5.22 `FeatureCorrespondence`

    struct FeatureCorrespondence
    {
        cv::Point3f objectPoint;  // m，Aircraft Frame
        cv::Point2f imagePoint;   // pixel
    };

### 5.23 `CameraPose`

    struct CameraPose
    {
        Transform aircraftToCamera;   // 见 §2.1 方向裁决
        double    reprojectionError;  // pixel
    };

### 5.24 `ShipPoseResult`

    struct ShipPoseResult
    {
        bool      success;
        double    yaw;                // deg
        double    pitch;              // deg
        double    roll;               // deg
        Transform aircraftToShip;     // 见 §2.1 方向裁决
        double    reprojectionError;  // pixel
    };

`yaw` 定义：Aircraft Frame 相对 Ship Frame 绕 `Z_ship` 轴的旋转角（SYS-13 §6），ZYX 欧拉角顺序，Yaw → Pitch → Roll。

### 5.25 `PoseValidationResult`

    struct PoseValidationResult
    {
        bool   valid;
        double reprojectionError;  // pixel
        double inlierRatio;        // [0,1]
        double confidence;         // [0,1]
    };

### 5.26 `MeasurementTask`

    struct MeasurementTask
    {
        std::string           taskId;
        MeasurementState      state;
        ShipPoseResult        result;
        PoseValidationResult  validation;
    };

### 5.27 `ErrorInfo`

    struct ErrorInfo
    {
        int         code;
        std::string message;
        uint64_t    timestampNs;   // 主机 CLOCK_MONOTONIC，ns
    };

**裁决（C-09）：** SYS-06 §15 的相机异常、转台异常、触发异常三条链路都提到 `ErrorInfo`，但从未定义。冻结补定义，作为设备层向上层回报的统一错误载体。

**错误码分段（冻结）：**

| 段 | 归属 |
|---|---|
| 1000~1999 | 相机设备 |
| 2000~2999 | 转台设备 |
| 3000~3999 | 触发 / 同步 |
| 4000~4999 | 标定 |
| 5000~5999 | 模型与特征库 |
| 6000~6999 | 算法 |
| 9000~9999 | 系统级 |

**已占用的具体错误码（冻结）：**

| 码 | 含义 | 来源 |
|---|---|---|
| 1001 | 可用相机数不足（<2），无法继续 | ~~SYS-08 §7.5~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 1002 | 相机断连（已降级） | ~~SYS-08 §7.5~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 1003 | 相机取帧超时或无有效帧 | **裁决 C-015**（011-A1，2026-09-26）；语义见 §5.29 |
| 1004 | 相机通道未就绪或未启动 | **裁决 C-015**；语义见 §5.29 |
| 1005 | 相机 SDK 错误或帧不可用 | **裁决 C-015**；语义见 §5.29。⚠ 与 1006 **不得混用** |
| 1006 | 取帧契约错误或参数非法（**本地判定**；⚠ **不是**"未调用 SDK"，见 §5.29） | **裁决 C-015**；语义见 §5.29 |
| 1007 | 请求的相机格式/模式本批未实现 | **裁决 C-015**；语义见 §5.29 |
| 2001 | 转台通信失败 | ~~SYS-08 §7.7~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 2002 | 转台超出行程（能力边界） | ~~SYS-08 §7.7~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 2003 | 对准重试次数用尽 | ~~SYS-08 §7.3 / §7.7~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 3001 | 触发失效，已降级为软触发 | ~~SYS-08 §7.5~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 3002 | 三路时间戳同步超差（可重试的瞬态） | **裁决 C-006**（2026-09-23）—— V2.3 **回填**，见下 |
| 4001 | 标定数据缺失或无效，对准无法进行 | **裁决 C-006** —— V2.3 **回填**；本码是该段第一个 |
| 5001 | 机型模型/特征库缺失，无法解算 | **裁决 C-006** —— V2.3 **回填** |
| 6001 | PnP 重试次数用尽 | ~~SYS-08 §7.3~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 9001 | **单次测量任务超出 `T_task`** | ~~SYS-08 §7.1~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 9002 | **回退预算用尽** | ~~SYS-08 §7.4~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 9003 | 人工取消 | ~~SYS-08 §7.3（SEARCH 可由人工终止）~~ ⇒ **引用无效，依据待裁决（Q-D2）** |
| 9004 | 状态级失败（**兜底码**，全表唯一） | **裁决 C-006** —— V2.3 **回填**；使用纪律见代码注释 |
| 9005 | 系统配置加载失败（装配期） | **裁决 C-006** —— V2.3 **回填** |

**要求：** 状态机进入 FAILED 时，`ErrorInfo.code` 必须取自本表，**不得使用裸数字字面量**；代码中以具名常量（如 `kErrTaskTimeout = 9001`）引用。

> ⚠ **V2.3 对"来源"列的更正（2026-09-26 复核）**：本表原以 `SYS-08 §7.1`～`§7.7` 为来源，经核实
> **该节号族在冻结文档中不成立** —— `SYS-08 V2.1` 的 `# 7` 是 **TARGET_FOUND 状态**
> （§7.1 状态说明／§7.2 执行动作／§7.3 输出／§7.4 失败处理），与本工程引用的
> "§7.1 三级超时／§7.2 失败三分类／§7.3 次数表／§7.4 回退预算"**内容完全不同**（**节号碰撞**）；
> `§7.5`／`§7.6`／`§7.7` **在这份文档里根本不存在**（全文只到 `# 22`，每个 `# N` 最多到 `.4`）。
> ∴ 上表把每一处 `§7.x` 来源**划掉并就地标注为无效**，**依据待裁决**（登记于
> 《待裁决问题汇总》**Q-D2**；逐条勘误见工程根目录的《SYS-08-§7引用勘误.md》）。
> ⚠ **各码的"含义"与"号段"本身不变，代码无需改动** —— 失效的只是"来源"一栏所声称的依据。
> 其中 1001／1002 还牵涉一条**独立**的冲突（现行"2 路可用 ⇒ 降级继续"与 `SYS-08 §17.1`
> "断连／无帧／超时**一律 FAILED**"相反），登记为 **Q-D1**，**不得标为已批准**。
> ⚠ **不声称"旧版另有条文"**：`项目文档/` 内 `SYS-08` **只有这一份 V2.1**，`archive/` 下没有旧版
> —— 被引用的那套内容在冻结文档里**从来没有过**，其真实来源是**实施期自造的工程约定**。

> ⚠ **V2.3 回填 5 个已落地但未登记的错误码（3002／4001／5001／9004／9005）**：
> 这五个码由**裁决 C-01 v1.3**（2026-09-23，来源为全仓静态审查报告 R07 系列）新增、
> **代码中已全部落地**，但**当时未回填本表** —— 于是本表与代码长期不一致，
> 而 §1.1 第 2 条的流程要求恰恰是"**先改本文档**，再改代码"。
> 本版**按代码实际值回填**（含义照录代码注释，逐条在 §12.1 登记），
> 并**不追溯**该流程偏离的责任归属。⚠ 回填是**补登记**，不是"新增"：
> 这五个码的语义在 C-01 v1.3 里已经定过。

---

### 5.28 `RawImagePayload` 及其配套枚举（V2.3 新增）

**类型定义（冻结）：**

    enum class PixelFormat   { Mono8, Mono12, Mono12Packed, BGR8 };
    enum class Packing       { Unpacked, Packed12 };
    enum class BitAlignment  { LsbZeroPadded, MsbAligned, Unknown };
    enum class ByteOrder     { LittleEndian, BigEndian };
    enum class RawDataPolicy { RawOptional, RawRequired };

    struct RawImagePayload
    {
        std::shared_ptr<const std::vector<uint8_t>> bytes;   // 自有字节，逐字节等于 SDK 交付缓冲
        PixelFormat    format;
        uint16_t       validBits;             // 8 / 12
        Packing        packing;
        BitAlignment   bitAlignment;
        uint32_t       width, height;         // 设备回报（不是配置里的期望值）
        uint64_t       sdkPayloadBytes;       // = IMV_FrameInfo::size，**唯一权威长度**
        uint64_t       expectedCompactBytes;  // 按紧凑契约算出的期望长度
        bool           compactSizeMatches;    // **只是校验条件**，见第 3 条命名纪律
        ByteOrder      declaredByteOrder;     // 适配层声明，见第 4 条
        int32_t        sdkPixelFormatCode;    // ─┬─ 以下三项**仅作诊断**：
        int32_t        sdkPaddingX;           //  │ 语义未经确认，**禁止**用于任何
        int32_t        sdkPaddingY;           // ─┘ 寻址或长度计算
    };

**1 合法组合表**（矛盾即契约错误，发布时断言 + Recorder 落盘时复检）：

| `PixelFormat` | 合法 `Packing` | `validBits` | 有效位对齐（**依据**） | 每像素字节 |
|---|---|---|---|---|
| `Mono8` | `Unpacked` | 8 | 8 位整字节（PFNC `0x01080001`） | 1 |
| `Mono12` | `Unpacked` | 12 | **低位对齐、高位补零**（PFNC 2.4 §6.1.1／图 6-3；SDK 值＝`0x01100005`） | 2（`OCCUPY16BIT`） |
| `Mono12Packed` | `Packed12` | 12 | `0x010C0006` 属 **GigE Vision 2.0**（PFNC 的 `Mono12p` 是 `0x010C0047`，**另一项**）；**本批按范围不做** | 1.5（`OCCUPY12BIT`） |
| `BGR8` | `Unpacked` | 8 | 三通道整字节（PFNC `0x02180015`） | 3 |

本批**支持** `Mono8`／`Mono12`／`BGR8`（`Mono12` 的**正常路径已实现**，不留空）；
`Mono12Packed` 返回 `NotImplemented`；**未知格式码 ⇒ 明确失败**（不回落、不猜测）。
`BitAlignment::MsbAligned`／`Unknown` 在本批**不可达**：出现即按契约错误拒绝 ——
那是"上游给了个本批不实现的声明"，**不是**"未知就失败"的运行期分支。
`ByteOrder::BigEndian` 本批**拒绝**（见第 4 条）。

**2 `rawPolicy` 与 `captureFormat` 的交叉一致**：`RawOptional` **仅**当 `captureFormat`
为 `Mono8`／`BGR8`（8 位）；`RawRequired` 用于**凡是有效位深 > 8** 的格式（本批即 `Mono12`）。
其它组合判契约错误。这条是从数据上把"虚拟 8U 帧丢载荷无妨"与"真实 12 位帧丢载荷必须报错"
**分开**的判据 —— 不靠文件名、逻辑 `cameraId` 或运行时重查配置去猜。

**3 布局：本批声明契约，长度等式只作校验**（⚠ `IMV_FrameInfo` **没有** `step`／`stride` 字段）：

- **支持契约**：`Mono8`／`Mono12`／`BGR8` 的载荷按 **1／2／3 字节每像素的紧凑布局**解释 ——
  **无行尾填充、无整帧填充**。这是**本批明确声明的契约**，**不是**从字节数"推"出来的结论，
  也**不是** SDK 的任何保证。
- **长度等式只是校验条件**：`sdkPayloadBytes == width × height × 每像素字节`
  （`sdkPayloadBytes` ＝ `IMV_FrameInfo::size`，**总长的唯一权威来源**）。
  该布尔量名为 `compactSizeMatches`，⚠ **不得**改成"布局已确认"之类的名字：
  一次字节数比较只能说明"**长度与紧凑契约相容**"，**证明不了像素怎么排列**
  （相同字节数对应多种排列是可能的）。把校验条件说成结论，会让下游以为
  "布局已被验证过"，从而放心地按一个未经验证的假设去寻址。
- **不匹配 ⇒ `CorruptFrame`，且失败即不发布帧**；把 `sdkPayloadBytes`／`expectedCompactBytes`／
  `sdkPixelFormatCode`／`sdkPaddingX`／`sdkPaddingY` 记进**错误记录与日志**（"长度不符"是可查事实）。
- **不派生 `strideBytes`／`paddingBytes`**：`sdkPaddingX`／`sdkPaddingY` 的"行内／整帧"语义
  **未文档化**（§13 附录 A），且 SDK 自身的 `Rotate`／`Flip`（按 `width*height*channels`）
  与 `PixelConvert`（收 `paddingX`）**约定不一致**（§14 附录 B）⇒
  它们**禁止**出现在任何寻址或长度计算里。

**4 字节序：适配层声明，依据与所采用的格式布局绑定**（**不写成"SDK 保证"**）：
本批依据＝① 本工程主机为 x86_64（UOS）；② SDK 样例把**8 位**格式的 `pData` 直接写进 BMP 体。
⚠ ② **只覆盖 8 位格式**，对 **16 位容器**（`Mono12` 正是这一类）**不构成证明** ——
BMP 体只吃 8 位字节流，它证明不了 16 位样本的组装顺序。
∴ 本批取 `LittleEndian` 作为**待实机核实的声明**（§14 附录 B 第 2 条）；核实方式＝同一帧
用自实现转换与 `IMV_PixelConvert` 出图比对。**两者不一致时，排查顺序是三件事，不是一件**：
先查（a）布局假设、（b）有效位对齐，最后才查（c）字节序声明 ——
**不得**直接认定"只是字节序声明错了"。代码上它是一个**单点可改的属性**，
不做"按探测结果静默转换"的逻辑。

**5 所有权与生命周期**：`bytes` 必须**真正拥有字节**；**归还 SDK 缓冲区前完成复制**；
`ImageFrame::image` **同样**不得悬挂在 SDK 缓冲区上；发布后不再修改共享数据。
**每帧一份独立缓冲**（`grab` 每次新建，**不做缓冲池复用**）—— 池复用会把 SDK 缓冲区的
复用问题搬进后端内部，而调用方仍可能持有引用。
⚠ 注释里**不得**写"`shared_ptr` 使浅拷贝天然安全"：安全来自
**自有 + 生命周期正确 + 发布后不变**三件事，不是来自智能指针本身。

**6 显示转换**（SYS-04 V2.6 记调用责任）：`8U = 像素值 >> (validBits − 8)`，
像素值来自**按第 1 条解释、且 `compactSizeMatches` 为真**的载荷；16 位容器按
`declaredByteOrder` 组装。**不缩放、不直方图拉伸、不饱和**。
⚠ `image.type()` **只描述 8U 显示图**，**永远不作为原始载荷的解码类型** ——
原始载荷的解码依据**只有** `format`／`validBits`／`packing`／`declaredByteOrder`（第 1／4 条）。
`captureFormat` 与后端**实际收到**的格式做一致性断言：不一致**即失败**，不是静默转换。
⚠ **本批不委派 `IMV_PixelConvert` 做 12 位转 8 位**（尽管 SDK 样例一律走它）：
它会把"缩放／对齐规则"藏进 SDK 内部，而我们**无法核实其是否缩放** ——
直接冲突于本条的"不缩放"。委派路径记为实机核实的**对照项**（§14 附录 B 第 2 条）。

**7 Recorder 读取规则（三分支）**：判断**只依据帧自身携带的字段**，不靠文件名、逻辑
`cameraId` 或运行时重查配置。

| 情况 | 行为 |
|---|---|
| `bytes` 非空且合法 | 保存 `bytes`，元数据描述**实际写出的字节布局** |
| `bytes` 为空，且 `rawPolicy == RawOptional`、`captureFormat` 为 8 位 | **可**保存 `image`，如实标注实际格式与 `data_source = "image"` |
| `bytes` 为空，但 `rawPolicy == RawRequired` 或 `captureFormat` 为 12 位 | **报契约／数据错误，拒绝回落**（禁止静默存 8 位图） |

⚠ **进入 Recorder 的帧一定是"已交付的成功帧"**：`CorruptFrame` 不发布帧
（第 3 条），∴ "长度不符的 RAW 进结果包"这条路**不存在**。坏帧字节若要留存，
属**独立诊断通道**，不在本批。

**7bis 落盘复检：四个长度事实互证（V2.4 新增，011-A1 九项缺口 §6）**

⚠ V2.3 写的"Recorder 复检"**名不副实**：它只读了**上游算好的两个字段**
（`compactSizeMatches` 与 `expectedCompactBytes`），**从不重算** ——
于是上游把期望长度算错（或布尔量填错）时，这里原样放行，**错误的元数据进了结果包**。
"不信任上游"若只体现为"抄上游的另一个字段"，就仍是在转录结论。V2.4 冻结为：

1. **重算**：`computeExpectedCompactBytes(width, height, format)` **当场算**，
   与 ①`raw.expectedCompactBytes`、②`raw.sdkPayloadBytes`、③`bytes->size()`
   **四者必须相等**（全部走 64 位）。任一项不等 ⇒ **拒绝**，错误文本带上**四个数字**。
   重算失败（格式不支持或乘法溢出）同样拒绝。
2. **`compactSizeMatches` 与这四个事实矛盾时同样拒绝**：四者已一致 ⇒ 它必须为真；
   它为假说明"上游算出的布尔量与它自己的两个字段矛盾"，**不得**忽略它再原样保存。
   （V2.3 只在此处查它是否为真，却没查它与长度事实是否一致。）
3. **格式／策略／字节序同样只看帧自身字段**：`rawPolicy` 必须等于
   `requiredRawPolicyOf(captureFormat)`；`declaredByteOrder` 必须为 `LittleEndian`
   （`BigEndian` **本批拒绝**，第 4 条）；`bitAlignment` 必须为 `LsbZeroPadded`
   （由 `isValidCombination` 一并拦住）。
4. **回落分支的元数据取自实际写出的 `image` 与已校验的 `captureFormat`**：
   格式／有效位／打包／对齐一律由这两者推出，**不得照抄 `raw` 的默认字段** ——
   `RawImagePayload` 的默认值恰好是 `Mono8/8/Unpacked`，于是
   "`captureFormat = BGR8` 的 3 通道图"会被描述成 `Mono8` 的单通道文件，
   元数据与文件自相矛盾（且矛盾方向诱导人按 1 字节/像素去读一条 3 字节/像素的行）。
   8 位回落的**通道数**也必须与格式相符（`Mono8` → 1、`BGR8` → 3），
   判据取自 `captureFormat` 而**不是** `image.channels()`。

**结果包元数据（14 个键 ＋ `display_image` 子对象，同批落盘）—— ⚠ 下表的 14 个键是
两个来源的**并集**，**不是**"每路都写这 14 个"；每一路实际写出哪些键见紧随其后的分组表：**

| 键 | 对应字段 |
|---|---|
| `data_source` | 实际载体（`"raw"` / `"image"` / `"none"`） |
| `raw_image` | **裸缓冲自己的宽高** `{width, height}`（V2.4 新增，**仅真身路径写出**，见下） |
| `pixel_format` | `format` |
| `valid_bits` | `validBits` |
| `packing` | `packing` |
| `valid_bit_alignment` | `bitAlignment` |
| `declared_byte_order` | `declaredByteOrder` |
| `sdk_payload_bytes` | `sdkPayloadBytes` |
| `expected_compact_bytes` | `expectedCompactBytes` |
| `compact_size_matches` | `compactSizeMatches`（已交付帧**恒为真**，Recorder 仍**复检**一次：为假即判契约错误） |
| `sdk_pixel_format_code` | `sdkPixelFormatCode`（诊断） |
| `sdk_padding_x` | `sdkPaddingX`（诊断，**不用于寻址**） |
| `sdk_padding_y` | `sdkPaddingY`（诊断，**不用于寻址**） |

⚠ **键集按数据来源分组**（V2.5 新增；011-A1 清理责任批）：上表**不是**"每路都写这 14 个键"，
而是**两个来源的并集**。每一路的键**必须由该路实际取得的事实决定**：

| 键 | 真身（`data_source == "raw"`） | 回落（`data_source == "image"`） |
|---|---|---|
| `data_source` | 写 | 写 |
| `pixel_format`／`valid_bits`／`packing`／`valid_bit_alignment`／`declared_byte_order` | 写（取自 `raw`＋已校验的 `captureFormat`） | 写（取自**已校验的 `captureFormat`**与**实际写出的 `image`**） |
| `raw_image` | 写 | **不写** |
| `sdk_payload_bytes`／`expected_compact_bytes`／`compact_size_matches` | 写 | **不写** |
| `sdk_pixel_format_code`／`sdk_padding_x`／`sdk_padding_y` | 写 | **不写** |

理由（与 `raw_image` **完全同构**，V2.4 只对 `raw_image` 做了这件事）：

· 回落路径**没有**原始载荷，也**没有任何一次 SDK 调用**产生过这些元数据 ——
  它们的值只会是 `raw` 的**默认值**（`0`／`false`）；
· 读包的人会看到"载荷 0 字节、长度相符 = false"配着一份几百字节的显示图文件，
  与"写一个会被误读的 0×0"是**同一类**误导；
· **"不存在的键"胜过"值不对的键"**：缺键一眼看得出**没有这项事实**，
  而 `0` 会被当成一个**测出来的数字** —— 后者不可证伪。
· 回落路径自己的格式描述**必须描述那份真被写出的文件**，
  故它取自 `captureFormat` 与 `image`，**不得照抄 `raw` 的字段**。

⚠ **不再有 `stride_bytes`／`padding_bytes`**（那两个名字表示"已确认的布局推导"，第 3 条已删）；
`ImageFrame` 原有的 `"type": <image.type()>` **不再是裸缓冲的解码依据**，
它只描述 8U 显示图（§5.5 第 2 条）。

**8 裸缓冲的完整解码依据里必须有"它自己的宽高"（V2.4 新增，011-A1 九项缺口 §4）**

⚠ V2.3 的结果包里**唯一**的宽高是 `display_image.width/height`，而它取自
`image.cols/rows` —— **加工产物**（12 位时是右移 4 位的结果）。于是：

- "有原始载荷、没有显示图"的那种包（显示转换尚未做、或调用方只要原始数据）
  **宽高为 0**：载荷完好、长度正确，却**没有任何字段说明它是几乘几**，
  二维图像**无法独立恢复**；
- 即便显示图在，拿它当载荷的几何依据也属"用显示图解释原始载荷"——
  与 V2.3 第 6 条明令禁止的 `image.type()` 同类错误。

∴ **真身路径（`data_source == "raw"`）必须写出 `raw_image:{width, height}`**，
取自 `RawImagePayload::width`／`height`（**设备回报**的原始尺寸）。
**回落路径不写该键**：那份文件就是显示图，写出它会让人以为存在一份原始载荷；
`data_source` 已如实表明来源，**不写**比"写一个会被误读的 0×0"好。

冻结后的解码依据：

| 路径 | 格式依据 | 几何依据 |
|---|---|---|
| 真身（`raw`） | `pixel_format`／`valid_bits`／`packing`／`valid_bit_alignment`／`declared_byte_order` | **`raw_image.width/height`** |
| 回落（`image`） | 同上（取自 `captureFormat`） | `display_image.width/height` |

⚠ **`width != height` 的包必须能被逐位置解码**：验收这类改动的用例不得只比
"总字节数"—— 宽高互换的两种解释**字节数相同**，长度断言一律通过。
必须用**非方形**几何、按包内几何**逐位置**校验像素值（见
`RecorderPackageTest.Mono12FrameWithRawButNoDisplayImageIsStillSaved`）。

### 5.29 取帧结果与诊断出口（V2.3 新增；**V2.4／V2.5 修订**）

    enum class OpStatus
    {
        Unset,            // 默认构造值 = 漏赋值；**不是成功**
        Ok,
        InvalidArgument, ContractViolation, NotImplemented,   // **判定类别**属本地
                                                              // ⚠ **不等于**"未调用过 SDK"，见下
        Timeout, NoFrame, NotStarted, Disconnected,           // 设备／取帧状态
        CorruptFrame,     // **SDK 调用成功，但帧不满足可交付条件**（不得归为 Timeout）
        SdkError          // 已调用 SDK，且按有依据的语义无法归类（原码保留）
    };

    enum class SdkCall
    {
        None,
        ImvEnumDevices, ImvCreateHandle, ImvOpen, ImvGetDeviceInfo,        // 打开阶段
        ImvSetEnumFeatureSymbol, ImvGetEnumFeatureSymbol,                 // 触发模式 / 像素格式
        ImvSetIntFeatureValue, ImvGetIntFeatureValue,                     // 曝光等
        ImvSetDoubleFeatureValue, ImvGetDoubleFeatureValue,               // 增益等
        ImvExecuteCommandFeature,                                         // 软件触发
        ImvStartGrabbing, ImvGetFrame, ImvReleaseFrame, ImvStopGrabbing,
        ImvClose, ImvDestroyHandle
    };

    struct SdkFailure { SdkCall call; int32_t code; };  // **操作名 + 返回码**，不留裸码

    // **调用抛出、没有返回码**的标记值（V2.5 新增）。**不是** SDK 返回码，
    // 也不属于 SDK 的任何码段 —— 它是"这次调用抛出了异常，我们没拿到码"。
    // ⚠ 显示时必须走**专用措辞**（"调用抛出异常、无返回码"），
    //   **不得**把它当原码打印（`-2147483648` 会被读成一个真实错误码）。
    inline constexpr int32_t kCallThrewCode = std::numeric_limits<int32_t>::min();

    struct OperationResult
    {
        OpStatus status = OpStatus::Unset;      // ← 消费者**只**依据它（ok() 也只看它）
        std::optional<SdkFailure> sdkError;     // **首次失败**的归属；清理失败**不覆盖**
        std::optional<SdkFailure> cleanupError; // 「**资源还回去了没有**」的唯一字段（V2.5）
        bool ok() const { return status == OpStatus::Ok; }
    };
    struct GrabResult : OperationResult
    {
        // ⚠ V2.4 新增：诊断**随本次结果走**，不继承任何历史状态
        std::optional<uint32_t> frameStatusRaw;  // SDK 帧状态原值；**未取得 = nullopt**
        std::string             diagnosis;       // 现场数值（长度/padding/格式原值）；成功时为空
    };   // **不存在**第二个状态字段（无 `GrabStatus`）

**为什么只有一个状态字段**：若同时有 `kind` 与 `status`，则 `kind == Ok && status == Timeout`
成为**可表示的状态**，而 `ok()` 只看其一；且第二个字段表达不了"格式不支持／参数错误／契约违背"。
现在结果只有 `status` 一个权威字段，`ok()` 与所有分支判断**一律**用它。
默认构造是 `Unset`（**不是** `Ok`）——"忘了赋值"不会被当成成功；
release 中被放过 ⇒ 记 `9004`（§5.27），并**必须在日志里点名是哪一路的哪个调用**。

**两个字段回答两个不同的问题**（V2.5 冻结，**这是全文的判据，不得再按单一维度理解**）：

· `sdkError` 回答"**失败的首因是什么**"（哪次调用、哪个码）；
· `cleanupError` 回答"**资源还回去了没有**"。

**判定类别与实际调用历史分别记录**（V2.5 新增的总原则）：

> 「本次判定属**本地类**（`InvalidArgument`／`ContractViolation`／`NotImplemented`）」
> 说的是**结论从哪里来**，它与"**此前有没有调用过 SDK**"是**两个问题**。
> ∴ **本地判定不代表"未调用过 SDK"** —— 判定若依据的是某次**成功**调用的返回值，
> 那条路径**必须如实保留** `{该调用, IMV_OK}`，写成 `nullopt` 是在抹掉一次真实调用。

V2.3／V2.4 曾写"**本地参数／格式／契约错误 ⇒ 两者都是 `nullopt`**"，
**该判据已废除**：它把一个**调用历史**问题塞进了**判定类别**里，
于是 §5.31 触发读回（读回**成功**但与请求不一致 ⇒ `ContractViolation`）
这类路径在文档上"必须为空"、在代码里却带着真实的 `{ImvGetEnumFeatureSymbol, 0}` ——
**同类矛盾**，本版一并消除。

| 情形 | `status` | `sdkError` / `cleanupError` |
|---|---|---|
| 本次**确实没有发起**任何 SDK 调用（如 `timeoutMs == 0`、入口预算耗尽） | 本地类／`Timeout` | `sdkError` 为 `nullopt`；`cleanupError` 为 `nullopt`（**没有资源可还**） |
| 调用了 SDK 且返回 `IMV_OK` | 先 `Ok`；**随后**检查帧，帧不可用则改 `CorruptFrame` | `{ImvGetFrame, 0}`（`IMV_OK` 如实保留） |
| 调用了 SDK 且**成功**，而本地判定不通过（如触发读回不一致 ⇒ `ContractViolation`） | 本地类 | **`{该成功调用, IMV_OK}`**（**不得**写 `nullopt`，见上） |
| 调用了 SDK 且失败 | 按映射表归类，归不了则 `SdkError` | `{该调用, 原码}` |
| **调用抛出了异常、没有返回码** | 按该步的性质归类；无法归类则 `SdkError` | `{该调用, kCallThrewCode}` |
| **已有主失败**，清理也失败 | **保留首因**（`status` 与 `sdkError` 都不变） | 失败进 `cleanupError` **也**进（**这是"资源没还回去"的唯一出口**） |
| **原操作成功、必要清理失败**（如 `GetFrame` 成功而 `ReleaseFrame` 失败） | **整体返回失败**，以**该清理调用**为错误来源（按同一映射表定 `status`） | `sdkError = {ImvReleaseFrame, 原码}`，**同时** `cleanupError = 同一项` |

⚠ "收尾失败永不影响成功判定"**只在"已有主失败"时成立**：主操作成功而必要清理失败时
**不能仍报 `Ok`** —— 那是把"资源没还回去"说成成功。
⚠ **`cleanupError` 的写入与 `status` 无关**（V2.5 的关键修正）：**上面最后两行都要写它**。
上一版只在"已有主失败"那一行写、提升为失败那一行**不写** ⇒ 同一个"释放未获确认"，
**帧检查通过**时（状态被提升为 `SdkError` ⇒ 通道被禁用）与**帧已损坏**时
（首因被保留 ⇒ 状态不在禁用集合里 ⇒ 通道**继续可用**）拿到**相反**的处置，
区别只在于"帧本身好不好" —— 而帧好不好与"缓冲还回去了没有"是**两个不相干的问题**。
下游（`MultiCameraManager`）的通道处置判据因此改为**只看 `cleanupError`**。
⚠ **清理错误必须同时带操作名与返回码**：同是 −119，`ImvGetFrame` 超时与 `ImvReleaseFrame`
超时是**完全不同**的故障。
⚠ **`kCallThrewCode` 是标记值、不是码**：同一次 `ImvReleaseFrame`，
"返回 −119"与"调用抛出、没拿到码"是**两种**事实（前者说明 SDK 拒绝或超时，
后者说明**那次调用是否已经把缓冲还回去了无法判断**）。显示时必须走专用措辞，
**不得**把 `-2147483648` 当原码念出来。
⚠ **RAII 的落点要保证清理结果能进返回值**：守卫**不得**在析构里静默丢弃结果
（析构时返回值已定，改不回去）⇒ 唯一出口处**显式**调用清理并合并结果
（例如守卫提供 `std::optional<SdkFailure> cleanup()`，函数体在返回前合并）。
**输出帧只在"成功条件全部满足"之后才写入** ⇒ 最终 `status == Ok` 才交付帧，
清理失败同样不交付。

**清理责任与异常安全**（V2.5 新增；接口面的调用责任见 SYS-04 V2.6 §6.2）：

1. **`cleanup()` 不得抛异常**（`noexcept`）。它在**栈展开中**被调用，
   从这里抛出去会让函数尾部的 `throw;` **执行不到**，调用方收到的是**二次异常**，
   原异常连同它的类型与文本**永久丢失**。
2. **释放调用本身抛出时不重试**：如实返回 `{ImvReleaseFrame, kCallThrewCode}`。
   那次调用**是否已经把缓冲还回去了无法判断**，盲目重试就是"一个帧释放两次"
   （SDK 内部缓存计数错乱，而错误码可能仍是 0 —— **无声的破坏**）。
3. **异常出口的三步各自独立保护**（缺一不可）：① 描述原异常、② 显式清理、
   ③ 写诊断。这三步**每一步都要分配内存**，任一步抛出都会让末尾的 `throw;`
   执行不到 ⇒ 每步关在自己的保护里，**失败只损失那一步的产物**；
   **末尾的 `throw;` 无条件执行 ⇒ 原异常必定原样上抛**。
4. **释放未获确认 ⇒ 停止该路后续采集**（不只是记一笔）：未归还的缓冲会被 SDK
   内部缓存复用、污染后续帧，故该路**不再续采**；⚠ 但这**不是**"判定这台相机坏了"
   —— 未知的是**资源状态**，措辞只谈资源；**主失败与清理诊断都必须保留**，
   自动恢复须先有**资源恢复依据**（本版不做）。
5. **诊断必须能在栈展开后读到**：局部结果随栈展开销毁，故异常路径的现场留在
   **后端诊断状态**（由既有的"最近一次失败文本"出口读取），正常返回路径仍走返回值。

**错误映射：先判"调用是否失败"，再判"帧是否可用"**（顺序是**硬性**的）：

1. `ret == IMV_OK` ⇒ **先**置 `Ok`，**然后**才检查取得的帧（长度等式、宽高非零、格式已支持）。
   帧检查不过 ⇒ 改 `CorruptFrame`，`sdkError` 仍是 `{ImvGetFrame, IMV_OK}`（SDK 调用本身确实成功了）。
   **调用成功但帧损坏，绝不算超时。**
   ⚠ **不得**在 `IMV_GetFrame` 失败时去读 `frameInfo.status`／`frameInfo.size` 来"交叉判断超时" ——
   SDK **没有**保证失败时这些输出字段有效。帧侧判据只在**调用成功**之后使用。
2. `ret != IMV_OK` ⇒ 按**该码自身的文档化含义**归类；**证据不足的一律 `SdkError` ＋ 原码保留**：

| SDK 码（常量注释原文） | 归类 | 依据 |
|---|---|---|
| `IMV_TIMEOUT` −119「超时」 | `Timeout`（**仅当** `sdkError->call == SdkCall::ImvGetFrame`） | 常量注释明确；⚠ **"该函数返回 −119"本身是推断**，见 §13 附录 A |
| `IMV_NOT_GRABBING` −117「相机已停止取图」 | `NotStarted` | 常量注释明确 |
| `IMV_NOT_CONNECTED` −118「设备未连接」 | `Disconnected` | 常量注释明确 |
| `IMV_NOT_SUPPORT` −113「设备不支持的功能」 | `NotImplemented` | 常量注释明确 |
| `IMV_INVALID_PARAM` −103「错误的参数」 | `InvalidArgument` | 常量注释明确 |
| −114「取图恢复中」／−115「重连恢复中」 | **`SdkError`**（原码保留） | 注释说的是"恢复中"，**没说断连**，也没说调用方该做什么 |
| −116「连接不可达」 | **`SdkError`**（原码保留） | "不可达"≠"已断连"，语义不足以支撑 `Disconnected` |
| −122「调用时序错误」 | **`SdkError`** | 时序错误不等于"未就绪"，归 `NotStarted` 属推断 |
| 其余（−101／−102／−104−112／−123−130 等） | `SdkError`（原码保留） | 无"调用方该如何处置"的依据 |

**状态全表**（每个取值都写明**应用码／是否交付帧／是否改变通道可用性／如何参与聚合**，
**不留"表里没有"的取值**）：

| `OpStatus` | 来源（谁产出） | 交付帧 | 通道可用性 | 应用码 | 聚合严重度（大者胜） |
|---|---|---|---|---|---|
| `Unset` | **默认构造／漏赋值**（⚠ 本仓**没有**运行期 `assert()`，"出现即断言"与实现不符 ⇒ 本行只写**实际行为**：被放过即记 `9004` 并点名通道与调用） | 否 | — | `9004`（被放过时） | **最高**（100） |
| `ContractViolation` | 本地判定（组合表矛盾、`compactSizeMatches` 为假后仍被交付、`capturedCount` 与帧数不符、触发读回不一致、报 `Ok` 却交付空图） | 否 | **不变** | `1006` 取帧契约错误（本地判定） | 90（正常运行时**不可达** ⇒ 出现即需人看） |
| `InvalidArgument` | 本地判定（`timeoutMs == 0`、路径／序列号等参数非法）；⚠ **判定属本地 ≠ 本次未调用过 SDK** | 否 | **不变** | `1006` | 85（开发期错误，同上） |
| `Disconnected` | SDK 码 −118 等**注释明确**者 | 否 | **禁用**（R09 保留的现行行为） | `1002`（措辞＝"按 SDK 错误码判定，未经连接事件确认"） | 80 |
| `SdkError` | 已调用 SDK 且归不了类（含 −114／−115／−116／−122） | 否 | **禁用**（措辞＝"分类未明确"） | `1005` | 70 |
| `CorruptFrame` | 调用成功但帧不可交付：**`IMV_FrameInfo::status != 0`**／长度为零／长度不符／格式未知／几何非法／**后端报成功却交付空图**（见下） | 否 | **不变** | `1005`（文本＝"SDK 调用成功，但帧不满足可交付条件"） | 60 |
| `NotImplemented` | 本地判定（`Mono12Packed`、`FreeRun`、其它未实现格式码）；⚠ **判定属本地 ≠ 本次未调用过 SDK**，本仓现有路径确实都在调用前拒绝，但这是**路径事实**、不是可依赖的规则 | 否 | **不变** | `1007` | 50 |
| `Timeout` | SDK −119（**仅当** `sdkError->call == ImvGetFrame`）或**本地预算耗尽**；⚠ 预算耗尽有**三种**情形，**其中一种可发生在发令之后**：入口／读回后＝本次**未调用** SDK，**发令后取帧前**＝**已经调用过** `ImvExecuteCommandFeature`（调用历史如实保留，见 §5.32） | 否 | **不变** | `1003` | 40 |
| `NoFrame` | ⚠ **V2.4：真实后端目前不产出它**（见下） | 否 | **不变** | `1003` | 30 |
| `NotStarted` | SDK −117，或**通道不可用／未启动**（含入口 `availableCount < 2`） | 否 | **不变** | `1004` | 20 |
| `Ok` | 成功，且帧已通过全部检查 | **是** | 不变 | — | 0（不参与） |

**聚合的完整规则**：
· 按上表严重度取最大者作为 `aggregate` 的来源，**全部取值都在表内**。
· ⚠ **本地判定不得冒充 SDK 错误**（V2.5 改写）：`ContractViolation`／`InvalidArgument` 的
  **应用码是 `1006`**、**不是 `1005`**，这是本条的**实质**（消费者据码分流）。
  ⚠ **但不得由它推出"`sdkError` 必须为 `nullopt`"** —— V2.4 及更早的那句判据
  **已废除**：判定类别与实际调用历史**分别记录**，本地判定若依据一次**成功**的调用，
  `sdkError` **必须**如实保留 `{该调用, IMV_OK}`（本工程现有的此类路径＝
  §5.31 **触发读回不一致**：读回调用确实成功，不一致是本地比对得出的）。
  把两者绑在一起会在"应用码对了"的同时**抹掉一次真实调用**。
  ⚠ 反向的误读同样禁止：这不等于"本地判定**必须**带 `sdkError`" ——
  **报 `Ok` 却交付空图**（管理器判定 `ContractViolation`）时，管理器调用的是
  `ICameraBackend::grab()`，**原样转发**后端给出的调用诊断即可，
  **不得**补造 `{ImvGetFrame, 0}`（见 §5.29 修订二第 5 条）。
· ⚠ **通道处置以 `cleanupError` 为第一判据**（V2.5）：`cleanupError` 有值 ⇒
  **停止该路后续采集**（措辞＝"释放未获确认"，**不改写**首因），
  其后再按 `status` 的分类走禁用集合；两者**同时成立时以"释放未获确认"为准**
  （它是资源事实，与"设备为什么失败"无关）。
· ⚠ **`Unset` 不是"未尝试"的正常取值**：未尝试的每一路也必须**显式赋值** ——
  预算耗尽 ⇒ `Timeout`＋`skippedReason`；通道不可用 ⇒ `NotStarted`＋`skippedReason`；
  入口不足 ⇒ 每路 `NotStarted`＋`skippedReason`。**不得**靠默认构造混过去。
· ⚠ **`attempted == false` 的本地超时也必须进聚合**（见 §5.32）。
· 归属三分：**未尝试的原因** → `ChannelGrabRecord::skippedReason`；
  **发令失败** → `trigger.sdkError->call == SdkCall::ImvExecuteCommandFeature`；
  **取帧失败** → `result.sdkError->call == SdkCall::ImvGetFrame`。三者**不得混成一个 `bool`**。

**V2.4 修订一：帧状态参与有效性判断，且只推出 `CorruptFrame`**（011-A1 九项缺口 §2）

`IMV_FrameInfo::status != 0` ⇒ **`CorruptFrame`**，文本＝"**SDK 调用成功，但帧不满足
可交付条件**（frameInfo.status=…）—— 该状态的位含义**未经文档化**，本批**不解释**"。
冻结规则：

1. **只此一种归类**：`status != 0` 既**不能**证明"本拍无帧"、也**不能**证明"帧损坏"，
   故**不得**推出 `NoFrame`（V2.3 的"任意非零状态即可推出 `NoFrame`"表述**已删除**），
   也**不得**降级为 `Timeout`。`NoFrame` **保留**给将来**有明确"无有效帧"依据**的判据；
   真实后端**目前没有任何路径产出它**（这是**如实陈述**，不是"忘了实现"）。
2. **`sdkError` 保持 `{ImvGetFrame, IMV_OK}`**（0）：SDK 调用**确实成功**了，
   改写成错误码等于篡改事实。
3. **原值进入本次诊断**：`GrabResult::frameStatusRaw = view.status`（**不是** `nullopt`，
   也**不是**被当成 0）。`nullopt` 的含义被严格限定为"**本次调用未取得帧状态**"。
4. **调用失败时不读 `view` 的任何字段**：`ret != IMV_OK` 路径不得交叉判断
   （SDK **没有**保证失败时输出字段有效）⇒ 该路径 `frameStatusRaw` 一律 `nullopt`。
   ⚠ "未取得"与"正常值 0"**必须可区分** —— 否则"这一路根本没取到帧状态"
   与"取到了、设备说没问题"在结果里长得一样。
5. **检查顺序**：帧状态是**设备对该帧自身的判定**，不依赖我们的字段校验 ⇒
   放在**空指针检查之后、几何／格式／长度之前**。
6. **仍须释放且恰好一次**；释放失败按既有规则合并（`cleanupError`）。
   `status != Ok` ⇒ **不写输出帧**（`ICameraBackend::grab` 的既有契约，`ImageFrame` 一个字段都不动）。

**V2.4 修订二：诊断的传递路径，以及"后端报成功却交付空图"**（011-A1 九项缺口 §2／§8／§9）

**诊断传递路径**（固定为这一条，不得另开）：

    SDK 帧视图 → GrabResult（本次诊断：frameStatusRaw / diagnosis） → ChannelGrabRecord → 日志／失败记录

1. **`GrabResult` 承载"这一次调用"的现场**：`frameStatusRaw` 与 `diagnosis`
   （长度为例：`sdk_payload_bytes=`／`expected_compact_bytes=`／`recomputed_compact_bytes=`／
   `sdk_pixel_format_code=0x…`／`padding_x=`／`padding_y=`）。**成功时 `diagnosis` 为空**。
2. ⚠ **上层不得改去读后端的 `lastErrorText()`**：那是"最近一次失败文本"，
   **可能属于更早的调用**（陈旧），当作本轮事实就是张冠李戴。
   异常路径是**唯一**例外（栈展开后返回值已丢失，见 SYS-04 V2.6 的异常安全条款）。
3. **失败时不得为了传诊断而修改输出 `ImageFrame`** —— 不交付失败帧是一条独立纪律。
4. **"后端报 `Ok` 却交付空图" ⇒ `ContractViolation`（管理器判定）**：
   后端调用的语义是"交付一帧"，报成功却给空图是**契约违背**，不是成功。
   ⚠ 它**不得**被记成 `Ok`：聚合取严重度最大者，一路空图会把**整轮**判成 `Ok`，
   而控制器又跳过所有 `Ok` 通道拼降级说明 ⇒ **空图的原因进不了任何用户可见文本**。
5. **调用诊断原样转发，**不**补造**：管理器调用的是 `ICameraBackend::grab()`，
   虚拟后端／脚本替身**可能根本没调用 SDK** ⇒ 原来是 `nullopt` 就继续是 `nullopt`，
   `cleanupError`／`diagnosis`／`frameStatusRaw` 同样原样转发。
   ⚠ **不得**补一个 `{ImvGetFrame, 0}` —— 那是在替后端声明"我们调用过 SDK 且成功了"，
   而管理器根本不知道这件事。
6. **`capturedCount` 的判据不变**：`status == Ok` **且**交付的图合法非空 ——
   交付判据**不依赖**"某个实现当前恰好让两者等价"。空图记为契约违背后，
   该路自然不计入 `capturedCount`，两件事在语义上保持独立。

### 5.30 `DeviceIdentity`（V2.3 新增）

    struct DeviceIdentity
    {
        std::string modelName;                     // 未取得 = 空
        std::optional<std::string> serialNumber;   // 未取得 = nullopt
        bool queried = false;                      // 是否真的问过 SDK（虚拟后端 = false）
    };

**来源（已核 `IMVDefines.h` 的 `IMV_DeviceInfo`）**：型号＝`.modelName`，
序列号＝`.serialNumber`，厂商＝`.vendorName`，接口类型＝`.nInterfaceType`，
设备键＝`.cameraKey`（内容即 `"厂商:序列号"`）。
⚠ **该结构体没有"序列号是否有效"的标志位** ∴ "未取得"的判据是**字段为空串**
（`serialNumber[0] == '\0'`）⇒ `std::optional` 置 `nullopt`。

**三条归属纪律**：

1. **`DeviceIdentity` 不进 `ImageFrame`**：它是**设备**的属性，不是**帧**的属性。
2. **型号匹配不能替代设备身份匹配**：按**序列号**打开（`serialNumber` 精确匹配；
   配置里该值为空 ⇒ **明确失败**，不取"第 0 个设备"），不按型号、不按设备序号。
   匹配到 0 个或**多个** ⇒ 明确失败，并把枚举到的型号／序列号**全列进错误信息**。
   **不得**从配置复制目标序列号去填实际身份；未取得就写"未取得"。
3. **后端类型由装配摘要的独立一栏表达**，型号栏**不得**填非型号内容
   （虚拟后端的 `modelName` 是空串，**不是** `"virtual"`）。

### 5.31 `CameraTriggerMode` 与 `TriggerModeState`（V2.3 新增）

    enum class CameraTriggerMode { FreeRun, Software, Hardware };

    struct TriggerModeState
    {
        CameraTriggerMode requested;                  // 请求值（来自配置/装配）
        std::optional<std::string> selectorReported;  // 实际生效的 TriggerSelector
        std::optional<std::string> switchReported;    // 实际 TriggerMode
        std::optional<std::string> sourceReported;    // 实际 TriggerSource
        std::optional<CameraTriggerMode> reported;    // 由上面三项**合成**；读不全 ⇒ nullopt
        bool consistent = false;                      // 读全了且与 requested 一致才为 true

        // ⚠ V2.4 新增：**逐项**读回现场，与三项一一对应
        std::array<TriggerFeatureReadback, 3> readbacks;
    };

    struct TriggerFeatureReadback          // V2.4 新增
    {
        std::string                     feature;          // "TriggerSelector"/"TriggerMode"/"TriggerSource"
        bool                            callAttempted = false;  // 是否真的调用过 SDK
        std::optional<SdkFailure>       failure;          // 调用失败：**操作名 + 原码**
        bool                            returnedEmpty = false;  // 调用成功但返回空串
        std::string                     value;            // 返回值（可能为空）
    };

**V2.4 补充：为什么读回必须逐项带名字，以及两个分支的处置**（011-A1 九项缺口 §7）

1. **只有 `vector<SdkFailure>` 无法说明失败的是哪个特性**：三项读回都走
   `IMV_GetEnumFeatureSymbol`，码相同、调用名相同 ⇒ 记录里只写"读回失败"
   等于把"选择器没读到"与"触发源没读到"混成一件事，而两者的现场动作不同。
2. **"调用成功但返回空串"必须与"SDK 调用失败"分开**：前者说明**特性存在、值为空**
   （设备状态异常），后者说明**这次调用没成功**（链路或参数问题）。
   `returnedEmpty` 与 `failure` 是**两个**字段，**不得**用一个空串同时表达两件事。
3. **读回不完整（`reported == nullopt`）或被请求值不一致时，管理器一律不发令、不取帧**
   （见下表）—— 这是**取帧前置条件**的检查，不是"尽力而为"。
4. ⚠ **V2.3 的三元表达式（`reported` 为空 ⇒ 用 `requested`）必须删除**：
   它把"读回失败"悄悄变成"读回成功且恰好一致"，于是后续按**请求值**去发令 ——
   而设备实际处于什么模式**无人知道**。读回不完整时**只能**如实报"未知"。

| 情形 | 本路结果 | 后续动作 |
|---|---|---|
| 读回不完整、`reported` 未知 | `NotStarted`／`1004`，文本"本轮无法确认触发前置条件" | **不发令、不取帧** |
| 读回完整但与请求不一致 | `ContractViolation`／`1006`，记录**请求值与实际值** | **不发令、不取帧** |
| 读回完整且一致 | 按**实际回读模式**执行 | **仅 `Software`** 发软件触发 |

5. **两种失败都不永久禁用通道**：下一轮**允许重新读回**（读回失败是瞬态）。
   "未知"**不得**写成"设备已停止取流"（那是 `NotStarted` 的另一条来源，
   措辞与现场动作都不同）；**读回调用的失败信息必须保留**（进 `readbacks[].failure`），
   不能因为"结果归类是 `NotStarted`"就把 `sdkError` 当成它的替代品。
6. **`sdkError` 的取值按实际调用情况如实给出**，**不得**从"未调用取帧"反推任何结论。

**V2.4 补充：`consistent` 的消费者**（V2.3 只定义了它，却没有任何代码读它）

`consistent` 是"读回完整**且**与请求一致"的判据。管理器必须**显式**读它并按上表分支 ——
"读回完整但请求值与实际值不符"这一格此前既不诊断也不记录，静默按 `reported` 走：
一次"配置没生效"因此表现为**正常运行**，而它的真实后果是"按错误的触发方式取帧"。

**冻结规则：**

1. **`bool triggerMode` 扩为三值枚举**：`camera.yaml` 的 `trigger_mode` 同步改为
   `software|hardware|free_run`；**旧数字值（`0`／`1`）显式拒绝并给迁移提示**，
   不让 `1` 静默变成别的东西。
2. **读回必须读到三项，不能只读一个开关**：⚠ **只读 `TriggerMode == "On"` 分不出
   软件触发与硬件触发** —— 两者都是 `"On"`，区别只在 `TriggerSource`。
   选择器／开关／触发源三项都要读，且**读不到要能表达"未知"**（`nullopt`），
   **不得**把"没读到"默认成某个合法模式（那会让一次读失败伪装成"模式正确"）。
3. **落地为 GenICam 特性字符串**（SDK **没有** `TriggerMode_Off` 这类 C 枚举成员，
   模式是**运行时字符串特性名**；SDK 全树**没有 `"FreeRun"` 这个词**）：

| `CameraTriggerMode` | `TriggerMode`（symbol） | `TriggerSource`（symbol） |
|---|---|---|
| `FreeRun` | `"Off"` | 不设置 |
| `Software` | `"On"` | `"Software"` |
| `Hardware` | `"On"` | `"Line1"`（SDK 样例中唯一出现的硬触发源） |

   读写用 `IMV_Set/GetEnumFeatureSymbol`；读回用 `IMV_GetEnumFeatureSymbol` 分别取
   `TriggerSelector`／`TriggerMode`／`TriggerSource` 三项。**任一项读失败或返回空 ⇒
   对应 `optional` 置空**，合成 `reported` 随之 `nullopt`、`consistent = false`。
   ⚠ 因 SDK 无枚举成员，**"Off 即自由运行"这一语义只由样例代码支持、无文档保证**
   （§14 附录 B），∴ `consistent` 必须如实反映实测差异，**不得**在文档里把该语义写成 SDK 保证。
4. **配置序列冻结为 `TriggerSelector`（`"FrameStart"`）→ `TriggerSource` → `TriggerMode`**
   —— 依据＝SDK 每个样例在设 `TriggerMode` 之前都先设 `TriggerSelector`（§14 附录 B）。
   跳过 `TriggerSelector` 会让触发设置落到别的选择器上，表现为"设了却不起作用"。
5. **硬触发另需 `TriggerActivation = "RisingEdge"`**（§14 附录 B）。本批**不实施**
   `Hardware`（PH-01），但本条在此**冻结**，免得 PH-01 时重新发现。
6. **本批 CAM25 实施 `Software`**；`Hardware` 归 **PH-01** 不实施；
   `FreeRun` 可表达但真实后端**返回 `NotImplemented`**，**不静默当自由运行**。

### 5.32 `CaptureRound` 与 `ChannelGrabRecord`（V2.3 新增）

    struct ChannelGrabRecord
    {
        CameraRole      role;
        bool            attempted;      // 本轮是否真的尝试过
        std::string     skippedReason;  // `attempted == false` 时：**为什么没尝试**
        GrabResult      result;         // 取帧结果（含 status / sdkError / **本次诊断**）
        OperationResult trigger;        // 软件触发发令结果（`SdkCall::ImvExecuteCommandFeature`）
        uint64_t        timestampNs;

        // ⚠ V2.4 新增：**本次取证**，直接转发 `GrabResult` 的同名字段与
        //    `TriggerModeState::readbacks`（不得改读后端的 `lastErrorText()`）
        std::string                          diagnosis;
        std::optional<uint32_t>              frameStatusRaw;
        std::array<TriggerFeatureReadback, 3> readbacks;
    };

    struct CaptureRound
    {
        std::array<ChannelGrabRecord, 3> channels;
        int      capturedCount;   // 本轮**成功交付的有效帧数**（见下）
        bool     succeeded;
        OpStatus aggregate;       // 三路按 §5.29 严重度收敛
        uint64_t startedNs;       // 本次 capture() 的起止（**实测耗时**，见下）
        uint64_t finishedNs;
    };

**冻结规则：**

1. **`capturedCount` 的判据 = "本轮成功交付的有效帧数"**：该路 `status == Ok`
   **且**交付的图合法（与上层既有"图非空"判据同源）。
   ⚠ **不承诺**它等于最终成功写盘的文件数 —— 落盘还受 Recorder 的分支与失败影响，
   两者若不一致是**正常**的，**不得**据此断言"计数错了"。
2. **每轮 `capture()` 覆盖一次**，读取走 `IMultiCameraManager::lastCaptureRound()`
   （SYS-04 V2.6 冻结为**纯虚**）。
3. **成功时每路结果仍保留**（"两路成功、一条超时"那条超时的证据必须留下）。
4. **`aggregate` 按 §5.29 状态全表取最大严重度**；⚠ **`attempted == false` 且
   `result.status != Ok` 的通道同样参与聚合** —— 预算耗尽时**没有调用 SDK**，
   却确实产生了一次本地 `Timeout`，只挑"尝试过且失败"会把它整条漏掉；
   其 `skippedReason` 记明**为什么没尝试**（预算耗尽／通道不可用／入口 `availableCount < 2`）。
5. **`startedNs`／`finishedNs` 是实测耗时**：本批**保留同步采集**，但把
   "一次 GUI 回调里的 `capture()` 实际占用了多久"变成**可测事实**。
   ⚠ **删除"组预算 = GUI 线程硬上限"这一承诺**（不成立）：一次回调里会连采多组，
   复制、转换、发令也都不受 `GetFrame` 等待参数约束。

**V2.4 补充：每轮记录的"清旧、留新"，以及 P1 的三个预算检查点**
（011-A1 九项缺口 §3／§9）

1. **每轮开始处只清除上一轮的陈旧错误信息**；**本次实际取得的事实一律保留**：

| 情形 | 本轮应保留的记录 |
|---|---|
| 正常取帧 | `frameStatusRaw = 0`（**不是** `nullopt`）、实际调用诊断、实际触发读回；**错误文本为空** |
| 入口预算不足 | 无本次读回／触发／帧状态信息（**确实没有取得**） |
| 读回后、发令前预算不足 | 保留**本次读回结果** |
| 发令返回后、取帧前预算不足 | 保留**读回与触发结果**、`attempted = true`、帧状态**未取得**（`nullopt`） |

   ⚠ **不得读 `backend->lastErrorText()` 进本轮记录**（§5.29 修订二第 2 条）。
   ⚠ **"正常事实的存在"本身不得触发 WARN**：诊断出口的判据只看该路
   `status != Ok` 或本次诊断非空 ⇒ **全部正常的一轮不产生任何输出**。
2. **期限传递的三个检查点**（同一份预算，三处各自重读时钟、各自措辞）：

| 检查点 | 条件 | 软件触发次数 | `grab` 次数 | 该路记录 |
|---|---|---|---|---|
| 入口（连模式读回都不执行） | 剩余 < 1 ms | **0** | **0** | `Timeout` ＋ `sdkError = nullopt` ＋"预算耗尽" |
| 读回后、**发令前** | 重读钟 ⇒ 剩余 < 1 ms | **0** | **0** | `Timeout` ＋"**发令前预算耗尽**" |
| **发令返回后**、取帧前 | 重读钟 ⇒ 剩余 < 1 ms | **1** | **0** | `attempted = true`、**保留触发结果**、`Timeout` ＋"**已发令、未取帧（取帧前预算耗尽）**" |

   ⚠ 第三行**不得**写成"既未发令也未取帧" —— 脉冲已发出，设备侧会产出一帧
   （无人回收），把它记成"没发令"会让现场去查触发链路，而根源是预算。
   `timeoutMs` **每个检查点用重读后的读数重算**：`min(单次上限, 组剩余(重算), 距 deadlineNs 的剩余(重算))`
   （"无期限"的取值不参与取小，见 §2.5）。
   ⚠ **时限传到下游、使用时已过期**是 V2.3 的真实缺陷：当时只在入口读一次钟，
   随后三次阻塞 SDK 读（模式读回）＋ 发令 ＋ `grab` **全部复用那一个读数** ⇒
   "还剩 300 ms"可以对应实际已经到期。

### 5.33 `DeviceState` 增 `DISCONNECTED`（V2.3 新增）

`DeviceState`（§4.1）新增一个值 `DISCONNECTED`。**冻结规则：**

1. **仅在"设备不在"时置位**：① SDK 错误码的注释**明确**表示未连接（−118
   `IMV_NOT_CONNECTED`）；或 ② 枚举结果里**没有**配置的目标序列号
   （这是一个**直接观察到的事实**，不是从错误码推断的）。
   `Timeout`／`NoFrame`／`NotStarted`／`CorruptFrame`／`SdkError` **都不置位** ——
   它们是本轮取帧的瞬态或未知结果，**不是"设备不在了"**。
2. **措辞纪律**：与本值对应的文本一律写"**按 SDK 错误码判定为断连，未经连接事件确认**"。
   本批**不注册** `IMV_SubscribeConnectArg`（理由见 §14 附录 B 第 1 条），
   ∴ 没有任何事件确认发生，**不得**写成"已确认断连"。
3. ⚠ **本枚举只表达状态，不在枚举注释里裁定恢复策略** —— 此处**不写**"断连可重连、
   故障不重试"之类的处置结论：那属于**待裁决**的 Q-D1，且 `SYS-08` 的相关条文本身
   存在撞号／悬空（见《SYS-08-§7引用勘误.md》）。**不**新增第二个并行的就绪性布尔量。

---

## 6 配置结构体（冻结）

### 6.1 `CameraConfig`

    struct CameraConfig
    {
        std::string       cameraId;      // **通道逻辑名**（`cam25`），取自 camera.yaml 的 `id`
        std::string       serialNumber;  // V2.3 新增：设备**序列号**期望值（空 = 未绑定）
        std::string       backend;       // V2.3 新增：`"virtual"` 或 `"imv"`（**缺省即失败**）
        CameraRole        role;
        int               width;
        int               height;
        double            exposureTime;  // s
        double            gain;          // dB
        CameraTriggerMode triggerMode;   // V2.3：由 `bool` 改三值枚举
    };

**V2.3 变更（裁决 C-015）：**

1. **新增 `serialNumber`**（期望值）与 **`backend`**。
   ⚠ `cameraId` 是**通道逻辑名**，**不是**设备序列号 ——
   本文档 V2.2 及更早的代码注释曾写"cameraId 是设备序列号（华睿 A7A20MU201 的唯一标识）"，
   那**把型号当成了唯一标识**，是 011-A1 验收清单中"逻辑 ID ↔ 硬件身份混淆"一类的根源。
   V2.3 **明确**：逻辑 `cameraId` 由配置决定、**不参与设备匹配**；设备匹配**只**看 `serialNumber`。
2. **`backend` 缺键 = 配置错误、启动失败**（**不做隐式默认**）：不按"检测到 SDK 就切换"，
   也**不得**在真实相机打开失败后静默换虚拟。理由：若按"检测到 SDK 就用真实后端"来默认，
   则现场 SDK 装好那一刻，虚拟配置会**静默变成**真实采集，而操作者以为自己在跑仿真。
3. **`triggerMode` 由 `bool`（true = 硬触发）改三值 `CameraTriggerMode`**，语义见 §5.31；
   ⚠ 旧配置里的数字值 `0`／`1` **显式拒绝**并给迁移提示 —— 不让 `1` 静默变成别的东西。
4. **`camera.yaml` 的键表归 SYS-17 V1.3 管**（本文档只冻结结构体字段）。

### 6.2 `OpticalRigConfig`

    struct OpticalRigConfig
    {
        std::string calibrationDir;
        std::string calibrationId;
    };

### 6.3 `TurntableConfig`

    struct TurntableConfig
    {
        std::string protocol;         // "sdk" | "pekod" | "rs485" | "network"
        double      azimuthMin;       // deg
        double      azimuthMax;       // deg
        double      elevationMin;     // deg
        double      elevationMax;     // deg
        double      coarseSpeed;      // deg/s
        double      fineSpeed;        // deg/s
        double      centerThreshold;  // pixel，默认 50
    };

**裁决 C-20：** 原 `maxIterations`（对准最大迭代次数）**删除**。它与 SYS-08 §7.3〔引用无效·依据待裁决·见 Q-D2〕 的 ALIGN 最大尝试次数是同一语义，保留两个名字会导致两处数值可能不一致（SYS-10 §14 与 ENG-09 各自定义）。冻结为：**唯一数据源是 `MeasurementConfig::maxAlignAttempts`**，`AlignmentController` 向 `RetryManager` 查询剩余次数，不再自持计数器（SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 1）。

### 6.4 `TriggerConfig`

    struct TriggerConfig
    {
        std::string source;              // "hardware" | "virtual"
        double      periodMs;            // 触发周期
        uint64_t    syncToleranceNs;     // 三相机时间戳允许偏差
    };

### 6.5 `MeasurementConfig`

    struct MeasurementConfig
    {
        // 评分权重：Score = w1*Q + w2*F + w3*M - w4*E
        double w1;
        double w2;
        double w3;
        double w4;

        double minSharpness;         // 图像质量下限
        double minTargetPixelSize;   // 目标像素尺寸下限
        int    minFeatureCount;
        double minMatchRatio;

        int    captureFrameCount;    // CAPTURE 阶段采集帧数，5~10

        // ---- 重试与超时（SYS-08 §7 的唯一数据源）----
        uint64_t taskTimeoutNs;      // 任务级时限 T_task，默认 60e9；测试可注入短值
        uint64_t searchTimeoutNs;    // 0.5e9
        uint64_t targetFoundTimeoutNs; // 0.2e9
        uint64_t alignTimeoutNs;     // 2.5e9
        uint64_t stabilizeTimeoutNs; // 3.0e9
        uint64_t selectTimeoutNs;    // 0.3e9
        uint64_t captureTimeoutNs;   // 1.5e9
        uint64_t solveTimeoutNs;     // 3.0e9
        uint64_t validateTimeoutNs;  // 0.1e9
        uint64_t saveTimeoutNs;      // 1.0e9

        int maxAlignAttempts;        // 8
        int maxSolveAttempts;        // 2
        int maxValidateAttempts;     // 2
        int maxSelectAttempts;       // 3
        int maxCaptureAttempts;      // 3
        int maxSaveAttempts;         // 3

        int maxRollbackTotal;        // 4，单次任务总回退次数上限
        int maxRollbackPerEdge;      // 2，同一回退边次数上限

        // ---- E 项 predictor 系数（ENG-10 §3.2/§3.5）----
        // σ_px_est = a + b·(1/S) + c·(1/C)
        double sigmaA;
        double sigmaB;
        double sigmaC;
        double sigmaPxFallback;      // 未标定时的保守值，默认 0.5 pixel

        // ---- 评分归一化参考值（ENG-10 §3.6）----
        double nRef;                 // F 归一化参考点数，默认 100（SYS-15 §4.5 校核值）

        // ---- 历史匹配成功率分桶（ENG-10 §4.2，边界冻结）----
        int    matchStatsMinSamples; // N_min，默认 10
        double matchStatsColdStartPrior;  // 空表先验，默认 0.5
        double distanceBandEdges[3]; // {80, 150, 220} m
        double illumBandEdges[2];    // exposure 分位边界，须与实际曝光设置一同标定
    };

**裁决 C-20：** 上述重试/超时字段为新增，**唯一数据源**。数值不得在代码中重复定义——`RetryManager`（SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕）是唯一读取者，各状态（含 `AlignmentController`、`StateMachine`）不得自持计数器或超时常量。

**`taskTimeoutNs` 必须可注入：** SYS-08 §10 的收敛性测试需将 `T_task` 设为短值（如 2 s），故该字段不得以编译期常量形式硬编码。

### 6.6 `ValidationConfig`

    struct ValidationConfig
    {
        double maxReprojectionError;  // pixel
        double minInlierRatio;        // [0,1]
        double minConfidence;         // [0,1]
        double yawMin;                // deg
        double yawMax;                // deg
    };

### 6.7 `SystemConfig`

    struct SystemConfig
    {
        std::string logDir;
        std::string outputDir;
        std::string modelDir;
        int         logLevel;
    };

**裁决（C-16/C-17）：** 原配置文件清单不一致（`system.yaml` 仅在 ENG-01 §3.2 出现；SYS-05 §13.1 的 `CameraConfig` 与 `CameraChannel` 字段重叠）。冻结：

- 配置文件清单以 ENG-01 §3.2 为准（含 `system.yaml`），**共 7 个 yaml**；
- `CameraConfig` = **运行时可调**参数（曝光、增益、触发模式）；`CameraChannel` = **静态描述 + 标定**（相机 ID、焦段角色、焦距、标定）。两者职责不重叠，均保留。

### 6.8 配置版本追踪（新增要求）

`measurement_xxx/` 必须保存 `config_snapshot/`，包含本次测量实际生效的全部 yaml。

**理由：** SYS-05 §16 要求"支持离线复现"，但验证阈值（`ValidationConfig`）直接决定"结果是否有效"。若回放时不带当时的阈值，同一批图像在不同版本下会得出不同判定，复现承诺无法兑现。SYS-12 §17 与 SYS-15 §16 只要求保存 `calibration_id` / `model_id`，不足以复现判定。

---

## 7 冲突裁决汇总表

| 编号 | 冲突 | 涉及文档 | 冻结结论 |
|---|---|---|---|
| C-01 | `ImageFrame` 字段三处不一致 | SYS-05 §5.1 / ENG-02 §4.1 / ENG-04 §4.1 | 取并集 + 补 `deviceTimestampNs`，见 §5.5 |
| C-02 | `CameraCalibration` 缺 `imageWidth/Height` | SYS-05 §4.2 / SYS-11 §4.1 | 以 SYS-05 为准，见 §5.2 |
| C-03 | `CameraChannel` 缺 `enabled` | SYS-05 §4.4 / ENG-02 §5.3 | 以 SYS-05 为准，见 §5.4 |
| C-04 | `TurntableState` 同名不同义（struct vs enum） | SYS-05 §6.1 / SYS-10 §10 | 结构体保名，枚举更名 `TurntableMotionState`，见 §5.9 |
| C-05 | `ITurntableController` 方法名 `state()` vs `getState()` | SYS-06 §10.1 / SYS-10 §5.1 | 以 SYS-06 的 `state()` 为准 |
| C-06 | 变换方向语义未定义，姿态字段名与坐标链相反 | SYS-05 §11.1/§11.2 / SYS-01 §5.2 | 保留 SYS-13 链方向，改字段名为 `aircraftToCamera` / `aircraftToShip`，见 §2.1 |
| C-07 | `CameraRole` 归属致 data/algorithm 无法编译 | SYS-05 §4.1 / ENG-01 §6 / ENG-02 §5.2 / ENG-03 §2.2 | 下沉 `src/data/`，见 §3.2 |
| C-08 | `TargetModel` 两套互斥建模 | SYS-05 §10.1 / SYS-12 §5.2 §9 | 以 SYS-12 为准，见 §5.20 |
| C-09 | 5 个类型被引用但未定义 | SYS-06 §15 / SYS-07 §8 / SYS-12 §4.2 / ENG-01 §5 | `TrackingResult`、`FeatureDatabase`、`PoseResult` **删除**；`FeatureSet`、`ErrorInfo` **补定义** |
| C-10 | 角度 / 长度单位从未定义 | 全文档 | 角度 = 度，长度 = 米，见 §2.2 / §2.3 |
| C-11 | `TargetOffset` 参考原点未定义 | SYS-05 §6.3 / SYS-10 §7 | 相对图像中心，见 §2.4 |
| C-12 | `ImageQuality` 字段不足以算出 Q | SYS-05 §9.1 / SYS-07 §6.2 / SYS-14 §7 | 补 `exposure` / `contrast`，见 §5.15 |
| C-13 | `MeasurementState` / `DeviceState` 归属未定 | SYS-05 §12 / SYS-06 §14 / SYS-08 §4 | 均置于 `src/data/`，见 §4.1 |
| C-14 | 评分式 E 项无载体字段 | SYS-07 §6.2 / SYS-05 §9.2 | 补 `MeasurementCandidate.predictedError`，见 §5.16 |
| C-15 | `MeasurementController` 接口仅 SYS-03 给出 | SYS-03 §4.1 / ENG-02 §10.1 / ENG-04 §10.1 | 以 SYS-03 为准：`startMeasurement()` / `stopMeasurement()` / `state()` |
| C-16 | 配置文件清单不一致 | SYS-02 §7 / SYS-06 §17 / ENG-01 §3.2 | 以 ENG-01 为准（含 `system.yaml`），见 §6 |
| C-17 | `CameraConfig` 与 `CameraChannel` 字段重叠 | SYS-05 §13.1 / §4.4 | 拆分职责，均保留，见 §6.7 |
| C-18 | `OpticalRig` 层级归属矛盾 | SYS-06 §4 / ENG-01 §6 / ENG-03 §2.2 | 归属 optical 层，见 §3.3 |
| C-19 | `ShipFrame` 冗余 | SYS-05 §3.3 / §4.3 | 删除，见 §4.1 |
| C-20 | 对准最大迭代次数有两处定义，且状态机全部恢复路径无次数/超时上限 | SYS-10 §14 / ENG-09 §6.3 / SYS-08 §7 / ENG-01 §9 | `TurntableConfig::maxIterations` **删除**；唯一数据源 = `MeasurementConfig::maxAlignAttempts`；重试与超时策略冻结于 SYS-08 §7，见 §6.3 / §6.5 / §9.1 |
| C-21 | 算法链四处未闭环：CAD 结构点无定位方法；评分 E 项循环依赖（SYS-07 §6.2 用 PnP 输出的 `ReprojectionError` 作为选前指标）；历史成功率无存储；配置注入路径不完整 | SYS-07 §6.2 §8.2 / SYS-12 §6.1 §6.2 / SYS-14 §7.4 §8 §10 / SYS-04 §4.2 | 全部冻结于 **ENG-10**；§5.16 `MeasurementCandidate` 增 9 字段，`predictedError` 单位改角分；§6.5 `MeasurementConfig` 增 predictor 与分桶参数 |

### 7.1 `TrackingResult` 删除说明

SYS-06 §13 与 SYS-10 §6.1 的数据流为：

    TrackingResult → TargetOffset → AlignmentController → TurntableCommand

但 SYS-08 §5.4 的 ALIGN 状态数据流为：

    DetectionResult → TargetOffset → AlignmentController → TurntableCommand

`DetectionResult` 已完整覆盖对准所需信息（目标框中心 → 像素偏差）。`TrackingResult` 引入了从未定义的跟踪概念，而 V2.1 是**单次测量**架构（SYS-01 §1），不存在跨帧跟踪目标。冻结为删除，统一以 `DetectionResult` 为唯一输入。

### 7.2 `FeatureDatabase` 删除说明

SYS-12 §4.2 声明 `TargetModelManager::getFeatureDatabase()` 返回 `FeatureDatabase`，但该类型从未定义。且三焦段特征库已由 `feature25.bin` / `feature50.bin` / `feature100.bin` 物理分离。

冻结接口为：

    class TargetModelManager
    {
    public:
        bool        load(const std::string& modelId);
        TargetModel get(CameraRole role) const;
    };

按 `CameraRole` 直接索引 `TargetModel`，无需额外容器类型。

---

## 8 变更流程

1. 任何跨模块类型的字段增删改、枚举值变更、单位变更，**先提 PR 改本文档**；
2. 本文档变更后，同步更新受影响的设计文档与代码；
3. 类型变更必须同步更新 `tests/unit/` 中的序列化测试；
4. 冻结基线随版本号发布，见 ENG-07 §12 的五元组基线冻结。

---

## 9 遗留待办

以下问题在本表范围内**无法裁决**，需专题设计，已在对应任务中跟进：

| 待办 | 说明 | 归属 |
|---|---|---|
| ~~`predictedError` 计算方法~~ | ~~评分式 E 项的数据来源未定义；需解决"选择阶段拿不到 PnP 残差"的循环依赖~~ | **已冻结**，见 §9.2 |
| ~~历史匹配成功率的存储与生命周期~~ | ~~SYS-14 §7.4 要求按焦段/距离段/光照统计，但未设计存储载体与跨任务持久化~~ | **已冻结**，见 §9.2 |
| ~~CAD 结构点的图像自动定位方法~~ | ~~SYS-12 §6.1 以 CAD 结构点为主力，SYS-07 §8 用 SIFT，两者未对接~~ | **已冻结**，见 §9.2 |
| ~~误差预算合成规则~~ | ~~SYS-15 §4 未说明 RSS 还是线性合成~~ | **已冻结**，见 §9.1 |
| ~~重试与超时策略~~ | ~~状态机全部恢复路径无最大重试次数与总超时~~ | **已冻结**，见 §9.1 |
| 评分权重 `w1~w4` 的具体数值 | 归一化方式已冻结（ENG-10 §3.6），但取值需由 Golden 数据整定 | 算法专题（ENG-10 §3.5 已定顺序：**E 标定先于权重整定**） |
| `ValidationConfig` 阈值取值 | `E_ref` 依赖该取值 | 算法专题 |
| CAD 模型结构点坐标的实机核对 | ENG-10 §6 提出输入要求，需实物验证 | 硬件 + 算法 |

### 9.1 已结项待办的回填说明

**误差预算合成规则（已冻结）：** 冻结于 **SYS-15 §4**——系统误差线性相加、随机误差 RSS 合成，总误差 = Σ(系统) + √(Σ(随机²)) = 0.97 角分（余量 3%）。同时新增大气扰动项（SYS-15 §13.4）与特征定位定量校核（SYS-15 §4.5）。

~~**重试与超时策略（已冻结）：** 冻结于 **SYS-08 §7**——三级超时体系（§7.1）、失败三分类（§7.2）、各状态次数与超时表（§7.3）、回退预算（§7.4）、硬件降级（§7.5）、`RetryManager` 接口（§7.6）。~~
⇒ **本条来源自 2026-09-26 起作废，依据待裁决（Q-D2）**：`SYS-08 V2.1` 的 `# 7` 是 **TARGET_FOUND 状态**
（§7.1 状态说明／§7.2 执行动作／§7.3 输出／§7.4 失败处理），与上列内容**完全不同**（**节号碰撞**）；
`§7.5`／`§7.6`／`§7.7` **不存在**（全文只到 `# 22`）。上述策略的**现行落点在代码**（`RetryManager`、
`MeasurementStrategy`、`MeasurementConfig` 的 `*_timeout_ns`），其效力来源是
**实施期自造的工程约定 + 既有测试**，**不是**冻结文档。逐条勘误与四类处置见工程根目录的
《SYS-08-§7引用勘误.md》；⚠ **不声称"旧版另有条文"** —— `项目文档/` 内 `SYS-08` 只有这一份 V2.1，
`archive/` 下没有旧版，被引用的那套内容在冻结文档里**从来没有过**。
⚠ 另有一条**独立**的冲突（现行"2 路可用 ⇒ 降级继续"与 `SYS-08 §17.1`"断连／无帧／超时一律 FAILED"相反）
登记为 **Q-D1**，**不得标为已批准**；本批**不改任何等待／重试／降级决策**。

由此产生的本表变更：

- 新增裁决 **C-20**：`TurntableConfig::maxIterations` 删除，唯一数据源改为 `MeasurementConfig::maxAlignAttempts`；
- `MeasurementConfig` 新增 16 个重试/超时字段（§6.5）；
- `ErrorInfo` 新增 10 个已占用错误码（§5.27）。

### 9.2 算法链三项待办的结项说明（ENG-10）

**`predictedError` 计算方法（已冻结）：** 冻结于 **ENG-10 §3.2**。核心是把 E 改写为**可预测量**的表达式：

    E = σ_px_est · sqrt(12) / ( W · sqrt(N_est) ) × (180/π×60)     // 角分

⚠ **V2.2 修正（R07）：** V2.1 此式**漏了末尾的换算因子** —— `σ_px_est/W` 是比值（同为
pixel），故前半段是**弧度**，乘 `180/π×60 ≈ 3437.7468` 之后才是角分。详见 §5.16 的 V2.2 修正。

`W`、`N_est`、`σ_px_est` 三项均可在 PnP 之前获得或预测，**循环依赖由此消除**。同时：

- **E 的单位为角分**（原 §5.16 注释误写为 pixel；但 C-14 只改标签未改算式，
  **换算因子于 V2.2 才补入**，见 §5.16）；
- §5.16 的 `MeasurementCandidate` 新增 9 个字段（四分项、E 的三个输入、标定标记）；
- **四分量必须先归一化再加权**（ENG-10 §3.6），否则权重失去意义；
- **标定必须先于 `w1~w4` 整定**（ENG-10 §3.5）。

**历史匹配成功率存储（已冻结）：** 冻结于 **ENG-10 §4**——载体 `runtime/match_stats.yaml`，按 `target_model_id` 分组，36 桶（焦段 3 × 距离段 4 × 光照 3），冷启动用**收缩估计器**，**任务内只读、结束后写回**。§6.5 相应新增 4 个分桶参数。

**CAD 结构点图像定位方法（已冻结）：** 冻结于 **ENG-10 §2.3**——"投影—匹配—拟合三步法"，并确立**特征来源分层**：B 类（CAD 结构点）保展布 `W`、A 类（自然纹理）保数量 `N`，**二者互补**。原 SYS-12 §6.1/§6.2 的"CAD 优先、纹理补充"排序**作废**。

由此产生的本表变更：新增裁决 **C-21**（见 §7）。

---

## 10 设计约束

必须满足：

1. 本文档是类型与命名的唯一权威；
2. 所有跨模块类型置于 `src/data/`，`data` 不依赖任何其他模块；
3. 角度 = 度，长度 = 米，时间 = 主机单调时钟纳秒；
4. 变换命名方向唯一：`AToB` 表示 A→B；
5. 不存在未定义类型被引用；
6. 不存在同名类型异义；
7. 配置随测量结果一同保存，保证判定可复现；
8. **同一语义的数值只允许存在一处定义**，其余位置一律引用（C-20 即为该原则的一例，代码中的重试计数与超时常量同样受此约束）；
9. **不得在代码中使用裸错误码字面量**，一律引用 §5.27 的具名常量。

---

## 11 后续关联文档

关联：

- SYS-05 数据结构设计（类型定义职能由本文档取代）；
- SYS-06 Device 设备抽象层设计；
- SYS-10 转台控制与目标对准设计；
- SYS-11 光机刚体标定设计；
- SYS-12 目标模型与特征库设计；
- SYS-13 坐标系设计；
- ENG-01 工程目录结构设计；
- ENG-02 C++ 类与文件规划；
- ENG-04 核心类设计。

---

## 12 修订记录

### 12.1 本版变动说明

V2.5 是 **011-A1 清理责任与异常安全批**（裁决 C-01 v1.9）的类型与语义侧落地，**共四类改动**。
⚠ **V2.4 的说明原文保留在本节末尾**（"**V2.4（历史）**"一段开始），V2.3 的说明在其后
（"**V2.3（历史）**"）—— §12.3 按节号引用了它们的内容，删除会让条目悬空。
更早的 V2.2 修订记录见 `archive/ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md` 的 §12。

**一、废除"本地判定 ⇒ `sdkError` 必须为 `nullopt`"，改为"判定类别与实际调用历史分别记录"（§5.29）**

这条判据自 V2.3 起就在，V2.4 只是**照抄**了它（"本地契约错误不得冒充 SDK 错误"）——
而它与 §5.31 的**触发读回不一致**（读回调用**成功**、不一致是本地比对得出的 ⇒
`ContractViolation`）**直接矛盾**：文档要求 `nullopt`，代码里如实带着
`{ImvGetEnumFeatureSymbol, IMV_OK}`。本版按**语义**检索补齐全部同类处：
`OpenStatus` 的枚举注释（`InvalidArgument, ContractViolation, NotImplemented` 后的
"本地判定，未调用 SDK"）、`Unset` 通道的"出现即断言"、`Timeout`／`NotImplemented` 的
来源列、1006 错误码的"来源"列（"**本地判定，未调用 SDK**"）。
统一原则：**判定类别是"结论从哪里来"，调用历史是"此前发生了哪几次调用"，两者分别记录。**
新增 `data::kCallThrewCode`（`INT32_MIN`，**标记值、不是码**）：`kCallThrewCode` 表示
"调用抛出了异常、没拿到返回码"，**显示必须走专用措辞**，**不得**把 `-2147483648` 当原码念。

**二、清理责任两条（§5.29 `sdkError`／`cleanupError` 表与"清理责任与异常安全"）**

① `cleanupError` 是"**资源还回去了没有**"的**唯一**字段：**主操作成功时的清理失败也进它**
（旧形态只在"已有主失败"那行写）⇒ 同一个"释放未获确认"在"帧检查通过"与"帧已损坏"
两条路径上会拿到**相反**的通道处置，区别只在于"帧本身好不好"。
② **释放未获确认 ⇒ 停止该路后续采集**（不只是记一笔）：未归还的缓冲会被 SDK 内部缓存
复用、污染后续帧。⚠ 措辞只谈**资源**、**不改写首因**、**不盲目重试**、
**自动恢复须先有资源恢复依据**（本版不做）。

**三、异常出口冻结为三步各自独立保护（§5.29）**

① 描述原异常、② 显式清理、③ 写诊断 —— 三步**各自**关在保护里，
**每步内部的失败只损失那一步的产物**，末尾的 `throw;` **无条件执行** ⇒ 原异常**必定原样上抛**。
`FrameLeaseGuard::cleanup()` **`noexcept`**；释放调用本身抛出时如实返回
`{ImvReleaseFrame, kCallThrewCode}` 且**不重试**（那次调用是否已把缓冲还回去**无法判断**，
重试就是"一个帧释放两次"）。⚠ 兜底析构的**可达性**不得用"变异全绿"来论证
（见 §12.3 第 5 条的登记）。

**四、结果包键集按数据来源分组（§5.28）**

`raw_image`／`sdk_payload_bytes`／`expected_compact_bytes`／`compact_size_matches`／
`sdk_pixel_format_code`／`sdk_padding_x`／`sdk_padding_y` **只在真身路径写出**；
回落路径只写**这份文件自己**的描述（取自已校验的 `captureFormat` 与实际写出的 `image`）。
理由与 V2.4 给 `raw_image` 的理由**完全同构**：**"不存在的键"胜过"值不对的键"**。

**五、权威指针同批同步（本版补做，语义未动）**

本批 SYS-04 升 **V2.6**、SYS-05 升 **V2.4**、SYS-06 升 **V2.5**、SYS-17 升 **V1.3**，
本文档**正文**里指向它们的版本号随之更新，实测 **2 处**：
§1 的 V2.5 变更块中"接口签名在 … **SYS-06 V2.5 §5.1**"（原 V2.4）、
§6.1 第 4 条"`camera.yaml` 的键表归 **SYS-17 V1.3** 管"（原 V1.2）。
⚠ §1 的 **V2.4（历史）** 变更块里"**当时**的 SYS-04 V2.5 … 与 SYS-06 V2.4 §5.1"**保留不动** ——
那是 V2.4 当年的指向，属**历史事实**（口径同上：**正文的现时指针改、历史清单的字不改**）。

**不修改的内容（明确记录）：**

- **错误码表（§5.27）未改**：本版**没有新增码**；1006 只改了"来源"列的一处措辞。
- **§5.28 第 1～6 条未改**（合法组合表、`rawPolicy` 交叉一致、紧凑布局契约、
  字节序声明、所有权、显示转换规则全部照旧）；V2.4 的四方一致判据**照旧**。
- **V2.4 的四条结论逐条保留**：帧状态只推出 `CorruptFrame`、触发读回三分支、
  `raw_image` 与四方互证、`kNoDeadlineNs` 与三个预算检查点。
- **§12.3 的登记项一条都未关闭**：Q-D1、Q-D2、`SYS-15 §4.5` 悬空引用、
  `IPosePipeline::validate()` 无失败通道。R09 仍为"**部分修复**"。

---

**V2.4（历史）**：V2.4 是 **011-A1 九项缺口定向修复批**（裁决 C-01 v1.8）的类型与语义侧落地，**共五类改动**。
⚠ **V2.3 的说明原文保留在本节末尾**（"**V2.3（历史）**"一段开始）——
因为 §12.3 第 1 条按节号引用了它的内容，删除会让该条目悬空。
更早的 V2.2 修订记录见 `archive/ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md` 的 §12。

**一、`GrabResult` 增两个诊断字段与一条固定的传递路径（§5.29 修订二）**

`frameStatusRaw`（`optional`，**"未取得"与"正常值 0"必须可区分**）与
`diagnosis`（本次现场数值，成功时为空）。路径固定为
`SDK 帧视图 → GrabResult → ChannelGrabRecord → 日志／失败记录`；
**上层不得改读后端的 `lastErrorText()`**（那是"最近一次失败文本"，可能陈旧）。

**二、帧状态参与有效性判断，且只推出 `CorruptFrame`（§5.29 修订一）**

`IMV_FrameInfo::status != 0` ⇒ `CorruptFrame`（`1005`），文本为
"**SDK 调用成功，但帧不满足可交付条件**"；`sdkError` 保持 `{ImvGetFrame, IMV_OK}`、
原值进 `frameStatusRaw`。⚠ **删除** V2.3 的"任意非零帧状态即可推出 `NoFrame`"表述 ——
该状态的**位含义未经文档化**，本批**不解释**它。`NoFrame` **保留但当前无产出路径**。
另：**"后端报 `Ok` 却交付空图" ⇒ `ContractViolation`**，且**调用诊断原样转发、
不补造 `{ImvGetFrame, 0}`**（管理器并不拥有"我们调用过 SDK"这个事实）。

**三、触发读回逐项化，并给出一张三分支的处置表（§5.31 补充）**

`TriggerModeState` 增 `readbacks[3]`（`TriggerFeatureReadback`：特性名／
`callAttempted`／`failure`／`returnedEmpty`／`value`）。**删除**"`reported` 为空 ⇒ 用
`requested`"的三元表达式：读回不完整 ⇒ `NotStarted`／`1004`（"本轮无法确认触发前置条件"）、
读回完整但不一致 ⇒ `ContractViolation`／`1006`（记录**请求值与实际值**），
**两种情形都不发令、不取帧、也不永久禁用通道**。

**四、结果包增 `raw_image`，落盘复检改为四个长度事实互证（§5.28 第 7bis／8 条）**

`raw_image:{width, height}`（裸缓冲**自己的**宽高，**仅真身路径写出**）——
V2.3 的包里唯一的宽高取自显示图（加工产物），"有载荷、无显示图"的包因此**无法独立恢复**。
落盘复检改为 `重算 == expected_compact_bytes == sdk_payload_bytes == 载体长度`
四者互证，**`compactSizeMatches` 与这四个事实矛盾时同样拒绝**；
回落分支的元数据**取自实际写出的图**（不得照抄 `raw` 的默认字段）。

**五、"无期限"与"已到期"分开，且每轮只清旧、保留本次取证（§2.5 补充／§5.32 补充）**

`data::kNoDeadlineNs = UINT64_MAX`（与 `RetryManager::kNoDeadline` **同值同义**）不参与取小；
除哨兵外 `deadline ≤ now` 一律"已到期"、剩余记 **0**。控制器的 `deadline == 0` 哨兵
**删除**（它让"已到期"掉进缝里并被**重新注入预算**；⚠ 该分支在当前控制器里**可达**、是缺陷本体，
已由用例 `A1_44` ＋ 变异 M18 实测复现；**只有**"无期限被算成已到期"这一方向仍未实测
—— 见 §2.5 第 3 条的实测登记）。
`ChannelGrabRecord` 增 `diagnosis`／`frameStatusRaw`／`readbacks`，
并冻结**三个预算检查点**（入口／发令前／发令后取帧前）各自的措辞与
`triggerCalls`／`grabCalls` 取值。

**不修改的内容（明确记录）：**

- **错误码表（§5.27）未改**：本版没有新增码，`1005`／`1006` 的分工与 V2.3 一致
  （`CorruptFrame` 与"空图契约违背"都落在既有码上）。
- **`capturedCount` 的判据未改**（§5.32 第 1 条）：`status == Ok` **且**图合法非空 ——
  交付判据不依赖"某个实现当前恰好让两者等价"。
- **§5.28 第 1～6 条未改**（合法组合表、`rawPolicy` 交叉一致、紧凑布局契约、
  字节序声明、所有权、显示转换规则全部照旧）。
- **§12.3 的四条登记项一条都未关闭**：本批**不碰** Q-D1（"2 路可用 ⇒ 降级继续"）、
  Q-D2（`SYS-08 §7.x` 引用失效与时限零余量）、`SYS-15 §4.5` 悬空引用、
  `IPosePipeline::validate()` 无失败通道。R09 仍为"**部分修复**"。

---

**V2.3（历史）**：V2.3 是 **011-A1 数据契约批**（裁决 C-015）的类型侧落地，**共四类改动**。
V2.2 的修订记录见 `archive/ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md` 的 §12。

**一、新增类型（6 个新小节）**

| 小节 | 内容 | 配套 |
|---|---|---|
| §5.28 | `RawImagePayload` 及 `PixelFormat`／`Packing`／`ByteOrder`／`BitAlignment`／`RawDataPolicy` | 合法组合表、12 键元数据、Recorder 三分支 |
| §5.29 | 取帧结果与诊断出口：`OpStatus`／`SdkCall`／`SdkFailure`／`OperationResult`／`GrabResult` | 状态全表、错误码映射表、聚合规则 |
| §5.30 | `DeviceIdentity` | 三条归属纪律（不进 `ImageFrame`、序列号匹配、后端类型另栏） |
| §5.31 | `CameraTriggerMode`／`TriggerModeState` | 三值枚举、三项读回、GenICam 特性字符串映射 |
| §5.32 | `CaptureRound`／`ChannelGrabRecord` | `capturedCount` 判据、聚合含"未尝试"通道 |
| §5.33 | `DeviceState` 增 `DISCONNECTED` | 仅"设备不在"时置位、措辞纪律 |

**二、既有定义的字段/枚举变更**

1. **§5.5 `ImageFrame` 增三个字段**（`raw`／`captureFormat`／`rawPolicy`），另七个字段语义未改。
2. **§4.1 的 `DeviceState` 增一个值**（`DISCONNECTED`，语义见 §5.33）。
3. **§6.1 `CameraConfig` 增 `serialNumber`／`backend`；`triggerMode` 由 `bool` 改三值枚举**。
   ⚠ 同时**更正**本文档 V2.2 及更早代码注释中"cameraId 是设备序列号（华睿 A7A20MU201 的唯一标识）"
   的说法 —— 它把**型号**当成了唯一标识：型号是**设备类型**、序列号才是**本台**的身份。
4. **§2.5 补一条时间戳来源纪律**：`timestampNs` 由**主机**生成、`deviceTimestampNs` 由**设备**生成，
   两者不可互相替代、不可比较。

**三、错误码表（§5.27）的两处改动**

1. **新增 5 个码**：`1003`（取帧超时或无有效帧）／`1004`（通道未就绪）／
   `1005`（SDK 错误或帧不可用）／`1006`（取帧契约或参数错误，**本地判定**）／
   `1007`（格式/模式本批未实现）。号段已核实**未占用**（现用 1001／1002、2001-2003、
   3001-3002、4001、5001、6001、9001-9005）。
   ⚠ **`1005` 与 `1006` 不得混用**：前者是"相机给的帧有问题"，后者是"我们自己的代码或配置有问题"，
   现场动作完全不同。
2. ⚠ **回填 5 个已落地但未登记的错误码**：`3002`／`4001`／`5001`／`9004`／`9005`
   由**裁决 C-01 v1.3**（2026-09-23）新增、代码中已全部落地，但**当时未回填本表**。
   本版**按代码实际值回填**（含义照录代码注释）。⚠ 这是**补登记，不是新增** ——
   语义在 C-01 v1.3 里已经定过；**不追溯**该流程偏离的责任归属。
3. **改写全部 `SYS-08 §7.x` 来源列**（见下条）。

**四、引用勘误：`SYS-08 §7.x` 全族失效**

本表 §5.27 的"来源"列与 §9.2 的"重试与超时策略（已冻结）"一条，原以 `SYS-08 §7.1`～`§7.7`
为依据。2026-09-26 逐条复核（`SYS-08 V2.1` 原文）：

- `# 7` 是 **TARGET_FOUND 状态**（§7.1 状态说明／§7.2 执行动作／§7.3 输出／§7.4 失败处理）
  ⇒ 与被引用的"§7.1 三级超时／§7.2 失败三分类／§7.3 次数表／§7.4 回退预算"**内容完全不同**
  （**节号碰撞**）；
- `§7.5`／`§7.6`／`§7.7` **不存在** —— 全文只到 `# 22`，且每个 `# N` 下最多到 `.4`；
- ⚠ **`项目文档/` 内 `SYS-08` 只有这一份 V2.1，`archive/` 下没有旧版**
  ⇒ 本表**不声称"旧版另有条文"**：被引用的那套内容在冻结文档里**从来没有过**，
  其真实来源是**实施期自造的工程约定**（现行落点在代码：`RetryManager`／`MeasurementStrategy`／
  `MeasurementConfig` 的 `*_timeout_ns`）。

处置：**各码的"含义"与"号段"一字未改、代码无需改动**；只把失效的"来源"划掉并就地标注
「引用无效，依据待裁决（Q-D2）」。逐条勘误见工程根目录的《SYS-08-§7引用勘误.md》；
登记见《待裁决问题汇总》**Q-D2**。⚠ 另有一条**独立**的冲突登记为 **Q-D1**（现行"2 路可用 ⇒
降级继续"与 `SYS-08 §17.1`"断连／无帧／超时**一律 FAILED**"相反），**不得标为已批准**。

**五、新增附录（两条 SDK 证据）**

- **§13 附录 A**：SDK 事实核验结论（头文件逐项）；
- **§14 附录 B**：SDK 自带样例核查结果（含**一条被更正的结论**，留痕在其中）。

⚠ **两条证据必须成对引用**：附录 A 里曾有一条"**不存在连接事件订阅 API**"的结论，
**被附录 B 的样例核查推翻**（`IMV_SubscribeConnectArg` 确实存在）—— 原结论是**检索遗漏**所致。
故凡引用附录 A 的行，都必须同时看附录 B 是否更正了它。

**不修改的内容（明确记录）：**

- **§5.16／§9.2 的 E 单位结论未改** —— V2.2 的换算因子修正**依然有效**，本版未回退。
- **§5.16 的字段表、§1.1 权威性规则未改**，包括第 3 条"其 §3~§12 的类型定义降级为历史版本"。
  ⚠ SYS-05 已同步升为 **V2.2**（补入 `ImageFrame` 的三个新字段），但它**仍属"历史版本"** ——
  类型与字段的**唯一权威仍是本表**（本次配套同步只是为了让 SYS-05 不与本表**明显矛盾**）。
- 全文其余引用 `SYS-15 §4.5` 之处**原样保留** —— 该节号是**已登记的悬空引用**，
  与 IF-SW-02 同类。本版**不做"顺手修掉"**：那会静默改变一个未决项、破坏登记纪律。

> **为什么新增附录放在 §12 之后（而不是插在 §12 之前）：** 本目录的"增量修改"纪律要求
> **先重编号、后插入**。把附录插在 §12 之前就必须把"修订记录"由 §12 改号，
> 而"修订记录"是其他文档最常引用的节之一 —— 重编号会静默使一批既有引用指向别处，
> 正是本次勘误要治的那类缺陷。故两条附录顺延为 §13／§14，**不动任何既有节号**。

### 12.2 版本关系

| 文件 | 状态 |
|---|---|
| `ENG-09_类型与命名冻结表_V2.5_清理责任与异常安全版.md` | **唯一有效版本** |
| `archive/ENG-09_类型与命名冻结表_V2.4_取帧语义与触发读回版.md` | 已归档，仅作历史留存，**不再作为依据** |
| `archive/ENG-09_类型与命名冻结表_V2.3_A1数据契约版.md` | 已归档，仅作历史留存，**不再作为依据** |
| `archive/ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md` | 已归档，仅作历史留存，**不再作为依据** |
| `archive/ENG-09_类型与命名冻结表_V2.1_多相机闭环测量版.md` | 已归档，仅作历史留存，**不再作为依据** |

⚠ 引用 ENG-09 时一律指向 **V2.5**。V2.1 的 §5.16 / §9.2 算式**有错**（缺换算因子）；
V2.2 的 §5.27"来源"列与 §9.2 的一条策略**依据失效**（见 §12.1 的 V2.3 段第四类）；
V2.3 的 §5.29"任意非零帧状态即可推出 `NoFrame`"与 §5.31 的三元读回**已被删除**；
V2.4 的"本地判定 ⇒ `sdkError` 必须为 `nullopt`"与"提升为失败那一行不写 `cleanupError`"
**已被本版废除**。上述各版**均不得**再作引用依据。

⚠ **SYS-04 的配对版本**：本版与 **SYS-04 V2.6** 同批发布、**须成对引用**
（本文档管**类型与字段**，SYS-04 管**接口签名与调用责任**；本版改的清理／异常责任
落在 SYS-04 V2.6 §6.2）。引用"接口唯一权威"时写 **SYS-04 V2.6**、**不是** V2.5。

### 12.3 未关闭的登记项

⚠ **V2.5 一条都未关闭**：本版是语义修订，**不裁决**下列任何一项。逐条状态与 V2.3 完全相同。

1. **`SYS-15 §4.5` 悬空引用 —— 只登记，不修。** 见 §12.1 的说明与 ENG-10 V2.2 §11.3 第 3 条。
   附带登记：SYS-15 的**文件名写 `V1.0`，其 H1 写 `V1.2`**，文件名与版本不自洽。
2. **`MeasurementSelectionResult` 无法区分"未选择"与"得分 0"** ——
   §5.17 该结构体按本表冻结为 `selectedCamera` + `score` 两字段，**无 `valid` 标志**，
   而 `0.0` 是合法得分。当前修复（R06）只保证写入的得分来自真实选择结论，
   **未消除该歧义** —— 消除它要改本表 §5.17 的冻结结构体，需另行裁决。
3. **`IPosePipeline::validate()` 没有表达"执行失败"的通道** ——
   判不合格与无法执行都只能表现为 `out.valid == false`，二者靠 `reason`（C-008）区分。
   R05 已把它们统一为"一律读 `out.valid`"并加了契约对账，但**接口层面仍无该通道**，
   且 IF-SW-02 的签名冻结使增设通道属不兼容变更。
4. **`SYS-08 §7.x` 全族引用失效 + 时限体系零余量（V2.3 登记，**待裁决**）** ——
   本表 §5.27 与 §9.2 的 `SYS-08 §7.x` 来源已就地划掉并标注无效
   （**Q-D2**，逐条勘误见《SYS-08-§7引用勘误.md》）。
   附带问题：`capture_frame_count = 5`、`grab_timeout_ms = 100`、`capture_timeout_ns = 1.5e9`、
   `grab_group_budget_ns = 3.0e8` 这几个值**没有任何冻结依据**，
   且 `5 × 300 ms = 1.5 s` 与 `capture_timeout_ns` **零余量**
   （复制／格式转换／评分耗时未计入）⇒ CAPTURE 在真实相机上**可能被时限截断**。
   ⚠ 本批**不改**这些冻结值，只把事实测出来并登记。
5. **硬件降级策略缺少冻结依据，且与 `SYS-08 §17.1` 冲突（V2.3 登记，**待裁决**）** ——
   现行"2 路可用 ⇒ 降级继续、≤1 路 ⇒ `FAILED`(1001)、禁用故障相机、降级须在 UI 可见"
   这一整套**在冻结文档里查不到出处**（`§7.5` 不存在），而 `SYS-08 §17.1` 明写
   断连／无帧／超时**一律 `FAILED`** ⇒ **正面冲突**。登记为 **Q-D1**，**不得标为已批准**；
   本批记**临时处置**（**既不重连、也不改任何等待／重试／降级决策**）。

---

## 13 附录 A：SDK 事实核验结论（V2.3 新增，冻结）

**核实对象**：`third_party/imvsdk/include/` 的四个头文件（逐项人工核对）。
**核实日期**：2026-09-26。
**标注约定（每条**必须**保留其一，不得在冻结文档里把"推断"或"未文档化"写成 SDK 保证）**：

| 标注 | 含义 |
|---|---|
| **文档支持** | 头文件/常量注释**明确**写明 |
| **样例支持** | 头文件未写，但 SDK 自带样例**实际这么用**（见附录 B）—— 是厂商实跑过的用法，**不是** SDK 保证 |
| **推断** | 从相邻事实推出，**无直接依据** |
| **未文档化** | 头文件**没写**，且样例也**没有**可判别的用法 |

| 事项 | 结论 | 标注 | 依据 |
|---|---|---|---|
| 软件触发入口 | `IMV_ExecuteCommandFeature(h, "TriggerSoftware")` | 文档支持＋样例支持 | `IMVApi.h` 的命令特性接口 + `SoftTrigger` 样例 |
| 触发模式读写 | `IMV_Set/GetEnumFeatureSymbol`，特性名 `TriggerMode`／`TriggerSource` | 文档支持＋样例支持 | `IMVApi.h` 枚举特性接口 |
| 触发模式枚举成员 | **不存在**（无 `TriggerMode_Off` 等）；`"FreeRun"` 一词全树**零出现** | 文档支持（"不存在"由全 `include/` 检索得出） | 全 `include/` 检索 |
| "`TriggerMode = "Off"` 即自由运行" | **仅由样例代码支持，无文档保证** | **样例支持** | `SoftTrigger` 等样例；⇒ §5.31 要求 `consistent` 如实反映实测差异 |
| 硬触发额外要求 | `TriggerSource = "Line1"` **且** `TriggerActivation = "RisingEdge"`；设 `TriggerMode` 前须先设 `TriggerSelector = "FrameStart"` | **样例支持** | `LineTrigger` 等样例 |
| 设备型号／序列号 | `IMV_DeviceInfo.modelName`／`.serialNumber`；**无"序列号是否有效"标志** | 文档支持 | `IMVDefines.h` 的 `IMV_DeviceInfo` |
| 设备列表所有权 | SDK 内部缓存，**无释放接口**（不得 `free`） | 文档支持 | 枚举接口的注释 |
| 帧缓冲所有权 | `IMV_GetFrame` 用 SDK 内部缓存，**必须 `IMV_ReleaseFrame`**；`IMV_CloneFrame` 的克隆体**也用 `IMV_ReleaseFrame`** | 文档支持 | `IMVApi.h` 的取帧/克隆接口 |
| 行步长 | **无 `step`／`stride` 字段** | 文档支持 | `IMV_FrameInfo` 的完整字段表 |
| `paddingX`／`paddingY` 语义 | **未文档化**（"行内填充"还是"整帧填充"没有说明） | **未文档化**＋样例**反向证据** | `IMV_FrameInfo` 无注释；附录 B 第 3 条 |
| 整帧字节数 | `frameInfo.size`（**唯一权威总长**） | 文档支持 | 同上 |
| `Mono12` 的有效位位置 | SDK 头文件**未文档化**（`OCCUPY*BIT` 只说占多少位）；**但格式码逐位等于 PFNC**（`0x01100005` ＝ PFNC `Mono12`）⇒ 有效位对齐**由 PFNC 2.4 §6.1.1／图 6-3 规定为低位对齐、高位补零** | 格式码**实测相符** ⇒ 由标准**推实现**（本批**唯一**一处） | 格式常量 + PFNC 标准 |
| `Mono12Packed` 的位布局 | SDK 头文件**未文档化**（该行无注释）；`0x010C0006` 归 **GigE Vision 2.0** | **未文档化**；本批**按范围**不做 | 格式常量 + 官方格式值表 |
| `IMV_PixelConvert` 的输出 | **仅 8 位**（Mono8/BGR8/RGB8/BGRA8）—— **不能**用来解包 12 位 | 文档支持 | 转换接口的输出格式枚举 |
| `IMV_GetFrame` 的 `timeoutMS = 0` | **未文档化**（只提 `INFINITE`，而 `INFINITE` 宏**未定义**） | **未文档化** | 取帧接口的参数注释 |
| `IMV_GetFrame` 超时返回 `-119` | `IMV_TIMEOUT` 常量**确实存在**；但**该函数自身的注释并未链接它** | **推断**（调用点关联性）＋样例**从不判它** | 常量定义；附录 B 第 4 条 |
| 断开／重连／不可达码 | `-118`（未连接）／`-115`（重连恢复中）／`-114`（取图恢复中）／`-116`（连接不可达） | 文档支持（**字面含义**）；⚠ **"表示断连"是推断**，故 §5.29 只把 `-118` 归 `Disconnected` | 常量注释 |
| 是否已打开 | `IMV_IsOpen` 返回 `bool`，**混淆"关闭"与"掉线"**（两者都返回 false） | 文档支持 | 接口注释 |
| ~~连接事件订阅~~ | ⚠ **本行原写"无（无连接回调 API）"—— 该结论已作废** | —— | **更正依据见附录 B 第 1 条** |

> ⚠ **本附录里"推断"与"未文档化"的每一行，都只允许记"已实现／待实机核实"，不得记"已验证"**：
> `-119` 即超时、`timeoutMS = 0`、`-114`／`-115`／`-116` 的断连语义、
> `paddingX/Y` 的行内 vs 整帧、`"Off"` 即自由运行 —— 逐条在册。
> `Mono12` 的**实现依据是 PFNC 标准**（格式码逐位相符），故记"**已实现**"；
> 实机核实的是"**本台设备的输出符合该约定**"，核实否**只改这一处实现**，不动标准依据。

## 14 附录 B：SDK 自带样例核查结果（V2.3 新增，冻结）

**核实对象**：SDK 自带的 C 样例（逐样例扫）。
**核实日期**：2026-09-26。
**为什么必须与附录 A 成对使用**：样例**不是文档**，**不构成 SDK 保证**；但它是**厂商自己实跑过的用法**，
对"未文档化／不存在"这类结论有**直接判别力**。本次核查**推翻**了附录 A 的一条结论 —— 见下第 1 条。

| 事项 | 结论 | 样例证据 |
|---|---|---|
| **1 连接事件订阅（⚠ 更正附录 A 的一行）** | ⚠ **附录 A 原写"无连接回调 API"—— 那是错的**。SDK **有**连接状态**推送**回调 `IMV_SubscribeConnectArg(h, proc, pUser)`，事件的 `offLine`／`onLine` 取值为枚举成员（`IMVDefines.h` 的 `IMV_SConnectArg`）。另有流事件／消息通道／参数更新三类订阅接口 | `ResumeConnect` 样例注册该回调并处理 `offLine`／`onLine`；FG 侧同构。**原结论是检索遗漏所致** |
| 2 12 位转 8 位不委派 `IMV_PixelConvert` | 本批**不采用**委派路径：它会把"缩放／对齐规则"藏进 SDK 内部，而我们**无法核实其是否缩放** —— 直接冲突于 §5.28 第 6 条的"不缩放、不直方图拉伸、不饱和"。委派路径记为实机核实时的**对照项**（同一帧两法出图比对） | 样例**从不手工解包或移位**，12 位一律交给 `IMV_PixelConvert` —— 但样例只喂 8 位源格式 |
| 3 `paddingX` 语义 | 样例**从不参与寻址**（只原样透传给转换接口）⇒ **仍无法判别行内／整帧**；且有**反向证据**：同一函数里 `Rotate`／`Flip` 按 `width*height*channels`（**不含 padding**）给长度，而 `PixelConvert` 却收 `paddingX`／`paddingY` ⇒ 两个 API 的 padding 约定**不一致** | 图像旋转／镜像／转换三个样例 |
| 4 `IMV_TIMEOUT` 作判据 | **样例零处使用** —— 全部取帧调用只判 `IMV_OK != ret`，超时与其它错误**在样例里不可区分** | 全部取帧样例 |
| 5 `INFINITE` | 样例**零出现**；所有超时实参都是正整数常量，**无 0、无变量、无宏** | 全部取帧调用点 |
| 6 `Mono12`／`Mono12Packed` | 样例**完全不涉及**（`Mono12`／`OCCUPY*BIT`／`packed` 全树 0 命中） | 全样例检索；转换样例只出现 Mono8/BGR8/RGB8/BGRA8 |
| 7 帧缓冲释放 | 样例的写法是"**取到就立刻在同一条执行线上释放**"，**没有**"出错才释放"的写法（无 `goto cleanup`、无延迟释放标志）；克隆体的**确实**用 `IMV_ReleaseFrame` 释放 | 取帧／克隆样例 |
| 8 `TriggerSoftware` | 发令唯一入口 `IMV_ExecuteCommandFeature(h, "TriggerSoftware")`，**6 处样例一致**；读回用 `IMV_GetEnumFeatureSymbol(h, "TriggerMode", …)`；硬触发另需 `TriggerSelector`＝`"FrameStart"`、`TriggerActivation`＝`"RisingEdge"` | 软触发／清缓冲／硬触发／通信属性各样例 |
| 9 取帧失败后的纪律 | 失败分支样例**分两派**：**继续轮询**（可配 `sleep`）或**中止**。本工程取**中止**（与 §5.29 的映射表一致，避免在 GUI 线程上无限轮询） | 两个取帧样例分别代表两派 |

**由样例得出的三条设计更正**（均已并入对应小节）：

1. **"断连没有事件可订阅"这一理由作废**（上表第 1 条）⇒ 本批不做重连的真正理由改为：
   **范围**（Q-D1 策略待裁决）＋**线程模型**（回调在 SDK 自己的线程上执行，
   而 `ICameraBackend` 明确"不负责线程安全、归设备线程独占"，`ICameraBackend.h:49-50`），
   要接回调就必须先建一条**跨线程投递路径**，那属于"完整恢复策略"的一部分。
   ∴ 本批**不注册回调**，但**必须把标签收紧**：断连一律写"**按 SDK 错误码判定，
   未经连接事件确认**"（§5.31／§5.33）。
2. **12 位不委派 SDK 转换**（上表第 2 条）：⚠ 此处**不再**援引"RAW 逐字节一致"——
   那条判据由"保留未经改动的原始副本"达成，**与显示转换方式无关**（§5.28 第 5／6 条）。
3. **`frameInfo.size` 是唯一权威长度这一条因样例的不一致而更强**（上表第 3 条）：
   既然连 SDK 自己的两个 API 对 padding 的约定都不一致，派生步长就更**不能**反过来当校验依据。

> ⚠ **样例对 12 位的沉默不构成"没有依据"**：本批 `Mono12` 的依据是 **PFNC 标准 ＋ 格式码逐位相符**
> （§5.28 第 1 条），**不依赖样例也未经样例否定**；样例只说明"厂商样例从不手工解包"。
> ⚠ **附录 A 里凡被本附录更正的条目，引用时必须两条一起看** —— 单看附录 A 会引用到一个
> 已被推翻的结论（这正是"检索遗漏"这类错源的形态：结论看起来同样斩钉截铁）。
