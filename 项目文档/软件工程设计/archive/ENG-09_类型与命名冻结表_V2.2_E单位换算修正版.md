# AircraftPoseSystem 类型与命名冻结表 V2.2（E单位换算修正版）

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
| `DeviceState.h` | `enum class DeviceState` | `UNKNOWN` / `INIT` / `READY` / `RUNNING` / `ERROR` |
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
    };

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
| 1001 | 可用相机数不足（<2），无法继续 | SYS-08 §7.5 |
| 1002 | 相机断连（已降级） | SYS-08 §7.5 |
| 2001 | 转台通信失败 | SYS-08 §7.7 |
| 2002 | 转台超出行程（能力边界） | SYS-08 §7.7 |
| 2003 | 对准重试次数用尽 | SYS-08 §7.3 / §7.7 |
| 3001 | 触发失效，已降级为软触发 | SYS-08 §7.5 |
| 6001 | PnP 重试次数用尽 | SYS-08 §7.3 |
| 9001 | **单次测量任务超出 `T_task`** | SYS-08 §7.1 |
| 9002 | **回退预算用尽** | SYS-08 §7.4 |
| 9003 | 人工取消 | SYS-08 §7.3（SEARCH 可由人工终止） |

**要求：** 状态机进入 FAILED 时，`ErrorInfo.code` 必须取自本表，**不得使用裸数字字面量**；代码中以具名常量（如 `kErrTaskTimeout = 9001`）引用。

---

## 6 配置结构体（冻结）

### 6.1 `CameraConfig`

    struct CameraConfig
    {
        std::string cameraId;
        CameraRole  role;
        int         width;
        int         height;
        double      exposureTime;   // s
        double      gain;           // dB
        bool        triggerMode;    // true = 硬触发
    };

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

**裁决 C-20：** 原 `maxIterations`（对准最大迭代次数）**删除**。它与 SYS-08 §7.3 的 ALIGN 最大尝试次数是同一语义，保留两个名字会导致两处数值可能不一致（SYS-10 §14 与 ENG-09 各自定义）。冻结为：**唯一数据源是 `MeasurementConfig::maxAlignAttempts`**，`AlignmentController` 向 `RetryManager` 查询剩余次数，不再自持计数器（SYS-08 §7.6 约束 1）。

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

**裁决 C-20：** 上述重试/超时字段为新增，**唯一数据源**。数值不得在代码中重复定义——`RetryManager`（SYS-08 §7.6）是唯一读取者，各状态（含 `AlignmentController`、`StateMachine`）不得自持计数器或超时常量。

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

**重试与超时策略（已冻结）：** 冻结于 **SYS-08 §7**——三级超时体系（§7.1）、失败三分类（§7.2）、各状态次数与超时表（§7.3）、回退预算（§7.4）、硬件降级（§7.5）、`RetryManager` 接口（§7.6）。

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

V2.2 相比 V2.1 **只做一处实质修改**：E（`predictedError`）的算式补回弧度→角分换算因子。

1. **§5.16 单位修正段改写** —— 原文的理由"其计算式 `σ_θ = σ_px·√12/(W·√N)` 的**输出就是角度**"
   **不成立**：`σ_px/W` 是比值（同为 pixel），该式输出**弧度**。已改写为
   "输出为弧度，得角分须再乘 `180/π×60 ≈ 3437.7468`"，并给出两行对照式。
   ⚠ **C-14 的结论（E 的单位是角分）依然正确** —— 错的是它的论证。
   C-14 只改了**标签**，没改**算式**，这才是"标签对、数值错 3437.75 倍"的由来。
2. **§5.16 C-14 裁决文本加注勘误** —— 该处所引 "SYS-05 §9.2" **节号有误**（SYS-05 §9 只有 §9.1；
   `MeasurementCandidate` 实在 §12.1）。按"**历史引用不改号**"的纪律，
   **裁决正文原样保留**，只在其下加注。
3. **§9.2 结论式修正** —— `E = σ_px_est·√12/(W·√N_est)` 末尾补换算因子；
   并把"**E 的单位修正为角分**"一句改为"**E 的单位为角分**"，
   加注"C-14 只改标签未改算式，换算因子于 V2.2 才补入"。§9.2 的其余结论**未改**。

**不修改的内容（明确记录）：**

- **§5.16 的字段表一字未改**（`predictedError`、四个归一化分项、E 的三个输入、
  `predictedErrorCalibrated` 等）—— 缺陷在算式不在字段。
- **§1.1 权威性规则未改**，包括第 3 条"其 §3~§12 的类型定义降级为历史版本"。
- 全文其余引用 `SYS-15 §4.5` 之处（含 §9.1 的"特征定位定量校核（SYS-15 §4.5）"）
  **原样保留** —— 该节号是**已登记的悬空引用**，与 IF-SW-02 同类。
  V2.2 **不做"顺手修掉"**：那会静默改变一个未决项、破坏登记纪律。
  （事实记录：SYS-15 无 §4.5，`# 4 总体误差链` 是纯框图无子节，
  且该模型在 SYS-15 中**整份不存在**；详见 ENG-10 V2.2 §11.3 第 3 条。）

> **为什么一处换算因子要升版而不是就地改：** 本目录约定"一文件一版、版本号进文件名、
> 旧版移入 `archive/` 绝不删除"（项目非 git 仓库，档案即审计链）。
> 就地改会让"实现当初照哪个版本写的"永久不可考 —— 而 R07 恰是
> **实现、注释、文档三者一致地错**的那一类缺陷，最需要留下版本痕迹。

### 12.2 版本关系

| 文件 | 状态 |
|---|---|
| `ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md` | **唯一有效版本** |
| `archive/ENG-09_类型与命名冻结表_V2.1_多相机闭环测量版.md` | 已归档，仅作历史留存，**不再作为依据** |

⚠ 引用 ENG-09 时一律指向 V2.2。V2.1 的 §5.16 / §9.2 算式**有错**（缺换算因子），
**不得**再作引用依据。

### 12.3 未关闭的登记项

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
