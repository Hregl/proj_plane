# AircraftPoseSystem 工程目录结构设计 V2.1（多相机闭环测量版）

## 1 文档目的

本文档定义AircraftPoseSystem V2.1软件工程目录结构。

V2.1相比V2.0增加：

-   OpticalRig光机系统；
-   转台控制模块；
-   硬触发模块；
-   多焦段测量选择；
-   目标模型管理；
-   姿态验证模块。

本文件作为：

-   C++类规划；
-   CMake组织；
-   Git管理；
-   编码实现；

的工程基线。

------------------------------------------------------------------------

# 2 工程总体目录

    AircraftPoseSystem/

    ├── CMakeLists.txt

    ├── cmake/

    ├── docs/

    ├── config/

    ├── models/

    ├── calibration/

    ├── src/

    ├── tests/

    ├── deploy/

    └── scripts/

------------------------------------------------------------------------

# 3 根目录说明

## 3.1 docs

保存设计文档。

    docs/

    ├── system/

    ├── interface/

    ├── algorithm/

    ├── engineering/

    ├── test/

    └── deployment/

------------------------------------------------------------------------

## 3.2 config

运行配置。

    config/

    ├── system.yaml

    ├── camera.yaml

    ├── optical_rig.yaml

    ├── turntable.yaml

    ├── trigger.yaml

    ├── measurement.yaml

    └── validation.yaml

------------------------------------------------------------------------

## 3.3 models

目标模型和算法模型。

    models/

    ├── aircraft/

    │
    ├── points3d.yaml

    ├── feature25.bin

    ├── feature50.bin

    └── feature100.bin


    ├── yolo/

    └── runtime/

------------------------------------------------------------------------

## 3.4 calibration

标定数据。

    calibration/

    ├── cam25.yaml

    ├── cam50.yaml

    ├── cam100.yaml

    ├── cam25_to_rig.yaml

    ├── cam50_to_rig.yaml

    ├── cam100_to_rig.yaml

    └── rig_to_ship.yaml

------------------------------------------------------------------------

# 4 src源码目录

    src/

    ├── app/

    ├── ui/

    ├── application/

    ├── preview/

    ├── optical/

    ├── device/

    ├── algorithm/

    ├── infrastructure/

    ├── runtime/

    └── data/

------------------------------------------------------------------------

# 5 data模块

路径：

    src/data/

职责：

公共数据结构。

包含：

    ImageFrame.h

    MultiCameraFrame.h

    PreviewFrame.h

    ShipPoseResult.h

    Transform.h

    TurntableCommand.h

    TurntableState.h

    TurntableMotionState.h

    MeasurementCandidate.h

    MeasurementState.h

    DeviceState.h

    CameraRole.h

    CoordinateFrame.h

    CameraCalibration.h

    OpticalRigCalibration.h

    CameraChannel.h

    TargetOffset.h

    DetectionResult.h

    AlignmentResult.h

    TargetScaleEstimate.h

    ImageQuality.h

    MeasurementSelectionResult.h

    ModelPoint3D.h

    FeatureDescriptor.h

    TargetModel.h

    FeatureSet.h

    FeatureCorrespondence.h

    CameraPose.h

    PoseValidationResult.h

    ErrorInfo.h

    IMatchStatsStore.h

    MeasurementTask.h

    ErrorInfo.h

    CameraConfig.h

    OpticalRigConfig.h

    TurntableConfig.h

    TriggerConfig.h

    MeasurementConfig.h

    ValidationConfig.h

    SystemConfig.h

**本次变更（对齐 ENG-09《类型与命名冻结表 V2.1》）：**

| 变更 | 说明 |
|---|---|
| 删除 `PoseResult.h` | 全套文档从未定义 `PoseResult` 类型，与 `ShipPoseResult` 重复 |
| 新增 `CameraRole.h` | 从 `optical/` 下沉至 `data/`（裁决 C-07）。`ImageFrame`、`DetectionResult`、`MeasurementCandidate` 均含该枚举，而 `data` 不得依赖 `optical` |
| 新增 `TurntableState.h` / `TurntableMotionState.h` | 原清单缺这两项，而它们是 `ITurntableController::state()` 的返回类型 |
| 新增 `MeasurementState.h` / `DeviceState.h` | 原归属未定（裁决 C-13） |
| 新增 `CoordinateFrame.h` | 原散落在 SYS-05 §3.1 未定文件 |
| 新增 `FeatureSet.h` / `ErrorInfo.h` | 原被 SYS-07 §8、SYS-06 §15 引用但从未定义（裁决 C-09） |
| 新增 7 个 `*Config.h` | 原仅为散文描述，无类型定义（§13） |

规则：

只保存数据。

禁止：

-   设备调用；
-   算法逻辑；
-   UI依赖。

------------------------------------------------------------------------

# 6 optical模块

路径：

    src/optical/

职责：

固定光机系统抽象。

结构：

    optical/

    ├── OpticalRig.h

    ├── CalibrationManager.h

    ├── CoordinateTransformer.h

    └── CameraSynchronizer.h

------------------------------------------------------------------------

负责：

-   三相机关系；
-   光机坐标；
-   标定管理；
-   坐标转换。

**本次变更（对齐 ENG-09）：**

- `CameraRole.h`、`CameraChannel.h` 移出 optical，下沉 `src/data/`（裁决 C-07）；
- **裁决 C-18：`OpticalRig` 归属 optical 层，不是 Device 层设备抽象。** 其语义冻结为"静态描述：三相机通道注册表 + 光机标定数据 + 只读查询接口"，**不含任何设备控制能力**。相机控制由 `device/camera/MultiCameraManager` 负责，后者**依赖** `OpticalRig` 获取通道与标定信息。SYS-06 §4 把 `OpticalRig` 写在 Device 章节，与本节依赖图（optical 在 device 之下）矛盾，以 ENG-09 §3.3 裁决为准。

------------------------------------------------------------------------

# 7 preview模块

路径：

    src/preview/

职责：

实时预览。

结构：

    preview/

    ├── PreviewManager.h

    ├── PreviewQueue.h

    ├── PreviewWorker.h

    └── PreviewConfig.h

------------------------------------------------------------------------

# 8 device模块

路径：

    src/device/

职责：

硬件抽象。

结构：

    device/

    ├── camera/

    │
    ├── MultiCameraManager.h

    ├── ICameraBackend.h

    ├── ImvCameraBackend.h

    └── VirtualCameraBackend.h


    ├── turntable/

    │
    ├── ITurntableController.h

    ├── PekoTurntableController.h

    └── VirtualTurntable.h


    └── trigger/

        ├── ITriggerController.h

        ├── HardwareTriggerController.h

        └── VirtualTriggerController.h

------------------------------------------------------------------------

# 9 application模块

路径：

    src/application/

职责：

业务流程。

结构：

    application/

    ├── MeasurementController.h

    ├── MeasurementStrategy.h

    ├── AlignmentController.h

    ├── StateMachine.h

    └── RetryManager.h

**`RetryManager.h` 职责说明（原清单只列文件名，无任何策略定义）：**

原 ENG-01 只给出该文件名，而 SYS-08 §7 的全部异常恢复路径均无次数与超时上限，导致 `RetryManager` 无从实现。现已冻结：

- **策略与数值来源：** ~~SYS-08 §7（三级超时体系 §7.1、失败三分类 §7.2、状态次数与超时表 §7.3、回退预算 §7.4、硬件降级 §7.5）~~ 〔引用无效·依据待裁决·见 Q-D2〕
- **接口：** ~~SYS-08 §7.6~~ 〔引用无效·依据待裁决·见 Q-D2〕

> ⚠ **勘误（2026-09-26 核，Q-D2）**：上列 `§7.x` 引用**全部无效**。`SYS-08 V2.1` 的 `# 7`
> 是 **TARGET_FOUND 状态**（§7.1 状态说明／§7.2 执行动作／§7.3 输出／§7.4 失败处理），
> 与本处所称"三级超时／失败三分类／次数表／回退预算"**内容完全不同**（节号碰撞）；
> §7.5／§7.6／§7.7 **在该文档中不存在**（悬空）。
> ∴ **`RetryManager` 的策略与数值目前没有冻结依据**，其现行取值来源是
> `config/measurement.yaml`（本批只登记，**不裁定**）。
> 详见《SYS-08-§7引用勘误.md》与《待裁决问题汇总》**Q-D2**。
> 本行**不改变**"`RetryManager` 是重试计数器与任务时限唯一持有者"这一结论
> （该结论的依据是 ENG-09 §10 约束 8，与 `§7.x` 无关）。
⚠ 本文件经复核**未随之升版**（它不在本批五份升版文档之列）：本处是**引用勘误标记**，
不是设计变更 —— 见 `AircraftPoseSystem/V2.1-C01_架构裁决变更说明.md`。
- **配置载体：** `MeasurementConfig`（ENG-09 §6.5）
- **错误码：** ENG-09 §5.27

**`RetryManager` 是重试计数器与任务时限的唯一持有者**，`StateMachine` 与 `AlignmentController` 均不得自持计数器（ENG-09 §10 约束 8）。故该文件是 `application` 模块的**强制实现项**，不可省略。

------------------------------------------------------------------------

# 10 algorithm模块

路径：

    src/algorithm/

职责：

视觉算法。

结构：

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

------------------------------------------------------------------------

## detection

目标检测：

    YoloDetector

------------------------------------------------------------------------

## scale

尺度估计：

    TargetScaleEstimator

------------------------------------------------------------------------

## selection

测量选择：

    selection/

    └── MeasurementSelector.h

**`IMatchStatsStore` 不在本目录（裁决 C-21）：** 该接口定义在 **`src/data/`**，原因见 §11 的依赖分析——若放在 `algorithm/`，其实现所在的 `infrastructure/` 将无法引用它（§17 的依赖方向为 `algorithm → infrastructure`，反向引用被 §18 禁止）。

------------------------------------------------------------------------

## model

模型管理：

    TargetModelManager

------------------------------------------------------------------------

## feature

特征提取：

    feature/

    ├── FeatureExtractor.h        // A 类：SIFT 关键点 + 描述子
    └── CadStructureLocator.h     // B 类：CAD 结构点定位（新增，裁决 C-21）

**`CadStructureLocator` 的用途：** 实现 ENG-10 §2.3 的"投影—匹配—拟合三步法"，从图像中定位 CAD 结构点（翼尖、进气口、尾翼边缘），**保证特征点覆盖目标全长**。

**它必须独立于 `FeatureExtractor`：** 两者面对的是不同的几何对象——`FeatureExtractor` 找**纹理不连续**，`CadStructureLocator` 找**几何不连续**（棱线交点）。用 SIFT 代替后者会失去展布保证，使 Yaw 精度可能突破 1 角分（SYS-15 §4.5 / ENG-10 §8）。

职责还包括 ENG-10 §2.4 的可用性判据计算（对应结构点数、展布宽度 `W`、亚像素拟合残差）。

------------------------------------------------------------------------

## matcher

特征匹配：

    FeatureMatcher

**融合要求：** 输入为 A 类（SIFT）与 B 类（CAD 结构点）两路特征，须实现 ENG-10 §2.5 的**冲突规则**——同一区域内两类对应反投影后位置差 > 2 pixel 时**以 B 类为准**。

------------------------------------------------------------------------

## pose

姿态：

    PnPPoseEstimator

------------------------------------------------------------------------

## validation

结果验证：

    PoseValidator

------------------------------------------------------------------------

# 11 infrastructure模块

路径：

    src/infrastructure/

包含：

    logger/

    config/

    recorder/

    performance/

    persistence/          // 新增，见下

负责：

-   日志；
-   配置；
-   数据保存；
-   性能统计；
-   运行期统计量持久化。

**新增 `persistence/`（裁决 C-21）：**

    persistence/

    ├── FileMatchStatsStore.h

    └── FileMatchStatsStore.cpp

职责：读写 `runtime/match_stats.yaml`（历史匹配成功率统计表，ENG-10 §4.1）。**只由 `infrastructure` 实现文件 I/O，算法层通过接口访问。**

**接口与实现分离（ENG-10 §5.1）：**

| 角色 | 文件 | 位置 | 说明 |
|---|---|---|---|
| 接口 | `IMatchStatsStore.h` | **`src/data/`** | 见下方的依赖分析 |
| 实现 | `FileMatchStatsStore.h/.cpp` | `infrastructure/persistence/` | 文件 I/O、写临时文件 + rename、解析失败隔离 |

**为什么接口必须放在 `data/`（依赖分析）：**

接口的消费者 `MeasurementSelector` 在 `algorithm/`，实现 `FileMatchStatsStore` 在 `infrastructure/`。按 §17 的冻结依赖方向：

    algorithm  →  infrastructure  →  data

若接口放在 `algorithm/`，则 `infrastructure/` 必须反向引用 `algorithm/` 才能实现它，**直接违反 §18 的禁止依赖**（这正是 C-07 中 `CameraRole` 必须下沉到 `data/` 的同一类问题）。

`data/` 是所有层都能引用的最低层，接口放此处则两侧均可引用：

    data/IMatchStatsStore.h
        ↑                 ↑
    algorithm/         infrastructure/
    MeasurementSelector   FileMatchStatsStore

**注意与 `ICameraBackend` 的区别：** `ICameraBackend` 及其全部实现（`ImvCameraBackend`、`VirtualCameraBackend`）**同处 `device/camera/` 一层**，不存在反向引用，故可放在消费者旁边。`IMatchStatsStore` 的实现必须落在 `infrastructure/`（文件 I/O 归其职责），跨了层，因此不能照搬该做法。

**为什么不能放在 `runtime/` 或 `config/`：**

- 它不是配置文件——内容在运行中增长，放入配置会破坏 ENG-09 §6.8 的"配置任务内冻结"；
- 它也不是纯运行时内存结构——需要跨任务持久化，并纳入 `config_snapshot` 以便复现（ENG-10 §4.4）。

------------------------------------------------------------------------

# 12 runtime模块

路径：

    src/runtime/

负责：

计算后端。

结构：

    runtime/

    ├── ComputeBackend.h

    ├── CpuRuntime.cpp

    └── GpuRuntime.cpp

------------------------------------------------------------------------

# 13 ui模块

路径：

    src/ui/

结构：

    ui/

    ├── MainWindow.h

    ├── widgets/

    │
    ├── ImageViewer.h

    ├── PosePanel.h

    ├── StatusPanel.h

    └── TurntablePanel.h

------------------------------------------------------------------------

# 14 app模块

入口：

    src/app/

    └── main.cpp

负责：

-   QApplication；
-   依赖创建；
-   系统启动。

------------------------------------------------------------------------

# 15 tests目录

    tests/

    ├── unit/

    ├── optical/

    ├── device/

    ├── turntable/

    ├── trigger/

    ├── preview/

    ├── algorithm/

    ├── integration/

    ├── hardware/

    ├── stability/

    └── golden/

**修正：** 原清单缺 `tests/preview/`，而 ENG-06 §6 的预览测试明确要求该目录。补齐后与 ENG-06、ENG-03 §15 三处一致。

------------------------------------------------------------------------

# 16 deploy目录

部署包：

    deploy/

    ├── bin/

    ├── lib/

    ├── config/

    ├── models/

    ├── calibration/

    └── logs/

------------------------------------------------------------------------

# 17 模块依赖关系

冻结：

    app

    ↓

    ui

    ↓

    application

    ↓

    preview / optical

    ↓

    device / algorithm

    ↓

    infrastructure

    ↓

    data

------------------------------------------------------------------------

# 18 禁止依赖

禁止：

## UI访问SDK

    MainWindow

    ↓

    ImvSDK

------------------------------------------------------------------------

## Algorithm访问设备

    PosePipeline

    ↓

    CameraSDK

------------------------------------------------------------------------

## Data包含业务

    ImageFrame

    ↓

    startMeasurement()

------------------------------------------------------------------------

# 19 V2.1新增模块总结

新增目录：

    device/turntable

    device/trigger

    optical/rig

    algorithm/scale

    algorithm/selection

    algorithm/model

    algorithm/validation

------------------------------------------------------------------------

# 20 工程验收标准

满足：

1.  模块职责清晰；
2.  依赖方向正确；
3.  支持虚拟设备；
4.  支持真实设备替换；
5.  支持离线部署；
6.  支持后续测试扩展。
