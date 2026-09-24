# proj_plane 全仓审查报告

审查基线：`main@9775ea67cbae76741f8f1545f0faf3e33e4982c8`。日期：2026-09-24。

本轮只读审查，未修改、提交或推送仓库。范围为 251 个 Git 跟踪文件的结构清点、源码静态检索，设备、应用、算法、光机、预览、UI、配置、记录器与构建模块的主要实现和调用链阅读，以及相关文档和测试夹具核对；不等于逐行复核每份历史归档。

## 结论

项目具备虚拟设备开发骨架、算法实现和较多测试，但仍有构建交付缺口及跨模块断点，不能据现有测试数量认定应用级成功测量链路已验证。

最新三份文档修改及两份新文件均已在这个完整仓库中找到。它们对 A0 已完成、真实后端仍未实现、CAM25 先接入和同步证据边界的修订方向成立。但新验收清单尚有型号/序列号混淆、元数据来源口径过宽，以及 Mono12 双路径没有落实到数据契约等问题，应先补正再据此实施。

建议先修复源码漏入库问题，并修正现成代码的关键断点；同步补齐 A1 的图像与错误语义。PH-01 继续保留，不要求先完成转台、硬触发或 CAD 基准才能接入单相机。

## 证据与验证边界

| 检查 | 本轮结果 |
|---|---|
| 仓库清点 | 251 个跟踪文件；`src` 149 个文件；`tests` 32 个文件；`项目文档` 35 个文件 |
| 测试定义清点 | 10 个测试源文件中有 242 个 `TEST` / `TEST_F` 定义；这是静态计数，不是执行通过数 |
| 完整 CMake 配置 | GCC 编译器检查通过；在 `cmake/Dependencies.cmake:47` 因当前审查环境缺少 OpenCV 开发配置而停止。未完成整个工程的编译和 CTest |
| 选择器局部执行 | 用 GCC 直接编译仓库原版 `MeasurementSelector.cpp`，复现角分单位错误，详见 R07 |
| SDK 安装方式复现 | 用独立小型 CMake 工程复现相同的绝对软链接创建及目录安装操作；安装后仍指向构建目录，详见 R03。使用占位文件，没有运行厂商 SDK |
| 静态构建结构检查 | 所有字面量 `add_subdirectory` 中，确认只有 `src/runtime` 缺少对应目录/构建文件 |
| 实机与 GUI | 未进行相机、转台、硬触发、Qt 窗口或 8 小时稳定性实测 |

README 中以前“242 用例通过”的证据有最小 GTest 垫片适用范围，本轮没有将其升级为原生 GTest/CTest、实机或完整应用成功链路的证明。

## 确认的问题

优先级含义：P1 会阻断构建/交付，或显著影响对应路径的正确性；P2 影响结果追溯或异常恢复。优先级与实施阶段分开：例如 CAD 匹配缺陷是 P1，但仍可按 013/算法路线修复，不必把全部后续工作塞进 A1。

| 编号 | 级别 | 问题 | 建议归属 |
|---|---|---|---|
| R01 | P1 | CMake 引用缺失的 `src/runtime`，且忽略规则会排除该源码目录 | 立即修复基线 |
| R02 | P1 | 安装规则没有安装主程序，`install_check` 仍可能通过 | 构建交付 / 011-B 前 |
| R03 | P1 | SDK 运行库安装后保留指向开发目录的绝对软链接 | 011-B / 扩充 H-10 |
| R04 | P1 | 测量进行中没有向预览队列提交采集帧 | 应用集成 / 011-B |
| R05 | P1 | 真实验证器与控制器的 bool 语义不一致，验证失败不能按设计排除相机 | 应用与算法集成 |
| R06 | P2 | 选择结果未写回成员，记录中的 `selectedScore` 恒为 0 | 结果记录 |
| R07 | P1 | 误差预测缺少弧度到角分换算，代码、公式及测试共同漏项 | 算法与 ENG-10 同步修正 |
| R08 | P1 | CAD/纹理冲突处理重复插入 CAD 对应点 | 013 / 算法集成 |
| R09 | P1 | 任意一次 `grab()` 失败就永久禁用通道，未区分超时和断连 | 011-A1 错误语义 |
| R10 | P1 | 文件标定中的非法矩阵可被接受，姿态路径绕过已有变换校验 | 014 / 真实测量前 |
| R11 | P2 | 部分保存失败后写失败包，可能残留此前 RAW 文件 | 011-B 保存链 |

### R01 — 源码目录漏入库，干净克隆的构建不完整

**证据：** [CMakeLists.txt](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/CMakeLists.txt#L213-L224) 无条件 `add_subdirectory(runtime)`；该目录不在仓库中。[.gitignore](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/.gitignore#L104-L113) 的 `runtime/` 匹配任意深度目录。本轮执行 `git check-ignore -v AircraftPoseSystem/src/runtime/CMakeLists.txt`，返回该规则。

**影响：** 依赖齐备后，配置仍会在缺失源码目录处失败。开发机曾编译成功不能证明 Git 仓库可复现；仅凭当前仓库也不能断言工作 agent 本地是否仍保留着这些源码。

**修改：** 若本意是忽略项目根运行数据，应把规则限定为 `/runtime/`，找回并正常跟踪 `src/runtime` 的源文件与 CMakeLists。若模块确已取消，应按目录设计同步撤销引用，不能仅为消除报错而盲删。

**验收：** 新目录不再被忽略；从新克隆、安装齐备依赖后完成 configure/build。不要仅在已有构建树上复编。

### R02 — 安装成功不代表有可执行程序

**证据：** [CMakeLists.txt](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/app/CMakeLists.txt) 只有主程序创建和链接，没有 `install(TARGETS ...)`。全仓 CMake 安装调用只找到配置、模型、标定及 SDK 目录安装；[InstallRules.cmake](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/cmake/InstallRules.cmake#L140-L150) 的 `install_check` 仅执行安装命令。

**影响：** 可以得到一份没有主程序的“安装成功”目录。现有 H-10 讨论动态库查找路径，还不足以覆盖这个更前置的问题。

**修改：** 补主程序及必要自有产物的安装规则；安装自检增加目标程序存在、权限正确，以及从安装目录实际启动的检查。

**验收：** 在独立安装前缀中确认 `bin` 下的实际目标程序，并按单相机清单完成采集。未连接硬件时，只能记录安装/启动级验证。

### R03 — SDK 安装产物仍依赖开发目录

**证据：** [FindImvSdk.cmake](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/cmake/FindImvSdk.cmake#L152-L184) 把 SDK 库的绝对路径链接到构建树 `imvsdk_runtime`；[InstallRules.cmake](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/cmake/InstallRules.cmake#L127-L133) 再用 `install(DIRECTORY ...)` 安装。小型 CMake 复现后，部署目录内 `libaudit.so` 仍是指向构建目录的绝对软链接。

**影响：** 在开发机原路径上可用，复制安装包到另一位置或移除 SDK 解包目录后断链。只增加 `LD_LIBRARY_PATH` 或 RPATH 不能修复不存在的链接目标。

**修改：** 安装包应包含需要的真实库文件，别名使用包内相对软链接；继续排除不需要的 SDK Qt 库。运行库查找路径按 H-10 一并处理。

**验收：** 把安装包复制到独立目录、令构建树和 SDK 原目录不可访问，再检查库解析、版本调用及采集；不能只运行仍能访问开发路径的 `install_check`。

### R04 — 测量期间预览没有生产者

**证据：** [SystemInitializer.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/app/SystemInitializer.cpp#L684-L735) 仅在 IDLE / COMPLETE / FAILED 调用 `pumpIdlePreview()`；其注释声称 controller 会提交预览。但全仓实际生产调用 `submitFrom()` 只在这个空闲路径。[MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L1163-L1205) 的采集封装只处理采集和降级，未投递帧。

**影响：** 进入活动测量状态后没有新预览帧；原队列耗尽即停帧。切换显示源还会清空已有显示帧，可能变成空白。既有流程测试检查了显示源随状态改变，没有证明新画面持续产生。

**修改：** 把测量链已经采到的帧送入预览，明确无采集状态如何维持预览；不要为显示另起一次无协调的抓帧。

**验收：** 使用带递增帧号的输入，从 IDLE 进入 SEARCH / ALIGN，断言显示帧号继续推进、切换角色后取得对应角色新帧。

### R05 — VALIDATE 的真假含义在三层之间不一致

**证据：** [PoseValidator.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/validation/PoseValidator.cpp#L170-L222) 对不合格结果返回 false；[PosePipeline.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/pipeline/PosePipeline.cpp#L753-L765) 原样返回。[MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L980-L1015) 却把 false 当成“验证过程失败”，在保存 `validationResult_`、排除当前相机之前直接返回。流程测试中的 [MeasurementFlowTest.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/tests/integration/MeasurementFlowTest.cpp#L590-L599) 则让“不合格”表现为返回 true、`out.valid=false`。

**影响：** 真实调用会丢失本次验证详情，并绕过当前相机排除动作。策略仍可能回退到 MEASURE_SELECT，但没有该排除信息，可能再次选中同一相机；不是说所有回退都不发生。

**修改：** 明确执行状态和验证结论的契约，使真实实现、控制器和测试桩一致。可规定 bool 表示是否完成验证、`out.valid` 表示合格与否，或调整控制器适配既定契约；不能只改单测桩掩盖差异。

**验收：** 用真实 `PoseValidator` 产生可控的不合格结果，经真实 Pipeline 适配与控制器处理，断言保存原因/指标、排除本相机、重新选择；另测真正无法执行的错误。

### R06 — `selectedScore` 始终来自默认成员

**证据：** [MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L759-L783) 和 [MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L1351-L1369) 都创建局部 `selection`，只写回相机角色。成员 `selection_` 在全源码中仅见初始化清零和 [MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L1021-L1033) 读取 `selection_.score`。

**影响：** 无论选择器真实得分多少，结果记录都是 0，削弱选择依据的追溯。记录器测试手工提供非零分数，不能覆盖这个生产断点。

**修改与验收：** 两个成功选择入口写回完整选择结果；通过控制器产出记录并保存，分别验证首次选择和失败换机后的角色与得分一致，使用输入侧给定的非零得分断言。

### R07 — E 误差项缺少单位换算，测试也复写了错误公式

**证据：** [MeasurementSelector.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/selection/MeasurementSelector.cpp#L334-L348) 将 `sigmaPx × sqrt(12) / (W × sqrt(N))` 直接当作角分，与 1 角分比较。[ENG-10 §1 数值例子](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/项目文档/软件工程设计/ENG-10_算法链接口与配置注入设计_V2.1_多相机闭环测量版.md#L22-L31) 中，σ=0.3 px、W=1930 px、N=100 给出约 0.19 角分；这正是先求弧度、再乘 `180/π×60` 的结果。

直接编译执行原版选择器的输出：

```text
production_predictedError_arcmin=5.38461390954e-05
ENG10_example_expected_arcmin=0.185109390794
production_eNorm=0.999946153861
```

**影响：** E 数值低估约 3437.75 倍，误差分项在常见条件下接近满分，其相对权重和相机排序可能被改变。

**修改：** ENG-10 §3.2 的显式公式也漏了换算因子，需同步修正文档与代码。不能把这次修正包装成仅代码偏离冻结式。[AlgorithmStageTest.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/tests/algorithm/AlgorithmStageTest.cpp#L145-L195) 的期望值复写了同一公式，应换成独立单位基准。

**验收：** 至少采用文档的两组已知数值：W=1930 得约 0.1851 角分；W=386 得约 0.9255 角分。另测达到 1 角分参考值时归一化结果归零。此处修复不证明整个误差模型已经通过实机标定。

### R08 — 冲突处理把 CAD 对应点加入两遍

**证据：** [MockFeatureMatcher.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/matcher/MockFeatureMatcher.cpp#L203-L267) 中的真实 `DescriptorFeatureMatcher`：遇到纹理点与 CAD 点冲突，先 `out.push_back(merged[j])`；随后组装结果时又把全部 CAD 点加入一次。

**影响：** 每个冲突会多出一个重复 CAD 对应点；`matchCount` 包含重复，而 `cadCount` 只统计后一次插入。实际独立点数被高估，匹配比例、最少点数门槛及后续 PnP 权重可能失真。文件名含 Mock，不代表该处只有测试桩。

**修改：** 冲突检查只标记要丢弃的纹理点，最终统一输出一次；同一三维点的重复关联是否去重另按明确策略处理。

**验收：** 构造“1 个 CAD 点 + 1 个不同三维点的近邻纹理冲突”：输出 CAD 一次、纹理零次、冲突数 1；再测多个纹理点冲突到同一个 CAD 点，校验 `matchCount == cadCount + textureCount`。

**阶段：** 当前生产 CAD 粗姿态注入尚未接入，因此本缺陷主要影响后续 CAD 路径，不要求先重启 013 基准才能做 A1。

### R09 — 一次取帧失败被直接当作永久掉线

**证据：** [MultiCameraManager.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/device/camera/MultiCameraManager.cpp#L197-L230) 对任意 `grabOne()` 失败都设置 `available=false`，后续采集跳过该通道；该分支没有根据后端原因区分取帧超时、暂时无帧与设备断连。

**影响：** 若 A1 按通常错误返回方式将暂时无帧/超时返回 false，一次事件就会令通道在本轮运行中持续不可用；两路被禁用后报相机不足。上层“瞬态失败可重试”的测试不能证明真实管理器会恢复，它绕过了这一行为。

**修改：** 在接 SDK 时明确无帧、超时、断连、不可恢复错误的可表示语义，保持原始 SDK 错误信息。瞬态情况保留通道并有限重试，真实断连才走禁用/重连策略；不得将失败伪装成成功帧。

**验收：** 用真实 MultiCameraManager 搭配可控后端，验证“成功→一次暂时无帧→成功”可恢复，以及“明确断连”按预期降级，两个情形不能共用同一错误分类。

### R10 — 无效标定可能进入“成功”姿态结果

**证据：** [CalibrationManager.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/optical/CalibrationManager.cpp#L72-L96) 的 rotation 读取只校验矩阵尺寸；3×3 零矩阵可原样进入系统。[PosePipeline.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/pipeline/PosePipeline.cpp#L727-L746) 自行组合矩阵、分解欧拉角并设置 success，没有使用现有 `CoordinateTransformer` 的旋转有效性检查。验证器检查有限数，但不检验旋转正交性/行列式。

另一个入口：[CalibrationManager.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/optical/CalibrationManager.cpp#L175-L205) 将错误内参回退为单位矩阵；如果图像尺寸仍为正，[PnPPoseEstimator.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/algorithm/pose/PnPPoseEstimator.cpp#L82-L111) 的内参可用性判断仍能放行。

**影响：** 文件存在或尺寸合法不能证明标定有效；在其他质量指标合格时，无效变换可能被接受。对准控制器的一些额外保护不能代替算法入口自身的校验。

**修改：** 对真实文件标定拒绝缺失/非法关键矩阵，检查有限性、旋转正交性、det≈+1、关键内参及图像尺寸匹配；收敛变换校验入口，避免两套路径行为不同。显式开发用合成标定可以保留，不能与读取坏文件后默默回退混为一谈。

**验收：** 有尺寸但缺内参、零旋转矩阵、反射矩阵、NaN 和图像尺寸不匹配均被可定位地拒绝；有效真实标定和标识清楚的合成标定分别验证。

### R11 — 保存失败重试可能留下混合结果包

**证据：** [Recorder.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/infrastructure/recorder/Recorder.cpp#L774-L809) 在同一任务目录依次写 JSON、RAW 和后续文件；中途失败没有隔离未完成产物。失败包路径 [MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L1712-L1729) 仅清空 `bestFrame` 再保存，[Recorder.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/infrastructure/recorder/Recorder.cpp#L628-L657) 遇到空帧只跳过，并不删除旧 RAW。

**影响：** 例如 RAW 已写成功、后续文件保存失败，最终失败包可能残留此前的 RAW，违背当前“不带原图”的失败包语义；也可能留下 JSON 与部分文件不一致的目录。已有失败包测试从空目录开始，没有覆盖此种覆盖路径。

**修改：** 保存采用临时目录/明确完成标志并在成功后发布，或实现同等可证明的事务边界；失败包与未完成成功包隔离。不要为清理而删除其他任务目录。

**验收：** 在 RAW 已落盘后注入一次后续写失败，再走控制器最终失败包路径，确认产物状态清楚且没有不属于该失败包的旧 RAW。

## A1 开始前应落实的接入约束

这些是即将实施的设计边界；不把尚未实现的真实设备功能伪装成已经跑出来的故障。

### A1-1 — 明确 Mono12 原始缓冲区和 8 位处理图如何共存

[ImageFrame.h](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/data/ImageFrame.h#L42-L66) 只有一份 `image`，当前契约为 8U；记录器 RAW 又直接写这份 `image`。[011-A1_A7A20MU201单相机接入验收清单.md](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/011-A1_A7A20MU201单相机接入验收清单.md#L160-L181) 要求显示 8 位、RAW 保留原位深，并禁止整体降位深，但没有给出承载两份数据的契约。

应先提交一个具体接口增量：例如保留既有 8U 处理/显示图，并增加明确所有权的原始载荷与像素格式、有效位深、布局元数据；也可以另设专用原始帧类型与适配器。实施前必须定清原始 RAW 指 SDK 打包字节还是规范化解包像素，并记录/冻结步长、字节序、打包方式和有效位对齐。不得在 backend 转成 8U 后让 Recorder 无意中保存显示图。

同时验证：SDK 缓冲区归还后图像仍有效；帧进入预览队列、测量缓存和记录器后，元数据与像素仍属于同一帧。涉及冻结数据类型的新增应按现有变更流程登记；本报告没有代用户裁定具体接口形态。

### A1-2 — 确定单真实相机的执行入口与真实/虚拟标识

`MultiCameraManager::initializeAll/startAll/capture` 维持至少两路可用这一测量约束；应用空闲预览也经由该管理器。只留下 CAM25 一路，现有主程序路径不会自动变成单相机诊断工具。

当前 README 明确允许 CAM25 真实、CAM50/100 虚拟的过渡形态，因此不应为了单相机验收直接把正式测量门槛改成一台。A1/B 应明确使用混合装配还是独立诊断入口；混合模式记录每路后端类型、逻辑角色、真实设备标识，真实设备计数不能从总可用相机数推导。

[SystemInitializer.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/app/SystemInitializer.cpp#L301-L321) 先逐个 initialize，随后 MultiCameraManager 又 initialize 一遍。应统一生命周期所有权或明确幂等语义，避免接入真实 SDK 后重复打开设备。

### A1-3 — 真正执行触发，并控制阻塞时间

全仓生产调用未发现相机触发命令的发出路径，现有控制器主要是 enable/disable；只设置 TriggerMode 或使能触发器不足以证明单次触发采集已经闭合。A1 应实际读回模式，并分别验证自由运行、软件触发发令后出帧和外部硬触发条件；硬触发阶段归属仍按 PH-01。

[main.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/app/main.cpp#L139-L155) 从 GUI 定时器同步调用应用 tick；[MeasurementController.cpp](https://github.com/Hregl/proj_plane/blob/9775ea67cbae76741f8f1545f0faf3e33e4982c8/AircraftPoseSystem/src/application/MeasurementController.cpp#L807-L839) 单次 tick 内可连续抓多组帧，解算与保存也是同步执行。若真实 SDK 抓帧长时间阻塞，界面与停止操作会一起等待，状态机的下一次时限检查不能中断正在阻塞的调用。A1 需要有界的 SDK 超时，B/C 需要验证取消响应、预览时延，并按需要落到工作线程/异步状态推进；不能凭协作式 tick 的名称认定它不会阻塞。

## 新验收清单与索引的具体修改意见

| 位置 | 当前问题 | 建议修改 |
|---|---|---|
| 清单 §1 第2项、§2.2 | 把 `A7A20MU201` 当作要匹配的序列号 | 分开记录型号和本台实际序列号；型号匹配目标机型，序列号匹配设备标签/选定绑定 |
| 清单 §1 表后说明 | 说 1–6 的证据只能由 ImvCameraBackend 产出，但 USB 枚举是系统工具证据 | 区分 OS 枚举、SDK 探针证据和应用实际装配证据；探针可辅助定位，但不能替代后端集成验收 |
| 清单 §1.1 | 把 P-1/P-2/P-3 都列为 1–6 开始前条件，其中接口类型和兼容性又要靠枚举验证 | 改为按证据逐步排查的依赖关系；错误码映射不应阻止先运行 lsusb |
| 清单 P-1/P-2 | 将通用 SDK 驱动/内存设置写得过于绝对 | 针对实测接口核验适用性。SDK 包含内核驱动不等于这台相机必需该模块；本轮没有厂商 SDK 包与硬件，不对兼容性作已验证结论 |
| 清单 §2.6 `cameraId` | 既有代码只冻结“对应 CameraChannel.cameraId”，清单又宣布真实后端下必为序列号 | 明确逻辑 ID 与硬件序列号的绑定，若改变既有语义则登记，不在验收表中暗改契约 |
| 清单 §3.1 | 正确提出双位深要求，但承载契约尚未闭合 | 按 A1-1 先补数据路径；`QtImageConverter::toQImage()` 实际方法名为 `fromMat()`，一并改正 |
| 清单 §4 M-1 与“由谁生成” | 统称所有字段来自 SDK、禁止 Application 填充 | 设备型号/序列号/格式等来自 SDK；主机时间来自主机时钟；角色与逻辑绑定来自配置/装配。采集层产出事实、记录器负责落盘，不要求后端负责组装结果包文件 |
| 清单 §2.8 | 仅 width/height/pixel_format 未必足以还原所有 RAW 布局 | 明确行步长、打包、字节序、有效位对齐，或者冻结统一无填充格式并记录转换方式 |
| 清单 §0/§2.5 | 硬触发可受阻，但缺少 A1/B/C 各阶段通过判定汇总 | 分阶段列必验项和阶段外项；硬触发不属于本次裁决范围时不冒充通过，也不让整份后续清单反向阻断已授权的软触发单相机工作 |
| README 第842行 | `[ENG-08](项目文档/)` 指向应用目录下不存在的路径 | 改为真实相对路径，最好直达具体 ENG-08 文件 |

清单中的 SYS-06 / ENG-09 链接目前落到文档目录，目录存在但不是精确的权威文档入口，也建议直接指向文件。内链文件存在性检查不能证明节号、字段语义或验收依赖正确。

## 已知未完成项，不作为本轮新增缺陷重复计数

- ImvCameraBackend、HardwareTriggerController、PekoTurntableController 仍为诚实桩；实际装配全虚拟，PH-02 的提醒成立。
- SDK 查找、链接和版本调用不证明设备枚举、打开或采集通过。
- CameraSynchronizer 尚无应用装配，设备帧号/相对序号相等不能证明三路同曝光。
- `setCoarseAttitude` 无生产调用，CAD 结构定位的前置输入未闭合。
- 历史匹配统计的 record/save 与部分诊断落盘字段仍缺生产消费方，不能将已有接口视为需求已完成。
- 开发用合成模型、真实标定和模型资产的阶段边界仍按现有计划；模型目录并不构成可直接投入真实测量的资产交付。
- PH-01、IF-SW-02 保持未决。这里只提出具体实现契约需要补齐，没有替用户完成阶段范围裁决。

## 建议交给工作 agent 的执行顺序

1. **恢复可复现基线**：处理 R01，补 R02；核查旧机器遗漏源码，不改历史编号、不删除归档。
2. **修正已有集成断点**：R04、R05、R06；分别补“测量中有新预览帧”“真实验证失败排除相机”“结果记录保留非零分数”的针对性测试。
3. **补齐 A1 契约并接 CAM25**：落实原始/显示图双路径、暂时无帧与断连分类、单次初始化、触发发令与实际装配；修订清单中的设备身份和元数据来源。继续保留 PH-01。
4. **完成 B 的可见与可保存交付**：预览真实画面、RAW 读回一致性、R11 异常保存、R03 安装包自包含及 H-10 启动路径；从安装目录完成采集后才关对应条目。
5. **再做 C 的 8 小时运行**：记录实际后端构成、模式、帧号、内存、错误与恢复、停止响应，只给出所测模式的结论。
6. **同步登记算法/标定问题**：R07 可以作为独立小修同步代码、ENG-10 和测试；R08、R10 纳入相应算法/标定阶段。CAD 基准仍在 013，不借本次扫描提前扩大 A1 工作范围。

每个关闭项应引用修复提交及真实覆盖该调用路径的证据。模块单测、集成夹具、应用运行、实机验收分别描述，避免再次出现“局部实现和测试已有，实际调用链仍断开”。
