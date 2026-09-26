# AircraftPoseSystem V2.1（多相机闭环测量版）

船载飞机姿态测量系统。三相机固定光机刚体（25 / 50 / 100 mm 焦段）+ 两轴转台，
测量飞机相对**舰体坐标系**的姿态，目标指标 **Yaw 误差 ≤ 1 角分**，作用距离 40~300 m。

- 目标平台：UOS x86_64（离线部署）
- 技术栈：C++17 / Qt5 Widgets / OpenCV / CMake / GoogleTest
- 当前基线：**v2.1-framework**（工程骨架 + 数据/设备/光机/预览/应用/算法六层 + Qt 界面；
  **里程碑 M1 已达成** —— 可执行文件**可启动、显示虚拟预览**，并运行至 `POSE_SOLVE`
  **后进入失败回退路径**；日志落盘，六项验证清单逐条实测通过，见 §1。
  ⚠ **尚未完成应用级成功测量全流程**（`SAVE → COMPLETE` 未走通，根因是机型库缺失）

---

## 1 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| 001 | CMake 工程初始化文件集 | ✅ |
| 002 | Data 基础数据层（31 类型 + 8 配置 + 错误码 + 单调时钟） | ✅ |
| 003 | 设备抽象层（相机 / 转台 / 触发，含虚拟与真实两套后端） | ✅ |
| 004 | Optical 光机系统（OpticalRig / CalibrationManager / CoordinateTransformer / CameraSynchronizer） | ✅ |
| 005 | Preview 实时预览（PreviewQueue / PreviewManager / PreviewWorker） | ✅ |
| 006 | Application 业务控制层（状态机 / 重试 / 对准 / 测量闭环） | ✅ |
| 007 | Algorithm 算法链（检测 / 特征 / 匹配 / PnP / 校验 / 尺度 / B 类定位 / 管线） | ✅ |
| 008 | Qt 界面与 main | ✅ |
| 009 | 系统集成与依赖注入 | ✅ |
| 010 | 第一次完整编译运行闭环（里程碑 M1） | ✅ |

**里程碑 M1 已达成**：`AircraftPoseSystem V2.1 Framework Alpha` —— **可启动、显示虚拟预览，
并运行至 `POSE_SOLVE` 后进入失败回退路径**；日志落盘，六项验证清单逐条实测通过
（结果见 [待裁决问题汇总.md](待裁决问题汇总.md) §6）。

> **需注意**：M1 的闭环**止于 `POSE_SOLVE`**，未走到 `SAVE → COMPLETE`，
> **尚未完成应用级成功测量全流程**。原因不是缺陷，而是机型库（`models/`）这一
> 真实交付物尚不存在 —— 11.md §十 的验证清单第 5 项只要求"状态变化：
> IDLE ↓ SEARCH ↓ TARGET_FOUND ↓ ALIGN ↓…"，未要求走完全程。
> 相关裁决项见 [待裁决问题汇总.md](待裁决问题汇总.md) Q-B16。
>
> ⚠ **单元测试里的"成功保存""第二次测量通过"不能当作本条的替代证据**：
> 那些是**测试夹具**（`StubRecorder` / 合成机型库夹具）下的验证，
> 属夹具自洽性，**不等于真实应用全流程验收**。

已完成各层的**验证方式**。008 之前没有可执行文件，故不能用"能跑起来"当依据，各层用独立探针验证；008 起可执行文件存在，验证方式增加一条"能启动并进入事件循环"：

| 层 | 验证 |
|---|---|
| data | 编译出 `libdata.a` 且含 `errorCodeName()` 符号；消费方探针打印角色枚举、角度单位、配置默认值 |
| device | 34 项行为探针：SYS-08 §7.5 的三行降级、`triggerTimestamp` 取最大、采集中途断连不消耗重试预算、1001 下限、转台越程 2002 |
| optical | 63 项行为探针 + 38 个 GTest 用例：毫米→米换算、坐标链顺序、平移逐级复合、万向锁拒绝、同步超差与 2 路降级 |
| preview | 39 个 GTest 用例（含并发用例）；**ThreadSanitizer 无数据竞争报告**；丢旧保新满足"压入 = 取出 + 丢弃"守恒 |
| application | 67 个 GTest 用例：StateMachine/RetryManager/MeasurementStrategy 27 项（正常转换、非法转换、异常恢复、§7.3 次数表、§7.4 双预算、§7.1 时限优先级）、AlignmentController 21 项（几何换算、量纲、符号、越程、标定完整性）、**SYS-08 §10 的 7 个收敛性用例 + 闭环正常路径 19 项** |
| algorithm | 45 个 GTest 用例，分两个可执行文件：`test_algorithm_stages` 20 项（尺度估计的量纲与距离、PnP 的退化与内点判据、校验、检测、匹配）、`test_algorithm_pipeline` 25 项（模型库落盘/读回的完整路径、三级坐标复合与平移次序、Yaw 分解、阈值注入、通道选择、B 类结构点定位） |
| ui | **39 项断言的一次性离线探针**（`QT_QPA_PLATFORM=offscreen`，源码不入库——ENG-01 §15 的冻结测试目录清单里没有 `tests/ui/`）：通道序未互换（写入 BGR=(10,20,200) 读回 R=200 G=20 B=10）与四类输入拒绝、深拷贝（改 Mat 后 QImage 不变）、ROI（step > width）逐行正确、图像 1:1 不放大 / 大图居中留边不裁切、定时器把帧画到控件上、切源后旧画面被清除、四个面板的文字与占位符 |
| 可执行文件 | **`./AircraftPoseSystem` 可启动并进入事件循环**（008 起目标存在）；`QT_QPA_PLATFORM=offscreen` 下运行 3 秒被超时终止（退出码 124），无崩溃、无 qWarning 输出（仅 offscreen 平台自身的一条 `propagateSizeHints()` 提示，见 §2） |
| 009 装配 | 14 条启动日志（配置 → 标定 → 设备 → 预览 → 算法 → 装配完成）全部落盘；三路虚拟相机启动、虚拟触发使能、虚拟转台就绪；`PreviewWorker` 线程运行；空闲 20 拍后 **已提交预览帧 20 / 已发布 20** |
| 010 闭环 | `DISPLAY=:0` 下窗口 `AircraftPoseSystem V2.1`（1600×984 @ (160,50)）**经 `xdotool` 确认为可见窗口**；图像区实际渲染出合成图与 OSD（`cam25 1280x1024 3.9 ms` / `TRIG:soft`），非仅"进程还在跑"；状态机实测轨迹 `SEARCH → TARGET_FOUND → ALIGN → STABILIZE → MEASURE_SELECT → CAPTURE → POSE_SOLVE` 后进入回退链并终止（根因是机型库缺失，**已作为 Q-B15/Q-B16 登记**）；SIGTERM 后 `logs/aps_YYYYMMDD.log` 为 **88 行且内容完整**（此前实测为 0 字节，见 §6 的 D-3 与 [待裁决问题汇总.md](待裁决问题汇总.md) §4） |

各模块目录为空时构建脚本会打印"暂无源文件，跳过生成"，这是设计行为而非故障
——见 §4。

> **当前测试基线（2026-09-26，011-A1 九项缺口定向修复批后）**：
> **11 个套件 338 个用例，0 失败断言 / 0 异常** —— `flowtest` **45** · `RecorderPackageTest` 17 ·
> `RecorderAdapterTest` 7 · `StateMachineTest` 27 · `AlgorithmStageTest` 23 · `PipelineTest` 28 ·
> `DeviceLayerTest` 78 · `OpticalLayerTest` 38 · `PreviewLayerTest` 40 · `AlignmentControllerTest` 21 ·
> `sysinittest` 14。
> ⚠ 上一条基线（同日，CAM25 接入与数据契约批后）记的是 **11 套件 / 310 用例**；本批**净增 28**
> （设备 / 记录 / 装配侧 24 ＋ 控制器侧 **4**），全部为**新增用例**，没有把既有用例删掉凑数。
> 控制器侧的第 4 条是 **`A1_44_同一拍内跨过任务期限时一次采集都不发起`** —— 批内**补查 F-1** 时新增：
> 它证明控制器「已到期 ⇒ 期限＝`nowNs`」那一支**可达**（初版误判为"不可达／防御性"），
> 配套定向变异 **M18** 忠实复现修复前形态 ⇒ 4 条断言转红。
> 更早的基线（2026-09-24）是 **11 套件 / 250 用例**；310 那一批新增与迁移共 +60
> （`flowtest` +4、`RecorderPackageTest` +5、`DeviceLayerTest` +41、`PreviewLayerTest` +1、
> `sysinittest` +9）。
> 上表按**阶段**记录的是各阶段当时的数字（application 67 / algorithm 45 等），
> 两者不矛盾：阶段数字是"那一阶段验证了什么"，此处是"当前全部用例的总数"。
> 其中 `RecorderPackageTest` 是 C-02 Step 5 新建的套件（结果包落盘的**解析级**验证，
> 不是子串匹配——见 [C-02 §12.4](V2.1-C02_实施设计说明.md) 的 M8/M10）；
> C-007 为它补了失败包的用例（失败包不含 `cam*.raw`、`failure.json` 可复原轨迹）。
> `RecorderAdapterTest` 是 C-009 新建的套件（`RecorderSinkAdapter` 这一层：它补齐的
> 三个外部事实**是否覆盖**而不是透传、测量事实**有没有被改坏**、`config_snapshot/`
> 里**到底有几个文件**——见 §8.5）。
> `sysinittest` 是 R04 收尾批新建的套件（`tests/integration/SystemInitializerTest.cpp`）：
> 把**真实 `SystemInitializer.cpp`** 编进目标、真实装配、**零桩**，用真实单调时钟逐拍驱动，
> 覆盖"按状态/分支的采集归属"与"ALIGN 越程 ⇒ 同拍 FAILED 的终态切换边界"（见 §6 第 92 行）。
>
> ⚠ **"338 个用例通过"的适用范围（2026-09-24 收尾批更正、2026-09-26 三次更新数字，与 §1 一致）**：
> 数字与执行路径必须**分开记账**，一句话说"全部实跑通过"是过宽的：

| 验证内容 | 实际执行路径 |
|---|---|
| 上表 11 套件 / 338 用例 | **GTest 垫片**（`GoogleTest` 未安装，见 §3.6；`/tmp/gtshim/build_tests.sh`） |
| 两条安装自检（`install_check` / `install_launch_check`） | **CTest**，真跑（干净前缀 + 四条启动判据，见 §4.6） |
| 新增 `test_system_initializer` 的**原生 GoogleTest／CMake 目标** | **未验证** —— 缺 `GoogleTest` 时 `tests/CMakeLists.txt` 直接 `return()`，任何测试目标都不生成（与既有 10 个目标同命运） |

> ⚠ **垫片的运行配方（两条命令，顺序不可颠倒）**：从**仓库根**执行 ——
> ```bash
> cmake --build build -j8            # ① 必须先重建库
> bash /tmp/gtshim/build_tests.sh    # ② 再重链垫片
> LD_LIBRARY_PATH=$PWD/build/imvsdk_runtime /tmp/gtshim/<套件名>
> ```
> **只跑 ② 会静默测到陈旧库**：垫片链接的是**预编译**的 `build/lib/*.a`，
> **它不重编 `src/`**。曾在一次变异检查中因此得到"0 红"的假结论（改动根本没进二进制）。
> `LD_LIBRARY_PATH` 指向 `build/imvsdk_runtime` 是必需的（SDK 运行期库只在构建树里）。
>
> ⚠ **`--gtest_filter` 在垫片下不生效**（垫片不实现用例筛选）：给了过滤参数仍会跑**全部**用例，
> 定位单条断言时请直接 `grep` 输出里的失败行，不要以为"跑了 1 个用例"。
>
> **三个构建分支各跑一套**（对应 §9.0 的三侧判据）：
> | 分支 | 配置 | 测什么 |
> |---|---|---|
> | `build` | 默认根（SDK 已解包）⇒ `APS_HAVE_IMVSDK=1` | 真实适配分支：`/tmp/gtshim/build_tests_havsdk.sh` ⇒ `sysinittest_havsdk`、`DeviceLayerTest_havsdk` |
> | `build_nosdk` | `-DIMV_SDK_ROOT=/nonexistent` ⇒ `APS_HAVE_IMVSDK=0` | 无 SDK 的**诚实桩**分支，须同样构建通过 |
> | `build-sdk` | `-DIMV_SDK_ROOT=$PWD/third_party/imvsdk` | 与 `build` 同分支，进程级与安装态证据取自它 |

> 因此该数字证明的是"逻辑在当前夹具下自洽"，
> **不等于**"在正式测试框架下通过"，也不等于"应用级全流程验收通过"——
> 单元测试中的成功保存、第二次测量通过等证据属**测试夹具验证**，
> 不能直接替代真实应用全流程验收（见 §7）。
> 它已按 `aps_add_test` 登记进 `tests/integration/CMakeLists.txt`，
> 并在 007/008 用过的那个 `/tmp` 桩 GTest 包下实跑确认：**10 个测试目标**
> 全部生成、链接、运行（该次针对的是 007/008 那批目标；`sysinittest` 的
> CMake 目标路径**尚未**以同样方式走过，原因见上表第三行）。
> ⚠ 该次 `ctest` 只报告 `install_check` —— 垫片的 `gtest.h` 不实现
> `--gtest_list_tests`，而 `gtest_discover_tests` 用 `PRE_TEST` 在 ctest 期枚举用例，
> 故各套件需**直接运行**可执行文件（007/008 亦如此）。装上真实 GoogleTest 后此限自动消失。

> **GoogleTest 未安装**（§5），故 `tests/` 下的测试**当前不参与构建**。
> 上表中的 GTest 用例数是用一个临时最小垫片手工编译并运行得到的
> （只作编译期与基本行为验证，不替代真实 GTest）。装好 GoogleTest 后
> `cmake ..` 即自动启用这些测试，无需改动任何文件。
>
> 007 阶段的测试额外走了一遍**工程自身的 CMake 装配**：用一个放在
> `/tmp` 的桩 GTest 包（把同一份垫片头文件与 `main.cpp` 打成
> `libgtest.a` / `libgtest_main.a`）配置了一个独立构建目录，使
> `aps_add_test()` 真正执行 —— 8 个测试目标全部生成、链接、运行通过。
> 这一步是为了验证 `tests/algorithm/CMakeLists.txt` 本身（源文件清单、
> 依赖名、目标名）而不仅是"源码能被 g++ 编译"。桩包只存在于 `/tmp`，
> 不入库；入库的仍是真实的 `find_package(GTest)` 路径。
>
> 008 阶段在同一桩包下重跑了一遍（此时 `libui.a` 与可执行文件目标都已存在）：
> 7 个测试子目录、**8 个测试目标**全部生成、链接、运行，各 `test_*.out`
> 的失败断言数均为 0，`install_check` 通过，可执行文件也一并生成。

---

## 2 构建

```bash
cd AircraftPoseSystem
mkdir -p build && cd build
cmake ..
make
```

可执行文件输出到构建根目录，因此运行方式与 1.md《第一版代码里程碑》一致：

```bash
./AircraftPoseSystem
```

> 可执行文件目标自 008 起存在（`src/app/main.cpp` 已生成，1.md Phase 8）。
> 无显示设备的环境（CI、ssh）用 `QT_QPA_PLATFORM=offscreen ./AircraftPoseSystem`
> 验证能否启动；该模式下会打印一条 offscreen 平台自身的
> `propagateSizeHints()` 提示，不是错误。
>
> 008 阶段启动后界面显示"无图像"占位：窗口此时**没有** PreviewManager
> （009 的 SystemInitializer 负责创建并注入），故取帧定时器不启动。
> 这是 008 应有的样子，不是缺件——见 §6 第 59 行。

### 常用选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `BUILD_TESTING` | `ON` | 测试开关（ENG-03 §6） |
| `ENABLE_GPU` | `OFF` | GPU 加速；第一阶段无 GPU 后端，开启会给出警告 |
| `ENABLE_ASAN` | `OFF` | AddressSanitizer（ENG-03 §18） |
| `ENABLE_TSAN` | `OFF` | ThreadSanitizer；与 ASAN **互斥**，同时开启直接中断配置 |
| `ENABLE_WERROR` | `OFF` | 警告视为错误 |
| `APS_REQUIRE_DEVICE_SDK` | `OFF` | SDK 缺失时中断配置（发布构建用） |
| `IMV_SDK_ROOT` / `TURNTABLE_SDK_ROOT` | 自动探测 | 见 `cmake/FindImvSdk.cmake` |

示例：

```bash
# 内存检查（ENG-03 §18）
cmake .. -DENABLE_ASAN=ON && make && ctest

# 线程检查（SYS-09 双流水线是数据竞争高发区）
cmake .. -DENABLE_TSAN=ON && make && ctest

# 接入真实相机 SDK（Sprint 2）
cmake .. -DIMV_SDK_ROOT=/opt/ImvSdk
```

### 安装（离线交付包）

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make
cmake --install .        # 默认输出到工程内 deploy/
```

装出的 `deploy/` 含 `bin/`（可执行文件）、`lib/`（10 个模块库 + SDK 运行期库）、
`config/`（7 个 yaml）、`models/`、`calibration/`。

⚠ **用默认配置路径时必须从交付包根目录启动**（`cd deploy && ./bin/AircraftPoseSystem`）。
`main.cpp` 的 configDir 初值是相对的 `"config"`，故在 `bin/` 里直接运行会
找不到配置并以退出码 1 退出。也可以显式指定配置目录：
`./bin/AircraftPoseSystem /绝对/路径/config`。
但 `models/`、`calibration/`、`logs/`、`output/` 仍按当前工作目录解析
—— 该现象即已登记的 **Q-B10**（待裁决）。

**安装自检**（`BUILD_TESTING=ON` 时自动注册）：

```bash
ctest -R install_check         # 跑 cmake --install 并核对 bin/ 与 10 个模块库
ctest -R install_launch_check  # 从安装目录真的启动一次（跑满 3s 仍存活 = 通过）
```

前者的产物核对由 `src/app/CMakeLists.txt` 里的 `install(CODE ...)` 在**安装过程中**
执行 —— 因此"装了个没有主程序的目录"**不可能成功**，无论谁手工敲安装命令。
自检前缀是构建目录下的 `install_check/`，**不是 `deploy/`**：后者是真实交付树，
自检指向它会覆盖交付内容。

---

## 3 目录结构

依据 **ENG-01《工程目录结构设计》**。

```
AircraftPoseSystem/
├── CMakeLists.txt          工程入口（ENG-03 §4）
├── cmake/                  构建配置（ENG-03 §5）
│   ├── BuildOptions.cmake      开关、输出路径、版本基线（ENG-03 §6/§16/§19）
│   ├── CompilerOptions.cmake   警告集、Debug/Release、Sanitizer（ENG-03 §18）
│   ├── Dependencies.cmake      全工程唯一依赖查找点（ENG-03 §7~§11）
│   ├── FindImvSdk.cmake        华睿相机 SDK（ENG-03 §10）
│   ├── FindTurntableSdk.cmake  转台 SDK 预留（ENG-03 §11）
│   └── InstallRules.cmake      deploy/ 布局（ENG-03 §17）
├── src/                    源码（ENG-01 §4）
│   ├── app/                    可执行文件入口（ENG-01 §14）
│   ├── ui/                 →   libui           Qt 界面（ENG-01 §13）
│   ├── application/        →   libapplication  状态机 + 测量控制（ENG-01 §9）
│   ├── preview/            →   libpreview      预览流水线（ENG-01 §7）
│   ├── optical/            →   liboptical      光机 + 坐标变换（ENG-01 §6）
│   ├── device/             →   libdevice       相机 / 转台 / 触发（ENG-01 §8）
│   │   ├── camera/         →     device_camera
│   │   ├── turntable/      →     device_turntable
│   │   └── trigger/        →     device_trigger
│   ├── algorithm/          →   libalgorithm    检测/特征/匹配/PnP（ENG-01 §10）
│   ├── infrastructure/     →   libinfrastructure  日志/配置/记录（ENG-01 §11）
│   ├── runtime/            →   libruntime      计算后端（ENG-01 §12）
│   └── data/               →   libdata         公共数据结构（ENG-01 §5）
├── tests/                  测试（ENG-01 §15 / ENG-03 §15）
├── config/                 运行配置（ENG-01 §3.2）
├── models/                 目标模型与算法模型（ENG-01 §3.3）
├── calibration/            标定数据（ENG-01 §3.4）
├── deploy/                 安装输出（ENG-01 §16）
├── docs/                   见 docs/README.md
└── scripts/
```

**冻结依赖链**（ENG-01 §17 / ENG-03 §12）：

```
app → ui → application → preview · optical → device · algorithm → infrastructure → data
```

依赖方向由 `target_link_libraries` 强制，不是约定。违反时**链接期**失败，
不依赖代码评审。

---

## 4 构建系统的六条约定

这几条是为"按 Phase 逐阶段生成文件"这一开发方式专门设计的，新加 `.cpp`
时**不需要修改任何 CMakeLists.txt**——**前提是该模块的源文件全在模块根目录**；
源文件分布到子目录时要按 §4.4 / §4.5 显式声明（008 阶段在 `ui` 上实测踩到
§4.5 那一行）。

### 4.1 空模块自动跳过

`aps_add_module_library()`（见 `src/CMakeLists.txt`）在模块目录下没有 `.cpp`
时跳过生成该库，并打印说明。理由：CMake 会以 `No SOURCES given to target`
中断配置，使逐阶段生成的**每一个中间状态**都无法构建验证。

### 4.2 依赖缺失自动降级

只链接**已存在**的依赖目标。若 `application` 已有源文件而 `device` 尚未生成，
未过滤的 `target_link_libraries` 会把 `device` 当成裸库名，直到链接期才报
`cannot find -ldevice`——错误信息与真实原因（模块还没写）相距甚远。

### 4.3 第三方依赖集中查找

模块 CMakeLists.txt **不得自行 `find_package`**，只能链接 `cmake/Dependencies.cmake`
导出的目标：

| 目标 | 来源 | 必需性 |
|---|---|---|
| `aps::opencv` | `find_package(OpenCV)` | 必需 |
| `aps::qt5` | `find_package(Qt5 COMPONENTS Widgets)` | 必需 |
| `aps::gtest` | `find_package(GTest)` | 可选 |
| `aps::imvsdk` | `cmake/FindImvSdk.cmake` | 可选 |
| `aps::turntablesdk` | `cmake/FindTurntableSdk.cmake` | 可选 |

若各模块各自查找，两处可能拿到编译选项不同的同一库，导致同一类型在两个
编译单元中 ABI 不一致——**链接期不报错，运行期随机崩溃**。

### 4.4 源文件全在子目录的模块用聚合目标

`device` 的源文件全部在 `camera/`、`turntable/`、`trigger/` 三个子目录下，
模块根目录没有 `.cpp`。若照 4.1 判空，`libdevice.a` 会被当成"空模块"跳过，
于是 `preview`/`optical` 的 `DEPENDS device` 被 4.2 静默过滤掉——报错要到
链接期才出现，且表现为**看不出来源的 undefined reference**。

因此这类模块用 `aps_add_aggregate_module(<target> DEPENDS ...)` 生成
INTERFACE 聚合目标，把子目录库聚成一个可被依赖的名字。

> `libdevice.a` 这个名字因此**不存在**（ENG-03 §12.3 的措辞是"生成
> libdevice"）。这不影响任何消费者：链接 `device` 得到的是同样的三个库。

### 4.5 源文件跨层目录的模块要显式 `RECURSE`

`aps_add_module_library()` 默认只收集模块**根目录**的 `*.cpp`（设计要点见
`src/CMakeLists.txt`：保证"哪个模块拥有哪个文件"可回答）。因此只有源文件
全在根目录的模块才真的"加文件不用改 CMake"：

| 模块的情形 | 做法 | 漏掉时的现象 |
|---|---|---|
| 源文件全在根目录（`data` / `preview` / `app`…） | 默认即可 | —（`app` 于 008 新增 `main.cpp` 后确实无需改动任何 CMake 文件） |
| 源文件**全在子目录**（`device` 的 camera/turntable/trigger） | `aps_add_aggregate_module()`（§4.4） | 整个模块被"空模块"规则跳过，报错是链接期一句措辞误导的"`device` 尚未生成" |
| 根目录**有**、子目录里**也有**（`ui`：`MainWindow.cpp` 在根、控件在 `widgets/`、工具在 `utils/`） | `aps_add_module_library(<m> RECURSE …)` | **最隐蔽**：库非空，故不打印"暂无源文件，跳过生成"，配置阶段一切正常，只是库里只有根目录那一个 `.cpp`；直到链接可执行文件才以 undefined reference（`ImageViewer`/`PosePanel`…）暴露。008 首次配置的输出是"模块 ui：1 个源文件" |

**唯一能在不链接的情况下发现它的地方**：看 `cmake` 输出里"模块 X：N 个源文件"
那一行的 N 是否等于该模块实际的 `.cpp` 总数。

### 4.6 `install(TARGETS)` 写在建立该 target 的函数/文件里

R02（2026-09-24 审查报告）发现全工程 `install(TARGETS ...)` 实体 **0 处**，
于是 `deploy/` 只有空的 `config/models/calibration` —— **一个没有主程序的交付包**。
现在分三处，各写各的：

| 装什么 | 写在哪 | 为什么 |
|---|---|---|
| 10 个模块库 | `aps_add_module_library()`（`src/CMakeLists.txt`） | 该函数是**唯一知道 target 是否存在**的地方（对空模块提前 `return()`）。写在各模块文件里就要各自再判一次 `if(TARGET ...)`，漏一个就是配置期 `install TARGETS given target ... does not exist` |
| 可执行文件 | `src/app/CMakeLists.txt` | 它不经过上面那个函数 |
| 安装断言 | 紧随该 `install(TARGETS ...)` **之后**、**同一文件** | 见下 |

**两条实测踩出来的坑（都不是笔误，是 CMake 的既定行为）：**

1. **断言不能集中写在 `cmake/InstallRules.cmake`。** 生成的 `cmake_install.cmake`
   里，**父目录自己的 install 规则全部排在子目录 `include()` 之前**，与
   `add_subdirectory()` 出现的位置无关。故写在 `InstallRules.cmake` 的
   `install(CODE ...)` 会在 `bin/` 被拷贝**之前**执行，断言必然失败。
   （实测：本文件的 CODE 在生成脚本第 75~96 行，`build/src/cmake_install.cmake`
   的 include 在第 102 行。）**凡断言"某 target 已装好"的 CODE，都要与那条
   `install(TARGETS ...)` 同文件、且排其后。**
2. **`install_launch_check` 的 `WORKING_DIRECTORY` 必须是安装前缀。**
   `main.cpp` 的 configDir 初值是相对的 `"config"`，即**按当前工作目录**解析；
   ctest 的默认工作目录是构建目录，那里没有 `config/`，程序会以退出码 1
   打印"找不到配置文件"后退出 —— 测试恒红，且失败原因与安装规则毫无关系。
   （实测：cwd=安装根 → 124；cwd=`bin/` 或 `build/` → 1。）
   取"cwd=安装根 + 默认配置路径"而不是"用 `argv[1]` 显式指定"，是因为前者
   正是现场解包后的动作；⚠ 注意 `argv[1]` **确实**可以指定配置目录，
   程序并非只能从包根启动，但 `models/`/`calibration/`/`logs`/`output` 仍按
   CWD 解析（已登记 Q-B10）。

断言本身核对**两件事**：`bin/AircraftPoseSystem` 存在且可执行，
**以及 `lib/` 下每一个已建立的模块库都在**。只核对前者会漏掉 R02 缺陷的一半
（报告原文即"没有 bin/、没有 lib/"）。期望的库文件名在配置期从全局属性
`APS_INSTALLED_LIBRARY_TARGETS` 生成，故**以后新增模块自动纳入断言**，
不必回来改这里。

**R02 收尾（2026-09-24 复审）加的两条约定 —— 都不是洁癖，各有一条实测证据：**

3. **`install_check` 先清空自检前缀再安装。** 上一版只跑 `cmake --install`，
   而该前缀在多次运行之间**留存**，于是"本次运行什么都没装出来"完全可能被
   **上一轮的陈旧产物**掩盖。**已实测复现**：把 `src/app` 的整段 R02 安装规则
   整体停用（`install(TARGETS)` 与随之的 `install(CODE ...)` 断言一起停用），
   前缀里留着上一轮的 `bin/AircraftPoseSystem` 与 `logs/`，两条自检**依旧全绿**
   （`install_check` 0.01 s、`install_launch_check` 3.00 s 通过）—— 而一次全新
   安装的产物里**根本没有主程序**。加强后同一处变异立即转红
   （`install_launch_check` 报"退出码 127：没有那个文件或目录"，并把本次日志
   `cat` 出来）。清空只针对自检前缀 `${CMAKE_BINARY_DIR}/install_check`，
   **不碰 `deploy/`**（见下一条与 Q-C7）。
4. **`install_launch_check` 判四条，且日志落在本次运行的文件里。**
   判据是：① 退出码 `124`；② 日志含 `启动完成`；③ 日志含首拍标记
   `APP_FIRST_TICK_COMPLETED`；④ 日志**不含**`启动失败`。任一条不满足则
   `cat` 出 `${前缀}/launch_check.log` 后退出非零 —— 失败原因可见。
   · **为什么 ① 单独不够**：`timeout 3` 把进程杀掉只说明"跑满 3 秒仍存活"，
     而**初始化过程里卡死**同样满足 124；那种情况下窗口从未显示、状态机从未
     推进一拍，却能让自检通过。
   · 首拍标记 `APP_FIRST_TICK_COMPLETED` 由 `main.cpp` 的**真实定时器回调**
     在**正常走完之后**打一次（不是 `SystemInitializer::tick()` 的入口 ——
     入口只证明"进入了该函数"）。它使验收口径从"进程活着"精确到
     **"初始化完成，事件循环至少完成了一次应用定时回调"**。持续响应与长期
     稳定性仍归后续测试（本批**不**扩到 8 小时框架）。
   · **不能由它推出的事**（2026-09-26 删去原第 4 条 bullet）：首拍标记
     **不能**证明 `PreviewWorker` 这个**异步预览线程**已经处理并发布了图像
     —— 该回调只说明"初始化完成 + 事件循环至少完成一次应用定时回调"这
     一条因果链，预览线程的推进不在其中。`timeout` 发出的是**信号**，
     它不保证执行 C++ 析构函数，故也不能据此断言"预览线程的析构会 join"
     （这正是上一版那条 bullet 的两处越界，都已删）。
   · 可以够得着的两条**各自表述**，不再合并成一条更强的结论：
     ① 安装自检证明的是**初始化完成 + 事件循环至少完成一次应用定时回调**；
     ② **测试用例正常退出**是该**测试路径**下析构收尾的证据（测试替身
        路径，与安装自检不是同一条路径，不得互相代替）。

---

## 5 依赖状态与待办

```
$ cmake ..
-- [APS] 找到 OpenCV 4.6.0
-- [APS] 找到 Qt5 Widgets 5.15.8
-- [APS] 未找到 GoogleTest：测试目标将被跳过 ...     ← 需处理
-- [APS] 未找到 ImvSdk（非致命）...                  ← 不设 -DIMV_SDK_ROOT 时的输出
-- [APS] 未找到 TurntableSdk（预期行为）             ← Phase 2 处理
```

> ⚠ 上面两行是**不传 `-DIMV_SDK_ROOT` 时**的输出，与"SDK 没到货"时的输出**完全相同**
> —— 这正是 §6 第 31 行长期未被发现的原因（`CMAKE_MODULE_PATH` 从未设置，
> 查找器压根没跑）。2026-09-23 已修，现在传 `-DIMV_SDK_ROOT=` 会真正走到查找器。

| 依赖 | 状态 | 处理 |
|---|---|---|
| Qt5 5.15.8 | ✅ | — |
| OpenCV 4.6.0 | ✅ | — |
| GoogleTest | ❌ 未安装 | `sudo apt install libgtest-dev`，装后重跑 `cmake ..` 即自动启用。⚠ 当前 338 个用例是**临时 GTest 垫片**下跑的，正式 GoogleTest／CTest 验证待完成（见 §1） |
| ImvSdk | ⚠ **已解包到工程开发目录**（`third_party/imvsdk/`），**查找、链接及版本调用通过**；**尚未完成实机采集验收** | 见 [核验报告](V2.1-011A0.1_ImvSDK环境核验报告.md)（§6.3 端到端探针）。实机采集属 **011-A1**；`VirtualCameraBackend` 仍是当前装配的实际件 |
| TurntableSdk | ❌ 未安装 | ENG-08 §11：第一阶段不实现 Peko_D，属预期状态；真实转台受 PH-01 阶段归属待裁决约束 |

**GoogleTest 不装则 1.md 第一阶段范围的 "✅ GoogleTest框架" 与 ENG-08 §5
Sprint 1 的"测试框架"两项交付无法达成。** 该缺失不影响 `cmake`/`make` 通过
（依赖查找刻意用 `QUIET`），但会在配置输出中显著警告，不静默降级。

### 5.1 装配模式与新增的两个预算键（2026-09-26 批）

**装配模式是显式的，不靠探测。** `config/camera.yaml` 每个通道新增 `backend:` 键：

| `backend:` | 装配出的后端 | 说明 |
|---|---|---|
| `virtual` | `VirtualCameraBackend` | 无设备也能跑通全流程（当前开发机默认） |
| `imv` | `ImvCameraBackend` | 真实 SDK 路径；**打不开 ⇒ 启动失败**，明确报告接入失败 |

⚠ **缺 `backend:` 键 = 配置错误，启动失败** —— 刻意不设隐式默认，避免"检测到 SDK 就自动切换"
这类不可见的行为变化；**也不得**在真实相机打开失败后静默换虚拟件。
∴ 当前开发机上"有相机"与"用相机"是**两件事**，装配摘要行（启动即打印）会逐路写明
**实际装配的后端类型**与设备回报的型号／序列号（见 `011-A1_...验收清单.md` §4）。

**`measurement.yaml` 新增两个取帧预算键**（都在 `capture:` 段）：

| 键 | 默认 | 含义 |
|---|---|---|
| `grab_timeout_ms` | `100` | **单次**取帧上限（后端传给 SDK 的 `IMV_GetFrame` 超时实参） |
| `grab_group_budget_ns` | `3.0e8`（300 ms） | **一次 `capture()` 三路**的总预算 |

实际等待时长取**三者最小**：`min(grab_timeout_ms, 组剩余 ms, 距本轮绝对期限的剩余 ms)`；
距期限 **< 1 ms 时既不发软件触发令、也不取帧**（避免"发出了脉冲却收不回帧"），
该路如实记 `Timeout`＋"预算耗尽"而不是伪装成设备错误。

⚠ **时限余量是已知问题**：`capture_frame_count = 5` × `grab_timeout_ms = 100` × 3 路
与 `capture_timeout_ns = 1.5e9` 的关系**零余量**（复制／格式转换／评分的耗时未计入）
⇒ CAPTURE 在真实相机上**可能被时限截断**。这属时限体系的冻结值问题，
登记在 [待裁决问题汇总.md](待裁决问题汇总.md) **Q-D2**，本批**不改**这些值，只把事实测出来。

### 5.2 采集诊断的出口与期限取值的两条规则（2026-09-26 批二）

**① 诊断出口（分层干净）**：`MultiCameraManager::setDiagnosticSink(std::function<void(const std::string&)>)`
默认**空 ⇒ no-op**；由 **app 层**（`SystemInitializer`）接到 `ctx_.logger->warn("device", …)`。
`src/device/` **不包含 `Logger.h`** —— 设备层只负责"**把本次事实交出去**"，不负责写日志。
判据：**仅当该路 `status != Ok` 或本次诊断非空时输出**；
**全部正常的一轮不产生任何 sink 输出**（不逐路写 WARN）。

**文本由单一格式器 `data::channelGrabRecordText()` 生成**，设备层 sink 与控制器侧文本**共用**它：

```
role=opStatusName（调用 IMV_GetFrame 返回 N；清理失败：IMV_Close 返回 M；<diagnosis>）；skippedReason
```

⚠ `sdkError` 为空时写 **"未调用 SDK（本地判定）"**，**不伪造调用** —— 虚拟／脚本后端可能**根本没调 SDK**，
管理器**不得**替它补一个 `{ImvGetFrame, 0}`。同理 `frameStatusRaw` 用 `optional`：
**"未取得"（`nullopt`）与"正常值 0"必须可区分**。

**② 期限取值的两条规则**（`data::kNoDeadlineNs`，与 `RetryManager::kNoDeadline` **同值同义**）：

| 取值 | 含义 | 参与 `min(...)` 吗 |
|---|---|---|
| `UINT64_MAX`（`kNoDeadlineNs`） | **无期限** | **不参与**（不得被算成"已到期"） |
| 其余 `≤ nowNs` | **已到期** | 参与 ⇒ 结算为已耗尽 |
| 其余 `> nowNs` | 尚有余量 | 参与 |

⚠ **"已到期"这一支是可达的真实路径，不是防御性分支**（`A1_44` 用例 ＋ 变异 **M18** 实证，
2026-09-26 批二收尾更正）：`tick()` 的入口判定读的是**形参**（`main.cpp` 传入的快照），
而 `stepCapture()` 每一轮判停／取期限时**重新读一次注入时钟**；期限只要落在这两次读钟**之间**，
本拍就会带着**已到期**的任务走进 `acquireDeadlineNs()`。若退回"忽略 `taskRemaining == 0`"的旧逻辑，
这一拍会退化为状态预算／组预算、**重新获得一份预算**并发起本不该发起的采集。
⚠ 反向方向（"无期限"被算成已到期）**仍未实测** —— 现有用例无一条构造出无期限任务
（`kNoDeadlineNs` 只在 `RetryManager::beginTask` 的溢出分支产生），该方向只由上述判据守住。

**③ 三个预算检查点各自重读时钟**（原实现只读一次钟、三个下游全部复用那一读数 ⇒ 结算时早已过期）：

| 检查点 | 剩余 < 1 ms 时的动作 | 该路记录 |
|---|---|---|
| **入口** | 连模式读回都不执行；软件触发 **0** 次、`grab` **0** 次 | `Timeout` ＋ "预算耗尽（入口：组剩余 …）" |
| **读回后、发令前** | 软件触发 **0** 次、`grab` **0** 次 | `Timeout` ＋ "**发令前**预算耗尽（读回耗时计入后：组剩余 …）" |
| **发令返回后、取帧前** | 软件触发 **1** 次、`grab` **0** 次 | `Timeout` ＋ "**已发令、未取帧**（取帧前预算耗尽：组剩余 …）"；`attempted = true` 且**保留触发结果** |

⚠ 第三行**不得**写成"既未发令也未取帧" —— 脉冲确实已经发出去了。
`timeoutMs` 在**每个**检查点用**重读后的读数**重算。

**④ 触发前置判定**（读回不完整／不一致时**不发令、不取帧**）：

| 情形 | 本路结果 | 后续动作 |
|---|---|---|
| 读回不完整、`reported` 未知 | `NotStarted` / `1004` ＋ "本轮无法确认触发前置条件" | **不发令、不取帧** |
| 读回完整但与请求**不一致** | `ContractViolation` / `1006` ＋ **请求值与实际值都记录** | **不发令、不取帧** |
| 读回完整且一致 | 按**实际回读模式**执行 | 仅 `Software` 发软件触发 |

**两种失败都不永久禁用通道**（下一轮允许重新读回）；"未知"**不得**写成"设备已停止取流"。
逐项读回现场见 `data::TriggerFeatureReadback`（特性名／`callAttempted`／`failure`／`returnedEmpty`／`value`）——
**"调用成功但返回空串"与"SDK 调用失败"必须可区分**。

> 依据：`SYS-04 V2.5 §5.4`／`§6.2` 与 `ENG-09 V2.4 §2.5`（两文档**同批发布、须成对引用**）；
> 裁决正文见 [V2.1-C01_架构裁决变更说明.md](V2.1-C01_架构裁决变更说明.md) **C-016**。

---

## 6 与设计文档的对应关系

设计文档位于 `../项目文档/`，**不在本仓库内**（见 `docs/README.md`）。

| 文档 | 管辖范围 |
|---|---|
| **ENG-01** | 工程目录结构（本仓库目录布局的权威）；V2.1，其 `RetryManager` 段有一处 `§7.x` 勘误标记 |
| **ENG-02** | C++ 类与文件规划 |
| **ENG-03** | CMake 规范（本仓库构建脚本的权威） |
| **ENG-04** | 核心类设计（V2.1，其 §10 有一处 `§7.x` 勘误标记） |
| **ENG-06** | 测试工程设计 |
| **ENG-08** | 第一阶段开发任务清单（Sprint 1~6） |
| **ENG-09** | 类型与命名冻结表（**类型/命名/单位的唯一权威**）；当前版本 **V2.4（取帧语义与触发读回版）** —— 在 V2.3 的 §5.28~§5.33 与附录 A/B 之外，新增帧状态来源与 `NoFrame` 依据边界、`GrabResult` 的本次诊断字段、`TriggerFeatureReadback`、结果包 `raw_image`、`kNoDeadlineNs` |
| **ENG-10** | 算法链接口与配置注入设计 |
| ~~**SYS-08 §7**~~ | ⚠ **本行原写"重试 / 超时权威"，该依据 2026-09-26 核实为无效** —— SYS-08 V2.1 的 `# 7` 是 **TARGET_FOUND 状态**（§7.1 状态说明／§7.2 执行动作／§7.3 输出／§7.4 失败处理），`§7.5`／`§7.6`／`§7.7` **不存在**。重试／超时/降级的现行约定见下方**勘误块**与 [SYS-08-§7引用勘误.md](SYS-08-§7引用勘误.md) |
| **SYS-15 §4** | 误差预算合成规则权威 |
| **SYS-04** | 接口控制文件 ICD（接口权威）；当前版本 **V2.5（失败边界与清理责任版）** —— 与 **ENG-09 V2.4 同批发布、须成对引用**；⚠ 该版改的是"**被调用时发生了什么**"，冻结签名**一个字没动** |
| **SYS-05** | Data 数据结构设计（设计层叙述，**类型与字段的唯一权威是 ENG-09**）；当前版本 **V2.3（权威引用同步版）** —— 只把"以哪一版为准"改指 **ENG-09 V2.4**，正文一字未动 |
| **SYS-06** | Device 设备抽象层设计（设计层叙述，**接口面的唯一权威是 SYS-04 §6.1**）；当前版本 **V2.4（权威引用同步版）** —— 只把现时声明改指 **SYS-04 V2.5 §6.1／§6.2** 与 **ENG-09 V2.4**，接口面与正文一字未动 |
| **SYS-17** | 系统配置与参数管理设计（**键表的权威在本档 §5／§9**，键的类型／默认值／校验规则权威是 ENG-09 §6.1／§6.5）；当前版本 **V1.2（权威引用同步版）** —— 只把现时声明改指 **ENG-09 V2.4 §6.1／§6.5**，两张键表一字未动 |
| **SYS-01 §24** | **软件生命周期阶段**与逐阶段完成判据（阶段权威） |
| **SYS-02 §5** | **各阶段的需求范围**与虚拟 / 真实件构成（阶段范围权威） |
| **SYS-18 §3** | **各阶段部署形态及相应约束**（部署形态权威） |

> **三个入口分工（2026-09-24 补）**：`SYS-01 §24` 回答"**处于什么阶段**"，
> `SYS-02 §5` 回答"**该阶段需要什么**"，`SYS-18 §3` 回答"**该阶段如何部署**"。
> 三者合起来才是对"当前该做什么"的完整回答；单独引任何一条都会缺一角
> （三个入口的互相冲突另见 [待裁决问题汇总.md](待裁决问题汇总.md) PH-01）。

> ### ⚠ 勘误块：SYS-08 `§7.x` 为悬空／撞号引用（2026-09-26 复核）
>
> **本 README 全篇（以及本仓库 `src/`／`tests/` 的注释）长期引用的 `SYS-08 §7.1`～`§7.7`，经核实全部无效**：
>
> - `§7.1`／`§7.2`／`§7.3`／`§7.4` —— **节号存在但内容是别的**（`# 7` 是 TARGET_FOUND 状态：
>   状态说明／执行动作／输出／失败处理），本仓库引用的"三级超时／失败三分类／次数表／回退预算"
>   与其**毫无关系**（**节号碰撞**）；
> - `§7.5`（硬件降级）／`§7.6`（`RetryManager` 接口）／`§7.7`（恢复路径汇总）—— **节根本不存在**
>   （SYS-08 V2.1 只到 `# 22`，每个 `# N` 最多到 `.4`）；
> - 另：`# 17 异常处理设计`（§17.1 相机异常）原文是"**断连／无帧／超时 ⇒ 进入 FAILED**"，
>   与本仓库现行的"2 路可用 ⇒ 降级继续"**正面冲突**。
>
> **处置**：不裁定策略（待裁决 **Q-D1**／**Q-D2**），但**让每个引用可见地失效** ——
> 正文规范性引用处带 `〔引用无效·依据待裁决·见 Q-D2〕`，含描述性引用的源码文件各加一行文件头勘误块。
> 逐条登记、四类处置与实测分布见 **[SYS-08-§7引用勘误.md](SYS-08-§7引用勘误.md)**。
>
> ⚠ **引用本文档 §6 的任何一行时**：那些"依据 SYS-08 §7.x"的处置**仍然有效**（它们是已实施的工程决定，
> 有对应代码与测试），但**其"冻结依据"是空的** —— 不得据其引用反推"冻结文档里规定过"。

### 偏离登记

`2.md`~`11.md` 是**工作流文档**（逐阶段生成指令），`项目文档/` 下的 ENG/SYS
系列是**冻结设计文档**。二者冲突时以冻结文档为准；两份冻结文档冲突时取
**更具体**的一节。以下是全部偏离，每一条在对应源文件的文件头注释里都有
同样的说明。

> **需要裁决的缺口另有一份汇总**：**[待裁决问题汇总.md](待裁决问题汇总.md)**。
> 本节的偏离表记录的是"文档这么说、实际那么做"（已有处置），
> 那份文件记录的是"**冻结文档合起来仍未覆盖某种决定**"（尚无处置，等结论）。
> 两者互补：偏离表里标着"**需要裁决**"的行，都能在汇总文件里找到对应的编号。
>
> 汇总中的一部分已起草结论并**获批**，见
> **[V2.1-C01_架构裁决变更说明.md](V2.1-C01_架构裁决变更说明.md)**（C-001~C-009 + H-001/H-002，
> **已批准 2026-09-23**；v1.3 记实施中核实出的 4 处原文与实现不符；
> **v1.5 新增 C-013 / C-014**，来源＝本轮全仓静态审查报告的 R05 / R02）与
> **[V2.1-C02_实施设计说明.md](V2.1-C02_实施设计说明.md)**
> （C-002 的落地设计，**已批准（2026-09-23），Step 1–6 已实施完成**；
> 实施记录与该文件 §12.5 的逐条关闭状态为实现依据）。批准后的条目已回写本节对应行。

| # | 工作流文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| 1 | Phase 3 生成 `src/interfaces/`，含 `ICameraBackend.h`、`IMultiCameraManager.h`、`ITriggerController.h`、`ITurntableController.h`、`IPoseEstimator.h` | **不建 `src/interfaces/`**。接口随使用方落位（`device/camera/ICameraBackend.h` 等） | ENG-01 §4 的 src 模块表无该模块；ENG-02 §5、ENG-03 §12.3 均将接口置于使用方旁。1.md 自称冻结范围仅至 ENG-01~ENG-08，早于 ENG-09/ENG-10，已过时 |
| 2 | 同上，`IPoseEstimator.h` | **不实现** | 该名称在 22 份设计文档中**从未出现**，属 1.md 自创，未进入冻结基线。若确需，须先走 ENG-09 §8 变更流程登记裁决 |
| 3 | 目录树未含 `src/runtime/` 的构建 | 建 `libruntime` 但**不链入 app** | ENG-01 §4/§12 有该模块（ENG-01 是目录结构权威）；ENG-03 §3/§12.9 两处遗漏，依赖图（ENG-01 §17）中亦无它。当前无消费者，不扩大交付物 |
| 4 | ENG-03 §12.1：`data` 生成 libdata，**依赖：无** | `libdata` 链接 `aps::opencv`（PUBLIC） | ENG-09 §5.1/§5.2/§5.5 冻结的 `Transform`（`cv::Matx33d` / `cv::Vec3d`）、`CameraCalibration`（`cv::Mat`）、`ImageFrame`（`cv::Mat`）等**公开头文件直接暴露 cv:: 类型**，不链接则任何包含它们的编译单元都编译不过。§12.1 的"无"应读作"无本项目模块依赖"——§12.5 给 algorithm 写的是"data + OpenCV"，可见该列表本就混列模块与第三方库。OpenCV 是第三方引擎，不参与模块依赖图，故 ENG-01 §17 的分层与 §18 的禁止依赖均未被破坏。**由编译探针实测发现，建议 ENG-03 §12.1 补一行 OpenCV 以与 §12.2~§12.8 对齐。** |
| 5 | ENG-01 §5 的 `data` 文件清单 | 增加 `MonotonicClock.h`（`nowNs()`） | ENG-09 §2.5 冻结时间戳为宿主 `CLOCK_MONOTONIC` 纳秒，§3 多处类型引用之，属冻结语义所必需的工具头。不建则各模块各写一份 `clock_gettime` 包装，单位与基准迟早漂移 |
| 6 | 4.md：`MultiCameraManager` 无形参构造，仅 `capture()` | 构造注入三个 `ICameraBackend`，并增加 `enabledCount()` / `isDegraded()` / `lastError()` 三个查询 | SYS-08 §7.5 要求"2 路降级、≤1 路 FAILED(1001)"成为可上报给状态机与 UI 的**结论**，而非仅内部行为；无形参构造则无法注入虚拟后端，ENG-06 要求的无硬件测试无从实现 |
| 7 | 3.md / 5.md / 8.md 的片段用 `CameraRole::UNKNOWN` | 枚举只保留 `CAM25` / `CAM50` / `CAM100` | ENG-09 §4.1 冻结为三值（权威）。"无可用相机"用返回值或 `ErrorInfo` 表达——哨兵值会被 `switch` 静默漏掉，且使"焦段 → 标定"映射表凭空多出一行 |
| 8 | 5.md §六：焦距写作 25 / 50 / 100 | `focalLengthMetres` = **0.025 / 0.05 / 0.1**（米） | ENG-09 §2.3 冻结长度单位为米，并明示"100 mm 镜头写作 0.1"。毫米被当米用会让重投影误差放大 1000 倍，而程序**仍能跑通**——本项目最需防的就是这类静默量纲错误 |
| 9 | 5.md §九：`CoordinateTransformer` 持 `rigToShip` 拷贝 | 签名为 `transform(CameraRole, const CameraPose&) const`，`rigToShip` 每次向 `OpticalRig` 取 | 标定可在运行期重载（SYS-04），持副本会让"重标定后仍用旧外参"变成无报错的错误结果。签名不涉及单位与命名，属实现层选择 |
| 10 | 5.md / 6.md 未指定同步超差的错误码 | ~~超差返回 `false`，`lastError().code == 0`，不编造码~~ → **2026-09-23 起改置 `3002`（`kErrSyncOutOfTolerance`）** | 原处置：ENG-09 §5.27 的 3000 段只占用了 3001，**无**"同步超差"码位，且裸字面量被同节禁止，故记录事实并在消息中说明。**裁决 C-006（v1.3）已补入 3002**，本行随之变更。理由：它是**可重试的瞬态**，与 9004"无更具体码的兜底"语义不同；共用一个码会让"同步超差"与"真的无法归类"在结果包里分不开。见 [C-01 §C-006](V2.1-C01_架构裁决变更说明.md) |
| 11 | SYS-09 §15 / ENG-05 §15："QThread + QObject Worker" | `libpreview` **不链接 Qt**，`PreviewWorker` 用 `std::thread`；信号槽留给 ui 层适配 | ENG-03 §12.4 将 preview 依赖冻结为 `data + optical`，ENG-01 §17 把 Qt 归入 ui 层，6.md §12 的 CMakeLists 亦只链数据层。两份冻结文档冲突时取更具体的 §12.4；§15 的两条实质意图（不继承 QThread、生命周期可控可测）均已满足，且使预览测试无需事件循环与 xvfb |
| 12 | 6.md §五 / §十二出现 `PreviewMode` 与 `PreviewProvider` | `PreviewMode` 并入 `PreviewConfig.h`；**不建** `PreviewProvider.h` | ENG-01 §7 把 preview 冻结为恰 4 个文件，ENG-09 §249 明示"唯一变更为 MeasurementSelector"——这两个类型名在全部冻结文档中从未出现。其解耦意图由 `PreviewManager::getFrame` 保留为 `virtual` 实现 |
| 13 | 6.md §五：`PreviewQueue` 保存 `ImageFrame` | 保存 `data::PreviewFrame` | ENG-09 §5.7 冻结 `PreviewFrame { ImageFrame frame; uint64_t displayTimestamp; }`，它存在的意义就是"附图显示时间戳的待显示帧"。若只存 `ImageFrame`，`displayTimestamp` 无处安放，UI 便无法判断画面新旧（相机切换后尤甚） |
| 14 | 6.md §五：队列容量由调用方给定 | 构造时夹取到 **[3, 5]** 并置 `clamped()` 供上层记录 | SYS-09 §13.1 冻结容量 3~5。容量 1 会让丢帧率随 UI 刷新率上升，容量 100 会让"实时预览"延迟到秒级——两种错配都不报错，只是名不副实 |
| 15 | 7.md：`MeasurementController::run()` 用 `while (running_)` 阻塞循环 | 改为**事件驱动** `bool tick(uint64_t nowNs)`，由调用方（009 阶段的 QTimer）驱动 | SYS-08 §9 把状态机放在 Application 线程，而 ENG-03 §12.6 冻结 application **不链接 Qt**——阻塞循环跑在哪个线程都会冻结 UI 60 s；且 `while` + 普通 `bool running_` 是数据竞争（SYS-09 §9 禁止）。§7.6 约束 3 本就要求"每次事件循环检查 `deadlineExceeded()`"，即事件驱动是该节的设计意图 |
| 16 | 7.md：`tick()` 自行读时钟 | `tick(nowNs)` / `startMeasurement(nowNs)` 由调用方注入时刻 | 没有注入就没有 SYS-08 §10 要的收敛性测试："T_task = 60 s ±1 s 内终止"在真实时钟下只能靠睡眠等待，而注入后可在毫秒内**精确**判定（见 tests/integration 的 60 s 用例）。§10 原文只要求"`T_task` 必须可注入"，时刻注入是同一意图的延伸 |
| 17 | 7.md：`MeasurementState state()` | `data::MeasurementState state() const` | 仅加 `const`。全部冻结调用点仍编译通过（§7.6〔引用无效·依据待裁决·见 Q-D2〕 的查询族本就都是 const），且使"从 const 控制器读状态"成为可能 |
| 18 | `MeasurementController.h` 的公开面 | 新增 `IRecorderSink` 抽象（`bool save(const MeasurementTask&)`）与 `notices()` | SYS-04 §265 要求 `MeasurementController → MeasurementTask（值传递） → RecorderWorker`，而 RecorderWorker 属 infrastructure（009 阶段），application 不得依赖它（ENG-01 §17）。以最小抽象反向注入是唯一不破坏依赖图的接法；`notices()` 承载"降级/限位未生效/未挂记录器"等**必须可见**的既成事实（§7.5 要求降级在 UI 可见） |
| 19 | SYS-08 §5.26 `MeasurementTask` 无 `degraded` / `cameras_available` 字段 | **`MeasurementTask` 仍按冻结定义不改**；两个字段改由 **`MeasurementRecord`（data 层，C-002 新建）** 承载，测量结束时**取值冻结**，`result.json` 由 Recorder 从 `record.degraded` / `record.camerasAvailable` 读出 | §7.5 要求 result.json 记录 `degraded=true` 与 `cameras_available`，但 ENG-09 §5.26 / SYS-05 §12 冻结的结构体里没有对应字段。**2026-09-23 裁决修正了原处置**：原先"由 Recorder 落盘时去问控制器"**不成立** —— 控制器持有的是**瞬态**，任务结束后 `degraded()` 反映的是"此刻"而非"这次测量当时"，若期间发生过任何一次降级变化，写进包里的就是错的值**且不报错**。裁定"这两个字段不是状态机状态，而是**本次测量事实**"，与 `taskId` 同级，故进 `MeasurementRecord`（改 data 层结构已走 C-002 裁决，未动 `MeasurementTask`）。见 [C-02 §12.1](V2.1-C02_实施设计说明.md) |
| 20 | SYS-08 §7.5："2 路降级" 的可用相机数来源 | 由同步帧中**图像是否为空**判定，不经 `IMultiCameraManager` 查询 | 冻结的 `IMultiCameraManager` 无可用性查询方法，而 `MultiCameraFrame` 的三路 `ImageFrame` 天然携带"这一路是否取到图"。用空图表达"不可用"不新增接口，也与 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕"禁用故障相机"的语义一致 |
| 21 | SYS-08 §7.3 / §7.4 的"取先到者" | 保留原样，但实测确认 **9002 在 §10 的两个用例中均不可达**（先到者恒为某个状态的次数上限） | §7.3 的次数上限、§7.4 的双预算、§7.6 约束 2（"**所有状态进入时**必须调用 `beginAttempt()`"）三条同时生效时，§7.4 的边预算几乎总被更早触发。逐 tick 追踪结果见 `tests/integration/MeasurementFlowTest.cpp` 用例 3 / 用例 4 的注释。**这不是实现取舍**：本仓库严格实现了 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2。需要一条裁决指明 9002 何时才应可观测（或调整 §10 用例 3 的期望码）。实测可到达 9002 的唯一路径是 `CAPTURE → MEASURE_SELECT` 边（CAPTURE 是唯一上限 > 2 的回退起点），已由 `tests/unit/StateMachineTest.cpp` 覆盖 |
| 22 | SYS-08 §5.3 / §7.7 未给 `TARGET_FOUND` 失败路径 | 该状态失败**就地 FAILED**（码 0，消息指明状态），不设回退边 | §7.7 是无恢复行的状态即无经批准的恢复动作。曾实现"回退至 SEARCH"，但 §7.3 给 `TARGET_FOUND` 的上限 1 与 §7.6 约束 2/5 叠加后，该状态在一生中只能被进入一次——回退到 SEARCH 会使唯一前进边被永久拒绝，任务空转到 T_task 并报出**指向错误方向**的 9001。需要一条裁决：§7.3〔引用无效·依据待裁决·见 Q-D2〕 的"上限 1"是否只约束**状态内重试**而不约束回退后的再次进入 |
| 23 | SYS-08 §7.3 的"不设上限" | 哨兵值取 **-1**，不用 0 | 0 会让配置里任何一处把上限误写成 0 变为**无限重试**，只在 T_task 到点时以无指向性的 9001 收场；-1 使配置中的 0 恢复字面语义"一次都不放行"，与 §7.4 对预算 0 的处置（首次回退即拒绝）一致 |
| 24 | SYS-10 §8：`Δθ = arctan(Δpixel / f)` 的前置校验 | 增加两条：靶面尺寸必须已知；`fx > imageWidth/2`（视场 < 90°） | `data::CameraCalibration` 的默认构造给出 3x3 单位阵 + 尺寸 0，即 **fx = 1**。fx=1 代入 §8 公式得 arctan(200/1) ≈ 89.68°——一个"算得出来"、看上去合法、转台会真的照做的大角度命令。未标定时静默下发 90° 指令是本能导致机械事故的路径，故宁可拒绝并报警（码 0 + 消息）。判据取几何本身而非经验阈值 |
| 25 | SYS-08 §5.7："多帧采集（5~10 frames），**选择最佳帧**" | `selectBestFrame()` **确实执行了评分**，但其结果 `bestFrameIndex_` **只写不读**；PnP 的实际输入是 `lastFrame_`（**最后一次**采集） | 实测：`MeasurementController.cpp:735` 评分 → `:752` 写入成员 → `:790` 用的是 `lastFrame_`；全仓库 `grep bestFrameIndex_` 仅"清零 + 赋值"两处。即**"选择"没有作用到结果上**：M1 的姿态来自第 5 帧，而 `lastQuality_` 描述的是**评分最高的那一帧**——两者不同帧时**记录与被解算的图像对不上，且不报错**。**✅ 已裁决并修复**（2026-09-23）：见 [V2.1-C02_实施设计说明.md](V2.1-C02_实施设计说明.md) §10.1（D-C02-1，已实施并做变异验证）。**本行登记的行为修正是：PnP 输入由"第 5 帧"变为"评分最佳帧"，姿态数值随之变化，状态机轨迹不变**；C-02 顺带修掉同一链上的 D-C02-2（只存选定焦段一路，破坏 raw 与 result.json 的同一次测量追溯）、D-C02-3（空帧跳过导致索引错位）、D-C02-4（成员注释与实现不符）、D-C02-5（`selectedScore` 算完即丢），并新增 D-C02-6（三路 `frameId` 一致性判据，**真实硬件前提见 C-02 §11.4，需在 011-A1 前裁决**） |
| 26 | ENG-08 §16：版本基线须在 Qt 启动时可见（`cmake/BuildOptions.cmake:112` 的注释亦如此声称） | 宏 `APS_VERSION_STRING` / `APS_VERSION_STAGE` **已定义，但全仓库无任何引用**；主窗口标题与启动日志均无基线信息 | 全局 `grep APS_VERSION`（`src/` + `*.ui`）实测为空。后果有二：① ENG-08 §16 **未实现**；② "release 构建拒绝 `synthetic` 模型"（C-003 第 3 条）**当前无处可挂**。**已裁决待实施**：见 [V2.1-C01_架构裁决变更说明.md](V2.1-C01_架构裁决变更说明.md) §C-003 的 O-18（已定：新增 4 个 `APS_IS_*` 数值宏）。⚠ 实测 `grep -rn "APS_IS_" src/ cmake/` 仍为空 ⇒ **尚未实施**；且 C-003 裁决的是 `models/aircraft/synthetic/` + release 拒绝加载，**不是真实机型库** |
| 27 | `MeasurementRecord.statistics`（C-02 §2.5 设计）期望从 `pipeline_.lastMatchResult()` 取值 | **该访问器不存在于控制器所持有的接口上**：`lastMatchResult()` 只有具体类 `PosePipeline` 有（`src/algorithm/PosePipeline.h:176`），而 `MeasurementController` 持的是 `IPosePipeline&` | 实施 C-02 Step 5 时实测发现。后果：`MeasurementStatistics` 的 6 个字段**一个都取不到**，若跳过则记录里全为默认 0——而 0 在这里是**合法值**，属本项目反复出现的"字段存在、类型合法、数值看起来正常，但语义不成立"。**需要裁决**：给 `IPosePipeline` 增纯虚 `lastMatchResult()`（接口头自身规则允许增纯虚，`setCoarseAttitude` 是先例）**会动到本次评审明令不动的"算法层公开接口"**，故 Step 5 暂停等结论。详见 [V2.1-C02_实施设计说明.md](V2.1-C02_实施设计说明.md) §11.1。**✅ 已裁决并实施**（2026-09-23）：**否决**增 `lastMatchResult()`，改用**独立统计输出通道** `IPipelineObserver::onStatistics()`，算法在**产生统计量的那一刻**推送；裁决条文为"**C02 允许修改算法接口，但只能增加'结果输出'，禁止增加'内部状态查询'**"。`solvePose` 签名一字未动。见 [C-02 §12.1/§12.2](V2.1-C02_实施设计说明.md) |
| 28 | ENG-09 §5.5 的 `MultiCameraFrame` 只有三路 `ImageFrame` + `triggerTimestamp` | 新增第 5 个字段 `uint64_t exposureIndex` + `exposureIndexDegraded()`；虚拟路径 `= acquisitionId`，真实路径 `= frameId − baseFrame`（基准锁定），**取值规则与降级判据见 [C-02 §12.2](V2.1-C02_实施设计说明.md)** | C-02 §10.2 原按评审追加要求**断言三路原始 `frameId` 相等**，但该等式在真实硬件上**必然为假**（三台相机上电时刻不同，各自帧计数器起点不同），强行断言只会诱导日后放宽或删掉这条安全网。**2026-09-23 裁决撤回该等式**（"这个在真实系统错误"），改为断言三路 `exposureIndex` 相等 —— 语义更强（测的是"同一次曝光"，不是"两台相机碰巧数到同一个数"）。⚠ **011-A1 硬性前置**：`ImvCameraBackend` 必须给出**单调** `frameId`，且 `baseFrame25/50/100` 首帧锁定**必须在 011-A1 内完成**，否则降级标记全程置位、该字段失去判别力。（原文写"011-A0"是**旧编号**；该两项由后端产出，随"后端实现"从 011-B 改为 011-A1，`011-A0` 已于 2026-09-23 完成、不含此项。） |
| 29 | 8.md §十的 `PosePipeline` 诊断接口（`lastPnpStats` 等） | **6 个访问器全部删除**（2026-09-23，随 C-008 同批）。判据是**消费者实测**而非"看起来没人用"：`lastCadResult()` / `lastCadAssisted()` 全工程**零消费者**（生产与测试都没有），其余 4 个**只有测试**在用，而它们查询的是私有成员 —— 测试读被测对象的内部状态，等于把测试与实现绑在一起。**私有成员全部保留**（`lastMatchResult_` 仍是 `publishStatistics()` 的数据源） | 它们**不属于 IF-SW-02**（SYS-04 §4.2 冻结的调用面只有 detect / estimateScale / selectCamera / selectBestFrame / solvePose / validate），故删除**不需要动任何冻结接口** —— 这正是"不属 IF-SW-02"这句话的实际价值。与 C-02 裁决"**禁止**给 `IPosePipeline` 增加此类访问器"是同一条判据的两面。⚠ **一处未取消的要求**：ENG-10 §2.4 / §4.4 要求把 `cad_assisted` 与 M_hist 三元组 + 冷启动标记落进 result.json，这**不是诊断接口**而是**尚未实施的落盘字段**；实施时须经推送通道（`IPipelineObserver`）暴露，**不得**恢复查询接口。已登记为开放项（[汇总 §5.1 N-2](待裁决问题汇总.md)）。测试侧的改动是**加强**而非削弱：`PipelineTest` 原先拿 `lastMatchResult()` 当基准属**自比较**（推送载荷与查询读同一个私有成员），改为对照夹具**输入侧**的预置值（独立预言机）。实测证明：把 `MockFeatureMatcher` 的 `spreadPx` 改成忽略预置值，自比较版本**毫无反应**（0 失败），新预言机当场转红 |
| 30 | `RecorderPackageTest` 文件头登记"`RecorderSinkAdapter` 的填充**没有自动化判据**"，且该文件里对 `config_snapshot/` 的断言**只验目录存在**（失败消息却写"复盘时要用当时的阈值"） | **2026-09-23（C-009）新增 `tests/integration/RecorderAdapterTest.cpp`（7 用例）**，把这一层补上：适配器补齐的三个外部事实**是否覆盖**（而不是透传）、测量事实**有没有被改坏**、`config_snapshot/` 里**到底有几个文件** | ① 文件头那句的**推理**是错的（结论对）："app 目录只有可执行目标、测试链接不到它"—— `RecorderSinkAdapter` 是头里的**内联类**，包含头即可用，**不需要链接 app 的库**（实测仅需编译期版本宏 `APS_VERSION_*`，不需要 Qt）；② 那条只验目录存在的断言已补上实话并指向新套件 —— 它传的 `configDir` 是个不含 yaml 的临时目录，快照**必然为空**，故"复盘时要用当时的阈值"那句话在那里**验不了**；③ 顺带发现：全仓库**没有任何生产代码**给 `record.modelType` 赋值，故适配器对它是**透传**而非强制清空 —— `ApplicationContext.h` 里"留空"那句描述的是**当前结果**，不是一条被执行的约束。已用正对照钉住（变异 M28 只被该对照抓住，其余断言全绿）。见 [C-01 §C-009](V2.1-C01_架构裁决变更说明.md)、[汇总 §5.1 N-6](待裁决问题汇总.md) |

### 011-A0 SDK 环境准备

| # | 原文 / 声称 | 实际 | 依据 |
|---|---|---|---|
| 31 | `cmake/FindImvSdk.cmake` 与 `cmake/FindTurntableSdk.cmake` **存在即生效**（ENG-03 §7 要求依赖集中管理、§10 要求 ImvSdk 配置 discoverable；`Dependencies.cmake` 也确实调了 `find_package(ImvSdk QUIET)`） | ⚠ **两个 Find 模块从写下来那天起从未被加载过一次** —— 工程**从未设置** `CMAKE_MODULE_PATH`，而 `find_package` 的 MODULE 模式只查该变量与 CMake 自带 Modules 目录。实测 `--debug-find-pkg=ImvSdk` 只搜索了 `/usr/share/cmake-3.31/Modules/`。⇒ `-DIMV_SDK_ROOT=<真实路径>` **不可能生效** | ✅ **已修**（2026-09-23）：`Dependencies.cmake` §0 增加 `list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")`，并留下缺陷记录注释。⚠ **它长期未被发现的原因值得登记**：失效的**表现与预期状态完全一致**（打印"未找到 ImvSdk（非致命）… 拿到 SDK 后 `-DIMV_SDK_ROOT=`"，而"Sprint 2 之前本来就没装"正是当时的真实情况）—— **失败信息看不出是"没装"还是"查找器压根没跑"**，属本项目"断言没验它自称要验的东西"同族。见 [核验报告 §6.5](V2.1-011A0.1_ImvSDK环境核验报告.md) |
| 32 | 给出 `-L<runtime> -lMVSDK` 即构成可用的链接配置 | ⚠ **链接通过、启动即失败**：现代 binutils 默认 `--enable-new-dtags`，`-Wl,-rpath` 生成 **RUNPATH**，而 **RUNPATH 不传递**（只覆盖可执行文件**直接**链接的库，不覆盖那些库自己再加载的依赖）⇒ 可执行文件找得到 `libMVSDK.so`，但 `libMVSDK.so` 找不到 `libGCBase_gcc421_v3_0.so`，退出码 127。**只有真实调用 SDK 符号的探针才能发现**（空壳探针会被 `--as-needed` 优化掉，反而得到"无 DT_NEEDED"的假象） | ✅ **已修**：`aps::imvsdk` 加 `$<BUILD_INTERFACE:-Wl,--disable-new-dtags>` → **DT_RPATH**（可传递）。⚠ **安装态（`deploy/`）未修**：`CMAKE_SKIP_INSTALL_RPATH ON` 会清空 RPATH，而没有任何机制为 `deploy/bin` 设 `LD_LIBRARY_PATH` ⇒ **现场解压后启动失败**。登记为 **011-B 必办**。见 [核验报告 §6.4 / §7.2](V2.1-011A0.1_ImvSDK环境核验报告.md) |
| 33 | 厂商 `share/C/IMV/*/Makefile` 的 `-L../../../../lib -lMVSDK` 可直接照抄 | 该写法按 `install.sh` **安装后**的布局写；本工程按 O-19 **只解包不安装**，主库在 `lib/m64x86/` 且**无 `libMVSDK.so` 软链** ⇒ 照抄必然连不上 | 走 `FindImvSdk.cmake` → `aps::imvsdk`。见 [核验报告 §5.1](V2.1-011A0.1_ImvSDK环境核验报告.md) |

### SYS-04 ICD 版本更新（V2.2 → V2.3）

**2026-09-24 裁决**：SYS-04 由 V2.2 更新至 **V2.3**，原因是**代码一致性修正**。

SYS-04 V2.2 的最大问题不是接口缺少，而是：

```text
设计接口  ≠  实际代码接口
```

后果是三条同时发生：Agent 实现时参考错误、测试无法据此确认、后续硬件接入风险增加。
V2.3 解决的是这个根问题 —— **从"设计预期接口"回归到"代码真实接口"**。

**主要变化**：

* **删除不存在的接口** —— `IRecorderSink::lastErrorText()`、
  `CalibrationManager::getCameraToRig()`、`PosePipeline::process()`；
* **修正类型名称** —— 消除 5 个代码中零命中的类型名
  （`CalibrationPackage` / `CameraRigTransform` / `RigShipTransform` /
  `ErrorCode` / `ScaleEstimate`），3 个虚构接口类名去 `I` 前缀收敛为具体类名
  （`ICalibrationManager` → `CalibrationManager` 等）；
* **对齐真实数据结构** —— `ImageFrame` 删 `gain`/`valid`（⚠ V2.4 起 `ImageFrame`
  **重新增了三个字段**：`raw`/`captureFormat`/`rawPolicy` —— 与本行不矛盾：
  本行删的是"代码里没有的 `gain`/`valid`"，V2.4 加的是"代码里真实存在的新字段"）、
  `MeasurementRecord` 由 6 概念补为 14 字段、
  `MeasurementController` 公开面补全。

**方向**：文档向代码收敛（"不要为了文档迁就代码，采用实际架构"）。
**不改变软件架构、不新增接口、不改变代码。**

| 文件 | 状态 |
|---|---|
| `项目文档/系统设计/SYS-04_接口控制文件ICD_V2.3_代码一致性修正版.md` | **唯一有效版本** |
| `项目文档/系统设计/archive/SYS-04_接口控制文件ICD_V2.2_多相机闭环测量版.md` | 已归档，保留审计链，**不再作为接口依据** |

> ⚠ 归档而非删除：项目当前无版本管理仓库，直接删除冻结文档会同时失去历史与审计链。

> ⚠ **本节已被后续批次取代**（2026-09-26 补）：SYS-04 已升版至
> **`项目文档/系统设计/SYS-04_接口控制文件ICD_V2.4_取帧契约与诊断出口版.md`** ——
> **它才是当前的接口权威**，V2.3 已 `cp -p` 归档进同级 `archive/`。
> 上表"V2.3 为唯一有效版本"的表述是**该批次的当时状态**，此处**保留原样不追改**
> （历史引用不全局替换编号），只加本行指向现行版本。

> ⚠ **本节再次被取代**（2026-09-26 批二补）：SYS-04 又升一版至
> **`项目文档/系统设计/SYS-04_接口控制文件ICD_V2.5_失败边界与清理责任版.md`** ——
> **它才是当前的接口权威**；V2.4 已 `cp -p` 归档进同级 `archive/`。
> 本版与 **ENG-09 V2.4（取帧语义与触发读回版）同批发布**，两者须**成对引用**：
> SYS-04 管"**调用责任**"（失败边界、清理责任、三个预算检查点、触发前置判定），
> ENG-09 管"**类型与命名**"（`frameStatusRaw`／`diagnosis`／`TriggerFeatureReadback`／`kNoDeadlineNs`／
> 结果包 `raw_image`）。只引其一都会缺一角。
> 裁决正文见 [V2.1-C01_架构裁决变更说明.md](V2.1-C01_架构裁决变更说明.md) **C-016**。

> ⚠ **同批另有三份文档升版（"权威引用同步版"）**（2026-09-26 批二收尾）：
> **`SYS-05` V2.2 → V2.3**、**`SYS-06` V2.3 → V2.4**、**`SYS-17` V1.1 → V1.2**，旧版均已 `cp -p` 入各自 `archive/`。
> ⚠ **它们升版的原因不是"内容变了"，而是"当前权威声明过期了"** —— 三份文档里写着
> "以 ENG-09 **V2.3** 为准"／"接口唯一权威是 SYS-04 **V2.4**"这类**裁决规则**（不是历史修订记录），
> 而旧版均已入 `archive/`；冲突时它们会把读者**指回旧版**，且 SYS-04 V2.5 改的恰恰是
> `ICameraBackend` 的**调用责任**。⇒ 同批改为指向 **ENG-09 V2.4／SYS-04 V2.5**（**正文一字未动**，
> 历史修订记录的旧版本号**保留**）。**"内容安全"不等于"可以不改"** —— 只以"无冲突"结案会让
> 引用链停在旧版本上。逐处清单见 [011-A1_修改位置对照表.md](011-A1_修改位置对照表.md) §11.3。
> ⚠ 同批另将 **ENG-09 V2.4 指向 `SYS-04`／`SYS-06` 的 3 处**与**`camera.yaml` 键表归属**
> （`SYS-17 V1.1` → **V1.2**）一并校正。

**未随本次修正关闭的两项**（见 V2.3 §26）：

| # | 项 | 状态 |
|---|---|---|
| 1 | **IF-SW-02 冻结面缺失** —— `IF-SW-02` 与「SYS-04 §4.2」被本文件 §6、代码注释、`V2.1-C01` 共 11 处引用为冻结权威，但该节在 SYS-04 中**不存在**（V2.3 §26.1） | 独立立项：`IF-SW-02_PipelineObserver接口一致性修正`。性质是**接口能力缺失**，不是接口文档错误，故不塞进 ICD |
| 2 | 本文件 §6 各行中出现的「SYS-04 §4.2」引用 | 待第 1 项立项后统一改为真实存在的节号 |

**支撑材料**（在 `AircraftPoseSystem/`）：

* [V2.1-SYS04接口×代码一致性审计.md](V2.1-SYS04接口×代码一致性审计.md) —— 21 个声明点的逐条判定与裁决登记；
* [V2.1-SYS04不一致逐条对照.md](V2.1-SYS04不一致逐条对照.md) —— 文档原文 × 代码原文并排，供核对。

### SYS-01 / SYS-02 / SYS-18 阶段内容补充（2026-09-24）

**2026-09-24 裁决**：按「SYS-04 ICD接口修正 → SYS-01 生命周期阶段 → SYS-01 资产链 →
SYS-02 阶段需求 → SYS-18 部署阶段 → 011-A1 真实相机」的顺序推进。本步完成第 2~5 项。

三份文档均为**增量修改，未重新生成全文**：

| 文档 | 新增 | 顺延 | 归档 |
|---|---|---|---|
| `SYS-01_软件总体架构设计_V2.2_多相机闭环测量版.md` | §24 软件生命周期阶段（四阶段 + 完成判据）、§25 工程资产链（Calibration / Target Model / Configuration） | 原 §24 → §26 | `archive/..._V2.1_...md` |
| `SYS-02_软件需求规格说明书_V2.2_阶段需求补充版.md` | §5 阶段需求 | 原 §5~§15 → §6~§16 | `archive/..._V2.1_...md` |
| `SYS-18_系统部署与运行环境设计_V1.1（部署阶段补充版）.md` | §3 部署阶段（开发部署 / 最终部署 / 中间形态） | 原 §3~§21 → §4~§22 | `archive/..._V1.0_...md` |

**SYS-01 §24** 把阶段与虚拟件的对应写成一张逐阶段表，并冻结**替换顺序：先单件，后组合** ——
`Phase 2 只允许 CAM25 先转真实`，CAM50 / CAM100 保持虚拟。
理由：三路同时接入时，任何一路失败都无法区分是"该路硬件问题"还是"多路协同问题"。

**SYS-02 §5** 据此规定各阶段的需求范围。**SYS-18 §3** 据此区分部署形态：

| 阶段 | 部署形态 | 标定 / 模型 |
|---|---|---|
| 开发部署（Phase 1 / 2） | 全虚拟相机，**或** CAM25 单真实相机 | 均**无真实件**（合成标定 / 无机型库） |
| 最终部署（Phase 4） | 三真实相机 + 真实转台 + 硬触发 | Calibration Package + Production Model |

⚠ SYS-18 V1.0 全文对「虚拟件 / 阶段 / 开发部署 / 生产模型」**零命中** ——
整份文档默认的就是最终部署形态。∴ §13 的环境检查在开发部署下一按就红，
而文档没有一处说明这是正常的。V1.1 §3.1 补上了这条：**开发部署下环境检查必然不全通过，
应作为诊断输出读取，不得当作启动门槛**（一按就红的门槛最后只会被绕过）。

**三处实测登记**：

| # | 登记内容 | 位置 |
|---|---|---|
| **PH-02** | ⚠ **不得表述为"真实实现已就绪、只差装配"**：`ImvCameraBackend` / `HardwareTriggerController` / `PekoTurntableController` **三者均为诚实桩**（每个方法返回 `false` 并说明原因），**且**装配点 `SystemInitializer::buildDevices()` 仍装配全部虚拟件。∴ **本轮针对 CAM25 完成真实后端实现及实际装配接入**；「实现 SDK 适配」与「切换装配点」**必须同时落实** | SYS-02 §5.4 / SYS-18 §3.4 |
| — | 真实转台接入**受文档缺口阻塞**：`PekoTurntableController` 不含 Peko_D 报文构造，因协议帧结构未在冻结文档中给出（SYS-10 只列协议种类） | SYS-02 §5.4 |
| **PH-01** | ⏸ **待裁决（保留）**：转台 / 触发归属哪一阶段，四种来源给出**三种读法**（SYS-01 §24.5 / ENG-08 §3 / ENG-08 §17 / 代码装配点） | SYS-02 §5.5 |

**PH-02 的范围（2026-09-24 明确）**：本轮**只要求当前目标设备真正接入** ——
即针对 **CAM25** 完成真实后端实现及实际装配接入，由 **011-B** 验证该装配路径输出的
真实图像能够进入预览和保存链路。验收清单须记录**实际后端类型、设备序列号、相机角色**，
以证明应用确实在使用真实相机。**其他焦段若仍为虚拟件，必须在运行记录和验收结果中明确标识**；
**混合运行不能作为三相机真实接入或同步通过的证据**。转台、硬触发按 PH-01 与协议缺口单独推进。

**PH-01 的影响范围（2026-09-24 修正）**：PH-01 待裁决的是**转台与硬触发进入真实设备阶段的
范围及完成判据**。当前继续执行已确定的 CAM25 单相机路线：**011-A1 后端实现 →
011-B 真实预览与保存 → 011-C 单相机稳定性**。转台与硬触发的后续排期，
**待 PH-01 裁决后同步相关文档**。

⚠ **证据分类（不得混用）**：
* **SYS-01、ENG-08 的条文**属**阶段范围依据** —— 需要核对各自 "Phase" / "Sprint" 的含义
  （ENG-08 是 Sprint 制，SYS-01 是 Phase 制，两者不是同一把尺子）；
* **代码仍装配虚拟设备**属**当前实现状态** —— 它本身**不能证明阶段要求应该是什么**。
  否则会把"**尚未实现**"与"**阶段不要求实现**"混为一谈。

**PH-01 与 PH-02 已登记至 [待裁决问题汇总.md](待裁决问题汇总.md)**（§5）。
两者均**不阻塞 011-A1** —— 三种读法唯一一致处是「CAM25 单路先真实」，而那正是 011-A1 的范围。

### ENG-07 远端仓库裁决（2026-09-24，C-012）

| # | 冻结文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| **A** | **ENG-07 §2**：「项目采用：本地Git仓库 + 离线备份 + 版本标签 + 发布归档 模式。**不依赖：公网GitHub；公网GitLab；云同步。**」 | **推翻该一概禁止**，改用**公网私有仓库**作远端。**ENG-07 升版 V2.2**（`ENG-07_Git版本管理规范_V2.2_远端仓库补充版.md`），V2.1 移入 `项目文档/软件工程设计/archive/` | **C-012 裁决（2026-09-24）**。⚠ 私有仓库**不能规避**原文 —— §2 禁止的是"公网托管平台"这**一个类别**，与仓库可见性无关，故必须走修订而非"私有即可" |

**修订范围**（严格限于"用什么平台托管"，其余章节一字未改）：

1. 远端**必须私有**；
2. 远端**不得是唯一副本** —— 本地仓库 + 离线备份仍为必备项；
3. **推送前须清理**：内网 IP / 本机路径 / 凭据 / 厂商 SDK 分发物 / 构建产物（新增 §2.3）；
4. 新增 §2.1，明确**本地仓库是唯一权威源**，远端只作备份与协作通道。

⚠ **§18 涉密环境管理未被解除，且与 §2.2 是两道独立闸门** —— §2 解除的是"平台类别禁令"，
§18 约束的是"**内容能不能出门**"。**不得据 C-012 认为"已获准外推"**。

**本裁决的实际落地**：远端 = GitHub **私有**仓库（账号 `Hregl`），仓库根 = `proj_plane/`
（含 `AircraftPoseSystem/` 与 `项目文档/`）；鉴权 = SSH ed25519，`github.com` 主机密钥
已与 GitHub 官方公布指纹**逐条核验一致**后写入 `known_hosts`。清理与本机信息处置的
逐项对照见 [待裁决问题汇总.md](待裁决问题汇总.md) §5「**C-012 的实际落地**」块。

### 007 Algorithm（8.md）

| # | 工作流文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| 25 | 8.md §二 的文件清单：`pose/PoseEstimator.h` + `MockPoseEstimator.cpp`、`scale/MockTargetScaleEstimator.cpp`，且**无** `CadStructureLocator`、无 `IPosePipeline.h` | 按 ENG-02 §11.1~§11.8 落位：`pose/PnPPoseEstimator.h/.cpp`、`scale/TargetScaleEstimator.cpp`（内含 `PinholeScaleEstimator`）；另建 `feature/CadStructureLocator.h/.cpp` 与 `pipeline/IPosePipeline.h` | ENG-02 §11 是 C++ 类与文件规划的权威；`CadStructureLocator` 由 ENG-01 §7 列明（裁决 C-21）并要求"必须独立于 `FeatureExtractor`"。8.md 的 `MockPoseEstimator` 只返回固定姿态，无法承载 PnP 输出与内点统计 |
| 26 | 8.md §四：`DetectionResult` 定义在 `TargetDetector.h`（`int x/y/w/h`） | 用 ENG-09 §5.12 冻结的 `data::DetectionResult`（`cv::Rect bbox` + `CameraRole`），不定义第二个同名结构体 | 两个同名结构体并存时，其定义随 include 顺序静默二选一（ODR 违反且链接期不报错），而字段名/单位还可能不同 |
| 27 | 8.md §六：`ScaleEstimate.pixelSize`，入参只有检测框 | 用 ENG-09 §5.14 的 `TargetScaleEstimate.targetPixelSize`；按 SYS-07 §5.2 补三项入参（bbox 尺寸、相机内参、模型真实尺寸） | 只给检测框时 `Z = f·L/(l·s)` 缺 `f` 与 `s` 两项，距离无从算起；字段名差异会让"毫米当米用"这类量纲错误难以察觉 |
| 28 | 8.md §七：`select()` 返回 `CameraRole`，以 `CameraRole::UNKNOWN` 表示无候选 | 返回 `bool` + `MeasurementSelectionResult&` 出参 | ENG-09 §4.1 只冻结三个角色、无 `UNKNOWN`（同本表第 7 行）。哨兵值会被 `switch` 静默漏掉 |
| 29 | 8.md §八：`TargetModel` 只装一个字符串、`TargetModelManager::model()` 无参 | 用 ENG-09 §5.20 的 `TargetModel`（点表 + 三焦段描述子库），接口为 `get(CameraRole)` | 8.md 的结构承载不了 PnP 的输入；三个焦段各有独立特征库（SYS-12 §8），无参接口无法表达"取哪一路" |
| 30 | 8.md §九：`struct FeatureSet { int count = 0; };` | 用 ENG-09 §5.21 的 `FeatureSet`（`keypoints` + `descriptors`） | 只有一个计数无法做匹配；且 ENG-10 §2.6 要求 B 类（无描述子）也能进入同一接口 |
| 31 | 8.md §十：`MatchResult` 只有计数，`match()` 的模型侧入参为 `FeatureSet` | 改为分项统计；模型侧入参为 `data::TargetModel` | ENG-10 §2.5 的冲突规则、§2.4 的可用性判据与 `matchRatio` 都需要分项；B 类没有描述子（§2.6） |
| 32 | 8.md §十四：`add_library(AircraftAlgorithm …)`，只列 4 个源文件，只链 `AircraftData` | `aps_add_module(algorithm …)` → `libalgorithm.a`，源文件按目录 GLOB，依赖 `data` + `aps::opencv` | 命名与装配方式见 §4.4（001 阶段确立，各模块一致，8.md 的 `Aircraft*` 命名全工程未采用）；依赖集以 ENG-03 §12.5 冻结的"data + OpenCV"为准（与第 4 行同源） |
| 33 | SYS-07 §9 的流程图把 RANSAC 画在**匹配**这一步 | RANSAC 下移到 `PnPPoseEstimator`，`FeatureMatcher` 不做 RANSAC | 几何模型即 PnP，残差必须先有位姿；且 SYS-14 §6.2/§17 要求通道选择在 PnP **之前**完成，匹配阶段不得引入位姿依赖 |
| 34 | ENG-09 §5.23 的 `CameraPose` 只有 `aircraftToCamera` + `reprojectionError` | 不改冻结类型，在返回值之外并列增加 `PnpStats`（内点数 / 内点比例 / 收敛标志） | ENG-09 §5.25 的 `PoseValidationResult` 与 SYS-07 §12.2 都需要内点比例，而 `ShipPoseResult` 里没有它。改 data 层结构需走 ENG-09 §8，故取并列结构 |
| 35 | SYS-04 §4.2 / §7 的 IF-SW-02：`validate(const ShipPoseResult&, PoseValidationResult&)` 要产出 `inlierRatio`/`confidence` | `PoseValidator` **不实现** `IPosePipeline`，作为纯判据函数对象由 `PosePipeline` 组合持有，多两个入参（内点比例、对应数） | 入参 `ShipPoseResult`（ENG-09 §5.24）里没有这两个量，**冻结签名在文档内部无法闭合**（与第 34 行同源）。实现在 `PosePipeline::validate` 中补齐，冻结的多态入口形状未变 |
| 36 | ENG-10 §5.1 的注入矩阵写作 `const&` 注入 | 6 个类（`TargetDetector` / `FeatureMatcher` / `PnPPoseEstimator` / `MeasurementSelector` / `PoseValidator` / `PosePipeline`）**按值**各存一份配置；构造函数入参仍是 `const&` | 存引用时 `const X cfg = f(); Foo foo(cfg);` 之外的写法（直接传临时量）**可编译而对象悬空**，表现为门槛随机生效/失效。007 单测实测踩到（以临时量构造的成员读到的已不是注入的那份配置）。注入面与"注入后不可变"（§5.2 约束 2）均未变 |
| 37 | ENG-10 §2.3 Step 1：粗姿态来自**转台角度 + 距离估计** | `PosePipeline::setCoarseAttitude(az, el, distance)`，由调用方（应用层，009 阶段接转台）注入 | IF-SW-02 冻结的入参里没有转台角度，且算法层不得依赖设备层（ENG-01 §17 / ENG-02 §16），否则只能违反分层。**需要裁决**：把标定与转台角度并入冻结入参，还是允许 algorithm 依赖 optical |
| 38 | SYS-04 的 `solvePose` 入参只有 `CameraCalibration`（内含 cameraToRig），**不带 rigToShip** | 把整份 `OpticalRigCalibration` 作构造注入，由 `PosePipeline` 自行合成三级坐标链（代价：本文件内重复约 30 行 ZYX 分解） | 冻结签名承诺产出 `ShipPoseResult.aircraftToShip`（舰体系结果），而舰体系变换不在入参里；同第 37 行的裁决 |
| 39 | ENG-09 §5.19 的 `FeatureDescriptor::descriptor` 被按 N×D 矩阵理解 | 按"每条特征一行（1×D）"处理；`saveFeatureLibrary` 遇维度不一致时返回 false，**不补零** | 按 N×D 理解时 `descriptors.rows == 1`（每条描述子各自成行），落盘数据只有第 0 条有值、其余全为 0 —— 而读回后特征数、维度、role 校验**全部合法**（实测：3 条入库，第 0 条的断言通过、第 1/2 条断言失败，错误信息只显示"值不对"）。最终表现为"匹配数远低于预期"，与"特征库没生成好"完全同形 |
| 40 | SYS-12 §7 的目录清单含 `model.yaml`；SYS-04 §6.3 要求它记录模型来源/生成时间/特征算法版本 | `loadModel` 把 `model.yaml` 作为**必需项**：缺失即整体加载失败；其 `model_id`/`version` **覆盖** `points3d.yaml` 的同名键 | 两个可独立编辑的身份来源会让 `match_stats` 的分组键（ENG-10 §4.1：换机型必须换统计表）与模型版本悄悄对不上，而两边都"看起来正常" |
| 41 | ENG-09 §5.2 冻结的默认构造 `CameraCalibration` = `Mat::eye(3,3)` + 尺寸 0（即 `f_x = f_y = 1`，**是合法正数**） | `PinholeScaleEstimator::estimate` 与 `CvPnPPoseEstimator::validIntrinsics` 增加"图像尺寸必须非零"的判据 | 只校验 `fx > 0` 时，一个从未赋值的标定对象会静默通过：尺度估计按 `Z = f·L/l` 给出一个由 f = 1 导出的、与真实距离差若干数量级的距离，PnP 给出形状合法、数值荒谬且 `converged = true` 的姿态。**尺寸非零是"是否填写过"的唯一可用判据** |
| 42 | ENG-10 §2.4 只写"亚像素拟合残差 ≤ 0.3 pixel"，未规定统计量 | `CadStructureLocator` 取像素到拟合直线的**中位数**（直线本身用 `cv::DIST_HUBER` 拟合） | 分组依据是梯度方向（倍角聚类），组内必然混入端头/纹理/压缩伪影像素。实测：164 点的组混入 8 个距拟合直线约 41 px 的离群像素，RMS 从 0.5 抬到 **9.476** px —— 0.3 判据退化为"有任何杂散像素就丢弃该结构点" → 结构点全丢 → B 类判不可用（`cad_assisted = false`）→ 退回纯 A 类，**且不报错**。中位数对少数离群点免疫，对"两条相距 1.5 px 的平行棱被并成一组"仍敏感（≈0.75 > 0.3），判别力没有损失 |
| 43 | ENG-10 §2.3："与 CAD **投影线框**做最近邻匹配" | 改为"投影点邻域内提取两条主方向边缘（Canny + 倍角聚类 + 求交）" | ENG-09 §5.20 冻结的 `TargetModel` 只有点表与描述子，**没有棱线连通关系**，投影线框无从构造。改法仍是 §2.3 的"几何不连续"语义（ENG-01 §7 亦要求它与纹理特征分开） |
| 44 | 8.md §十六 的第一阶段运行链：`… → MockTargetDetector → MeasurementSelector → MockPoseEstimator → ShipPoseResult` | 实际算法链含特征提取与匹配两级：`检测 → 特征 → 匹配 → PnP → 校验` | ENG-10 §2.2 的 A/B 两类特征都必须进匹配（缺一类则数量或展布不达标，SYS-15 §4.5），`MockPoseEstimator` 无内点统计。**真正的运行链接线在 009 阶段**，本阶段只保证各阶段可独立调用 |
| 45 | 8.md §十七 要求提交 `feat[algorithm]: initialize algorithm framework` | **未提交**（本目录不是 git 仓库） | 001~007 各阶段均未 `git init`/提交：用户未要求过提交，工程始终"只在被要求时提交"。若需入库，应先 `git init` 并补一次基线提交 |
| 46 | ENG-10 §3.6 的 F 分项要求 `N_detect` 由**特征提取**给出 | 选择阶段**不跑 SIFT**，改用检测框内 Shi-Tomasi 角点数作代理 | 选择必须在昂贵计算之前（§6.2 与 <100 ms 预算），SIFT 在 2448×2048 上远超该预算。代理与真实 SIFT 点数的关系**未标定**，故 F 分项目前只保证单调性，**不保证与 `N_ref = 100` 的可比性**（**待裁决/待标定**） |
| 47 | ENG-10 §3.6 的 E 分项需要 W（结构点展布） | 选择阶段尚无匹配结果，以**检测框长边**作 W 的上界代理 | 使 E 偏小、eNorm 偏大、得分偏高（乐观），但不改变候选之间的排序方向。**需要裁决**：接受该代理，还是把 E 从选择阶段移除 |
| 48 | SYS-07 §12 / ENG-09 §5.25 要求 `PoseValidationResult.confidence`，但**从未定义**它 | 取"重投影误差分项与内点比例分项的 **min**"（最弱一环）；角度先归一化到 (−180, 180] | 冻结文档只给了"≥ `minConfidence`"的判据与 ∈[0,1] 的值域，没有任何算式。取 min 而非加权平均，是因为 `ValidationConfig` 没有权重字段（SYS-14 §20 约束 3 禁止无依据的可调参数）。**必须由裁决确定**，否则两套实现的 confidence 互不可比 |
| 49 | ENG-10 §3.2：`σ_px_est = a + b·(1/S) + c·(1/C)` | S = 清晰度（Laplacian 方差），C = 对比度（归一化到 [0,1]，灰度差 64 视为饱和）；`ImageQuality::exposure` 最佳值取中灰 128 | 002 阶段 `MeasurementConfig.h` 的注释曾把 C 误写成 `targetPixelSize`（§3.2 明写清晰度/对比度），已一并修正该注释。`kContrastSaturation` 与 `minSharpness` 一样是**跨实现不可比**的口径，换口径必须重新整定（**待标定**） |
| 50 | ENG-10 §4.2 的 `illum_band` 分桶 | 边界由 `MeasurementConfig::illumBandEdges` 给出；**未标定（{0,0}）时显式返回 band 1（正常）** | 两条边界相等时按字面判断，所有曝光都落 band 2（"强光逆光"）——数值正常而含义错，会污染 `match_stats` 的分桶，进而污染下一轮的 `mHist` |
| 51 | ENG-10 §2.5 冲突规则："**反投影后**位置差 > 2 pixel 时以 B 类为准" | 判据落在**图像**距离上，且这 2 pixel 同时用作"同一区域"的半径 | "反投影"需要位姿，而位姿要到 RANSAC 之后的 PnP 才存在，与"选择/匹配都在 PnP 之前"（SYS-14 §17）在时序上不能同时成立。取图像距离是可用且同义的形式。**需要裁决**明确该 2 pixel 的定义域 |
| 52 | ENG-10 §2.4/§5.1 未给 `CadStructureLocator` 任何注入项，也未给任何可调参数 | 不注入 `MeasurementConfig`；引用的 0.3 px（残差）与 0.6（展布比）是**冻结判据值**而非新参数；新增 `kMinCrossingAngleDeg = 20°`（两棱线最小夹角） | 20° 不在任何冻结文档中，取值过小会把噪声方向当成第二条棱线（交点飞出窗口）。该常量已在定义处注明"本实现引入"（**待裁决**：是否补入 ENG-10 §2.4） |
| 53 | ENG-10 §5.1 把 `minTargetPixelSize` 注入检测器，但未给取值依据 | 取 50 px，并在注释中给出"须对照最短焦段 CAM25 反推"的方法 | 取值过大会使 SEARCH 永远检不出目标、以 9001（无指向性）收场，排查方向完全错。属**实机标定项**（ENG-08 §5） |
| 54 | ENG-10 §4.4 要求记录 `match_stats_version` | 007 阶段只按 `CameraRole` 取值，缺口写在 `MeasurementSelector.h` 文件头 | 见下方"内部歧义"第 9 条 |
| 55 | 目标的机头朝向（180° 二义性） | 粗姿态**假定进近（迎头）**，不区分机头/机尾 | 转台角度 + 距离无法区分迎头与尾追（两者角度相同）。若实为尾追，B 类定位失败 → `cad_assisted = false` → 退回 A 类（丢精度但**不出错值**）。**需要裁决**：是否引入"机头朝向"输入，或要求 A 类先判 |
| 56 | 冻结文档未规定的若干常量与口径（`featureNN.bin` 的端序假设、`TargetScaleEstimate.confidence` 的参考值、RANSAC 重投影阈值 3.0 px、Lowe ratio 0.75、SIFT 自身参数） | 均在定义处注明"无冻结依据"并给出取值理由 | 这些量必须与实际相机/镜头/机型一起**整定**，不能凭文档推导。特别地：RANSAC 阈值（定内点）与 `ValidationConfig::maxReprojectionError`（定判定）**同名不同义**，取同一数值会让临界点既算内点又判不通过；Lowe ratio（0.75）与 `minMatchRatio`（内点比例门槛）同样同名不同义 |
| 57 | ENG-10 §2.3 期望的边缘拟合精度 0.1~0.2 pixel | **未达到**：实测（合成棋盘格角点）定位偏差**被量化到整数**——把边界用抗锯齿方式滑动半个像素，输出坐标只在 199/200 之间跳变，不随边界连续变化 | 本实现对 `cv::Canny` 的**整数**边缘图做直线拟合，与像素栅格对齐的边缘因此无法取得亚像素位置（斜向边缘的阶梯链在拟合中会自行平均，才有亚像素效果）。实测残差 0.000、四点离散 0.000，即**重复性**很好但**绝对精度**受量化限制。这是本阶段最重要的已知局限，也是 SYS-15 §4.5 那个"最可能不达标的一环"的第一组实测数据（**待办**：在整数边缘之外补一步亚像素细化，例如渐变剖面拟合，或对交点做 `cv::cornerSubPix`）。**2026-09-23 更新**：本行结论已由 010.5 的 ss=8 超采样扫描**独立复现**（偏差 0 → −1.0 px 呈阶梯、`maxResidualPx` 恒为 0），且该扫描确认了本行未涉及的一点 —— **残差判据对量化完全不敏感**，故"残差 0.000"不能作为定位精度的证据。测量框架与判据已冻结于 [V2.1-C005_CadLocatorBenchmark测试设计说明.md](V2.1-C005_CadLocatorBenchmark测试设计说明.md)。**⚠ 2026-09-23 二次更新（本行属暂停前已测得、必须留档的结果）**：该扫描还发现一条**比量化更严重**的结论 —— **方向 θ ≥ 30° 时四个结构点被残差判据全部剔除**（`matched=4, rejected=4`，`available=false`），即 **B 类（CAD 辅助）在对角棱线上整个不可用**，而调用方据此**静默降级为纯 A 类、`cad_assisted` 记 false、不报错**（`CadStructureLocator.h:82-84`）。θ=30° 的残差 0.342 只比冻结的 0.3 pixel 判据高 **15%**，是一道**悬崖**；θ=0° 残差 0.000（整数量化）、θ=15° 偏差 −0.503 恒定而残差 0.248。**注意**：投影与梯度分组**从未失败**，失败**纯粹**是 ENG-10 §2.4 残差判据造成的。上述均在**理想合成图**（无噪声/模糊/压缩）上测得，实机只会更差；且是否部分源自旋转棋盘格构造，须以单边对照分离。**C-005 据此已暂停、改号 013**，见 [V2.1-011A0_真实相机接入实施计划.md](V2.1-011A0_真实相机接入实施计划.md) §0.4 |

### 008 Qt 界面与 main（9.md）

| # | 工作流文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| 58 | 9.md §一 / §五~§九 的文件清单只有 `MainWindow` / `ImageViewer` / `PosePanel` / `StatusPanel` / `main.cpp`，**没有 `TurntablePanel`**；§七 把"转台状态"放进 `StatusPanel` | 实现 `widgets/TurntablePanel.h/.cpp`（方位 / 俯仰 / 状态），`StatusPanel` 不再含转台行 | ENG-01 §13 的 ui 结构图、ENG-02 §14、ENG-04 §14 **三处都列出 TurntablePanel**；`data/TurntableState.h` 的注释亦写明"它是 StatusPanel / TurntablePanel 的显示数据源"。两份冻结文档与 9.md 冲突时以冻结文档为准，且 ENG-04 §14 比 9.md §七 更具体。把转台的运动状态与它的两个角度拆到两个面板，会让"转台到位没有"这一次判断变成两次扫视 |
| 59 | 9.md §八/§九：`MainWindow(QWidget* parent=nullptr)`，窗口不持有任何数据源 | 首参增加 `preview::PreviewManager* = nullptr`，并增加 `setPreviewManager()` | 9.md §十四 展示了一个**能显示图像**的界面，而其示例代码里没有任何取帧路径——中间缺的正是这一处连接。不让 MainWindow 自己 new 一个：ENG-02 §15/§16、ENG-04 §15 把"创建对象"明确禁止在 UI 层（"UI 创建设备"是冻结的禁止项）。默认 `nullptr` 使 9.md §十 与 10.md §九 都写着的 `MainWindow window;` 仍然编译通过（此时显示"无图像"占位） |
| 60 | 9.md §八 的 MainWindow 没有任何控件访问器 | 增加 `viewer()` / `posePanel()` / `statusPanel()` / `turntablePanel()` | 009/010 的职责是"把控制器的输出接到界面上"，必须能拿到具体面板。若不给，MainWindow 就得替应用层决定"哪个状态接哪个面板"——那等于把 009 的接线搬进 ui 层，而 ui 层一旦开始解释测量语义，就违反 ENG-01 §13"UI 不负责相机控制 / 算法计算 / 转台控制"。访问器只交零件、不含判断 |
| 61 | 9.md §五：用 `QLabel* label_` + `label_->setPixmap(...)` 显示 | 改为自绘 `paintEvent` + `painter.drawImage(targetRect, image)` | 两条都是"用 QLabel 会出错"：①相机出图远大于主窗口，QLabel 显示原尺寸时要么把窗口撑大、要么把图像裁掉一角，而 `setScaledContents` 是**不等比拉伸**——测量系统的预览窗口不能有显示失真（操作者据此判断目标是否变形）；②QLabel 只接受 QPixmap（GUI 线程限定、须随尺寸重建），而 QImage 是线程安全且与尺寸无关的，缩放发生在绘制那一刻，不存在"缓存的缩放图与当前尺寸不一致" |
| 62 | 9.md §五 没有清屏接口 | 增加 `clear()` 与"无图像"占位文字 | `PreviewManager::displayGeneration()` 的语义是"显示源一变，上一路的最后一帧就不该再停留在画面上"（其头文件原话）。界面没有清空能力就会出现"新焦段的标签 + 旧焦段的图像"，而这种错配看起来完全像是**测量结果不对**，会把排障方向引到算法上。占位文字同时覆盖"010 首次运行尚无相机出图"——一块全黑的窗口无法与"程序卡死"区分 |
| 63 | 9.md 未规定任何叠加信息 | 增加 `ImageViewer::setOverlayText()`（焦段 / 分辨率 / 预览时延） | `data/PreviewFrame.h` 与 SYS-07 §15 把 `displayTimestamp − frame.timestampNs` 定为"预览不得阻塞算法"这条约束的**可观测量**；不显示出来就只能在事后翻日志。文字由 MainWindow 拼好，控件不求值、不解释含义（保持"UI 只显示、不判断"） |
| 64 | 9.md §六：`updatePose(yaw,pitch,roll)` 直接显示三个数值 | 增加 `clear()`，未解算时显示 `—`；**Yaw 同时给出角分**，Pitch/Roll 只给度 | `data/ShipPoseResult.h` 的 `yaw/pitch/roll` 默认值都是 0.0 —— 直接显示时"解算还没跑过"与"结果恰好是 0°"在界面上**完全一样**。验收指标是 Yaw ≤ 1 角分 = 0.0167°（README 首段 / SYS-15 §4），只给度会让 0.1°（已超差 6 倍）看起来"接近指标"。两个数字是同一个量的两种单位（ENG-09 §2.2），不是新增观测量 |
| 65 | 9.md §七：`setStatus(const QString& text)` 只有自由文本一种形式 | 保留该签名，另加 `setMeasurementState(MeasurementState)` / `setCamera(CameraRole)` / `setMessage(QString)` | ENG-09 §4.1 冻结的枚举才是 data 层唯一的状态载体；枚举→文字的转换集中到 `ui/utils/UiText.h`，与 `application` 层日志里的枚举名**逐字一致**（现场排障时界面文字要能在 log.txt 里检索到）。`setStatus` 与 `setMeasurementState` 写的是**同一行**，后设置者覆盖——这一点写进了头文件，否则会出现"界面状态与日志不符"这类难查的问题 |
| 66 | 9.md §四 的 `fromMat` 只判空矩阵与通道数 | 增加三条拒绝：非 `CV_8U`、`step < cols × elemSize`、4 通道 | `Format_Grayscale8` / `Format_BGR888` 都假定每通道 8 位。传入 CV_16U（相机 SDK 的 HDR 模式常默认 16 位）时 QImage **不报错**，只会按字节重新解释同一块内存——16 位灰度图显示成宽度不变、高度减半的雪花图，而 `cv::Mat` 本身完好。4 通道不映射 `Format_ARGB32`：第 4 通道存的是 A 还是深度/置信度取决于后端，猜错会让整幅图带一个错误的透明通道 |
| 67 | 9.md §一 的目录清单里 `utils/` 下只有 `QtImageConverter` | 新增 `src/ui/utils/UiText.h`（header-only） | 三个控件同时需要枚举名（StatusPanel 要状态与相机、TurntablePanel 要运动状态），集中一份才不会分叉。`ui/utils/` 目录本就在 9.md §一 中，header-only 不新增编译单元（同 `data/MonotonicClock.h` 的做法） |
| 68 | 9.md §十一/§十二：`add_executable(AircraftPoseSystem …)`、`add_library(AircraftUI STATIC …)`、`target_link_libraries(AircraftUI PUBLIC AircraftData Qt5::Widgets ${OpenCV_LIBS})` | 沿用 001 阶段确立的 `aps_add_module_library(ui RECURSE …)`（目标名 `ui`，产物 `libui.a`）与既有的 `src/app/CMakeLists.txt`；依赖按 ENG-03 §12.8 冻结为 `application + preview + aps::qt5` | `Aircraft*` 命名全工程未采用（同第 32 行）；依赖集取 ENG-03 §12.8——它正是"哪个模块可以链接什么"的条款。**OpenCV 由 `data` 的 PUBLIC 链接传递**（`data/ImageFrame.h` 公开暴露 `cv::Mat`），ui 不另行列出（9.md 与 §12.8 在这一项上冲突）。**RECURSE 是必需的**：ui 的源文件分布在三层（`MainWindow.cpp` 在本目录、控件在 `widgets/`、工具在 `utils/`），不加时 `libui.a` 里只有 `MainWindow.cpp`，而模块"非空"故不会触发空模块跳过提示——首次 cmake 实测输出为"模块 ui：1 个源文件"，直到链接可执行文件时才以 undefined reference 暴露 |
| 69 | 9.md §十五 要求提交 `feat[ui]: initialize Qt application framework` | **未提交**（本目录不是 git 仓库） | 同第 45 行：001~008 各阶段均未 `git init` / 提交，用户未要求过提交 |
| 70 | 9.md §十 的 `main.cpp` 只有 `QApplication` + `MainWindow`，未提应用标识与高 DPI | 增加构造前的 `AA_EnableHighDpiScaling` / `AA_UseHighDpiPixmaps`，以及 `organizationName` / `applicationName` / `applicationVersion` | 必须在 `QApplication` 构造**之前**设置（Qt5 文档要求，之后设置静默无效）。现场一体机常用 4K 屏，不开启时整个界面以 1 倍逻辑像素渲染、文字小到无法判读，而开发机通常是 1080p——这是交付到靶场才暴露的问题。应用标识是 009 引入 ConfigManager 后定位配置目录的基准 |
| 71 | ENG-04 §14 与 9.md §二 都把"用户操作 / 用户交互"列为 UI 职责，但 9.md 的全部界面元素都是只读显示 | **不新增按钮等操作控件**，只实现显示；相机切换 / AUTO-MANUAL 切换留待 009 与状态机一起设计 | 预览层的 `setCamera` / `setMode` 已经存在，但"AUTO 模式下用户可否覆盖显示源"是 SYS-08 §8 的**语义**问题（该节明写"用户控制预览不影响测量状态机"），不是界面布局问题。在状态机接入前放一个按钮，按下去的效果无法确定。**需要一条裁决**：008 之外是否新增操作面板，以及哪些操作允许在 COMPLETE / FAILED 之外的状态下使用 |
| 72 | ENG-01 §15 的冻结测试目录清单里**没有 `tests/ui/`** | 不新增测试目录；改用一次性离线探针（39 项断言，源码不入库）验证 | ENG-01 §15 是测试目录的权威，新增目录属清单变更。做法同 001~006 各层的"消费方探针"：结论记入 §1 的验证方式表 |
| 73 | 9.md §九：`QHBoxLayout` 把三个控件平铺 | 改为"图像占满剩余空间 + 右侧面板列" | 三个等宽横条在 1280 宽下每块约 427 px，而预览窗口的用途就是看目标，图像越小越无用（`ImageViewer` 的 `sizeHint` 为 4:3，与三台相机的出图比例一致） |
| 74 | 9.md 未规定界面上的相机标识来自何处 | 取 `PreviewManager::cameraId()`（来自 `OpticalRig` 的配置 id，如 `cam25`），空则退回枚举名 | 该函数的存在理由就是"供 UI 取相机标识与焦距"（`PreviewManager.h`）。显示配置里的 id 而不是枚举名，才能看出"相机配置是否就是预期的那一台" |
| 75 | 9.md §八/§九 没有任何取帧逻辑（§十四 却展示了图像与状态） | MainWindow 增加 16 ms `QTimer` + `PreviewManager::getFrame()`（**单消费者、仅 GUI 线程**），并由 `showEvent` / `hideEvent` 启停 | `PreviewManager.h` 明写该接口"为配合 Qt 的定时重绘……界面以 60 Hz 定时调用，只有返回 true 才重绘"；`getFrame` 记录"上次已交付的序号"，从两个线程调用会让先到者把帧吃掉。窗口最小化后 Qt 仍派发定时器事件，而此时的降采样绘制是纯浪费，故按可见性启停 |

### 009 系统集成与依赖注入（10.md）

| # | 工作流文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| 76 | 10.md §八/§九：`ApplicationContext` 为**直值成员 + 默认构造**（示例写作 `ApplicationContext ctx;`） | 全部长生命周期对象改为 `std::unique_ptr`，构造仍无参、不做任何 I/O | 设备的构造函数**需要配置**（`VirtualCameraBackend(CameraConfig)`、`PosePipeline(measurement, validation, …)` 等），而配置只有 `ConfigManager::load()` **之后**才存在。直值成员只能"先默认构造、再赋值"，那要求每个类都有默认构造与赋值，且会短暂存在一批**半初始化**对象 —— 正是 ENG-10 §5.2"注入后不可变"要避免的状态。`unique_ptr` 用"存在即已完全构造"消掉该中间态。**这不是风格差异，10.md 的写法对这些真实构造函数无法编译** |
| 77 | 10.md §八：创建顺序为 `Logger → ConfigManager → …` | 按 **ENG-02 §15 原文**把 `ConfigManager` 排在 `Logger` **之前** | `Logger` 的 `log_dir` / `log_level` 只能读到配置才知道，它是 §15 清单的第一项也正是这个原因。10.md 的顺序在物理上无法成立（Logger 先建则无从知道往哪写） |
| 78 | ENG-02 §15 的对象清单只有九项（… → `PreviewManager` → `MeasurementController` → `MainWindow`） | 增列 `PreviewWorker`（并说明 `CalibrationManager` / `TargetModelManager` / `FileMatchStatsStore` / `PosePipeline` / `Recorder` 的必要插入点） | §15 是**对象**清单，而这几项是装配时才出现的**协作者**，其位置由依赖方向唯一确定（rig 的标定来自 CalibrationManager；pipeline 以裸指针持 models/stats；controller 以引用持 pipeline）。**`PreviewWorker` 必须显式增列**：它是独立线程对象，`PreviewManager` 只负责入队与发布，漏掉它整条预览链是死的却**不会有任何报错**（界面只是永远显示"无图像"，而"没有帧"与"没有消费者"看起来完全一样） |
| 79 | ENG-03 §12.8 冻结 libui 依赖为 `Qt + application + preview`（**无 device**） | `MainWindow` **不接** `ITurntableController*` / `MeasurementController*`；新增 `ui/MeasurementView.h`（纯数据 POD）+ `startRequested` / `stopRequested` 两个 Qt 信号 | 9.md/10.md 都写"把控制器输出接到面板"，而 `ITurntableController` 属 device —— 直接注入即破坏 §12.8 与 ENG-01 §18。POD 让 ui 只**显示**、不解释测量语义（ENG-01 §13），"按开始/停止"由 ui 发信号、由 app 决定其含义 |
| 80 | 未规定 `Logger` 的级别枚举、以及日志如何到达各模块 | `LogLevel { DEBUG, INFO, WARN, ERROR, OFF }` + `parseLogLevel()`；`Logger` 为普通对象（非单例，遵 ENG-10 §5.1 的精神），由 app 持有并显式传递 | 冻结文档未给枚举定义（属实现选择）。**但"分发"是真实缺口**：`algorithm` 不依赖 `infrastructure`（ENG-03 §12.5 冻结为 `data + OpenCV`），故**算法链内部一行日志也写不出来** —— 实测运行期的检测框、匹配数、内点数、RANSAC 迭代全部不可见。已登记为 [待裁决问题汇总.md](待裁决问题汇总.md) 的 **Q-B7** |
| 81 | 未规定标定装载策略与相机焦距的配置键 | 新增 `optical_rig.calibration_mode`（`file` / `synthetic`）与通道级 `focal_length`，并在 `ConfigManager.h` 注明"009 引入的装载策略" | 两者都不在 `OpticalRigConfig` / `CameraChannel` 的冻结字段表中。`calibration_mode` 是 M1/M2 打通链路所必需（无真实标定文件），且合成模式会**打 WARN 说明结果无物理意义**。已登记为 **Q-B9** |
| 82 | SYS-04 §265 要求 `MeasurementController → MeasurementTask → RecorderWorker` 落盘 | 实现 `infrastructure::Recorder` + `app::RecorderSinkAdapter` 桥接 | `Recorder` 的归属被 ENG-03 §12.7 冻结在 infrastructure，而 `IRecorderSink` 定义在 application/`MeasurementController.h`，两者方向相反（ENG-01 §17/§18）。**实测确认 `MeasurementTask` 的信息量不足以产出 SYS-04 §6.4 要求的结果包**（无原图、无转台角度、无 `degraded`），已登记为 **Q-B2**（高） |
| 83 | 10.md §十 的场景未提及 `FileMatchStatsStore` 的写入 | 实现 `FileMatchStatsStore` 并注入 pipeline，但**没有任何调用点写 `record()`** | C-21 把接口下沉到 data、007 预留了 `IMatchStatsStore*` 注入，009 的职责是"让它真正生效" —— 实测发现只完成了"读"的一半：`record()` **零调用者**，`successRate()` 永远返回冷启动先验 0.5，**ENG-10 §4.1 的整套历史统计机制实际未生效**，而"冷启动"是**合法状态**故不会有任何告警暴露它。已登记为 **Q-B3**（高） |
| 84 | 10.md §八 的 `main.cpp` 未规定配置目录、界面刷新节拍与退出顺序 | `argv[1]` 指定配置目录（默认 `config`）；16 ms `QTimer` 同时驱动 `tick()` 与界面刷新；`ctx`/`init`/`window` 全部用**栈对象** | 16 ms 的取值由 SYS-08 §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 3 反推（须远小于最短状态超时 100 ms，取 6 倍余量）。栈对象是为了让析构顺序由语言保证：`ctx` 若为堆对象则先于 `window` 销毁，而 window 的定时器仍在访问已销毁的 `preview` —— 表现为**关闭程序时偶发段错误** |
| 85 | 10.md 各节的提交要求 | **未提交**（本目录不是 git 仓库） | 同第 45/69 行：001~009 各阶段均未 `git init` / 提交，用户未要求过提交 |

### 010 第一次完整编译运行闭环（11.md）

| # | 工作流文档原文 | 实际执行 | 依据 |
|---|---|---|---|
| 86 | 11.md §十一 的 Mock 清单含 `MockPoseEstimator` | 存在（`algorithm/pose/PnPPoseEstimator.h` 的 `MockPnPPoseEstimator`）但**不可达**：其自注用途是"状态机与验证逻辑的**单测**"，且 `PosePipeline::solvePose` 在调用 `pnp_` 之前要求"CAD 定位或特征匹配已产出**非空对应集**"，两者都依赖机型库 | 与 §6 第 37/47 行同源：**模型相关的判据被放在 pipeline 而不是它该在的层**。只注入 mock 姿态估计器不够 —— 链在到达它之前就返回了。已登记为 **Q-B13**。**同类的另外两处本轮已修**（见下 D-1） |
| 87 | 11.md §一/§十 的 M1 目标未提及机型库 | 不造合成机型库，M1 闭环止于 `POSE_SOLVE` | 11.md 的清单只要求"状态变化：IDLE ↓ SEARCH ↓ TARGET_FOUND ↓ ALIGN ↓…"，**未要求走完全程**。机型库是真实交付物（`points3d.yaml` + 3×`feature*.bin`），软件阶段造假的代价是让"结果看起来已标定"。**已知可行**：`saveFeatureLibrary()` 是公开静态函数、`.bin` 格式由本工程自定义，故合成库可用**工程自己的写入器**产出而无需凭空构造字节。已登记为 **Q-B16** |
| 88 | 11.md §8.2 / §十 要求 010 新增 `src/runtime/{ComputeBackend.h, CpuRuntime.cpp, GpuRuntime.cpp}` | **未新增** | `libruntime` 已建但**不链入 app**（ENG-03 §3/§12.9 两处遗漏，ENG-01 §17 依赖图中亦无该模块，见 §6 第 3 行）。在归属未定前新增三个文件只会扩大无消费者的交付物。已登记为 **Q-B17** |
| 89 | 11.md §十二 要求提交 `feat(integration): first runnable V2.1 framework` | **未提交**（本目录不是 git 仓库） | 同第 45/69/85 行 |
| 90 | 11.md §十 的六项验证清单 | **逐条实测通过**（构建 / 窗口 / frameId 递增 / Preview 刷新 / 状态机 / 日志），证据见 §1 与 [待裁决问题汇总.md](待裁决问题汇总.md) §6 | 窗口一项用 `xdotool` 确认**可见窗口**存在（1600×984），Preview 一项用 `ffmpeg -f x11grab` 抓屏确认图像区**确实渲染出合成图与 OSD** —— 均未停留在"进程还在跑"这一级 |

两处**冻结文档互相矛盾**，均在代码中留有注释，并按"取更具体者"处理：

1. ENG-01 §17 的方框图把 `optical` 画在 `device` 之上，与 ENG-03 §12.3 及
   裁决 **C-18**（"optical 位于 device 之下，device 依赖 optical"）矛盾，
   **取 ENG-03 §12**（见 `src/CMakeLists.txt`）。
2. ENG-03 §12.4 的依赖冻结与 SYS-09 §15 / ENG-05 §15 的线程实现要求矛盾，
   **取 ENG-03 §12.4**（见上表第 11 行）。
3. SYS-08 §7.3（每状态次数上限）、§7.4（回退双预算，"取先到者"）与 §7.6
   约束 2（"**所有状态进入时**必须调用 `beginAttempt()`"）+ 约束 5（"计数器
   **不因进入新状态而清零**"）四条同时生效时，§7.4 的边预算恒被更早触发，
   于是 **§10 用例 3 / 用例 4 期望的 9002 实际不可达**。本仓库严格实现了
   §7.6〔引用无效·依据待裁决·见 Q-D2〕 约束 2（它是四条里最具体的一条），故以状态次数上限收场（用例 3
   报 MEASURE_SELECT 用尽、用例 4 报错误方向的 9001）。逐 tick 追踪见
   `tests/integration/MeasurementFlowTest.cpp` 用例 3 / 用例 4 的注释。
   可到达 9002 的唯一路径是 `CAPTURE → MEASURE_SELECT` 边（CAPTURE 是唯一
   上限 > 2 的回退起点），已由 `tests/unit/StateMachineTest.cpp` 覆盖。
   **需要一条裁决**：9002 应在何时可观测，或调整 §10 用例 3 的期望码。
4. §7.4 的"三台相机、每次排除一台、第 3 次无候选"这一推导**假定 VALIDATE
   是唯一的排除者**，但 `MeasurementStrategy::switchToNextCamera()` 在
   MEASURE_SELECT 无法给出候选时也会排除一台。实测（用例 3）表现为
   "三次排除只用了两次 VALIDATE 失败"。两者不是同一种故障：前者是"这台
   相机测不准"，后者是"这台相机连候选都给不出"。**需要一条裁决**：无候选
   而排除的相机是否计入 §7.4 的回退预算。
5. SYS-08 §5.3 未列 `TARGET_FOUND` 的失败行，§7.7 的恢复表也没有该状态，
   而 §7.3 又把它的次数上限定为 1。三条叠加的结果是：该状态一旦失败，
   **既无经批准的恢复动作，也无重新进入的可能**。本仓库取"就地 FAILED"
   （上表第 22 行）。**需要一条裁决**：§7.3〔引用无效·依据待裁决·见 Q-D2〕 的"上限 1"是否只约束状态内
   重试，而不约束回退后再进入。
6. ENG-09 §5.26 / SYS-05 §12 的 `MeasurementTask` 没有 `degraded` 与
   `cameras_available` 字段，而 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求 result.json 记录这两项。
   本仓库不改 data 层结构（需走 ENG-09 §8 流程），改由 Recorder 从控制器
   读取（上表第 19 行）。**建议**在下次 ENG-09 修订时为 `MeasurementTask`
   补两个字段，或在 SYS-05 §12 明确它们属于 result.json 的**外层**字段。
7. **SYS-14 §6.2 的减号与 §8.2 / ENG-10 §3.6 的 `eNorm` 不能同用**（007 阶段）。
   §6.2 的评分式写作 `score = w1·qNorm + w2·fNorm + w3·mNorm − w4·E`（`E`
   为**原始**角分误差），而 §8.2 与 ENG-10 §3.6（两处措辞一致）要求四项
   **先归一化到 [0,1] 再加权**，其中 `eNorm = 1 − min(1, E/E_ref)` —— 归一化
   之后 `eNorm` 已经是**优度**（越大越好）。本工程按冲突规则取更具体的
   §8.2/§3.6，实现为 `+ w4·eNorm`。
   **必须说明的是**：`+ w4·eNorm` 与 §6.2 的 `− w4·E`（原始误差）**排序方向
   相同**（`1 − E/E_ref` 与 `−E` 都是误差的单调减函数），差别只在归一化与
   那项常数 `w4`。真正的陷阱是**把 §6.2 的减号照搬到一个已归一化的优度上**
   （`− w4·eNorm`）：那样误差达上限的候选贡献恰好 0（表面正确），而误差为 0
   的最好候选贡献 −w4，排序完全颠倒。本实现在代码中写明了这一点，正是为了
   防止后来者"照字面补一个负号"。**建议在下次 SYS-14 修订时把 §6.2 的结果式
   与 §8.2 的归一化形式统一**。
8. **`ValidationConfig` 里没有角分级判据字段**（007 阶段）。`PoseValidator`
   的判定只有三条：重投影误差（**pixel**）≤ `maxReprojectionError`、内点
   比例、置信度；`yawMin/yawMax` 是 Yaw 的**物理可达范围**（拦截发散的
   PnP），不是精度判据。而 ENG-09 §6.6 冻结的 `ValidationConfig` 里
   **没有**任何以**角分**为单位的字段，重投影误差（pixel）与 Yaw 误差
   （角分）之间又没有固定换算（前者随距离与特征分布变化）。也就是说：
   "这次测量是否满足 1 角分验收指标"这个问题，在冻结的类型里**无处可问**。
   指标值 1 角分在本仓库中只作为 `MeasurementSelector` 的 `E_ref`
   （ENG-10 §3.6 的 eNorm 参考值）出现，那是**评分**用的相对参考，不是
   判定阈值。本仓库因此在 `MeasurementSelector.h` / `PoseValidator.h` 的
   文件头写明该缺口，不自行新增字段（改 data 层结构需走 ENG-09 §8）。
   **需要一条裁决**：在 `ValidationConfig` 补 `yawMaxErrorArcmin`
   （含默认值），还是明确"验收指标不可配置、恒为 1 角分且只用于评分"。
9. **`IMatchStatsStore` 没有版本查询**（007 阶段）。ENG-10 §4.1 要求
   `match_stats` 按 `model_id` **与模型版本**分组（换机型/换涂装必须换统计
   表），但冻结的 `IMatchStatsStore`（data 层，ENG-09 §5.x）只有按
   `CameraRole` 取历史值的接口，查不到版本。本仓库在 007 阶段以
   `MeasurementSelector` 只按 role 取值为准，并在头文件写明该缺口 ——
   版本一旦不匹配，历史 `mHist` 会**静默**串用（数值看起来正常，只是
   偏乐观/偏悲观）。**建议**在下次 ENG-09 修订时为该接口补一个
   `modelVersion` 入参或在实现侧以文件名分组。
10. **冻结文档与它自己的表格矛盾**（R07，2026-09-24 审查报告；与前 9 条
   "两份文档之间"的矛盾不同，这一条在**同一页之内**）。ENG-10 §2.1 / §3.2
   的 `σ_θ` 估算式 `σ_θ ≈ σ_px·√12 / (W·√N)` 漏了弧度→角分的换算因子：
   `σ_px / W` 是**比值**（两者同为 pixel），该式量纲是**弧度**，而同一页的
   表格、注释与结论式都按**角分**使用它，相差 `180/π×60 ≈ 3437.7468` 倍。
   ENG-09 §5.16 / §9.2 复述了同一个错。后果是 E 分项退化为常数
   （`eRatio ≈ 0`、`eNorm ≈ 1`），**通道排序被改变**。
   **本次是"把公式对齐到表格"**——表格数值（W=1930→0.19、W=386→0.93 角分）
   自 V2.1 起一直是对的，错的是算式。已升版为 ENG-09 V2.2 / ENG-10 V2.2
   （旧版归档），代码侧补 `kRadToArcmin`，并重写原先**把错公式抄了一遍**的
   测试期望值（那是自比较，所以"实现错而用例照样绿"）。
   ⚠ **本次只重建了单位一致性，不证明整个误差模型已通过实机标定**。

另有两处**跨文档**问题已实测确认但尚未在冻结文档中修订：

- `data` 必须链接 OpenCV（上表第 4 行）。建议在 ENG-03 §12.1 补一行
  OpenCV，使其与 §12.2~§12.8 的写法一致；在修订落地前，`libdata` 的
  PUBLIC 链接就是事实基线。
- 冻结的 `IMultiCameraManager` 没有可用性查询方法，而 SYS-08 §7.5〔引用无效·依据待裁决·见 Q-D2〕 要求
  统计可用相机数。本仓库以"同步帧中该路图像是否为空"判定（上表第 20 行），
  未新增接口。若将来 §7.5 的降级判据需要更细的区分（例如"相机在线但
  曝光失败"与"相机离线"），需要在 SYS-04 补一个查询方法。

### 全仓静态审查 R02 / R04 / R05 / R06 / R07 处置（2026-09-24）

审查报告 `proj_plane_review_9775ea6.md`（基线 `main@9775ea6`，251 个文件）列出 11 项缺陷
R01–R11。⚠ **该报告的环境缺少 OpenCV 开发包、无法构建**，也未声称 242 项测试通过或做过
任何实机验收 —— 故它给出的是**静态调用链证据**。因此本轮每一项都补了**走真实调用路径**
的实证（下表"落点"列即用例名），**不接受"模块单测已有"作为关闭依据**
—— R04 正是"模块单测早已存在、调用链照样是断的"这一形态。

本轮关闭 R02 / R04 / R05 / R06 / R07 五项（**R01 已单独提交 `03f5abc`**：
`.gitignore` 的 `runtime/` 改为 `/runtime/`，原来那个模式匹配任意深度的同名目录、
把 `src/runtime/` 一起忽略，使推上去的仓库**根本无法配置**）。
R03 / R08 / R09 / R10 / R11 与 A1 契约、验收清单补正留待后续阶段。

| # | 报告原文 / 声称 | 实际执行 | 依据 / 落点 |
|---|---|---|---|
| 91 | **R02**：全工程 `install(TARGETS ...)` 实体 **0 处**（7 处 `install(` 全是 `install(DIRECTORY`），故 `deploy/` 只有空的 config/models/calibration，**没有 bin/、没有 lib/**；而 `install_check` 只跑 `cmake --install`、不校验任何产物，所以该缺陷不被任何检查覆盖 | `install(TARGETS)` 写进 `src/CMakeLists.txt` 的 `aps_add_module_library()`（**10 个模块库**）与 `src/app/CMakeLists.txt`（可执行文件）；并在**同文件、紧随其后**加 `install(CODE ...)` 断言（bin/ 有可执行文件且可执行、lib/ 下每个模块库都在）；新增 `install_launch_check`。约定写进 §4.6。**裁决条文：C-014** | 见 §4.6 的三方分工与两个陷阱。**分工修正**：`cmake/InstallRules.cmake` 原文写"各模块 → 各自的 `install(TARGETS ...)`"，描述的是一个**从未实现的约定** —— 约定写了不等于做了，故该段现在只描述代码里**实际存在**的分工。落点：`ctest -R 'install_check\|install_launch_check'`，自检打印 `…/lib 下 10 个模块库齐备` 与 `安装自检通过：…/bin/AircraftPoseSystem`；变异（注释掉 `src/app` 的 `install(TARGETS ...)`）后 `install_check` 必转红。**⚠ R04 收尾加强（2026-09-24 复审）**：`install_check` 先 `cmake -E rm -rf` 清空自检前缀再安装（"本次运行全新安装"），`install_launch_check` 判**四条**（124 + `启动完成` + 首拍标记 `APP_FIRST_TICK_COMPLETED` + 无 `启动失败`）并把日志留在 `${前缀}/launch_check.log`，失败即 `cat`。依据是一次**实测复现的假通过**：停用整段 R02 安装规则、保留陈旧前缀 ⇒ 两条自检仍全绿（0.01 s / 3.00 s），而全新安装根本没有主程序；加强后同一变异立即转红（退出码 127）。两条约定见 §4.6 第 3、4 条 |
| 92 | **R04**：`MeasurementController` 持有 `preview_` 却**从未调用 `submitFrom()`**；生产代码里唯一的调用点在 `SystemInitializer::pumpIdlePreview()`，而它只在 IDLE/COMPLETE/FAILED 执行 ⇒ **测量期间画面空白**。（`SystemInitializer` 的注释声称"controller 每拍自己采集并提交预览"—— 该假设自写下来就从未实现） | 在 `acquire()` 内 `capture()` 成功之后、`updateDegradation()` **之前**投递预览，抽 `submitPreview()` 让"唯一投递点"成为一个可 grep 的符号 | **为什么在 `updateDegradation()` 之前**：降级越界会就地 FAILED，补帧若排在后面，**导致任务终止的那一帧永远到不了屏幕**，而那一刻画面正是唯一的现场证据。**⚠ 双重采集的判据已更正（R04 收尾，2026-09-24 复审），原论证作废**：本行原写"两个状态集**恰好互补**（12 = 3 + 9）⇒ 任一 tick 至多一次 `capture()`"，两处错 —— ① **前提不成立**：`stepCapture` 本来就一拍内连采 `captureFrameCount`（默认 5）帧，"至多一次 `capture()`"从来不是系统的不变量；② **推理越界**：控制器对空闲态早返回、app 只在空闲态补帧，这两条只能推出"空闲补帧不与活动态采集同拍"，**推不出**"活动态采集不可能当拍转入终态"—— 后者才是重复采集真正的入口。**真实可达路径**：ALIGN 的对准命令**越程**（`AlignmentController::calculate()` 在 `azimuth` 超出 `azimuthMax` 时置 `commandValid_ = false`、码 2002）⇒ `handleFailure(CAPABILITY)` ⇒ `makeFail` 不重试 ⇒ **同拍 FAILED**，而这一拍**已经采集成功**。故现行规则是两条：空闲与否按**推进后**状态判定（AUTO 映射必须跟上），但**本拍推进了状态就不补帧**（`!stateAdvanced`）—— 即"转入终态当拍不补帧，下一拍恢复"。**采集归属按状态与分支列账**（不再用状态集互补去推；实测值）：IDLE/COMPLETE/FAILED **1**（本拍未推进时）、SEARCH **1**、TARGET_FOUND **0**（复用 `lastFrame_`）、ALIGN **1**（等待到位/超时分支 0）、STABILIZE **1**（MOVING/超时分支 0）、MEASURE_SELECT **1**、CAPTURE **`captureFrameCount`**（默认 5）、POSE_SOLVE/VALIDATE/SAVE **0**。落点：原有 `R04_测量期间预览必须有生产者`（断言帧号**严格递增**、且提交的角色等于**当时**的显示源）、`R04_未注入预览时不崩`；**新增** `tests/integration/SystemInitializerTest.cpp`（**真实装配、零桩、真实单调时钟**，两个用例）—— 场景 A 逐状态核对上表的增量，场景 B 用 `ctx.turntable->setInitialAngles(azimuthMax, 0)` 与 CAM50 `setTargetPixelOffset(居中阈值 + 50 px)` 摆出越程，断言**终止当拍**三路增量 == 1 且 `idleFrames` 不变、**下一拍**三路 +1 且 `idleFrames` +1；撤掉 `!stateAdvanced` 后该拍增量变 2 ⇒ 用例转红（实测：5 条断言同时红，场景 A 的终态拍另红 1 条） |
| 93 | **R05**：`stepValidate` 把 `validate()` 的 `false` 读成"验证过程失败" → `handleFailure` → **提前 return**，使 `validationResult_ = validation` 与 `strategy_.excludeCamera(...)` **两步都执行不到**。真实 `PoseValidator` 判不合格就返回 false ⇒ 生产路径必然踩中；而测试桩**恒 `return true`**，所以这条断点从未在任何测试里出现过 | 契约统一为 **`bool ≡ out.valid ≡ "是否通过全部判据"`**（不是"过程是否执行成功"）；加**对账分支**（返回值与 `out.valid` 不一致 = 违背契约 ⇒ **可见地 FAILED**，不静默按任一方继续）；`validationResult_ = validation` 提到整段**最前**；`IPosePipeline::validate` 补 `@return` 注释（该接口原本**没有任何** `@return`，是歧义的源头）；测试桩改为 `return out.valid` —— **必须与控制器同批落地**，否则旧桩会命中新对账分支。**裁决条文：C-013** | **对称性**：`solvePose` 的 `false` = 没算出结果，`validate` 的 `false` = 算出来了但不合格，两者语义不同，故各自写明。⚠ **实施中更正了设计稿的一处**：设计稿说"把 `validationResult_ = validation;` 提到**分支之前**"，照字面放在**对账分支之后**会在新的失败通路上**原样复现 R05 要修的缺陷**，故位置钉死为**整段最前**（详见 C-01 §C-013）。落点：`R05_验证契约被违背时必须可见失败且详情仍留存`（用**故意自相矛盾**的桩逼出对账分支，并内建因果对照 —— 撤掉那个谎之后必须走完 COMPLETE）、`用例3_回退预算_VALIDATE恒不通过时的有界性` 判据四（`valid == false` + `0.42 / 0.91 / 0.88` 三个**来自桩**的实测值，即"细节不再丢失"）。**⚠ R05 收尾（2026-09-24 复审）**：对账分支原实现走 `handleFailure(TRANSIENT, ...)`，那是**实现偏离了裁决** —— C-013 第 2 条原文是"控制器必须**可见地** FAILED"，而 VALIDATE 的 TRANSIENT 处置是**回退**，违约与失败之间隔着若干次重试与回退，期间状态机在用一份自相矛盾的实现继续跑（原用例用**持续违约**的桩，靠回退预算耗尽才失败，证明的是"迟早失败"而非"当次 FAILED"）。现改为新增私有 `failTerminal()`（= `recordFailureCause()` + `failWith()`），并在注释里写明为什么不套 §7.2〔引用无效·依据待裁决·见 Q-D2〕 的三分类：那三类描述的是**设备与运行条件**，而契约违背是**实现缺陷**，重试只是再调一次同一个坏实现、回退则是用自相矛盾的实现跑完测量。用例相应改为**逐拍驱动**并断言违约那**一刻**即为 FAILED、`rollbackCount() == 0`；桩由"持续违约"改为**只违约一次**（回退语义下它会恢复并跑完 COMPLETE，故能区分"当次终止"与"迟早失败"）。变异（换回 `handleFailure(TRANSIENT)`）→ 3 条断言转红 |
| 94 | **R06**：`MeasurementController.h` 的成员 `selection_` **从未被赋真实值**，全文件只有 reset 置空与 `record.selectedScore = selection_.score` 两处引用；两处 `selectCamera()` 调用都写进**局部变量** ⇒ **"选了哪台"对、"得分"恒 0**，又一个"字段合法但语义不成立" | 抽 `adoptSelection()` 私有助手，把 `selectedCamera_` / `selection_` / `setAutoCamera()` 三件事**收口到一处**，两个调用点各换一行 | **为什么必须成对写回**：分开赋时将来任一处被单独改动，就会得到"新角色 + 旧得分"—— 两个值都合法、都不报错，正是 D-C02-5 换个位置复现。修复后可给出穷尽式论证：`selectedCamera_` 全仓只有 3 个赋值点。落点：`R06_记录里的选中得分必须来自选择结论`、`R06_换机路径的得分也必须写回`（后者覆盖失败包一侧；得分由测试**自己的桩**给出 ⇒ 输入侧预言机，读回实现成员无法伪装）。**⚠ R06 收尾（2026-09-24 复审）**：桩原先给**所有**通道同一个 `1.0`，故"新角色配旧得分"照样能过 —— 预言机分辨不出通道。现改为按通道取分（CAM25 = 0.31 / CAM50 = 0.72 / CAM100 = 0.90），并新增 `solveFailTimes`（前 N 次 `solvePose` 失败）以构造"首次 PnP 失败 → 换机后成功"这条路径，断言 `selectedCamera == CAM50` 且 `selectedScore == 0.72`（旧得分会是 0.31）；两条既有用例的期望值也改为 `scoreOf(record.selectedCamera)`（与角色交叉校验）。变异（换机处只写角色、不写 `selection_`，即 D-C02-5 的原形）→ 得分断言转红 |
| 95 | **R07**：`σ_px / W` 是**比值**（两者同为 pixel）⇒ `predictedError` 的量纲是**弧度**，而注释与冻结文档都写"单位角分"，差 `180/π×60 ≈ 3437.7468` 倍。后果：`eRatio ≈ 0`、`eNorm ≈ 1`，E 分项**退化为常数**，通道排序被改变 | 补回换算因子（`kRadToArcmin`，含推导注释），使算式与**它自己那一页的表格**同量纲；冻结文档同步升版为 **ENG-09 V2.2 / ENG-10 V2.2**（旧版 `cp -p` 归档进 `archive/`，见下） | **是"把公式对齐到表格"，不是改表格** —— `ENG-10 §2.1` 的数值（W=1930→0.19、W=386→0.93 角分）自 V2.1 起一直是对的，错的是算式，**同页自相矛盾正是该缺陷的形态**。落点：`MeasurementSelector.EFollowsEng10Formula`（期望值取自 §2.1 表格，**不是**抄实现）、`PredictedErrorIsArcminNotRadians`（量纲回归，删掉换算因子即以 ≈3437.75 倍当场转红）。⚠ **本次修复只重建了单位一致性，不证明整个误差模型已通过实机标定** —— 见下方 Q-C5 |

**回归面（逐条确认过，不是推断）**：R04/R05/R06 三项都在 `MeasurementController.cpp`
里，改动集中在 `acquire()` / `stepValidate` / 选择写回三处，**均不改变任何
`capture()` 次数**（R04 不新增采集调用、R05 只改返回值与赋值顺序、R06 不改流程），
故 C-02 那批按 `armCapturePhase()` 计数的最佳帧数据链用例计数不变。

**本批的文档升版**（R07）：`ENG-09_类型与命名冻结表_V2.2_E单位换算修正版.md`、
`ENG-10_算法链接口与配置注入设计_V2.2_角分换算修正版.md`；V2.1 两版已 `cp -p`
移入同级 `archive/`，**绝不删除**（本目录不是 git 仓库，归档是唯一的旧版留存）。
两份 V2.2 各带"修订记录"节（本版变动说明 / 版本关系 / 未关闭的登记项）。
⚠ **`SYS-15 §4.5` 的悬空引用在 V2.2 里原样保留**，属已登记的悬空引用，**本版不修**。

**本批只登记、不处置的 7 项**（代码侧已按既有裁决实施完毕，登记的是实施中暴露、
但本轮不处置的事项）—— 详表见 [待裁决问题汇总.md](待裁决问题汇总.md) §5.2：

| # | 一句话 |
|---|---|
| Q-C1 | `SYS-15 §4.5` 悬空引用 —— 比"该节不存在"更强：**该误差模型在 SYS-15 里整份不存在**（`√12`/`σ_θ`/`σ_px` 零命中），而引用它的有 11 处以上含生产代码。**附带**：SYS-15 文件名写 V1.0、H1 写 V1.2 |
| Q-C2 | `validate()` **没有**表达"验证过程执行失败"的通道（契约部分 ✅ **已冻结 → C-013**；补通道要动 `IPosePipeline` 签名，须与 IF-SW-02 一并处置） |
| Q-C3 | `selectedScore == 0.0` 与"未选择"不可区分（`MeasurementSelectionResult` 按 ENG-09 §5.17 冻结为**两字段**，加 `valid` 标志须另行裁决） |
| Q-C4 | CAPTURE 期间 `droppedCount()` **合法**上升 —— 将来加积压断言**不得**写成 `EXPECT_EQ(droppedCount(), 0)`。⚠ 原附注"当前 `droppedCount` 在 `tests/` 零出现"**已更正**（见 Q-C8）：`PreviewLayerTest.cpp` 里本就有队列层的丢旧保新断言，缺的是"CAPTURE 期间"这一层 |
| Q-C5 | R07 修复后 E 分项在 `W ≤ ≈595 px` 处**饱和为 0**（默认 σ=0.5、N=100 时 `E = 595.4/W` 角分）。饱和是 §3.6 的**本意**，但对最小目标/退化展布降低了区分度。⚠ 标定落地后（σ≈0.3）饱和点移到 `W ≤ 357 px`，区分区间**反而变宽** |
| Q-C6 | ENG-10 §10 跨文档义务表的**状态列不可信**（SYS-05 那行错了两版都没被发现，正因为"已改"从不被回查）。已在 V2.2 就地加警告 |
| Q-C7 | `deploy/` 交付树至今**从未被真实填充** —— 机制部分 ✅ **已冻结 → C-014**，"能装出正确的树"≠"交付树已就位"。⚠ 现在跑 `cmake --install` 只会得到一个有 bin/lib 而配置与模型皆空的包，**比空目录更容易让人误以为已可交付**，故本轮**刻意不跑** |
| Q-C8 | 断言基线的现状更正：`idleFrames` 现已有基线断言（app 层用例的四条：空闲拍 +1 / 活动拍 0 / 转入终态当拍 0 / 下一拍 +1）；`droppedCount` 的"零出现"表述有误（队列层早有），缺的是"CAPTURE 期间"那一层 |
| Q-C9 | ❌ **已撤销**（2026-09-24）：原拟登记"app 层空闲补帧边界无法自动化验证、需新增注入槽"。前提不成立 —— `ApplicationContext` 由调用方构造，测试用真实设备的**既有**公开 API 即可摆出场景，本批已落地并做变异，**无缺口可登记** |

> ⚠ **Q-C5 的限定句不得省略**：R07 关掉的是"算式与文档差 3437.75 倍"这一条，
> **不是**"误差模型正确"这一条。`σ_px = 0.5` 仍是**回落值**而非实测值
> （见 §6 第 8 条），`N_est` 依赖的 `M_hist` 至今**冷启动**（Q-B3）。

### 由此产生的顺序约束

`data` 是所有模块的**硬前置**：其余模块的 OpenCV 头文件与运行期符号都经
`data` 的 PUBLIC 传递获得。若 `data` 目录无源文件而被跳过，消费者会因找不到
`opencv2/*.hpp` 而编译失败——构建脚本会先打印 `模块 data：暂无源文件，跳过生成`
与 `依赖 data 尚未生成，暂不链接` 两条提示，按此方向排查即可。

这与 1.md 把 Data 层排在 **Phase 2**（仅次于工程骨架）一致：先有 `data`，
其余模块才谈得上编译。

**第二条：不是所有模块都适用"不用改 CMake"这条约定**——见 §4.5。

---

## 7 版本基线

| 基线 | 含义 | 已具备 |
|---|---|---|
| `v2.1-framework` | 工程骨架（**当前**） | 构建系统、数据层、设备层、光机层、预览层、应用层、算法层、Qt 界面、**系统集成（009）**；**M1 已达成**：可执行文件可启动、显示虚拟预览，运行至 `POSE_SOLVE` 后进入失败回退路径（010）—— ⚠ **尚未完成应用级成功测量全流程** |
| `v2.1-device` | 硬件抽象 | 接入 ImvSdk / 转台 SDK 的真实后端 |
| `v2.1-algorithm` | 算法链 | 检测 → 特征 → 匹配 → PnP → 校验 |
| `v2.1-release` | 交付版本 | 完整闭环 + 1 角分指标验证 |

基线切换不改变已冻结的接口与单位，只决定哪些后端被编入。

由 `-DAPS_VERSION_STAGE=...` 控制，编译期注入为 `APS_VERSION_STAGE` 宏
（见 `cmake/BuildOptions.cmake` §4）。

---

## 8 下一步

> **009 与 010 均已完成**，下列两节由"下一步"转为**已办事项的记录**，保留原文以便对照
> 当时的计划与实际执行的差异（差异逐条登记在 §6 的第 76~90 行）。

### 8.1 009 系统集成与依赖注入（10.md）—— 已完成

008 交付的是一个**能启动但还没有数据源**的界面（§6 第 59 行）。009 要补的
正是这一处，且 9.md/10.md 的示例在这里是**不够用的**，需要按冻结文档展开：

| 要做的事 | 依据 | 与 008 的接口 |
|---|---|---|
| 建 `ApplicationContext` / `SystemInitializer` | 10.md §八/§九 | — |
| 按顺序创建 Logger → ConfigManager → OpticalRig → Device → Preview → Algorithm → Controller | ENG-02 §15 | — |
| 把 `PreviewManager*` 注入 MainWindow | ENG-02 §16（禁止 UI 创建对象） | `MainWindow::setPreviewManager()` |
| 驱动状态机 | ENG-03 §12.6（application **不链接 Qt**）+ 7.md 的偏离（第 15/16 行） | `MeasurementController::tick(nowNs)`，由 app 侧的 QTimer 驱动，时刻取自 `data::monotonicNowNs()` |
| 把控制器输出接到四个面板 | ENG-01 §13（UI 不解释测量语义） | `viewer()` / `posePanel()` / `statusPanel()` / `turntablePanel()` |
| 实现 `IRecorderSink` 与 `FileMatchStatsStore` | SYS-04 §265、ENG-10 §4.1 | 使 007 阶段预留的 `IMatchStatsStore*` 注入真正生效 |
| 配置注入 | ENG-10 §5.1 的注入矩阵 | 算法链各类的构造函数入参（007 已按矩阵实现） |
| 创建 QApplication 与 MainWindow 之间的全部装配 | ENG-01 §14 | **app 是全工程唯一允许 include SDK 的地方** |

### 8.2 010 第一次完整闭环（11.md）—— 已完成（M1 达成）

完整构建 + 运行，核对 M1 清单 —— **六项逐条实测通过**，证据见 §1 与
[待裁决问题汇总.md](待裁决问题汇总.md) §6。

`src/runtime/{ComputeBackend.h, CpuRuntime.cpp, GpuRuntime.cpp}` **未新增**，
理由见 §6 第 88 行（归属未定，见 [待裁决问题汇总.md](待裁决问题汇总.md) Q-B17）。

### 8.3 下一次进入编码前的必办事项

本轮（009/010）的实测发现已**全部汇总到一个文件**：

> **[待裁决问题汇总.md](待裁决问题汇总.md)** —— 17 条新条目（`Q-B*`）+ 11 条旧条目（`Q-A*`）
> + 5 条已自行修复的缺陷（`D-*`），每条写明"现状 / 依据 / 不改会怎样 / 建议选项"。

其中**建议在进 011（真实相机接入）之前定**的四条，已起草结论并获批准：

> **[V2.1-C01_架构裁决变更说明.md](V2.1-C01_架构裁决变更说明.md)**（v1.4，**已批准 2026-09-23**）
> —— 裁决 C-001~C-009 + 追加冻结项 H-001/H-002；v1.3/v1.4 记实施中核实出的多处原文与实现不符，
> 以及 C-008/C-009 编号的重新分配。**编号最终裁定**：C-008 / C-009 由本批的两个新条目保持，
> 原持有者「标定流程 / 真实 PnP 链」**改判为 C-010 / C-011**（将来排期时启用）。
> **[V2.1-C02_实施设计说明.md](V2.1-C02_实施设计说明.md)**（v1.2，**已批准 2026-09-23；
> Step 1–6 全部实施完成，无阻塞项**）—— C-002 的落地设计。

| 缺口 | 裁决 | 状态 |
|---|---|---|
| Q-A5 | **C-001** 在线自洽性 / 离线精度两分；真值两阶段且**不绑定转台** | ✅ 冻结 |
| Q-B2 | **C-002** `MeasurementRecord` + `MeasurementStatistics`（data 层）+ 可追溯键 | ✅ **全部落地**，含 `statistics` 填充（§6 第 27 行，2026-09-23 经 `IPipelineObserver` 打通） |
| Q-B14 | **C-006** 补 4001 / 5001 / 9004 / 9005（实施时追加 3002）；**9004 仅兜底** | ✅ **已落地**（8 处 + 20 处 `makeFail` 逐个归类；`errorCodeName()` 同步补齐） |
| Q-B15 | **C-007** `FailureTrace` + 失败包落盘（不含原图） | ✅ **已落地**（含 `failure.json`；实施中修掉"盘上轨迹比声明的 `state` 少一跳"） |
| Q-B16 | **C-003** `models/aircraft/synthetic/`，release 拒绝加载 | ✅ **已裁决待实施**：裁决内容为**开发用合成机型库** + release 拒绝加载，**不是"真实机型库"**；实测 `models/` 目前**只有 `.gitkeep`**（无 `aircraft/synthetic/`）⇒ **"已冻结"不等于"已实施"** |
| C-02 §12.5 第 5 条 | **C-008** 非有限姿态由 `PoseValidator` 拒绝 + `reason` 分类 | ✅ **已落地**（3 处变异检查按预期转红后还原） |
| C-02 §12.5 第 6 条 | **C-009** `RecorderSinkAdapter` 测试（三层：组装 / 结果包 / 快照） | ✅ **已落地**（7 用例；6 处变异检查按预期转红后还原；**并关闭 §5.1 N-6**） |

**本轮新发现（比 Q-B2 更严重，已登记 §6 第 25 行）**：`selectBestFrame()` 的结果
`bestFrameIndex_` **只写不读**，PnP 实际用的是**最后一次采集** `lastFrame_` ——
即 SYS-08 §5.7 的"**选择**最佳帧"没有作用到结果上。故 C-002 的实施顺序必须是
**先让最佳帧生效，再谈留存三路原图**。设计、修改清单、批准记录与实施证据见
[V2.1-C02_实施设计说明.md](V2.1-C02_实施设计说明.md) §10。

> **A 步（C-02）实施状态（2026-09-23）**：`MeasurementController` 的 CAPTURE → POSE_SOLVE
> 数据链已重构 —— `capturedFrames_` 改为 `vector<MultiCameraFrame>`（三路同步留存）、
> 新增 `bestFrame_` / `bestAcqIndex_` / `selection_`、`view` + `viewToAcq` 索引映射、
> `captureFrameCount` 夹取上界 100→20 且日志可见、任务终止处新增 75 MB 释放点；
> `lastQuality()` getter 改名 `bestQuality()`；`data` 层新增
> `MeasurementRecord` / `MeasurementStatistics` / `FailureTrace` / `StateTransition`。
> 证据：209 个测试用例 0 失败 + 离线探针全通过 + **三处变异检查均按预期转红后还原**
> （判据 1 / 4 / 6，"先红后绿"）。
>
> **Step 5 已完成（2026-09-23，v1.2）**：按裁决 A→B→C 三步实施 ——
> **A** `MeasurementRecord` 冻结为 **14 字段**（11 + `modelType` / `degraded` / `camerasAvailable`）；
> **B** 新增 `IPipelineObserver`（`onStatistics` 推送通道），`PosePipeline` 在**匹配完成、
> `correspondences.empty()` 判空之前**推送，`solvePose` 签名未动；
> **C** `Recorder` 组装 `result.json`。证据：**10 套件 242 用例 0 失败**、
> **变异检查 M1–M10 全部按预期转红后还原**（源文件与备份 `diff` 逐字节一致）。
> **实施中发现并修复两个真实缺陷**（都不是设计里的条目）：
> ① `startMeasurement()` 把 `retry_.beginTask()` 排在 `transition(IDLE)` **之后**，
> 使**第二次测量永远开不出来**（且只在"隔一会儿再点"时发作，首次后立刻再点会被掩盖）；
> ② `Recorder` 把非有限姿态写成 `nan`，**整份结果包事后解析失败，而 `save()` 返回 true**
> —— 新增 `num()`（非有限一律写 0）+ `result.json` 的 `non_finite_fields` 点名单。
> 详见 [C-02 §12.3](V2.1-C02_实施设计说明.md)。

**SDK 情况（O-8，✅ 已完成）**：SDK **已找到并落位** —— 原在 `…/proj_plane/sdk` 的
**MVviewer 客户端安装包**（Makeself 自解压）**内部**，2026-09-23 已**非执行解包**到
`third_party/imvsdk/`（**不跑厂商 `install.sh`**，避免污染系统）。
**查找、链接及版本调用均已通过**（真实调用 `IMV_GetVersion()` 得 `2.7.0.1.422704`）；
**尚未完成实机采集验收**（相机不在本机）。⚠ 不再需要"先提供 SDK 目录树"——
该前置已由 [核验报告](V2.1-011A0.1_ImvSDK环境核验报告.md) 结清。
包内真实名称为 `include/IMVApi.h` / `IMVDefines.h` / `IMVFGApi.h` / `IMVFGDefines.h`
与 `lib/m64x86/libMVSDK.so.2.7.0.1.422704`（**只有带版本号的文件**）——
与工程内 `FindImvSdk.cmake` 的**猜测完全不同**。故 011 拆为
**011-A0（SDK 适配准备，现在可做）/ 011-A1（真机采集，待 SDK 落位 + 相机）**。
⚠ 运行期目录**不得**包含包内的 `lib/m64x86/Qt/`（Qt **5.6.3**，与本工程的系统 Qt 5.15.8 冲突）。

**实施顺序（评审已定，2026-09-23 **三次**更新）**：`A. C-02` ✅ → `B. C-006/C-007/C-008` ✅
→ `C. C-009` ✅ → `D. N-6` ✅ → **`E. 011-A0 SDK 环境准备` ✅（已完成，见下）**
→ **`F. 011-A1 ImvCameraBackend 实现` ← 当前**
→ `G. 011-B 单相机真实预览（Camera→ImageFrame→Preview→Recorder）` → `H. 011-C 稳定性`
→ `I. 012 三相机` → `J. 013 CadLocatorBenchmark` → `K. 014 标定` → `L. 015 真实算法`；
C-001 / C-003 与 F 并行。

⚠ **动手改后端之前，先按验收清单逐项对一遍**：
**[《011-A1_A7A20MU201单相机接入验收清单》](011-A1_A7A20MU201单相机接入验收清单.md)**
（10 项按 `011-A1`/`011-B`/`011-C` 归属分列；含三处必须特别写清的事项 ——
**Mono12 与预览的衔接**、**时间戳**、**同步边界**；以及运行记录必须记载的
**实际后端类型 / 设备序列号 / 相机角色 / 各路真实与虚拟构成**）。
**没有设备或条件未满足的项目标"未执行／受阻"，不得勾选通过。**

⚠ **本段编号于 2026-09-23 第三次改写（评审第五节）**：原写
`E. 011-A0 真实相机接入 / F. 011-B ImvCameraBackend / G. 011-C 保存链 / H. 011-D 稳定性`。
评审把三段定名为 **`011-A0 SDK 环境准备` / `011-A1 ImvCameraBackend 实现` /
`011-B 单相机真实预览`**，且**保存链并入 011-B**（011-B 的定义就是
"真实 Camera → ImageFrame → Preview → Recorder"，拆开会让"预览通了但没存下来"
看起来像一个可接受的中间状态）。**三次 commit 边界**见
[011-A0 计划 §0.5.1](V2.1-011A0_真实相机接入实施计划.md)。

✅ **E 步（011-A0）已完成**（2026-09-23）：`third_party/imvsdk/` 解包 + 台账、
`FindImvSdk.cmake` 重写、`aps::imvsdk` 接口**端到端实测通过**（真实调用
`IMV_GetVersion()` 拿到 `2.7.0.1.422704`）。核验报告见
[V2.1-011A0.1_ImvSDK环境核验报告.md](V2.1-011A0.1_ImvSDK环境核验报告.md)。
⚠ 报告登记了 **3 个新缺陷**（`CMAKE_MODULE_PATH` 从未设置 / RUNPATH 不传递 /
`FAIL_MESSAGE` 列表切分），均已修复；另有 **2 个未闭环风险**必须先处理：
**内核 6.6.0 超出厂商上限**（011-A1 第一步）与**安装后 `deploy/bin` 找不到 SDK 库**
（011-B 必办）。

⚠ **本段于 2026-09-23 二次改写**：原先把 `010.5 CadLocatorBenchmark` 排在 011 之前
（"评审要求插入，优先级已提高"）。**该排序已撤销** —— 评审认定那是
**"设计验证过度"**，偏离了近期最有价值的工程目标。**注意这不是新增阶段，而是回到冻结顺序**：
[ENG-08](../项目文档/软件工程设计/ENG-08_第一阶段开发任务清单_V2.1_多相机闭环测量版.md) **§6 Sprint 2 = 真实单相机 SDK 接入**，**§10 Sprint 6 = 算法链接入
（才含 `CadStructureLocator` 可行性验证）** —— 冻结文档本就要求先接真实相机。
010.5 的插入把它提到了 Sprint 2 之前，是**对冻结顺序的颠倒**。
本计划的依据、现状盘点与验收判据见
[V2.1-011A0_真实相机接入实施计划.md](V2.1-011A0_真实相机接入实施计划.md)。

⚠ **本段此前过期**（原写"A 步已完成，下一步即 B 步 C-006/C-007"），于 2026-09-23
按实际进度改写。冻结路线见 [汇总 §5.1](待裁决问题汇总.md)。
**A 步（C-02 Step 1–6）已完成**；原先"B 步多一项前置（Step 5 的 `statistics` 来源）"
**已消除** —— 该阻塞已于 2026-09-23 经 `IPipelineObserver` 解掉（§6 第 27 行）。
**C-02 的 `frameId` 一致性判据已在 F 步之前裁决**：原始等式撤回，改为
`exposureIndex`（§6 第 28 行、[C-02 §12.2](V2.1-C02_实施设计说明.md)）；
⚠ **F 步（011-A1 `ImvCameraBackend` 实现）因此带两条硬性前置**：
交上来的 `frameId` 必须**单调**，且三路 `baseFrame` 首帧锁定**必须随 011-A1 完成**。
（原文写"必须在 011-A0 内完成、不得推迟到 011-A1"—— 那是**旧编号**。评审第五节的
重新编号把"后端实现"从 011-B 改为 **011-A1**，而 `frameId` / `baseFrame` 由后端产出，
故它们的归属随之移到 **011-A1**；011-A0 已于 2026-09-23 完成，不含此项。）

**~~010.5 CadLocatorBenchmark~~ → 013 CadLocatorBenchmark（⛔ 已暂停，2026-09-23 二次裁决）**：
原按"不依赖 SDK / 相机 / 转台 / CAD"排在 011 之前。**该排序已撤销** —— 评审认定属
"**设计验证过度**"："现在测出的东西：**只能证明数学模型。不能证明真实系统。**"
设计**冻结保留、不实现**，阶段编号改为 **013**，等有真实图像 / 镜头 / 标定数据再测。
这不是作废：**ENG-08 §10（Sprint 6）任务 A** 要求该验证的输入是
**Golden 数据集 + CAD 模型**、判据含**"与人工标注的结构点位置偏差 ≤ 0.5 pixel"**，
本阶段两者都不具备 —— 故它是 Sprint 6 的前置预研，不是替代。
**暂停前已测得的结果全部留档**（见 §6 第 57 行末段：三处设计纠正、`ss` 敏感性、
以及 **θ ≥ 30° 结构点被全部剔除**）。设计说明（含 ⛔ 状态横幅与 v1.3 修订行）：

**原设计说明（设计已批准、实施暂停）**：
[V2.1-C005_CadLocatorBenchmark测试设计说明.md](V2.1-C005_CadLocatorBenchmark测试设计说明.md)。

按该设计说明，输入维度为**超采样合成边**（亚像素真值须**解析**、逐点独立相位）
× **方向 `θ ∈ {0°,15°,30°,45°}`** × **噪声 `σ ∈ {0,5,10}` 灰度级** × 模糊 × 压缩；
方法为**三个**：**Baseline（现有 `Canny→fitLine→求交`）** / 方案 A（梯度剖面拟合）
/ 方案 B（`cornerSubPix`）；输出**分两级**且**不得混**：

1. **定位误差（pixel）**：`mean_error_px`（系统误差口径）、`std_error_px`（随机误差口径，
   即 SYS-15 §4.5 的 `σ_px`）、`max_error_px`、跳变数、伴随的 `max_residual_px`；
2. **姿态传播误差（arcmin）**：经 **synthetic PnP Monte Carlo** 独立求解得到 `σ_θ`
   —— **不得**由 pixel 数乘以任何系数换算（PnP 不是逐点线性叠加）。

只有测出这个数，才能判 C-005（亚像素定位）该不该做、做到什么程度 ——
否则就是在没有基线的情况下选方案。对应 C-005 的"`CadStructureLocator` 内部：
edge → subpixel fitting → 3D-2D point"是**测量之后**的结论，不是现在。

⚠ **落位是 `tests/algorithm/CadLocatorBenchmark/`，不是 `tests/optical/`**（评审 2026-09-23 裁决）：
`CadStructureLocator` 在 `src/algorithm/feature/`，而 `optical` 与 `algorithm` 是**兄弟层**
（都只依赖 `data`），放 `tests/optical/` 会形成测试侧跨层依赖。
评审理由：**"测试应该跟随被测模块，而不是跟随测试目的名称。"**

**已核实但与 v1.0 判断不同的一点**：Q-B3（`IMatchStatsStore::record()` 无调用者）
**不属 C-002 范围** —— 其入参是 `(targetModelId, role, distanceBand, illumBand, success)`，
不含 `MatchResult`；真实卡点是 `distanceBand` / `illumBand` **全仓库无生产者**，需单独立项。

### 8.4 仍待用户决定的其他事项

以下均已登记在 §5 / §6，**未擅自执行**：

1. 是否修订 ENG-03 §12.1，为 `data` 补一行 OpenCV（§6 第 4 行）；
2. 是否安装 GoogleTest（`sudo apt install libgtest-dev`，§5）；
3. 是否 `git init` 并补 001~010 的基线提交（§6 第 45/69/85/89 行）；
4. §6"内部歧义"第 3 条（9002 何时应可观测）等若干**需要裁决**的条目；
5. §6 第 71 行：008 之外是否新增操作控件（相机切换 / AUTO-MANUAL）；
6. §6 第 87 行：合成机型库 —— **已并入 C-003**（`models/aircraft/synthetic/`，release 拒绝加载）。

### 8.5 待确认项

**已裁决（2026-09-23）**：

| 编号 | 问题 | 裁决 | 归属 |
|---|---|---|---|
| **O-19** | SDK 落位方式：`make install` 装到系统 vs **解包到 `third_party/imvsdk/`** | **解包**（避免污染系统、避免 Qt 5.6.3 混入）；`FindImvSdk` 只找 `include` 与 `lib/m64x86`，不递归 Qt | ✅ **已实施**（011-A0，2026-09-23）：`third_party/imvsdk/` 解包树 + 台账，见 [核验报告](V2.1-011A0.1_ImvSDK环境核验报告.md) |
| **A-1** | 换相机重试时是否对**新焦段**重跑 `selectBestFrame` | **不重跑**（"换通道不换帧"；未来若发现不同焦段最佳时间不同，属 V2.2 优化） | C-02 §2.4 ✅已实施 |
| **A-2** | `captureFrameCount` 夹取上界由 **100 收到 20** | 是（§5.7 取值域 5~10，防 100×3×5 MB ≈ 1.5 GB 误配），且夹取须在日志可见 | C-02 §2.6 ✅已实施 |
| §5 | 可测性方案：改 VirtualCamera 图案 vs 加测试专用接口 | **方案 1**（"不要增加测试专用接口"；按 `frameId` 定分"容易变成测试作弊"） | C-02 §5 ✅已实施 |

**实施中新暴露、已于 2026-09-23 裁决并闭环**（原 4 条，全部关闭）：

| # | 问题 | 裁决 | 归属 |
|---|---|---|---|
| ~~**1**~~ | `IPosePipeline` 上没有 `lastMatchResult()` → `MeasurementStatistics` 6 个字段取不到（§6 第 27 行） | **否决增访问器**，改用 `IPipelineObserver` 推送通道；条文"**只能增加'结果输出'，禁止增加'内部状态查询'**" | ✅ C-02 §12.1 |
| ~~**2**~~ | `MeasurementRecord` 是否保留 `softwareVersion` | **保留**（H-002 要求） | ✅ C-02 §12.1 |
| ~~**3**~~ | `modelType` / `degraded` / `camerasAvailable` 是否入记录 | **全部入记录**（"不是状态机状态，而是**本次测量事实**"）；14 字段 | ✅ C-02 §12.1 |
| ~~**4**~~ | 三路原始 `frameId` 等式的真实硬件前提 | **等式撤回**（"这个在真实系统错误"），改 `exposureIndex`；基准锁定**必须在 011-A1 内完成**（原文写"011-A0"为**旧编号**，见 §8.5 末说明） | ✅ C-02 §12.2 |

**本轮（C-02 Step 5）新开的待确认项**（7 条；**5 / 7 / 10 / 11 已闭环，6 / 8 / 9 保留**）：

| # | 问题 | 归属 | 状态 |
|---|---|---|---|
| ~~**5**~~ | `Frame::synchronized`（`bool`）**仍未加**：`CameraSynchronizer` 在运行的应用里**从未被构造**，SYS-06 §8 的同步判据实际没在跑。加一个恒为 `true` 的字段只会是"注入了但没人读"的又一例，故**拒绝** | C-02 §12.5 | ✅ **关闭：维持"暂不加"**。判据**保留不删** —— 删了会让"同步没在跑"这件事从文档里消失；等接上 `CameraSynchronizer` 再一并解决。⚠ **本条关闭的只是"暂不增加字段"这个决定，不等于同步问题已解决**：`CameraSynchronizer` **至今仍未在运行的应用里被构造**，"同步器未接入"这一缺陷**仍然存在**（见 §6 第 27/28 行同族） |
| **6** | `exposureIndex = frameId − baseFrame` 是**相对量**：若目标只是"给设备日志做关联"，纯自增计数器更简单且无基准锁定失败的风险 | C-02 §12.5 | ⏸ **保留**：语义取舍需与 **011-A1** 的实机 `frameId` 语义一起定（原文写"011-A0"，该步已于 2026-09-23 完成且不含此项） |
| ~~**7**~~ | ~~**`PoseValidator` 对 NaN 是盲的**（重投影 / 内点率 / 置信度 / 偏航四处比较遇 NaN 一律为假 → 不触发任何拒绝）~~ | C-02 §12.5 | ✅ **关闭 → C-008**。⚠ **原文的事实描述需更正**（实施时实测）：`reprojectionError` 与 `inlierRatio` **已有** `isfinite` 守护，真正没拦的是 **`yaw` / `pitch` / `roll` 与 `aircraftToShip` 的 12 个元素**。裁决：由 `PoseValidator` 加**非有限值闸门**拒绝（第一道防线），Recorder 的点名表退为**最后一道**；`PoseValidationResult` 增 `reason` 分类 |
| **8** | `modelType` 已入记录但**取值仍空**，故落盘时出现在 `missing_required_fields` 里（**有意**让它可见） | C-003 | ⏸ **保留**（待 C-003 **实施开发用合成机型库**）。⚠ C-003 的裁决内容是 `models/aircraft/synthetic/` 且 **release 拒绝加载**，**不是**"交付真实机型库"，"已冻结"也**不等于"已实施"**。C-007 后该列表已收缩到只剩 `model_type` 一项 |
| **9** | `cam*.raw` 的格式：选"**无头缓冲区 + 几何写进 `result.json`**"，未选自描述容器 | C-02 §12.5 | ⏸ **保留**：代价是单独一个 `.raw` **不自证尺寸**（只影响成功包，失败包不含 raw） |
| ~~**10**~~ | `RecorderSinkAdapter` 的填充路径**无自动化覆盖** | C-02 §12.5 | ✅ **关闭 → C-009 已实施**（2026-09-23）。新增 `tests/integration/RecorderAdapterTest.cpp`（7 用例）。实测**无需搬迁任何文件**：它是 `src/app/ApplicationContext.h` 里的**内联类**，该头不含 Qt，测试直接 `#include` 即可（只需编译期版本宏） |
| ~~**11**~~ | `PosePipeline` 的 5 个诊断接口中 **4 个是死出口** | C-02 §12.5 | ✅ **关闭：已清理**（见 §6 第 29 行）。实测为 **6 个**访问器（不是 5 个）：2 个全工程零消费者、4 个仅测试消费 |
| ~~**N-6**~~（新） | **`config_snapshot/` 的填充零测试覆盖**：既有断言只验目录**存在**，而夹具的 `configDir` 不含 yaml —— 故它在**空快照**上照样通过，且失败消息写的是"复盘时要用**当时**的阈值"（**断言没验它自称要验的东西**） | 汇总 §5.1 N-6 | ✅ **关闭（随 C-009）**。**生产侧无缺陷**（探针：真 configDir → 7/7；空 configDir → 0 文件但 `save()` 仍 true、仅 `lastErrorText()` 报告）。已补三用例：真 configDir → 7 文件逐个在且非空；空 configDir → 必须报错**点名**缺失文件；部署侧 `config/*.yaml` 与落盘侧期望名集合一致。原断言的消息已改成实话 |

另有评审要求**必须写成测试**（不得只写在文档里）的两条：VirtualCamera `timestampNs` **严格递增**；
011-A 中 SDK **无设备时间戳时必须 `deviceTimestampNs = 0`**，**禁止** `deviceTimestampNs == timestampNs`。

> ### ⚠ 编号变更说明（2026-09-24，用于读上文所有 `011-A0`/`011-B` 字样）
>
> 评审第五节重新编号后，**"后端实现"这一步的名称从 `011-B` 改为 `011-A1`**。
> 凡**由 `ImvCameraBackend` 产出**的交付物（单调 `frameId`、`baseFrame` 首帧锁定、
> 逐路元数据、SDK 错误码映射、内核版本处置）**归属随之从 `011-A0` / `011-B` 移到 `011-A1`**。
> `011-A0` 已于 **2026-09-23 完成**，其范围只有"SDK 环境准备"，**不含上述任何一项**。
>
> **当前执行路线（以 §8.3 第三次调整为准）**：
> `011-A0 SDK 环境准备 ✅` → **`011-A1 ImvCameraBackend 实现` ← 当前** →
> `011-B 单相机真实预览与保存（Camera → ImageFrame → Preview → Recorder）` →
> `011-C 单相机稳定性` → `012 三相机验证` → `013 CadLocatorBenchmark` →
> `014 标定` → `015 真实算法`。
>
> 本文件中**逐处改过号**的位置已在本轮改为新编号；**历史叙述**（如 §8.3 对前两次调整的
> 复述、偏离登记里的原文引用）保留旧编号并就地标注"**已被后续裁决替代**"，**不做全局替换**。
