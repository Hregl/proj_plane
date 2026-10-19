# Qt 仿真交付证据（2026-10-20）

> **这批证据回答什么**：交付入口能否从干净构建树启动；主界面空格键能否在
> **同一进程内连续完成两次测量**；两次是否各自生成**独立、完整**的结果包。
>
> **这批证据不回答什么**：不是实机链路，不是测量精度，不是长时稳定性，
> 不是运动授权。检测器仍是 `MockTargetDetector`，纹理匹配数为 0。

---

## 1 代码基线

| 项 | 值 |
|---|---|
| 分支 | `develop` |
| 起始提交 | `cd37cbf`（批次 1 收口） |
| 主交付提交 | `a69df50`（37 个文件） |
| 补正提交 | `e0b61be`（本目录 11–14 号、UI 用例补强与哈希同步）；本页状态登记为随后一次文档提交 |
| 构建树 | `build-native-on` / `build-native-off`：两棵原生树**启用测试**；`build-simulation` / `build-simulation-fresh-20261020`：两棵 simulation 树**关闭 SDK 和测试** |
| 工具链 | GCC 12.3.0、CMake 3.31.4、Qt 5.15.8、OpenCV 4.6.0、GoogleTest 1.12.1 |
| 平台 | UOS Desktop 25；自动用例 `QT_QPA_PLATFORM=offscreen`，另有 `DISPLAY=:0` 真实 X 窗口运行 |

⚠ 06/07 的逐套件记录标注执行时的 HEAD。补正执行时主交付提交为
`a69df50`，补正改动尚未提交；这是执行时事实，不是当前工作树状态。

> **状态（2026-10-20，提交 `e0b61be` 复核通过）**：Qt 仿真启动入口与
> **指定场景**连续两次测量的自动验收，独立源码及证据复核通过
> （`e0b61be`）；**人工视觉验收仍未完成**。
>
> 下一步：真实窗口人工视觉检查并留截图。安装态启动、实机链路、测量
> 精度和长时运行继续单列验收。

---

## 2 材料清单

| 文件 | 内容 |
|---|---|
| `01_界面两次测量_ON.txt` | ON 树 `test_qt_simulation_ui` 完整输出：启动、两次空格、两个结果包 |
| `02_界面两次测量_OFF.txt` | OFF 树的同一用例完整输出 |
| `03_结果包摘录_ON/第一次_result.json` | ON 第一次结果包 `result.json` 原样摘录 |
| `03_结果包摘录_ON/第二次_result.json` | ON 第二次结果包 `result.json` 原样摘录 |
| `04_结果包摘录_OFF/第一次_result.json` | OFF 第一次结果包 `result.json` 原样摘录 |
| `04_结果包摘录_OFF/第二次_result.json` | OFF 第二次结果包 `result.json` 原样摘录 |
| `05_交付脚本启动.txt` | `scripts/run_simulation.sh` 端到端输出：CMake 配置、无 SDK 构建、模型生成、应用启动、`APP_FIRST_TICK_COMPLETED` |
| `06_逐套件回归_ON.txt` | ON 树 13 个套件逐个运行**完整记录**：命令、SDK 值、每条 `Passed`、通过/失败数、每套件 `exit_code`、总退出码 |
| `07_逐套件回归_OFF.txt` | OFF 树同一清单、同一格式的完整记录 |
| `08_真实窗口两次测量.txt` | `DISPLAY=:0` 下真实 X 窗口：窗口树/几何、两次 `xdotool` 空格时序、日志、结果包字段 |
| `09_真实窗口结果包摘录/第一次_result.json` | 真实窗口第一次结果包 `result.json` 原样摘录 |
| `09_真实窗口结果包摘录/第二次_result.json` | 真实窗口第二次结果包 `result.json` 原样摘录 |
| `10_安装启动自检_环境限制.txt` | `install_launch_check` 在 ON/OFF 两树的实测失败原因（现场 `config/` 需要真实相机/SDK） |
| `11_从零构建启动.txt` | 补正：在**执行前不存在**的 `build-simulation-fresh-20261020` 上跑交付脚本，含 49 条编译、12 条链接、`timeout` 退出码 124 |
| `12_第一包不变性_变异检查.txt` | 补正：定向改写第一包 `turntable.json` 后用例变红（长度与内容两条断言） |
| `13_第一包不变性_恢复后复跑.txt` | 补正：变异移除后的 ON/OFF 复跑，14 文件快照通过 |
| `14_模型目录重建.txt` | 补正：删除 `simulation/models/` 后由生成器重建，聚合 SHA-256 与删除前相同 |
| `SHA256SUMS` | 本目录除自身外全部文件的 SHA-256 |

**原始载荷**（结果包完整目录、三路 `cam*.raw`、配置快照、日志、模型文件）
不在本目录，按证据约定存放在仓库根的
`验收证据/Qt仿真_2026-10-20/`（被 `.gitignore` 排除，但必须留存）：

```text
验收证据/Qt仿真_2026-10-20/
├── MANIFEST.txt
├── SHA256SUMS              # 125 个文件逐件哈希
├── run_on/session_20261020_052456/simulation/
│   ├── config/ models/ logs/
│   └── output/measurement_2381029145433244/
│       output/measurement_2381029509079188/
├── run_off/session_20261020_052457/simulation/
│   ├── config/ models/ logs/
│   └── output/measurement_2381029898800940/
│       output/measurement_2381030280754046/
└── run_gui_x11/session_20261020_052945/
    ├── run.log             # 真实 X 窗口启动 + 两次测量日志
    └── simulation/
        ├── config/ models/ logs/
        └── output/measurement_2381318307093340/
            output/measurement_2381322353144937/
```

核验：

```bash
cd 验收证据/Qt仿真_2026-10-20 && sha256sum -c SHA256SUMS
```

⚠ 哈希不能替代文件。`/tmp` 下的工作目录会被清理，故不作为追溯依据；
持久目录中的结果包才是可重新解析的原始载荷。

---

## 3 运行事实

### 3.1 界面路径

用例：`QtSimulationUiTest.界面空格连续两次测量生成两个独立结果包`
（CTest 名 `test_qt_simulation_ui.*`）。

开始动作由 `QKeyEvent(Qt::Key_Space)` 送入 `MainWindow`，经
`MainWindow::keyPressEvent → startRequested` 的生产路径触发；**没有**直接调
`SystemInitializer::startMeasurement()`。配置是 `simulation/config/` 的
**原样拷贝**，不重写任何取值。

| 构建树 | 工作目录 | 第一次任务号 | 第二次任务号 | 结果 |
|---|---|---|---|---|
| ON | `run_on/session_20261020_052456` | `measurement_2381029145433244` | `measurement_2381029509079188` | 1 passed，733 ms |
| OFF | `run_off/session_20261020_052457` | `measurement_2381029898800940` | `measurement_2381030280754046` | 1 passed，753 ms |

四个结果包均为：

- `state=COMPLETE`、`run_mode=simulation`、`success=true`；
- 任务号与目录两两不同，第二次**没有**复用或覆盖第一次；
- `result.json` / `failure.json` / `turntable.json` / `calibration_id.txt` /
  `cam25.raw` / `cam50.raw` / `cam100.raw` / 7 个 `config_snapshot/*.yaml`
  齐全（完整清单见持久目录）。

补正后的用例还做**第一包不变性**校验：第一次测量结束后对整包 14 个文件
逐文件做字节快照；第二次测量结束后**重新读回**第一包的完整文件清单与内容，
逐项比较文件数、长度与字节。同时校验三路 RAW 的精确字节数
（1280×1024×3）、`calibration_id.txt` 含 `SYNTHETIC`、7 个配置快照齐全。
定向变异（第二次结束后向第一包 `turntable.json` 追加一个换行）使
“文件长度变化”和“文件内容被改写”两条断言同时变红，见
`12_第一包不变性_变异检查.txt`；恢复后两树复跑通过，见
`13_第一包不变性_恢复后复跑.txt`。现有六包未被改写。

### 3.2 真实 X 窗口（同一交付入口）

自动化用例在 `offscreen` 平台运行。为验证“可直接启动的窗口”本身，另做了一次
`DISPLAY=:0` 的真实 X 运行，**不使用** offscreen：

- 启动：`scripts/run_simulation.sh --no-build`；
- 窗口树中存在 `AircraftPoseSystem V2.1`，几何 `1600x984+0+0 +160+50`
  （`xwininfo` / `xdotool` 实测，见 `08_真实窗口两次测量.txt`）；
- 两次空格均由 `xdotool key --window` 送入该窗口：

| 次序 | 发送时刻 | 任务号 | 结果 |
|---|---|---|---|
| 第一次 | `05:29:45.509` | `measurement_2381318307093340` | `COMPLETE` |
| 第二次 | `05:29:49.554` | `measurement_2381322353144937` | `COMPLETE` |

- 两个结果包均为 `run_mode=simulation`、`success=true`，任务号与目录不同；
- 本机没有 `xwd`/`import`/`scrot` 等截图工具，**未留截图**；窗口已映射与按键
  路径有窗口树和日志证据，但这不等于人工视觉验收。

### 3.3 结果包内容（同一次合成场景）

| 字段 | 六次运行取值 |
|---|---|
| `model_id` / `model_type` | `synthetic_aircraft_6pt` / `synthetic` |
| `component_realism` | `camera=virtual;turntable=virtual;trigger=virtual;scene=synthetic_projection;detector=mock;scale=pinhole;features=sift;matcher=descriptor;pose=iterative_pnp` |
| 三路相机 | `CAM25/CAM50/CAM100` 均为 `backend=virtual`、`real_or_virtual=virtual` |
| 标定 | `SYNTHETIC-NO-CALIBRATION`（占位内参，外参单位阵） |
| `statistics` | CAD 点 6、纹理点 0、匹配数 6、冲突丢弃 0 |
| `validation.valid` / `reprojection_error` | `true` / 0.324563 px |
| 真值误差 | 旋转 0.321607°、平移 0.143426 m |
| 转台字段 | `azimuth=0`、`elevation=0`、`motion=IDLE`、控制帧与停止字段均为 false |

⚠ **真值误差不是精度验收。** 合成标定是占位值、检测器是桩，误差只用于
区分“链在解算”与“没解算却报成功”。同一场景六次结果一致，说明路径可重复，
不说明测量准确。

### 3.4 交付入口端到端

`QT_QPA_PLATFORM=offscreen timeout 20 ./scripts/run_simulation.sh`：

1. 配置 `build-simulation/`（该目录在 05 执行时已存在，故 05 只作为
   启动证据；从零目录见下方的 11），配置结果为 `APS_SDK_MODE=OFF`、
   未找到 ImvSdk/TurntableSdk、无 GoogleTest（预期，该树
   `BUILD_TESTING=OFF`）；
2. 构建 `AircraftPoseSystem` 与 `gen_synthetic_model` 两个目标；
3. 生成 `simulation/models/`：6 个 CAD 结构点、三个 24 字节空纹理库；
4. 应用启动并打印 `运行模式：simulation`、三路虚拟相机、机型库
   `synthetic_aircraft_6pt / 1`、`APP_FIRST_TICK_COMPLETED`。

退出码 124 来自 `timeout 20`，表示入口在观察窗口内持续运行，**不是**程序
自身失败。完整输出见 `05_交付脚本启动.txt`（该次在**已存在**的
`build-simulation/` 上启动，只作为启动证据）。

“从零构建”由 `11_从零构建启动.txt` 单独证明：执行前 `build-simulation-fresh-20261020`
不存在（`test -e` 返回 1），日志包含 49 条 `Building CXX`、12 条
`Linking CXX`、CMake 配置行、生成器输出、应用启动行与
`APP_FIRST_TICK_COMPLETED`，末尾记录 `TIMEOUT_EXIT=124`。

---

## 4 回归计数

两个构建树分别以**逐个套件**方式运行 13 个 `test_*` 目标，不使用通配汇总：

| 套件 | 用例数 |
|---|---:|
| `test_algorithm_pipeline` | 29 |
| `test_algorithm_stages` | 23 |
| `test_alignment_controller` | 21 |
| `test_device_layer` | 83 |
| `test_measurement_flow` | 46 |
| `test_optical_layer` | 48 |
| `test_pelco_d` | 87 |
| `test_preview_layer` | 40 |
| `test_qt_simulation_ui` | 1 |
| `test_recorder_adapter` | 8 |
| `test_recorder_package` | 29 |
| `test_state_machine` | 27 |
| `test_system_initializer` | 26 |
| **合计** | **468** |

ON / OFF 均 468 条通过、0 条失败，13 个套件逐个退出码均为 0，
`OVERALL_EXIT=0`。可复核记录见 `06_逐套件回归_ON.txt` /
`07_逐套件回归_OFF.txt`：每份含执行时间、当时的 HEAD、实际
`APS_SDK_MODE` / `APS_HAVE_IMVSDK` 值、逐套件命令行、ctest 完整输出
（每个 `Passed` 行与 `100% tests passed, 0 tests failed out of N`）
以及每套件的 `exit_code`；两份均以
`TOTAL tests=468 failed=0 suites=13` 收尾。**不是**从计数行补填“通过”。

CTest 清单另有 `install_check`、`install_launch_check` 两项安装检查，
**不计入**上表；CTest 总计 470 项。新增的 `test_qt_simulation_ui`
1 条**已包含在 468 内**（批次 1 为 467 条），不是 468＋1＝469。

⚠ **`install_launch_check` 当前两棵树都失败，且不是本批引入。** 它从安装
前缀以默认 `config/`（现场配置）启动：ON 树要求真实相机
`FD88772AAK00078`，本机当前枚举到 0 台；OFF 树没有 SDK，装配阶段即拒绝。
两者的退出码均为 1，未进入事件循环。`install_check` 本身两棵树都通过。
该检查不经过 `simulation/config`，与 Qt 仿真入口无关；实测输出见
`10_安装启动自检_环境限制.txt`。本批**没有**修改它，也未把它改写成
自动跳过——那会把环境限制藏起来。

---

## 5 配置差异与边界

`simulation/config/` 相对现场 `config/` 的 9 处取值差异及理由，见
`simulation/README.md §2`。它们是**仿真场景前提**，不是放宽验收判据：
重投影、内点率、置信度仍取现场值。

本批能证明：

- 无 SDK 构建树可由交付脚本从零配置、构建、生成模型并启动；
- `run_mode=simulation` 的一致性校验接受三路虚拟相机/虚拟转台/虚拟触发；
- 主界面空格键可启动测量，同一进程连续两次均到 `COMPLETE`（offscreen
  自动化用例与 `DISPLAY=:0` 真实 X 窗口各验一次）；
- 两次生成两个独立、字段完整的结果包，序号/目录不覆盖。

本批**不证明**：

- 人工视觉验收。真实 X 窗口已映射（1600×984）且按键路径成立，但本机无
  截图工具、未留截图；offscreen 用例只证明输入路径与状态机/落盘连通。
  `propagateSizeHints()` 警告是 offscreen 插件的已知提示。
- 通用场景适用性、测量精度、实机链路、动态转台外参、多相机曝光同步、
  正式测量准入、长时运行。
- 运动授权。转台发送门禁仍固定为只读，本批不发送任何实机控制帧。

---

## 6 复核补正记录（2026-10-20）

主交付提交 `a69df50` 复核后提出的四项，逐条处理如下；补正不改动
仿真结果、不放宽判据，也不扩大批次范围。

| # | 复核意见 | 补正产物 | 结论 |
|---|---|---|---|
| 1 | 468/468 缺可复核执行记录（只有 `total=数量`） | 重跑两树并保留完整输出：`06` / `07` | 各 468 通过、0 失败、13 套件退出码全 0、总退出码 0 |
| 2 | `05` 不能证明“从零构建”（只有 `Built target`） | 在原先不存在的目录上重跑：`11`；原 `05` 保留为已有构建树上的启动证据 | 49 编译 + 12 链接 + 超时 124 + 首 tick |
| 3 | UI 用例未验证第一包未被第二次改写 | 用例增加整包字节快照与重新读回比较、14 文件完整性、RAW 精确大小、标定标识、配置快照；变异反例 `12`，恢复复跑 `13` | 变异必红；恢复后两树绿；现有六包未发现覆盖 |
| 4 | 旧证据哈希表与 Qt README 基线未同步 | `../正式测试入口_2026-10-20/SHA256SUMS.txt` 重算（19 件全部通过）；本 README 更新基线 | 19/19 通过；历史 `cd37cbf` 的 19 件仍全部通过 |

**复核结论（2026-10-20，`e0b61be`）**：四项补正均通过独立复核——
468 个唯一通过项与源码逐名一致、13 套件退出码全 0；从零构建记录完整；
第一包重新读取与逐字节比较成立；Qt 哈希 18/18、正式测试入口哈希 19/19。
复核方另用 6 条独立 GoogleTest 探针验证了等长改写、缺文件、空文件、
RAW 截断均可检出。⚠ 两树 468 条仍属**开发方运行记录**，本次复核没有
重跑整个 Qt 工程。

**安装检查保持分列**：`install_check` 两树通过，`install_launch_check`
两树失败（ON 找不到 `FD88772AAK00078`，OFF 无 SDK）。现场配置与相关
启动源码与父提交逐字节相同，该硬件前提不是本批引入；本批未修改安装策略。

**未因补正重跑**：实机运动、测量精度、长时运行仍分别列为未验证项；
本批不发送任何实机控制帧。

---

## 7 复现命令

```bash
# 交付入口（从零构建并启动）
scripts/run_simulation.sh

# 自动验收：界面空格连续两次测量
ctest --test-dir build-native-on  -R "^test_qt_simulation_ui\." --output-on-failure
ctest --test-dir build-native-off -R "^test_qt_simulation_ui\." --output-on-failure

# 真实 X 窗口（需 DISPLAY 与 xdotool；窗口 id 每次不同）
DISPLAY=:0 scripts/run_simulation.sh --no-build &
wid=$(DISPLAY=:0 xdotool search --name '^AircraftPoseSystem V2.1$' | head -1)
DISPLAY=:0 xdotool key --window "$wid" space   # 等一次 COMPLETE
DISPLAY=:0 xdotool key --window "$wid" space   # 再等一次 COMPLETE

# 持久化证据复跑（结果留在 验收证据/）
APS_QT_SIM_WORKDIR="$PWD/../验收证据/Qt仿真_2026-10-20/run_on" \
  ./build-native-on/test_qt_simulation_ui
```
