# Qt 仿真交付（2026-10-20）

> **这是什么**：一条**不接任何真实设备**的可复现启动入口。启动后出现
> 主界面，按空格开始测量；一次测量完成后再次按空格，可在同一进程内
> 连续完成第二次测量，并各自生成结果包。
>
> **这不是什么**：不是实机链路证据，不是测量精度验收，不是长时稳定性
> 验收，也不是运动授权。结果包中的 `run_mode=simulation` 只描述本次
> 软件运行形态。

---

## 1 启动

在仓库根 `AircraftPoseSystem/` 下执行：

```bash
scripts/run_simulation.sh
```

脚本依次做三件事：

1. 用**独立的无 SDK 构建树** `build-simulation/` 配置并编译
   `AircraftPoseSystem` 与 `gen_synthetic_model`：
   `-DAPS_SDK_MODE=OFF -DBUILD_TESTING=OFF`。仿真不需要相机 SDK，
   也不把本机 SDK 可用性混进这条交付路径。
2. 从 `data/SyntheticTargetGeometry.h` **重新生成** `simulation/models/`。
   每次启动覆盖，避免“渲染几何”与“PnP 点表”来自不同版本。
3. `cd` 到仓库根后启动：

   ```bash
   build-simulation/AircraftPoseSystem "$PWD/simulation/config"
   ```

配置里的日志/结果/模型目录都是**相对进程当前目录**的路径，故脚本中的
`cd` 是必需的。手工启动时也必须从仓库根执行。

界面操作：

| 按键 | 行为 |
|---|---|
| `空格` | 开始一次测量；已完成后再次按下即开始下一次 |
| `Esc` | 请求停止当前测量 |

结果包位置：`simulation/output/measurement_<任务号>/`。日志：
`simulation/logs/aps_<日期>.log`。

跳过构建（已构建过）：

```bash
scripts/run_simulation.sh --no-build
```

---

## 2 `simulation/config/` 与现场配置 `config/` 的差异

`simulation/config/` 是 `config/` 的副本，只改下列取值；其余 3 个文件
（`optical_rig.yaml` / `turntable.yaml` / `trigger.yaml`）逐字相同：

| 文件 | 键 | 现场值 | 仿真值 | 理由 |
|---|---|---|---|---|
| system.yaml | `run_mode` | `hardware_readonly` | `simulation` | 启动时校验：任一真实相机/触发/转台即拒绝 |
| system.yaml | `log_dir` | `logs` | `simulation/logs` | 仿真日志与现场日志分开 |
| system.yaml | `output_dir` | `output` | `simulation/output` | 仿真结果包与现场结果包分开 |
| system.yaml | `model_dir` | `models` | `simulation/models` | 用生成器产物，不碰真实机型库 |
| camera.yaml | CAM25 `backend` | `imv` | `virtual` | 仿真不得打开真实相机 |
| camera.yaml | CAM25 `serial` | 实机序列号 | `""` | 虚拟后端没有设备身份可匹配 |
| camera.yaml | CAM25 `width`/`height` | 4096 × 3000 | 1280 × 1024 | synthetic 标定按 CAM25 尺寸生成内参并供三路共用；三路不一致会让主点错配 |
| measurement.yaml | `min_feature_count` | 8 | 4 | 合成几何只有 6 个 CAD 点；PnP 门槛为 `max(4, min_feature_count)` |
| validation.yaml | `yaw_min`/`yaw_max` | −5 / +5 | −120 / −60 | 默认合成真值按本工程坐标约定分解为 yaw ≈ −90°（迎头）；改的是场景可达范围，不是精度判据 |

⚠ 这些差异是**仿真场景前提**，不是放宽验收判据。重投影、内点率、
置信度三项仍取现场值；结果包仍记录实际装载的标定参数与真值。
修改 `config/` 的任一对应键时，应同步核对本表。

---

## 3 自动验收：界面空格连续两次测量

对应测试：`tests/integration/QtSimulationUiTest.cpp`，
CTest 名称 `test_qt_simulation_ui.*`。

它做四件事：

1. 把本目录的 `simulation/config/` **原样拷贝**到临时工作目录；
2. 用 `gen_synthetic_model` 的同一几何来源生成机型库；
3. 构造 `QApplication` + `MainWindow`，把
   `QKeyEvent(Qt::Key_Space)` 送入窗口（走
   `MainWindow::keyPressEvent` → `startRequested` 的生产路径，
   **不直接调** `SystemInitializer::startMeasurement()`）；
4. 第一次 COMPLETE 后对第一包做逐文件字节快照，再送一次空格；第二次
   COMPLETE 后断言两次的 `task_id`、结果目录不同，两份 `result.json`
   均为 `state=COMPLETE`、`run_mode=simulation`、`success=true`，并
   **重新读回第一包**逐文件、逐字节比较，证明它没有被第二次改写。
5. 两包各自校验完整性：14 个文件齐全（4 个顶层文件 + 3 路 RAW +
   7 个配置快照），三路 RAW 均恰为 `1280×1024×3` 字节，
   `calibration_id.txt` 为合成标定标识。定向改写第一包时用例必须变红
   （记录见 `evidence/Qt仿真_2026-10-20/12_第一包不变性_变异检查.txt`）。

运行：

```bash
ctest --test-dir build-native-on -R "^test_qt_simulation_ui\." --output-on-failure
```

持久化证据运行（结果包留在仓库根 `验收证据/`，该目录被 .gitignore
排除但必须留存）：

```bash
APS_QT_SIM_WORKDIR="$PWD/../验收证据/Qt仿真_2026-10-20/run" \
  ./build-native-on/test_qt_simulation_ui
```

⚠ 用例在 `QT_QPA_PLATFORM=offscreen` 下运行。它证明**界面输入路径与状态机/
落盘连通**，**不是**人工观察真实窗口的视觉验收。

另有 2026-10-20 的真实 X 运行（`DISPLAY=:0`）：窗口 `AircraftPoseSystem V2.1`
映射为 1600×984，两次空格经 `xdotool` 送入窗口，同样得到两个 `COMPLETE`
结果包。本机无截图工具，未留截图；窗口几何与按键时序见
`evidence/Qt仿真_2026-10-20/08_真实窗口两次测量.txt`。这仍不是精度、
实机或长跑验收。

---

## 4 结论边界

本入口能证明（仅在指定合成场景、指定构建快照下）：

- 三路虚拟相机 + 虚拟转台 + 虚拟触发可启动，`run_mode=simulation`
  拒绝混入真实部件；
- 主界面空格键可启动测量，同一进程内连续两次均到 `COMPLETE`；
- 两次测量生成两个独立结果包，序号/目录不覆盖。

本入口**不证明**：

- 通用场景适用性。当前合成场景为轴对齐、无背景网格；斜边光栅化与
  背景平行线干扰会触发现有 CAD 定位器拒绝（见
  `evidence/B_仿真链路_2026-10-20/03_定位器适用性复现.txt`）。
- 测量精度。合成标定是占位内参，检测器是 `MockTargetDetector`，
  纹理匹配数为 0；结果包中的真值误差只用于区分“链在解算”与
  “没解算却报成功”。
- 实机链路、动态转台外参、多相机曝光同步、正式测量准入、长时运行。
- 运动授权。转台发送门禁仍固定为只读；本入口不发送任何实机控制帧。
