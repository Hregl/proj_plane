# 正式测试入口验证 + B 批复核收口（2026-10-20）

> **这是什么**：在**真实 GoogleTest + 项目原生 CMake/CTest** 路径上，
> 对两条构建树（有 SDK / 无 SDK）各做一次全量执行，并复跑 B 批指定
> 合成场景与 R08/R10/R11 定向用例的本地证据。
>
> **这不是什么**：不是实机验收，不是精度验收，不是运动授权；也不是
> 对 CMake 3.16–3.19、ASan/UBSan 的验证。本批没有新增功能，只修了一个
> 阻塞原生无 SDK 构建的测试缺陷（见 §2.3、14 号补丁）。
>
> **状态**：本机证据已完成（已实施待验证）。最终关闭待固定 SHA 的独立复核。

---

## 1 材料清单

| # | 文件 | 内容 | 类别 |
|---|---|---|---|
| 01 | [01_run_tests_ON.txt](01_run_tests_ON.txt) | `APS_SDK_MODE=ON` 原生配置/构建/CTest 全量执行 | 本机执行 |
| 02 | [02_run_tests_OFF.txt](02_run_tests_OFF.txt) | `APS_SDK_MODE=OFF` 原生全量执行 | 本机执行 |
| 03 | [03_ctest_N_ON.txt](03_ctest_N_ON.txt) | ON 构建树的 CTest 完整登记清单（467） | 本机执行 |
| 04 | [04_ctest_N_OFF.txt](04_ctest_N_OFF.txt) | OFF 构建树的 CTest 完整登记清单（467） | 本机执行 |
| 05 | [05_B_scenarioS.txt](05_B_scenarioS.txt) | 场景 S 正例（`COMPLETE`）与三个启动拒绝负例 | 本机执行 |
| 06 | [06_R08.txt](06_R08.txt) | R08 重复 CAD 对应点定向用例 | 本机执行 |
| 07 | [07_R10.txt](07_R10.txt) | R10 标定校验/拒绝定向用例（21 条） | 本机执行 |
| 08 | [08_R11.txt](08_R11.txt) | R11 保存事务定向用例（5 条） | 本机执行 |
| 09 | [09_真值误差独立复算.py](09_真值误差独立复算.py) | 不引用工程源码的误差复算脚本 | 复算方法 |
| 10 | [10_真值误差独立复算.out](10_真值误差独立复算.out) | 复算输出 | 本机执行 |
| 11 | [11_ctest_零匹配负探针.txt](11_ctest_零匹配负探针.txt) | 零匹配必须非 0 退出（实测 exit=8） | 本机执行 |
| 12 | [12_GoogleTest来源与配置.txt](12_GoogleTest来源与配置.txt) | GTest 版本/包哈希/CMake 实际使用路径 | 环境事实 |
| 13 | [13_shim逐套件重跑.txt](13_shim逐套件重跑.txt) | 测试文件修改后 12 个垫片套件逐个重跑（467） | 本机执行 |
| 14 | [14_测试修正diff.patch](14_测试修正diff.patch) | 本批唯一的源码修改（无 SDK 测试分支） | 源码变更 |
| 15 | [15_B结果包摘录/](15_B结果包摘录/) | 本次 `COMPLETE` 包的 result/failure/turntable/calibration_id 摘录 | 应用实际落盘 |

**持久化原始载荷**（不在版本库）：仓库根
`验收证据/正式测试入口_2026-10-20/`，含完整 `result_package/`
（三路 `cam*.raw`、7 份 `config_snapshot/`、`result.json`、
`failure.json`、`turntable.json`、`calibration_id.txt`）与
`sim_logs/`，逐件 SHA-256 见同目录 `SHA256SUMS`。

---

## 2 源码快照与构建形态

### 2.1 快照

- 本批快照：**包含本目录的提交**。查询命令：
  `git log -1 --format=%H -- evidence/正式测试入口_2026-10-20`
  （提交正文不回填自身 SHA，避免循环追加）。
- 该提交的父基线：`b3c9b9c`（develop）；唯一源码修改是
  `tests/integration/SystemInitializerTest.cpp`（见 14 号补丁）。
- B 批源码提交：`618856f`；C-23 软件整改最终复核快照：`78da4a7`
  （两者均未在本批改动）。

### 2.2 两条构建树

| 构建目录 | 配置入口 | 实际检测结果 |
|---|---|---|
| `build-native-on` | `APS_SDK_MODE=ON` | `APS_HAVE_IMVSDK=TRUE`，找到 ImvSdk |
| `build-native-off` | `APS_SDK_MODE=OFF` | `APS_HAVE_IMVSDK=FALSE`，明确跳过 ImvSdk |

两条树均通过仓库内正式入口执行（不是手工拼 ctest）：

```bash
cd AircraftPoseSystem
# 首次配置时显式给出本地 GTest 的 CMake 包目录；之后 run_tests.sh
# 复用缓存（脚本本身不接收 GTest_DIR 参数）。
G=/home/hjm/opt/gtest-1.12.1/root/usr/lib/x86_64-linux-gnu/cmake/GTest
cmake -S . -B build-native-on  -DBUILD_TESTING=ON -DAPS_SDK_MODE=ON  -DGTest_DIR="$G"
cmake -S . -B build-native-off -DBUILD_TESTING=ON -DAPS_SDK_MODE=OFF -DGTest_DIR="$G"

APS_SDK_MODE=ON  BUILD_DIR=$PWD/build-native-on  bash tests/run_tests.sh
APS_SDK_MODE=OFF BUILD_DIR=$PWD/build-native-off bash tests/run_tests.sh
```

`tests/run_tests.sh` 会核对 CMakeCache 中的**实际** `APS_HAVE_IMVSDK`，
在“要求 SDK 但没找到”或“要求 OFF 却检测到 SDK”时直接失败。

### 2.3 本批唯一的阻塞缺陷（已修）

`场景D_显式请求真实后端不可用即启动失败且不静默换虚拟` 原断言无条件要求
`ctx.backend25 != nullptr`。无 SDK 构建下后端**根本不会构造**，该断言
在原生 OFF 构建中失败（垫片路径此前未暴露这一分支）。

修正只把**后端对象存在性与身份检查**按构建形态分派：
OFF 下要求 `backend25 == nullptr`；ON 下要求非空、非
`VirtualCameraBackend`、且不得把配置序列号当成实际设备身份。

**共同失败判据原样保留、两条构建都执行**（位于分派之前的同一段）：

- `initialize(dir) == false`；
- `ready() == false`；
- 错误文本点名 `cam25` 与 `imv`；
- 不得半装配：`ctx.cameras == nullptr`、`ctx.controller == nullptr`。

即：修正的是“OFF 下后端根本不会构造”这一错误测试前提，不是放宽
“启动必须失败、不得静默回落虚拟”的要求。修正后 OFF/ON 两条原生树均
467/467。补丁只有测试文件，无生产代码变更。

---

## 3 原生执行结果

| 构建 | 配置 | 编译 | CTest | 退出码 | 用时 |
|---|---|---|---|---|---|
| ON | 成功，`APS_HAVE_IMVSDK=TRUE` | 成功 | **467/467 passed，0 failed** | **0** | 9.36 s |
| OFF | 成功，`APS_HAVE_IMVSDK=FALSE` | 成功 | **467/467 passed，0 failed** | **0** | 8.48 s |

- 日志均为 `tests/run_tests.sh` 的完整输出，含配置、编译、
  `ctest -R "^test_" --no-tests=error` 的逐条执行与最终统计。
- 退出码 0 的日志内证：脚本为 `set -euo pipefail`，只有 CTest 返回 0
  才会打印末行 `[run_tests] 全部已注册测试通过`；两份日志均含该行。
- CTest 清单 03/04 的 `Total Tests: 467` 与执行日志的
  `467/467` 一致。

### 3.1 执行前清单的观测事实（原因尚未定位）

首次执行前，`ctest -N` 当时观察到的是 **370 条**，不是完整的 467 条。
该次输出在后续重采时被完整清单覆盖，**执行前逐条清单没有留存**，
因此本包**不提供前后差集，也不把差异原因写成结论**。

- 可保留的观测：同一构建树，执行前 370 条，执行后 467 条；
- 未定位：相差的 97 条为何未出现在执行前清单中；
- `gtest_discover_tests(... DISCOVERY_MODE PRE_TEST)`（CMake ≥ 3.18）
  会延后测试发现，这只是**可能解释之一**，单凭数量变化不能证明；
- 本包 03/04 是**执行后重采**的完整清单（`Total Tests: 467`），与
  执行日志的 `467/467` 相互独立地一致。

本包不补造执行前清单。最终以“执行后登记清单 467 = 实际执行 467”
为验收事实；清单登记与执行的一致性见 03/04 与 01/02。

### 3.2 Qt 定向用例确实执行

`test_pelco_d.PelcoDTurntableControllerTest.QtEventLoopDispatchesStopWhileMoveInFlight`
在 ON 与 OFF 日志中均有 `Start` 与 `Passed   0.23 sec` 记录
（ON 日志第 217/467 条）。该目标在 `tests/device/CMakeLists.txt:39`
链接 `Qt5::Core`；这是本项目中真正使用 `QCoreApplication`/`QTimer`
的用例，不是“链接了 Qt 但没起事件循环”。

### 3.3 `test_system_initializer` 确实执行

ON/OFF 日志均含该目标全部 26 条，包括：

- `场景S_合成靶标六点非共面性可复核`
- `场景S_仿真运行模式走通生产链并落盘真值对照`
- `场景S_模型标识相同但点表不同源时拒绝启动`
- `场景S_仿真模式拒绝混入真实部件`
- `场景S_机型库不是合成几何时拒绝启动`

### 3.4 零匹配负探针

```text
ctest --test-dir build-native-on -R "^definitely_no_such_test$" --no-tests=error
→ No tests were found!!! / Errors while running CTest / exit=8
```

证明本入口不会把“一条用例都没匹配到”报告成通过。

---

## 4 与垫片 467 的逐项对照

两条原生构建树与垫片路径的**每个套件用例数完全一致**，没有注册、
排除或参数化差异：

| 套件 | 原生 ON | 原生 OFF | 垫片 |
|---|---:|---:|---:|
| `test_algorithm_pipeline` | 29 | 29 | 29 |
| `test_algorithm_stages` | 23 | 23 | 23 |
| `test_alignment_controller` | 21 | 21 | 21 |
| `test_device_layer` | 83 | 83 | 83 |
| `test_measurement_flow` | 46 | 46 | 46 |
| `test_optical_layer` | 48 | 48 | 48 |
| `test_pelco_d` | 87 | 87 | 87 |
| `test_preview_layer` | 40 | 40 | 40 |
| `test_recorder_adapter` | 8 | 8 | 8 |
| `test_recorder_package` | 29 | 29 | 29 |
| `test_state_machine` | 27 | 27 | 27 |
| `test_system_initializer` | 26 | 26 | 26 |
| **合计** | **467** | **467** | **467** |

说明：此前 B 批证据记录的 454 是当时垫片快照的用例数；本批在
`78da4a7` 之后的源码上重新构建，12 个套件逐个重跑（13 号文件），
总数为 467。**没有为了让数字对齐而增删或过滤用例**。

---

## 5 GoogleTest 来源

- 包：`libgtest-dev_1.12.1-0.2_amd64.deb`（UOS/Debian 源）。
- SHA-256：`e53f75127338c750b9c624c8036e0a27f6b44138c9e26efcf6b2c62e23384ce1`。
- 本机无 root，未做系统级安装；用 `dpkg-deb -x` 解包到
  `/home/hjm/opt/gtest-1.12.1/root/usr`，CMake 实际使用
  `GTest_DIR=/home/hjm/opt/gtest-1.12.1/root/usr/lib/x86_64-linux-gnu/cmake/GTest`。
- 详细清单与库文件哈希见 12 号文件。

---

## 6 B 批场景 S 复跑

### 6.1 成功链路

- 任务号：`measurement_2379404865646228`（本次原生 ON 运行新生成）。
- 终态：`COMPLETE`；`run_mode=simulation`；
  `model_id=synthetic_aircraft_6pt`、`model_type=synthetic`、
  `feature_version=1`。
- 状态迁移：10 条，全部 `error.code=0`；`failure.json` 保留正常历史，
  无失败记录。
- 三路相机：`backend=virtual`、`real_or_virtual=virtual`。
- 真值来源：`truth_source=synthetic_geometry`；`metadata_conflicts=[]`。

### 6.2 实际标定参数（结果包原样记录）

`calibration_id=SYNTHETIC-NO-CALIBRATION`（合成标定，不具测量意义）。
三路相机参数相同，外参为单位变换：

```text
fx=fy=1545.096679919
cx=640.000000000  cy=512.000000000
image=1280x1024
dist=[0, 0, 0, 0, 0]
cameraToRig R=I, t=[0,0,0]；rigToShip R=I, t=[0,0,0]
```

⚠ 该标定是“默认矩阵标定”，**不能**用于任何精度结论。

### 6.3 真值误差与独立复算

| 指标 | 应用包内记录 | 独立复算（09/10） | 差值 |
|---|---:|---:|---:|
| 旋转误差 | 0.321607° = 19.296 角分 | 0.321605° | 2.3e-6° |
| 平移误差 | 0.143426 m = 143.4 mm | 0.143427 m | 5.6e-7 m |
| 重投影误差 | 0.324563 px | —（仅图像拟合） | — |

差值来自 `result.json` 对真值/估计值的小数位舍入，不是算法不一致。
复算脚本只读取结果包 JSON，按 `acos((trace(R_truthᵀR_est)-1)/2)`
与欧氏距离计算，不调用工程代码。

**19.296 角分不构成一角分精度证据**；0.324563 px 也不能替代 CAD 定位器
的 0.3 px 中位残差判据。

### 6.4 负例

同一次定向执行中：

- `场景S_模型标识相同但点表不同源时拒绝启动` — 通过；
- `场景S_仿真模式拒绝混入真实部件` — 通过（cam25 配 `imv` 时拒绝启动）；
- `场景S_机型库不是合成几何时拒绝启动` — 通过。

---

## 7 R08 / R10 / R11 定向结果

| 项 | 命令过滤 | 用例数 | 结果 |
|---|---|---:|---|
| R08 | `DescriptorMatcherDoesNotDuplicateCadPointOnConflict` | 1 | 1/1 passed |
| R10 | `CalibrationDirFixture` | 21 | 21/21 passed |
| R11 | `RecorderPackageTest.(FailedSave…/ExistingPackage…/SuccessfulSave…/SecondSave…)` | 5 | 5/5 passed |

R11 覆盖：失败保存不留最终/暂存目录、既有包不被覆盖、既有空目录不被
替换、成功只发布最终目录、同一任务第二次保存不能替换第一次。

---

## 8 边界（不得合并进“全部已验证”）

- **CMake 版本**：本机仅 3.31.4。项目最低 3.16；`TEST_PREFIX` 自 3.16
  可用，但 `DISCOVERY_MODE PRE_TEST` 自 3.18。3.16–3.19 未实测。
- **ASan/UBSan**：本批未重跑；此前结果不覆盖新增用例。
- **复核方环境**：C-23 复核方报告的官方 GTest 85/86 条仍是复核方环境，
  本批没有把其转写成本机证据。
- **实机**：本批不连接真实 UDP/相机；只读会话控制帧要求仍为 0。
- **B 批范围**：仅指定合成场景；通用场景适用性、精度、动态外参、
  多相机同步、正式测量准入均不在内。
- 本批没有新增运动授权；软件测试通过不是实机验收，也不是运动许可。

---

## 9 哈希

本目录逐件 SHA-256 见 `SHA256SUMS.txt`；需要复核时可重新生成：

```bash
cd AircraftPoseSystem/evidence/正式测试入口_2026-10-20
find . -type f ! -name SHA256SUMS.txt -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS.txt
```

持久化原始载荷的哈希在仓库根
`验收证据/正式测试入口_2026-10-20/SHA256SUMS`（与源码提交无关，
不随版本库变化）。
