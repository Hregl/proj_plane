# 转台实机验收入口与验收运动许可（C-24）

> **状态：已实施待验证。**
> 软件与离线验证完成；**实机 T0 只读复测已于 2026-10-22 执行并通过**
> （控制帧 0，独立抓包未执行）；**T1–T5 未执行**。
> T1 的“静止时实际发送停止帧”实现与正反测试已完成（工具 1.0.3-C24），
> 但**实机 T1 仍未执行**。本目录与其中的执行卡**不构成实机运动授权**。

## 1. 基线与范围

| 项 | 值 |
|---|---|
| 代码基线 | `develop` `65e4f64` + 本批工作树改动（另含未提交的 V1–V4 UI 批，本批未修改、未复用） |
| 工具 | `turntable_acceptance` `1.0.3-C24`（`AircraftPoseSystem/tools/turntable_acceptance.cpp`） |
| 构建 | CMake 3.31.4 / GCC 12.3.0 / UOS Desktop 25；`build-native-on`、`build-native-off` |
| 设备 | Pelco-D/UDP，`pelco_address=1`，端口 6666；**host 在记录中一律脱敏** |
| 验收范围 | 停止、单轴/双轴定位、状态反馈、正常退出、故障收口 |
| 不在范围 | 视觉闭环、精度/1′ 验收、转速控制协议（`use_speed_control=true`）、自动回零、自动重试整段序列 |

## 2. 本批代码改动

| 文件 | 改动 |
|---|---|
| `src/data/TurntableAcceptancePermit.h`（新增） | 验收运动许可值类型：设备、允许操作、两轴范围、单次位移、次数、有效期 |
| `src/data/TurntableDiagnostics.h` | 四个许可诊断字段 |
| `src/device/turntable/PelcoDSession.{h,cpp}` | 许可安装/核对；发送点复核；只拒新运动，stop/退出不受影响；`sendAcceptanceStopFrameAtRest()`（T1 静止停止帧） |
| `src/device/turntable/PelcoDTurntableController.{h,cpp}` | `enableAcceptancePermit()`、`lastAcceptedCommandSeq()`、命令槽预检；静止停止帧经 worker 停止屏障、结果按停止序号回传 |
| `tools/turntable_acceptance.cpp`（新增） | 独立验收工具：只读默认、许可解析（含 `allow_stop_at_rest`）、`stop_at_rest` 命令、TX/RX 单调时间记录、T0–T5 记录格式；SIGINT/SIGTERM 走停止路径；退出前等待停止观测窗口；`REPEAT` 按目标分组 |
| `tests/device/PelcoDAcceptancePermitTest.cpp`（新增） | **22 条**许可门禁/静止停止帧用例 |

冻结文档已配对升版（旧版均已移入同级 `archive/`）：

* SYS-10 V1.3 §6.5／§19；SYS-04 V2.9 §8.1／§8.2／§21.1；
* ENG-09 V2.10 §5.34／§5.35／§6.3；SYS-17 V1.7 §8.3；SYS-06 V2.9 §11.2。

## 3. 已执行的验证

| 项 | 结果 |
|---|---|
| `test_pelco_d`（`build-native-on`） | **109/109** 通过 |
| `test_pelco_d`（`build-native-off`） | **109/109** 通过 |
| `test_system_initializer` | 26/26 通过 |
| `test_measurement_flow` | 46/46 通过 |
| 定向变异（13 项：原 8 + 静止停止帧 5） | 每项先红后绿，见 `02_变异检查记录.md` |
| 离线假设备 A–N（14 组） | 见 `01_离线验证_结果摘录.txt`；J 为普通 `stop` 静止无帧、M 为 `stop_at_rest` 实发 1 帧、N 为未授权零帧；K 为 Ctrl-C 停止路径，L 为 `REPEAT` 分组 |
| 实机 T0 只读（60 s） | 控制帧 0、样本 valid/fresh、正常退出；见 `05_T0_实机只读_记录.md` |

**离线验证只证明工具与会话在假设备上的行为，不证明任何实机特性。**

## 4. 证据位置

| 内容 | 位置 |
|---|---|
| 本目录摘录、变异记录、执行卡、diff、T0 记录 | `AircraftPoseSystem/evidence/转台验收工具_C-24/` |
| 原始离线日志、假设备脚本、许可样例、配置副本 | `<仓库根>/验收证据/转台验收工具_2026-10-21/`（gitignored） |
| T0 实机原始日志（逐帧 TX/RX） | `<仓库根>/验收证据/转台验收工具_2026-10-22/`（gitignored） |
| 逐件 SHA-256 | 本目录 `SHA256SUMS` |
| 复核包（补丁＋执行卡＋T0/离线原始日志） | `<仓库根>/验收证据/转台验收_C-24_复核包_2026-10-22.zip`（gitignored；同名 `.sha256`） |

`/tmp` 不作为追溯依据；哈希不能代替原始载荷。

## 5. 结论边界

* “停止帧已交付” ≠ 设备已收到；`stopFrameDelivered` 只表示交给传输层。
* “反馈角稳定” ≠ 独立实测物理静止；T4 只给**设备反馈重复性**，不是绝对精度，
  更不是 1′ 验收。
* 许可安装 ≠ `WorkMode` 正常；`WorkModeTrust` 保持实际值。
* `move()==true` 只表示**已受理**，不表示交付或到位。
* 实机 T0–T5 必须在执行卡参数经用户确认并另行书面授权后进行。
