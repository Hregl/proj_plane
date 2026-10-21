# 转台实机验收入口与验收运动许可（C-24）

> **状态：已实施待验证。**
> 软件与离线验证完成；**实机 T0 只读复测已于 2026-10-22 执行并通过**
> （控制帧 0，独立抓包未执行）；**T1–T5 未执行**。
> 2026-10-22 独立复核提出的 4 处代码缺口与 3 处执行卡问题已集中补正
> （工具 1.0.4-C24）：逐定位帧许可复查、T1 单次发送预算、会话级
> “最多一帧”、仅停止许可可安装；执行卡的前置阻塞、角度口径与 T5b
> 判别路径同步修正。**实机 T1 仍未执行**。本目录与其中的执行卡
> **不构成实机运动授权**。

## 1. 基线与范围

| 项 | 值 |
|---|---|
| 代码基线 | `develop` `65e4f64` + 本批工作树改动（另含未提交的 V1–V4 UI 批，本批未修改、未复用） |
| 工具 | `turntable_acceptance` `1.0.4-C24`（`AircraftPoseSystem/tools/turntable_acceptance.cpp`） |
| 构建 | CMake 3.31.4 / GCC 12.3.0 / UOS Desktop 25；`build-native-on`、`build-native-off` |
| 设备 | Pelco-D/UDP，`pelco_address=1`，端口 6666；**host 在记录中一律脱敏** |
| 验收范围 | 停止、单轴/双轴定位、状态反馈、正常退出、故障收口 |
| 不在范围 | 视觉闭环、精度/1′ 验收、转速控制协议（`use_speed_control=true`）、自动回零、自动重试整段序列 |

## 2. 本批代码改动

| 文件 | 改动 |
|---|---|
| `src/data/TurntableAcceptancePermit.h`（新增） | 验收运动许可值类型：设备、两项独立操作授权、两轴范围、单次位移、次数、有效期 |
| `src/data/TurntableDiagnostics.h` | 四个许可诊断字段 |
| `src/device/turntable/PelcoDSession.{h,cpp}` | 许可安装/核对；**每个定位帧发送前（含重传）复查期限与定位授权**，部分发送后到期进入停止收口，逻辑命令次数只扣一次；`sendAcceptanceStopFrameAtRest()`（T1：单次发送预算、会话级最多一帧、独立于运动事务的 `stopSent_`）；仅停止许可可安装 |
| `src/device/turntable/PelcoDTurntableController.{h,cpp}` | `enableAcceptancePermit()`、`lastAcceptedCommandSeq()`、命令槽预检；静止停止帧经 worker 停止屏障、结果按停止序号回传 |
| `tools/turntable_acceptance.cpp`（新增） | 独立验收工具：只读默认、许可解析（含 `allow_stop_at_rest`）、`stop_at_rest` 命令、TX/RX 单调时间记录、T0–T5 记录格式；SIGINT/SIGTERM 走停止路径；退出前等待停止观测窗口；`REPEAT` 按目标分组 |
| `tests/device/PelcoDAcceptancePermitTest.cpp`（新增） | **28 条**许可门禁/静止停止帧/复核反例用例（复核后新增 6 条） |

冻结文档已配对升版（旧版均已移入同级 `archive/`）：

* SYS-10 V1.3 §6.5／§19；SYS-04 V2.9 §8.1／§8.2／§21.1；
* ENG-09 V2.10 §5.34／§5.35／§6.3；SYS-17 V1.7 §8.3；SYS-06 V2.9 §11.2。

## 3. 已执行的验证

| 项 | 结果 |
|---|---|
| `test_pelco_d`（`build-native-on`） | **115/115** 通过 |
| `test_pelco_d`（`build-native-off`） | **115/115** 通过 |
| `test_system_initializer`（两树） | 26/26 通过 |
| `test_measurement_flow`（两树） | 46/46 通过 |
| 定向变异（19 项：原 8 + 静止停止帧 5 + 复核补正 6） | 每项先红后绿，见 `02_变异检查记录.md` |
| 离线假设备 A–O（15 组） | 见 `01_离线验证_结果摘录.txt`；J 为普通 `stop` 静止无帧、M 为 `stop_at_rest` 实发 1 帧、N 为未授权零帧、O 为仅停止许可（T1 1 帧、定位 0 帧）；K 为 Ctrl-C 停止路径，L 为 `REPEAT` 分组 |
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

## 6. 2026-10-22 复核补正对照

| 复核问题 | 补正 | 证据 |
|---|---|---|
| 许可到期后仍发送后续定位帧（含重传） | `sendAndAwaitEcho()` 每次发送（含重传）前复查期限与 `allowAbsolutePosition`；部分发送后到期进入停止收口；次数仍按逻辑命令扣一次 | 单测 `PermitExpiryBetweenAxesStopsWithoutSecondAxis`、`PermitExpiryDuringEchoWaitDoesNotRetransmit`；M14/M18 |
| T1 借用生产停止重试 | `performStop()` 增加 `singleAttempt`，T1 首次失败即返回失败、尝试数 1；普通/故障停止语义不变 | 单测 `AtRestStopSingleAttemptNoAutomaticRetry`；M15 |
| “会话最多一帧”被新运动重置 | 新增独立会话级 `acceptanceStopAtRestDelivered_`，不复用随运动事务清零的 `stopSent_` | 单测 `AtRestStopOnlyOncePerSessionAcrossMotion`；M16 |
| 仅停止许可无法安装 | 安装时要求两项授权至少一项；定位仍由 `move()` 门禁拒绝 | 单测 `StopOnlyPermitInstallsButNeverAuthorizesMotion`；工具离线 O 组；M17 |
| T5b 零帧无法归因锁存门禁 | 执行卡将实机 T5b 记为“未鉴别/未执行”；离线改用“固定时钟保持样本新鲜 + 看门狗锁存”的确定性路径，并删除“旧读数作目标也不会位移”的保证 | 单测 `PermitDoesNotBypassLatchedFaultWithFreshSample`；M19；执行卡 §T5b |
| 角度坐标口径冲突 | 执行卡 §2.0 区分软件有符号角与协议角（mod 360），统一单侧/双侧规则与 §2.1 具体值；不以数值跨度推定设备路径 | 执行卡 §2.0/§2.1 |
| 前置条件互相矛盾 | 执行卡 §1 改为逐项阻塞矩阵（P2 仅阻塞 T3，P3/P5/P6/P8 阻塞 T1–T5 等），保留未满足状态 | 执行卡 §1 |

T0 原始日志复核计数仍为 **2003 条 TX**（水平查询 1001、垂直查询 1001、工作模式查询 1）；该计数只支持工具日志口径，不替代独立抓包。
