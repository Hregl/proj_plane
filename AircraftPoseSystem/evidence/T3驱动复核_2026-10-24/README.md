# T3 驱动复核材料（2026-10-24）

> 范围仅限新增 T3 驱动。未修改生产控制器、许可门禁或已关闭的
> `t2_acceptance_driver.py`；未连接设备、未发送 UDP、未执行 T3。
> 供复核使用，不代表已通过独立复核。

## 文件

| 文件 | 对应工作树路径 |
|---|---|
| `01_t3_acceptance_driver.py` | `tools/t3_acceptance_driver.py` |
| `02_fake_t3_tool.py` | `tools/tests/fake_t3_tool.py` |
| `03_test_t3_acceptance_driver.py` | `tools/tests/test_t3_acceptance_driver.py` |
| `04_测试输出_九条.txt` | 复核前九条输出（历史，9 tests, OK） |
| `09_测试输出_十一条.txt` | 第一轮复核修复后输出（历史，11 tests, OK） |
| `12_测试输出_十二条.txt` | 本次完整输出（12 tests, OK, EXIT_CODE 0） |
| `10_t3_real_stop_lines.txt` | 真实工具输出格式回放样本 |
| `11_stop_id工具改动.diff` | `turntable_acceptance` 增加 `stop_id` 的最小改动 |

逐件 SHA-256 见 `05_SHA256SUMS.txt`。

## 复核点与源码位置

### 1. 触发窗口只属于本次命令

* `MOVE_ACCEPTED` 时显式清空连续变化与基线
  （`01_t3_acceptance_driver.py:253-264`）。
* 命令受理前收到、但仍在队列里的 RX，由 `move_accepted` 门控忽略
  （`:394-396`）；基线只在受理后第一个有效样本建立。
* 3 个连续同向变化（`:439`）至少需要 4 个有效样本。
* `test_gap_resets_run_before_trigger` 先注入受理前的递减缓存，断言
  基线仍为空、未触发，再验证断档清零。

### 2. 停止结果只认本次 stop

* `turntable_acceptance` 每次普通 `stop` 生成 `stop_id`，在同一行的
  `STOP_REQUEST` 与 `STOP_RESULT` 中打印同一个值（`11_...diff`）。
  控制器 `request_seq`/`completed_seq` 保持原义，驱动只记录、不据此
  猜号配对。
* 驱动解析 `STOP_REQUEST stop_id`（`:290-294`）。
* 只有同时满足以下条件的 `STOP_RESULT` 才置 `stop_result_seen`：
  已发出本次 stop；该行接收时刻不早于 stop 请求时刻；`stop_id` 与
  本次 `STOP_REQUEST` 一致（`:296-333`）。
* 已排队但接收于请求之前的旧结果记 `STOP_RESULT_BEFORE_REQUEST`；
  `stop_id` 不符记 `STOP_RESULT_ID_MISMATCH`；两者都忽略。
* 真实工具只读验证：仓库根
  `验收证据/转台低速与T3准备_2026-10-24/46_只读stop_id格式验证.log`
  两行 `stop_id=1`，`request_seq=0 completed_seq=0`，`control_frames=0`。
* 反例：先放旧结果、后处理 stop，旧结果仍被拒；`stop_id` 不符也被拒。
* 变异检查：取消 `stop_id` 配对或接收时刻门控后
  `test_queued_old_stop_result_is_rejected` 变红；恢复后 12/12 绿。

### 3. 反向锁存

* 达到反向判据后立即置 `direction_fault` 并发送一次普通 stop
  （`:425-432`）；后续递减样本因 `stop_requested` 被忽略，不能改回
  正常触发。
* 反例：`3.19→3.20→3.18→3.16→3.14` 必须停在反向点，不得在后续
  三个递减变化后记 `triggered`。
* 变异检查：去掉反向 stop 后 `test_reverse_then_normal_cannot_trigger`
  变红；恢复后 12/12 绿。

### 4. 收尾

* EOF 只在读取线程正常结束时置位；读取异常单独记 `reader_error`，
  不冒充 EOF（`:202-215`）。
* `_wait_for` 全部有期限；退出等待有期限，超时则强制终止并记
  `forced_termination`。
* 实际进程退出码取自 `Popen.returncode`；`driver_ok` 同时要求
  `exit_code == 0`、EOF、无读取/日志/驱动异常（`:700-712`）。

## 运行

```text
cd AircraftPoseSystem
python3 tools/tests/test_t3_acceptance_driver.py -v
```

## 执行前复测（T3 未执行）

2026-10-24 19:44 在授权时间窗内做了一次 10 s 只读复测：

* 全部 11 条 `STATE` 为 `az=-0.5300`、`el=3.1900`、`IDLE`、`valid=1`、
  `fresh=1`；
* `control_frames=0`、`query_frames=335`、退出码 `0`。

现场确认的运动范围为 `az[-0.5200,-0.5200]`、`el[1.5600,3.1700]`；
实际起点方位低 `0.0100°`、俯仰高 `0.0200°`，均超出。按指示停止，
未启动带许可的会话，未发送定位或停止帧，未自动扩范围、未重试。
详见 `06_T3执行前复测_未执行.md` 与 `07_...log`。

## 第二次现场尝试（20:00–21:00，T3 未执行）

* 20:06 只读预检：11/11 条 `az=-0.4900 el=3.1900 IDLE valid=1 fresh=1`；
  `control_frames=0`、`query_frames=335`、退出码 `0`。起点超出首版
  包络，停止并请示；现场随后批准固定包络
  `az[-0.5800,-0.4600]`、`el[3.1400,3.2400]`，目标与位移上限不变。
* 21:25 重建后的工具只读 `stop`+`quit`：两行 `stop_id=1`、
  `control_frames=0`、退出码 `0`。
* 21:26 带许可启动时已超过 21:00 窗口；工具输出
  `PERMIT_REJECTED "许可已过期，拒绝安装"`，`CONTROL_FRAMES sent=0`，
  驱动 `OUTCOME=init_timeout`、`STOP_SEND_COUNT=0`、退出码 2。
  未打开 socket、未发定位或停止帧，未在窗口外重试。详见仓库根
  `验收证据/转台低速与T3准备_2026-10-24/48_...md`。

## 边界

离线假工具只验证驱动状态机；不证明真实 `STOP_REQUEST at_motion`
时刻、停止帧送达、反馈减速/平台，也不构成设备 ACK、独立物理静止或
制动性能认证。实机 T3 需另行现场确认与授权。
