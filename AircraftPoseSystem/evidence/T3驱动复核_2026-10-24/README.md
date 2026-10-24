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
| `04_测试输出_九条.txt` | 上述测试的完整输出（9 tests, OK, EXIT_CODE 0） |

逐件 SHA-256 见 `05_SHA256SUMS.txt`。

## 复核点与源码位置

### 1. 触发窗口只属于本次命令

* `MOVE_ACCEPTED` 时显式清空连续变化与基线
  （`01_t3_acceptance_driver.py:252-257`）。
* 命令受理前收到、但仍在队列里的 RX，由 `move_accepted` 门控忽略
  （`:357-359`）；基线只在受理后第一个有效样本建立。
* 3 个连续同向变化（`:395`）至少需要 4 个有效样本。
* `test_gap_resets_run_before_trigger` 先注入受理前的递减缓存，断言
  基线仍为空、未触发，再验证断档清零。

### 2. 停止结果只认本次 stop

* 只有 `stop_requested` 之后的 `STOP_RESULT` 才置 `stop_result_seen`
  （`:283-292`）；请求前的旧结果记 `STOP_RESULT_BEFORE_REQUEST`
  并忽略。
* 到位、拒绝、反向、超时、工具终结、读取异常都走同一收尾路径
  （`:518-568`、`:600-650`）。
* `no_stop_result` 假工具在启动时先输出一条无请求的旧结果，收到 stop
  后不再输出结果；测试断言旧结果未被采用、`stop_delivered` 保持未知。
* 变异检查：取消上述门控后该用例变红；恢复后 9/9 绿。

### 3. 收尾

* EOF 只在读取线程正常结束时置位；读取异常单独记 `reader_error`，
  不冒充 EOF（`:202-215`）。
* `_wait_for` 全部有期限（`:473-499`）；退出等待有期限，超时则强制
  终止并记 `forced_termination`（`:620-632`）。
* 实际进程退出码取自 `Popen.returncode`；`driver_ok` 同时要求
  `exit_code == 0`、EOF、无读取/日志/驱动异常（`:653-665`）。

## 运行

```text
cd AircraftPoseSystem
python3 tools/tests/test_t3_acceptance_driver.py -v
```

## 边界

离线假工具只验证驱动状态机；不证明真实 `STOP_REQUEST at_motion`
时刻、停止帧送达、反馈减速/平台，也不构成设备 ACK、独立物理静止或
制动性能认证。实机 T3 需另行现场确认与授权。
