#!/usr/bin/env python3
"""T3 单次定位＋运动中停止执行驱动（离线可假工具验证）.

职责边界：

* 只驱动 ``turntable_acceptance`` 的交互 stdin，不实现 Pelco-D 协议；
* 定位命令只发送一次；**不重试定位**；
* 从工具 stdout 的 RX 角度回复解码俯仰角，只有连续、新鲜、方向一致的
  至少 ``--trigger-samples`` 个变化才发送一次普通 ``stop``；
* 未观测到方向一致的运动、触发等待超时、已先到位、工具终结或初始化失败
  时，都进入同一条停止收尾：发送一次 ``stop``、等待 ``STOP_RESULT``、
  收集输出至 EOF；
* 不把“stop 已发送”“STOP_RESULT 已出现”或“反馈平台”写成设备 ACK、
  独立物理静止或制动性能认证；这些由现场日志/曲线另行判读；
* 不修改生产控制器、许可门禁或验收阈值。

时间戳为驱动进程读到该行时的 ``time.monotonic()``；工具日志里的
``t=...ms`` 是工具自己的 ``steady_clock`` 主机接收时刻，两者都**不是**
设备内部采样时刻。
"""

from __future__ import annotations

import argparse
import queue
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

INIT_TIMEOUT_S = 10.0
STATE_TIMEOUT_S = 5.0
MOVE_ACK_TIMEOUT_S = 5.0
MOTION_TIMEOUT_S = 3.0
STOP_WAIT_S = 3.0
STOP_OBSERVE_S = 5.0
EXIT_TIMEOUT_S = 10.0

RX_RE = re.compile(
    r"^RX t=(\d+)ms n=\d+ kind=\S+ bytes "
    r"FF ([0-9A-Fa-f]{2}) ([0-9A-Fa-f]{2}) ([0-9A-Fa-f]{2}) "
    r"([0-9A-Fa-f]{2}) ([0-9A-Fa-f]{2}) ([0-9A-Fa-f]{2})$"
)
STATE_RE = re.compile(r"\bSTATE\b.*\baz=([-+0-9.]+)\s+el=([-+0-9.]+)")
MOVE_ACCEPTED_RE = re.compile(r"\bMOVE_ACCEPTED\s+accepted=(\d+)")
STOP_RESULT_RE = re.compile(
    r"\bSTOP_RESULT\b.*?\bstop_delivered=(\d).*?"
    r"\bstop_stationary=(\d).*?\bsend_path=(\S+)"
)
EXIT_CODE_RE = re.compile(r"^\s*EXIT_CODE\s+(-?\d+)\s*$")
NORMAL_EXIT_RE = re.compile(r"\bnormal_exit=(\d+)\b")


@dataclass
class T3Result:
    driver_ok: bool
    outcome: str
    reason: str
    exit_code: Optional[int]
    log_path: str
    notes_path: str
    start_state: Optional[Tuple[float, float]] = None
    move_accepted: bool = False
    stop_requested: bool = False
    stop_write_ok: bool = False
    stop_send_count: int = 0
    trigger_samples: int = 0
    trigger_tool_ms: Optional[int] = None
    trigger_angle: Optional[float] = None
    trigger_deltas: List[float] = field(default_factory=list)
    arrival_before_trigger: bool = False
    stop_result_seen: bool = False
    stop_result_before_request: int = 0
    stop_delivered: Optional[bool] = None
    stop_stationary: Optional[bool] = None
    stop_send_path: Optional[str] = None
    stop_observed_stationary: bool = False
    stop_not_observed: bool = False
    eof_seen: bool = False
    timed_out: bool = False
    forced_termination: bool = False
    reader_error: Optional[str] = None
    log_error: Optional[str] = None
    driver_exception: Optional[str] = None
    elapsed_s: float = 0.0


class _Session:
    def __init__(
        self,
        command: Sequence[str],
        log_path: Path,
        notes_path: Path,
        move_az: float,
        move_el: float,
        address: int,
        direction: str,
        trigger_samples: int,
        min_step_deg: float,
        fresh_s: float,
        max_gap_s: float,
        init_timeout_s: float,
        state_timeout_s: float,
        move_ack_timeout_s: float,
        motion_timeout_s: float,
        stop_wait_s: float,
        stop_observe_s: float,
        exit_timeout_s: float,
    ) -> None:
        self.command = list(command)
        self.log_path = Path(log_path)
        self.notes_path = Path(notes_path)
        self.move_az = move_az
        self.move_el = move_el
        self.address = address
        self.direction = direction
        self.trigger_samples = trigger_samples
        self.min_step_deg = min_step_deg
        self.fresh_s = fresh_s
        self.max_gap_s = max_gap_s
        self.init_timeout_s = init_timeout_s
        self.state_timeout_s = state_timeout_s
        self.move_ack_timeout_s = move_ack_timeout_s
        self.motion_timeout_s = motion_timeout_s
        self.stop_wait_s = stop_wait_s
        self.stop_observe_s = stop_observe_s
        self.exit_timeout_s = exit_timeout_s

        self.proc: Optional[subprocess.Popen] = None
        self.log = None
        self.q: "queue.Queue[Tuple[float, str]]" = queue.Queue()
        self.eof = threading.Event()
        self.reader_done = threading.Event()
        self.reader_error: Optional[str] = None
        self.log_error: Optional[str] = None
        self.driver_exception: Optional[str] = None
        self.reader: Optional[threading.Thread] = None

        self.init_ok = False
        self.state_seen = False
        self.start_state: Optional[Tuple[float, float]] = None
        self.move_accepted = False
        self.move_rejected = False
        self.move_accepted_at: Optional[float] = None
        self.terminal_reason: Optional[str] = None
        self.arrival_before_trigger = False
        self.stop_requested = False
        self.stop_write_ok = False
        self.stop_send_count = 0
        self.stop_requested_at: Optional[float] = None
        self.stop_result_seen = False
        self.stop_result_before_request = 0
        self.stop_delivered: Optional[bool] = None
        self.stop_stationary: Optional[bool] = None
        self.stop_send_path: Optional[str] = None
        self.stop_observed_stationary = False
        self.stop_not_observed = False
        self.triggered = False
        self.trigger_tool_ms: Optional[int] = None
        self.trigger_angle: Optional[float] = None
        self.trigger_deltas: List[float] = []
        self.last_sample_recv: Optional[float] = None
        self.last_sample_angle: Optional[float] = None
        self.last_sample_tool_ms: Optional[int] = None
        self.consecutive_deltas: List[float] = []
        self.saw_direction = False
        self.saw_opposite = False
        self.timed_out = False
        self.forced_termination = False
        self.quit_sent = False
        self.exit_request_seen = False
        self.exit_stop_seen = False
        self.final_seen = False
        self.normal_exit = False
        self.exit_code_line: Optional[int] = None
        self.notes: List[str] = []
        self.started_at = 0.0

    # ---- 日志与输出收集 -------------------------------------------------
    def _write(self, text: str) -> None:
        if self.log is None:
            return
        try:
            self.log.write(text)
            self.log.flush()
        except (OSError, ValueError) as exc:
            if self.log_error is None:
                self.log_error = "%s: %s" % (type(exc).__name__, exc)

    def _note(self, text: str) -> None:
        self.notes.append(text)
        self._write("DRIVER_NOTE " + text + "\n")

    def _safe_note(self, text: str) -> None:
        try:
            self._note(text)
        except Exception as exc:
            self.notes.append("NOTE_FAILED %s: %s" % (type(exc).__name__, exc))

    def _reader_main(self, stream) -> None:
        try:
            while True:
                line = stream.readline()
                if line == "":
                    break
                self.q.put((time.monotonic(), line))
        except Exception as exc:
            self.reader_error = "%s: %s" % (type(exc).__name__, exc)
        finally:
            if self.reader_error is None:
                self.eof.set()
            self.reader_done.set()

    def _handle_line(
        self, line: str, recv_at: Optional[float] = None,
        cutoff: Optional[float] = None,
    ) -> None:
        self._write(line)
        if self.log_error is not None:
            return
        if cutoff is not None and recv_at is not None and recv_at > cutoff:
            self._note("LATE_EVENT late_by=%.3fs line=%r"
                       % (recv_at - cutoff, line.strip()))
            return
        self._apply_line(line, recv_at)

    def _apply_line(self, line: str, recv_at: Optional[float] = None) -> None:
        if "INIT_OK" in line:
            self.init_ok = True
            return

        m = STATE_RE.search(line)
        if m:
            try:
                if self.start_state is None:
                    self.start_state = (float(m.group(1)), float(m.group(2)))
                self.state_seen = True
            except ValueError:
                pass
            return

        m = MOVE_ACCEPTED_RE.search(line)
        if m:
            if int(m.group(1)) == 1:
                self.move_accepted = True
                self.move_accepted_at = time.monotonic()
                # 本次命令的触发窗口从这里开始：此前缓存的 RX 不能作为
                # 基线或变化计数。3 个变化至少需要 4 个有效样本。
                self.consecutive_deltas = []
                self.last_sample_recv = None
                self.last_sample_angle = None
                self.last_sample_tool_ms = None
                self._note("MOVE_ACCEPTED seq=1 trigger_window_reset=1")
            else:
                self.move_rejected = True
                self._note("MOVE_REJECTED accepted=0")
            return

        if "MOVE_NOT_ACCEPTED" in line:
            self.move_rejected = True
            self._set_terminal("MOVE_NOT_ACCEPTED: " + line.strip())
            return

        if "ARRIVAL_OBSERVED" in line:
            if not self.stop_requested and not self.triggered:
                self.arrival_before_trigger = True
                self._note("ARRIVAL_BEFORE_TRIGGER " + line.strip())
            return

        if "CMD_RESULT" in line:
            if "outcome=Delivered" not in line:
                self._set_terminal("CMD_RESULT non-delivered: " + line.strip())
            return

        if "MOTION_ERROR" in line:
            self._set_terminal("MOTION_ERROR: " + line.strip())
            return

        m = STOP_RESULT_RE.search(line)
        if m:
            if not self.stop_requested:
                # 只接受本次 stop 请求之后的 STOP_RESULT；会话开始或
                # 其他路径留下的旧结果不得解除本次收尾等待。
                self.stop_result_before_request += 1
                self._note("STOP_RESULT_BEFORE_REQUEST ignored path=%s"
                           % m.group(3))
                return
            self.stop_result_seen = True
            self.stop_delivered = int(m.group(1)) == 1
            self.stop_stationary = int(m.group(2)) == 1
            self.stop_send_path = m.group(3)
            self._note(
                "STOP_RESULT delivered=%d stationary=%d path=%s"
                % (int(m.group(1)), int(m.group(2)), m.group(3))
            )
            return

        if "STOP_OBSERVED_STATIONARY" in line:
            self.stop_observed_stationary = True
            self._note("STOP_OBSERVED_STATIONARY seen")
            return
        if "STOP_NOT_OBSERVED" in line:
            self.stop_not_observed = True
            self._note("STOP_NOT_OBSERVED seen")
            return

        if "EXIT_REQUEST" in line:
            self.exit_request_seen = True
            return
        if "EXIT_STOP" in line:
            self.exit_stop_seen = True
            return
        if "FINAL" in line:
            self.final_seen = True
            nm = NORMAL_EXIT_RE.search(line)
            if nm:
                self.normal_exit = int(nm.group(1)) == 1
            return
        m = EXIT_CODE_RE.match(line)
        if m:
            try:
                self.exit_code_line = int(m.group(1))
            except ValueError:
                pass
            return

        self._on_angle_line(line, recv_at)

    def _on_angle_line(self, line: str, recv_at: Optional[float]) -> None:
        m = RX_RE.match(line.strip())
        if not m or recv_at is None:
            return
        t_ms = int(m.group(1))
        addr, d1, d2, d3, d4, cks = (
            int(m.group(i), 16) for i in range(2, 8)
        )
        if (addr + d1 + d2 + d3 + d4) & 0xFF != cks:
            self._note("RX_BAD_CHECKSUM t=%dms" % t_ms)
            return
        if addr != self.address:
            self._note("RX_WRONG_ADDRESS t=%dms addr=0x%02X expected=0x%02X"
                       % (t_ms, addr, self.address))
            return
        if d1 != 0x00 or d2 != 0x5B:
            return
        raw = (d3 << 8) | d4
        angle = raw / 100.0
        self._on_angle_sample(recv_at, t_ms, angle)

    def _on_angle_sample(
        self, recv_at: float, t_ms: int, angle: float
    ) -> None:
        if (not self.move_accepted or self.triggered or self.stop_requested
                or self.arrival_before_trigger):
            return
        now = time.monotonic()
        if now - recv_at > self.fresh_s:
            self._note("STALE_SAMPLE tool_t=%dms age=%.3fs"
                       % (t_ms, now - recv_at))
            return

        if self.last_sample_recv is None:
            self.consecutive_deltas = []
        else:
            gap = recv_at - self.last_sample_recv
            tool_gap = (t_ms - self.last_sample_tool_ms) / 1000.0
            delta = angle - self.last_sample_angle
            if tool_gap <= 0 or gap > self.max_gap_s or tool_gap > self.max_gap_s:
                self._note("SAMPLE_GAP_RESET recv_gap=%.3fs tool_gap=%.3fs"
                           % (gap, tool_gap))
                self.consecutive_deltas = []
            else:
                if self.direction == "decrease":
                    good = delta <= -self.min_step_deg
                    opposite = delta >= self.min_step_deg
                else:
                    good = delta >= self.min_step_deg
                    opposite = delta <= -self.min_step_deg
                if good:
                    self.saw_direction = True
                    self.consecutive_deltas.append(delta)
                elif opposite:
                    self.saw_opposite = True
                    self.consecutive_deltas = []
                else:
                    self.consecutive_deltas = []
        self.last_sample_recv = recv_at
        self.last_sample_angle = angle
        self.last_sample_tool_ms = t_ms

        if len(self.consecutive_deltas) >= self.trigger_samples:
            self.triggered = True
            self.trigger_tool_ms = t_ms
            self.trigger_angle = angle
            self.trigger_deltas = list(self.consecutive_deltas)
            self._note(
                "T3_TRIGGER samples=%d tool_t=%dms angle=%.4f "
                "deltas=%s"
                % (self.trigger_samples, t_ms, angle,
                   ",".join("%.4f" % d for d in self.trigger_deltas))
            )
            self._send_stop("trigger")

    def _set_terminal(self, reason: str) -> None:
        if self.terminal_reason is None:
            self.terminal_reason = reason
            self._note("TOOL_TERMINAL " + reason)

    # ---- 发送 -----------------------------------------------------------
    def send(self, command: str) -> bool:
        if self.proc is None or self.proc.poll() is not None:
            self._note("SEND_SKIPPED process_not_running cmd=%r" % command)
            return False
        self._write('DRIVER_SEND cmd="%s"\n' % command)
        try:
            assert self.proc.stdin is not None
            self.proc.stdin.write(command + "\n")
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError) as exc:
            self._note("SEND_FAILED cmd=%r error=%s" % (command, exc))
            return False

    def _send_stop(self, reason: str) -> None:
        if self.stop_requested:
            return
        self.stop_requested = True
        self.stop_send_count += 1
        self.stop_requested_at = time.monotonic()
        self._note("T3_STOP_REQUEST reason=%s" % reason)
        self.stop_write_ok = self.send("stop")
        self._note("T3_STOP_WRITE write_ok=%d"
                   % (1 if self.stop_write_ok else 0))

    def _send_quit(self) -> None:
        if self.quit_sent:
            return
        self.quit_sent = True
        self.send("quit")
        self._close_stdin()

    def _close_stdin(self) -> None:
        if self.proc is None or self.proc.stdin is None:
            return
        try:
            self.proc.stdin.close()
        except OSError:
            pass

    # ---- 等待 -----------------------------------------------------------
    def _pump_once(
        self, timeout_s: float, deadline: Optional[float] = None
    ) -> bool:
        try:
            recv_at, line = self.q.get(timeout=max(0.0, timeout_s))
        except queue.Empty:
            return False
        self._handle_line(line, recv_at, deadline)
        return True

    def _drain_available(self, cutoff: Optional[float] = None) -> None:
        while True:
            try:
                recv_at, line = self.q.get_nowait()
            except queue.Empty:
                return
            self._handle_line(line, recv_at, cutoff)

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        label: str,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            if self.log_error is not None:
                return False
            if self.reader_error is not None and self.q.empty():
                self._drain_available(deadline)
                return False
            if predicate():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._drain_available(deadline)
                self.timed_out = True
                self._note("WAIT_TIMEOUT label=%s after=%.2fs"
                           % (label, timeout_s))
                return predicate()
            if (self.proc is not None and self.proc.poll() is not None
                    and self.reader_done.is_set() and self.q.empty()):
                self._drain_available(deadline)
                return predicate()
            self._pump_once(min(0.05, remaining), deadline)

    def _wait_exit(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            self._drain_available()
            if (self.proc is not None and self.proc.poll() is not None
                    and self.reader_done.is_set() and self.q.empty()):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._drain_available()
                return (self.proc is not None
                        and self.proc.poll() is not None
                        and self.reader_done.is_set()
                        and self.q.empty())
            time.sleep(min(0.02, remaining))

    # ---- 主流程 ---------------------------------------------------------
    def _execute(self) -> str:
        if not self._wait_for(lambda: self.init_ok,
                              self.init_timeout_s, "INIT_OK"):
            return "init_timeout"
        if not self._wait_for(lambda: self.state_seen,
                              self.state_timeout_s, "STATE"):
            return "state_timeout"

        if not self.send("move %.4f %.4f" % (self.move_az, self.move_el)):
            return "move_send_failed"
        accepted = self._wait_for(
            lambda: self.move_accepted or self.move_rejected
            or self.terminal_reason is not None,
            self.move_ack_timeout_s, "MOVE_ACK")
        if not accepted:
            return "move_ack_timeout"
        if self.move_rejected:
            return "move_rejected"
        if self.terminal_reason is not None:
            return "tool_terminal"

        # 触发或超时；stop 只由这里（或收尾路径）发送一次。
        self._wait_for(
            lambda: (self.triggered or self.arrival_before_trigger
                     or self.terminal_reason is not None),
            self.motion_timeout_s, "TRIGGER")

        if self.triggered:
            outcome = "triggered"
        elif self.arrival_before_trigger:
            outcome = "arrival_before_trigger"
        elif self.terminal_reason is not None:
            outcome = "tool_terminal"
        elif self.saw_direction:
            outcome = "trigger_timeout"
        elif self.saw_opposite:
            outcome = "wrong_direction_timeout"
        else:
            outcome = "no_motion_timeout"

        if not self.stop_requested:
            self._send_stop("closeout:" + outcome)

        # 停止收尾：先等 STOP_RESULT，再等反馈平台（有交付时）。
        self._wait_for(lambda: self.stop_result_seen or self.terminal_reason,
                       self.stop_wait_s, "STOP_RESULT")
        if (self.stop_result_seen and self.stop_delivered
                and not self.stop_observed_stationary
                and not self.stop_not_observed):
            self._wait_for(
                lambda: (self.stop_observed_stationary
                         or self.stop_not_observed),
                self.stop_observe_s, "STOP_OBSERVED")

        self._send_quit()
        return outcome

    def run(self) -> T3Result:
        self.started_at = time.monotonic()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.notes_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("w", encoding="utf-8", newline="\n")

        outcome = "driver_error"
        fatal: Optional[BaseException] = None
        try:
            try:
                self.proc = subprocess.Popen(
                    self.command, stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="strict", bufsize=1,
                )
                assert self.proc.stdout is not None
                self.reader = threading.Thread(
                    target=self._reader_main,
                    args=(self.proc.stdout,),
                    name="t3-driver-reader", daemon=True)
                self.reader.start()
                outcome = self._execute()
            except BaseException as exc:
                self.driver_exception = "%s: %s" % (
                    type(exc).__name__, exc)
                self._safe_note("DRIVER_EXCEPTION " + self.driver_exception)
                outcome = "driver_exception"
                if not isinstance(exc, Exception):
                    fatal = exc

            if (self.proc is not None and self.proc.poll() is None
                    and not self.stop_requested):
                try:
                    self._send_stop("failure_closeout")
                except Exception as exc:
                    self._safe_note("STOP_SEND_FAILED %s" % exc)
            if self.proc is not None and self.proc.poll() is None:
                if self.stop_requested and not self.stop_result_seen:
                    self._wait_for(
                        lambda: self.stop_result_seen,
                        self.stop_wait_s, "STOP_RESULT_FAILURE")
                self._send_quit()

            exit_ok = False
            if self.proc is not None:
                exit_ok = self._wait_exit(self.exit_timeout_s)
            if not exit_ok and self.proc is not None:
                if self.proc.poll() is None:
                    self.forced_termination = True
                    self._note("EXIT_TIMEOUT after=%.2fs; killing process"
                               % self.exit_timeout_s)
                    try:
                        self.proc.kill()
                    except OSError:
                        pass
                    try:
                        self.proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        self._note("KILL_WAIT_TIMEOUT")
                self._drain_available()
                self.timed_out = True
                if outcome == "triggered":
                    outcome = "exit_timeout"

            self._drain_available()
            if self.reader is not None:
                self.reader.join(timeout=1.0)
            self._drain_available()

            if self.driver_exception is not None:
                outcome = "driver_exception"
            elif self.log_error is not None:
                outcome = "log_error"
            elif self.reader_error is not None:
                outcome = "reader_error"

            exit_code = (self.proc.returncode
                         if self.proc is not None else None)
            driver_ok = (
                outcome in (
                    "triggered", "no_motion_timeout", "trigger_timeout",
                    "wrong_direction_timeout", "arrival_before_trigger",
                    "move_rejected")
                and self.stop_requested
                and self.stop_write_ok
                and self.stop_result_seen
                and exit_code == 0
                and self.eof.is_set()
                and self.reader_error is None
                and self.log_error is None
                and self.driver_exception is None
                and not self.forced_termination
            )
            result = T3Result(
                driver_ok=driver_ok,
                outcome=outcome,
                reason=outcome,
                exit_code=exit_code,
                log_path=str(self.log_path),
                notes_path=str(self.notes_path),
                start_state=self.start_state,
                move_accepted=self.move_accepted,
                stop_requested=self.stop_requested,
                stop_write_ok=self.stop_write_ok,
                stop_send_count=self.stop_send_count,
                trigger_samples=len(self.trigger_deltas),
                trigger_tool_ms=self.trigger_tool_ms,
                trigger_angle=self.trigger_angle,
                trigger_deltas=list(self.trigger_deltas),
                arrival_before_trigger=self.arrival_before_trigger,
                stop_result_seen=self.stop_result_seen,
                stop_result_before_request=self.stop_result_before_request,
                stop_delivered=self.stop_delivered,
                stop_stationary=self.stop_stationary,
                stop_send_path=self.stop_send_path,
                stop_observed_stationary=self.stop_observed_stationary,
                stop_not_observed=self.stop_not_observed,
                eof_seen=self.eof.is_set(),
                timed_out=self.timed_out,
                forced_termination=self.forced_termination,
                reader_error=self.reader_error,
                log_error=self.log_error,
                driver_exception=self.driver_exception,
                elapsed_s=time.monotonic() - self.started_at,
            )
            self._write_result(result)
            self._write_notes(result)
            if fatal is not None:
                raise fatal
            return result
        finally:
            if self.proc is not None and self.proc.poll() is None:
                try:
                    self._close_stdin()
                except Exception:
                    pass
                try:
                    self.proc.kill()
                    self.proc.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            if self.log is not None:
                try:
                    self.log.close()
                except OSError:
                    pass
            if (self.proc is not None and self.proc.stdout is not None
                    and self.reader_done.is_set()):
                try:
                    self.proc.stdout.close()
                except OSError:
                    pass

    def _write_result(self, result: T3Result) -> None:
        self._write(
            "T3_DRIVER_RESULT ok=%d outcome=%s exit_code=%s "
            "stop_requested=%d stop_write_ok=%d stop_send_count=%d trigger_samples=%d "
            "stop_result_seen=%d stop_delivered=%s stop_stationary=%s "
            "eof=%d timed_out=%d forced=%d\n"
            % (1 if result.driver_ok else 0, result.outcome,
               result.exit_code, 1 if result.stop_requested else 0,
               1 if result.stop_write_ok else 0, result.stop_send_count, result.trigger_samples,
               1 if result.stop_result_seen else 0,
               str(result.stop_delivered), str(result.stop_stationary),
               1 if result.eof_seen else 0,
               1 if result.timed_out else 0,
               1 if result.forced_termination else 0))

    def _write_notes(self, result: T3Result) -> None:
        lines = [
            "T3_DRIVER_OK=%d" % (1 if result.driver_ok else 0),
            "T3_OUTCOME=%s" % result.outcome,
            "EXIT_CODE=%s" % result.exit_code,
            "MOVE_ACCEPTED=%d" % (1 if result.move_accepted else 0),
            "STOP_REQUESTED=%d" % (1 if result.stop_requested else 0),
            "STOP_WRITE_OK=%d" % (1 if result.stop_write_ok else 0),
            "STOP_SEND_COUNT=%d" % result.stop_send_count,
            "TRIGGER_SAMPLES=%d" % result.trigger_samples,
            "TRIGGER_TOOL_MS=%s" % result.trigger_tool_ms,
            "TRIGGER_ANGLE=%s" % result.trigger_angle,
            "TRIGGER_DELTAS=%s" % ",".join(
                "%.4f" % d for d in result.trigger_deltas),
            "ARRIVAL_BEFORE_TRIGGER=%d"
            % (1 if result.arrival_before_trigger else 0),
            "STOP_RESULT_SEEN=%d" % (1 if result.stop_result_seen else 0),
            "STOP_RESULT_BEFORE_REQUEST=%d"
            % result.stop_result_before_request,
            "STOP_DELIVERED=%s" % result.stop_delivered,
            "STOP_STATIONARY=%s" % result.stop_stationary,
            "STOP_SEND_PATH=%s" % result.stop_send_path,
            "STOP_OBSERVED_STATIONARY=%d"
            % (1 if result.stop_observed_stationary else 0),
            "STOP_NOT_OBSERVED=%d" % (1 if result.stop_not_observed else 0),
            "EOF=%d" % (1 if result.eof_seen else 0),
            "TIMED_OUT=%d" % (1 if result.timed_out else 0),
            "FORCED_TERMINATION=%d"
            % (1 if result.forced_termination else 0),
            "READER_ERROR=%s" % (result.reader_error or ""),
            "LOG_ERROR=%s" % (result.log_error or ""),
            "DRIVER_EXCEPTION=%s" % (result.driver_exception or ""),
            "ELAPSED_S=%.3f" % result.elapsed_s,
            "LOG=%s" % result.log_path,
        ]
        lines.extend("NOTE " + n for n in self.notes)
        try:
            self.notes_path.write_text("\n".join(lines) + "\n",
                                       encoding="utf-8")
        except OSError as exc:
            print("T3_DRIVER: notes write failed: %s: %s"
                  % (type(exc).__name__, exc), file=sys.stderr)


def run_driver(
    command: Sequence[str],
    log_path: Path,
    notes_path: Path,
    move_az: float,
    move_el: float,
    address: int = 1,
    direction: str = "decrease",
    trigger_samples: int = 3,
    min_step_deg: float = 0.005,
    fresh_s: float = 0.5,
    max_gap_s: float = 0.5,
    init_timeout_s: float = INIT_TIMEOUT_S,
    state_timeout_s: float = STATE_TIMEOUT_S,
    move_ack_timeout_s: float = MOVE_ACK_TIMEOUT_S,
    motion_timeout_s: float = MOTION_TIMEOUT_S,
    stop_wait_s: float = STOP_WAIT_S,
    stop_observe_s: float = STOP_OBSERVE_S,
    exit_timeout_s: float = EXIT_TIMEOUT_S,
) -> T3Result:
    session = _Session(
        command=command, log_path=Path(log_path),
        notes_path=Path(notes_path), move_az=move_az, move_el=move_el,
        address=address, direction=direction, trigger_samples=trigger_samples,
        min_step_deg=min_step_deg, fresh_s=fresh_s, max_gap_s=max_gap_s,
        init_timeout_s=init_timeout_s, state_timeout_s=state_timeout_s,
        move_ack_timeout_s=move_ack_timeout_s,
        motion_timeout_s=motion_timeout_s, stop_wait_s=stop_wait_s,
        stop_observe_s=stop_observe_s, exit_timeout_s=exit_timeout_s)
    return session.run()


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="T3 单次定位＋运动中停止驱动（不连接设备）")
    p.add_argument("--tool", required=True)
    p.add_argument("--config", required=True)
    p.add_argument("--permit", required=True)
    p.add_argument("--move-az", type=float, required=True)
    p.add_argument("--move-el", type=float, required=True)
    p.add_argument("--address", type=int, default=1)
    p.add_argument("--direction", choices=("decrease", "increase"),
                   default="decrease")
    p.add_argument("--trigger-samples", type=int, default=3)
    p.add_argument("--min-step-deg", type=float, default=0.005)
    p.add_argument("--fresh-ms", type=float, default=500.0)
    p.add_argument("--max-gap-ms", type=float, default=500.0)
    p.add_argument("--motion-timeout", type=float, default=MOTION_TIMEOUT_S)
    p.add_argument("--stop-wait", type=float, default=STOP_WAIT_S)
    p.add_argument("--stop-observe", type=float, default=STOP_OBSERVE_S)
    p.add_argument("--exit-timeout", type=float, default=EXIT_TIMEOUT_S)
    p.add_argument("--log", required=True)
    p.add_argument("--notes", required=True)
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    a = _parse_args(argv)
    result = run_driver(
        command=[a.tool, "--config", a.config, "--permit", a.permit],
        log_path=Path(a.log), notes_path=Path(a.notes),
        move_az=a.move_az, move_el=a.move_el, address=a.address,
        direction=a.direction,
        trigger_samples=a.trigger_samples, min_step_deg=a.min_step_deg,
        fresh_s=a.fresh_ms / 1000.0, max_gap_s=a.max_gap_ms / 1000.0,
        motion_timeout_s=a.motion_timeout, stop_wait_s=a.stop_wait,
        stop_observe_s=a.stop_observe, exit_timeout_s=a.exit_timeout)
    print("T3_DRIVER_OK=%d OUTCOME=%s EXIT_CODE=%s STOP_SEND_COUNT=%d "
          "TRIGGER_SAMPLES=%d STOP_DELIVERED=%s"
          % (1 if result.driver_ok else 0, result.outcome,
             result.exit_code, result.stop_send_count,
             result.trigger_samples, result.stop_delivered))
    return 0 if result.driver_ok else 2


if __name__ == "__main__":
    sys.exit(main())
