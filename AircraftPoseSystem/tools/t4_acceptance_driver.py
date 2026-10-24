#!/usr/bin/env python3
"""T4 往返重复定位执行驱动（离线可假工具验证）.

职责边界：

* 只驱动 ``turntable_acceptance`` 的交互 stdin，不实现 Pelco-D 协议；
* 固定两个目标 A/B，按 ``A,B,A,...,A`` 的固定顺序**每条只发一次**，
  等待工具对每条命令给出 ``ARRIVAL_OBSERVED`` 后才发下一条；
* 到位判据只采用工具的 ``ARRIVAL_OBSERVED``（设备反馈稳定，
  ``feedback_only=1``）；驱动校验 seq、目标值、fresh 反馈轮次（cycle）
  不得重复，防止旧样本或错目标被当作本次到位；到位等待与事件处理
  共用同一截止时刻，截止后到达的行不得释放下一条命令；
* 按目标分组统计：反馈偏差（feedback - target）、均值、总体标准差、
  极差、最大绝对偏差；**不把 A/B 混合统计**；
* 同时解析工具退出时的 ``REPEAT`` 行做交叉核对；不一致即判失败；
* 任一步骤失败/超时：不重试定位、不自动回零、不补发运动；只要
  ``move`` 已写入 stdin（即使未收到 ``MOVE_ACCEPTED``，受理状态未知，
  按可能在途处理）且未到位，只发一次 ``stop`` 收尾，再 ``quit``
  收集至 EOF；
* 成功要求工具日志 ``EXIT_CODE 0``、``normal_exit=1`` 且子进程真实
  ``Popen.returncode == 0``，三者独立核对；
* 驱动日志写入失败（``OSError``）立即阻止后续定位命令；已发出的
  运动按“可能在途”进入停止收尾，本次不得报告成功；
* A/B 至少一个轴上的间距必须严格大于两倍到位容差（默认 τ=0.2°，
  即 >0.4°），两个目标的到位区间不得在两条轴上同时重叠；
* 不把设备反馈重复性写成绝对精度、独立物理位置或 1′ 验收。

时间戳为驱动进程读到该行时的 ``time.monotonic()``；工具日志里的
``t=...ms`` 是工具自己的 ``steady_clock`` 主机接收时刻，两者都**不是**
设备内部采样时刻。
"""

from __future__ import annotations

import argparse
import math
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
MOVE_TIMEOUT_S = 20.0
STOP_WAIT_S = 3.0
EXIT_TIMEOUT_S = 10.0
CROSSCHECK_TOL = 5e-4
DEFAULT_ARRIVAL_TOLERANCE_DEG = 0.2

STATE_RE = re.compile(r"\bSTATE\b.*\baz=([-+0-9.]+)\s+el=([-+0-9.]+)")
MOVE_ACCEPTED_RE = re.compile(r"\bMOVE_ACCEPTED\s+accepted=(\d+)\s+seq=(\d+)")
CMD_RESULT_RE = re.compile(
    r"\bCMD_RESULT\s+seq=(\d+)\s+outcome=(\S+)")
ARRIVAL_RE = re.compile(
    r"\bARRIVAL_OBSERVED\s+seq=(\d+)\s+cycle=(\d+)\s+"
    r"az=([-+0-9.]+)\s+el=([-+0-9.]+)\s+"
    r"target=\[([-+0-9.]+),([-+0-9.]+)\].*?"
    r"elapsed_ms=(\d+).*?feedback_only=(\d)")
STOP_REQUEST_RE = re.compile(r"\bSTOP_REQUEST\b.*?\bstop_id=(\d+)")
STOP_RESULT_RE = re.compile(
    r"\bSTOP_RESULT\b.*?\bstop_id=(\d+).*?"
    r"\bstop_delivered=(\d).*?"
    r"\bstop_stationary=(\d).*?\bsend_path=(\S+)")
REPEAT_RE = re.compile(
    r"^\s*REPEAT\s+n=(\d+)\s+target_az=([-+0-9.]+)\s+"
    r"target_el=([-+0-9.]+)\s+az_mean=([-+0-9.]+)\s+"
    r"az_std=([-+0-9.]+)\s+el_mean=([-+0-9.]+)\s+"
    r"el_std=([-+0-9.]+)")
EXIT_CODE_RE = re.compile(r"^\s*EXIT_CODE\s+(-?\d+)\s*$")
NORMAL_EXIT_RE = re.compile(r"\bnormal_exit=(\d+)\b")


def circular_delta_deg(value: float, reference: float) -> float:
    """返回 value - reference 的最短有符号角差，范围 [-180, 180)."""
    return (value - reference + 180.0) % 360.0 - 180.0


@dataclass
class Visit:
    index: int
    target_name: str
    target_az: float
    target_el: float
    seq: int
    cycle: int
    az: float
    el: float
    dev_az: float
    dev_el: float
    elapsed_ms: Optional[int]


@dataclass
class TargetStats:
    name: str
    n: int
    target_az: float
    target_el: float
    az_mean: float
    el_mean: float
    az_bias: float
    el_bias: float
    az_std: float
    el_std: float
    az_range: float
    el_range: float
    az_max_abs_dev: float
    el_max_abs_dev: float


@dataclass
class T4Result:
    driver_ok: bool
    outcome: str
    reason: str
    exit_code: Optional[int]
    process_returncode: Optional[int]
    log_path: str
    notes_path: str
    visits: List[Visit] = field(default_factory=list)
    stats: Dict[str, TargetStats] = field(default_factory=dict)
    repeat_crosscheck: Dict[str, bool] = field(default_factory=dict)
    start_state: Optional[Tuple[float, float]] = None
    init_ok: bool = False
    state_seen: bool = False
    move_accepted_count: int = 0
    stop_requested: bool = False
    stop_send_count: int = 0
    stop_result_seen: bool = False
    stop_delivered: Optional[bool] = None
    stop_stationary: Optional[bool] = None
    stop_send_path: Optional[str] = None
    eof_seen: bool = False
    timed_out: bool = False
    reader_error: Optional[str] = None
    log_error: Optional[str] = None
    driver_exception: Optional[str] = None
    elapsed_s: float = 0.0


@dataclass
class _Pending:
    index: int
    name: str
    target_az: float
    target_el: float
    seq: Optional[int] = None
    sent_at: Optional[float] = None
    accepted_at: Optional[float] = None
    arrival: Optional[Tuple[int, float, float, int, int]] = None
    sent: bool = False
    rejected: bool = False
    cmd_failed: bool = False


class _Session:
    def __init__(
        self,
        command: Sequence[str],
        log_path: Path,
        notes_path: Path,
        plan: Sequence[Tuple[str, float, float]],
        init_timeout_s: float,
        state_timeout_s: float,
        move_ack_timeout_s: float,
        move_timeout_s: float,
        stop_wait_s: float,
        exit_timeout_s: float,
        arrival_tolerance_deg: float = DEFAULT_ARRIVAL_TOLERANCE_DEG,
    ) -> None:
        self.command = list(command)
        self.log_path = Path(log_path)
        self.notes_path = Path(notes_path)
        self.plan = list(plan)
        self.init_timeout_s = init_timeout_s
        self.state_timeout_s = state_timeout_s
        self.move_ack_timeout_s = move_ack_timeout_s
        self.move_timeout_s = move_timeout_s
        self.stop_wait_s = stop_wait_s
        self.exit_timeout_s = exit_timeout_s
        self.arrival_tolerance_deg = arrival_tolerance_deg

        self.proc: Optional[subprocess.Popen] = None
        self.process_returncode: Optional[int] = None
        self.log = None
        self.q: "queue.Queue[Tuple[float, str]]" = queue.Queue()
        self.eof = threading.Event()
        self.reader_done = threading.Event()
        self.reader_error: Optional[str] = None
        self.log_error: Optional[str] = None
        self.arrival_window_open = False
        self.arrival_deadline: Optional[float] = None
        self.driver_exception: Optional[str] = None
        self.reader: Optional[threading.Thread] = None

        self.init_ok = False
        self.state_seen = False
        self.start_state: Optional[Tuple[float, float]] = None
        self.pending: Optional[_Pending] = None
        self.visits: List[Visit] = []
        self.used_cycles: set = set()
        self.move_accepted_count = 0
        self.fatal_reason: Optional[str] = None
        self.permit_rejected = False
        self.motion_error = False

        self.stop_requested = False
        self.stop_sent = False
        self.stop_requested_at: Optional[float] = None
        self.stop_id: Optional[int] = None
        self.stop_result_seen = False
        self.stop_delivered: Optional[bool] = None
        self.stop_stationary: Optional[bool] = None
        self.stop_send_path: Optional[str] = None
        self.stop_send_count = 0

        self.repeat_lines: List[Dict[str, float]] = []
        self.quit_sent = False
        self.exit_code_line: Optional[int] = None
        self.normal_exit = False
        self.final_seen = False
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

    def _handle_line(self, line: str, recv_at: Optional[float] = None,
                     cutoff: Optional[float] = None,
                     apply_state: bool = True) -> None:
        self._write(line)
        if self.log_error is not None:
            return
        if not apply_state:
            tag = ("LATE_EVENT"
                   if (cutoff is not None and recv_at is not None and
                       recv_at > cutoff)
                   else "DRAINED_AFTER_DEADLINE")
            self.notes.append(
                "%s recv_at=%s cutoff=%s line=%r"
                % (tag, recv_at, cutoff, line.strip()))
            return
        if (cutoff is not None and recv_at is not None and
                recv_at > cutoff):
            self.notes.append(
                "LATE_EVENT recv_at=%.6f cutoff=%.6f line=%r"
                % (recv_at, cutoff, line.strip()))
            return

        m = STATE_RE.search(line)
        if m:
            try:
                if self.start_state is None:
                    self.start_state = (float(m.group(1)), float(m.group(2)))
                self.state_seen = True
            except ValueError:
                pass

        if "INIT_OK" in line:
            self.init_ok = True

        if "PERMIT_REJECTED" in line:
            self.permit_rejected = True
            self._note("PERMIT_REJECTED line=%r" % line.strip())

        m = MOVE_ACCEPTED_RE.search(line)
        if m:
            accepted = int(m.group(1)) == 1
            seq = int(m.group(2))
            self.move_accepted_count += 1
            if self.pending is not None:
                if accepted:
                    self.pending.seq = seq
                    self.pending.accepted_at = recv_at or time.monotonic()
                    self._note(
                        "MOVE_ACCEPTED index=%d seq=%d target=%s"
                        % (self.pending.index, seq, self.pending.name))
                else:
                    self.pending.rejected = True
            return

        if "MOVE_REJECTED" in line:
            if self.pending is not None:
                self.pending.rejected = True
            self._note("MOVE_REJECTED line=%r" % line.strip())
            return

        m = ARRIVAL_RE.search(line)
        if m:
            seq = int(m.group(1))
            cycle = int(m.group(2))
            az = float(m.group(3))
            el = float(m.group(4))
            target_az = float(m.group(5))
            target_el = float(m.group(6))
            elapsed_ms = int(m.group(7))
            feedback_only = int(m.group(8)) == 1
            reason = None
            if not self.arrival_window_open:
                reason = "after_arrival_window"
            elif (self.arrival_deadline is None or recv_at is None or
                  recv_at > self.arrival_deadline):
                reason = "received_after_deadline"
            elif self.pending is None:
                reason = "no_pending_move"
            elif self.pending.arrival is not None:
                reason = "duplicate_arrival"
            elif self.pending.seq is None or seq != self.pending.seq:
                reason = ("seq_mismatch expected=%s got=%d"
                          % (self.pending.seq, seq))
            elif (abs(target_az - self.pending.target_az) > 1e-4 or
                  abs(target_el - self.pending.target_el) > 1e-4):
                reason = ("target_mismatch expected=[%.4f,%.4f] "
                          "got=[%.4f,%.4f]"
                          % (self.pending.target_az, self.pending.target_el,
                             target_az, target_el))
            elif cycle in self.used_cycles:
                reason = "cycle_reused cycle=%d" % cycle
            elif not feedback_only:
                reason = "feedback_only=0"
            if reason is not None:
                self._note("ARRIVAL_REJECTED reason=%s" % reason)
                return
            self.used_cycles.add(cycle)
            self.pending.arrival = (cycle, az, el, seq, elapsed_ms)
            return

        m = CMD_RESULT_RE.search(line)
        if m:
            seq = int(m.group(1))
            outcome = m.group(2)
            if (self.pending is not None and
                    self.pending.seq == seq and
                    outcome == "Failed"):
                self.pending.cmd_failed = True
                self._note("CMD_RESULT_FAILED seq=%d" % seq)
            return

        if "MOTION_ERROR" in line:
            self.motion_error = True
            self.fatal_reason = "motion_error"
            self._note("MOTION_ERROR line=%r" % line.strip())
            return

        m = STOP_REQUEST_RE.search(line)
        if m and "STOP_REQUEST" in line:
            self.stop_id = int(m.group(1))
            self.stop_requested = True
            self.stop_requested_at = recv_at or time.monotonic()
            self._note("STOP_REQUEST stop_id=%d" % self.stop_id)
            return

        m = STOP_RESULT_RE.search(line)
        if m:
            stop_id = int(m.group(1))
            reason = None
            if not self.stop_requested:
                reason = "no_stop_request"
            elif (recv_at is None or self.stop_requested_at is None or
                  recv_at < self.stop_requested_at):
                reason = "received_before_stop"
            elif self.stop_id is None:
                reason = "stop_id_unknown"
            elif stop_id != self.stop_id:
                reason = "stop_id_mismatch"
            if reason is not None:
                self._note("STOP_RESULT_REJECTED reason=%s stop_id=%d"
                           % (reason, stop_id))
                return
            self.stop_result_seen = True
            self.stop_delivered = int(m.group(2)) == 1
            self.stop_stationary = int(m.group(3)) == 1
            self.stop_send_path = m.group(4)
            self._note(
                "STOP_RESULT stop_id=%d delivered=%d stationary=%d path=%s"
                % (stop_id, int(self.stop_delivered),
                   int(self.stop_stationary), self.stop_send_path))
            return

        m = REPEAT_RE.search(line)
        if m:
            self.repeat_lines.append({
                "n": int(m.group(1)),
                "target_az": float(m.group(2)),
                "target_el": float(m.group(3)),
                "az_mean": float(m.group(4)),
                "az_std": float(m.group(5)),
                "el_mean": float(m.group(6)),
                "el_std": float(m.group(7)),
            })
            return

        if "FINAL " in line:
            self.final_seen = True
            m = NORMAL_EXIT_RE.search(line)
            if m:
                self.normal_exit = int(m.group(1)) == 1
            return

        m = EXIT_CODE_RE.match(line)
        if m:
            self.exit_code_line = int(m.group(1))

    # ---- 子进程交互 ------------------------------------------------------
    def _send(self, text: str, require_log: bool = False) -> bool:
        if self.proc is None or self.proc.stdin is None:
            return False
        self._write('DRIVER_SEND cmd="%s"\n' % text)
        if require_log and self.log_error is not None:
            self.notes.append(
                "LOG_ERROR_BLOCK_SEND cmd=%r %s"
                % (text, self.log_error))
            return False
        try:
            self.proc.stdin.write(text + "\n")
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError) as exc:
            self._note("SEND_FAILED cmd=%r %s: %s"
                       % (text, type(exc).__name__, exc))
            return False

    def _drain_late(self, cutoff: float) -> None:
        while True:
            try:
                recv_at, line = self.q.get_nowait()
            except queue.Empty:
                return
            self._handle_line(line, recv_at, cutoff=cutoff,
                              apply_state=False)

    def _wait(self, pred: Callable[[], bool], timeout_s: float,
              label: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            if self.log_error is not None:
                self.notes.append(
                    "LOG_ERROR_WAIT label=%s %s" % (label, self.log_error))
                return False
            now = time.monotonic()
            if now >= deadline:
                self._drain_late(deadline)
                self._note("WAIT_TIMEOUT label=%s" % label)
                return False
            if pred():
                return True
            if self.eof.is_set() and self.q.empty():
                self._note("WAIT_EOF label=%s" % label)
                return pred()
            remaining = deadline - now
            try:
                recv_at, line = self.q.get(timeout=min(remaining, 0.05))
            except queue.Empty:
                continue
            processed_at = time.monotonic()
            self._handle_line(
                line, recv_at, cutoff=deadline,
                apply_state=(processed_at <= deadline))
            # 处理也在截止内完成，才允许本事件满足谓词/释放下一条命令。
            if processed_at <= deadline and pred():
                return True

    def _move_remaining(self, timeout_s: float) -> float:
        p = self.pending
        if p is None or p.sent_at is None:
            return timeout_s
        return max(0.0, p.sent_at + timeout_s - time.monotonic())

    def _wait_ack(self, timeout_s: float, label: str) -> bool:
        def pred() -> bool:
            p = self.pending
            return (p is not None and
                    (p.seq is not None or p.rejected or p.cmd_failed or
                     self.permit_rejected or self.motion_error or
                     self.fatal_reason is not None))
        return self._wait(pred, self._move_remaining(timeout_s), label)

    def _wait_arrival(self, timeout_s: float, label: str) -> bool:
        def pred() -> bool:
            p = self.pending
            return (p is not None and
                    (p.arrival is not None or p.rejected or p.cmd_failed or
                     self.permit_rejected or self.motion_error or
                     self.fatal_reason is not None))

        # 截止时刻从 move 写入 stdin 起算，不因主线程暂停、事件排队
        # 或进入等待的早晚而顺延。
        remaining = self._move_remaining(timeout_s)
        p = self.pending
        base = (p.sent_at if p is not None and p.sent_at is not None
                else time.monotonic())
        self.arrival_window_open = True
        self.arrival_deadline = base + timeout_s
        try:
            return self._wait(pred, remaining, label)
        finally:
            self.arrival_window_open = False

    def _pending_failure_reason(self) -> Optional[str]:
        p = self.pending
        if p is None:
            return None
        if self.permit_rejected:
            return "permit_rejected"
        if p.rejected:
            return "move_rejected"
        if p.cmd_failed:
            return "cmd_failed"
        if self.motion_error:
            return "motion_error"
        if self.fatal_reason is not None:
            return self.fatal_reason
        return None

    def _send_stop_once(self) -> bool:
        if self.stop_sent:
            return True
        self.stop_sent = True
        self.stop_send_count += 1
        self._note("T4_STOP_REQUEST reason=closeout")
        return self._send("stop")

    def _wait_stop_result(self) -> None:
        self._wait(lambda: self.stop_result_seen or
                   (self.eof.is_set() and self.q.empty()),
                   self.stop_wait_s, "STOP_RESULT")
        if not self.stop_result_seen:
            self._note("STOP_RESULT_MISSING")

    def _quit_and_wait_exit(self) -> None:
        if not self.quit_sent:
            self.quit_sent = True
            self._send("quit")
        self._wait(lambda: self.exit_code_line is not None and
                   self.eof.is_set(), self.exit_timeout_s, "EXIT")
        if self.proc is not None:
            try:
                self.proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._note("PROC_FORCE_KILL")
                try:
                    self.proc.kill()
                except OSError:
                    pass
                try:
                    self.proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
            self.process_returncode = self.proc.returncode
            if self.reader is not None:
                self.reader.join(timeout=1.0)
            for stream in (self.proc.stdin, self.proc.stdout):
                try:
                    if stream is not None:
                        stream.close()
                except OSError:
                    pass

    # ---- 统计 ------------------------------------------------------------
    @staticmethod
    def _stats_for(visits: Sequence[Visit], name: str,
                   target_az: float, target_el: float) -> TargetStats:
        dev_az = [v.dev_az for v in visits]
        dev_el = [v.dev_el for v in visits]
        n = len(visits)

        def mean(xs: Sequence[float]) -> float:
            return sum(xs) / float(len(xs))

        def pop_std(xs: Sequence[float]) -> float:
            mu = mean(xs)
            return math.sqrt(sum((x - mu) ** 2 for x in xs) / float(len(xs)))

        az_bias = mean(dev_az)
        el_bias = mean(dev_el)
        return TargetStats(
            name=name,
            n=n,
            target_az=target_az,
            target_el=target_el,
            az_mean=mean([v.az for v in visits]),
            el_mean=mean([v.el for v in visits]),
            az_bias=az_bias,
            el_bias=el_bias,
            az_std=pop_std(dev_az),
            el_std=pop_std(dev_el),
            az_range=max(dev_az) - min(dev_az),
            el_range=max(dev_el) - min(dev_el),
            az_max_abs_dev=max(abs(d) for d in dev_az),
            el_max_abs_dev=max(abs(d) for d in dev_el),
        )

    def _compute_stats(self) -> Dict[str, TargetStats]:
        groups: Dict[str, List[Visit]] = {}
        targets: Dict[str, Tuple[float, float]] = {}
        for v in self.visits:
            groups.setdefault(v.target_name, []).append(v)
            targets[v.target_name] = (v.target_az, v.target_el)
        out: Dict[str, TargetStats] = {}
        for name, visits in groups.items():
            taz, tel = targets[name]
            out[name] = self._stats_for(visits, name, taz, tel)
        return out

    def _crosscheck_repeat(self,
                           stats: Dict[str, TargetStats]) -> Dict[str, bool]:
        result: Dict[str, bool] = {}
        for name, st in stats.items():
            if st.n < 2:
                continue
            match = None
            for row in self.repeat_lines:
                if (abs(row["target_az"] - st.target_az) <= 1e-4 and
                        abs(row["target_el"] - st.target_el) <= 1e-4):
                    match = row
                    break
            if match is None:
                result[name] = False
                self._note("REPEAT_CROSSCHECK target=%s ok=0 reason=missing"
                           % name)
                continue
            ok = (
                match["n"] == st.n and
                abs(match["az_mean"] - st.az_mean) <= CROSSCHECK_TOL and
                abs(match["el_mean"] - st.el_mean) <= CROSSCHECK_TOL and
                abs(match["az_std"] - st.az_std) <= CROSSCHECK_TOL and
                abs(match["el_std"] - st.el_std) <= CROSSCHECK_TOL)
            result[name] = ok
            self._note(
                "REPEAT_CROSSCHECK target=%s ok=%d tool_n=%d "
                "tool_az_mean=%.4f tool_az_std=%.4f "
                "tool_el_mean=%.4f tool_el_std=%.4f"
                % (name, int(ok), match["n"], match["az_mean"],
                   match["az_std"], match["el_mean"], match["el_std"]))
        return result

    # ---- 主流程 ----------------------------------------------------------
    def run(self) -> T4Result:
        self.started_at = time.monotonic()
        outcome = "complete"
        reason = ""
        result = T4Result(
            driver_ok=False, outcome="not_started", reason="",
            exit_code=None, process_returncode=None,
            log_path=str(self.log_path),
            notes_path=str(self.notes_path))

        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self.notes_path.parent.mkdir(parents=True, exist_ok=True)
            self.log = self.log_path.open("w", encoding="utf-8")
            self.proc = subprocess.Popen(
                self.command, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
            self.reader = threading.Thread(
                target=self._reader_main, args=(self.proc.stdout,),
                daemon=True)
            self.reader.start()

            init_seen = self._wait(
                lambda: self.init_ok or self.permit_rejected or
                (self.eof.is_set() and self.q.empty()),
                self.init_timeout_s, "INIT")
            if self.log_error is not None:
                outcome, reason = "log_error", self.log_error
            elif not init_seen:
                outcome, reason = "init_timeout", "no INIT_OK"
            elif self.permit_rejected:
                outcome, reason = "permit_rejected", "permit rejected"
            elif not self._wait(lambda: self.state_seen or
                                (self.eof.is_set() and self.q.empty()),
                                self.state_timeout_s, "STATE"):
                if self.log_error is not None:
                    outcome, reason = "log_error", self.log_error
                else:
                    outcome, reason = "state_timeout", "no STATE"

            if outcome == "complete":
                sep = max(
                    abs(circular_delta_deg(self.plan[0][1],
                                           self.plan[1][1])),
                    abs(self.plan[0][2] - self.plan[1][2]))
                self._note(
                    "T4_PLAN pattern=%s moves=%d A=(%.4f,%.4f) "
                    "B=(%.4f,%.4f) separation=%.4f "
                    "arrival_tolerance=%.4f two_tau=%.4f"
                    % ("".join(name for name, _, _ in self.plan),
                       len(self.plan), self.plan[0][1], self.plan[0][2],
                       self.plan[1][1], self.plan[1][2], sep,
                       self.arrival_tolerance_deg,
                       2.0 * self.arrival_tolerance_deg))

            for index, (name, az, el) in enumerate(self.plan, start=1):
                if outcome != "complete":
                    break
                if self.log_error is not None:
                    outcome, reason = "log_error", self.log_error
                    break
                self.pending = _Pending(index, name, az, el)
                self._note(
                    "T4_MOVE index=%d target=%s request=(%.4f,%.4f)"
                    % (index, name, az, el))
                if not self._send("move %.4f %.4f" % (az, el),
                                  require_log=True):
                    if self.log_error is not None:
                        outcome, reason = "log_error", self.log_error
                    else:
                        outcome, reason = "send_failed", "stdin write failed"
                    break
                self.pending.sent = True
                self.pending.sent_at = time.monotonic()
                if not self._wait_ack(self.move_ack_timeout_s,
                                      "MOVE_ACK"):
                    if self.log_error is not None:
                        outcome, reason = "log_error", self.log_error
                    else:
                        outcome, reason = ("move_ack_timeout",
                                           "no MOVE_ACCEPTED")
                    break
                fail = self._pending_failure_reason()
                if fail is not None:
                    outcome, reason = fail, fail
                    break
                if not self._wait_arrival(self.move_timeout_s,
                                          "ARRIVAL"):
                    if self.log_error is not None:
                        outcome, reason = "log_error", self.log_error
                    else:
                        outcome, reason = ("arrival_timeout",
                                           "no ARRIVAL_OBSERVED")
                    break
                fail = self._pending_failure_reason()
                if fail is not None:
                    outcome, reason = fail, fail
                    break
                p = self.pending
                if p is None or p.arrival is None:
                    outcome, reason = "arrival_missing", "no arrival data"
                    break
                cycle, faz, fel, seq, elapsed_ms = p.arrival
                visit = Visit(
                    index=index, target_name=name, target_az=az,
                    target_el=el, seq=seq, cycle=cycle, az=faz, el=fel,
                    dev_az=circular_delta_deg(faz, az), dev_el=fel - el,
                    elapsed_ms=elapsed_ms)
                self.visits.append(visit)
                self._note(
                    "T4_VISIT index=%d target=%s seq=%d cycle=%d "
                    "feedback=(%.4f,%.4f) dev=(%.4f,%.4f)"
                    % (index, name, seq, cycle, faz, fel,
                       visit.dev_az, visit.dev_el))
                self.pending = None

            if outcome != "complete":
                p = self.pending
                if (p is not None and p.sent and not p.rejected and
                        p.arrival is None):
                    self._send_stop_once()
                    self._wait_stop_result()
                self._quit_and_wait_exit()
            else:
                self._quit_and_wait_exit()
                if self.log_error is not None:
                    outcome, reason = "log_error", self.log_error
                elif not self.normal_exit:
                    outcome, reason = "abnormal_exit", "normal_exit!=1"
                elif self.exit_code_line != 0:
                    outcome = "exit_code"
                    reason = "exit_code=%s" % self.exit_code_line
                elif self.process_returncode != 0:
                    outcome = "process_exit_code"
                    reason = ("Popen.returncode=%s log_exit_code=%s"
                              % (self.process_returncode,
                                 self.exit_code_line))
                elif not self.eof.is_set():
                    outcome, reason = "exit_timeout", "no EOF"

            stats = self._compute_stats()
            crosscheck = self._crosscheck_repeat(stats)
            for name, st in stats.items():
                self._note(
                    "T4_STATS target=%s n=%d target=(%.4f,%.4f) "
                    "az_mean=%.4f el_mean=%.4f "
                    "az_bias=%.4f el_bias=%.4f "
                    "az_std=%.4f el_std=%.4f "
                    "az_range=%.4f el_range=%.4f "
                    "az_max_abs_dev=%.4f el_max_abs_dev=%.4f "
                    "device_feedback_only=1"
                    % (name, st.n, st.target_az, st.target_el,
                       st.az_mean, st.el_mean, st.az_bias, st.el_bias,
                       st.az_std, st.el_std, st.az_range, st.el_range,
                       st.az_max_abs_dev, st.el_max_abs_dev))

            expected_names = sorted(set(name for name, _, _ in self.plan))
            cross_ok = all(crosscheck.get(name, False)
                           for name in expected_names)
            if outcome == "complete" and not cross_ok:
                outcome, reason = "repeat_crosscheck", "REPEAT mismatch"
            if outcome == "complete" and len(self.visits) != len(self.plan):
                outcome, reason = "visit_count", "missing visits"

            result = T4Result(
                driver_ok=(outcome == "complete"),
                outcome=outcome, reason=reason,
                exit_code=self.exit_code_line,
                process_returncode=self.process_returncode,
                log_path=str(self.log_path), notes_path=str(self.notes_path),
                visits=list(self.visits), stats=stats,
                repeat_crosscheck=crosscheck,
                start_state=self.start_state, init_ok=self.init_ok,
                state_seen=self.state_seen,
                move_accepted_count=self.move_accepted_count,
                stop_requested=self.stop_requested,
                stop_send_count=self.stop_send_count,
                stop_result_seen=self.stop_result_seen,
                stop_delivered=self.stop_delivered,
                stop_stationary=self.stop_stationary,
                stop_send_path=self.stop_send_path,
                eof_seen=self.eof.is_set(),
                timed_out=("timeout" in outcome),
                reader_error=self.reader_error,
                log_error=self.log_error,
                driver_exception=self.driver_exception)
        except Exception as exc:  # noqa: BLE001 - 记录后按失败收尾
            self.driver_exception = "%s: %s" % (type(exc).__name__, exc)
            outcome, reason = "exception", self.driver_exception
            try:
                self._quit_and_wait_exit()
            except Exception:
                pass
            result = T4Result(
                driver_ok=False, outcome=outcome, reason=reason,
                exit_code=self.exit_code_line,
                process_returncode=self.process_returncode,
                log_path=str(self.log_path), notes_path=str(self.notes_path),
                visits=list(self.visits),
                start_state=self.start_state, init_ok=self.init_ok,
                state_seen=self.state_seen,
                move_accepted_count=self.move_accepted_count,
                stop_requested=self.stop_requested,
                stop_send_count=self.stop_send_count,
                stop_result_seen=self.stop_result_seen,
                stop_delivered=self.stop_delivered,
                stop_stationary=self.stop_stationary,
                stop_send_path=self.stop_send_path,
                eof_seen=self.eof.is_set(),
                reader_error=self.reader_error,
                log_error=self.log_error,
                driver_exception=self.driver_exception)
        finally:
            result.elapsed_s = time.monotonic() - self.started_at
            try:
                if self.log is not None:
                    self.log.close()
            except OSError:
                pass
        return result

    # ---- notes -----------------------------------------------------------
    def write_notes(self, result: T4Result) -> None:
        lines: List[str] = []
        lines.append("# T4 往返重复定位驱动结果")
        lines.append("")
        lines.append("driver_ok=%d outcome=%s reason=%s exit_code=%s "
                     "process_returncode=%s"
                     % (int(result.driver_ok), result.outcome,
                        result.reason or "-", result.exit_code,
                        result.process_returncode))
        lines.append("elapsed_s=%.3f init_ok=%d state_seen=%d "
                     "move_accepted=%d visits=%d"
                     % (result.elapsed_s, int(result.init_ok),
                        int(result.state_seen),
                        result.move_accepted_count, len(result.visits)))
        if result.start_state is not None:
            lines.append("start=(%.4f,%.4f)" % result.start_state)
        lines.append("stop_requested=%d stop_send_count=%d "
                     "stop_result_seen=%d stop_delivered=%s "
                     "stop_stationary=%s stop_send_path=%s"
                     % (int(result.stop_requested),
                        result.stop_send_count,
                        int(result.stop_result_seen),
                        result.stop_delivered, result.stop_stationary,
                        result.stop_send_path))
        lines.append("eof=%d timed_out=%d reader_error=%s log_error=%s "
                     "driver_exception=%s"
                     % (int(result.eof_seen), int(result.timed_out),
                        result.reader_error or "-",
                        result.log_error or "-",
                        result.driver_exception or "-"))
        lines.append("")
        lines.append("## 逐次到位（设备反馈）")
        for v in result.visits:
            lines.append(
                "index=%d target=%s seq=%d cycle=%d "
                "feedback=(%.4f,%.4f) target_xy=(%.4f,%.4f) "
                "dev=(%.4f,%.4f)"
                % (v.index, v.target_name, v.seq, v.cycle, v.az, v.el,
                   v.target_az, v.target_el, v.dev_az, v.dev_el))
        lines.append("")
        lines.append("## 按目标分组统计")
        for name in sorted(result.stats):
            st = result.stats[name]
            lines.append(
                "T4_STATS target=%s n=%d target=(%.4f,%.4f) "
                "az_mean=%.4f el_mean=%.4f "
                "az_bias=%.4f el_bias=%.4f "
                "az_std=%.4f el_std=%.4f "
                "az_range=%.4f el_range=%.4f "
                "az_max_abs_dev=%.4f el_max_abs_dev=%.4f"
                % (name, st.n, st.target_az, st.target_el,
                   st.az_mean, st.el_mean, st.az_bias, st.el_bias,
                   st.az_std, st.el_std, st.az_range, st.el_range,
                   st.az_max_abs_dev, st.el_max_abs_dev))
        lines.append("")
        lines.append("## 工具 REPEAT 交叉核对")
        for name in sorted(result.repeat_crosscheck):
            lines.append("target=%s ok=%d"
                         % (name, int(result.repeat_crosscheck[name])))
        lines.append("")
        lines.append("## 驱动事件")
        lines.extend(self.notes)
        lines.append("")
        lines.append("## 边界")
        lines.append("反馈偏差与重复性只来自设备反馈，不是绝对精度、"
                     "独立物理位置或 1′ 验收；")
        lines.append("未验证设备 ACK、独立实测静止或制动性能。")
        lines.append("")
        self.notes_path.write_text("\n".join(lines), encoding="utf-8")


def build_plan(a_az: float, a_el: float, b_az: float, b_el: float,
               round_trips: int,
               arrival_tolerance_deg: float = DEFAULT_ARRIVAL_TOLERANCE_DEG,
               min_separation_deg: Optional[float] = None
               ) -> List[Tuple[str, float, float]]:
    if round_trips < 2:
        raise ValueError("round_trips must be >= 2 (A and B each >= 2)")
    if not math.isfinite(arrival_tolerance_deg) or arrival_tolerance_deg < 0:
        raise ValueError("arrival_tolerance must be finite and >= 0")
    separation = max(abs(circular_delta_deg(a_az, b_az)), abs(a_el - b_el))
    required = 2.0 * arrival_tolerance_deg
    if min_separation_deg is not None:
        required = max(required, min_separation_deg)
    # 严格“大于”：吸收浮点表示误差，恰好等于 2τ 仍视为区间重叠。
    if separation <= required + 1e-9:
        raise ValueError(
            "A/B separation %.4f deg <= required %.4f deg "
            "(2*arrival_tolerance=%s)"
            % (separation, required, arrival_tolerance_deg))
    plan: List[Tuple[str, float, float]] = [("A", a_az, a_el)]
    for _ in range(round_trips):
        plan.append(("B", b_az, b_el))
        plan.append(("A", a_az, a_el))
    return plan


def run_driver(
    command: Sequence[str],
    log_path: Path,
    notes_path: Path,
    plan: Sequence[Tuple[str, float, float]],
    init_timeout_s: float = INIT_TIMEOUT_S,
    state_timeout_s: float = STATE_TIMEOUT_S,
    move_ack_timeout_s: float = MOVE_ACK_TIMEOUT_S,
    move_timeout_s: float = MOVE_TIMEOUT_S,
    stop_wait_s: float = STOP_WAIT_S,
    exit_timeout_s: float = EXIT_TIMEOUT_S,
    arrival_tolerance_deg: float = DEFAULT_ARRIVAL_TOLERANCE_DEG,
) -> T4Result:
    session = _Session(
        command=command, log_path=log_path, notes_path=notes_path,
        plan=plan, init_timeout_s=init_timeout_s,
        state_timeout_s=state_timeout_s,
        move_ack_timeout_s=move_ack_timeout_s,
        move_timeout_s=move_timeout_s, stop_wait_s=stop_wait_s,
        exit_timeout_s=exit_timeout_s,
        arrival_tolerance_deg=arrival_tolerance_deg)
    result = session.run()
    session.write_notes(result)
    return result


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="T4 往返重复定位执行驱动（固定 A/B，逐条到位统计）")
    parser.add_argument("--tool", required=True, help="turntable_acceptance 路径")
    parser.add_argument("--config", required=True, help="配置目录")
    parser.add_argument("--permit", required=True, help="本次验收许可文件")
    parser.add_argument("--a-az", type=float, required=True, help="A 方位")
    parser.add_argument("--a-el", type=float, required=True, help="A 俯仰")
    parser.add_argument("--b-az", type=float, required=True, help="B 方位")
    parser.add_argument("--b-el", type=float, required=True, help="B 俯仰")
    parser.add_argument("--round-trips", type=int, default=2,
                        help="A→B→A 往返次数；2 表示 A,B,A,B,A（默认 2）")
    parser.add_argument("--arrival-tolerance", type=float,
                        default=DEFAULT_ARRIVAL_TOLERANCE_DEG,
                        help="单目标到位容差 τ（默认 0.2°）；A/B 间距须 >2τ")
    parser.add_argument("--min-separation", type=float, default=None,
                        help="额外最小间距下限（默认不设；不得低于 2τ）")
    parser.add_argument("--expect-az-min", type=float, default=None)
    parser.add_argument("--expect-az-max", type=float, default=None)
    parser.add_argument("--expect-el-min", type=float, default=None)
    parser.add_argument("--expect-el-max", type=float, default=None)
    parser.add_argument("--init-timeout", type=float, default=INIT_TIMEOUT_S)
    parser.add_argument("--state-timeout", type=float, default=STATE_TIMEOUT_S)
    parser.add_argument("--ack-timeout", type=float,
                        default=MOVE_ACK_TIMEOUT_S)
    parser.add_argument("--move-timeout", type=float, default=MOVE_TIMEOUT_S)
    parser.add_argument("--stop-timeout", type=float, default=STOP_WAIT_S)
    parser.add_argument("--exit-timeout", type=float, default=EXIT_TIMEOUT_S)
    parser.add_argument("--log", required=True, help="完整 stdout 日志")
    parser.add_argument("--notes", required=True, help="统计与逐项判据")
    return parser.parse_args(argv)


def _within(value: float, lo: Optional[float], hi: Optional[float]) -> bool:
    if lo is not None and value < lo:
        return False
    if hi is not None and value > hi:
        return False
    return True


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    for name, value in (("a-az", args.a_az), ("a-el", args.a_el),
                        ("b-az", args.b_az), ("b-el", args.b_el)):
        if not math.isfinite(value):
            print("T4_DRIVER_RESULT ok=0 outcome=invalid_target reason=%s"
                  % name)
            return 2
    if (abs(args.a_az - args.b_az) <= 1e-9 and
            abs(args.a_el - args.b_el) <= 1e-9):
        print("T4_DRIVER_RESULT ok=0 outcome=invalid_plan "
              "reason=a_and_b_are_identical")
        return 2
    if not (_within(args.a_az, args.expect_az_min, args.expect_az_max) and
            _within(args.b_az, args.expect_az_min, args.expect_az_max) and
            _within(args.a_el, args.expect_el_min, args.expect_el_max) and
            _within(args.b_el, args.expect_el_min, args.expect_el_max)):
        print("T4_DRIVER_RESULT ok=0 outcome=target_outside_envelope "
              "reason=target_not_within_expected_bounds")
        return 2
    try:
        plan = build_plan(args.a_az, args.a_el, args.b_az, args.b_el,
                          args.round_trips, args.arrival_tolerance,
                          args.min_separation)
    except ValueError as exc:
        print("T4_DRIVER_RESULT ok=0 outcome=invalid_plan reason=%s" % exc)
        return 2

    command = [args.tool, "--config", args.config,
               "--permit", args.permit]
    result = run_driver(
        command=command, log_path=Path(args.log),
        notes_path=Path(args.notes), plan=plan,
        init_timeout_s=args.init_timeout,
        state_timeout_s=args.state_timeout,
        move_ack_timeout_s=args.ack_timeout,
        move_timeout_s=args.move_timeout,
        stop_wait_s=args.stop_timeout,
        exit_timeout_s=args.exit_timeout,
        arrival_tolerance_deg=args.arrival_tolerance)
    counts = " ".join("%s_n=%d" % (name, st.n)
                      for name, st in sorted(result.stats.items()))
    print("T4_DRIVER_RESULT ok=%d outcome=%s exit_code=%s "
          "process_returncode=%s visits=%d %s "
          "stop_send_count=%d crosscheck=%d"
          % (int(result.driver_ok), result.outcome, result.exit_code,
             result.process_returncode, len(result.visits), counts,
             result.stop_send_count,
             int(all(result.repeat_crosscheck.values())
                 if result.repeat_crosscheck else 0)))
    return 0 if result.driver_ok else 1


if __name__ == "__main__":
    sys.exit(main())
