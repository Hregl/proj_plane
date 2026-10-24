#!/usr/bin/env python3
"""T2 两步定位执行驱动（超时可读、单调时钟、收尾证据完整）.

本驱动只负责“逐条发送、等待明确到位、正常收尾”，不包含任何设备
协议实现，也不改变生产代码或验收门禁。它替换执行卡中的临时驱动：

* 读取 stdout 由独立 reader 线程完成，主线程通过 queue 做**可超时读取**；
* 所有截止时间使用 ``time.monotonic()``；
* 发送 ``quit`` 后持续收集输出直到 EOF，不提前关闭日志；
* 成功必须同时满足：P1/P2 均出现 ``ARRIVAL_OBSERVED``、工具退出码 0、
  ``EXIT_REQUEST``/``EXIT_STOP``/``FINAL normal_exit=1``/``EXIT_CODE 0``
  齐全、观察到 EOF；
* 等待到位超时、退出超时或强制终止一律明确判失败，不以“已发送”代替
  “已到位”。

本文件不连接真实设备；实机执行仍需单独书面授权。
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

SEQ_RE = re.compile(r"\bseq=(\d+)\b")
STATE_RE = re.compile(r"\bSTATE\b.*\baz=([-+0-9.]+)\s+el=([-+0-9.]+)")
NORMAL_EXIT_RE = re.compile(r"\bnormal_exit=(\d+)\b")
EXIT_CODE_RE = re.compile(r"^\s*EXIT_CODE\s+(-?\d+)\s*$")


@dataclass
class RunResult:
    """一次驱动会话的可复核结果。"""

    success: bool
    reason: str
    exit_code: Optional[int]
    arrivals: List[int] = field(default_factory=list)
    init_ok: bool = False
    state_seen: bool = False
    terminal_reason: Optional[str] = None
    final_seen: bool = False
    normal_exit: bool = False
    exit_code_line: Optional[int] = None
    exit_request_seen: bool = False
    exit_stop_seen: bool = False
    eof_seen: bool = False
    timed_out: bool = False
    forced_termination: bool = False
    reader_error: Optional[str] = None
    log_error: Optional[str] = None
    driver_exception: Optional[str] = None
    elapsed_s: float = 0.0
    log_path: str = ""
    notes_path: str = ""

    def checks(self) -> Dict[str, bool]:
        """返回成功判据的逐项结果，供 notes/复核直接引用。"""
        return {
            "arrival_seq1": 1 in self.arrivals,
            "arrival_seq2": 2 in self.arrivals,
            "exit_code_zero": self.exit_code == 0,
            "exit_request": self.exit_request_seen,
            "exit_stop": self.exit_stop_seen,
            "final_normal_exit_1": self.final_seen and self.normal_exit,
            "exit_code_line_zero": self.exit_code_line == 0,
            "eof": self.eof_seen,
            "reader_ok": self.reader_error is None,
            "log_ok": self.log_error is None,
            "no_driver_exception": self.driver_exception is None,
            "not_timed_out": not self.timed_out,
            "not_forced": not self.forced_termination,
            "no_tool_terminal_error": self.terminal_reason is None,
        }


class _Session:
    def __init__(
        self,
        command: Sequence[str],
        log_path: Path,
        notes_path: Path,
        move_timeout_s: float,
        init_timeout_s: float,
        state_timeout_s: float,
        stop_timeout_s: float,
        exit_timeout_s: float,
        expect_az: Optional[Tuple[float, float]],
    ) -> None:
        self.command = list(command)
        self.log_path = Path(log_path)
        self.notes_path = Path(notes_path)
        self.move_timeout_s = move_timeout_s
        self.init_timeout_s = init_timeout_s
        self.state_timeout_s = state_timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.exit_timeout_s = exit_timeout_s
        self.expect_az = expect_az

        self.proc: Optional[subprocess.Popen] = None
        self.log = None
        # 每个元素是 (单调接收时刻, 原始行)。时间戳由 reader 线程在读到
        # 行之后立即打点，主线程即使被调度停顿，也不会把迟到行当准时行。
        self.q: "queue.Queue[Tuple[float, str]]" = queue.Queue()
        self.eof = threading.Event()
        self.reader_done = threading.Event()
        self.reader_error: Optional[str] = None
        self.log_error: Optional[str] = None
        self.driver_exception: Optional[str] = None
        self.reader: Optional[threading.Thread] = None

        self.init_ok = False
        self.state_seen = False
        self.last_state: Optional[Tuple[float, float]] = None
        self.arrivals: List[int] = []
        self.late_arrivals: List[int] = []
        self.arrival_deadlines: Dict[int, float] = {}
        self.deferred: List[Tuple[float, str]] = []
        self.terminal_reason: Optional[str] = None
        self.final_seen = False
        self.normal_exit = False
        self.exit_code_line: Optional[int] = None
        self.exit_request_seen = False
        self.exit_stop_seen = False
        self.stop_result_seen = False
        self.timed_out = False
        self.forced_termination = False
        self.notes: List[str] = []
        self.started_at = 0.0

    # ---- 日志与输出收集 -------------------------------------------------
    def _write(self, text: str) -> None:
        # 日志写入失败不能让收尾路径抛异常；记录错误，由主流程判失败。
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
        except Exception as exc:  # 收尾阶段不允许日志问题再次打断清理
            self.notes.append("NOTE_FAILED %s: %s" % (type(exc).__name__, exc))

    def _reader_main(self, stream) -> None:
        # 正常 EOF 才置 eof；读取失败单独记录，不能被当成完整输出。
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
        self,
        line: str,
        recv_at: Optional[float] = None,
        cutoff: Optional[float] = None,
    ) -> None:
        self._write(line)
        if self.log_error is not None:
            return
        if cutoff is not None and recv_at is not None and recv_at > cutoff:
            # 迟到行保留并记入日志，但不能满足本次等待或触发下一步。
            self.deferred.append((recv_at, line))
            self._note(
                "LATE_EVENT late_by=%.3fs line=%r"
                % (recv_at - cutoff, line.strip())
            )
            return
        self._apply_line(line, recv_at)

    def _apply_line(self, line: str, recv_at: Optional[float] = None) -> None:
        if "INIT_OK" in line:
            self.init_ok = True
            return

        m = STATE_RE.search(line)
        if m:
            try:
                self.last_state = (float(m.group(1)), float(m.group(2)))
                self.state_seen = True
            except ValueError:
                pass
            return

        if "ARRIVAL_OBSERVED" in line:
            m = SEQ_RE.search(line)
            if m:
                seq = int(m.group(1))
                deadline = self.arrival_deadlines.get(seq)
                if (
                    deadline is not None
                    and recv_at is not None
                    and recv_at > deadline
                ):
                    if seq not in self.late_arrivals:
                        self.late_arrivals.append(seq)
                        self.timed_out = True
                        self._note(
                            "LATE_ARRIVAL seq=%d late_by=%.3fs"
                            % (seq, recv_at - deadline)
                        )
                    return
                if seq not in self.arrivals:
                    self.arrivals.append(seq)
                    self._note("ARRIVAL seq=%d" % seq)
            return

        # 这些是工具侧明确终结或拒绝，不允许继续发送下一条定位。
        # Delivered 只表示已交付，不等于到位；其余 outcome 都按终结处理。
        if "CMD_RESULT" in line and "outcome=Delivered" not in line:
            self._set_terminal("CMD_RESULT non-delivered: " + line.strip())
            return
        if "MOVE_NOT_ACCEPTED" in line:
            self._set_terminal("MOVE_NOT_ACCEPTED: " + line.strip())
            return
        if "MOVE_REJECTED" in line:
            self._set_terminal("MOVE_REJECTED: " + line.strip())
            return
        if "MOTION_ERROR" in line:
            self._set_terminal("MOTION_ERROR: " + line.strip())
            return
        if "CONFIG_FAILED" in line or "SELF_CHECK_FAILED" in line:
            self._set_terminal("tool self-check/config failed: " + line.strip())
            return

        if "STOP_RESULT" in line:
            self.stop_result_seen = True
            return
        if "EXIT_REQUEST" in line:
            self.exit_request_seen = True
            return
        if "EXIT_STOP" in line:
            self.exit_stop_seen = True
            return
        if "FINAL" in line:
            self.final_seen = True
            m = NORMAL_EXIT_RE.search(line)
            if m:
                self.normal_exit = int(m.group(1)) == 1
            return
        m = EXIT_CODE_RE.match(line)
        if m:
            try:
                self.exit_code_line = int(m.group(1))
            except ValueError:
                pass

    def _set_terminal(self, reason: str) -> None:
        if self.terminal_reason is None:
            self.terminal_reason = reason
            self._note("TOOL_TERMINAL " + reason)

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

    def _drain_deferred(self) -> None:
        while self.deferred:
            recv_at, line = self.deferred.pop(0)
            self._apply_line(line, recv_at)

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        timeout_s: float,
        deadline: Optional[float] = None,
    ) -> bool:
        if deadline is None:
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
                # 截止前消费已到达的行；接收时刻晚于截止时间的行只留档，
                # 不参与判据，避免调度停顿后把迟到到位当成功。
                self._drain_available(deadline)
                return (
                    predicate()
                    and self.reader_error is None
                    and self.log_error is None
                )
            if (
                self.proc is not None
                and self.proc.poll() is not None
                and self.reader_done.is_set()
                and self.q.empty()
            ):
                self._drain_available(deadline)
                return (
                    predicate()
                    and self.reader_error is None
                    and self.log_error is None
                )
            self._pump_once(min(0.05, remaining), deadline)

    def _wait_arrival(self, seq: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        self.arrival_deadlines[seq] = deadline
        return self._wait_for(
            lambda: seq in self.arrivals
            or self.terminal_reason is not None,
            timeout_s,
            deadline,
        )

    def _fault_reason(self) -> Optional[str]:
        if self.driver_exception is not None:
            return "driver exception: " + self.driver_exception
        if self.log_error is not None:
            return "log write failed: " + self.log_error
        if self.reader_error is not None:
            return "reader failed: " + self.reader_error
        return None

    def _timeout_reason(self, label: str, timeout_s: float) -> str:
        fault = self._fault_reason()
        if fault is not None:
            return fault
        self.timed_out = True
        return "%s timeout after %.2fs" % (label, timeout_s)

    # ---- 发送与收尾 -----------------------------------------------------
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

    def _close_stdin(self) -> None:
        if self.proc is None or self.proc.stdin is None:
            return
        try:
            self.proc.stdin.close()
        except OSError:
            pass

    def _wait_exit(self, timeout_s: float) -> bool:
        """等待进程退出，同时持续消费输出；返回是否在截止前正常收束。"""
        deadline = time.monotonic() + timeout_s
        while True:
            self._drain_available()
            if (
                self.proc is not None
                and self.proc.poll() is not None
                and self.reader_done.is_set()
                and self.q.empty()
            ):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._drain_available()
                return (
                    self.proc is not None
                    and self.proc.poll() is not None
                    and self.reader_done.is_set()
                    and self.q.empty()
                )
            time.sleep(min(0.02, remaining))

    def _stop_then_quit(self) -> None:
        """失败路径：先 stop 并收集其输出，再 quit；不在这里强杀。"""
        if self.proc is None or self.proc.poll() is not None:
            return
        self.send("stop")
        self._wait_for(
            lambda: self.stop_result_seen or self.terminal_reason is not None,
            self.stop_timeout_s,
        )
        if self.proc.poll() is None:
            self.send("quit")
        self._close_stdin()

    # ---- 主流程 ---------------------------------------------------------
    def _execute_steps(self, p1: float, p2: float) -> str:
        """依次完成初始化、P1、P2、quit；返回成功 "ok" 或失败原因。"""
        if not self._wait_for(lambda: self.init_ok, self.init_timeout_s):
            return self._timeout_reason("INIT_OK", self.init_timeout_s)
        fault = self._fault_reason()
        if fault is not None:
            return fault

        if not self._wait_for(lambda: self.state_seen, self.state_timeout_s):
            return self._timeout_reason("STATE", self.state_timeout_s)
        fault = self._fault_reason()
        if fault is not None:
            return fault

        if (
            self.expect_az is not None
            and self.last_state is not None
            and not (
                self.expect_az[0] <= self.last_state[0] <= self.expect_az[1]
            )
        ):
            return "preflight az=%.4f outside [%.4f, %.4f]" % (
                self.last_state[0],
                self.expect_az[0],
                self.expect_az[1],
            )
        if self.last_state is not None:
            self._note(
                "PREFLIGHT az=%.4f el=%.4f"
                % (self.last_state[0], self.last_state[1])
            )
        fault = self._fault_reason()
        if fault is not None:
            return fault

        # P1：发送后必须等到 seq=1 的明确到位或工具侧失败。
        if not self.send("move el %.4f" % p1):
            return "P1 send failed"
        if not self._wait_arrival(1, self.move_timeout_s):
            return self._timeout_reason("P1", self.move_timeout_s)
        fault = self._fault_reason()
        if fault is not None:
            return fault
        if self.terminal_reason is not None:
            return "P1 tool terminal: " + self.terminal_reason
        self._note("P1 ARRIVED")
        fault = self._fault_reason()
        if fault is not None:
            return fault

        # P2：序号必须递增，不能只看“某次到位”。
        if not self.send("move el %.4f" % p2):
            return "P2 send failed"
        if not self._wait_arrival(2, self.move_timeout_s):
            return self._timeout_reason("P2", self.move_timeout_s)
        fault = self._fault_reason()
        if fault is not None:
            return fault
        if self.terminal_reason is not None:
            return "P2 tool terminal: " + self.terminal_reason
        self._note("P2 ARRIVED")
        fault = self._fault_reason()
        if fault is not None:
            return fault

        # 两步都到位后才正常退出。
        if not self.send("quit"):
            return "quit send failed"
        self._close_stdin()
        return "ok"

    def run(self, p1: float, p2: float) -> RunResult:
        self.started_at = time.monotonic()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.notes_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("w", encoding="utf-8", newline="\n")

        reason = "ok"
        fatal: Optional[BaseException] = None
        try:
            try:
                self.proc = subprocess.Popen(
                    self.command,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="strict",
                    bufsize=1,
                )
                assert self.proc.stdout is not None
                self.reader = threading.Thread(
                    target=self._reader_main,
                    args=(self.proc.stdout,),
                    name="t2-driver-reader",
                    daemon=True,
                )
                self.reader.start()
                reason = self._execute_steps(p1, p2)
            except BaseException as exc:
                # 驱动自身异常（含受控的日志写入失败）也必须走有界收尾，
                # 不能让子进程留在运行状态。
                self.driver_exception = "%s: %s" % (type(exc).__name__, exc)
                reason = "driver exception: " + self.driver_exception
                self._safe_note("DRIVER_EXCEPTION " + self.driver_exception)
                if not isinstance(exc, Exception):
                    fatal = exc

            # 失败路径：先 stop 再 quit；成功路径已经发过 quit。
            if (
                reason != "ok"
                and self.proc is not None
                and self.proc.poll() is None
            ):
                try:
                    self._stop_then_quit()
                except Exception as exc:
                    cleanup_error = "%s: %s" % (type(exc).__name__, exc)
                    if self.driver_exception is None:
                        self.driver_exception = cleanup_error
                    self._safe_note("STOP_QUIT_FAILED " + cleanup_error)

            # 无论成功失败，都收集到 EOF；退出超时明确判失败并强杀。
            exit_ok = False
            if self.proc is not None:
                exit_ok = self._wait_exit(self.exit_timeout_s)
            if not exit_ok and self.proc is not None:
                if self.proc.poll() is None:
                    self.forced_termination = True
                    self._note(
                        "EXIT_TIMEOUT after %.2fs; killing process"
                        % self.exit_timeout_s
                    )
                    try:
                        self.proc.kill()
                    except OSError:
                        pass
                    try:
                        self.proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        self._note("KILL_WAIT_TIMEOUT")
                # 强杀后给 reader 最多 1 s 收 EOF。
                eof_deadline = time.monotonic() + 1.0
                while (
                    not self.reader_done.is_set()
                    and time.monotonic() < eof_deadline
                ):
                    self._drain_available()
                    time.sleep(0.01)
                self._drain_available()
                if reason == "ok":
                    reason = "exit timeout after %.2fs" % self.exit_timeout_s
                self.timed_out = True

            # 读取剩余队列与迟到留档，确保日志与 EOF 判定一致。
            self._drain_available()
            if self.reader is not None:
                self.reader.join(timeout=1.0)
            self._drain_available()
            self._drain_deferred()

            fault = self._fault_reason()
            if reason == "ok" and fault is not None:
                reason = fault

            exit_code = (
                self.proc.returncode if self.proc is not None else None
            )
            checks = self._checks_dict(
                exit_code,
                eof_seen=self.eof.is_set(),
            )
            failed = [name for name, ok in checks.items() if not ok]
            if reason == "ok" and failed:
                reason = "success criteria failed: " + ", ".join(failed)
            success = reason == "ok"

            result = RunResult(
                success=success,
                reason=reason,
                exit_code=exit_code,
                arrivals=sorted(set(self.arrivals)),
                init_ok=self.init_ok,
                state_seen=self.state_seen,
                terminal_reason=self.terminal_reason,
                final_seen=self.final_seen,
                normal_exit=self.normal_exit,
                exit_code_line=self.exit_code_line,
                exit_request_seen=self.exit_request_seen,
                exit_stop_seen=self.exit_stop_seen,
                eof_seen=self.eof.is_set(),
                timed_out=self.timed_out,
                forced_termination=self.forced_termination,
                reader_error=self.reader_error,
                log_error=self.log_error,
                driver_exception=self.driver_exception,
                elapsed_s=time.monotonic() - self.started_at,
                log_path=str(self.log_path),
                notes_path=str(self.notes_path),
            )
            self._write_result(result)
            self._write_notes(result)
            if fatal is not None:
                raise fatal
            return result
        finally:
            # 最后一道兜底：即使收尾代码自身异常，也不留下运行中的子进程。
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
            if (
                self.proc is not None
                and self.proc.stdout is not None
                and self.reader_done.is_set()
            ):
                try:
                    self.proc.stdout.close()
                except OSError:
                    pass

    # ---- 判据与落盘 -----------------------------------------------------
    def _checks_dict(
        self, exit_code: Optional[int], eof_seen: bool
    ) -> Dict[str, bool]:
        return {
            "arrival_seq1": 1 in self.arrivals,
            "arrival_seq2": 2 in self.arrivals,
            "exit_code_zero": exit_code == 0,
            "exit_request": self.exit_request_seen,
            "exit_stop": self.exit_stop_seen,
            "final_normal_exit_1": self.final_seen and self.normal_exit,
            "exit_code_line_zero": self.exit_code_line == 0,
            "eof": eof_seen,
            "reader_ok": self.reader_error is None,
            "log_ok": self.log_error is None,
            "no_driver_exception": self.driver_exception is None,
            "not_timed_out": not self.timed_out,
            "not_forced": not self.forced_termination,
            "no_tool_terminal_error": self.terminal_reason is None,
        }

    def _write_result(self, result: RunResult) -> None:
        self._write(
            "DRIVER_RESULT success=%d reason=%r exit_code=%s "
            "arrivals=%s eof=%d timed_out=%d forced=%d\n"
            % (
                1 if result.success else 0,
                result.reason,
                result.exit_code,
                result.arrivals,
                1 if result.eof_seen else 0,
                1 if result.timed_out else 0,
                1 if result.forced_termination else 0,
            )
        )

    def _write_notes(self, result: RunResult) -> None:
        lines = [
            "RESULT=%s" % ("PASS" if result.success else "FAIL"),
            "REASON=%s" % result.reason,
            "EXIT_CODE=%s" % result.exit_code,
            "ARRIVALS=%s" % ",".join(str(x) for x in result.arrivals),
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
        for name, ok in result.checks().items():
            lines.append("CHECK %s=%d" % (name, 1 if ok else 0))
        for note in self.notes:
            lines.append("NOTE " + note)
        try:
            self.notes_path.write_text(
                "\n".join(lines) + "\n", encoding="utf-8"
            )
        except OSError as exc:
            # 结果仍返回给调用方；notes 写失败只报告到 stderr。
            print(
                "DRIVER: notes write failed: %s: %s"
                % (type(exc).__name__, exc),
                file=sys.stderr,
            )


def run_driver(
    command: Sequence[str],
    log_path: Path,
    notes_path: Path,
    p1: float,
    p2: float,
    expect_az: Optional[Tuple[float, float]] = None,
    move_timeout_s: float = 15.0,
    init_timeout_s: float = 10.0,
    state_timeout_s: float = 5.0,
    stop_timeout_s: float = 2.0,
    exit_timeout_s: float = 10.0,
) -> RunResult:
    """运行一次两步定位会话；所有时间参数单位为秒。"""
    session = _Session(
        command=command,
        log_path=Path(log_path),
        notes_path=Path(notes_path),
        move_timeout_s=move_timeout_s,
        init_timeout_s=init_timeout_s,
        state_timeout_s=state_timeout_s,
        stop_timeout_s=stop_timeout_s,
        exit_timeout_s=exit_timeout_s,
        expect_az=expect_az,
    )
    return session.run(p1=p1, p2=p2)


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="T2 两步定位执行驱动（不连接设备）"
    )
    parser.add_argument("--tool", required=True, help="turntable_acceptance 路径")
    parser.add_argument("--config", required=True, help="配置目录")
    parser.add_argument("--permit", required=True, help="本次验收许可文件")
    parser.add_argument("--p1", type=float, required=True, help="第一步俯仰目标")
    parser.add_argument("--p2", type=float, required=True, help="第二步俯仰目标")
    parser.add_argument("--expect-az-min", type=float, default=None)
    parser.add_argument("--expect-az-max", type=float, default=None)
    parser.add_argument("--log", required=True, help="完整 stdout 日志")
    parser.add_argument("--notes", required=True, help="驱动结论与逐项判据")
    parser.add_argument("--move-timeout", type=float, default=15.0)
    parser.add_argument("--init-timeout", type=float, default=10.0)
    parser.add_argument("--state-timeout", type=float, default=5.0)
    parser.add_argument("--stop-timeout", type=float, default=2.0)
    parser.add_argument(
        "--exit-timeout",
        type=float,
        default=10.0,
        help="等待工具退出的上限；实机须覆盖 stop_timeout_ms+观测预算，0.3 仅测试",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    expect_az = None
    if (args.expect_az_min is None) != (args.expect_az_max is None):
        print("--expect-az-min 与 --expect-az-max 必须同时给出", file=sys.stderr)
        return 2
    if args.expect_az_min is not None:
        expect_az = (args.expect_az_min, args.expect_az_max)

    command = [
        args.tool,
        "--config",
        args.config,
        "--permit",
        args.permit,
    ]
    result = run_driver(
        command=command,
        log_path=Path(args.log),
        notes_path=Path(args.notes),
        p1=args.p1,
        p2=args.p2,
        expect_az=expect_az,
        move_timeout_s=args.move_timeout,
        init_timeout_s=args.init_timeout,
        state_timeout_s=args.state_timeout,
        stop_timeout_s=args.stop_timeout,
        exit_timeout_s=args.exit_timeout,
    )
    print(
        "RESULT: %s reason=%s exit_code=%s arrivals=%s eof=%d"
        % (
            "PASS" if result.success else "FAIL",
            result.reason,
            result.exit_code,
            result.arrivals,
            1 if result.eof_seen else 0,
        )
    )
    for name, ok in result.checks().items():
        print("CHECK %s=%d" % (name, 1 if ok else 0))
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
