#!/usr/bin/env python3
"""离线反例：只启动假子进程，不连接真实设备、不发送 UDP。

覆盖用户要求的四类反例：
  1. 迟到到位（等待超时）
  2. 非零退出，拆成两个独立反例：
     a. 真实进程退出码 3，日志 EXIT_CODE 0
     b. 真实进程退出码 0，日志 EXIT_CODE 3
  3. quit 后的尾部输出（必须持续收集到 EOF）
  4. 退出卡住（必须强杀并判失败）
另含两个对照：正常成功、缺少 FINAL 但退出码 0 不能误判成功。

并覆盖独立复核提出的三处驱动缺陷：
  5. 主线程调度停顿后，迟到到位仍须按接收时刻判超时，不得发送 P2；
  6. 读取解码失败不能被当成正常 EOF；
  7. 日志写入异常仍须执行有界停止/退出并回收子进程。
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

TOOLS_DIR = Path(__file__).resolve().parents[1]
# 兼容仓库内 tools/tests/ 布局，也兼容把三个 .py 平铺到同一目录的复核包。
if not (TOOLS_DIR / "t2_acceptance_driver.py").exists():
    TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import t2_acceptance_driver as driver  # noqa: E402

FAKE = Path(__file__).resolve().with_name("fake_t2_tool.py")


class DriverCounterexampleTest(unittest.TestCase):
    def run_mode(self, mode: str, fake_args=(), **kwargs):
        tmp = tempfile.TemporaryDirectory(prefix="t2-driver-test-")
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        log = root / "session.log"
        notes = root / "notes.txt"
        command = [
            sys.executable,
            str(FAKE),
            "--mode",
            mode,
            *fake_args,
        ]
        result = driver.run_driver(
            command=command,
            log_path=log,
            notes_path=notes,
            p1=0.3500,
            p2=0.0500,
            expect_az=(-0.1200, -0.1000),
            **kwargs,
        )
        text = log.read_text(encoding="utf-8")
        notes_text = notes.read_text(encoding="utf-8")
        return result, text, notes_text

    def test_normal_success_control(self):
        result, text, notes = self.run_mode("normal")
        self.assertTrue(result.success, result.reason)
        self.assertEqual(result.exit_code, 0)
        self.assertEqual(result.arrivals, [1, 2])
        self.assertTrue(result.eof_seen)
        self.assertFalse(result.timed_out)
        self.assertFalse(result.forced_termination)
        self.assertIn("EXIT_REQUEST", text)
        self.assertIn("EXIT_STOP", text)
        self.assertIn("FINAL", text)
        self.assertIn("normal_exit=1", text)
        self.assertIn("EXIT_CODE 0", text)
        self.assertIn("CHECK arrival_seq1=1", notes)

    def test_late_arrival_times_out(self):
        result, text, notes = self.run_mode(
            "late_arrival",
            move_timeout_s=0.20,
            stop_timeout_s=1.0,
            exit_timeout_s=2.0,
        )
        self.assertFalse(result.success)
        self.assertTrue(result.timed_out)
        self.assertIn("P1 timeout", result.reason)
        self.assertNotIn("CHECK arrival_seq2=1", notes)
        # 迟到记录可以留在日志里，但不得被当成成功。
        self.assertIn("ARRIVAL_OBSERVED seq=1", text)
        self.assertFalse(result.forced_termination)

    def test_real_exit_nonzero_fails_even_with_log_exit_zero(self):
        # 真实进程退出码 3；日志收尾看起来完全正常（EXIT_CODE 0）。
        # 只有真正检查 returncode 才会失败，不能靠日志退出码兜底。
        result, text, _ = self.run_mode("real_exit_nonzero")
        self.assertFalse(result.success)
        self.assertEqual(result.exit_code, 3)
        self.assertEqual(result.arrivals, [1, 2])
        self.assertIn("FINAL", text)
        self.assertIn("normal_exit=1", text)
        self.assertIn("EXIT_CODE 0", text)
        self.assertIn("exit_code_zero", result.reason)
        self.assertNotIn("exit_code_line_zero", result.reason)
        self.assertTrue(result.checks()["exit_code_line_zero"])

    def test_log_exit_nonzero_fails_even_with_real_exit_zero(self):
        # 真实进程退出码 0；日志 EXIT_CODE 3。只有真正检查收尾行
        # EXIT_CODE 才会失败，不能靠真实退出码兜底。
        result, text, _ = self.run_mode("log_exit_nonzero")
        self.assertFalse(result.success)
        self.assertEqual(result.exit_code, 0)
        self.assertEqual(result.arrivals, [1, 2])
        self.assertIn("FINAL", text)
        self.assertIn("normal_exit=1", text)
        self.assertIn("EXIT_CODE 3", text)
        self.assertIn("exit_code_line_zero", result.reason)
        self.assertNotIn("exit_code_zero", result.reason)
        self.assertTrue(result.checks()["exit_code_zero"])

    def test_tail_output_is_collected_to_eof(self):
        result, text, _ = self.run_mode("tail_output", exit_timeout_s=2.0)
        self.assertTrue(result.success, result.reason)
        self.assertIn("TAIL_AFTER_QUIT", text)
        self.assertIn("EXIT_STOP", text)
        self.assertIn("FINAL", text)
        self.assertIn("EXIT_CODE 0", text)
        self.assertTrue(result.eof_seen)

    def test_exit_hang_is_killed_and_fails(self):
        result, text, _ = self.run_mode("exit_hang", exit_timeout_s=0.30)
        self.assertFalse(result.success)
        self.assertTrue(result.forced_termination)
        self.assertTrue(result.timed_out)
        self.assertIn("EXIT_TIMEOUT", text)
        self.assertEqual(result.arrivals, [1, 2])

    def test_missing_final_is_not_success(self):
        result, text, _ = self.run_mode("missing_final")
        self.assertFalse(result.success)
        self.assertEqual(result.exit_code, 0)
        self.assertIn("EXIT_CODE 0", text)
        self.assertIn("final_normal_exit_1", result.reason)
        self.assertNotIn("FINAL initialized", text)

    def test_late_arrival_after_scheduling_pause_does_not_release_timeout(self):
        # P1 等待 50 ms；假工具 150 ms 才报到位；主线程处理
        # MOVE_ACCEPTED 时停顿 300 ms。迟到行接收时刻必须按 reader
        # 打点判断，不能因为主线程晚消费就解除超时或发送 P2。
        original = driver._Session._handle_line
        state = {"paused": False}

        def paused_handle(session, line, *args, **kwargs):
            if "MOVE_ACCEPTED" in line and not state["paused"]:
                state["paused"] = True
                time.sleep(0.30)
            return original(session, line, *args, **kwargs)

        with mock.patch.object(
            driver._Session, "_handle_line", new=paused_handle
        ):
            result, text, notes = self.run_mode(
                "late_arrival",
                fake_args=("--late-delay", "0.15"),
                move_timeout_s=0.05,
                stop_timeout_s=1.0,
                exit_timeout_s=2.0,
            )
        self.assertFalse(result.success)
        self.assertTrue(result.timed_out)
        self.assertEqual(result.arrivals, [])
        self.assertIn("P1 timeout", result.reason)
        self.assertNotIn('move el 0.0500', text)
        self.assertIn("LATE_ARRIVAL seq=1", notes)

    def test_reader_decode_error_is_not_normal_eof(self):
        # 收尾行完整之后写入非法 UTF-8；读取失败必须单独报告，
        # eof 判据不能因此变为通过。
        result, text, _ = self.run_mode(
            "invalid_tail", exit_timeout_s=2.0
        )
        self.assertFalse(result.success)
        self.assertFalse(result.eof_seen)
        self.assertIsNotNone(result.reader_error)
        self.assertIn("UnicodeDecodeError", result.reader_error)
        self.assertIn("reader failed", result.reason)
        self.assertFalse(result.checks()["reader_ok"])
        self.assertFalse(result.checks()["eof"])
        self.assertIn("FINAL", text)

    def test_log_write_exception_stops_and_reaps_child(self):
        # P1 发出后，在处理 MOVE_ACCEPTED 时注入受控 OSError。
        # 驱动必须仍尝试 stop/quit、回收子进程，并明确判失败。
        original = driver._Session._write
        state = {"raised": False}

        def flaky_write(session, text):
            if "MOVE_ACCEPTED" in text and not state["raised"]:
                state["raised"] = True
                raise OSError("injected log write failure")
            return original(session, text)

        with mock.patch.object(driver._Session, "_write", new=flaky_write):
            result, text, notes = self.run_mode(
                "normal", exit_timeout_s=2.0
            )
        self.assertFalse(result.success)
        self.assertIn("driver exception", result.reason)
        self.assertIn("injected log write failure", result.reason)
        self.assertEqual(result.exit_code, 0)
        self.assertIn('cmd="stop"', text)
        self.assertIn('cmd="quit"', text)
        self.assertNotIn('move el 0.0500', text)
        self.assertIn("DRIVER_EXCEPTION", notes)
        self.assertFalse(result.checks()["no_driver_exception"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
