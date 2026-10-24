#!/usr/bin/env python3
"""离线验证 T4 驱动：只启动假子进程，不连接设备、不发 UDP。

覆盖：
  1. A,B,A,B,A 五条逐次到位，按 A/B 分组统计偏差与重复性；
  2. 末尾停在 A，不自动回零、不追加第六条运动；
  3. 旧 seq 的到位行被拒绝，不能当作本次到位；
  4. 未到位超时：只发一次 stop 收尾，然后 quit；
  5. 命令失败：只发一次 stop 收尾；
  6. 命令被拒：无运动，不发 stop；
  7. 重复 cycle：不得把同一轮样本当作两次到位；
  8. 目标不符的到位行被拒绝；
  9. 工具 REPEAT 与驱动统计不一致时判失败；
  10. 工具退出码非 0 判失败；
  11. 目标超出预期包络：不启动工具；
  12. A/B 相同：不启动工具（避免混合统计）。
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

TOOLS_DIR = Path(__file__).resolve().parents[1]
if not (TOOLS_DIR / "t4_acceptance_driver.py").exists():
    TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import t4_acceptance_driver as driver  # noqa: E402

FAKE = Path(__file__).resolve().with_name("fake_t4_tool.py")
PLAN = driver.build_plan(-0.6200, 0.0500, -0.1200, 0.0500, 2)


class T4DriverTest(unittest.TestCase):
    def run_mode(self, mode: str, env=None, plan=PLAN, **kwargs):
        tmp = tempfile.TemporaryDirectory(prefix="t4-driver-test-")
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        log = root / "session.log"
        notes = root / "notes.txt"
        command = [sys.executable, str(FAKE), "--mode", mode]
        params = dict(
            init_timeout_s=2.0,
            state_timeout_s=2.0,
            move_ack_timeout_s=2.0,
            move_timeout_s=0.5,
            stop_wait_s=1.0,
            exit_timeout_s=3.0,
        )
        params.update(kwargs)
        with mock.patch.dict(os.environ, env or {}, clear=False):
            result = driver.run_driver(
                command=command, log_path=log, notes_path=notes,
                plan=plan, **params)
        text = log.read_text(encoding="utf-8")
        note_text = notes.read_text(encoding="utf-8")
        return result, text, note_text

    def test_five_moves_ababa_and_per_target_stats(self):
        result, text, notes = self.run_mode("normal")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "complete")
        self.assertEqual(len(result.visits), 5)
        self.assertEqual([v.target_name for v in result.visits],
                         ["A", "B", "A", "B", "A"])
        self.assertEqual(result.stats["A"].n, 3)
        self.assertEqual(result.stats["B"].n, 2)
        self.assertAlmostEqual(result.stats["A"].az_bias, 0.000667, places=4)
        self.assertAlmostEqual(result.stats["B"].az_bias, -0.0005, places=4)
        self.assertGreater(result.stats["A"].az_std, 0.0)
        self.assertLess(result.stats["A"].az_std, 0.002)
        self.assertGreaterEqual(result.stats["A"].az_range, 0.003)
        self.assertTrue(result.repeat_crosscheck["A"])
        self.assertTrue(result.repeat_crosscheck["B"])
        self.assertEqual(text.count('DRIVER_SEND cmd="move '), 5)
        self.assertEqual(text.count('DRIVER_SEND cmd="quit"'), 1)
        self.assertNotIn('DRIVER_SEND cmd="stop"', text)
        self.assertEqual(result.visits[-1].target_name, "A")
        self.assertIn("T4_STATS target=A n=3", notes)
        self.assertIn("T4_STATS target=B n=2", notes)
        self.assertIn("device_feedback_only=1", notes)
        self.assertIn("不是绝对精度", notes)

    def test_stale_old_seq_arrival_is_rejected(self):
        result, text, notes = self.run_mode("stale_arrival")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(len(result.visits), 5)
        self.assertIn("ARRIVAL_REJECTED reason=seq_mismatch", notes)
        self.assertEqual([v.seq for v in result.visits], [1, 2, 3, 4, 5])
        self.assertEqual(len(set(v.cycle for v in result.visits)), 5)

    def test_arrival_timeout_sends_one_stop_then_quit(self):
        result, text, notes = self.run_mode("no_arrival")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "arrival_timeout")
        self.assertEqual(result.stop_send_count, 1)
        self.assertTrue(result.stop_requested)
        self.assertTrue(result.stop_result_seen)
        self.assertTrue(result.stop_delivered)
        self.assertTrue(result.eof_seen)
        self.assertEqual(text.count('DRIVER_SEND cmd="move '), 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="quit"'), 1)
        self.assertIn("WAIT_TIMEOUT label=ARRIVAL", notes)

    def test_cmd_failed_sends_one_stop(self):
        result, text, notes = self.run_mode("cmd_failed")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "cmd_failed")
        self.assertEqual(result.stop_send_count, 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertIn("CMD_RESULT_FAILED", notes)

    def test_move_rejected_does_not_send_stop(self):
        result, text, notes = self.run_mode("reject")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "move_rejected")
        self.assertEqual(result.stop_send_count, 0)
        self.assertNotIn('DRIVER_SEND cmd="stop"', text)
        self.assertEqual(text.count('DRIVER_SEND cmd="quit"'), 1)

    def test_reused_cycle_is_not_two_arrivals(self):
        result, text, notes = self.run_mode("duplicate_cycle")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "arrival_timeout")
        self.assertEqual(len(result.visits), 1)
        self.assertIn("ARRIVAL_REJECTED reason=cycle_reused", notes)
        self.assertEqual(result.stop_send_count, 1)

    def test_wrong_target_arrival_is_rejected(self):
        result, text, notes = self.run_mode("wrong_target")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "arrival_timeout")
        self.assertEqual(len(result.visits), 0)
        self.assertIn("ARRIVAL_REJECTED reason=target_mismatch", notes)
        self.assertEqual(result.stop_send_count, 1)

    def test_repeat_crosscheck_mismatch_fails(self):
        result, text, notes = self.run_mode("bad_repeat")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "repeat_crosscheck")
        self.assertEqual(len(result.visits), 5)
        self.assertFalse(result.repeat_crosscheck["A"])
        self.assertTrue(result.repeat_crosscheck["B"])
        self.assertIn("REPEAT_CROSSCHECK target=A ok=0", notes)

    def test_nonzero_exit_code_fails(self):
        result, text, notes = self.run_mode("exit_nonzero")
        self.assertFalse(result.driver_ok)
        self.assertEqual(result.outcome, "exit_code")
        self.assertEqual(result.exit_code, 1)
        self.assertEqual(len(result.visits), 5)

    def test_target_outside_expected_envelope_does_not_launch(self):
        tmp = tempfile.TemporaryDirectory(prefix="t4-driver-test-")
        self.addCleanup(tmp.cleanup)
        log = Path(tmp.name) / "session.log"
        notes = Path(tmp.name) / "notes.txt"
        rc = driver.main([
            "--tool", sys.executable, "--config", "cfg", "--permit", "p",
            "--a-az", "-0.6200", "--a-el", "0.0500",
            "--b-az", "-0.1200", "--b-el", "0.0500",
            "--expect-az-min", "-0.5000", "--expect-az-max", "-0.1000",
            "--log", str(log), "--notes", str(notes)])
        self.assertEqual(rc, 2)
        self.assertFalse(log.exists())
        self.assertFalse(notes.exists())

    def test_one_round_trip_has_no_per_target_repeatability(self):
        tmp = tempfile.TemporaryDirectory(prefix="t4-driver-test-")
        self.addCleanup(tmp.cleanup)
        log = Path(tmp.name) / "session.log"
        notes = Path(tmp.name) / "notes.txt"
        rc = driver.main([
            "--tool", sys.executable, "--config", "cfg", "--permit", "p",
            "--a-az", "-0.6200", "--a-el", "0.0500",
            "--b-az", "-0.1200", "--b-el", "0.0500",
            "--round-trips", "1",
            "--log", str(log), "--notes", str(notes)])
        self.assertEqual(rc, 2)
        self.assertFalse(log.exists())
        self.assertFalse(notes.exists())

    def test_invalid_target_pairs_do_not_launch(self):
        tmp = tempfile.TemporaryDirectory(prefix="t4-driver-test-")
        self.addCleanup(tmp.cleanup)
        log = Path(tmp.name) / "session.log"
        notes = Path(tmp.name) / "notes.txt"
        rc = driver.main([
            "--tool", sys.executable, "--config", "cfg", "--permit", "p",
            "--a-az", "-0.1200", "--a-el", "0.0500",
            "--b-az", "-0.1200", "--b-el", "0.0500",
            "--log", str(log), "--notes", str(notes)])
        self.assertEqual(rc, 2)
        self.assertFalse(log.exists())
        self.assertFalse(notes.exists())
        # 间距 0.03° 小于默认 0.2°：无法从反馈区分两点，也不启动。
        rc = driver.main([
            "--tool", sys.executable, "--config", "cfg", "--permit", "p",
            "--a-az", "-0.1500", "--a-el", "0.0500",
            "--b-az", "-0.1200", "--b-el", "0.0500",
            "--log", str(log), "--notes", str(notes)])
        self.assertEqual(rc, 2)
        self.assertFalse(log.exists())
        self.assertFalse(notes.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
