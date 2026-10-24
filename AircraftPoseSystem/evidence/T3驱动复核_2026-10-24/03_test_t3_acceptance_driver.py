#!/usr/bin/env python3
"""离线验证 T3 驱动状态机：只启动假子进程，不连接设备、不发 UDP。

覆盖：
  1. 正常触发：连续 3 个同向俯仰变化后只发一次普通 stop；
  2. 未观测到运动：等待超时后进入同一条停止收尾；
  3. 方向相反：不得触发 stop，超时后收尾；
  4. 触发前已到位：不得把收尾 stop 当作运动中的停止；
  5. 缺少 STOP_RESULT：不得把“stop 已发”当成“发送结果已知”，也不得
     用本次 stop 请求之前的旧 STOP_RESULT 解除等待。
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1]
if not (TOOLS_DIR / "t3_acceptance_driver.py").exists():
    TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import t3_acceptance_driver as driver  # noqa: E402

FAKE = Path(__file__).resolve().with_name("fake_t3_tool.py")


class T3DriverTest(unittest.TestCase):
    def run_mode(self, mode: str, fake_args=(), **kwargs):
        tmp = tempfile.TemporaryDirectory(prefix="t3-driver-test-")
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        log = root / "session.log"
        notes = root / "notes.txt"
        command = [sys.executable, str(FAKE), "--mode", mode, *fake_args]
        params = dict(
            move_az=-0.5200,
            move_el=1.5600,
            direction="decrease",
            trigger_samples=3,
            min_step_deg=0.005,
            fresh_s=0.5,
            max_gap_s=0.5,
            init_timeout_s=2.0,
            state_timeout_s=2.0,
            move_ack_timeout_s=2.0,
            motion_timeout_s=0.35,
            stop_wait_s=1.0,
            stop_observe_s=1.0,
            exit_timeout_s=3.0,
        )
        params.update(kwargs)
        result = driver.run_driver(
            command=command, log_path=log, notes_path=notes, **params)
        text = log.read_text(encoding="utf-8")
        note_text = notes.read_text(encoding="utf-8")
        return result, text, note_text

    def test_normal_trigger_sends_exactly_one_stop(self):
        result, text, notes = self.run_mode("normal")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "triggered")
        self.assertTrue(result.stop_requested)
        self.assertEqual(result.stop_send_count, 1)
        self.assertGreaterEqual(result.trigger_samples, 3)
        self.assertTrue(all(d <= -0.005 for d in result.trigger_deltas))
        self.assertTrue(result.stop_result_seen)
        self.assertTrue(result.stop_delivered)
        self.assertEqual(result.stop_send_path, "acceptance_stop")
        self.assertTrue(result.stop_observed_stationary)
        self.assertTrue(result.eof_seen)
        self.assertEqual(result.exit_code, 0)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('USER t=0ms cmd="stop"'), 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="move '), 1)
        self.assertNotIn("ARRIVAL_OBSERVED", text)
        self.assertIn("T3_TRIGGER", notes)
        self.assertTrue(result.stop_write_ok)
        self.assertIn("T3_STOP_REQUEST reason=trigger", notes)
        self.assertIn("T3_STOP_WRITE write_ok=1", notes)

    def test_no_motion_timeout_enters_stop_closeout(self):
        result, text, notes = self.run_mode("no_motion")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "no_motion_timeout")
        self.assertTrue(result.stop_requested)
        self.assertEqual(result.stop_send_count, 1)
        self.assertEqual(result.trigger_samples, 0)
        self.assertTrue(result.stop_result_seen)
        self.assertTrue(result.stop_delivered)
        self.assertTrue(result.eof_seen)
        self.assertEqual(result.exit_code, 0)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('USER t=0ms cmd="stop"'), 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="move '), 1)
        self.assertIn("WAIT_TIMEOUT label=TRIGGER", notes)
        self.assertTrue(result.stop_write_ok)
        self.assertIn("T3_STOP_REQUEST reason=closeout:no_motion_timeout", notes)
        self.assertNotIn("T3_TRIGGER", notes)

    def test_opposite_direction_does_not_trigger(self):
        result, text, notes = self.run_mode("wrong_direction")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "wrong_direction_timeout")
        self.assertEqual(result.stop_send_count, 1)
        self.assertEqual(result.trigger_samples, 0)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('USER t=0ms cmd="stop"'), 1)
        self.assertNotIn("T3_TRIGGER", notes)

    def test_arrival_before_trigger_is_not_a_motion_stop(self):
        result, text, notes = self.run_mode("fast_arrival")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "arrival_before_trigger")
        self.assertTrue(result.arrival_before_trigger)
        self.assertEqual(result.trigger_samples, 0)
        self.assertEqual(result.stop_send_count, 1)
        self.assertFalse(result.stop_delivered)
        self.assertEqual(result.stop_send_path, "no_motion_in_flight")
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('USER t=0ms cmd="stop"'), 1)
        self.assertIn("ARRIVAL_BEFORE_TRIGGER", notes)
        self.assertNotIn("T3_TRIGGER", notes)

    def test_missing_stop_result_is_not_reported_as_delivered(self):
        result, text, notes = self.run_mode("no_stop_result")
        self.assertEqual(result.outcome, "triggered")
        self.assertEqual(result.stop_send_count, 1)
        # 启动时的无请求旧结果必须被忽略，不能解除本次收尾等待。
        self.assertEqual(result.stop_result_before_request, 1)
        self.assertIn("STOP_RESULT_BEFORE_REQUEST ignored", notes)
        self.assertFalse(result.stop_result_seen)
        self.assertIsNone(result.stop_delivered)
        self.assertFalse(result.driver_ok)
        self.assertEqual(text.count('DRIVER_SEND cmd="stop"'), 1)
        self.assertEqual(text.count('USER t=0ms cmd="stop"'), 1)
        self.assertEqual(text.count('DRIVER_SEND cmd="move '), 1)

    def test_wrong_address_is_ignored(self):
        result, text, notes = self.run_mode(
            "normal", fake_args=("--address", "2"))
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "no_motion_timeout")
        self.assertEqual(result.trigger_samples, 0)
        self.assertIn("RX_WRONG_ADDRESS", notes)
        self.assertNotIn("T3_TRIGGER", notes)

    def test_not_observed_is_reported_as_such(self):
        result, text, notes = self.run_mode("not_observed")
        self.assertTrue(result.driver_ok, result.__dict__)
        self.assertEqual(result.outcome, "triggered")
        self.assertTrue(result.stop_delivered)
        self.assertFalse(result.stop_observed_stationary)
        self.assertTrue(result.stop_not_observed)
        self.assertIn("STOP_NOT_OBSERVED seen", notes)
        self.assertNotIn("STOP_OBSERVED_STATIONARY seen", notes)

    def make_session(self, fresh_s=5.0):
        tmp = tempfile.TemporaryDirectory(prefix="t3-driver-unit-")
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        session = driver._Session(
            command=[sys.executable, "-c", "pass"],
            log_path=root / "log", notes_path=root / "notes",
            move_az=-0.5200, move_el=1.5600, address=1,
            direction="decrease",
            trigger_samples=3, min_step_deg=0.005, fresh_s=fresh_s,
            max_gap_s=0.5, init_timeout_s=1.0, state_timeout_s=1.0,
            move_ack_timeout_s=1.0, motion_timeout_s=1.0,
            stop_wait_s=0.1, stop_observe_s=0.1, exit_timeout_s=1.0)
        session.move_accepted = True
        return session

    def test_stale_sample_is_not_used(self):
        session = self.make_session(fresh_s=0.5)
        session._on_angle_sample(time.monotonic() - 10.0, 1000, 3.0900)
        self.assertIsNone(session.last_sample_recv)
        self.assertFalse(session.triggered)
        self.assertIn("STALE_SAMPLE", "\n".join(session.notes))

    def test_gap_resets_run_before_trigger(self):
        session = self.make_session(fresh_s=5.0)
        base = time.monotonic() - 2.0
        # 命令受理前的缓存 RX 不得成为基线或变化计数。
        session.move_accepted = False
        session._on_angle_sample(base - 0.3, 500, 3.1500)
        session._on_angle_sample(base - 0.2, 600, 3.1300)
        self.assertIsNone(session.last_sample_recv)
        self.assertFalse(session.triggered)
        session.move_accepted = True
        # 基线，然后 1.0 s 断档：断档后的第一个变化不得接上旧序列。
        session._on_angle_sample(base, 1000, 3.0900)
        session._on_angle_sample(base + 1.0, 2000, 3.0700)
        session._on_angle_sample(base + 1.06, 2060, 3.0500)
        session._on_angle_sample(base + 1.12, 2120, 3.0300)
        self.assertFalse(session.triggered)
        self.assertEqual(len(session.consecutive_deltas), 2)
        self.assertIn("SAMPLE_GAP_RESET", "\n".join(session.notes))
        # 第三个连续同向变化才触发，且只请求一次 stop。
        session._on_angle_sample(base + 1.18, 2180, 3.0100)
        self.assertTrue(session.triggered)
        self.assertEqual(len(session.trigger_deltas), 3)
        self.assertEqual(session.stop_send_count, 1)
        session._on_angle_sample(base + 1.24, 2240, 2.9900)
        self.assertEqual(session.stop_send_count, 1)


if __name__ == "__main__":
    unittest.main()
