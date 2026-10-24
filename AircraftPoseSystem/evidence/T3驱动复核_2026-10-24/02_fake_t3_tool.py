#!/usr/bin/env python3
"""t3_acceptance_driver 的离线假子进程；不连接 UDP/真实设备。

模式：
  normal          俯仰角连续下降，驱动应在第 3 个同向变化后发一次 stop
  no_motion       俯仰角恒定，驱动应等待超时后进入停止收尾
  wrong_direction 俯仰角连续上升，驱动不得触发 stop；超时后收尾
  fast_arrival    少于触发样本数就 ARRIVAL，驱动不得把 stop 当运动停止
  no_stop_result  正常运动，收到 stop 后不输出 STOP_RESULT；启动时先输出
                  一条无请求的旧结果（验证驱动不把“停止请求已发”或旧结果
                  当成本次“发送结果已知”）
  not_observed    停止帧已交付，但观测窗口内未观测到反馈平台

假进程输出行格式刻意与 tools/turntable_acceptance 的关键行一致，
仅用于验证驱动状态机，不代表真实设备行为。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

_LOCK = threading.Lock()
_STOP = threading.Event()
_EMITTED_STOP_RESULT = False


def out(text: str) -> None:
    with _LOCK:
        sys.stdout.write(text + "\n")
        sys.stdout.flush()


def checksum(frame: list[int]) -> int:
    return sum(frame) & 0xFF


def emit_rx(tool_ms: int, address: int, data1: int, data2: int,
            data3: int, data4: int) -> None:
    frame = [0xFF, address, data1, data2, data3, data4]
    frame.append(checksum(frame[1:]))
    out("RX t=%dms n=1 kind=reply bytes %s"
        % (tool_ms, " ".join("%02X" % b for b in frame)))


def emit_state(tool_ms: int, az: float, el: float, cycle: int,
               motion: str = "MOVING") -> None:
    out("STATE t=%dms az=%.4f el=%.4f motion=%s valid=1 fresh=1 "
        "cycle=%d txn=1" % (tool_ms, az, el, motion, cycle))


def motion_thread(mode: str, az: float, start_raw: int, seq: int,
                  address: int) -> None:
    global _EMITTED_STOP_RESULT
    step = 0
    if mode in ("normal", "fast_arrival", "no_stop_result", "not_observed"):
        step = -2
    elif mode == "wrong_direction":
        step = +2
    cycle = 2
    for i in range(1, 201):
        if _STOP.is_set():
            return
        time.sleep(0.01)
        if _STOP.is_set():
            return
        raw = start_raw + step * i
        angle = raw / 100.0
        emit_rx(60 + i * 10, address, 0x00, 0x5B,
                (raw >> 8) & 0xFF, raw & 0xFF)
        emit_state(60 + i * 10, az, angle, cycle)
        cycle += 1
        if mode == "fast_arrival" and i >= 2:
            out("ARRIVAL_OBSERVED seq=%d cycle=%d az=%.4f el=%.4f "
                "target=[%.4f,%.4f] elapsed_ms=20 feedback_only=1"
                % (seq, cycle, az, angle, az, 1.56))
            _STOP.set()
            return


def emit_stop_result(mode: str, seq: int, stop_id: int) -> None:
    global _EMITTED_STOP_RESULT
    delivered = 0 if mode == "fast_arrival" else 1
    stationary = 0 if mode == "not_observed" else 1
    path = "no_motion_in_flight" if delivered == 0 else "acceptance_stop"
    out("STOP_RESULT t=300ms stop_id=%d request_seq=%d completed_seq=%d "
        "stop_delivered=%d stop_stationary=%d uncertain=%d "
        "stop_txn=%d motion_txn_before=1 send_path=%s"
        % (stop_id, seq, seq, delivered, stationary,
           1 if mode == "not_observed" else 0, seq + 1, path))
    if mode == "not_observed":
        out("STOP_NOT_OBSERVED stop_txn=%d uncertain=1 "
            "reason=no_stationary_in_window" % (seq + 1))
    elif delivered:
        out("STOP_OBSERVED_STATIONARY t=420ms samples=4 "
            "span_ms=180 stop_seq=%d" % seq)
    _EMITTED_STOP_RESULT = True


def emit_exit(seq: int, stopped: bool, delivered: int) -> None:
    out("EXIT_REQUEST t=500ms reason=user_or_observe_done")
    out("EXIT_STOP stop_req=%d stop_done=1 delivered=%d stationary=1 "
        "uncertain=0 waited_ms=1"
        % (1 if stopped else 0, delivered))
    out("FINAL initialized=1 normal_exit=1 control_frames=%d "
        "query_frames=40 unknown_frames=0 stop_delivered=%d "
        "stop_stationary=1 uncertain=0 latched=0 first_fault=0"
        % (2 + (1 if delivered else 0), delivered))
    out("EXIT_CODE 0")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="normal")
    parser.add_argument("--address", type=int, default=1)
    args = parser.parse_args()
    mode = args.mode

    az = -0.5200
    start_raw = 309
    seq = 0
    stop_id = 0
    stopped = False
    delivered = 0
    worker = None

    out("TOOL fake_t3_tool")
    out("CONFIG dir=\"fake\" summary=\"offline T3 counterexample\" "
        "protocol=pelco-d")
    out("INIT_OK")
    emit_state(0, az, start_raw / 100.0, 1, "STABLE")
    if mode == "no_stop_result":
        # 会话开始时的无请求旧结果：驱动不得用它解除本次停止收尾等待。
        out("STOP_RESULT t=5ms stop_id=0 request_seq=0 completed_seq=0 "
            "stop_delivered=1 stop_stationary=1 uncertain=0 "
            "stop_txn=0 motion_txn_before=0 "
            "send_path=stop_frame_delivered")

    for line in sys.stdin:
        cmd = line.strip()
        if not cmd:
            continue
        out('USER t=0ms cmd="%s"' % cmd)

        if cmd.startswith("move "):
            seq += 1
            parts = cmd.split()
            target_az = float(parts[1])
            target_el = float(parts[2])
            out("MOVE_REQUEST t=20ms az=%.4f el=%.4f reference_cycle=1 "
                "reference_az=%.4f reference_el=%.4f reference_fresh=1 "
                "at_motion=STABLE speed_control=1"
                % (target_az, target_el, az, start_raw / 100.0))
            out("MOVE_ACCEPTED accepted=1 seq=%d" % seq)
            out("CMD_RESULT seq=%d outcome=Delivered msg=\"两轴指令已交付\" "
                "delivered_is_arrival=0" % seq)
            worker = threading.Thread(
                target=motion_thread,
                args=(mode, az, start_raw, seq, args.address), daemon=True)
            worker.start()
        elif cmd == "stop":
            stopped = True
            stop_id += 1
            _STOP.set()
            if worker is not None:
                worker.join(timeout=0.5)
            out("STOP_REQUEST t=290ms stop_id=%d at_motion=MOVING "
                "cycle=2 az=%.4f el=3.0100 fresh=1 txn=1 stop_txn=0"
                % (stop_id, az))
            if mode == "no_stop_result":
                continue
            emit_stop_result(mode, seq, stop_id)
            delivered = 0 if mode == "fast_arrival" else 1
        elif cmd in ("quit", "q"):
            _STOP.set()
            if worker is not None:
                worker.join(timeout=0.5)
            if not _EMITTED_STOP_RESULT and mode != "no_stop_result":
                emit_stop_result(mode, seq, stop_id)
                delivered = 0 if mode == "fast_arrival" else 1
            emit_exit(seq, stopped, delivered)
            return 0
        else:
            out('UNKNOWN_COMMAND "%s"' % cmd)

    _STOP.set()
    if worker is not None:
        worker.join(timeout=0.5)
    emit_exit(seq, stopped, delivered)
    return 0


if __name__ == "__main__":
    sys.exit(main())
