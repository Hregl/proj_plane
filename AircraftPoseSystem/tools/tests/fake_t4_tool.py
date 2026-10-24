#!/usr/bin/env python3
"""T4 离线假工具：只模拟 ``turntable_acceptance`` 的 stdout/stdin 协议。

不连接设备、不发送任何 UDP。用于验证 T4 驱动的逐条到位、目标分组统计、
失败收尾与 REPEAT 交叉核对。场景由 ``--mode`` 与环境变量控制：

* ``FAKE_T4_OFFSETS_A`` / ``FAKE_T4_OFFSETS_B``：每次到位的反馈偏差，
  逗号分隔，按该目标的访问顺序取用（不足则循环）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Tuple


def out(text: str) -> None:
    sys.stdout.write(text + "\n")
    sys.stdout.flush()


def offsets(name: str, default: List[float]) -> List[float]:
    raw = os.environ.get("FAKE_T4_OFFSETS_%s" % name)
    if not raw:
        return default
    try:
        values = [float(x) for x in raw.split(",") if x.strip()]
    except ValueError:
        return default
    return values or default


def pop_std(values: List[float]) -> float:
    mean = sum(values) / float(len(values))
    return math.sqrt(sum((v - mean) ** 2 for v in values) / float(len(values)))


class Fake:
    def __init__(self, mode: str) -> None:
        self.mode = mode
        self.seq = 0
        self.cycle = 10
        self.arrivals: List[Tuple[str, float, float, float, float]] = []
        self.offsets = {
            "A": offsets("A", [0.0020, -0.0010, 0.0010]),
            "B": offsets("B", [0.0010, -0.0020]),
        }
        self.visit_index: Dict[str, int] = {"A": 0, "B": 0}

    def state(self) -> None:
        for i in range(3):
            out("STATE cycle=%d az=-0.1200 el=0.0500 motion=IDLE "
                "valid=1 fresh=1" % (100 + i))

    def arrival(self, name: str, az: float, el: float,
                seq: int, cycle: int) -> None:
        idx = self.visit_index[name]
        offs = self.offsets[name]
        da = offs[idx % len(offs)]
        idx += 1
        self.visit_index[name] = idx
        de = 0.0010 * (1 if idx % 2 else -1)
        faz = az + da
        fel = el + de
        out("ARRIVAL_OBSERVED seq=%d cycle=%d az=%.4f el=%.4f "
            "target=[%.4f,%.4f] elapsed_ms=120 "
            "feedback_only=1（设备反馈稳定，不等于独立实测静止）"
            % (seq, cycle, faz, fel, az, el))
        self.arrivals.append((name, az, el, faz, fel))

    def handle_move(self, parts: List[str]) -> None:
        try:
            az = float(parts[1])
            el = float(parts[2])
        except (IndexError, ValueError):
            out("MOVE_REJECTED reason=usage_or_nonfinite")
            return
        if self.mode == "reject":
            out("MOVE_REJECTED reason=permit_target_outside")
            return
        name = "A" if abs(az - self.target_a) <= 1e-9 else "B"
        self.seq += 1
        self.cycle += 1
        out("MOVE_REQUEST t=10ms az=%.4f el=%.4f reference_cycle=%d "
            "reference_az=-0.1200 reference_el=0.0500 reference_fresh=1 "
            "at_motion=IDLE speed_control=1" % (az, el, self.cycle - 1))
        out("MOVE_ACCEPTED accepted=1 seq=%d" % self.seq)
        if self.mode == "no_arrival":
            return
        if self.mode == "cmd_failed":
            out("CMD_RESULT seq=%d outcome=Failed msg=\"fake failure\" "
                "delivered_is_arrival=0" % self.seq)
            return
        if self.mode == "stale_arrival":
            out("ARRIVAL_OBSERVED seq=%d cycle=%d az=%.4f el=%.4f "
                "target=[%.4f,%.4f] elapsed_ms=90 feedback_only=1"
                % (self.seq - 1, self.cycle - 1, az, el, az, el))
        if self.mode == "wrong_target":
            out("ARRIVAL_OBSERVED seq=%d cycle=%d az=%.4f el=%.4f "
                "target=[%.4f,%.4f] elapsed_ms=100 feedback_only=1"
                % (self.seq, self.cycle, az, el, az + 1.0, el))
            return
        cycle = self.cycle
        if self.mode == "duplicate_cycle":
            cycle = 11
        self.arrival(name, az, el, self.seq, cycle)

    def handle_stop(self) -> None:
        out("STOP_REQUEST t=200ms stop_id=1 at_motion=MOVING cycle=%d "
            "az=0.0000 el=0.0000 fresh=1 txn=1 stop_txn=0" % self.cycle)
        out("STOP_RESULT t=200ms stop_id=1 request_seq=1 completed_seq=1 "
            "stop_delivered=1 stop_stationary=0 uncertain=0 stop_txn=1 "
            "motion_txn_before=1 send_path=stop_frame_delivered")
        out("STOP_OBSERVED_STATIONARY stop_txn=1 cycle=%d "
            "az=0.0000 el=0.0000 elapsed_ms=100" % self.cycle)

    def repeat(self) -> None:
        groups: Dict[Tuple[float, float], List[Tuple[str, float, float]]] = {}
        for name, az, el, faz, fel in self.arrivals:
            groups.setdefault((az, el), []).append((name, faz, fel))
        for (az, el), rows in groups.items():
            if len(rows) < 2:
                continue
            azs = [r[1] for r in rows]
            els = [r[2] for r in rows]
            sd_az = pop_std(azs)
            if self.mode == "bad_repeat" and abs(az - self.target_a) <= 1e-9:
                sd_az += 0.5
            out("REPEAT n=%d target_az=%.4f target_el=%.4f "
                "az_mean=%.4f az_std=%.4f el_mean=%.4f el_std=%.4f "
                "device_feedback_only=1 absolute_accuracy=NOT_MEASURED "
                "arcmin_acceptance=NOT_MEASURED"
                % (len(rows), az, el, sum(azs) / len(azs), sd_az,
                   sum(els) / len(els), pop_std(els)))

    def finish(self) -> int:
        self.repeat()
        out("EXIT_STOP stop_req=2 stop_done=2 delivered=1 stationary=1 "
            "uncertain=0 waited_ms=0")
        if self.mode == "abnormal_exit":
            out("FINAL initialized=1 normal_exit=0 control_frames=%d "
                "query_frames=20 unknown_frames=0 stop_delivered=0 "
                "stop_stationary=1 uncertain=0 latched=0 first_fault=0"
                % self.seq)
            out("EXIT_CODE 0")
            return 0
        if self.mode == "exit_nonzero":
            out("FINAL initialized=1 normal_exit=1 control_frames=%d "
                "query_frames=20 unknown_frames=0 stop_delivered=0 "
                "stop_stationary=1 uncertain=0 latched=0 first_fault=0"
                % self.seq)
            out("EXIT_CODE 1")
            return 1
        out("FINAL initialized=1 normal_exit=1 control_frames=%d "
            "query_frames=20 unknown_frames=0 stop_delivered=0 "
            "stop_stationary=1 uncertain=0 latched=0 first_fault=0"
            % self.seq)
        out("EXIT_CODE 0")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="normal")
    parser.add_argument("--config", default="")
    parser.add_argument("--permit", default="")
    args = parser.parse_args()

    fake = Fake(args.mode)
    fake.target_a = float(os.environ.get("FAKE_T4_A_AZ", "-0.6200"))
    fake.target_b = float(os.environ.get("FAKE_T4_B_AZ", "-0.1200"))

    out('CONFIG dir="fake" summary="fake" protocol=pelco_d_udp '
        'pelco_address=1 use_speed_control=1 coarse_speed=1.200')
    if args.mode == "permit_rejected":
        out('PERMIT_REJECTED "许可已过期，拒绝安装"')
        return 2
    out("PERMIT_INSTALLED scope=\"fake\"")
    out("SEND_POLICY motion_enabled(acceptance_permit)")
    out("INIT_OK initialized=1")
    fake.state()

    if args.mode == "no_init":
        return 0

    while True:
        line = sys.stdin.readline()
        if line == "":
            return 1
        line = line.strip()
        if not line:
            continue
        if line == "quit" or line == "q":
            return fake.finish()
        if line == "stop":
            fake.handle_stop()
            continue
        if line == "status":
            fake.state()
            continue
        if line.startswith("move "):
            fake.handle_move(line.split())
            continue
        out("UNKNOWN_COMMAND \"%s\"" % line)


if __name__ == "__main__":
    sys.exit(main())
