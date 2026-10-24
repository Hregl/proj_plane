#!/usr/bin/env python3
"""t2_acceptance_driver 的离线假子进程；不连接 UDP/真实设备。

模式：
  normal       正常：两次到位后 quit，输出完整收尾记录，退出码 0
  late_arrival 第一次到位晚于驱动超时
  real_exit_nonzero 两次到位、日志写 EXIT_CODE 0，但真实进程退出码为 3
  log_exit_nonzero  真实进程退出码 0，但日志写 EXIT_CODE 3
  tail_output  quit 后延迟输出尾部记录（验证驱动会收集到 EOF）
  exit_hang    quit 后不再输出也不退出（验证驱动超时强杀并判失败）
  missing_final 退出码 0，但缺少 FINAL（验证不能误判成功）
  invalid_tail  完整收尾行之后写入无法按 UTF-8 解码的字节（验证读取
                异常不能被当成正常 EOF）

假进程的输出行格式刻意与 tools/turntable_acceptance 的关键行一致，
仅用于验证驱动状态机，不代表真实设备行为。
"""

from __future__ import annotations

import argparse
import sys
import time


def out(text: str) -> None:
    sys.stdout.write(text + "\n")
    sys.stdout.flush()


def emit_state(cycle: int, az: float, el: float) -> None:
    out(
        "STATE t=%dms az=%.4f el=%.4f motion=STABLE valid=1 fresh=1 "
        "cycle=%d" % (cycle * 50, az, el, cycle)
    )


def emit_normal_exit(
    seq: int,
    process_exit_code: int,
    *,
    log_exit_code: int | None = None,
    include_final: bool = True,
    normal_exit: int | None = None,
) -> None:
    """输出收尾行；真实进程退出码与日志 EXIT_CODE 可分别控制。"""
    if log_exit_code is None:
        log_exit_code = process_exit_code
    out("EXIT_REQUEST t=500ms reason=user_or_observe_done")
    out("EXIT_STOP stop_req=1 stop_done=1 delivered=0 stationary=1 "
        "uncertain=0 waited_ms=1")
    if include_final:
        if normal_exit is None:
            normal_exit = 1 if process_exit_code == 0 else 0
        # 两个定位命令、每个命令两个轴帧；只用于让收尾行看起来完整。
        out(
            "FINAL initialized=1 normal_exit=%d control_frames=%d "
            "query_frames=9 unknown_frames=0 stop_delivered=0 "
            "stop_stationary=1 uncertain=0 latched=0 first_fault=0"
            % (normal_exit, seq * 2)
        )
    out("EXIT_CODE %d" % log_exit_code)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="normal")
    parser.add_argument("--late-delay", type=float, default=0.8)
    parser.add_argument("--tail-delay", type=float, default=0.35)
    args = parser.parse_args()
    mode = args.mode

    az = -0.1100
    el = 0.0600
    seq = 0
    cycle = 1

    out("TOOL fake_t2_tool")
    out("CONFIG dir=\"fake\" summary=\"offline counterexample\" "
        "protocol=pelco-d")
    out("INIT_OK")
    emit_state(cycle, az, el)

    for raw in sys.stdin:
        cmd = raw.strip()
        if not cmd:
            continue
        out('USER t=0ms cmd="%s"' % cmd)

        if cmd.startswith("move el "):
            seq += 1
            target = float(cmd.split()[-1])
            out(
                "MOVE_REQUEST t=%dms az=%.4f el=%.4f reference_cycle=%d "
                "reference_az=%.4f reference_el=%.4f reference_fresh=1 "
                "at_motion=STABLE speed_control=0"
                % (seq * 100, az, target, cycle, az, el)
            )
            out("MOVE_ACCEPTED accepted=1 seq=%d" % seq)
            if mode == "late_arrival" and seq == 1:
                time.sleep(args.late_delay)
            else:
                time.sleep(0.02)
            out(
                "CMD_RESULT seq=%d outcome=Delivered msg=\"两轴指令已交付\" "
                "delivered_is_arrival=0" % seq
            )
            el = target
            cycle += 1
            out(
                "ARRIVAL_OBSERVED seq=%d cycle=%d az=%.4f el=%.4f "
                "target=[%.4f,%.4f] elapsed_ms=20 feedback_only=1"
                % (seq, cycle, az, el, az, target)
            )
            emit_state(cycle, az, el)
        elif cmd == "stop":
            out(
                "STOP_RESULT t=300ms request_seq=1 completed_seq=1 "
                "stop_delivered=0 stop_stationary=1 uncertain=0 "
                "stop_txn=1 motion_txn_before=0 send_path=no_frame"
            )
        elif cmd in ("quit", "q"):
            if mode == "exit_hang":
                out("EXIT_REQUEST t=400ms reason=user_or_observe_done")
                # 故意不输出 EXIT_STOP/FINAL/EXIT_CODE，也不退出。
                time.sleep(60.0)
                return 0
            if mode == "tail_output":
                out("EXIT_REQUEST t=400ms reason=user_or_observe_done")
                time.sleep(args.tail_delay)
                out("TAIL_AFTER_QUIT marker=collected")
                emit_normal_exit(seq, 0)
                return 0
            if mode == "real_exit_nonzero":
                # 日志尽量“完整且正常”（EXIT_CODE 0），只让真实进程
                # 退出码非 0，以单独隔离驱动对 returncode 的检查。
                emit_normal_exit(seq, 3, log_exit_code=0, normal_exit=1)
                return 3
            if mode == "log_exit_nonzero":
                # 真实进程退出码 0，只让日志 EXIT_CODE 非 0，以单独隔离
                # 驱动对收尾行 EXIT_CODE 的检查。
                emit_normal_exit(seq, 0, log_exit_code=3, normal_exit=1)
                return 0
            if mode == "missing_final":
                emit_normal_exit(seq, 0, include_final=False)
                return 0
            if mode == "invalid_tail":
                # 收尾行本身完整；留出读取时间后写入非法 UTF-8，模拟读取
                # 解码失败，确保驱动已经收到完整收尾行。
                emit_normal_exit(seq, 0)
                sys.stdout.flush()
                time.sleep(0.10)
                sys.stdout.buffer.write(b"\xff\xfeTAIL_NOT_UTF8\n")
                sys.stdout.buffer.flush()
                return 0
            emit_normal_exit(seq, 0)
            return 0
        else:
            out('UNKNOWN_COMMAND "%s"' % cmd)

    # stdin EOF：与真实工具的退出路径类似，仍输出完整收尾。
    emit_normal_exit(seq, 0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
