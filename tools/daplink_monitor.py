#!/usr/bin/env python3
"""Continuously decode the STM32 DAPLink monitor stream and save it as CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time


NODE_NAMES = (
    "S1", "P1", "N1", "B1", "B2", "B3", "N2", "P2", "S2", "P3",
    "N3", "N4", "N5", "N6", "P4", "N7", "P5", "B8", "B9", "N8",
    "C1", "C2", "C3", "N9", "N10", "N12", "N13", "P6", "N14", "S3",
    "S4", "N15", "S5", "C4", "C5", "B4", "B5", "B6", "B7", "N16",
    "N18", "N19", "P7", "N20", "N22", "C6", "C7", "C8", "C9", "P8",
    "N11", "C10",
)

FIELD_NAMES = (
    "tick", "round", "route_index", "last_node", "current_node", "next_node",
    "function", "mode", "stop_reason", "line_detail", "line_error_x10",
    "line_count", "led_count", "cross_detail", "cross_count", "cross_led_count",
    "yaw_x10", "pitch_x10", "roll_x10", "target_x10", "gyro_g_x10",
    "gyro_t_x10", "line_pid_x10", "target_speed_x10", "actual_speed_x10",
    "left_speed_x10", "right_speed_x10", "mileage_x10",
)

HEX_FIELDS = {"line_detail", "cross_detail"}
SCALED_FIELDS = tuple(name for name in FIELD_NAMES if name.endswith("_x10"))


def node_name(node_id: int) -> str:
    return NODE_NAMES[node_id] if 0 <= node_id < len(NODE_NAMES) else f"?{node_id}"


def parse_monitor_line(line: str) -> dict[str, int | float] | None:
    parts = line.strip().split(",")
    if len(parts) != len(FIELD_NAMES) + 1 or parts[0] != "MON":
        return None

    frame: dict[str, int | float] = {}
    try:
        for name, raw in zip(FIELD_NAMES, parts[1:]):
            frame[name] = int(raw, 16) if name in HEX_FIELDS else int(raw, 10)
    except ValueError:
        return None

    for name in SCALED_FIELDS:
        frame[name.removesuffix("_x10")] = float(frame[name]) / 10.0
    return frame


def format_status(frame: dict[str, int | float]) -> str:
    return (
        f"{node_name(int(frame['current_node']))}->{node_name(int(frame['next_node']))} "
        f"route={frame['route_index']} mode={frame['mode']} "
        f"line=0x{int(frame['line_detail']):04X}/{frame['line_error']:+.1f} "
        f"yaw={frame['yaw']:+.1f} target={frame['target']:+.1f} "
        f"pitch={frame['pitch']:+.1f} speed={frame['actual_speed']:.1f}/{frame['target_speed']:.1f} "
        f"mileage={frame['mileage']:.1f} stop={frame['stop_reason']}"
    )


def open_csv(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    handle = path.open("a", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=("host_time", *FIELD_NAMES, *[name.removesuffix("_x10") for name in SCALED_FIELDS]))
    if new_file:
        writer.writeheader()
        handle.flush()
    return handle, writer


def run(port_name: str, baud: int, csv_path: Path) -> None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请运行：python -m pip install pyserial") from exc

    csv_handle, writer = open_csv(csv_path)
    try:
        while True:
            try:
                with serial.Serial(port_name, baudrate=baud, timeout=1) as port:
                    print(f"已连接 {port_name} @ {baud}，日志：{csv_path}")
                    while True:
                        frame = parse_monitor_line(port.readline().decode("ascii", errors="replace"))
                        if frame is None:
                            continue
                        print(format_status(frame), flush=True)
                        row = {"host_time": time.strftime("%Y-%m-%dT%H:%M:%S"), **frame}
                        writer.writerow(row)
                        csv_handle.flush()
            except serial.SerialException as exc:
                print(f"串口断开：{exc}；1 秒后重连…", file=sys.stderr)
                time.sleep(1)
    finally:
        csv_handle.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3", help="DAPLink 虚拟串口（默认：COM3）")
    parser.add_argument("--baud", type=int, default=230400, help="波特率（默认：230400）")
    parser.add_argument("--csv", type=Path, default=Path("logs/daplink-monitor.csv"), help="CSV 日志路径")
    args = parser.parse_args()
    run(args.port, args.baud, args.csv)


if __name__ == "__main__":
    main()
