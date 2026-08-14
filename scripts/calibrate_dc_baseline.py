#!/usr/bin/env python3
"""Trigger a TX-off DC/self-interference baseline capture on a running BS.

Mutes TX (TXGN), requests the CALD capture, waits for you to confirm the
backend has finished (watch its console for "captured TX-off DC baseline"),
then restores TX gain. The backend does this per-channel and applies the
result automatically on every future startup once the .bin file exists next
to wherever BS was run from -- delete that file (or never run this script) to
go back to no DC-baseline correction.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sensing_runtime_protocol import (  # noqa: E402
    build_control_command,
    build_dc_baseline_calibration_command,
    make_control_dealer,
    make_tcp_endpoint,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1", help="BS control host (default: 127.0.0.1)")
    p.add_argument("--control-port", type=int, default=9999, help="control_port (default: 9999)")
    p.add_argument("--mute-gain-db", type=float, default=0.0,
                    help="TX gain (dB) to drive during capture (default: 0.0)")
    p.add_argument("--restore-gain-db", type=float, required=True,
                    help="TX gain (dB) to restore after capture -- your normal operating gain")
    p.add_argument("--symbols", type=int, default=1000,
                    help="Symbols to average over (default: 1000, matches backend default)")
    return p.parse_args()


def send_tx_gain(sock, gain_db: float) -> None:
    sock.send_multipart([build_control_command(b"TXGN", int(round(gain_db * 10.0)))])


def main() -> int:
    args = parse_args()
    endpoint = make_tcp_endpoint(args.host, args.control_port)
    sock = make_control_dealer(endpoint, identity="calibrate-dc-baseline")

    print(f"[calibrate-dc-baseline] control -> {endpoint}")
    print(f"[calibrate-dc-baseline] muting TX to {args.mute_gain_db} dB")
    send_tx_gain(sock, args.mute_gain_db)
    time.sleep(0.2)

    print(f"[calibrate-dc-baseline] requesting CALD capture over {args.symbols} symbols")
    sock.send_multipart([build_dc_baseline_calibration_command(args.symbols)])

    input(
        "\nWatch the BS console for a line like:\n"
        "  [Sensing DCBL ... ] captured TX-off DC baseline over N symbols; saving to ...\n"
        "Press Enter once you see it (or after a few seconds at your frame rate) "
        "to restore TX gain: "
    )

    print(f"[calibrate-dc-baseline] restoring TX to {args.restore_gain_db} dB")
    send_tx_gain(sock, args.restore_gain_db)
    time.sleep(0.2)
    print("[calibrate-dc-baseline] done. The saved .bin loads automatically on BS's next start;")
    print("delete sensing_dc_baseline_*.bin next to where BS runs to disable the correction.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
