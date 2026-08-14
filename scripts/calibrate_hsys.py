#!/usr/bin/env python3
"""Trigger an Hsys loopback system-response calibration on a running BS.

This is the OTHER calibration -- CALB, not CALD. It characterizes the SDR's
own frequency response (filters/DAC/ADC) and needs TX and RX physically
connected by a direct RF cable, NOT your normal antennas. Because a direct
cable has essentially none of the path loss real air propagation would give
you, TX/RX gain gets dropped first to avoid saturating the receiver, then
restored once the capture is done.

A cable alone at minimum software gain may still be too hot for the RX
front end -- USRP "gain" settings are relative, not an attenuator down to
zero. If you see clipping/garbage results, add a physical inline attenuator
(20-30 dB is typical) between TX and RX in addition to lowering gain here.
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
    build_system_response_calibration_command,
    make_control_dealer,
    make_tcp_endpoint,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--host", default="127.0.0.1", help="BS control host (default: 127.0.0.1)")
    p.add_argument("--control-port", type=int, default=9999, help="control_port (default: 9999)")
    p.add_argument("--loopback-tx-gain-db", type=float, default=0.0,
                    help="TX gain (dB) to drive during loopback capture (default: 0.0)")
    p.add_argument("--loopback-rx-gain-db", type=float, default=0.0,
                    help="RX gain (dB) to drive during loopback capture (default: 0.0)")
    p.add_argument("--restore-tx-gain-db", type=float, required=True,
                    help="TX gain (dB) to restore after capture -- your normal operating gain")
    p.add_argument("--restore-rx-gain-db", type=float, required=True,
                    help="RX gain (dB) to restore after capture -- your normal operating gain")
    p.add_argument("--symbols", type=int, default=1000,
                    help="Symbols to average over (default: 1000, matches backend default)")
    return p.parse_args()


def send_tx_gain(sock, gain_db: float) -> None:
    sock.send_multipart([build_control_command(b"TXGN", int(round(gain_db * 10.0)))])


def send_rx_gain(sock, gain_db: float) -> None:
    sock.send_multipart([build_control_command(b"RXGN", int(round(gain_db * 10.0)))])


def main() -> int:
    args = parse_args()
    endpoint = make_tcp_endpoint(args.host, args.control_port)
    sock = make_control_dealer(endpoint, identity="calibrate-hsys")

    print(f"[calibrate-hsys] control -> {endpoint}")
    input(
        "\n*** Physically connect TX to RX with a direct RF cable now ***\n"
        "(through an inline attenuator if you have one -- see this script's --help)\n"
        "Do NOT run this over the air, and do NOT use your normal sensing antennas.\n"
        "Press Enter once the loopback cable is connected: "
    )

    print(
        f"[calibrate-hsys] dropping gains for loopback: "
        f"TX={args.loopback_tx_gain_db} dB, RX={args.loopback_rx_gain_db} dB"
    )
    send_tx_gain(sock, args.loopback_tx_gain_db)
    send_rx_gain(sock, args.loopback_rx_gain_db)
    time.sleep(0.2)

    print(f"[calibrate-hsys] requesting CALB capture over {args.symbols} symbols")
    sock.send_multipart([build_system_response_calibration_command(args.symbols)])

    input(
        "\nWatch the BS console for a line like:\n"
        "  [Sensing Hsys ... ] saved system response calibration: ...\n"
        "Press Enter once you see it to restore gains and disconnect the loopback: "
    )

    print(
        f"[calibrate-hsys] restoring gains: "
        f"TX={args.restore_tx_gain_db} dB, RX={args.restore_rx_gain_db} dB"
    )
    send_tx_gain(sock, args.restore_tx_gain_db)
    send_rx_gain(sock, args.restore_rx_gain_db)
    time.sleep(0.2)
    print("[calibrate-hsys] done. Reconnect your normal sensing antennas (TX/RX + RX2) now.")
    print("The saved sensing_system_response_*.bin loads automatically on BS's next start;")
    print("delete it to go back to no Hsys equalization.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
