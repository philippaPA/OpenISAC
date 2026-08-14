#!/usr/bin/env python3
"""Standalone drone detector for the OpenISAC mono sensing ZMQ feed.

Physics-based v1 (no ML, no training data): tracks the strongest CFAR target
in the range-Doppler map, accumulates its slow-time range-bin history, and
looks for a persistent harmonic comb of Doppler sidebands in the resulting
micro-Doppler spectrum -- the blade-flash signature of a rotor. Does not
modify or import sensing_viewer/fast_viewer.py; only depends on the
lightweight sensing_runtime_protocol / sensing_targets modules it already
uses for wire decode and OS-CFAR.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sensing_runtime_protocol import (  # noqa: E402
    CTRL_HEADER,
    PARAMS_COMMAND,
    READY_COMMAND,
    ViewerRuntimeParams,
    build_params_request,
    decode_aggregate_sensing_payload,
    decode_sensing_payload,
    make_control_dealer,
    make_data_sub,
    make_tcp_endpoint,
    parse_params_packet,
    recv_sensing_frame,
)
from sensing_targets import (  # noqa: E402
    OsCfarParams,
    cluster_detected_targets,
    run_os_cfar_2d,
)

try:
    import zmq
except ImportError as exc:  # pragma: no cover - startup dependency check
    raise SystemExit("pyzmq is required. Install it with `pip install pyzmq`.") from exc


C_LIGHT_MPS = 299_792_458.0

# The backend's advertised doppler_fft_size (e.g. 100 in BS_B205.yaml) is too
# small for OS-CFAR's default train+guard window (30 cells each side) to have
# a usable interior -- a near-hover target would fall in the excluded margin
# and never be seen. Oversample locally, same as the viewer does.
MIN_DOPPLER_PROCESSING_FFT_SIZE = 512

# Confidence scaling: heuristic, not a calibrated probability. Tune these
# against real labeled flight data once collected (v2).
CONFIDENCE_SNR_DB_SCALE = 8.0
CONFIDENCE_HARMONIC_BONUS = 5.0


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)
    sys.stderr.flush()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Physics-based (non-ML) drone micro-Doppler monitor for the mono sensing feed."
    )
    p.add_argument("--host", default="127.0.0.1", help="Backend host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=8888, help="mono_sensing_port (default: 8888)")
    p.add_argument("--control-port", type=int, default=9999, help="control_port (default: 9999)")

    p.add_argument("--update-interval", type=float, default=1.0,
                    help="Seconds between cadence analyses / JSON emissions (default: 1.0)")
    p.add_argument("--history-seconds", type=float, default=2.0,
                    help="Seconds of slow-time samples kept for micro-Doppler analysis (default: 2.0)")

    p.add_argument("--stft-nperseg", type=int, default=256, help="STFT window length in samples (default: 256)")
    p.add_argument("--stft-noverlap", type=int, default=192, help="STFT overlap in samples (default: 192)")

    p.add_argument("--body-exclusion-hz", type=float, default=40.0,
                    help="Exclude |f| below this (body/bulk-motion Doppler) from the sideband/comb search (default: 40.0)")
    p.add_argument("--cadence-min-hz", type=float, default=20.0,
                    help="Lowest candidate blade-flash fundamental to search (default: 20.0)")
    p.add_argument("--cadence-max-hz", type=float, default=1000.0,
                    help="Highest candidate blade-flash fundamental to search (default: 1000.0)")
    p.add_argument("--cadence-step-hz", type=float, default=2.0,
                    help="Step size when scanning candidate fundamentals (default: 2.0)")
    p.add_argument("--min-harmonics", type=int, default=2,
                    help="Minimum corroborating harmonics required to call it periodic (default: 2)")
    p.add_argument("--max-harmonics", type=int, default=8,
                    help="Cap on harmonics tested per candidate fundamental, to bound false comb matches from broadband noise (default: 8)")
    p.add_argument("--harmonic-snr-floor-db", type=float, default=4.77,
                    help="Per-harmonic power must exceed the sideband noise floor by this many dB to count (default: 4.77 = 3x linear)")

    p.add_argument("--drone-threshold", type=float, default=50.0,
                    help="Confidence (0-100) above which drone=true (default: 50.0)")

    p.add_argument("--cfar-min-power-db", type=float, default=-10.0,
                    help="OS-CFAR minimum accepted power, dB relative to per-bin noise floor scale (default: -10.0)")
    p.add_argument("--min-range-bin", type=int, default=2,
                    help="Ignore range bins below this (near-zero-range self-interference) (default: 2)")
    p.add_argument("--dc-exclusion-bins", type=int, default=3,
                    help="Doppler bins to exclude around zero-Doppler (static clutter) (default: 3)")

    p.add_argument("--track-gate-bins", type=int, default=4,
                    help="Max range-bin jump to keep following the same target frame-to-frame (default: 4)")
    p.add_argument("--track-timeout", type=float, default=2.0,
                    help="Seconds without a detection before dropping the track (default: 2.0)")

    return p.parse_args()


@dataclass
class Track:
    range_bin: int | None = None
    last_seen: float = 0.0
    last_cluster: dict | None = None


@dataclass
class State:
    params: ViewerRuntimeParams | None = None
    params_requested_at: float = 0.0
    frames_processed: int = 0
    track: Track = field(default_factory=Track)
    slow_time_buffer: deque = field(default_factory=lambda: deque(maxlen=1))
    last_emit: float = 0.0
    warned_metadata_sidecar: bool = False


def slow_time_fs(params: ViewerRuntimeParams) -> float:
    if params.sample_rate_hz <= 0 or params.ofdm_fft_size <= 0 or params.sensing_symbol_stride <= 0:
        return 0.0
    symbol_len = params.ofdm_fft_size + params.cp_length
    return params.sample_rate_hz / (params.sensing_symbol_stride * symbol_len)


def range_bin_resolution_m(params: ViewerRuntimeParams, range_fft_size: int) -> float:
    if params.sample_rate_hz <= 0 or params.ofdm_fft_size <= 0:
        return 0.0
    subcarrier_spacing_hz = params.sample_rate_hz / params.ofdm_fft_size
    return C_LIGHT_MPS / (2.0 * subcarrier_spacing_hz * range_fft_size)


def doppler_bin_resolution_hz(params: ViewerRuntimeParams, doppler_fft_size: int) -> float:
    fs = slow_time_fs(params)
    if fs <= 0:
        return 0.0
    return fs / doppler_fft_size


def compute_range_doppler(raw_frame: np.ndarray, params: ViewerRuntimeParams):
    raw_rows, raw_cols = raw_frame.shape
    range_fft_size = max(raw_cols, int(params.range_fft_size))
    doppler_fft_size = max(raw_rows, int(params.doppler_fft_size), MIN_DOPPLER_PROCESSING_FFT_SIZE)

    range_win = np.hamming(raw_cols).astype(np.float32)
    doppler_win = np.hamming(raw_rows).astype(np.float32)

    windowed = raw_frame * range_win[None, :]
    padded = np.zeros((raw_rows, range_fft_size), dtype=np.complex64)
    padded[:, :raw_cols] = windowed
    range_time = np.fft.ifft(padded, axis=1) * range_fft_size

    doppler_windowed = range_time * doppler_win[:, None]
    padded_doppler = np.zeros((doppler_fft_size, range_fft_size), dtype=np.complex64)
    padded_doppler[:raw_rows, :] = doppler_windowed
    doppler_fft = np.fft.fft(padded_doppler, axis=0)
    doppler_shifted = np.fft.fftshift(doppler_fft, axes=0)

    rd_db = 20.0 * np.log10(np.abs(doppler_shifted) + 1e-12).astype(np.float32)
    return rd_db, range_time, range_fft_size, doppler_fft_size


def detect_targets(rd_db: np.ndarray, raw_rows: int, args: argparse.Namespace) -> list[dict]:
    doppler_fft_size = rd_db.shape[0]
    # The Doppler axis is zero-padded well beyond the raw symbol count (see
    # MIN_DOPPLER_PROCESSING_FFT_SIZE), so one physical target's mainlobe now
    # spans multiple interpolated bins. Scale CFAR/cluster suppression to match,
    # or a single target over-segments into many near-duplicate detections.
    oversample = max(1.0, doppler_fft_size / max(1, raw_rows))
    suppress_doppler = max(2, round(2 * oversample))
    cfar_params = OsCfarParams(
        min_range_bin=args.min_range_bin,
        dc_exclusion_bins=args.dc_exclusion_bins,
        min_power_db=args.cfar_min_power_db,
        suppress_doppler=suppress_doppler,
    )
    points, _raw_hits, _shown_hits, _method, _stats = run_os_cfar_2d(
        rd_db, cfar_params, dc_center_row=doppler_fft_size // 2,
    )
    if points.shape[0] == 0:
        return []
    strengths_db = rd_db[points[:, 0], points[:, 1]]
    return cluster_detected_targets(points, rd_db, strengths_db, eps_doppler=suppress_doppler)


def update_track(track: Track, clusters: list[dict], now: float, gate_bins: int, timeout_s: float) -> dict | None:
    chosen = None
    if track.range_bin is not None:
        candidates = [c for c in clusters if abs(c["peak_range_idx"] - track.range_bin) <= gate_bins]
        if candidates:
            chosen = max(candidates, key=lambda c: c["peak_strength_db"])
    if chosen is None and clusters:
        chosen = clusters[0]
    if chosen is not None:
        track.range_bin = int(chosen["peak_range_idx"])
        track.last_seen = now
        track.last_cluster = chosen
        return chosen
    if track.range_bin is not None and (now - track.last_seen) > timeout_s:
        track.range_bin = None
    return None


def stft_single(signal: np.ndarray, fs: float, nperseg: int, noverlap: int):
    step = nperseg - noverlap
    n = len(signal)
    n_frames = 1 + max(0, (n - nperseg) // step)
    if n_frames <= 0:
        return None, None
    window = np.hamming(nperseg).astype(np.float32)
    Zxx = np.empty((nperseg, n_frames), dtype=np.complex64)
    for i in range(n_frames):
        start = i * step
        seg = signal[start:start + nperseg]
        Zxx[:, i] = np.fft.fft(seg * window)
    f = np.fft.fftshift(np.fft.fftfreq(nperseg, d=1.0 / fs))
    power_db = 20.0 * np.log10(np.abs(np.fft.fftshift(Zxx, axes=0)) + 1e-12)
    return f, power_db


def analyze_cadence(f_hz: np.ndarray, power_db: np.ndarray, args: argparse.Namespace) -> dict:
    mean_power_db = power_db.mean(axis=1)
    mean_power_lin = np.power(10.0, mean_power_db / 10.0)

    # The target's own bulk-motion return is a strong line wherever its Doppler
    # shift happens to be -- not necessarily at 0 Hz. Find it and search/exclude
    # relative to that line, not relative to zero, or a moving target's own
    # carrier gets mistaken for periodic sideband content.
    body_idx = int(np.argmax(mean_power_lin))
    f_body = float(f_hz[body_idx])

    sideband_mask = np.abs(f_hz - f_body) >= args.body_exclusion_hz
    if not np.any(sideband_mask):
        return {"fundamental_hz": None, "harmonics": 0, "avg_snr_db": None, "confidence": 0.0, "body_doppler_hz": f_body}

    noise_floor = max(float(np.median(mean_power_lin[sideband_mask])), 1e-12)
    f_min, f_max = float(f_hz.min()), float(f_hz.max())
    bin_width_hz = float(f_hz[1] - f_hz[0]) if len(f_hz) > 1 else 1.0
    tolerance_hz = max(bin_width_hz / 2.0, 1e-6)

    # Only real spectral lines -- local maxima -- count as harmonic evidence.
    # A strong tone's window-sidelobe skirt decays monotonically and is never a
    # local maximum, so this rejects leakage that would otherwise masquerade as
    # widespread "comb" energy at high SNR.
    is_peak = np.zeros_like(mean_power_lin, dtype=bool)
    is_peak[1:-1] = (mean_power_lin[1:-1] > mean_power_lin[:-2]) & (mean_power_lin[1:-1] > mean_power_lin[2:])
    peak_mask = is_peak & sideband_mask
    peak_freqs = f_hz[peak_mask]
    peak_snr_db = 10.0 * np.log10(mean_power_lin[peak_mask] / noise_floor + 1e-12)

    def nearest_peak_snr(freq: float) -> float | None:
        if freq < f_min or freq > f_max or peak_freqs.size == 0:
            return None
        idx = int(np.argmin(np.abs(peak_freqs - freq)))
        if abs(peak_freqs[idx] - freq) <= tolerance_hz:
            return float(peak_snr_db[idx])
        return None

    # A candidate spacing finer than a few STFT bins is statistically
    # meaningless here -- with dozens of noise-driven local maxima scattered
    # every bin or two, an unconstrained fine-grained delta will almost always
    # find *some* nearby peak by chance. Floor the search below this.
    effective_min_hz = max(args.cadence_min_hz, 3.0 * bin_width_hz)

    best = None
    for delta in np.arange(effective_min_hz, args.cadence_max_hz, args.cadence_step_hz):
        n_start = max(1, int(np.ceil(args.body_exclusion_hz / delta)))
        matched = []
        for n in range(n_start, n_start + args.max_harmonics):
            f_hi, f_lo = f_body + n * delta, f_body - n * delta
            if f_hi > f_max and f_lo < f_min:
                break
            snr_hi = nearest_peak_snr(f_hi)
            snr_lo = nearest_peak_snr(f_lo)
            candidates = [s for s in (snr_hi, snr_lo) if s is not None]
            if not candidates:
                continue
            snr_db = max(candidates)
            if snr_db >= args.harmonic_snr_floor_db:
                matched.append((n, snr_db))
        if len(matched) >= args.min_harmonics:
            avg_snr_db = float(np.mean([m[1] for m in matched]))
            score = len(matched) * avg_snr_db
            if best is None or score > best["score"]:
                best = {
                    "fundamental_hz": float(delta),
                    "harmonics": len(matched),
                    "avg_snr_db": avg_snr_db,
                    "score": score,
                }

    if best is None:
        return {"fundamental_hz": None, "harmonics": 0, "avg_snr_db": None, "confidence": 0.0, "body_doppler_hz": f_body}

    confidence = float(np.clip(
        best["avg_snr_db"] * CONFIDENCE_SNR_DB_SCALE
        + (best["harmonics"] - args.min_harmonics) * CONFIDENCE_HARMONIC_BONUS,
        0.0, 100.0,
    ))
    return {
        "fundamental_hz": best["fundamental_hz"],
        "harmonics": best["harmonics"],
        "avg_snr_db": best["avg_snr_db"],
        "confidence": confidence,
        "body_doppler_hz": f_body,
    }


def build_json(now: float, state: State, target: dict | None, cadence: dict, args: argparse.Namespace) -> dict:
    params = state.params
    range_m = None
    velocity_mps = None
    peak_db = None
    if target is not None and params is not None:
        _rd_db, _range_time, range_fft_size, doppler_fft_size = state._last_rd_shapes  # type: ignore[attr-defined]
        bin_res_m = range_bin_resolution_m(params, range_fft_size)
        range_m = float(target["peak_range_idx"] * bin_res_m) if bin_res_m > 0 else None
        doppler_res_hz = doppler_bin_resolution_hz(params, doppler_fft_size)
        if doppler_res_hz > 0 and params.center_freq_hz > 0:
            doppler_hz = (target["peak_doppler_idx"] - doppler_fft_size // 2) * doppler_res_hz
            wavelength_m = C_LIGHT_MPS / params.center_freq_hz
            velocity_mps = float(doppler_hz * wavelength_m / 2.0)
        peak_db = float(target["peak_strength_db"])

    confidence = cadence.get("confidence", 0.0) or 0.0
    fs = slow_time_fs(params) if params is not None else 0.0
    history_s = (len(state.slow_time_buffer) / fs) if fs > 0 else 0.0

    return {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
        "unix_time": now,
        "drone": bool(confidence >= args.drone_threshold),
        "confidence": round(confidence, 1),
        "target_detected": target is not None,
        "range_m": round(range_m, 2) if range_m is not None else None,
        "velocity_mps": round(velocity_mps, 2) if velocity_mps is not None else None,
        "target_peak_db": round(peak_db, 1) if peak_db is not None else None,
        "cadence_fundamental_hz": cadence.get("fundamental_hz"),
        "cadence_harmonics_detected": cadence.get("harmonics", 0),
        "cadence_avg_harmonic_snr_db": (
            round(cadence["avg_snr_db"], 1) if cadence.get("avg_snr_db") is not None else None
        ),
        "body_doppler_hz": (
            round(cadence["body_doppler_hz"], 1) if cadence.get("body_doppler_hz") is not None else None
        ),
        "micro_doppler_history_s": round(history_s, 2),
        "frames_processed": state.frames_processed,
        "params_ready": params is not None,
    }


def main() -> int:
    args = parse_args()

    data_endpoint = make_tcp_endpoint(args.host, args.port)
    control_endpoint = make_tcp_endpoint(args.host, args.control_port)

    data_sock = make_data_sub(data_endpoint, rcvhwm=4)
    control_sock = make_control_dealer(control_endpoint, identity=f"drone-monitor-{os.getpid()}")

    eprint(f"[drone-monitor] data <- {data_endpoint}  control <-> {control_endpoint}")

    poller = zmq.Poller()
    poller.register(data_sock, zmq.POLLIN)
    poller.register(control_sock, zmq.POLLIN)

    state = State()
    state._last_rd_shapes = (None, None, 0, 0)  # type: ignore[attr-defined]

    try:
        while True:
            now = time.monotonic()
            if state.params is None and (now - state.params_requested_at) > 0.5:
                control_sock.send_multipart([build_params_request(0)])
                state.params_requested_at = now

            events = dict(poller.poll(timeout=200))

            if control_sock in events:
                parts = control_sock.recv_multipart(flags=zmq.NOBLOCK)
                data = parts[-1] if parts else b""
                if len(data) >= 8 and data[:4] == CTRL_HEADER:
                    command = data[4:8]
                    if command == PARAMS_COMMAND:
                        parsed = parse_params_packet(data)
                        if parsed is not None:
                            fs = slow_time_fs(parsed)
                            history_len = max(1, int(args.history_seconds * fs)) if fs > 0 else 1
                            state.slow_time_buffer = deque(state.slow_time_buffer, maxlen=history_len)
                            state.params = parsed
                            eprint(f"[drone-monitor] viewer params: {parsed.describe()}  slow_time_fs={fs:.1f}Hz")
                    elif command == READY_COMMAND and state.params is None:
                        control_sock.send_multipart([build_params_request(0)])
                        state.params_requested_at = now

            if data_sock in events and state.params is not None:
                frame = recv_sensing_frame(data_sock, flags=zmq.NOBLOCK)
                if frame is not None:
                    data_payload, metadata_payload = frame
                    if metadata_payload is not None and not state.warned_metadata_sidecar:
                        eprint(
                            "[drone-monitor] backend is sending a metadata sidecar "
                            "(backend_processing_enabled?); this monitor only reads raw "
                            "dense frames and will ignore sidecar-bearing frames"
                        )
                        state.warned_metadata_sidecar = True
                    if data_payload and metadata_payload is None:
                        try:
                            if state.params.aggregated_stream():
                                _fid, channel_frames = decode_aggregate_sensing_payload(
                                    state.frames_processed, data_payload, state.params
                                )
                                decoded = channel_frames[0][1] if channel_frames else None
                            else:
                                decoded = decode_sensing_payload(state.frames_processed, data_payload, state.params)
                        except ValueError as exc:
                            eprint(f"[drone-monitor] frame decode error: {exc}")
                            decoded = None

                        if decoded is not None:
                            state.frames_processed += 1
                            raw_rows = min(decoded.matrix.shape[0], state.params.active_rows)
                            raw_cols = min(decoded.matrix.shape[1], state.params.active_cols)
                            raw_frame = decoded.matrix[:raw_rows, :raw_cols]

                            rd_db, range_time, range_fft_size, doppler_fft_size = compute_range_doppler(
                                raw_frame, state.params
                            )
                            state._last_rd_shapes = (rd_db, range_time, range_fft_size, doppler_fft_size)  # type: ignore[attr-defined]

                            clusters = detect_targets(rd_db, raw_rows, args)
                            target = update_track(
                                state.track, clusters, now, args.track_gate_bins, args.track_timeout
                            )

                            if state.track.range_bin is not None:
                                range_idx = min(state.track.range_bin, range_time.shape[1] - 1)
                                state.slow_time_buffer.extend(range_time[:, range_idx])

            if (now - state.last_emit) >= args.update_interval:
                state.last_emit = now
                target = None
                if (
                    state.track.range_bin is not None
                    and state.track.last_cluster is not None
                    and (now - state.track.last_seen) <= args.track_timeout
                ):
                    target = state.track.last_cluster

                cadence = {"fundamental_hz": None, "harmonics": 0, "avg_snr_db": None, "confidence": 0.0}
                fs = slow_time_fs(state.params) if state.params is not None else 0.0
                if fs > 0 and len(state.slow_time_buffer) >= args.stft_nperseg:
                    signal = np.asarray(state.slow_time_buffer, dtype=np.complex64)
                    f_hz, power_db = stft_single(signal, fs, args.stft_nperseg, args.stft_noverlap)
                    if f_hz is not None:
                        cadence = analyze_cadence(f_hz, power_db, args)

                record = build_json(time.time(), state, target, cadence, args)
                print(json.dumps(record), flush=True)

    except KeyboardInterrupt:
        eprint("[drone-monitor] interrupted, shutting down")
    finally:
        data_sock.close(0)
        control_sock.close(0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
