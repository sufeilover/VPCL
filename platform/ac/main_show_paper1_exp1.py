#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dedicated exp1 AC/G29 client.
Send AC+ACTI state to UDP 5005; receive exp1_done JSON on UDP 56060.
UI retains DATA telemetry Excel and TIMING CSV switches (mutually exclusive).
Run alongside the companion exp1.py. No drift/out result interpretation.
"""
import argparse
import json
import math
import atexit
import csv
import datetime
import os
import socket
import struct
import sys
import threading
import time
import ctypes as C
from collections import deque
from typing import Dict, Any, Tuple, List, Optional

import tkinter as tk
from openpyxl import Workbook

from accardata_to_sim import build_packet_from_live, ACTI_COLS_ORIG, ACTI_LOADED_RADIUS_COLS
from G29_vjoy import main as start_g29
import sim_info_acti
from acti_udp_receiver import (
    open_udp2_receiver as open_acti_receiver,
    PACK_FORMAT as ACTI_PACK_FORMAT,
    PACK_SIZE as ACTI_PACK_SIZE,
    FIELDS as ACTI_FIELDS,
)

# ────────────────────────── 基本节拍与端口 ──────────────────────────
try:
    sys.setswitchinterval(0.001)
except Exception:
    pass

SIM_HZ = 333.0
DT = 1.0 / SIM_HZ
RECV_ADDR = ("0.0.0.0", 56060)
UDP_TARGET: Tuple[str, int] = ("127.0.0.1", 5005)
ACTI_UDP_PORT = 27152
BURST_REPEATS = 1

RESULT_DISPLAY_HOLD_SEC = 0.100
NO_RESULT_SEND_PERIOD_SEC = 0.010
# Allow cold-start/500-frame processing; keep only one pending request.
OUTSTANDING_REQUEST_TIMEOUT_SEC = 10.0

WINDOWS_SCHED_TUNING_DEFAULT = True
WINDOWS_TIMER_RESOLUTION_MS = 1
WINDOWS_PROCESS_PRIORITY_CLASS = 0x00008000  # ABOVE_NORMAL_PRIORITY_CLASS
WINDOWS_COMM_THREAD_PRIORITY = 1  # THREAD_PRIORITY_ABOVE_NORMAL
_windows_sched_tuning_active = False
_windows_timer_resolution_enabled = False

HUD_WINDOW_GEOMETRY = "680x400+40+40"
HUD_CONTROL_FONT = ("Arial", 13)
_shutdown_event = threading.Event()

sim = sim_info_acti.SimInfo()
_udp_sock: socket.socket | None = None

# ────────────────────────── 工具函数 ──────────────────────
def convert_ctypes(val):
    if isinstance(val, C.Array) and getattr(val, "_type_", None) is C.c_char:
        return bytes(val).split(b"\x00", 1)[0].decode("ascii", "ignore")
    if isinstance(val, C.Array):
        return [convert_ctypes(x) for x in val]
    if isinstance(val, C.c_char_p):
        try:
            return (val.value or b"").decode("ascii", "ignore")
        except Exception:
            return ""
    if hasattr(val, "_fields_"):
        return {name: convert_ctypes(getattr(val, name)) for name, _ in val._fields_}
    return val


def _extract_packet_id(d: Dict[str, Any]) -> Optional[int]:
    for k in ("packetId", "packet_id", "id", "packetID"):
        if k in d:
            try:
                return int(d[k])
            except Exception:
                return None
    return None


def _enable_windows_timer_resolution() -> bool:
    global _windows_timer_resolution_enabled
    if os.name != "nt" or _windows_timer_resolution_enabled:
        return False
    try:
        result = C.windll.winmm.timeBeginPeriod(WINDOWS_TIMER_RESOLUTION_MS)
        if result == 0:
            _windows_timer_resolution_enabled = True
            return True
    except Exception:
        pass
    return False


def _disable_windows_timer_resolution() -> None:
    global _windows_timer_resolution_enabled
    if os.name != "nt" or not _windows_timer_resolution_enabled:
        return
    try:
        C.windll.winmm.timeEndPeriod(WINDOWS_TIMER_RESOLUTION_MS)
    except Exception:
        pass
    _windows_timer_resolution_enabled = False


def _set_windows_process_priority() -> bool:
    if os.name != "nt":
        return False
    try:
        kernel32 = C.windll.kernel32
        handle = kernel32.GetCurrentProcess()
        return bool(kernel32.SetPriorityClass(handle, WINDOWS_PROCESS_PRIORITY_CLASS))
    except Exception:
        return False


def _set_windows_current_thread_priority() -> bool:
    if os.name != "nt":
        return False
    try:
        kernel32 = C.windll.kernel32
        handle = kernel32.GetCurrentThread()
        return bool(kernel32.SetThreadPriority(handle, WINDOWS_COMM_THREAD_PRIORITY))
    except Exception:
        return False


def configure_windows_scheduling(enable: bool) -> None:
    global _windows_sched_tuning_active
    if not enable or os.name != "nt":
        _windows_sched_tuning_active = False
        print("[WIN] scheduling tuning disabled")
        return
    _windows_sched_tuning_active = True
    timer_ok = _enable_windows_timer_resolution()
    priority_ok = _set_windows_process_priority()
    if timer_ok:
        atexit.register(_disable_windows_timer_resolution)
    print(
        "[WIN] scheduling tuning: "
        f"timeBeginPeriod({WINDOWS_TIMER_RESOLUTION_MS})={'ok' if timer_ok else 'skip/fail'}, "
        f"process_above_normal={'ok' if priority_ok else 'fail'}"
    )


def init_udp_sender(addr: Tuple[str, int] = UDP_TARGET) -> None:
    global _udp_sock, UDP_TARGET
    UDP_TARGET = addr
    if _udp_sock is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(addr)
        _udp_sock = s


def udp_send_values(values: List[float]) -> int:
    if _udp_sock is None:
        init_udp_sender()
    payload = [float(x) for x in values]
    fmt = "<" + "f" * len(payload)
    data = struct.pack(fmt, *payload)
    try:
        return _udp_sock.send(data)
    except Exception:
        return _udp_sock.sendto(data, UDP_TARGET)


# ────────────────────────── 全局“最新快照” ─────────────────────────
_snap_lock = threading.Lock()
_latest_ac: Dict[str, Any] = {}
_latest_ac_udp: Dict[str, Any] = {}

_acti_cache_lock = threading.Lock()
_acti_cache: deque = deque(maxlen=64)

# ────────────────────────── 结果接收 / UI / 调度状态 ─────────────────────────
_comm_lock = threading.Lock()
_last_completion: Dict[str, Any] = {}
_last_result_generation: int = 0
_last_result_time: Optional[float] = None
_next_send_due_time: Optional[float] = None
_last_send_time: float = 0.0

_request_lock = threading.Lock()
_next_request_id: int = 0
_pending_requests: Dict[int, Dict[str, Any]] = {}

_timing_log_lock = threading.Lock()
_timing_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "timing_logs")
_timing_client_csv = os.path.join(
    _timing_log_dir,
    f"timing_client_exp1_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
)
_timing_client_fields = [
    "request_id",
    "reason",
    "payload_floats",
    "bytes_sent",
    "client_build_ms",
    "client_send_call_ms",
    "client_send_start_to_recv_ms",
    "client_send_end_to_recv_ms",
    "result_parse_ms",
    "optimizer_processing_ms",
    "prediction_steps",
    "round_idx",
    "rollout_with_logging_ms",
    "optimizer_total_ms",
    "state_csv_enabled",
]
_timing_client_rows: List[Dict[str, Any]] = []
_timing_client_flushed: int = 0
_timing_enabled_event = threading.Event()
# _timing_enabled_event.set() #timing on


_ui_enabled_event = threading.Event()
_ui_enabled_event.set()

_log_enabled_event = threading.Event()
_log_enabled_event.clear()
_log_lock = threading.Lock()
_log_records: List[tuple] = []
_log_out_xlsx: Optional[str] = None
_log_t0: float = 0.0
_log_last_pid: int = -1


def ui_is_enabled() -> bool:
    return _ui_enabled_event.is_set()


def ui_set_enabled(enabled: bool) -> None:
    if enabled:
        _ui_enabled_event.set()
    else:
        _ui_enabled_event.clear()


def log_is_enabled() -> bool:
    return _log_enabled_event.is_set()


def log_set_enabled(enabled: bool, out_xlsx: Optional[str] = None) -> None:
    global _log_out_xlsx, _log_t0, _log_last_pid
    if enabled:
        _stop_timing_and_flush("telemetry logging enabled")
        with _log_lock:
            _log_records.clear()
        _log_last_pid = -1
        _log_t0 = time.perf_counter()
        if out_xlsx is None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_xlsx = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"telemetry_exp1_{ts}.xlsx")
        _log_out_xlsx = out_xlsx
        _log_enabled_event.set()
        print("[LOG] enabled ->", _log_out_xlsx)
    else:
        _log_enabled_event.clear()
        if _log_out_xlsx:
            flush_log_to_excel(_log_out_xlsx)


def _alloc_request_id() -> int:
    global _next_request_id
    with _request_lock:
        _next_request_id = (_next_request_id + 1) % 10000000
        if _next_request_id == 0:
            _next_request_id = 1
        return _next_request_id


def _remember_pending_request(request_id: int, meta: Dict[str, Any]) -> None:
    with _request_lock:
        _pending_requests[request_id] = meta
        if len(_pending_requests) > 2000:
            for old_id in sorted(_pending_requests)[:1000]:
                _pending_requests.pop(old_id, None)


def _update_pending_request(request_id: int, meta: Dict[str, Any]) -> None:
    with _request_lock:
        if request_id in _pending_requests:
            _pending_requests[request_id].update(meta)


def _expire_stale_pending_requests(now: Optional[float] = None) -> None:
    if now is None:
        now = time.perf_counter()
    with _request_lock:
        stale_ids = []
        for request_id, meta in _pending_requests.items():
            send_start = meta.get("send_start")
            if isinstance(send_start, (int, float)) and (now - send_start) >= OUTSTANDING_REQUEST_TIMEOUT_SEC:
                stale_ids.append(request_id)
        for request_id in stale_ids:
            _pending_requests.pop(request_id, None)
            print(f"[EXP1] request={request_id} timed out after {OUTSTANDING_REQUEST_TIMEOUT_SEC:g}s; check exp1.py/UDP 5005 and 56060", flush=True)


def _has_active_pending_request(now: Optional[float] = None) -> bool:
    _expire_stale_pending_requests(now)
    with _request_lock:
        return bool(_pending_requests)


def _pop_pending_request(request_id: Optional[int]) -> Optional[Dict[str, Any]]:
    if request_id is None:
        return None
    with _request_lock:
        return _pending_requests.pop(request_id, None)


def _write_client_timing_row(row: Dict[str, Any]) -> None:
    if not timing_is_enabled():
        return
    with _timing_log_lock:
        if not timing_is_enabled():
            return
        _timing_client_rows.append(dict(row))
    # Receive timestamp was captured before this disk I/O.
    try:
        _flush_client_timing_rows()
    except OSError as exc:
        _timing_enabled_event.clear()
        print(f"[TIMING] write failed; recording stopped, unsaved rows retained: {exc}", flush=True)


def _flush_client_timing_rows(create_file: bool = False) -> None:
    global _timing_client_flushed
    with _timing_log_lock:
        rows = _timing_client_rows[_timing_client_flushed:]
        if not rows and not create_file:
            return
        os.makedirs(_timing_log_dir, exist_ok=True)
        need_header = not os.path.exists(_timing_client_csv) or os.path.getsize(_timing_client_csv) == 0
        with open(_timing_client_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=_timing_client_fields, extrasaction="ignore")
            if need_header:
                writer.writeheader()
            writer.writerows(rows)
        _timing_client_flushed += len(rows)
        # Completed rows are on disk; don't grow memory during long experiments.
        _timing_client_rows.clear()
        _timing_client_flushed = 0


def _stop_timing_and_flush(reason: str = "") -> None:
    _timing_enabled_event.clear()
    try:
        _flush_client_timing_rows()
        print(f"[TIMING] stopped ({reason}); saved rows are in {_timing_client_csv}", flush=True)
    except OSError as exc:
        print(f"[TIMING] flush FAILED: {exc}", flush=True)


def timing_is_enabled() -> bool:
    return _timing_enabled_event.is_set()


def timing_set_enabled(enabled: bool, reason: str = "") -> bool:
    if enabled:
        if log_is_enabled():
            print("[TIMING] disable DATA first (same policy as original client)")
            return False
        try:
            _flush_client_timing_rows(create_file=True)
        except OSError as exc:
            print(f"[TIMING] cannot create CSV: {exc}", flush=True)
            return False
        _timing_enabled_event.set()
        print(f"[TIMING] enabled ({reason}) -> {_timing_client_csv}", flush=True)
        return True
    _stop_timing_and_flush(reason)
    return False


atexit.register(_flush_client_timing_rows)


def thread_g29_vjoy():
    try:
        start_g29()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[G29] G29 -> vJoy bridge stopped: {exc}", flush=True)


def get_latest_ac() -> Dict[str, Any]:
    with _snap_lock:
        return dict(_latest_ac) if _latest_ac else {}


def get_latest_ac_udp() -> Dict[str, Any]:
    with _snap_lock:
        return dict(_latest_ac_udp) if _latest_ac_udp else {}


def _select_best_acti_from_cache(ac_pid: Optional[int]) -> Optional[Dict[str, Any]]:
    with _acti_cache_lock:
        if not _acti_cache:
            return None
        if ac_pid is None:
            return _acti_cache[-1]
        scored: List[tuple[int, int, Dict[str, Any]]] = []
        for idx, pkt in enumerate(_acti_cache):
            pid = _extract_packet_id(pkt)
            if pid is None:
                continue
            scored.append((abs(pid - ac_pid), idx, pkt))
        if scored:
            scored.sort(key=lambda x: (x[0], x[1]))
            return scored[0][2]
        return _acti_cache[-1]


def _ms_to_s(ms: int) -> float:
    return float(ms) / 1000.0


def _pick_acti_exact_pid(ac_pid: int) -> Optional[Dict[str, Any]]:
    with _acti_cache_lock:
        for pkt in reversed(_acti_cache):
            pid = _extract_packet_id(pkt)
            if pid == ac_pid:
                return pkt
    return None


# ────────────────────────── 数据读取线程 ─────────────────────────
def thread_ac_sharedmemory(poll_interval: float = 0.0):
    local_sim = sim_info_acti.SimInfo()
    print("[AC] Attached to Assetto Corsa shared memory.")
    try:
        while True:
            physics = local_sim.physics
            packet_id = getattr(physics, "packetId", None)
            static = local_sim.static
            graphics = local_sim.graphics
            t0 = time.perf_counter()

            static_dict = {n: convert_ctypes(getattr(static, n)) for n, _ in type(static)._fields_}
            physics_dict = {n: convert_ctypes(getattr(physics, n)) for n, _ in type(physics)._fields_}
            graphics_dict = {n: convert_ctypes(getattr(graphics, n)) for n, _ in type(graphics)._fields_}

            with _snap_lock:
                _latest_ac.clear()
                _latest_ac.update({
                    "packetId": int(packet_id) if packet_id is not None else None,
                    "ts": t0,
                    "physics": physics_dict,
                    "graphics": graphics_dict,
                    "static": static_dict,
                })

            if poll_interval:
                time.sleep(poll_interval)
    except KeyboardInterrupt:
        if log_is_enabled() and _log_out_xlsx:
            flush_log_to_excel(_log_out_xlsx)
        return


def thread_acti_udp():
    sock = open_acti_receiver(port=ACTI_UDP_PORT, timeout=1.0)
    print(f"[ACTI] listening UDP:{ACTI_UDP_PORT}, expect {ACTI_PACK_SIZE} bytes...")
    try:
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < ACTI_PACK_SIZE:
                continue
            try:
                values = struct.unpack(ACTI_PACK_FORMAT, data[:ACTI_PACK_SIZE])
            except struct.error:
                continue

            pkt = dict(zip(ACTI_FIELDS, values))
            with _snap_lock:
                _latest_ac_udp.clear()
                _latest_ac_udp.update(pkt)
            with _acti_cache_lock:
                _acti_cache.append(pkt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass


# ────────────────────────── 构造并发送状态包 ─────────────────────────
def read_one_ac_snapshot() -> Dict[str, Any]:
    try:
        physics = sim.physics
        graphics = sim.graphics
        static = sim.static
        return {
            "packetId": int(getattr(physics, "packetId", -1)),
            "physics": {n: convert_ctypes(getattr(physics, n)) for n, _ in type(physics)._fields_},
            "graphics": {n: convert_ctypes(getattr(graphics, n)) for n, _ in type(graphics)._fields_},
            "static": {n: convert_ctypes(getattr(static, n)) for n, _ in type(static)._fields_},
        }
    except Exception:
        return {}


def build_values_once_with_pid_match() -> List[float]:
    ac = read_one_ac_snapshot()
    ac_pid = _extract_packet_id(ac)
    acti_sel = _select_best_acti_from_cache(ac_pid)

    phys = ac.get("physics", {})
    grap = ac.get("graphics", {})

    if acti_sel is None:
        acti_sel = {k: float("nan") for k in (ACTI_COLS_ORIG + ACTI_LOADED_RADIUS_COLS + ["time"])}

    try:
        values, _ = build_packet_from_live(
            physics=phys,
            graphics=grap,
            acti=acti_sel,
            include_times=True,
            trigger_hotstart=True,
            include_calc=True,
        )
    except KeyError:
        acti_fallback = {k: float("nan") for k in (ACTI_COLS_ORIG + ACTI_LOADED_RADIUS_COLS + ["time"])}
        values, _ = build_packet_from_live(
            physics=phys,
            graphics=grap,
            acti=acti_fallback,
            include_times=True,
            trigger_hotstart=True,
            include_calc=True,
        )
    return values


def send_state_once(reason: str = "") -> bool:
    global _last_send_time
    if _has_active_pending_request():
        return False
    request_id = _alloc_request_id()
    build_start = time.perf_counter()
    values = build_values_once_with_pid_match()
    build_end = time.perf_counter()
    values = list(values)
    if values:
        values.insert(len(values) - 1, float(request_id))
    send_start = time.perf_counter()
    _remember_pending_request(
        request_id,
        {
            "request_id": request_id,
            "reason": reason,
            "payload_floats": len(values),
            "bytes_sent": "",
            "build_start": build_start,
            "build_end": build_end,
            "send_start": send_start,
            "send_end": None,
        },
    )
    try:
        bytes_sent = 0
        for _ in range(BURST_REPEATS):
            bytes_sent = udp_send_values(values)
    except Exception:
        _pop_pending_request(request_id)
        print("[EXP1] state send failed; will retry", flush=True)
        return False
    send_end = time.perf_counter()
    _last_send_time = send_end
    _update_pending_request(request_id, {"bytes_sent": bytes_sent, "send_end": send_end})
    # if reason:
    #     print(f"[SEND] {reason}")
    return True


# ────────────────────────── exp1 completion replies ─────────────────────────
def handle_exp1_completion(data: bytes, recv_perf: float) -> bool:
    global _last_completion, _last_result_generation, _last_result_time, _next_send_due_time
    parse_start = time.perf_counter()
    try:
        msg = json.loads(data.decode("utf-8"))
        if not isinstance(msg, dict) or msg.get("type") != "exp1_done" or msg.get("version") != 1:
            return False
        request_id = msg["request_id"]
        if type(request_id) is not int or request_id <= 0:
            return False
        for key in ("optimizer_processing_ms", "optimizer_total_ms", "rollout_with_logging_ms"):
            value = float(msg[key])
            if not math.isfinite(value) or value < 0:
                return False
        if type(msg["prediction_steps"]) is not int or msg["prediction_steps"] <= 0:
            return False
    except (ValueError, TypeError, KeyError, UnicodeError):
        return False
    parse_end = time.perf_counter()
    pending = _pop_pending_request(request_id)
    if pending is None:
        # Ignore duplicate, expired or unrelated replies; never pair with another request.
        return False
    row = {
        "request_id": request_id,
        "reason": pending["reason"],
        "payload_floats": pending["payload_floats"],
        "bytes_sent": pending.get("bytes_sent", ""),
        "client_build_ms": (pending["build_end"] - pending["build_start"]) * 1000.0,
        "client_send_start_to_recv_ms": (recv_perf - pending["send_start"]) * 1000.0,
        "result_parse_ms": (parse_end - parse_start) * 1000.0,
    }
    if isinstance(pending.get("send_end"), (int, float)):
        row["client_send_call_ms"] = (pending["send_end"] - pending["send_start"]) * 1000.0
        row["client_send_end_to_recv_ms"] = (recv_perf - pending["send_end"]) * 1000.0
    for key in ("optimizer_processing_ms", "optimizer_total_ms", "rollout_with_logging_ms",
                "prediction_steps", "round_idx", "state_csv_enabled"):
        row[key] = msg.get(key, "")
    with _comm_lock:
        _last_completion = dict(row)
        _last_result_generation += 1
        _last_result_time = recv_perf
        _next_send_due_time = recv_perf + RESULT_DISPLAY_HOLD_SEC
    _write_client_timing_row(row)
    # print(
    #     f"[EXP1] request={request_id} steps={row['prediction_steps']} "
    #     f"processing_ms={float(row['optimizer_processing_ms']):.3f} "
    #     f"rollout_ms={float(row['rollout_with_logging_ms']):.3f} "
    #     f"roundtrip_ms={row['client_send_start_to_recv_ms']:.3f}", flush=True
    # )
    return True


class Exp1CompletionReceiver:
    def __init__(self, addr=RECV_ADDR):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(addr)
        self.sock.settimeout(0.5)

    def poll(self):
        try:
            data, _ = self.sock.recvfrom(65535)
            recv_perf = time.perf_counter()
        except socket.timeout:
            return
        handle_exp1_completion(data, recv_perf)


def thread_result_receiver(recv: Exp1CompletionReceiver):
    if _windows_sched_tuning_active:
        _set_windows_current_thread_priority()
    try:
        while not _shutdown_event.is_set():
            recv.poll()
    finally:
        recv.sock.close()


# ────────────────────────── 发送调度线程 ─────────────────────────
def thread_send_scheduler():
    if _windows_sched_tuning_active:
        _set_windows_current_thread_priority()
    while not _shutdown_event.is_set():
        now = time.perf_counter()
        with _comm_lock:
            due = _next_send_due_time
        if (due is None or now >= due) and not _has_active_pending_request(now):
            try:
                send_state_once(reason="exp1-completion-paced")
            except Exception as exc:
                print(f"[EXP1] cannot build/send state: {exc}", flush=True)
                _shutdown_event.wait(1.0)
        _shutdown_event.wait(NO_RESULT_SEND_PERIOD_SEC)


# ────────────────────────── 日志线程 ─────────────────────────
def flush_log_to_excel(out_xlsx: str) -> None:
    with _log_lock:
        recs = list(_log_records)
    if not recs:
        print("[LOG] no samples, skip saving.")
        return

    wb = Workbook()
    ws = wb.active
    ws.title = "telemetry"
    ws.append([
        "t_sec", "packetId",
        "x", "y", "z", "gas", "brake", "steerAngle", "heading", "pitch", "roll",
        "speedKmh",
        "completedLaps",
        "iCurrentTime_s", "iLastTime_s", "iBestTime_s", "normalizedCarPosition", "currentSectorIndex", "lastSectorTime",
        "wheelSlip_FL", "wheelSlip_FR", "wheelSlip_RL", "wheelSlip_RR",
        "udp_pid_used",
        "SlipAngle_FL", "SlipAngle_FR", "SlipAngle_RL", "SlipAngle_RR",
        "NdSlip_FL", "NdSlip_FR", "NdSlip_RL", "NdSlip_RR",
    ])
    for r in recs:
        ws.append(r)
    ws.freeze_panes = "A2"
    wb.save(out_xlsx)
    print(f"[LOG] saved: {out_xlsx} (samples={len(recs)})")


def thread_telemetry_logger(poll_dt: float = 0.0, fallback_to_latest_udp: bool = True) -> None:
    global _log_last_pid
    last_udp_pid: Optional[int] = None
    last_slip4 = (None, None, None, None)
    last_nd4 = (None, None, None, None)

    while True:
        if not log_is_enabled():
            time.sleep(0.05)
            continue

        ac = get_latest_ac()
        pid = ac.get("packetId", None)
        if pid is None:
            time.sleep(0.01)
            continue
        pid = int(pid)

        if pid == _log_last_pid:
            continue
        _log_last_pid = pid

        t_sec = time.perf_counter() - float(_log_t0)
        phys = ac.get("physics", {}) or {}
        grap = ac.get("graphics", {}) or {}

        latest_acti = get_latest_ac_udp()
        if latest_acti:
            try:
                last_udp_pid = int(latest_acti.get("packetId"))
                last_slip4 = (
                    float(latest_acti.get("SlipAngle_FL")),
                    float(latest_acti.get("SlipAngle_FR")),
                    float(latest_acti.get("SlipAngle_RL")),
                    float(latest_acti.get("SlipAngle_RR")),
                )
                last_nd4 = (
                    float(latest_acti.get("NdSlip_FL")),
                    float(latest_acti.get("NdSlip_FR")),
                    float(latest_acti.get("NdSlip_RL")),
                    float(latest_acti.get("NdSlip_RR")),
                )
            except Exception:
                pass

        x, y, z = (None, None, None)
        try:
            xyz = grap.get("carCoordinates", None)
            if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
                x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        except Exception:
            pass

        speed_kmh = float(phys.get("speedKmh", float("nan")))
        gas = phys.get("gas", 0)
        brake = phys.get("brake", 0)
        steerAngle = phys.get("steerAngle", 0)
        heading = phys.get("heading", 0)
        pitch = phys.get("pitch", 0)
        roll = phys.get("roll", 0)

        ws_fl = ws_fr = ws_rl = ws_rr = None
        try:
            ws = phys.get("wheelSlip", None)
            if isinstance(ws, (list, tuple)) and len(ws) >= 4:
                ws_fl, ws_fr, ws_rl, ws_rr = map(float, ws[:4])
        except Exception:
            pass

        completed_laps = int(grap.get("completedLaps", -1))
        i_cur_s = _ms_to_s(int(grap.get("iCurrentTime", 0)))
        i_last_s = _ms_to_s(int(grap.get("iLastTime", 0)))
        i_best_s = _ms_to_s(int(grap.get("iBestTime", 0)))
        normalizedCarPosition = grap.get("normalizedCarPosition", 0)
        currentSectorIndex = grap.get("currentSectorIndex", 0)
        lastSectorTime = grap.get("lastSectorTime", 0)

        udp_pid_used = None
        slip4 = (None, None, None, None)
        nd4 = (None, None, None, None)

        pkt = _pick_acti_exact_pid(pid)
        if pkt is not None:
            udp_pid_used = pid
            try:
                slip4 = (
                    float(pkt.get("SlipAngle_FL")),
                    float(pkt.get("SlipAngle_FR")),
                    float(pkt.get("SlipAngle_RL")),
                    float(pkt.get("SlipAngle_RR")),
                )
                nd4 = (
                    float(pkt.get("NdSlip_FL")),
                    float(pkt.get("NdSlip_FR")),
                    float(pkt.get("NdSlip_RL")),
                    float(pkt.get("NdSlip_RR")),
                )
                last_udp_pid = udp_pid_used
                last_slip4, last_nd4 = slip4, nd4
            except Exception:
                pass
        elif fallback_to_latest_udp and last_udp_pid is not None:
            udp_pid_used = int(last_udp_pid)
            slip4, nd4 = last_slip4, last_nd4

        row = (
            float(t_sec), pid,
            x, y, z, gas, brake, steerAngle, heading, pitch, roll,
            speed_kmh,
            completed_laps,
            float(i_cur_s), float(i_last_s), float(i_best_s), float(normalizedCarPosition), float(currentSectorIndex), float(lastSectorTime),
            ws_fl, ws_fr, ws_rl, ws_rr,
            udp_pid_used,
            slip4[0], slip4[1], slip4[2], slip4[3],
            nd4[0], nd4[1], nd4[2], nd4[3],
        )
        with _log_lock:
            _log_records.append(row)
        if poll_dt:
            time.sleep(poll_dt)


# ────────────────────────── exp1 UI (no drift/out matrix) ─────────────────────────
def gui_thread():
    root = tk.Tk()
    root.title("Exp1 — DATA / TIMING")
    root.geometry(HUD_WINDOW_GEOMETRY)
    try:
        root.attributes("-topmost", True)
    except Exception:
        pass
    var_log = tk.BooleanVar(value=log_is_enabled())
    var_timing = tk.BooleanVar(value=timing_is_enabled())
    # status = tk.StringVar(value="Waiting for exp1.py completion reply...")
    # paths = tk.StringVar()
    top = tk.Frame(root)
    top.pack(fill="x", padx=16, pady=14)

    def apply_data():
        try:
            log_set_enabled(bool(var_log.get()))
        except Exception as exc:
            status.set(f"DATA save failed: {exc}")
            print(f"[DATA] {exc}", flush=True)
        var_log.set(log_is_enabled())
        var_timing.set(timing_is_enabled())

    def apply_timing():
        var_timing.set(timing_set_enabled(bool(var_timing.get()), "UI"))

    tk.Checkbutton(top, text="TIMING", font=HUD_CONTROL_FONT, variable=var_timing,
                   command=apply_timing).pack(side="left", padx=(0, 24))
    tk.Checkbutton(top, text="DATA", font=HUD_CONTROL_FONT, variable=var_log,
                   command=apply_data).pack(side="left")
    tk.Label(root, text="DATA: telemetry Excel  |  TIMING: per-request CSV\n"
             "DATA and TIMING are mutually exclusive, as in the original client.",
             justify="left", anchor="w").pack(fill="x", padx=16)
    # tk.Label(root, textvariable=status, justify="left", anchor="w",
    #          font=("Arial", 12), wraplength=640).pack(fill="x", padx=16, pady=18)
    # tk.Label(root, textvariable=paths, justify="left", anchor="w",
    #          wraplength=640).pack(fill="x", padx=16)

    # def update():
    #     with _comm_lock:
    #         row = dict(_last_completion)
    #         last_rx = _last_result_time
    #     var_log.set(log_is_enabled())
    #     var_timing.set(timing_is_enabled())
    #     if row:
    #         status.set(
    #             f"Request {row['request_id']} | {row['prediction_steps']} frames\n"
    #             f"Processing: {float(row['optimizer_processing_ms']):.3f} ms\n"
    #             f"Round trip: {row['client_send_start_to_recv_ms']:.3f} ms\n"
    #             f"Last reply: {time.perf_counter() - last_rx:.1f} s ago"
    #         )
    #     paths.set(f"TIMING: {_timing_client_csv}\nDATA: {_log_out_xlsx or '(enable DATA to create a session)'}")
    #     if _shutdown_event.is_set():
    #         root.destroy()
    #         return
    #     root.after(200, update)

    def close():
        _shutdown_event.set()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", close)
    # update()
    root.mainloop()


# ────────────────────────── 主函数 ─────────────────────────
def main():
    init_udp_sender(("127.0.0.1", 5005))

    parser = argparse.ArgumentParser()
    parser.add_argument("--ui", choices=["on", "off"], default="on", help="HUD UI 开关")
    parser.add_argument(
        "--win-sched-tuning",
        choices=["on", "off"],
        default="on" if WINDOWS_SCHED_TUNING_DEFAULT else "off",
        help="Windows timer/priority tuning for communication latency",
    )
    parser.add_argument("--timing", choices=["on", "off"], default="off")
    args = parser.parse_args()

    configure_windows_scheduling(args.win_sched_tuning == "on")

    print(f"[MAIN] UDP listen {RECV_ADDR}, send to {UDP_TARGET}")
    if args.timing == "on":
        timing_set_enabled(True, "CLI")
    print("[MAIN] Use the companion exp1.py. DATA/TIMING are off by default.")

    t_ac = threading.Thread(target=thread_ac_sharedmemory, daemon=True)
    t_acti = threading.Thread(target=thread_acti_udp, daemon=True)
    t_recv = threading.Thread(target=thread_result_receiver, args=(Exp1CompletionReceiver(),), daemon=True)
    t_sched = threading.Thread(target=thread_send_scheduler, daemon=True)
    t_log = threading.Thread(target=thread_telemetry_logger, kwargs={"fallback_to_latest_udp": True}, daemon=True)
    t_g29 = threading.Thread(target=thread_g29_vjoy, daemon=True)

    t_ac.start()
    t_acti.start()
    t_recv.start()
    t_sched.start()
    t_log.start()
    t_g29.start()

    print("[MAIN] AC thread started.")
    print("[MAIN] ACTI thread started.")
    print("[MAIN] Result receiver started.")
    print("[MAIN] Send scheduler started.")
    print("[MAIN] Telemetry logger thread started (toggle via GUI: DATA).")
    print("[MAIN] G29 -> vJoy thread started.")

    try:
        if args.ui == "on":
            gui_thread()  # Tk stays on the main thread.
        else:
            while not _shutdown_event.wait(1.0):
                pass
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_event.set()
        t_sched.join(timeout=2.0)
        t_recv.join(timeout=2.0)
        try:
            if log_is_enabled():
                log_set_enabled(False)
        finally:
            _stop_timing_and_flush("exit")
            _disable_windows_timer_resolution()


if __name__ == "__main__":
    main()
