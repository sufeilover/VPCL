#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplified tester for optimizer_samplenoview_serial_abs.py
- Sends current AC+ACTI state packets to UDP 5005
- Receives 3x2 result matrix (9 float32, row-major) from UDP 56060
- Shows the result as a 3x2 dot matrix UI
- Timing logic:
    * If no result has been received yet, send every 10 ms
    * When a result is received, show it immediately and schedule one send 100 ms later
    * After that send, if no newer result arrives, fall back to sending every 10 ms again
"""
import argparse
import datetime
import os
import socket
import struct
import threading
import time
import ctypes as C
from collections import deque
from typing import Dict, Any, Tuple, List, Optional

import tkinter as tk
from openpyxl import Workbook

from accardata_to_sim import build_packet_from_live, ACTI_COLS_ORIG, ACTI_LOADED_RADIUS_COLS
import sim_info_acti
from acti_udp_receiver import (
    open_udp2_receiver as open_acti_receiver,
    PACK_FORMAT as ACTI_PACK_FORMAT,
    PACK_SIZE as ACTI_PACK_SIZE,
    FIELDS as ACTI_FIELDS,
)

# ────────────────────────── 基本节拍与端口 ──────────────────────────
SIM_HZ = 333.0
DT = 1.0 / SIM_HZ
RECV_ADDR = ("0.0.0.0", 56060)
UDP_TARGET: Tuple[str, int] = ("127.0.0.1", 5005)
ACTI_UDP_PORT = 27152
BURST_REPEATS = 1

RESULT_PACKET_FLOATS = 6
RESULT_PACKET_SIZE = RESULT_PACKET_FLOATS * 4
RESULT_DISPLAY_HOLD_SEC = 0.100
NO_RESULT_SEND_PERIOD_SEC = 0.010

ROW_LABELS = ["100", "200", "300"]
COL_LABELS = ["OUT", "DRIFT"]

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
_last_result_matrix: List[List[int]] = [[0, 0], [0, 0], [0, 0]]
_last_result_generation: int = 0
_last_result_time: Optional[float] = None
_next_send_due_time: Optional[float] = None
_last_send_time: float = 0.0

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
        with _log_lock:
            _log_records.clear()
        _log_last_pid = -1
        _log_t0 = time.perf_counter()
        if out_xlsx is None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_xlsx = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"telemetry_{ts}.xlsx")
        _log_out_xlsx = out_xlsx
        _log_enabled_event.set()
        print("[LOG] enabled ->", _log_out_xlsx)
    else:
        _log_enabled_event.clear()
        if _log_out_xlsx:
            flush_log_to_excel(_log_out_xlsx)


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


def send_state_once(reason: str = "") -> None:
    global _last_send_time
    values = build_values_once_with_pid_match()
    for _ in range(BURST_REPEATS):
        udp_send_values(values)
    _last_send_time = time.perf_counter()
    # if reason:
    #     print(f"[SEND] {reason}")


# ────────────────────────── 接收 optimizer 3x2 结果矩阵 ─────────────────────────
class ResultMatrixReceiver:
    def __init__(self, addr=RECV_ADDR):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(addr)
        self.sock.setblocking(False)

    def poll(self) -> int:
        received = 0
        while True:
            try:
                data, src = self.sock.recvfrom(65535)
            except BlockingIOError:
                break

            if len(data) < RESULT_PACKET_SIZE:
                continue

            try:
                vals = struct.unpack("<6f", data[:RESULT_PACKET_SIZE])
            except struct.error:
                continue

            matrix = []
            for r in range(3):
                row = []
                for c in range(2):
                    row.append(1 if float(vals[r * 2 + c]) > 0.5 else 0)
                matrix.append(row)

            now = time.perf_counter()
            with _comm_lock:
                global _last_result_matrix, _last_result_generation, _last_result_time, _next_send_due_time
                _last_result_matrix = matrix
                _last_result_generation += 1
                _last_result_time = now
                _next_send_due_time = now + RESULT_DISPLAY_HOLD_SEC
                gen = _last_result_generation

            print(f"[RECV] from {src} gen={gen} matrix={matrix}")
            received += 1
        return received


def thread_result_receiver(recv: ResultMatrixReceiver):
    try:
        while True:
            recv.poll()
            time.sleep(0.001)
    except KeyboardInterrupt:
        return


# ────────────────────────── 发送调度线程 ─────────────────────────
def thread_send_scheduler():
    global _next_send_due_time
    """
    Timing policy:
    - If no result has ever been received, send every 10ms.
    - When a new result is received, show it immediately; schedule one send at t_rx + 100ms.
    - After that scheduled send, if no newer result arrives, fall back to 10ms periodic sending again.
    """
    last_processed_generation = 0
    last_fallback_send = 0.0

    try:
        while True:
            now = time.perf_counter()
            should_send = False
            reason = ""

            with _comm_lock:
                gen = _last_result_generation
                last_rx = _last_result_time
                due = _next_send_due_time

            if gen > last_processed_generation:
                # Fresh result received; wait until due time.
                if due is not None and now >= due:
                    should_send = True
                    reason = f"scheduled-after-result gen={gen}"
                    last_processed_generation = gen
                    with _comm_lock:
                        _next_send_due_time = None
            else:
                # No fresh result pending. If no result has ever arrived, or no newer result has arrived
                # since the last send, keep polling the optimizer every 10ms.
                no_result_yet = (last_rx is None)
                no_new_result_since_last_send = (last_rx is not None and _last_send_time >= last_rx)
                if (no_result_yet or no_new_result_since_last_send) and (now - last_fallback_send >= NO_RESULT_SEND_PERIOD_SEC):
                    should_send = True
                    reason = "fallback-10ms"
                    last_fallback_send = now

            if should_send:
                send_state_once(reason=reason)
            else:
                time.sleep(0.001)
    except KeyboardInterrupt:
        return


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


# ────────────────────────── 3x2 点阵 UI ─────────────────────────
def gui_thread():
    root = tk.Tk()
    root.title("Optimizer Matrix Viewer")
    try:
        root.attributes("-topmost", True)
    except Exception:
        pass
    root.geometry("320x280+40+40")
    root.resizable(False, False)

    top = tk.Frame(root)
    top.pack(fill="x", padx=10, pady=10)

    var_on = tk.BooleanVar(value=True)
    var_log = tk.BooleanVar(value=False)

    def _apply_toggle():
        ui_set_enabled(bool(var_on.get()))
        if ui_is_enabled():
            matrix_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        else:
            matrix_frame.pack_forget()

    def _apply_log_toggle():
        on = bool(var_log.get())
        if on:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_xlsx = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"telemetry_{ts}.xlsx")
            log_set_enabled(True, out_xlsx=out_xlsx)
        else:
            log_set_enabled(False)

    tk.Checkbutton(top, text="HUD", variable=var_on, command=_apply_toggle).pack(side="left", padx=(0, 12))
    tk.Checkbutton(top, text="DATA", variable=var_log, command=_apply_log_toggle).pack(side="left")

    # status_var = tk.StringVar(value="waiting result...")
    # tk.Label(root, textvariable=status_var, anchor="w").pack(fill="x", padx=10, pady=(0, 6))

    matrix_frame = tk.Frame(root)
    matrix_frame.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    dot_labels: List[List[tk.Label]] = []

    tk.Label(matrix_frame, text="").grid(row=0, column=0, padx=6, pady=6)
    for c, col_name in enumerate(COL_LABELS, start=1):
        tk.Label(matrix_frame, text=col_name, width=7).grid(row=0, column=c, padx=6, pady=6)

    for r, row_name in enumerate(ROW_LABELS, start=1):
        tk.Label(matrix_frame, text=row_name, width=5).grid(row=r, column=0, padx=6, pady=6)
        row_widgets: List[tk.Label] = []
        for c in range(1, 3):
            lbl = tk.Label(matrix_frame, text="●", font=("Arial", 26), width=2, fg="#bfbfbf")
            lbl.grid(row=r, column=c, padx=8, pady=8)
            row_widgets.append(lbl)
        dot_labels.append(row_widgets)

    def _update():
        with _comm_lock:
            matrix = [row[:] for row in _last_result_matrix]
            gen = _last_result_generation
            last_rx = _last_result_time
            due = _next_send_due_time

        if last_rx is None:
            pass
            # status_var.set("waiting result... fallback send every 10ms")
        else:
            remain_ms = 0.0 if due is None else max(0.0, (due - time.perf_counter()) * 1000.0)
            # status_var.set(f"gen={gen}  next send in {remain_ms:.0f} ms")

        if ui_is_enabled():
            for r in range(3):
                for c in range(2):
                    val = int(matrix[r][c])
                    color = "#00c853" if val == 0 else "#ff1744"
                    dot_labels[r][c].configure(fg=color)
        root.after(32, _update)

    _apply_toggle()
    _update()
    root.mainloop()


# ────────────────────────── 主函数 ─────────────────────────
def main():
    init_udp_sender(("127.0.0.1", 5005))

    parser = argparse.ArgumentParser()
    parser.add_argument("--ui", choices=["on", "off"], default="on", help="HUD UI 开关")
    args = parser.parse_args()

    print(f"[MAIN] UDP listen {RECV_ADDR}, send to {UDP_TARGET}")

    t_ac = threading.Thread(target=thread_ac_sharedmemory, daemon=True)
    t_acti = threading.Thread(target=thread_acti_udp, daemon=True)
    t_recv = threading.Thread(target=thread_result_receiver, args=(ResultMatrixReceiver(),), daemon=True)
    t_sched = threading.Thread(target=thread_send_scheduler, daemon=True)
    t_log = threading.Thread(target=thread_telemetry_logger, kwargs={"fallback_to_latest_udp": True}, daemon=True)

    t_ac.start()
    t_acti.start()
    t_recv.start()
    t_sched.start()
    t_log.start()

    print("[MAIN] AC thread started.")
    print("[MAIN] ACTI thread started.")
    print("[MAIN] Result receiver started.")
    print("[MAIN] Send scheduler started.")
    print("[MAIN] Telemetry logger thread started (toggle via GUI: DATA).")

    if args.ui == "on":
        t_gui = threading.Thread(target=gui_thread, daemon=True)
        t_gui.start()
    else:
        ui_set_enabled(False)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        if log_is_enabled() and _log_out_xlsx:
            flush_log_to_excel(_log_out_xlsx)
        return


if __name__ == "__main__":
    main()
