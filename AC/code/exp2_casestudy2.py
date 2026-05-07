#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#包括实时数据读取和输出功能
import datetime
import os
import sys, threading, time, ctypes, struct, socket, json, argparse
import mmap
import tkinter as tk
import math
import numpy as np
from openpyxl import Workbook
from typing import Dict, Any, Tuple, List, Optional
from collections import deque

from accardata_to_sim import build_packet_from_live
from opt_vjoy  import VJoyOut   # 在当前版本中仅 g29 模式可能用到，opt 模式不再依赖
from G29_vjoy import main as start_g29

_has_opt_data_lock = threading.Lock()
_has_opt_data = False

def set_has_opt_data():
    global _has_opt_data
    with _has_opt_data_lock:
        _has_opt_data = True

def has_opt_data() -> bool:
    with _has_opt_data_lock:
        return _has_opt_data

# ────────────────────────── 基本节拍与端口 ──────────────────────────
SIM_HZ = 333.0
DT = 1.0 / SIM_HZ
RECV_ADDR = ("0.0.0.0", 56060)

# ────────────────────────── 配置 ──────────────────────────
ACTI_UDP_PORT   = 27152
BURST_REPEATS   = 1

# ────────────────────────── 依赖模块 ──────────────────────
import sim_info_acti
sim = sim_info_acti.SimInfo()

from acti_udp_receiver import (
    open_udp2_receiver as open_acti_receiver,     # 仅线程里使用
    PACK_FORMAT as ACTI_PACK_FORMAT,
    PACK_SIZE as ACTI_PACK_SIZE,
    FIELDS as ACTI_FIELDS,
)

from accardata_to_sim import ACTI_COLS_ORIG, ACTI_LOADED_RADIUS_COLS

UDP_TARGET: Tuple[str, int] = ("127.0.0.1", 5005)
_udp_sock: socket.socket | None = None

def init_udp_sender(addr: Tuple[str, int] = UDP_TARGET) -> None:
    global _udp_sock, UDP_TARGET
    UDP_TARGET = addr
    if _udp_sock is None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(addr)   # 仍为 UDP，只是省掉每次 sendto 的地址参数
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

class UdpReceiver:
    def __init__(self, addr=RECV_ADDR):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(addr)
        self.sock.setblocking(False)
        self.latest: Optional[Dict] = None
        self.lock = threading.Lock()

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                msg = json.loads(data.decode("utf-8"))
                with self.lock:
                    self.latest = msg
            except BlockingIOError:
                break

    def fetch_latest(self) -> Optional[Dict]:
        with self.lock:
            return self.latest

# ────────────────────────── 工具函数 ──────────────────────
import ctypes as C
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

def now_ms() -> int:
    return int(round(time.perf_counter() * 1000.0))

# ────────────────────────── 全局“最新快照” ─────────────────────────
_snap_lock = threading.Lock()
_latest_ac: Dict[str, Any]      = {}
_latest_ac_udp: Dict[str, Any]  = {}
_latest_g29: Dict[str, Any]     = {}

def get_latest_ac() -> Dict[str, Any]:
    with _snap_lock:
        return dict(_latest_ac) if _latest_ac else {}

def get_latest_ac_udp() -> Dict[str, Any]:
    with _snap_lock:
        return dict(_latest_ac_udp) if _latest_ac_udp else {}

def get_latest_g29() -> Dict[str, Any]:
    with _snap_lock:
        return dict(_latest_g29) if _latest_g29 else {}

# === 新增：从 AC 共享内存中读取当前 steer0（物理方向盘状态） ===
def get_steer0_from_ac() -> float:
    """
    从最新的 AC 共享内存快照中读取当前方向盘 steer0。
    优先使用 physics['steer']，若没有则尝试几个常见字段名。
    读取失败时返回 0.0。
    """
    ac = get_latest_ac()
    phys = ac.get("physics", {}) if isinstance(ac, dict) else {}
    for key in ("steerAngle",):
        if key in phys:
            try:
                return float(phys[key])
            except Exception:
                pass
    return 0.0
def get_gas0_from_ac() -> float:
    """
    从最新的 AC 共享内存快照中读取当前方向盘 steer0。
    优先使用 physics['gas']，若没有则尝试几个常见字段名。
    读取失败时返回 0.0。
    """
    ac = get_latest_ac()
    phys = ac.get("physics", {}) if isinstance(ac, dict) else {}
    for key in ("gas",):
        if key in phys:
            try:
                return float(phys[key])
            except Exception:
                pass
    return 0.0

def get_brake0_from_ac() -> float:
    """
    从最新的 AC 共享内存快照中读取当前方向盘 steer0。
    优先使用 physics['steer']，若没有则尝试几个常见字段名。
    读取失败时返回 0.0。
    """
    ac = get_latest_ac()
    phys = ac.get("physics", {}) if isinstance(ac, dict) else {}
    for key in ("brake",):
        if key in phys:
            try:
                return float(phys[key])
            except Exception:
                pass
    return 0.0

# 维护 acti UDP2 的环形缓存，集中读取，其他地方不再直接 recvfrom
_acti_cache_lock = threading.Lock()
_acti_cache: deque = deque(maxlen=64)  # 存最近 64 个 acti 包（dict）

def _extract_packet_id(d: Dict[str, Any]) -> Optional[int]:
    for k in ("packetId", "packet_id", "id", "packetID"):
        if k in d:
            try:
                return int(d[k])
            except Exception:
                return None
    return None

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

# ────────────────────────── 线程：AC 共享内存 ────────────────────────
def thread_ac_sharedmemory(poll_interval: float = 0.001):
    sim = sim_info_acti.SimInfo()
    print("[AC] Attached to Assetto Corsa shared memory.")
    try:
        while True:
            physics   = sim.physics
            packet_id = getattr(physics, "packetId", None)
            static    = sim.static
            graphics  = sim.graphics
            t0 = time.perf_counter()

            static_dict   = {n: convert_ctypes(getattr(static, n))   for n, _ in type(static)._fields_}
            physics_dict  = {n: convert_ctypes(getattr(physics, n))  for n, _ in type(physics)._fields_}
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

            # spd = physics_dict.get("speedKmh", 0.0)
            # rpm = physics_dict.get("rpms", 0.0)
            # steer = physics_dict.get("steerAngle", 0.0)
            # gas = physics_dict.get("gas", 0.0)
            # brake = physics_dict.get("brake", 0.0)
            # sys.stdout.write(f"\r[AC] id={packet_id}  {spd:.1f} km/h  steer:{steer:.3f} gas:{gas:.3f} brake:{brake:.3f}")
            # sys.stdout.flush()

            if poll_interval:
                time.sleep(poll_interval)
    except KeyboardInterrupt:
            # 程序退出前尝试落盘
            if log_is_enabled() and _log_out_xlsx:
                flush_log_to_excel(_log_out_xlsx)
            return

# ────────────────────────── 线程：集中接收 acti UDP2 → 写缓存 ─────────────────
def thread_acti_udp():
    sock = open_acti_receiver(port=ACTI_UDP_PORT, timeout=1.0)
    print(f"\n[acti] listening UDP:{ACTI_UDP_PORT}, expect {ACTI_PACK_SIZE} bytes...")
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
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
            # 进入环形缓存
            with _acti_cache_lock:
                _acti_cache.append(pkt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass

# ────────────────────────── G29 → 仅供监控（不在主循环 apply） ─────────────────
def g29_callback(state: Dict[str, Any]):
    with _snap_lock:
        _latest_g29.clear()
        _latest_g29.update(state or {})

def thread_g29():
    try:
        start_g29(on_update=g29_callback)
    except KeyboardInterrupt:
        pass

# ────────────────────────── 构造 values：使用缓存，不再直接读 UDP ───────────────
def read_one_ac_snapshot() -> Dict[str, Any]:
    try:
        physics  = sim.physics
        graphics = sim.graphics
        static   = sim.static
        snap = {
            "packetId": int(getattr(physics, "packetId", -1)),
            "physics":  {n: convert_ctypes(getattr(physics,  n)) for n, _ in type(physics )._fields_},
            "graphics": {n: convert_ctypes(getattr(graphics, n)) for n, _ in type(graphics)._fields_},
            "static":   {n: convert_ctypes(getattr(static,   n)) for n, _ in type(static  )._fields_},
        }
        return snap
    except Exception:
        return {}

def build_values_once_with_pid_match() -> List[float]:
    """AC 读 1 次 + 从缓存中选取与 AC packetId 最接近的 acti 包 → build → values"""
    # 1) AC
    ac = read_one_ac_snapshot()
    ac_pid = _extract_packet_id(ac)

    # 2) 从缓存选 acti
    acti_sel = _select_best_acti_from_cache(ac_pid)

    # 3) 构造 values（兜底）
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
        acti_fallback = {k: float("nan") for k in (ACTI_COLS_ORIG + ACTI_LOADED_RADIUS_COLS + ["time"]) }
        values, _ = build_packet_from_live(
            physics=phys,
            graphics=grap,
            acti=acti_fallback,
            include_times=True,
            trigger_hotstart=True,
            include_calc=True,
        )
    return values

# ────────────────────────── 仅发送 ─────────────────────────
def run_burst_send_and_collect(values: List[float],
                               repeats: int = BURST_REPEATS,
                               window_sec: float = 0.0,
                               seq: int = 0) -> None:
    for _ in range(repeats):
        udp_send_values(values)

# ────────────────────────── opt 模式：双缓冲 A/B ─────────────────────
_play_lock = threading.Lock()
_play_buf: deque = deque()  # 作为空间 B
_last_frame = {"steer": 0.0, "gas": 0.0, "brake": 0.0}

# 空间 A：SIM 返回的“最新一整段控制序列”
_bufferA_lock = threading.Lock()
_bufferA_seq: Optional[List[Dict[str, float]]] = None

# Space A meta (for node expansion when incoming sequence is fixed 8 nodes)
_bufferA_group: int = 10
_FIXED_GROUP = 10  # fixed node-step group (sender does not provide group/horizon_steps)
_bufferA_horizon_steps: int = 0

_unbound_mode: str = "both"

# ───────── GUI 状态：用于在播放器线程和 GUI 线程之间传递 steer/gas/brake ─────────
_gui_lock = threading.Lock()
_gui_values = {"steer": 0.0, "gas": 0.0, "brake": 0.0}

# ───────── 控制量覆盖（用于 UI=OFF 时“融合写入一次 + 发一次包”） ─────────
# ──────────────────────────────────────────────────────────────────────────────
# UnboundExt shared memory (for external consumer)
# Writes 16 bytes payload: <ffIf  (brake, gas, seq, steer)
# Only written once per A→B fetch event when UI is disabled.
_UNBOUND_TAGNAME = r"Local\UnboundExt"
_UNBOUND_SIZE = 16

_unbound_mm = None
_unbound_lock = threading.Lock()
_unbound_seq: int = 0

def _init_unbound_mmap():
    """Create/open the shared memory region used to publish control values.

    Windows: uses mmap tagname 'Local\\UnboundExt'.
    Non-Windows fallback: file-backed mmap at /tmp/UnboundExt.bin (for testing).
    """
    global _unbound_mm
    if _unbound_mm is not None:
        return

    try:
        # Windows named shared memory
        _unbound_mm = mmap.mmap(-1, _UNBOUND_SIZE, tagname=_UNBOUND_TAGNAME, access=mmap.ACCESS_WRITE)
    except Exception:
        # Fallback for non-Windows environments
        try:
            import os
            p = "/tmp/UnboundExt.bin"
            fd = os.open(p, os.O_CREAT | os.O_RDWR, 0o666)
            os.ftruncate(fd, _UNBOUND_SIZE)
            _unbound_mm = mmap.mmap(fd, _UNBOUND_SIZE, access=mmap.ACCESS_WRITE)
            os.close(fd)
        except Exception as e:
            _unbound_mm = None
            raise RuntimeError(f"Failed to create UnboundExt mmap: {e}")

def write_unboundext_once(steer: float, gas: float, brake: float) -> int:
    """Write one payload into UnboundExt. Returns incremented seq."""
    global _unbound_seq
    with _unbound_lock:
        if _unbound_mm is None:
            _init_unbound_mmap()
        _unbound_seq += 1
        seq = int(_unbound_seq)

        # Clamp to sane ranges
        steer_f = float(max(-1.0, min(1.0, steer)))
        gas_f   = float(max(0.0,  min(1.0, gas)))
        brake_f = float(max(0.0,  min(1.0, brake)))

        payload = struct.pack("<ffIf", brake_f, gas_f, seq, steer_f)
        _unbound_mm.seek(0)
        _unbound_mm.write(payload)
        return seq


# ──────────────────────────────────────────────────────────────────────────────
# Unbound1 shared memory (BATCH format for expanded controls)
#
# Goal: write the whole expanded `group` frames in ONE shot.
#
# Layout (little-endian):
#   Header (16 bytes):
#     magic   : 4s  = b'UB2B'
#     seq     : I   (uint32, increments each write)
#     group   : I   (uint32, number of frames written, <= MAX_GROUP)
#     reserved: I   (uint32, currently 0)
#   Payload (MAX_GROUP * 12 bytes):
#     repeated frames, each: steer(float32), gas(float32), brake(float32)
#
# Total size = 16 + 12*MAX_GROUP bytes.
#
_UNBOUND2_TAGNAME = r"Local\Unbound1"
# Mode2-lite: write ONLY two nodes (u0, u1), no expansion on Python side.
# Layout (little-endian):
#   seq : uint32
#   u0  : steer0, gas0, brake0  (3*float32)
#   u1  : steer1, gas1, brake1  (3*float32)
# Total size = 4 + 6*4 = 28 bytes.
_UNBOUND2_SIZE = 16

_unbound2_mm = None
_unbound2_lock = threading.Lock()
_unbound2_seq = 0

def _init_unbound2_mmap():
    global _unbound2_mm
    if _unbound2_mm is not None:
        return
    try:
        _unbound2_mm = mmap.mmap(-1, _UNBOUND2_SIZE, tagname=_UNBOUND2_TAGNAME, access=mmap.ACCESS_WRITE)
    except Exception:
        try:
            p = "/tmp/Unbound1.bin"
            fd = os.open(p, os.O_CREAT | os.O_RDWR, 0o666)
            os.ftruncate(fd, _UNBOUND2_SIZE)
            _unbound2_mm = mmap.mmap(fd, _UNBOUND2_SIZE, access=mmap.ACCESS_WRITE)
            os.close(fd)
        except Exception as e:
            _unbound2_mm = None
            raise RuntimeError(f"Failed to create Unbound1 mmap: {e}")

def write_Unbound1_u01(u1: Dict[str, float]) -> int:
    global _unbound2_seq, _unbound2_mm

    if u1 is None:
        u1 = {}

    def _get(d: Dict[str, float], k: str, default: float = 0.0) -> float:
        try:
            return float(d.get(k, default))
        except Exception:
            return float(default)

    s1 = float(np.clip(_get(u1, "steer", 0.0), -1.0, 1.0))
    g1 = float(np.clip(_get(u1, "gas",   0.0),  0.0, 1.0))
    b1 = float(np.clip(_get(u1, "brake", 0.0),  0.0, 1.0))

    with _unbound2_lock:
        if _unbound2_mm is None:
            _init_unbound2_mmap()

        _unbound2_seq += 1
        seq = int(_unbound2_seq)
        
        payload = struct.pack("<ffIf", b1, g1, seq, s1)
        _unbound2_mm.seek(0)
        _unbound2_mm.write(payload)
        return seq

# ──────────────────────────────────────────────────────────────────────────────
# Unbound2 shared memory (same 16B payload as UnboundExt/Unbound1)
# Writes 16 bytes payload: <ffIf  (brake, gas, seq, steer)
# Used to publish the 11th control point (seq16[10]) when available.
_UNBOUND3_TAGNAME = r"Local\Unbound2"
_UNBOUND3_SIZE = 16

_unbound3_mm = None
_unbound3_lock = threading.Lock()
_unbound3_seq: int = 0

def _init_unbound3_mmap():
    global _unbound3_mm
    if _unbound3_mm is not None:
        return
    try:
        _unbound3_mm = mmap.mmap(-1, _UNBOUND3_SIZE, tagname=_UNBOUND3_TAGNAME, access=mmap.ACCESS_WRITE)
    except Exception:
        try:
            p = "/tmp/Unbound2.bin"
            fd = os.open(p, os.O_CREAT | os.O_RDWR, 0o666)
            os.ftruncate(fd, _UNBOUND3_SIZE)
            _unbound3_mm = mmap.mmap(fd, _UNBOUND3_SIZE, access=mmap.ACCESS_WRITE)
            os.close(fd)
        except Exception as e:
            _unbound3_mm = None
            raise RuntimeError(f"Failed to create Unbound2 mmap: {e}")

def write_Unbound2_u(u2: Dict[str, float]) -> int:
    """Write ONE node (u2) into Unbound2 with the same 16B payload format as UnboundExt/Unbound1.
    Payload: <ffIf = brake, gas, seq, steer
    Returns incremented seq.
    """
    global _unbound3_seq, _unbound3_mm
    if u2 is None:
        u2 = {}

    def _get(d: Dict[str, float], k: str, default: float = 0.0) -> float:
        try:
            return float(d.get(k, default))
        except Exception:
            return float(default)

    s = float(np.clip(_get(u2, "steer", 0.0), -1.0, 1.0))
    g = float(np.clip(_get(u2, "gas",   0.0),  0.0, 1.0))
    b = float(np.clip(_get(u2, "brake", 0.0),  0.0, 1.0))

    with _unbound3_lock:
        if _unbound3_mm is None:
            _init_unbound3_mmap()

        _unbound3_seq += 1
        seq = int(_unbound3_seq)

        payload = struct.pack("<ffIf", b, g, seq, s)
        _unbound3_mm.seek(0)
        _unbound3_mm.write(payload)
        return seq
    
# ────
# ───── UI 开关：允许开启/关闭 HUD 显示（不影响后台接收/播放线程） ─────────
_ui_enabled_event = threading.Event()
_ui_enabled_event.set()   # 默认开启

def ui_is_enabled() -> bool:
    return _ui_enabled_event.is_set()

def ui_set_enabled(enabled: bool) -> None:
    if enabled:
        _ui_enabled_event.set()
    else:
        _ui_enabled_event.clear()

# ───── 数据输出开关：记录遥测并在关闭/退出时落盘到 Excel ─────────
_log_enabled_event = threading.Event()
_log_enabled_event.clear()  # 默认关闭

_log_lock = threading.Lock()
_log_records: List[tuple] = []
_log_out_xlsx: Optional[str] = None
_log_t0: float = 0.0
_log_last_pid: int = -1

def log_is_enabled() -> bool:
    return _log_enabled_event.is_set()

def log_set_enabled(enabled: bool, out_xlsx: Optional[str] = None) -> None:
    """开启/关闭记录。开启时会清空缓存并重置计时；关闭时会保存到 Excel。"""
    global _log_out_xlsx, _log_t0, _log_last_pid

    if enabled:
        with _log_lock:
            _log_records.clear()
        _log_last_pid = -1
        _log_t0 = time.perf_counter()

        if out_xlsx is None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_xlsx = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    f"telemetry_{ts}.xlsx")
        _log_out_xlsx = out_xlsx
        _log_enabled_event.set()
        print("[LOG] enabled ->", _log_out_xlsx)
    else:
        _log_enabled_event.clear()
        # 关闭时立即保存（如果有数据）
        if _log_out_xlsx:
            flush_log_to_excel(_log_out_xlsx)

def _ms_to_s(ms: int) -> float:
    return float(ms) / 1000.0

def _pick_acti_exact_pid(ac_pid: int) -> Optional[Dict[str, Any]]:
    """在 _acti_cache 内找同 pid；找不到返回 None（不做 abs(pid-ac_pid) 近似）。"""
    with _acti_cache_lock:
        # 从新到旧找，命中就返回
        for pkt in reversed(_acti_cache):
            pid = _extract_packet_id(pkt)
            if pid == ac_pid:
                return pkt
    return None

def flush_log_to_excel(out_xlsx: str) -> None:
    """把当前缓存落盘到 Excel（telemetry + laps 两张表）。"""
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
        "x", "y", "z","gas", "brake", "steerAngle","heading", "pitch", "roll",
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
    print(f"[LOG] saved: {out_xlsx}  (samples={len(recs)})")

def thread_telemetry_logger(poll_dt: float = 0.0,
                            fallback_to_latest_udp: bool = True,
                            log_every_n_frames: int = 3) -> None:
    """后台线程：以共享内存 packetId 为主时钟，采样并对齐 UDP(SlipAngle/NdSlip)。"""
    global _log_last_pid

    last_udp_pid: Optional[int] = None
    last_slip4 = (None, None, None, None)
    last_nd4   = (None, None, None, None)
    frame_count = 0  # ✅ 新增：按“新 pid”计帧

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

        # 只在 packetId 变化时认为“新帧”
        if pid == _log_last_pid:
            time.sleep(0.001)
            continue
        _log_last_pid = pid

        frame_count += 1

        # ✅ 抽样：不是第 N 帧就跳过（但仍然更新 last_pid，避免重复）
        # if log_every_n_frames > 1 and (frame_count % log_every_n_frames) != 0:
        #     continue

        t_sec = time.perf_counter() - float(_log_t0)

        phys = ac.get("physics", {}) or {}
        grap = ac.get("graphics", {}) or {}

        # 更新“最近一帧 acti UDP”（用于 fallback）
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

        # coordinates
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

        # wheelSlip: 4 floats
        ws_fl = ws_fr = ws_rl = ws_rr = None
        try:
            ws = phys.get("wheelSlip", None)
            if isinstance(ws, (list, tuple)) and len(ws) >= 4:
                ws_fl, ws_fr, ws_rl, ws_rr = map(float, ws[:4])
        except Exception:
            pass

        completed_laps = int(grap.get("completedLaps", -1))

        i_cur_s  = _ms_to_s(int(grap.get("iCurrentTime", 0)))
        i_last_s = _ms_to_s(int(grap.get("iLastTime", 0)))
        i_best_s = _ms_to_s(int(grap.get("iBestTime", 0)))
        normalizedCarPosition = grap.get("normalizedCarPosition", 0)
        currentSectorIndex = grap.get("currentSectorIndex", 0)
        lastSectorTime = grap.get("lastSectorTime", 0)

        # 对齐 UDP：优先同 pid，否则（可选）回退到最近一帧
        udp_pid_used = None
        slip4 = (None, None, None, None)
        nd4   = (None, None, None, None)

        pkt = _pick_acti_exact_pid(pid)
        if pkt is not None:
            udp_pid_used = pid
            # acti_udp_receiver.FIELDS 里字段名就是这些（你文件里是 SlipAngle_FL 等）
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
            x, y, z, gas, brake, steerAngle,heading, pitch, roll,
            speed_kmh,
            completed_laps,
            float(i_cur_s), float(i_last_s), float(i_best_s),float(normalizedCarPosition),float(currentSectorIndex),float(lastSectorTime),
            ws_fl, ws_fr, ws_rl, ws_rr,
            udp_pid_used,
            slip4[0], slip4[1], slip4[2], slip4[3],
            nd4[0], nd4[1], nd4[2], nd4[3],
        )

        with _log_lock:
            _log_records.append(row)
        if poll_dt:
            time.sleep(poll_dt)


# ====== epoch 计时相关（占位；当前方案不再用 t0/tall_ms）======
_epoch_lock = threading.Lock()
_epoch_t0_ms: int = 0
_need_latch_t0: bool = True

def latch_t0_if_needed() -> None:
    global _epoch_t0_ms, _need_latch_t0
    with _epoch_lock:
        if _need_latch_t0:
            _epoch_t0_ms = now_ms()
            _need_latch_t0 = False

def mark_need_next_t0() -> None:
    global _need_latch_t0
    with _epoch_lock:
        _need_latch_t0 = True

def get_epoch_t0_ms() -> int:
    with _epoch_lock:
        return _epoch_t0_ms

def thread_opt_player():
    """opt 模式下的“播放器”：从空间 B 取数据，以 30ms 间隔输出到 GUI，并在 A→B 抓取时向 SIM 发送状态。"""
    period = 0.030
    try:
        while True:
            frame_to_play: Optional[Dict[str, float]] = None
            need_send_state = False

            # 1) 先尝试从空间 B（_play_buf）取一帧
            with _play_lock:
                if _play_buf:
                    frm = _play_buf.popleft()
                    _last_frame.update(frm)
                    frame_to_play = frm

            # 2) 如果 B 为空，尝试从空间 A 抓取“后 50%”填充到 B
            if frame_to_play is None:
                with _bufferA_lock:
                    seq = list(_bufferA_seq) if _bufferA_seq else None

                if seq:
                    H = len(seq)
                    start = H // 2
                    if start >= H:
                        start = 0
                    sub_seq = seq[start:]
                    if sub_seq:
                        with _play_lock:
                            if not _play_buf:
                                for frm in sub_seq:
                                    _play_buf.append(dict(frm))
                                frm0 = _play_buf.popleft()
                                _last_frame.update(frm0)
                                frame_to_play = frm0
                                need_send_state = True

            # 3) 如果仍然没有可用帧，则说明 A 也没有数据 → 稍微等待
            if frame_to_play is None:
                time.sleep(0.01)
                continue

            # 4) 若 UI=ON：更新 GUI；若 UI=OFF：仍按 30ms tick 消耗 B，但不做 GUI 绘制更新
            steer_val = float(frame_to_play.get("steer", 0.0))
            gas_val   = float(frame_to_play.get("gas",   0.0))
            brake_val = float(frame_to_play.get("brake", 0.0))

            if ui_is_enabled():
                with _gui_lock:
                    _gui_values["steer"] = steer_val
                    _gui_values["gas"] = gas_val
                    _gui_values["brake"] = brake_val

            # 5) 若刚刚从 A 抓取了一段填充 B，则向 SIM 发一次新的状态
            if need_send_state:
                if not ui_is_enabled():
                    steer_cur = get_steer0_from_ac()
                    gas_cur   = get_gas0_from_ac()
                    brake_cur = get_brake0_from_ac()

                    steer_new = float(steer_val)
                    gas_new   = float(gas_val)
                    brake_new = float(brake_val)
                    
                    #print(steer_cur,steer_new)

                    # clamp (保持与 handle_opt_seq_message 一致的范围)
                    steer_new = max(-1.0, min(1.0, steer_new))
                    gas_new   = max(0.0,  min(1.0, gas_new))
                    brake_new = max(0.0,  min(1.0, brake_new))
                    # print(gas_new,brake_new,gas_cur,brake_cur)

                    if _unbound_mode in ("mode1", "both"):
                        _ = write_unboundext_once(steer_new, gas_new, brake_new)

                    # mode2: expand between tail_u0 and tail_u1 (nodes[4], nodes[5]) into `group` frames,
                    # then write the WHOLE segment into Unbound1 in one shot.
                    if _unbound_mode in ("mode2", "both"):
                        try:
                            with _bufferA_lock:
                                seq16 = list(_bufferA_seq) if _bufferA_seq else None
                            if seq16 and len(seq16) >= 10:
                            
                                _ = write_Unbound1_u01(seq16[9])
                                if len(seq16) >= 11:
                                    _ = write_Unbound2_u(seq16[10])
                        except Exception:
                            pass
                    # time.sleep(10)

                # 写入完成后，抓取当前状态构建数据集并发送给模拟器（只发送一次）
                values = build_values_once_with_pid_match()
                run_burst_send_and_collect(values, repeats=1)

            # 6) 间隔 30 ms
            time.sleep(period)
    except KeyboardInterrupt:
        return

# ───────── GUI：使用 tkinter 绘制四个三角形，绕中心对称 ─────────
def _value_to_color_hex(v: float) -> str:
    """
    将 [0,1] 数值映射为从白色到绿色的填充色：
    0 -> 白色 (#FFFFFF)，1 -> 亮绿色 (#00FF00)
    """
    try:
        v = float(v)
    except Exception:
        v = 0.0
    if v < 0.0:
        v = 0.0
    if v > 1.0:
        v = 1.0
    r = int(255 * (1.0 - v))
    g = 255
    b = int(255 * (1.0 - v))
    return f"#{r:02x}{g:02x}{b:02x}"

def _clamp01(x: float) -> float:
    return 0.0 if x <= 0.0 else (1.0 if x >= 1.0 else x)

def _inten_from_delta(delta: float, thr: float, vmax: float = 1.0, gamma: float = 1.0) -> float:
    """
    delta: 误差幅度（>=0）
    thr:   阈值（白色区）
    vmax:  显示饱和上限（>=thr）
    gamma: <1 会增强小差异（更显眼），>1 会压缩小差异
    """
    if delta <= thr:
        return 0.0
    denom = max(1e-9, (vmax - thr))
    x = (delta - thr) / denom
    x = _clamp01(x)
    return x ** gamma

def _color_white_to_green(inten: float) -> str:
    inten = _clamp01(inten)
    r = int(255 * (1.0 - inten))
    g = 255
    b = int(255 * (1.0 - inten))
    return f"#{r:02x}{g:02x}{b:02x}"

def _color_white_to_red(inten: float) -> str:
    inten = _clamp01(inten)
    r = 255
    g = int(255 * (1.0 - inten))
    b = int(255 * (1.0 - inten))
    return f"#{r:02x}{g:02x}{b:02x}"

def _color_white_to_orange_red(inten: float) -> str:
    """更醒目：白 -> 橙红（用于 brake 的“需要刹车”）"""
    inten = _clamp01(inten)
    r = 255
    g = int(255 * (1.0 - 0.55 * inten))  # 让绿色衰减慢一点，颜色更“亮”
    b = int(255 * (1.0 - inten))
    return f"#{r:02x}{g:02x}{b:02x}"

def _color_white_to_magenta(inten: float) -> str:
    """白 -> 品红（用于 brake 的“刹太多了”警告）"""
    inten = _clamp01(inten)
    r = 255
    g = int(255 * (1.0 - inten))
    b = 255
    return f"#{r:02x}{g:02x}{b:02x}"

def _triangle_coords(direction: str, cx: float, cy: float, size: float):
    """
    生成不同方向三角形的三个顶点坐标：
    direction: 'left' / 'right' / 'up' / 'down'
    """
    if direction == "left":
        return [cx + size, cy - size, cx + size, cy + size, cx - size, cy]
    elif direction == "right":
        return [cx - size, cy - size, cx - size, cy + size, cx + size, cy]
    elif direction == "up":
        return [cx - size, cy + size, cx + size, cy + size, cx, cy - size]
    else:  # "down"
        return [cx - size, cy - size, cx + size, cy - size, cx, cy + size]
    
def _clamp01(x: float) -> float:
    return 0.0 if x <= 0.0 else (1.0 if x >= 1.0 else x)

def _inten(delta: float, thr: float, vmax: float, gamma: float) -> float:
    """delta>=0：超过阈值的‘差异幅度’，映射到 0..1 强度"""
    if delta <= thr:
        return 0.0
    x = (delta - thr) / max(1e-9, (vmax - thr))
    return _clamp01(x) ** gamma

def _white_to_green(inten: float) -> str:
    inten = _clamp01(inten)
    r = int(255 * (1.0 - inten))
    g = 255
    b = int(255 * (1.0 - inten))
    return f"#{r:02x}{g:02x}{b:02x}"

def _white_to_red(inten: float) -> str:
    inten = _clamp01(inten)
    r = 255
    g = int(255 * (1.0 - inten))
    b = int(255 * (1.0 - inten))
    return f"#{r:02x}{g:02x}{b:02x}"


def gui_thread():
    """
    独立线程运行的 GUI（显示 + 控制开关）：
      - 控制窗：一个 Checkbutton，用于开启/关闭 HUD
      - HUD 窗：透明（色键），窗口置顶，无文字标签（仅图形）
      - 关闭 HUD 仅隐藏 HUD 窗口，不影响后台线程
    """
    # ── control window (always visible)
    ctrl = tk.Tk()
    ctrl.title("HUD Ctrl")
    try:
        ctrl.attributes("-topmost", True)
    except Exception:
        pass
    ctrl.geometry("220x90+40+40")
    ctrl.resizable(False, False)

    var_on = tk.BooleanVar(value=True)
    var_log = tk.BooleanVar(value=False)

    def _apply_toggle():
        on = bool(var_on.get())
        ui_set_enabled(on)
        try:
            if on:
                hud.deiconify()
                hud.lift()
            else:
                hud.withdraw()
        except Exception:
            pass

    chk = tk.Checkbutton(ctrl, text="HUD", variable=var_on, command=_apply_toggle)
    chk.pack(anchor="w", padx=10, pady=12)

    def _apply_log_toggle():
        on = bool(var_log.get())
        if on:
            # 开启：自动生成带时间戳文件名
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_xlsx = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    f"telemetry_{ts}.xlsx")
            log_set_enabled(True, out_xlsx=out_xlsx)
        else:
            log_set_enabled(False)

    chk2 = tk.Checkbutton(ctrl, text="DATA", variable=var_log, command=_apply_log_toggle)
    chk2.pack(anchor="w", padx=10, pady=0)

    # ── HUD window (transparent overlay)
    hud = tk.Toplevel(ctrl)
    hud.title("OPT HUD")
    try:
        hud.attributes("-topmost", True)
    except Exception:
        pass

    # ---- transparent background via color-key (Windows works best)
    key_color = "#ff00ff"  # magenta as transparency key
    try:
        hud.configure(bg=key_color)
        hud.wm_attributes("-transparentcolor", key_color)
    except Exception:
        pass

    # ---- canvas
    width, height = 520, 340
    canvas = tk.Canvas(hud, width=width, height=height, bg=key_color, highlightthickness=0, bd=0)
    canvas.pack()

    # ---- layout
    ring_cx, ring_cy = 170, 170
    r_outer, r_inner = 110, 82
    theta_max_deg = 450.0  # +/- 450 deg visualization range

    pedal_x0 = 350
    pedal_y0 = 60
    pedal_w  = 48
    pedal_h  = 230
    pedal_gap = 40

    # marker sizes (bigger on steering ring)
    r_marker_steer = 10
    pedal_handle_hw = 10

    # tags for dynamic items
    TAG_REAL = "real"
    TAG_CMD  = "cmd"
    TAG_DIFF = "diff"

    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def _draw_tick(cx, cy, ang_deg, r0, r1, color="#000", w=1):
        a = math.radians(ang_deg)
        x0 = cx + r0 * math.cos(a)
        y0 = cy + r0 * math.sin(a)
        x1 = cx + r1 * math.cos(a)
        y1 = cy + r1 * math.sin(a)
        canvas.create_line(x0, y0, x1, y1, fill=color, width=w)

    def _draw_ring_base():
        # outer ring
        canvas.create_oval(ring_cx - r_outer, ring_cy - r_outer, ring_cx + r_outer, ring_cy + r_outer,
                           outline="#333", width=2)
        # inner "hole" filled with key_color so it becomes transparent too
        canvas.create_oval(ring_cx - r_inner, ring_cy - r_inner, ring_cx + r_inner, ring_cy + r_inner,
                           outline=key_color, width=2, fill=key_color)

        # zero mark
        _draw_tick(ring_cx, ring_cy, -90, r_inner - 4, r_outer + 4, color="#111", w=3)

        # ticks for +/-360deg, every 30deg
        for deg in range(-360, 361, 30):
            a = -90 + deg
            w = 2 if deg % 90 == 0 else 1
            col = "#444" if deg % 90 == 0 else "#777"
            _draw_tick(ring_cx, ring_cy, a, r_inner + 2, r_outer - 2, color=col, w=w)

    def _draw_pedal_base(x, y):
        canvas.create_rectangle(x, y, x + pedal_w, y + pedal_h, outline="#333", width=2)
        for frac in (0.3, 0.7):
            yy = y + pedal_h * (1.0 - frac)
            canvas.create_line(x, yy, x + pedal_w, yy, fill="#ddd", width=1)

    def _draw_marker_on_ring(theta_deg, color, tag):
        screen_deg = -90 + theta_deg
        a = math.radians(screen_deg)
        r = (r_outer + r_inner) / 2.0
        x = ring_cx + r * math.cos(a)
        y = ring_cy + r * math.sin(a)
        rr = r_marker_steer
        canvas.create_oval(x-rr, y-rr, x+rr, y+rr, fill=color, outline="white", width=1, tags=(tag,))

    def _draw_diff_arc(theta_real_deg, theta_cmd_deg):
        # arc between angles (visualize |delta|)
        a0 = -90 + theta_real_deg
        a1 = -90 + theta_cmd_deg

        # Tkinter arc convention conversion:
        tk_a0 = 90 - a0
        tk_a1 = 90 - a1

        d = tk_a1 - tk_a0
        while d > 180:
            d -= 360
        while d < -180:
            d += 360

        mag = abs(d)
        w = 2 if mag < 10 else 4 if mag < 30 else 6
        col = "#bbb" if mag < 5 else "#999" if mag < 15 else "#666"

        bbox = (ring_cx - r_outer, ring_cy - r_outer, ring_cx + r_outer, ring_cy + r_outer)
        canvas.create_arc(*bbox, start=tk_a0, extent=d, style="arc", outline=col, width=w, tags=(TAG_DIFF,))

    def _draw_marker_on_pedal(x, y, value01, color, tag):
        v = _clamp(value01, 0.0, 1.0)
        yy = y + pedal_h * (1.0 - v)
        canvas.create_line(x, yy, x + pedal_w, yy, fill=color, width=3, tags=(tag,))
        hw = pedal_handle_hw
        canvas.create_rectangle(x + pedal_w/2 - hw, yy - 5, x + pedal_w/2 + hw, yy + 5,
                                fill=color, outline="white", width=1, tags=(tag,))

    # ---- draw static bases once
    _draw_ring_base()
    _draw_pedal_base(pedal_x0, pedal_y0)                          # GAS
    _draw_pedal_base(pedal_x0 + pedal_w + pedal_gap, pedal_y0)    # BRAKE

    def _update():
        # HUD 被关闭时：不画动态图形，保持 after 继续跑（便于随时重新开启）
        if not ui_is_enabled():
            hud.after(100, _update)
            return

        # cmd from optimizer (space B output)
        with _gui_lock:
            steer_cmd = _gui_values.get("steer", 0.0)
            gas_cmd   = _gui_values.get("gas",   0.0)
            brake_cmd = _gui_values.get("brake", 0.0)

        try:
            steer_cmd_val = float(steer_cmd)
            gas_cmd_val   = float(gas_cmd)
            brake_cmd_val = float(brake_cmd)
        except Exception:
            steer_cmd_val = 0.0
            gas_cmd_val   = 0.0
            brake_cmd_val = 0.0

        # real inputs from AC shared memory
        steer0 = get_steer0_from_ac()
        gas0   = get_gas0_from_ac()
        brake0 = get_brake0_from_ac()

        # ---- clear dynamic
        canvas.delete(TAG_REAL)
        canvas.delete(TAG_CMD)
        canvas.delete(TAG_DIFF)

        # ---- steering ring
        s_real = _clamp(float(steer0), -1.0, 1.0)
        s_cmd  = _clamp(float(steer_cmd_val), -1.0, 1.0)
        theta_real = s_real * theta_max_deg
        theta_cmd  = s_cmd  * theta_max_deg

        _draw_diff_arc(theta_real, theta_cmd)
        _draw_marker_on_ring(theta_real, color="#888", tag=TAG_REAL)
        _draw_marker_on_ring(theta_cmd,  color="#2b78ff", tag=TAG_CMD)

        # ---- pedals
        xg = pedal_x0
        xb = pedal_x0 + pedal_w + pedal_gap
        y0 = pedal_y0

        _draw_marker_on_pedal(xg, y0, float(gas0),        color="#888",    tag=TAG_REAL)
        _draw_marker_on_pedal(xg, y0, float(gas_cmd_val), color="#2b78ff", tag=TAG_CMD)

        _draw_marker_on_pedal(xb, y0, float(brake0),        color="#888",    tag=TAG_REAL)
        _draw_marker_on_pedal(xb, y0, float(brake_cmd_val), color="#2b78ff", tag=TAG_CMD)

        hud.after(33, _update)  # ~30 Hz

    # 初始应用一次（确保 HUD/控制窗同步）
    _apply_toggle()
    _update()
    ctrl.mainloop()

# ───────── 基于 marker/elapsed_ms 的去重（用于 A 的更新） ─────────
_last_elapsed_lock = threading.Lock()
_last_elapsed_ms_seen: Optional[int] = None
_last_marker_lock = threading.Lock()
_last_marker_seen: Optional[int] = None

def _get_last_elapsed() -> Optional[int]:
    with _last_elapsed_lock:
        return _last_elapsed_ms_seen

def _set_last_elapsed(v: Optional[int]):
    global _last_elapsed_ms_seen
    with _last_elapsed_lock:
        _last_elapsed_ms_seen = v

def _get_last_marker() -> Optional[int]:
    with _last_marker_lock:
        return _last_marker_seen

def _set_last_marker(v: Optional[int]):
    global _last_marker_seen
    with _last_marker_lock:
        _last_marker_seen = v

# ────────────────────────── opt 接收：填充空间 A ─────────────────────
def handle_opt_seq_message(msg: Dict[str, Any]) -> bool:
    """
    处理一条 opt_seq 消息。
    返回值:
        True  = 这次收到了“新的、有效的控制序列”，并成功写入空间 A；
        False = 没有写入空间 A（重复包 / 无效数据 / 空序列 等）。
    """
    # 1) 解析 elapsed_ms / marker / verbose
    try:
        elapsed_ms = int(msg.get("elapsed_ms", -1))
    except Exception:
        elapsed_ms = -1
    verbose = bool(msg.get("verbose", True))

    try:
        marker_raw = msg.get("marker", None)
        marker = int(marker_raw) if marker_raw is not None else None
    except Exception:
        marker = None

    # 2) 去重逻辑：
    #    - 若存在 marker，则优先使用 marker 去重
    #    - 若不存在 marker，则退回 elapsed_ms 去重
    if marker is not None:
        last_marker = _get_last_marker()
        if last_marker is not None and marker == last_marker:
            if verbose:
                print(f"[opt] duplicate marker={marker}, skip.")
            return False   # ← 重复包：这次“没有收到新数据”
        _set_last_marker(marker)
    else:
        last_em = _get_last_elapsed()
        if last_em is not None and elapsed_ms == last_em:
            if verbose:
                print(f"[opt] duplicate elapsed_ms={elapsed_ms}, skip.")
            return False   # ← 重复包：这次“没有收到新数据”
        _set_last_elapsed(elapsed_ms)

    # 3) 解析 u_seq
    seq_list = msg.get("u_seq")

    if not isinstance(seq_list, list) or len(seq_list) == 0:
        # u_seq 不存在或为空 → 没有有效数据
        if verbose:
            print("[opt] invalid or empty u_seq, skip.")
        return False

    parsed: List[Dict[str, float]] = []
    for step in seq_list:
        if not isinstance(step, dict):
            continue

        def _first(d, names, default):
            for k in names:
                if k in d:
                    try:
                        return float(d[k])
                    except Exception:
                        pass
            return float(default)

        steer = _first(step, ["steer", "s"], 0.0)                          # [-1,1]
        gas   = _first(step, ["gas", "g", "throttle", "accel"], 0.0)       # [0,1] 或 [-1,1]
        brake = _first(step, ["brake", "brakes", "b"], 0.0)                # [0,1] 或 [-1,1]

        if gas   < 0.0:
            gas   = 0.5 * (gas + 1.0)      # 允许 [-1,1] 容错
        if brake < 0.0:
            brake = 0.5 * (brake + 1.0)

        steer = max(-1.0, min(1.0, steer))
        gas   = max(0.0, min(1.0, gas))
        brake = max(0.0, min(1.0, brake))

        parsed.append({"steer": steer, "gas": gas, "brake": brake})

    if not parsed:
        # 所有 step 都解析失败 → 视为无效数据
        if verbose:
            print("[opt] parsed seq is empty after filtering, skip.")
        return False    # 3.5) group/horizon_steps are FIXED on receiver side (sender does not provide them)
    # _bufferA_group is fixed to _FIXED_GROUP

    # 4) 把完整序列存入空间 A（覆盖旧数据）
    global _bufferA_seq
    with _bufferA_lock:
        _bufferA_seq = parsed
    set_has_opt_data()

    if verbose:
        print(f"[opt] received seq: H={len(parsed)}  elapsed_ms={elapsed_ms}  marker={marker}")

    return True

def thread_opt_receiver(recv: UdpReceiver):
    try:
        while True:
            recv.poll()
            msg = recv.fetch_latest()
            if isinstance(msg, dict) and msg.get("type") == "opt_seq":
                _ = handle_opt_seq_message(msg)
            time.sleep(0.001)
    except KeyboardInterrupt:
        return

# ────────────────────────── 主函数 ─────────────────────────
def main():
    global _unbound_mode, _UNBOUND_TAGNAME, _UNBOUND2_TAGNAME, _UNBOUND3_TAGNAME, _unbound_mm, _unbound2_mm, _unbound3_mm
    init_udp_sender(("127.0.0.1", 5005))
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["opt", "g29"], default="opt",
                        help="控制来源: opt=优化器, g29=G29方向盘")
    parser.add_argument("--ui", choices=["on", "off"], default="on",
                        help="HUD UI 开关: on=显示, off=不启动 GUI")
    parser.add_argument("--unbound-mode", choices=["mode1", "mode2", "both"], default="both",
                        help="UI=off 时写共享内存模式: mode1=写 UnboundExt(单点融合), mode2=写 Unbound1(仅写两节点u0/u1; 不在Python端展开), both=两者都写")
    parser.add_argument("--unbound1-tag", default=_UNBOUND_TAGNAME,
                        help="mode1 共享内存 tagname (Windows), 默认 Local\\UnboundExt")
    parser.add_argument("--unbound2-tag", default=_UNBOUND2_TAGNAME,
                        help="mode2 共享内存 tagname (Windows), 默认 Local\\Unbound1")
    parser.add_argument("--unbound3-tag", default=_UNBOUND3_TAGNAME)
    args = parser.parse_args()
    # Apply unbound settings
    _unbound_mode = str(args.unbound_mode)
    _UNBOUND_TAGNAME = str(args.unbound1_tag)
    _UNBOUND2_TAGNAME = str(args.unbound2_tag)
    _UNBOUND3_TAGNAME = str(args.unbound3_tag)
    # reset mmap handles so new tag names take effect
    _unbound_mm = None
    _unbound2_mm = None
    _unbound3_mm = None

    print(f"[MAIN] mode={args.mode}  UDP listening on {RECV_ADDR} (apply @ {SIM_HZ:.0f} Hz)")

    # 启动 AC & acti 两个数据源线程（acti 只在此线程内 bind/recv）
    t_ac   = threading.Thread(target=thread_ac_sharedmemory, daemon=True)
    t_acti = threading.Thread(target=thread_acti_udp,        daemon=True)
    t_ac.start()
    t_acti.start()  


    t_log = threading.Thread(target=thread_telemetry_logger, kwargs={'fallback_to_latest_udp': True}, daemon=True)
    t_log.start()
    print('[MAIN] Telemetry logger thread started (toggle via GUI: DATA).')

    tg = threading.Thread(target=thread_g29, daemon=True)
    tg.start()
    print("[MAIN] G29 thread started.")

    if args.mode == "opt":
        recv = UdpReceiver()
        t_player = threading.Thread(target=thread_opt_player, daemon=True)
        t_player.start()
        t_opt = threading.Thread(target=thread_opt_receiver, args=(recv,), daemon=True)
        t_opt.start()
        # 启动 GUI 线程（可用 --ui off 关闭）
        if args.ui == "on":
            t_gui = threading.Thread(target=gui_thread, daemon=True)
            t_gui.start()
        else:
            ui_set_enabled(False)

        # ★ 初始化阶段：定期给 SIM 发状态，直到第一次收到 opt_seq
        try:
            while not has_opt_data():  # has_opt_data() 在 handle_opt_seq_message 里置 True
                values = build_values_once_with_pid_match()
                run_burst_send_and_collect(values, repeats=1)
                # print("[MAIN/opt] bootstrap send to SIM (no opt_seq yet)")
                time.sleep(0.001)
            # print("[MAIN/opt] first opt_seq arrived, stop bootstrap loop.")
            # 之后进入纯 sleep 的主循环，交给 player / GUI / receiver 驱动
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            return

if __name__ == "__main__":
    main()