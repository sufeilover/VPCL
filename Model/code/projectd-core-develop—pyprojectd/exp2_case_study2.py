import os, sys, site, time, math, json, socket, struct, threading, argparse
import multiprocessing as mp
import numpy as np
from dataclasses import dataclass, field
from multiprocessing import current_process, parent_process
import statistics

_seq_counter = 0  # 全局序列号

DEFAULT_SIM_HZ = 333.0
DEFAULT_TRACK = 'ks_silverstone1967'
DEFAULT_CAR   = 'ks_toyota_ae86_drift'

SEND_ADDR = ("127.0.0.1", 56060)
DATA_UDP_IP, DATA_UDP_PORT = "0.0.0.0", 5005

base_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
bin_dir  = os.path.join(base_dir, 'bin')
site.addsitedir(bin_dir)
os.add_dll_directory(bin_dir)
import PyProjectD as pd

# ================= UDP input =================
graphics_cols = ["carCoordinates_1", "carCoordinates_2", "carCoordinates_3"]
physics_cols = [
    "gas", "brake", "gear", "rpms", "steerAngle",
    "velocity_1", "velocity_2", "velocity_3",
    "wheelAngularSpeed_1", "wheelAngularSpeed_2", "wheelAngularSpeed_3", "wheelAngularSpeed_4",
    "tyreCoreTemperature_1", "tyreCoreTemperature_2", "tyreCoreTemperature_3", "tyreCoreTemperature_4",
    "tyreTempI_1", "tyreTempI_2", "tyreTempI_3", "tyreTempI_4",
    "tyreTempM_1", "tyreTempM_2", "tyreTempM_3", "tyreTempM_4",
    "tyreTempO_1", "tyreTempO_2", "tyreTempO_3", "tyreTempO_4",
    "brakeTemp_1", "brakeTemp_2", "brakeTemp_3", "brakeTemp_4",
    "heading", "pitch", "roll",
]
acti_cols = [
    "SlipRatio_FL", "SlipRatio_FR", "SlipRatio_RL", "SlipRatio_RR",
    "SlipAngle_FL", "SlipAngle_FR", "SlipAngle_RL", "SlipAngle_RR",
]
INCLUDE_TIMES = True
MIN_BASE = ((1+len(graphics_cols))+(1+len(physics_cols))+(1+len(acti_cols))) if INCLUDE_TIMES else (len(graphics_cols)+len(physics_cols)+len(acti_cols))
CALC_BLOCK = 15

ext_lock = threading.Lock()
ext_data_latest = None
data_received_once = threading.Event()
hotstart_request = threading.Event()
_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === ADD: mat44f <-> flat16 工具 ===
def _mat44f_to_flat16(m):
    return [float(m.M11), float(m.M12), float(m.M13), float(m.M14),
            float(m.M21), float(m.M22), float(m.M23), float(m.M24),
            float(m.M31), float(m.M32), float(m.M33), float(m.M34),
            float(m.M41), float(m.M42), float(m.M43), float(m.M44)]

def _flat16_to_mat44f(flat):
    m = pd.mat44f()
    (m.M11, m.M12, m.M13, m.M14,
     m.M21, m.M22, m.M23, m.M24,
     m.M31, m.M32, m.M33, m.M34,
     m.M41, m.M42, m.M43, m.M44) = map(float, flat)
    return m

# --- in optimizer_test.py ---
import json, time

_seq_counter = 0  # 放在文件顶层，给序列编号

def send_opt_controls_full(sock, addr,
                           U: np.ndarray,
                           elapsed_s: float,
                           sim_hz: float,
                           *,
                           verbose: bool = True,
                           marker: int | None = None,
                           send_mode: str = "raw",
                           meta: dict | None = None):
    """
    发送整段帧级控制序列 U (H×3)，不做时间对齐。
    每一帧仅包含 steer/gas/brake；整体附上 sim_hz 与 elapsed_ms。
    额外增加一个“唯一标志 marker”，用于接收端去重/判新。
    - 若未显式传入 marker，则使用函数级自增计数器生成（跨调用递增）。
    - 接收端建议优先使用 marker 判新，elapsed_ms 可作为辅助。

    返回：本次发送使用的 marker（便于日志/调试）
    """
    import json, time
    assert U.ndim == 2 and U.shape[1] == 3, "U 应为 (H,3)"
    H = U.shape[0]

    # —— 生成/继承 marker（单调递增，函数属性持久）——
    if marker is None:
        # 初始化函数属性计数器
        if not hasattr(send_opt_controls_full, "_marker_counter"):
            send_opt_controls_full._marker_counter = 0
        send_opt_controls_full._marker_counter += 1
        marker = int(send_opt_controls_full._marker_counter)
    else:
        # 兼容手动指定的 marker，同时维护计数器的单调性（可选）
        if not hasattr(send_opt_controls_full, "_marker_counter"):
            send_opt_controls_full._marker_counter = int(marker)
        else:
            send_opt_controls_full._marker_counter = max(
                int(send_opt_controls_full._marker_counter), int(marker)
            )

    # 裁剪 + 互斥（双保险；expand_controls 已做，这里再做一次以免中途调整过 U[0]）
    UU = np.empty_like(U, dtype=np.float32)
    UU[:,0] = np.clip(U[:,0], -1.0, 1.0)
    g = np.clip(U[:,1],  0.0, 1.0)
    b = np.clip(U[:,2],  0.0, 1.0)
    # 逐帧互斥，保持与 expand_controls 的逻辑一致
    gg = np.empty(H, dtype=np.float32)
    bb = np.empty(H, dtype=np.float32)
    for i in range(H):
        gi, bi = _mutual_exclusive_gb(float(g[i]), float(b[i]), thr=0.05)
        gg[i], bb[i] = gi, bi
    UU[:,1] = gg
    UU[:,2] = bb

    elapsed_ms = int(round(float(elapsed_s) * 1000.0))

    payload = {
        "type": "opt_seq",
        "sim_hz": float(sim_hz),
        "elapsed_ms": elapsed_ms,
        "marker": int(marker),  # ★ 新增唯一标志
        "verbose": verbose,
        # ★ 新增：发送模式标志，便于接收端解析（nodes / frames / raw）
        "send_mode": str(send_mode),
        # ★ 新增：序列长度（接收端不必再数 u_seq）
        "H": int(H),
        "u_seq": [
            {"steer": float(UU[i,0]),
             "gas":   float(UU[i,1]),
             "brake": float(UU[i,2])}
            for i in range(H)
        ],
    }

    # 可选附带更多元信息（group / horizon_steps / K / dt 等）
    if meta:
        try:
            payload["meta"] = dict(meta)
        except Exception:
            payload["meta"] = {"_meta_error": "meta not serializable"}

    # if verbose:
    #     print(f"[SEND] marker={marker}  mode={send_mode}  H={H}  sim_hz={sim_hz:.1f}  elapsed_ms={elapsed_ms}  "
    #           f"u0=({UU[0,0]:+.3f},{UU[0,1]:.3f},{UU[0,2]:.3f})  "
    #           f"u_last=({UU[-1,0]:+.3f},{UU[-1,1]:.3f},{UU[-1,2]:.3f})")

    sock.sendto(json.dumps(payload).encode("utf-8"), addr)
    return marker

def send_opt_controls_nodes(sock, addr,
                            nodes3: np.ndarray,
                            elapsed_s: float,
                            sim_hz: float,
                            *,
                            group: int,
                            horizon_steps: int,
                            verbose: bool = True,
                            marker: int | None = None):
    """保留原来的“发 K 个节点”方式（典型 K=15）。"""
    K = int(nodes3.shape[0])
    meta = {
        "group": int(group),
        "horizon_steps": int(horizon_steps),
        "K": int(K),
        "node_step": int(group),
    }
    return send_opt_controls_full(
        sock, addr, nodes3, elapsed_s, sim_hz,
        verbose=verbose, marker=marker,
        send_mode="nodes", meta=meta,
    )


def send_opt_controls_frames(sock, addr,
                             nodes3: np.ndarray,
                             elapsed_s: float,
                             sim_hz: float,
                             *,
                             group: int,
                             horizon_steps: int,
                             verbose: bool = True,
                             marker: int | None = None):
    """新：把节点序列 expand 成帧级序列（典型 H=150），再发送。"""
    U_full = expand_controls(nodes3, group=int(group), horizon_steps=int(horizon_steps))
    K = int(nodes3.shape[0])
    meta = {
        "group": int(group),
        "horizon_steps": int(horizon_steps),
        "K": int(K),
        "node_step": int(group),
    }
    return send_opt_controls_full(
        sock, addr, U_full, elapsed_s, sim_hz,
        verbose=verbose, marker=marker,
        send_mode="frames", meta=meta,
    )

def send_opt_controls_switch(sock, addr,
                             nodes3: np.ndarray,
                             elapsed_s: float,
                             sim_hz: float,
                             *,
                             send_mode: str,
                             group: int,
                             horizon_steps: int,
                             verbose: bool = True,
                             marker: int | None = None):
    """在两种发送方式间切换：

    - send_mode == "nodes": 直接发送 K 个节点
    - send_mode == "frames": expand 成 horizon_steps 帧后发送
    """
    mode = str(send_mode).lower().strip()
    if mode in ("node", "nodes", "k"):
        return send_opt_controls_nodes(
            sock, addr, nodes3, elapsed_s, sim_hz,
            group=group, horizon_steps=horizon_steps,
            verbose=verbose, marker=marker,
        )
    if mode in ("frame", "frames", "full", "h"):
        return send_opt_controls_frames(
            sock, addr, nodes3, elapsed_s, sim_hz,
            group=group, horizon_steps=horizon_steps,
            verbose=verbose, marker=marker,
        )
    raise ValueError(f"unknown send_mode={send_mode!r} (use 'nodes' or 'frames')")


def failure_multiplier(st,
                       w_collision: float = 10.0,
                       w_out: float = 6.0,
                       drift_div: float = 8.0,
                       w_drift_max: float = 4.0):
    """返回 (mult, collision, oot)"""
    collision = int(getattr(st, "collisionFlag", 0)) != 0
    oot       = int(getattr(st, "outOfTrackFlag", 0)) != 0
    driftNow  = int(getattr(st, "driftNow", 0))

    # 侧滑因子：你说 driftNow/8 决定，这里做成 1 + driftNow/8，并 clamp
    w_drift = 1.0 + (driftNow / float(drift_div))
    if w_drift > w_drift_max:
        w_drift = w_drift_max

    mult = 1.0
    if collision: mult *= w_collision
    if oot:       mult *= w_out
    # 注意：即使 driftNow=0，w_drift=1 不影响；有侧滑才>1
    mult *= w_drift

    return mult, collision, oot

def apply_mult_penalty(score_full: float, mult: float, bias: float = 1.0):
    """
    你希望“乘倍数放大惩罚”。如果 score_full 是负数，score*mult 会更负 -> OK；
    但如果 score_full 是正数，score*mult 会更大 -> 反而奖励，这是不对的。
    这个写法保证：mult>1 时一定让 score 变差（变小）。
    """
    if mult <= 1.0:
        return score_full
    return score_full - (abs(score_full) + bias) * (mult - 1.0)

def send_opt_controls(steer: float, gas: float, brake: float, alltime_sec: float, sim_hz: float):
    age_frames = int(round(alltime_sec * sim_hz))
    payload = {"steer": float(steer), "gas": float(gas), "brake": float(brake),
               "age_frames": age_frames, "sim_hz": sim_hz, "t": time.perf_counter()}
    _udp_sock.sendto(json.dumps(payload).encode("utf-8"), SEND_ADDR)

def data_udp_listener_loop(ip=DATA_UDP_IP, port=DATA_UDP_PORT):
    global ext_data_latest
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[DATA] UDP listener @ {ip}:{port}")
    try:
        while True:
            data, addr = sock.recvfrom(65535)
            if len(data) < 4*(MIN_BASE+1):
                continue
            total_floats = len(data)//4
            vals = struct.unpack("<"+"f"*total_floats, data[:total_floats*4])

            has_calc = (total_floats == (MIN_BASE+CALC_BLOCK+1))
            if not has_calc and (total_floats != (MIN_BASE+1)):
                has_calc = (total_floats - MIN_BASE - 1) >= CALC_BLOCK

            idx = 0
            parsed = {}
            if INCLUDE_TIMES:
                parsed["graphicstime"] = vals[idx]; idx += 1
            for c in graphics_cols:
                parsed[c] = vals[idx]; idx += 1
            if INCLUDE_TIMES:
                parsed["physicstime"] = vals[idx]; idx += 1
            for c in physics_cols:
                parsed[c] = vals[idx]; idx += 1
            if INCLUDE_TIMES:
                parsed["actitime"] = vals[idx]; idx += 1
            for c in acti_cols:
                parsed[c] = vals[idx]; idx += 1

            if has_calc and (total_floats - idx - 1) >= CALC_BLOCK:
                calc_names = [
                    "hub_FL_x","hub_FL_y","hub_FL_z",
                    "hub_FR_x","hub_FR_y","hub_FR_z",
                    "hub_RL_x","hub_RL_y","hub_RL_z",
                    "hub_RR_x","hub_RR_y","hub_RR_z",
                    "axle_center_x","axle_center_y","axle_center_z",
                ]
                for name in calc_names:
                    parsed[name] = vals[idx]; idx += 1

            parsed["hotstarttag"] = vals[-1]
            with ext_lock:
                ext_data_latest = parsed
            if not data_received_once.is_set():
                data_received_once.set()
            if parsed.get("hotstarttag", 1.0) < 0.5:
                hotstart_request.set()
    except Exception as e:
        print(f"[DATA] recv error: {e}")
    finally:
        try: sock.close()
        except: pass

# threading.Thread(target=data_udp_listener_loop, daemon=True).start()

# ================= hotstart =================
@dataclass
class HotstartParams:
    pose: tuple
    sethot: dict
    driver: dict
    snap: tuple | None = None  # 存 (body16, fuel16, hub1_16, hub2_16, axle16)

def build_hotstart_from_ext(ext):
    car_x = float(ext.get("carCoordinates_1", 0.0))
    car_y = float(ext.get("carCoordinates_2", 0.0))
    car_z = float(ext.get("carCoordinates_3", 0.0))
    heading = -float(ext.get("heading", 0.0))
    pitch   = -float(ext.get("pitch",   0.0))
    roll    = -float(ext.get("roll",    0.0))
    hub_FL = (float(ext.get("hub_FL_x",0.0)), float(ext.get("hub_FL_y",0.0)), float(ext.get("hub_FL_z",0.0)))
    hub_FR = (float(ext.get("hub_FR_x",0.0)), float(ext.get("hub_FR_y",0.0)), float(ext.get("hub_FR_z",0.0)))
    axle   = (float(ext.get("axle_center_x",0.0)), float(ext.get("axle_center_y",0.0)), float(ext.get("axle_center_z",0.0)))

    def f(name, d=0.0): return float(ext.get(name, d))
    wa = [f("wheelAngularSpeed_1"), f("wheelAngularSpeed_2"), f("wheelAngularSpeed_3"), f("wheelAngularSpeed_4")]
    slipR = [f("SlipRatio_FL"), f("SlipRatio_FR"), f("SlipRatio_RL"), f("SlipRatio_RR")]
    slipA = [math.radians(f("SlipAngle_FL")),math.radians(f("SlipAngle_FR")),math.radians(f("SlipAngle_RL")),math.radians(f("SlipAngle_RR")),]
    #print(slipA)
    tc = [f("tyreCoreTemperature_1",80.0), f("tyreCoreTemperature_2",80.0),
          f("tyreCoreTemperature_3",80.0), f("tyreCoreTemperature_4",80.0)]
    ti = [f("tyreTempI_1",75.0), f("tyreTempI_2",75.0), f("tyreTempI_3",75.0), f("tyreTempI_4",75.0)]
    tm = [f("tyreTempM_1",75.0), f("tyreTempM_2",75.0), f("tyreTempM_3",75.0), f("tyreTempM_4",75.0)]
    to = [f("tyreTempO_1",75.0), f("tyreTempO_2",75.0), f("tyreTempO_3",75.0), f("tyreTempO_4",75.0)]
    patch = [(ti[i]+tm[i]+to[i])/3.0 for i in range(4)]

    engineRPM = f("rpms", 4500.0)
    gear = int(round(ext.get("gear", 3)))
    vel = [f("velocity_1", 14.0), f("velocity_2", 0.0), f("velocity_3", 0.0)]
    brakeTemp = sum([f("brakeTemp_1",80.0), f("brakeTemp_2",80.0), f("brakeTemp_3",80.0), f("brakeTemp_4",80.0)])/4.0

    sethot = dict(
        newrootVelocity       = (wa[2]+wa[3])*4.778*2.022/2.0,
        newengineRPM          = engineRPM,
        newcurrentGear        = gear,
        newoutShaftLvelocity  = wa[2],
        newoutShaftRvelocity  = wa[3],
        newbraketemp          = brakeTemp,
        newVelocity           = vel,
        newslipAngleRAD       = slipA,
        newslipRatio          = slipR,
        newangularVelocity    = wa,
        newangularVelocityold = wa[:],
        newMz                 = [0.0, 0.0, 0.0, 0.0],
        newdirtyLevel         = [0.0, 0.0, 0.0, 0.0],
        newcoretemp           = tc,
        newpatchtemp          = patch,
    )
    driver = dict(
        steer=float(ext.get("steerAngle", 0.0)),
        gas  =float(ext.get("gas", 0.0)),
        brake=float(ext.get("brake", 0.0)),
        clutch=float(ext.get("clutch", 0.0))
    )
    return HotstartParams(
        pose=(car_x,car_y,car_z, heading, roll, pitch, *hub_FL, *hub_FR, *axle),
        sethot=sethot, driver=driver
    )



def _copy_state_matrices_to_snap(state):
    return (
        _mat44f_to_flat16(state.bodyMatrix),
        _mat44f_to_flat16(state.fuelTankyMatrix),
        _mat44f_to_flat16(state.hub1Matrix),
        _mat44f_to_flat16(state.hub2Matrix),
        _mat44f_to_flat16(state.strutBodyM1),
        _mat44f_to_flat16(state.strutBodyM2),
        _mat44f_to_flat16(state.axleMatrix),
    )


def _safe_f32(x, default=0.0):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return float(default)


def _state_arr4(state, name, fallback):
    try:
        arr = getattr(state, name)
        vals = [float(arr[i]) for i in range(4)]
        if len(vals) == 4 and all(math.isfinite(v) for v in vals):
            return vals
    except Exception:
        pass
    return [float(x) for x in fallback]


def build_hotstart_from_state(state, controls, fallback_hot: HotstartParams | None = None):
    fallback_sethot = fallback_hot.sethot if fallback_hot is not None else {}
    fallback_driver = fallback_hot.driver if fallback_hot is not None else {}

    wa = _state_arr4(state, 'tyreAngularSpeed', fallback_sethot.get('newangularVelocity', [0.0]*4))
    slipR = _state_arr4(state, 'tyreSlipRatio', fallback_sethot.get('newslipRatio', [0.0]*4))
    slipA = _state_arr4(state, 'slipAngleRAD', fallback_sethot.get('newslipAngleRAD', [0.0]*4))

    sethot = dict(
        newrootVelocity=_safe_f32(getattr(state, 'rootVelocity', None), fallback_sethot.get('newrootVelocity', 0.0)),
        newengineRPM=_safe_f32(getattr(state, 'engineRPM', None), fallback_sethot.get('newengineRPM', 0.0)),
        newcurrentGear=int(getattr(state, 'gear', fallback_sethot.get('newcurrentGear', 0))),
        newoutShaftLvelocity=_safe_f32(getattr(state, 'outShaftLvelocity', None), fallback_sethot.get('newoutShaftLvelocity', 0.0)),
        newoutShaftRvelocity=_safe_f32(getattr(state, 'outShaftRvelocity', None), fallback_sethot.get('newoutShaftRvelocity', 0.0)),
        newbraketemp=_safe_f32(getattr(state, 'brakeTemp', None), fallback_sethot.get('newbraketemp', 80.0)),
        newVelocity=[
            _safe_f32(getattr(state.velocity, 'x', None), fallback_sethot.get('newVelocity', [0.0,0.0,0.0])[0]),
            _safe_f32(getattr(state.velocity, 'y', None), fallback_sethot.get('newVelocity', [0.0,0.0,0.0])[1]),
            _safe_f32(getattr(state.velocity, 'z', None), fallback_sethot.get('newVelocity', [0.0,0.0,0.0])[2]),
        ],
        newslipAngleRAD=slipA,
        newslipRatio=slipR,
        newangularVelocity=wa,
        newangularVelocityold=list(fallback_sethot.get('newangularVelocityold', wa[:])),
        newMz=list(fallback_sethot.get('newMz', [0.0]*4)),
        newdirtyLevel=list(fallback_sethot.get('newdirtyLevel', [0.0]*4)),
        newcoretemp=list(fallback_sethot.get('newcoretemp', [80.0]*4)),
        newpatchtemp=list(fallback_sethot.get('newpatchtemp', [75.0]*4)),
    )
    driver = dict(
        steer=_safe_f32(getattr(controls, 'steer', None), fallback_driver.get('steer', 0.0)),
        gas=_safe_f32(getattr(controls, 'gas', None), fallback_driver.get('gas', 0.0)),
        brake=_safe_f32(getattr(controls, 'brake', None), fallback_driver.get('brake', 0.0)),
        clutch=_safe_f32(getattr(controls, 'clutch', None), fallback_driver.get('clutch', 1.0)),
    )
    return HotstartParams(
        pose=(
            _safe_f32(getattr(state.bodyPos, 'x', None), 0.0),
            _safe_f32(getattr(state.bodyPos, 'y', None), 0.0),
            _safe_f32(getattr(state.bodyPos, 'z', None), 0.0),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ),
        sethot=sethot,
        driver=driver,
        snap=_copy_state_matrices_to_snap(state),
    )


def apply_hotstart_main(sim, car, controls, hot: HotstartParams, dt: float):
    (b16, f16, h116, h216, s116, s216, a16) = hot.snap
    bodyM  = _flat16_to_mat44f(b16)
    fuelM  = _flat16_to_mat44f(f16)
    hub1M  = _flat16_to_mat44f(h116)
    hub2M  = _flat16_to_mat44f(h216)
    axleM  = _flat16_to_mat44f(a16)
    strutBodyM1 = _flat16_to_mat44f(s116)
    strutBodyM2 = _flat16_to_mat44f(s216)

    pd.applyWorldMatricesFast(sim, car, bodyM, fuelM, hub1M, hub2M, strutBodyM1, strutBodyM2, axleM, True, True)
    for _ in range(80):
        pd.setCarControls(sim, car, False, controls)
        pd.stepSimulator(sim, dt)

    S = hot.sethot
    pd.sethotstart(sim, car, False,
        S["newrootVelocity"], S["newengineRPM"], S["newcurrentGear"],
        S["newoutShaftLvelocity"], S["newoutShaftRvelocity"], S["newbraketemp"],
        S["newVelocity"],
        S["newslipAngleRAD"], S["newslipRatio"],
        S["newangularVelocity"], S["newangularVelocityold"],
        S["newMz"], S["newdirtyLevel"],
        S["newcoretemp"], S["newpatchtemp"]
    )

    controls.steer = float(hot.driver["steer"])
    controls.gas   = float(hot.driver["gas"])
    controls.brake = float(hot.driver["brake"])
    controls.clutch = float(hot.driver["clutch"])
    pd.setCarControls(sim, car, False, controls)
    pd.stepSimulator(sim, dt)


def build_mid_hotstart(sim, car, controls, state, hot0: HotstartParams, dt: float, prefix_steps: int):
    apply_hotstart_main(sim, car, controls, hot0, dt)
    pd.getCarState(sim, car, state)

    for _ in range(int(prefix_steps)):
        controls.steer = float(hot0.driver["steer"])
        controls.gas   = float(hot0.driver["gas"])
        controls.brake = float(hot0.driver["brake"])
        controls.clutch = float(hot0.driver["clutch"])
        pd.setCarControls(sim, car, False, controls)
        pd.stepSimulator(sim, dt)
        pd.getCarState(sim, car, state)

    return build_hotstart_from_state(state, controls, fallback_hot=hot0)

# ================= config & records =================
@dataclass
class SimCfg:
    track: str = DEFAULT_TRACK
    car:   str = DEFAULT_CAR
    dt:    float = 1.0/DEFAULT_SIM_HZ
    auto_clutch: bool = True
    auto_shift:  bool = True
    auto_blip:   bool = True

@dataclass
class OptCfg:
    horizon_steps: int = 300
    group: int = 1
    u_dims: int = 3
    max_iter: int = 10
    A_candidates: int = 192
    A_cand_step: int = 32
    A_cand_floor: int = 32

    A_elite_frac: float = 0.25
    A_noise_sigma: float = 2 ##参数
    A_temp: float = 2.5

    es_time_ms: float = 200.0
    es_rel_gain: float = 0.050
    es_mu_inf:  float = 0.030
    es_u0:      float = 0.020
    es_sigma_min: float = 0.020
    es_no_improve_rounds: int = 1

@dataclass
class Task:
    seed: int
    hot: 'HotstartParams'
    u_nodes: np.ndarray
    group: int
    horizon_steps: int
    dt: float

@dataclass
class Result:
    ok: bool
    score: float
    end_v: float = 0.0
    details: dict = field(default_factory=dict)

# ================= helpers =================
def expand_controls(u_nodes: np.ndarray, group: int, horizon_steps: int):
    K, D = u_nodes.shape
    u_nodes = u_nodes.copy()
    u_nodes[:,0] = np.clip(u_nodes[:,0], -1, 1)
    u_nodes[:,1] = np.clip(u_nodes[:,1],  0, 1)
    u_nodes[:,2] = np.clip(u_nodes[:,2],  0, 1)

    node_step = group
    full = np.zeros((horizon_steps, 3), dtype=np.float32)
    if K == 1:
        full[:] = u_nodes[0]
    else:
        for i in range(horizon_steps):
            a = i / node_step
            i0 = int(np.floor(a))
            i1 = min(i0+1, K-1)
            t = a - i0
            full[i] = (1-t)*u_nodes[i0] + t*u_nodes[i1]
    MAX_STEER_STEP = 0.01
    # steering 限坡
    for i in range(1, horizon_steps):
        d = full[i,0] - full[i-1,0]
        if abs(d) > MAX_STEER_STEP:
            full[i,0] = full[i-1,0] + MAX_STEER_STEP*np.sign(d)

    # 纵向坡度 + 互斥
    for i in range(1, horizon_steps):
        dg = full[i,1] - full[i-1,1]
        db = full[i,2] - full[i-1,2]
        if abs(dg) > MAX_STEER_STEP:
            full[i,1] = full[i-1,1] + MAX_STEER_STEP*np.sign(dg)
        if abs(db) > MAX_STEER_STEP:
            full[i,2] = full[i-1,2] + MAX_STEER_STEP*np.sign(db)
        full[i,1], full[i,2] = _mutual_exclusive_gb(full[i,1], full[i,2], thr=0.05)

    return full

def _mutual_exclusive_gb(g, b, thr=0.05):
    low, high = thr*0.9, thr*1.1
    g = float(np.clip(g, 0.0, 1.0)); b = float(np.clip(b, 0.0, 1.0))
    if g > high and b > high:
        if g >= b: b = 0.0
        else:      g = 0.0
    elif g < low: g = 0.0
    elif b < low: b = 0.0
    return g, b

import numpy as np

def steer_variants_smc(
    s_base: float,
    ey: float,            # 带符号横向误差(m)
    epsi: float,          # 航向误差(rad)
    v: float,             # m/s
    driftNow: int = 0,    # 0..8
    *,
    lambda_: float = 2.0,
    epsV: float = 2.0,
    ey_ok: float = 0.2,
    epsi_ok: float = np.deg2rad(3.0),
    v_ref: float = 30.0,
    v0: float = 35.0,
    delta_min: float = 0.03,
    delta_max: float = 0.30,
    steer_max_lo: float = 0.25,
    steer_max_hi: float = 0.65,
    # 去重阈值（太小会“看起来重复”，太大可能被迫大幅nudging）
    uniq_eps: float = 1e-3,
):
    # ---------- helpers ----------
    def clampf(x, lo, hi): 
        return lo if x < lo else hi if x > hi else x

    def sat01(x):
        return 0.0 if x < 0.0 else 1.0 if x > 1.0 else float(x)

    # 1) 滑模偏离度 -> rho in [0,1]
    v = float(max(0.0, v))
    s = abs(epsi) + lambda_ * (abs(ey) / (v + epsV))
    s0 = epsi_ok + lambda_ * (ey_ok / (v_ref + epsV))
    rho = float(np.clip(s / max(1e-6, s0), 0.0, 1.0))

    # 2) 速度门控 (0,1]
    gv = 1.0 / (1.0 + (v / max(1e-6, v0)) ** 2)

    # 3) 风险收缩
    R = float(np.clip(driftNow / 8.0, 0.0, 1.0))
    gr = 1.0 - 0.35 * R

    # 4) 探索半径
    delta = (delta_min + (delta_max - delta_min) * rho) * gv * gr
    delta = float(max(delta, 1e-6))

    # 5) 最大舵量上限
    steer_max = steer_max_lo + (steer_max_hi - steer_max_lo) * gv
    steer_max = float(np.clip(steer_max, 0.15, 1.0))

    # 6) s_base=0 时给一个起始方向
    if abs(s_base) < 1e-6:
        # 偏右(ey>0)->往左打(-)；偏左(ey<0)->往右打(+)
        dir_sign = -1.0 if ey > 0 else 1.0
        s_base = dir_sign * max(delta_min, 0.06)

    # 7) 恰好8个：围绕 s_base 生成 7 个 + s_base 本身
    #    注意：levels7 不含 0，避免与 s_base 重复
    levels7 = np.array([-1.0, -0.75, -0.50, 0.50, 0.75, 1.0, 1.25], dtype=np.float32)
    cand = [float(s_base)] + [float(s_base + lv * delta) for lv in levels7]  # len==8

    # clip
    cand = [float(np.clip(x, -steer_max, steer_max)) for x in cand]  # len==8

    # ---------- 去重与修复（保留你原来的稳健逻辑） ----------
    fixed = []
    for x in cand:
        x0 = x
        if fixed:
            tries = 0
            while any(abs(x - y) < uniq_eps for y in fixed) and tries < 50:
                step = (tries + 1) * uniq_eps * (1.0 if (tries % 2 == 0) else -1.0)
                x = float(np.clip(x0 + step, -steer_max, steer_max))
                tries += 1
        fixed.append(x)

    def _too_close(a, b, eps): 
        return abs(a - b) < eps

    ok = True
    for i in range(8):
        for j in range(i):
            if _too_close(fixed[i], fixed[j], uniq_eps):
                ok = False
                break
        if not ok:
            break

    if not ok:
        # 构造候选格点池：从两端到中间，优先“分散”
        grid = np.linspace(-steer_max, steer_max, 41, dtype=np.float32)
        repaired = []
        for x in fixed:
            if all(abs(x - y) >= uniq_eps for y in repaired):
                repaired.append(float(x))

        for g in grid:
            if len(repaired) >= 8:
                break
            gg = float(g)
            if all(abs(gg - y) >= uniq_eps for y in repaired):
                repaired.append(gg)

        if len(repaired) < 8:
            eps2 = max(uniq_eps * 0.2, 1e-5)
            for g in grid:
                if len(repaired) >= 8:
                    break
                gg = float(g)
                if all(abs(gg - y) >= eps2 for y in repaired):
                    repaired.append(gg)

        fixed = repaired[:8]

    if len(fixed) != 8:
        # 兜底：直接返回均匀8点（不会失败）
        fixed = [float(x) for x in np.linspace(-steer_max, steer_max, 8)]

    return np.array(fixed, dtype=np.float32)

def _lat_priority_from_ey_epsi(ey_avg, epsi_avg, *, ey0=2.5, epsi0=0.40, lam=1.2,
                              w_speed_floor=0.05):
    """
    rho_lat: 0..1  越大越需要“回线”
    w_steer = rho_lat
    w_speed = 1 - rho_lat，并给一个下限避免完全锁死纵向
    """
    ey = abs(float(ey_avg))
    ep = abs(float(epsi_avg))
    rho = (ep / max(1e-6, epsi0)) + lam * (ey / max(1e-6, ey0))
    rho_lat = float(np.clip(rho, 0.0, 1.0))
    w_steer = rho_lat
    w_speed = max(float(1.0 - rho_lat), float(w_speed_floor))
    # print(w_speed, w_steer)
    return rho_lat, w_steer, w_speed


import numpy as np

import numpy as np

def _pedal_variants_priority(
    mode: str,                # "gas" or "brake"
    p_base: float,
    *,
    v_last: float,
    drift_hint: int,          # 0..8
    v0: float = 50.0,
    kd: float = 0.60,         # drift 风险缩放（0.5~0.7）
    delta_gas0: float = 0.25,
    delta_brk0: float = 0.2,
    min_delta: float = 0.01,
    brk_cap: float = 1,
    enable_brake_emergency: bool = True,
    brake_emg_drift_thr: int = 6,
):
    assert mode in ("gas", "brake")

    p_base = float(np.clip(p_base, 0.0, 1.0))
    v = float(max(0.0, v_last))
    drift = int(np.clip(drift_hint, 0, 8))
    d = drift / 8.0  # 0..1

    # 速度风险（越快越小）
    gv = 1.0 / (1.0 + (v / max(1e-6, v0)) ** 2)   # (0,1]

    # 风险门控：速度越快、drift越大 => g_risk 越小
    g_risk = float(np.clip(gv * (1.0 - kd * d), 0.05, 1.0))

    # 探索半径
    delta0 = delta_gas0 if mode == "gas" else delta_brk0
    delta = float(max(min_delta, delta0 * g_risk))

    # ---- 新需求：围绕 base 生成 7 个（不含 0），再加上 base 本身 -> 8 个 ----
    # levels7 不含 0，避免与 p_base 重复
    levels7 = [-1.0, -0.75, -0.50, 0.50, 0.75, 1.0, 1.25]
    cand = [float(p_base)] + [float(p_base + lv * delta) for lv in levels7]  # len==8

    # clip
    if mode == "gas":
        cand = [float(np.clip(x, 0.0, 1.0)) for x in cand]
    else:
        cand = [float(np.clip(x, 0.0, brk_cap)) for x in cand]

    # 兜底：强制长度为 8（哪怕大量重复也允许）
    if len(cand) < 8:
        cand += [float(p_base)] * (8 - len(cand))
    return cand[:8]

def make_early_stop_condition(drift_thr: int = 1, drift_consec: int = 5):
    """
    Hard constraints (fail-fast):
      - collisionFlag != 0  -> fail immediately
      - outOfTrackFlag != 0 -> fail immediately
      - driftNow > drift_thr for drift_consec consecutive frames -> fail

    Notes:
      - drift_thr=2 means driftNow >= 3 will be counted (because condition is >2)
      - drift_consec=10 means 10 consecutive frames over threshold triggers failure
    """
    drift_run = 0  # consecutive counter inside one rollout

    def _to_int(x, default=0):
        try:
            return int(x)
        except Exception:
            return default

    def early_stop(st) -> bool:
        nonlocal drift_run

        # collision
        if _to_int(getattr(st, "collisionFlag", 0), 0) != 0:
            return True

        # off track
        if _to_int(getattr(st, "outOfTrackFlag", 0), 0) != 0:
            return True

        # drift: count consecutive frames
        dn = _to_int(getattr(st, "driftNow", 0), 0)

        if dn > drift_thr:
            drift_run += 1
        else:
            drift_run = 0

        if drift_run >= drift_consec:
            return True

        return False

    return early_stop


def _extract_speed_ms(st) -> float:
    v = getattr(st, "speedMS", None)
    if isinstance(v, (float,int)): return float(v)
    v = getattr(st, "speed", None)
    try:
        if v is not None and hasattr(v, "ms"):
            return float(v.ms())
    except Exception:
        pass
    v = getattr(st, "vAlongTrack", None)
    if isinstance(v, (float,int)): return float(abs(v))
    return 0.0

def _smooth_penalty(U: np.ndarray) -> float:
    """
    U: [H, 3] = (steer, gas, brake)
    返回一个“抖动成本”，越抖越大。
    """
    dU = np.diff(U, axis=0)   # [H-1, 3]
    # 给每个通道一些权重，比如 steer 比 gas 更敏感
    w = np.array([1.0, 0.3, 0.5], dtype=np.float32)  # 你可以以后慢慢调
    # sum (w * Δu^2)
    return float(np.sum((dU**2) * w))


def _smooth_nodes_temporal(cand2: np.ndarray,
                           max_steer_step_node: float = 0.10,
                           max_p_step_node: float = 0.15,
                           alpha_steer: float = 0.6,
                           alpha_pedal: float = 0.6):
    Ncand, K, D = cand2.shape
    if D != 2 or K <= 1:
        return
    for n in range(Ncand):
        # steer 通道
        prev = float(cand2[n, 0, 0])
        for k in range(1, K):
            raw = float(cand2[n, k, 0])
            smooth = alpha_steer * prev + (1.0 - alpha_steer) * raw
            delta = smooth - prev
            if abs(delta) > max_steer_step_node:
                smooth = prev + max_steer_step_node * np.sign(delta)
            cand2[n, k, 0] = smooth
            prev = smooth
        # pedal 通道
        prev = float(cand2[n, 0, 1])
        for k in range(1, K):
            raw = float(cand2[n, k, 1])
            smooth = alpha_pedal * prev + (1.0 - alpha_pedal) * raw
            delta = smooth - prev
            if abs(delta) > max_p_step_node:
                smooth = prev + max_p_step_node * np.sign(delta)
            cand2[n, k, 1] = smooth
            prev = smooth

# ================= worker with no early-stop =================
def worker_loop(work_q: mp.Queue, res_q: mp.Queue, sim_cfg: SimCfg):
    sim = pd.createSimulator(base_dir)
    pd.loadTrack(sim, sim_cfg.track)
    car = pd.addCar(sim, sim_cfg.car)
    pd.setCarAssists(sim, car, sim_cfg.auto_clutch, sim_cfg.auto_shift, sim_cfg.auto_blip)

    ctl = pd.CarControls(); st = pd.CarState()
    ctl.steer = 0.0; ctl.clutch = 0.0; ctl.brake = 0.0; ctl.gas = 0.0; ctl.requestedGearIndex = -1

    SETTLE_STEPS = 0

    def apply_hot(h: HotstartParams):

        (b16, f16, h116, h216, s116, s216, a16) = h.snap
        #print(b16)
        bodyM  = _flat16_to_mat44f(b16)
        fuelM  = _flat16_to_mat44f(f16)
        hub1M  = _flat16_to_mat44f(h116)
        hub2M  = _flat16_to_mat44f(h216)
        axleM = _flat16_to_mat44f(a16)
        strutBodyM1 = _flat16_to_mat44f(s116)
        strutBodyM2 = _flat16_to_mat44f(s216)
        pd.applyWorldMatricesFast(sim, car, bodyM, fuelM, hub1M, hub2M, strutBodyM1, strutBodyM2, axleM, True, True)
        pd.setCarControls(sim, car, False, ctl)
        pd.stepSimulator(sim, sim_cfg.dt)

        S = h.sethot

        pd.sethotstart(sim, car, False,
            S["newrootVelocity"], S["newengineRPM"], S["newcurrentGear"],
            S["newoutShaftLvelocity"], S["newoutShaftRvelocity"], S["newbraketemp"],
            S["newVelocity"],
            S["newslipAngleRAD"], S["newslipRatio"],
            S["newangularVelocity"], S["newangularVelocityold"],
            S["newMz"], S["newdirtyLevel"],
            S["newcoretemp"], S["newpatchtemp"]
        )

        ctl.steer = float(h.driver["steer"])
        ctl.gas   = float(h.driver["gas"])
        ctl.brake = float(h.driver["brake"])
        ctl.clutch = float(h.driver["clutch"])
        pd.setCarControls(sim, car, False, ctl)
        pd.stepSimulator(sim, t.dt)
        pd.getCarState(sim, car, st)

    def score_total_reward():
        pd.getCarState(sim, car, st)
        return float(st.totalReward)
    
    try:
        while True:
            cmd, payload = work_q.get()
            if cmd == "ROLL":
                t: Task = payload
                try:
                    apply_hot(t.hot)
                    base_total = score_total_reward()

                    U = expand_controls(t.u_nodes, t.group, t.horizon_steps)

                    for i in range(t.horizon_steps):
                        # print("worker horizon_steps =", t.horizon_steps, "group =", t.group)
                        ctl.steer = float(U[i,0])
                        ctl.gas   = float(U[i,1])
                        ctl.clutch  = 1
                        ctl.brake = float(U[i,2])
                        pd.setCarControls(sim, car, False, ctl)
                        pd.stepSimulator(sim, t.dt)
                        pd.getCarState(sim, car, st)

                    pd.getCarState(sim, car, st)
                    
                    score = float(st.totalReward) - base_total   # 分数越大越好
                    # print(U[i,0],U[i,1],U[i,2],score)
                    # time.sleep(1)       
                    details = {"vAlongTrack": float(st.vAlongTrack),
                            "beta": float(getattr(st, "betaRad", 0.0)),
                            "border": float(getattr(st, "borderMarginRatio", 1.0))}
                    res_q.put(Result(True, score, st.vAlongTrack, details))
                except Exception as e:
                    res_q.put(Result(False, -1e9, 0.0, {"err": str(e)}))
            elif cmd == "STOP":
                break
    finally:
        pd.shutAll()

# ================= coordinator (2D CEM) =================
class Coordinator:
    def __init__(self, sim_cfg: SimCfg, opt_cfg: OptCfg, workers: int):
        self.sim_cfg, self.opt = sim_cfg, opt_cfg
        self.workers = workers
        self.work_qs, self.res_qs, self.procs = [], [], []
        self._spawn_pool()
        self.K = max(1, int(np.ceil(self.opt.horizon_steps / self.opt.group)))

    def _spawn_pool(self):
        for _ in range(self.workers):
            wq, rq = mp.Queue(), mp.Queue()
            p = mp.Process(target=worker_loop, args=(wq, rq, self.sim_cfg))
            p.start()
            self.work_qs.append(wq); self.res_qs.append(rq); self.procs.append(p)

    def shutdown(self):
        for wq in self.work_qs: wq.put(("STOP", None))
        for p in self.procs: p.join()

    def _batched(self, tasks):
        N = len(tasks); results = [None]*N
        in_flight = {}; idx = 0
        for wid in range(min(self.workers, N)):
            self.work_qs[wid].put(("ROLL", tasks[idx])); in_flight[wid] = idx; idx += 1
        finished = 0
        while finished < N:
            for wid in list(in_flight.keys()):
                if not self.res_qs[wid].empty():
                    res = self.res_qs[wid].get()
                    t_idx = in_flight.pop(wid); results[t_idx] = res; finished += 1
                    if idx < N:
                        self.work_qs[wid].put(("ROLL", tasks[idx])); in_flight[wid] = idx; idx += 1
        return results

# ==================== 新版 baseline200_and_coarse32_hint ====================
def baseline200_and_coarse32_hint(coord: 'Coordinator', sim, car, controls, state, hot, seed):
    """
    300 步基线；按是否穿越与 eY/ePsi 平均值生成方向启发；
    纵向首选：基线若连续≥10 帧 slipAngle>0.1 → 刹车，否则油门；
    覆盖：framePenalty>0 → 刹车；若全程无 slipAngle>0.1 → 油门；
    严格二维冻结：冻结通道恒为 0.0。
    """
    # 1) 基线 160 步
    N_BASE = 160
    eY_seq, ePsi_seq, bvt_seq = [], [], []
    df_seq, v_seq, gfm_seq = [], [], []
    otf_seq =[]
    eY_last = None
    crossed = False
    cross_idx = None
    pd.setCarControls(sim, car, False, controls)
    pd.stepSimulator(sim, coord.sim_cfg.dt)
    for k in range(N_BASE):
        pd.setCarControls(sim, car, False, controls)
        pd.stepSimulator(sim, coord.sim_cfg.dt)
        pd.getCarState(sim, car, state)

        eY   = float(getattr(state, "eY", 0.0))
        ePsi = float(getattr(state, "ePsi", 0.0))
        bvt  = float(getattr(state, "bodyVsTrack", 0.0))
        df   = float(getattr(state, "driftNow", 0.0))
        vms  = float(getattr(state, "speedMS", 0.0))
        gfm   = float(getattr(state, "glastFramePenalty", 0.0))    
        otf = float(getattr(state, "outOfTrackFlag", 0.0))     

        eY_seq.append(eY); ePsi_seq.append(ePsi); bvt_seq.append(bvt)
        df_seq.append(df); v_seq.append(vms); gfm_seq.append(gfm); otf_seq.append(otf)

        if eY_last is not None:
            if (eY_last == 0.0) or (eY == 0.0) or (eY_last * eY < 0.0):
                crossed = True
                if cross_idx is None:
                    cross_idx = k
        eY_last = eY

    if crossed and cross_idx is not None and cross_idx > 0:
        ey_avg   = float(np.mean(eY_seq[:cross_idx]))
        epsi_avg = float(np.mean(ePsi_seq[:cross_idx]))
    else:
        ey_avg   = float(np.mean(eY_seq)) if eY_seq else 0.0
        epsi_avg = float(np.mean(ePsi_seq)) if ePsi_seq else 0.0

    # print(ey_avg,epsi_avg)

    v_last = float(v_seq[-1]) if v_seq else 0.0
    # 忽略前几帧异常 + 用 90 分位代替 max
    df_valid = df_seq[1:] if len(df_seq) > 1 else df_seq
    drift_hint = int(np.clip(np.percentile(df_valid, 90), 0, 8)) if df_valid else 0

    # 连续 ≥10 帧 df>2
    run = 0; max_run15 = 0
    for df in df_seq:
        if df > 0:   # <- 这里改
            run += 1
        else:
            max_run15 = max(max_run15, run); run = 0
    max_run15 = max(max_run15, run)
    
    otf_run = 0; otf_run10 = 0
    for otf in otf_seq:
        if otf > 0:   # <- 这里改
            otf_run += 1
        else:
            otf_run10 = max(otf_run10, otf_run); otf_run = 0
    otf_run10 = max(otf_run10, otf_run)
    # print(otf_run10)
    # 当前基线
    s_base = float(np.clip(controls.steer, -1.0, 1.0))
    g_base = float(np.clip(controls.gas,   0.0, 1.0))
    b_base = float(np.clip(controls.brake, 0.0, 1.0))

    # 2) 方向启发 + 首选油/刹
    left_side = (ey_avg < 0.0)
    if not crossed:
        steer_sign = +1.0 if left_side else -1.0
    else:
        steer_sign = -1.0 if left_side else +1.0

    def smoothstep(x, a, b):
        t = np.clip((x - a) / max(1e-6, (b - a)), 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    ey_norm   = float(np.clip(abs(ey_avg)   / 12.0, 0.0, 1.0))
    epsi_norm = float(np.clip(abs(epsi_avg) / 1.0,  0.0, 1.0))
    mix       = 0.6*ey_norm + 0.4*epsi_norm

    # 近轨迹不启用：mix < m_on 基本不偏置；mix > m_full 完全启用
    m_on, m_full = 0.10, 0.25
    gate = float(smoothstep(mix, m_on, m_full))   # 0..1

    gv = 1.0 / (1.0 + (v_last / 25.0)**2)     # v_last 是 m/s
    S_MAX = 0.25 + 0.20 * gv                  # 低速→0.45，高速→0.25
    S_MIN = 0.0
    s_mag  = gate * (S_MIN + (S_MAX - S_MIN) * mix)
    s_base = float(np.clip(s_base + steer_sign*s_mag, -1.0, 1.0))
    s_base = float(np.clip(controls.steer, -1.0, 1.0)) ##不启动steer修正

    # print(s_mag, s_base, steer_sign)

    # 首选油/刹（冻结值恒 0.0）
    prefer_brake_by_run15 = (max_run15 >= 15)
    prefer_brake_by_run10 = (otf_run10 >= 10)
    if prefer_brake_by_run15:
        lock_mode  = "freeze_gas";   frozen_val = 0.0
        active = "brake"; active_base = b_base
    else:
        if prefer_brake_by_run10:
            lock_mode  = "freeze_gas";   frozen_val = 0.0
            active = "brake"; active_base = b_base
        else:
            lock_mode  = "freeze_brake"; frozen_val = 0.0
            active = "gas";   active_base = g_base

    # print(lock_mode, max_run15)

    # 3) 覆盖规则
    gfm_avg = (sum(gfm_seq) / len(gfm_seq)) if gfm_seq else 0.0

    # 风险强度（0~1）
    risk_run15 = np.clip(max_run15 / 15.0, 0.0, 1.0)     # >=15 认为满风险
    risk_run3  = np.clip(otf_run10 / 10.0,  0.0, 1.0)      # >=3 认为满风险
    risk_mix   = gate                                   # 你已有的 gate: smoothstep(mix)

    risk = float(np.clip(0.50*risk_run15 + 0.35*risk_run3 + 0.15*risk_mix, 0.0, 1.0))

    # 风险刹车地板（自己调参：高速+高风险更大）
    brk_floor = 0.15 + 0.65 * risk   # risk=0 ->0.15, risk=1 ->0.80

    # 抬高基线，避免 driver.brake=0 把候选中心锁死
    b_base = float(max(b_base, brk_floor))

    # 抬高 cap：高风险允许更大刹车
    brk_cap = float(0.70 + 0.30 * risk)  # risk=1 -> 1.0

    if active == "brake":
        p_variants = _pedal_variants_priority(
            "brake",
            p_base=b_base,
            v_last=v_last,
            drift_hint=drift_hint,   # 0..8
            v0=50.0,
            kd=0.60,
            delta_brk0=0.20,
            min_delta=0.01,
            brk_cap=1.0,
        )
    else:
        p_variants = _pedal_variants_priority(
            "gas",
            p_base=g_base,
            v_last=v_last,
            drift_hint=drift_hint,   # 0..8
            v0=50.0,
            kd=0.60,
            delta_gas0=0.25,
            min_delta=0.01,
            brk_cap=1.0,
        )

    # 方向 8 档 + 组装 64 候选（被冻通道恒 0.0；warm-up 仅作用于自由维）
    s_values = steer_variants_smc(s_base=float(s_base),
    ey=ey_avg,
    epsi=epsi_avg,
    v=v_last,
    driftNow=drift_hint,
    )
    K = coord.K
    # print(s_values,p_variants)
    assert len(s_values) == 8 and len(p_variants) == 8
    cand = []
    warmup = 1

    if lock_mode == "freeze_brake":
        to_seq = lambda s, p: (s, float(np.clip(p, 0.0, 1.0)), 0.0)   # brake 恒 0
    else:
        to_seq = lambda s, p: (s, 0.0, float(np.clip(p, 0.0, 1.0)))   # gas   恒 0

    for s0 in s_values:
        s0 = float(np.clip(s0, -1.0, 1.0))
        for p0 in p_variants:
            p0 = float(np.clip(p0, 0.0, 1.0))
            s1, g1, b1 = to_seq(s0, p0)
            g1, b1 = _mutual_exclusive_gb(g1, b1, thr=0.05)

            seq = np.zeros((K, 3), dtype=np.float32)
            if lock_mode == "freeze_brake":
                # 自由维是 gas；brake 恒 0
                g0 = float(g1) * warmup
                seq[0,:] = [s1, g0, 0.0]
                if K >= 2:
                    g1_ = float(g1) * (0.5*warmup + 0.5)
                    seq[1,:] = [s1, g1_, 0.0]
                    if K > 2:
                        seq[2:,:] = [s1, g1, 0.0]
            else:
                # 自由维是 brake；gas 恒 0
                b0 = float(b1) * warmup
                seq[0,:] = [s1, 0.0, b0]
                if K >= 2:
                    b1_ = float(b1) * (0.5*warmup + 0.5)
                    seq[1,:] = [s1, 0.0, b1_]
                    if K > 2:
                        seq[2:,:] = [s1, 0.0, b1]
            cand.append(seq)

    cand = np.stack(cand, axis=0)

    # 5) 并行评估并返回首节点 hint
    tasks = [Task(int(20000 + i), hot, cand[i], coord.opt.group, coord.opt.horizon_steps, coord.sim_cfg.dt)
             for i in range(cand.shape[0])]
    res = coord._batched(tasks)

    ok_idx = [i for i, r in enumerate(res) if r.ok]
    if not ok_idx:
        hint_u0 = (float(s_base), float(g_base), float(b_base))
        # 返回 lock_mode 与 frozen_val=0.0，确保后续严格二维
        return hint_u0 + (lock_mode, 0.0)

    # --- Top-K(=6) 精英 + 均值 ---
    ok_scores = np.array([res[i].score for i in ok_idx], dtype=np.float64)

    # 按 score 从高到低排序，得到对应 cand 的索引
    order = np.argsort(-ok_scores)               # ok_idx 内部排序
    K_TOP = 1
    k = min(K_TOP, len(order))
    top_ok_pos = order[:k]                       # ok_idx 中的位置
    top_cand_idx = [ok_idx[p] for p in top_ok_pos]  # cand 的真实索引

    top_scores = [float(res[i].score) for i in top_cand_idx]

    # 计算 Top-k 的 node0 平均控制（精英均值）
    top_u0 = cand[top_cand_idx, 0, :]           # shape=(k,3)
    mean_u0 = np.mean(top_u0, axis=0)

    # 返回“精英均值”作为 hint（比单点 best 更稳）
    hint_u0 = (float(mean_u0[0]), float(mean_u0[1]), float(mean_u0[2]))
    return hint_u0 + (lock_mode, 0.0)


# ================= main =================
def main():   #####无论是setcontrol还是sethot等等，如果程序卡住，那就执行一次stepsimulator
    print("[MAIN] enter main()", flush=True)
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=32)
    ap.add_argument("--sim-hz", type=float, default=DEFAULT_SIM_HZ)
    ap.add_argument("--track",  default=DEFAULT_TRACK)
    ap.add_argument("--car",    default=DEFAULT_CAR)
    ap.add_argument("--horizon", type=int, default=160)
    ap.add_argument("--group",   type=int, default=10)
        # 控制发送方式：
    # - nodes  : 保留原来的 K 个节点发送（典型 K=15）
    # - frames : 把节点展开成 horizon 帧后发送（典型 150/200 帧）
    ap.add_argument("--send-mode", choices=["nodes", "frames"], default="nodes")
    ap.add_argument("--iters",   type=int, default=10)
    ap.add_argument("--view", action="store_true", default=False)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--es-ms", type=float, default=None)
    ap.add_argument("--es-gain", type=float, default=None)
    ap.add_argument("--es-mu", type=float, default=None)
    ap.add_argument("--es-u0", type=float, default=None)
    ap.add_argument("--es-sigma", type=float, default=None)
    ap.add_argument("--es-noimp", type=int, default=None)
    args = ap.parse_args()

    if current_process().name == "MainProcess":
        threading.Thread(
            target=data_udp_listener_loop,
            kwargs=dict(ip=DATA_UDP_IP, port=DATA_UDP_PORT),
            daemon=True
        ).start()

    print(f"[MAIN] UDP listener thread started on {DATA_UDP_IP}:{DATA_UDP_PORT}", flush=True)
    pd.setLogFile(os.path.join(base_dir, 'projectd.log'), True)
    mp.set_start_method("spawn", force=True)

    sim_rate = 1.0/args.sim_hz

    if args.view:
        pd.launchPlaygroundInOwnThread(base_dir, 1)

    sim = pd.createSimulator(base_dir)
    pd.loadTrack(sim, args.track)
    car = pd.addCar(sim, args.car)
    pd.setCarAssists(sim, car, True, True, True)
    controls = pd.CarControls(); state = pd.CarState()
    controls.steer = 0.0; controls.clutch=1.0; controls.brake=0.0; controls.gas=0.0

    if args.view:
        while not pd.isPlaygroundInitialized(): time.sleep(0.5)
        pd.setRenderHz(int(args.sim_hz), False)
        pd.setActiveSimulator(sim, False); pd.setActiveCar(car, True, True)

    full_horizon = int(args.horizon)
    prefix_steps = 50
    opt_horizon = max(1, full_horizon // 2)
    opt_group = max(1, int(round(float(args.group))))

    sim_cfg = SimCfg(track=args.track, car=args.car, dt=sim_rate)
    opt_cfg = OptCfg(horizon_steps=opt_horizon, group=opt_group, max_iter=args.iters)

    if args.es_ms is not None:     opt_cfg.es_time_ms = float(args.es_ms)
    if args.es_gain is not None:   opt_cfg.es_rel_gain = float(args.es_gain)
    if args.es_mu is not None:     opt_cfg.es_mu_inf = float(args.es_mu)
    if args.es_u0 is not None:     opt_cfg.es_u0 = float(args.es_u0)
    if args.es_sigma is not None:  opt_cfg.es_sigma_min = float(args.es_sigma)
    if args.es_noimp is not None:  opt_cfg.es_no_improve_rounds = int(args.es_noimp)

    coord = Coordinator(sim_cfg, opt_cfg, workers=args.workers)
    print(f"[MAIN] full_horizon={full_horizon}  prefix_steps={prefix_steps}  opt_horizon={opt_horizon}  opt_group={opt_group}  K={coord.K}", flush=True)

    print("[MAIN] waiting UDP hotstart...]")
    have_snap_main = False
    snap_mats_main = None
    prev_applied = (0.0, 0.0, 0.0)

    try:
        while True:
            hotstart_request.wait()
            t0 = time.perf_counter()
            with ext_lock:
                ext = dict(ext_data_latest) if ext_data_latest else None
            if not ext:
                hotstart_request.clear(); continue
            hot0 = build_hotstart_from_ext(ext)

            if not have_snap_main:
                (x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az) = hot0.pose
                pd.teleportCarToPose(sim, car, x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az)
                for _ in range(300):
                    controls.steer  = 0
                    controls.clutch = 1.0
                    controls.brake  = 0
                    controls.gas    = 0
                    pd.setCarControls(sim, car, False, controls)
                    pd.stepSimulator(sim, sim_rate)
                pd.getCarState(sim, car, state)
                snap_mats_main = _copy_state_matrices_to_snap(state)
            else:
                have_snap_main = False

            hot0.snap = snap_mats_main
            hot = build_mid_hotstart(sim, car, controls, state, hot0, sim_rate, prefix_steps)

            controls.steer = float(hot.driver["steer"])
            controls.gas   = float(hot.driver["gas"])
            controls.brake = float(hot.driver["brake"])
            controls.clutch = float(hot.driver["clutch"])
            pd.getCarState(sim, car, state)

            # 基线 + 粗优化（新版，返回 lock_mode 与 frozen_val=0.0）
            # print(controls.steer,controls.gas,controls.brake)
            sgb = baseline200_and_coarse32_hint(coord, sim, car, controls, state, hot, args.seed)
            hint_u0 = sgb[:3]; lock_mode = sgb[3]; frozen_val = sgb[4]  # frozen_val 恒为 0.0

            # print(f"cuyouhua {hint_u0} {lock_mode} {frozen_val}")

            # 2D → 3D，再展成全长；被冻通道恒 0
            K = coord.K
            
            nodes3 = np.zeros((K,3), dtype=np.float32)
            if lock_mode == "freeze_gas":
                nodes3[:,0] = hint_u0[0]
                nodes3[:,1] = 0.0
                nodes3[:,2] = hint_u0[2]
            else:
                nodes3[:,0] = hint_u0[0]
                nodes3[:,1] = hint_u0[1]
                nodes3[:,2] = 0.0
            
            nodes16 = np.zeros((16, 3), dtype=np.float32)
            nodes16[:8, :] = [float(hot.driver["steer"]), float(hot.driver["gas"]), float(hot.driver["brake"])]
            nodes16[8:, :] = nodes3   # nodes3 shape = (8,3)
            t1 = time.perf_counter()
            elapsed_s = (t1 - t0)
            # print(state.slipAngleRAD)

            # print(elapsed_s)

            send_opt_controls_switch(
                _udp_sock, SEND_ADDR,
                nodes16, elapsed_s,
                sim_hz=args.sim_hz,
                send_mode=args.send_mode,
                group=opt_group,
                horizon_steps=opt_horizon,
                verbose=False,
            )

            hotstart_request.clear()

    finally:
        coord.shutdown()
        pd.shutAll()

if __name__ == "__main__":
    mp.freeze_support()
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        try: input("Press <Enter> to exit...")
        except: pass
