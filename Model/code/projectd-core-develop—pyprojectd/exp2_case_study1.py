# projectd_demo_original.py  — with Hotstart integration
import os
import sys
import site
import time
import threading
import socket
import struct
from dataclasses import dataclass
from typing import Dict, Any, Optional, List, Tuple
import csv
import numpy as np
import math

sim_hz = 333.0
sim_rate = 1.0 / sim_hz
smooth_controls = False

teleport_on_hit = False
teleport_off_track = False

auto_clutch = True
auto_shift = True
auto_blip = True

track_name = 'ks_silverstone1967'
car_model = 'ks_toyota_ae86_drift'
# car_model = 'ks_toyota_supra_mkiv_drift'

car_tunes = {
    "ks_toyota_ae86_drift": {
        "FRONT_BIAS": 55.0,
        "DIFF_POWER": 30.0,
        "DIFF_COAST": 30.0,
        "FINAL_RATIO": 5.0,
        "PRESSURE_LF": 28.0,
        "PRESSURE_RF": 28.0,
        "PRESSURE_LR": 28.0,
        "PRESSURE_RR": 28.0,
    },
    "ks_toyota_supra_mkiv_drift": {
        "FRONT_BIAS": 55.0,
        "DIFF_POWER": 90.0,
        "DIFF_COAST": 90.0,
        "FINAL_RATIO": 5.0,
        "TURBO_0": 100.0,
        "TURBO_1": 100.0,
        "PRESSURE_LF": 28.0,
        "PRESSURE_RF": 28.0,
        "PRESSURE_LR": 28.0,
        "PRESSURE_RR": 28.0,
    }
}

base_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
bin_dir = os.path.join(base_dir, 'bin')
site.addsitedir(bin_dir)
os.add_dll_directory(bin_dir)

logs_dir = os.path.join(base_dir, 'logs')
os.makedirs(logs_dir, exist_ok=True)
csv_path = os.path.join(logs_dir, f"drift_slip_{time.strftime('%Y%m%d_%H%M%S')}.csv")

import PyProjectD as pd
pd.setLogFile(os.path.join(base_dir, 'projectd.log'), True)

# -------------------- Hotstart integration --------------------
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
CALC_BLOCK = 15
MIN_BASE = ((1 + len(graphics_cols)) + (1 + len(physics_cols)) + (1 + len(acti_cols))) if INCLUDE_TIMES else (len(graphics_cols) + len(physics_cols) + len(acti_cols))

DATA_UDP_IP, DATA_UDP_PORT = "0.0.0.0", 5005
RETURN_UDP_ADDR = ("127.0.0.1", 56060)

PREDICT_HORIZONS = (100, 200, 300)
PREDICT_IN_PARALLEL = False
FLAG_RUN_FRAMES = 10
DRIFT_RUN_FRAMES = 20
SLIP_SUM_THRESHOLD = 0.4
USE_ABS_SLIP_SUM = True

ext_lock = threading.Lock()
ext_data_latest: Optional[Dict[str, Any]] = None
hotstart_request = threading.Event()

latest_prediction_details: Dict[int, Dict[str, Any]] = {}
hotstart_counter = 0
return_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

@dataclass
class HotstartParams:
    pose: tuple
    sethot: dict
    driver: dict

@dataclass
class PredictionUnit:
    sim: Any
    car: Any

def _f(d: Dict[str, Any], name: str, default=0.0) -> float:
    try:
        return float(d.get(name, default))
    except Exception:
        return float(default)

def build_hotstart_from_ext(ext: Dict[str, Any]) -> HotstartParams:
    car_x = _f(ext, "carCoordinates_1", 0.0)
    car_y = _f(ext, "carCoordinates_2", 0.0)
    car_z = _f(ext, "carCoordinates_3", 0.0)
    heading = -_f(ext, "heading", 0.0)
    pitch = -_f(ext, "pitch", 0.0)
    roll = -_f(ext, "roll", 0.0)

    wa = [_f(ext, "wheelAngularSpeed_1"),
          _f(ext, "wheelAngularSpeed_2"),
          _f(ext, "wheelAngularSpeed_3"),
          _f(ext, "wheelAngularSpeed_4")]
    slipR = [_f(ext, "SlipRatio_FL"), _f(ext, "SlipRatio_FR"),
             _f(ext, "SlipRatio_RL"), _f(ext, "SlipRatio_RR")]

    slipA = [
        math.radians(_f(ext, "SlipAngle_FL")),
        math.radians(_f(ext, "SlipAngle_FR")),
        math.radians(_f(ext, "SlipAngle_RL")),
        math.radians(_f(ext, "SlipAngle_RR")),
    ]
    tc = [_f(ext, "tyreCoreTemperature_1", 80.0), _f(ext, "tyreCoreTemperature_2", 80.0),
          _f(ext, "tyreCoreTemperature_3", 80.0), _f(ext, "tyreCoreTemperature_4", 80.0)]
    ti = [_f(ext, "tyreTempI_1", 75.0), _f(ext, "tyreTempI_2", 75.0),
          _f(ext, "tyreTempI_3", 75.0), _f(ext, "tyreTempI_4", 75.0)]
    tm = [_f(ext, "tyreTempM_1", 75.0), _f(ext, "tyreTempM_2", 75.0),
          _f(ext, "tyreTempM_3", 75.0), _f(ext, "tyreTempM_4", 75.0)]
    to = [_f(ext, "tyreTempO_1", 75.0), _f(ext, "tyreTempO_2", 75.0),
          _f(ext, "tyreTempO_3", 75.0), _f(ext, "tyreTempO_4", 75.0)]
    patch = [(ti[i] + tm[i] + to[i]) / 3.0 for i in range(4)]

    engineRPM = _f(ext, "rpms", 4500.0)
    gear = int(round(ext.get("gear", 3)))
    vel = [_f(ext, "velocity_1", 14.0), _f(ext, "velocity_2", 0.0), _f(ext, "velocity_3", 0.0)]
    brakeTemp = sum([_f(ext, "brakeTemp_1", 80.0), _f(ext, "brakeTemp_2", 80.0),
                     _f(ext, "brakeTemp_3", 80.0), _f(ext, "brakeTemp_4", 80.0)]) / 4.0

    sethot = dict(
        newrootVelocity=(wa[2] + wa[3]) * 4.778 * 2.022 / 2.0,
        newengineRPM=engineRPM,
        newcurrentGear=gear,
        newoutShaftLvelocity=wa[2],
        newoutShaftRvelocity=wa[3],
        newbraketemp=brakeTemp,
        newVelocity=vel,
        newslipAngleRAD=slipA,
        newslipRatio=slipR,
        newangularVelocity=wa,
        newangularVelocityold=wa[:],
        newMz=[0.0, 0.0, 0.0, 0.0],
        newdirtyLevel=[0.0, 0.0, 0.0, 0.0],
        newcoretemp=tc,
        newpatchtemp=patch,
    )
    driver = dict(
        steer=_f(ext, "steerAngle", 0.0),
        gas=_f(ext, "gas", 0.0),
        brake=_f(ext, "brake", 0.0),
        clutch=float(ext.get("clutch", 0.0))
    )
    return HotstartParams(
        pose=(car_x, car_y, car_z, heading, roll, pitch, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        sethot=sethot,
        driver=driver
    )

def data_udp_listener_loop(ip=DATA_UDP_IP, port=DATA_UDP_PORT):
    """接收二进制 float 流，解析到 dict；末尾 float 为 hotstarttag<0.5 则触发热启动。"""
    global ext_data_latest
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[DATA] UDP listener @ {ip}:{port}")
    try:
        while True:
            data, _ = sock.recvfrom(65535)
            if len(data) < 4 * (MIN_BASE + 1):
                continue
            total_floats = len(data) // 4
            vals = struct.unpack("<" + "f" * total_floats, data[:total_floats * 4])

            has_calc = (total_floats == (MIN_BASE + CALC_BLOCK + 1))
            if not has_calc and (total_floats != (MIN_BASE + 1)):
                has_calc = (total_floats - MIN_BASE - 1) >= CALC_BLOCK

            idx = 0
            parsed: Dict[str, float] = {}
            if INCLUDE_TIMES:
                parsed["graphicstime"] = vals[idx]
                idx += 1
            for c in graphics_cols:
                parsed[c] = vals[idx]
                idx += 1
            if INCLUDE_TIMES:
                parsed["physicstime"] = vals[idx]
                idx += 1
            for c in physics_cols:
                parsed[c] = vals[idx]
                idx += 1
            if INCLUDE_TIMES:
                parsed["actitime"] = vals[idx]
                idx += 1
            for c in acti_cols:
                parsed[c] = vals[idx]
                idx += 1

            if has_calc and (total_floats - idx - 1) >= CALC_BLOCK:
                calc_names = [
                    "hub_FL_x", "hub_FL_y", "hub_FL_z",
                    "hub_FR_x", "hub_FR_y", "hub_FR_z",
                    "hub_RL_x", "hub_RL_y", "hub_RL_z",
                    "hub_RR_x", "hub_RR_y", "hub_RR_z",
                    "axle_center_x", "axle_center_y", "axle_center_z",
                ]
                for name in calc_names:
                    parsed[name] = vals[idx]
                    idx += 1

            parsed["hotstarttag"] = vals[-1]
            with ext_lock:
                ext_data_latest = parsed
            if parsed.get("hotstarttag", 1.0) < 0.5:
                hotstart_request.set()
    except Exception as e:
        print(f"[DATA] recv error: {e}")
    finally:
        try:
            sock.close()
        except Exception:
            pass

def clone_mat44(src):
    dst = pd.mat44f()
    dst.M11 = src.M11; dst.M12 = src.M12; dst.M13 = src.M13; dst.M14 = src.M14
    dst.M21 = src.M21; dst.M22 = src.M22; dst.M23 = src.M23; dst.M24 = src.M24
    dst.M31 = src.M31; dst.M32 = src.M32; dst.M33 = src.M33; dst.M34 = src.M34
    dst.M41 = src.M41; dst.M42 = src.M42; dst.M43 = src.M43; dst.M44 = src.M44
    return dst

def capture_world_snapshot(state_obj):
    return {
        "body": clone_mat44(state_obj.bodyMatrix),
        "fuel": clone_mat44(state_obj.fuelTankyMatrix),
        "hub1": clone_mat44(state_obj.hub1Matrix),
        "hub2": clone_mat44(state_obj.hub2Matrix),
        "strut1": clone_mat44(state_obj.strutBodyM1),
        "strut2": clone_mat44(state_obj.strutBodyM2),
        "axle": clone_mat44(state_obj.axleMatrix),
    }

def apply_snapshot_and_hotstart(unit: PredictionUnit, hot: HotstartParams, snapshot: Dict[str, Any], controls_obj, state_obj):
    pd.applyWorldMatricesFast(
        unit.sim, unit.car,
        snapshot["body"], snapshot["fuel"], snapshot["hub1"], snapshot["hub2"],
        snapshot["strut1"], snapshot["strut2"], snapshot["axle"], True, True
    )

    S = hot.sethot
    pd.sethotstart(
        unit.sim, unit.car, False,
        S["newrootVelocity"], S["newengineRPM"], S["newcurrentGear"],
        S["newoutShaftLvelocity"], S["newoutShaftRvelocity"], S["newbraketemp"],
        S["newVelocity"],
        S["newslipAngleRAD"], S["newslipRatio"],
        S["newangularVelocity"], S["newangularVelocityold"],
        S["newMz"], S["newdirtyLevel"],
        S["newcoretemp"], S["newpatchtemp"]
    )

    controls_obj.steer = float(hot.driver["steer"])
    controls_obj.clutch = float(hot.driver["clutch"])
    controls_obj.brake = float(hot.driver["brake"])
    controls_obj.gas = float(hot.driver["gas"])
    pd.setCarControls(unit.sim, unit.car, smooth_controls, controls_obj)
    pd.stepSimulator(unit.sim, sim_rate)
    pd.getCarState(unit.sim, unit.car, state_obj)

def has_consecutive_positive(seq: List[float], run_len: int) -> int:
    run = 0
    for v in seq:
        if float(v) > 0.0:
            run += 1
            if run >= run_len:
                return 1
        else:
            run = 0
    return 0

def has_consecutive_slip(frames: List[List[float]], run_len: int, threshold: float, use_abs: bool = False) -> int:
    run = 0
    for slip4 in frames:
        if use_abs:
            slip_sum = float(sum(abs(x) for x in slip4))
        else:
            slip_sum = float(sum(slip4))
        if slip_sum > threshold:
            run += 1
            if run >= run_len:
                return 1
        else:
            run = 0
    return 0

def summarize_prediction(records: Dict[str, Any]) -> List[int]:
    # 这里返回的是“该 horizon 的最终判定结果”，不是逐帧原始序列。
    # 例如 100 horizon 会基于 100 帧 outOfTrackFlag / collisionFlag / slipAngleRAD
    # 压缩得到 [out_flag, drift_flag] 2个汇总标志。
    out_flag = has_consecutive_positive(records["outOfTrackFlag"], FLAG_RUN_FRAMES)
    drift_flag = has_consecutive_slip(records["slipAngleRAD"], DRIFT_RUN_FRAMES, SLIP_SUM_THRESHOLD, USE_ABS_SLIP_SUM)
    return [out_flag, drift_flag]

def run_prediction(unit: PredictionUnit, hot: HotstartParams, snapshot: Dict[str, Any], horizon: int) -> Dict[str, Any]:
    state_local = pd.CarState()
    controls_local = pd.CarControls()
    controls_local.gas = 0.0
    controls_local.clutch = 1.0

    apply_snapshot_and_hotstart(unit, hot, snapshot, controls_local, state_local)

    out_track_seq: List[float] = []
    slip_seq: List[List[float]] = []

    steer_cmd = float(hot.driver["steer"])
    brake_cmd = float(hot.driver["brake"])
    gas_cmd = float(hot.driver["gas"])

    for _ in range(horizon):
        controls_local.steer = steer_cmd
        controls_local.clutch = 1.0
        controls_local.brake = brake_cmd
        controls_local.gas = gas_cmd
        pd.setCarControls(unit.sim, unit.car, smooth_controls, controls_local)
        pd.stepSimulator(unit.sim, sim_rate)
        pd.getCarState(unit.sim, unit.car, state_local)

        out_track_seq.append(float(getattr(state_local, "outOfTrackFlag", 0.0)))

        slip4 = list(state_local.slipAngleRAD)
        slip_seq.append([float(slip4[0]), float(slip4[1]), float(slip4[2]), float(slip4[3])])

    records = {
        "horizon": horizon,
        "outOfTrackFlag": out_track_seq,
        "slipAngleRAD": slip_seq,
    }
    return records


def slice_prediction_records(full_records: Dict[str, Any], horizon: int) -> Dict[str, Any]:
    records = {
        "horizon": horizon,
        "outOfTrackFlag": full_records["outOfTrackFlag"][:horizon],
        "slipAngleRAD": full_records["slipAngleRAD"][:horizon],
    }
    records["summary"] = summarize_prediction(records)
    return records

def send_result_matrix_udp(matrix3x3: List[List[int]]):
    flat = [float(v) for row in matrix3x3 for v in row]
    payload = struct.pack("<6f", *flat)
    return_sock.sendto(payload, RETURN_UDP_ADDR)

def create_prediction_unit() -> PredictionUnit:
    sim_local = pd.createSimulator(base_dir)
    pd.loadTrack(sim_local, track_name)
    car_local = pd.addCar(sim_local, car_model)
    pd.teleportCarToSpline(sim_local, car_local, 0.01)
    pd.setCarAutoTeleport(sim_local, car_local, teleport_on_hit, teleport_off_track, 1)
    pd.setCarAssists(sim_local, car_local, auto_clutch, auto_shift, auto_blip)
    return PredictionUnit(sim=sim_local, car=car_local)

def build_prediction_units(parallel_enabled: bool) -> Dict[int, PredictionUnit]:
    if parallel_enabled:
        return {h: create_prediction_unit() for h in PREDICT_HORIZONS}
    return {PREDICT_HORIZONS[0]: PredictionUnit(sim=sim, car=car)}

def stabilize_main_sim(hot: HotstartParams, state_obj, controls_obj) -> Dict[str, Any]:
    (x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az) = hot.pose
    pd.teleportCarToPose(sim, car, x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az)
     
    snapshot = None

    for i in range(300):
        controls_obj.steer = 0.0
        controls_obj.clutch = 1.0
        controls_obj.brake = 0.0
        controls_obj.gas = 0.0
        pd.setCarControls(sim, car, smooth_controls, controls_obj)
        pd.stepSimulator(sim, sim_rate)


    pd.getCarState(sim, car, state_obj)
    

    snapshot = capture_world_snapshot(state_obj)

    return snapshot

def run_all_predictions(hot: HotstartParams, snapshot: Dict[str, Any], predict_units: Dict[int, PredictionUnit]) -> Dict[int, Dict[str, Any]]:
    results: Dict[int, Dict[str, Any]] = {}

    max_horizon = max(PREDICT_HORIZONS)

    if PREDICT_IN_PARALLEL:
        unit = predict_units.get(max_horizon)
        if unit is None:
            unit = next(iter(predict_units.values()))
    else:
        unit = PredictionUnit(sim=sim, car=car)

    full_records = run_prediction(unit, hot, snapshot, max_horizon)

    for horizon in PREDICT_HORIZONS:
        results[horizon] = slice_prediction_records(full_records, horizon)

    return results

sim = pd.createSimulator(base_dir)
pd.loadTrack(sim, track_name)
car = pd.addCar(sim, car_model)

pd.teleportCarToSpline(sim, car, 0.01)
pd.setCarAutoTeleport(sim, car, teleport_on_hit, teleport_off_track, 1)
pd.setCarAssists(sim, car, auto_clutch, auto_shift, auto_blip)

predict_units = build_prediction_units(PREDICT_IN_PARALLEL)

state = pd.CarState()
controls = pd.CarControls()
controls.gas = 0.0
controls.clutch = 1.0

pd.writeLog('MAIN LOOP')

threading.Thread(target=data_udp_listener_loop, daemon=True).start()

try:
    while True:
        if hotstart_request.is_set():
            t0 = time.perf_counter()
            with ext_lock:
                ext = dict(ext_data_latest) if ext_data_latest else None

            if ext:

                hotstart_counter += 1
                hot = build_hotstart_from_ext(ext)

                stable_snapshot = stabilize_main_sim(hot, state, controls)

                prediction_results = run_all_predictions(hot, stable_snapshot, predict_units)

                latest_prediction_details = prediction_results

                matrix3x3 = [prediction_results[h]["summary"] for h in PREDICT_HORIZONS]

                # print(f"[RESULT] hotstart #{hotstart_counter}: {matrix3x3}")
                send_result_matrix_udp(matrix3x3)

                t1 = time.perf_counter()
                # print(t1-t0)

            hotstart_request.clear()
        else:
            time.sleep(0.001)

except KeyboardInterrupt:
    pass
finally:
    try:
        return_sock.close()
    except Exception:
        pass
    pd.shutAll()

