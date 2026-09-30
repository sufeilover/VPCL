# projectd_demo_original.py  — with Hotstart integration
import os
import sys
import site
import time
import atexit
import argparse
import threading
import socket
import struct
from dataclasses import dataclass
from typing import Dict, Any, Optional, List, Tuple
import csv
import numpy as np
import math

try:
    sys.setswitchinterval(0.001)
except Exception:
    pass

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
# track_name = 'ks_barcelona'

DRIVETRAIN_CONFIGS = {
    # Source: Assetto Corsa car data/drivetrain.ini.
    # ProjectD/AC gear indices: 0=R, 1=N, 2=1st, 3=2nd, ...
    "ks_toyota_ae86_drift": {
        "final_ratio": 4.778,
        "gear_ratios": {
            0: -3.727,
            1: 0.0,
            2: 3.587,
            3: 2.022,
            4: 1.384,
            5: 1.000,
            6: 0.861,
        },
    },
    "ks_toyota_supra_mkiv_drift": {
        "final_ratio": 3.5714,
        "gear_ratios": {
            0: -3.192,
            1: 0.0,
            2: 3.724,
            3: 2.246,
            4: 1.541,
            5: 1.205,
            6: 1.000,
            7: 0.818,
        },
    },
}

USE_DWB_RESTORE = (car_model == "ks_toyota_supra_mkiv_drift")

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

PREDICT_HORIZONS = (80, 160, 240)
PREDICT_IN_PARALLEL = False
FLAG_RUN_FRAMES = 10
DRIFT_SLIP_ANGLE_DEG = 9.3
DRIFT_SLIP_ANGLE_RAD = math.radians(DRIFT_SLIP_ANGLE_DEG)
DRIFT_RUN_FRAME_OPTIONS = (5, 10, 20)
DRIFT_RUN_FRAMES = 10
TYRE_LABELS = ("FL", "FR", "RL", "RR")

ext_lock = threading.Lock()
ext_data_latest: Optional[Dict[str, Any]] = None
hotstart_request = threading.Event()

latest_prediction_details: Dict[int, Dict[str, Any]] = {}
hotstart_counter = 0
return_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

timing_lock = threading.Lock()
timing_log_dir = os.path.join(logs_dir, "timing_logs")
timing_optimizer_csv = os.path.join(timing_log_dir, f"timing_optimizer_{time.strftime('%Y%m%d_%H%M%S')}.csv")
timing_optimizer_fields = [
    "request_id",
    "recv_parse_ms",
    "event_wait_ms",
    "hotstart_ms",
    "stabilize_ms",
    "predict_to_80_ms",
    "predict_to_160_ms",
    "predict_to_240_ms",
    "predict_total_ms",
    "matrix_build_ms",
    "return_send_ms",
    "optimizer_processing_ms",
    "optimizer_total_ms",
    "matrix",
]
timing_optimizer_rows: List[Dict[str, Any]] = []
timing_optimizer_flushed: int = 0
timing_optimizer_enabled = False

drift_analysis_lock = threading.Lock()
drift_analysis_log_dir = os.path.join(logs_dir, "drift_analysis")
drift_analysis_csv = os.path.join(
    drift_analysis_log_dir,
    f"drift_sensitivity_{time.strftime('%Y%m%d_%H%M%S')}.csv",
)
drift_analysis_fields = [
    "request_id",
    "car_model",
    "horizon_frames",
    "horizon_seconds",
    "threshold_deg",
    "threshold_rad",
    "selected_run_frames",
    "out_run_frames",
    "out_flag",
    "selected_drift_flag",
    "drift_flag_5",
    "drift_flag_10",
    "drift_flag_20",
    "first_trigger_frame_5",
    "first_trigger_frame_10",
    "first_trigger_frame_20",
    "max_run_FL",
    "max_run_FR",
    "max_run_RL",
    "max_run_RR",
    "frames_above_FL",
    "frames_above_FR",
    "frames_above_RL",
    "frames_above_RR",
    "max_abs_slip_deg_FL",
    "max_abs_slip_deg_FR",
    "max_abs_slip_deg_RL",
    "max_abs_slip_deg_RR",
]
drift_analysis_rows: List[Dict[str, Any]] = []
drift_analysis_flushed: int = 0
drift_analysis_enabled = True


def write_optimizer_timing_row(row: Dict[str, Any]) -> None:
    if not timing_optimizer_enabled:
        return
    with timing_lock:
        if not timing_optimizer_enabled:
            return
        timing_optimizer_rows.append(dict(row))


def flush_optimizer_timing_rows() -> None:
    global timing_optimizer_flushed
    if not timing_optimizer_enabled:
        return
    with timing_lock:
        if not timing_optimizer_enabled:
            return
        rows = timing_optimizer_rows[timing_optimizer_flushed:]
        if not rows:
            return
        os.makedirs(timing_log_dir, exist_ok=True)
        need_header = not os.path.exists(timing_optimizer_csv) or os.path.getsize(timing_optimizer_csv) == 0
        with open(timing_optimizer_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=timing_optimizer_fields, extrasaction="ignore")
            if need_header:
                writer.writeheader()
            writer.writerows(rows)
        timing_optimizer_flushed += len(rows)


def write_drift_analysis_rows(
    request_id: int,
    prediction_results: Dict[int, Dict[str, Any]],
) -> None:
    if not drift_analysis_enabled:
        return

    rows: List[Dict[str, Any]] = []
    for horizon in PREDICT_HORIZONS:
        records = prediction_results[horizon]
        analysis = records["drift_analysis"]
        max_runs = analysis["max_run_per_tyre"]
        frames_above = analysis["frames_above_per_tyre"]
        max_abs_deg = analysis["max_abs_slip_deg_per_tyre"]
        drift_flags = analysis["drift_flags"]
        first_trigger = analysis["first_trigger_frames"]

        row: Dict[str, Any] = {
            "request_id": request_id,
            "car_model": car_model,
            "horizon_frames": horizon,
            "horizon_seconds": horizon * sim_rate,
            "threshold_deg": DRIFT_SLIP_ANGLE_DEG,
            "threshold_rad": DRIFT_SLIP_ANGLE_RAD,
            "selected_run_frames": DRIFT_RUN_FRAMES,
            "out_run_frames": FLAG_RUN_FRAMES,
            "out_flag": records["summary"][0],
            "selected_drift_flag": records["summary"][1],
        }
        for option in DRIFT_RUN_FRAME_OPTIONS:
            row[f"drift_flag_{option}"] = drift_flags[option]
            trigger_frame = first_trigger[option]
            row[f"first_trigger_frame_{option}"] = (
                trigger_frame if trigger_frame is not None else ""
            )
        for tyre_index, label in enumerate(TYRE_LABELS):
            row[f"max_run_{label}"] = max_runs[tyre_index]
            row[f"frames_above_{label}"] = frames_above[tyre_index]
            row[f"max_abs_slip_deg_{label}"] = max_abs_deg[tyre_index]
        rows.append(row)

    with drift_analysis_lock:
        if drift_analysis_enabled:
            drift_analysis_rows.extend(rows)


def flush_drift_analysis_rows() -> None:
    global drift_analysis_flushed
    if not drift_analysis_enabled:
        return
    with drift_analysis_lock:
        rows = drift_analysis_rows[drift_analysis_flushed:]
        if not rows:
            return
        os.makedirs(drift_analysis_log_dir, exist_ok=True)
        need_header = (
            not os.path.exists(drift_analysis_csv)
            or os.path.getsize(drift_analysis_csv) == 0
        )
        with open(drift_analysis_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=drift_analysis_fields,
                extrasaction="ignore",
            )
            if need_header:
                writer.writeheader()
            writer.writerows(rows)
        drift_analysis_flushed += len(rows)


atexit.register(flush_optimizer_timing_rows)
atexit.register(flush_drift_analysis_rows)

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


def calculate_root_velocity(model: str, gear: int, rear_left: float,
                            rear_right: float, engine_rpm: float) -> float:
    config = DRIVETRAIN_CONFIGS.get(model)
    if config is None:
        raise ValueError(f"Unsupported drivetrain configuration: {model}")

    gear_ratio = config["gear_ratios"].get(gear)
    if gear_ratio is None or gear_ratio == 0.0:
        # Neutral or an unavailable gear cannot be inferred from wheel speed.
        return float(engine_rpm) * (2.0 * math.pi / 60.0)

    rear_axle_speed = (float(rear_left) + float(rear_right)) / 2.0
    return rear_axle_speed * float(config["final_ratio"]) * float(gear_ratio)


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
        newrootVelocity=calculate_root_velocity(
            car_model, gear, wa[2], wa[3], engineRPM
        ),
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
            recv_perf = time.perf_counter()
            parse_start = recv_perf
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

            if (total_floats - idx - 1) >= 1:
                try:
                    parsed["request_id"] = int(round(vals[idx]))
                except Exception:
                    parsed["request_id"] = -1
                idx += 1

            parsed["hotstarttag"] = vals[-1]
            parse_end = time.perf_counter()
            parsed["_timing_recv_perf"] = recv_perf
            parsed["_timing_parse_ms"] = (parse_end - parse_start) * 1000.0
            should_trigger = parsed.get("hotstarttag", 1.0) < 0.5
            with ext_lock:
                if should_trigger:
                    parsed["_timing_event_set_perf"] = time.perf_counter()
                ext_data_latest = parsed
            if should_trigger:
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

def apply_sethotstart(sim_obj, car_obj, S):
    pd.sethotstart(
        sim_obj, car_obj, False,
        S["newrootVelocity"], S["newengineRPM"], S["newcurrentGear"],
        S["newoutShaftLvelocity"], S["newoutShaftRvelocity"], S["newbraketemp"],
        S["newVelocity"],
        S["newslipAngleRAD"], S["newslipRatio"],
        S["newangularVelocity"], S["newangularVelocityold"],
        S["newMz"], S["newdirtyLevel"],
        S["newcoretemp"], S["newpatchtemp"],
    )

def apply_snapshot_and_hotstart(unit: PredictionUnit, hot: HotstartParams,
                                snapshot: Dict[str, Any], controls_obj, state_obj):
    if USE_DWB_RESTORE:
        # Supra: 恢复车身、燃油箱和四个 DWB hub 的真实状态。
        restore_dwb_rollout_s0(unit.sim, unit.car, snapshot)
    else:
        # AE86: 保留原有 STRUT + AXLE 的恢复逻辑。
        pd.applyWorldMatricesFast(
            unit.sim, unit.car,
            snapshot["body"], snapshot["fuel"], snapshot["hub1"], snapshot["hub2"],
            snapshot["strut1"], snapshot["strut2"], snapshot["axle"], True, True
        )
        apply_sethotstart(unit.sim, unit.car, hot.sethot)

    controls_obj.steer = float(hot.driver["steer"])
    controls_obj.clutch = float(hot.driver["clutch"])
    controls_obj.brake = float(hot.driver["brake"])
    controls_obj.gas = float(hot.driver["gas"])
    pd.setCarControls(unit.sim, unit.car, smooth_controls, controls_obj)

    # Supra 的 snapshot 已是“热启动后第一步”的状态，不重复推进。
    if not USE_DWB_RESTORE:
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

def analyze_single_tyre_slip_runs(
    frames: List[List[float]],
    threshold_rad: float,
) -> Dict[str, Any]:
    current_runs = [0, 0, 0, 0]
    max_runs = [0, 0, 0, 0]
    frames_above = [0, 0, 0, 0]
    max_abs_rad = [0.0, 0.0, 0.0, 0.0]
    first_trigger_frames = {
        option: None for option in DRIFT_RUN_FRAME_OPTIONS
    }

    for frame_index, slip4 in enumerate(frames, start=1):
        for tyre_index in range(4):
            angle = float(slip4[tyre_index])
            if math.isfinite(angle):
                abs_angle = abs(angle)
                max_abs_rad[tyre_index] = max(
                    max_abs_rad[tyre_index], abs_angle
                )
            else:
                abs_angle = 0.0

            if abs_angle >= threshold_rad:
                current_runs[tyre_index] += 1
                frames_above[tyre_index] += 1
                max_runs[tyre_index] = max(
                    max_runs[tyre_index], current_runs[tyre_index]
                )
            else:
                current_runs[tyre_index] = 0

        for option in DRIFT_RUN_FRAME_OPTIONS:
            if (
                first_trigger_frames[option] is None
                and any(run >= option for run in current_runs)
            ):
                first_trigger_frames[option] = frame_index

    drift_flags = {
        option: int(any(run >= option for run in max_runs))
        for option in DRIFT_RUN_FRAME_OPTIONS
    }
    return {
        "drift_flags": drift_flags,
        "first_trigger_frames": first_trigger_frames,
        "max_run_per_tyre": max_runs,
        "frames_above_per_tyre": frames_above,
        "max_abs_slip_deg_per_tyre": [
            math.degrees(angle) for angle in max_abs_rad
        ],
    }

def summarize_prediction(records: Dict[str, Any]) -> List[int]:
    # 这里返回的是“该 horizon 的最终判定结果”，不是逐帧原始序列。
    out_flag = has_consecutive_positive(records["outOfTrackFlag"], FLAG_RUN_FRAMES)
    analysis = records["drift_analysis"]
    drift_flag = analysis["drift_flags"][DRIFT_RUN_FRAMES]
    return [out_flag, drift_flag]

def run_prediction(
    unit: PredictionUnit,
    hot: HotstartParams,
    snapshot: Dict[str, Any],
    horizon: int,
    timing_marks: Optional[Dict[int, float]] = None,
) -> Dict[str, Any]:
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

    for step in range(1, horizon + 1):
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
        if timing_marks is not None and step in PREDICT_HORIZONS and step not in timing_marks:
            timing_marks[step] = time.perf_counter()

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
    records["drift_analysis"] = analyze_single_tyre_slip_runs(
        records["slipAngleRAD"],
        DRIFT_SLIP_ANGLE_RAD,
    )
    records["summary"] = summarize_prediction(records)
    return records

def send_result_matrix_udp(matrix3x3: List[List[int]], request_id: Optional[int] = None, optimizer_processing_ms: Optional[float] = None):
    flat = [float(v) for row in matrix3x3 for v in row]
    req_val = float(request_id) if request_id is not None else -1.0
    opt_ms_val = float(optimizer_processing_ms) if optimizer_processing_ms is not None else -1.0
    payload = struct.pack("<8f", *(flat + [req_val, opt_ms_val]))
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
    (x, y, z, hd, rl, pt, flx, fly, flz,
     frx, fry, frz, ax, ay, az) = hot.pose

    pd.teleportCarToPose(
        sim, car, x, y, z, hd, rl, pt,
        flx, fly, flz, frx, fry, frz, ax, ay, az
    )

    for _ in range(300):
        controls_obj.steer = 0.0
        controls_obj.clutch = 1.0
        controls_obj.brake = 0.0
        controls_obj.gas = 0.0
        pd.setCarControls(sim, car, smooth_controls, controls_obj)
        pd.stepSimulator(sim, sim_rate)

    pd.getCarState(sim, car, state_obj)

    if not USE_DWB_RESTORE:
        return capture_world_snapshot(state_obj)

    # Supra: 先写入 AC 热启动状态，再推进一步，形成可复用 rollout S0。
    apply_sethotstart(sim, car, hot.sethot)

    controls_obj.steer = float(hot.driver["steer"])
    controls_obj.clutch = float(hot.driver["clutch"])
    controls_obj.brake = float(hot.driver["brake"])
    controls_obj.gas = float(hot.driver["gas"])
    pd.setCarControls(sim, car, smooth_controls, controls_obj)
    pd.stepSimulator(sim, sim_rate)
    pd.getCarState(sim, car, state_obj)

    return capture_dwb_rollout_s0(state_obj, hot)

def run_all_predictions(
    hot: HotstartParams,
    snapshot: Dict[str, Any],
    predict_units: Dict[int, PredictionUnit],
    timing_marks: Optional[Dict[int, float]] = None,
) -> Dict[int, Dict[str, Any]]:
    results: Dict[int, Dict[str, Any]] = {}

    max_horizon = max(PREDICT_HORIZONS)

    if PREDICT_IN_PARALLEL:
        unit = predict_units.get(max_horizon)
        if unit is None:
            unit = next(iter(predict_units.values()))
    else:
        unit = PredictionUnit(sim=sim, car=car)

    full_records = run_prediction(unit, hot, snapshot, max_horizon, timing_marks)

    for horizon in PREDICT_HORIZONS:
        results[horizon] = slice_prediction_records(full_records, horizon)

    return results

ap = argparse.ArgumentParser()
ap.add_argument(
    "--timing",
    choices=["on", "off"],
    default="on",
    help="timing CSV switch: on=record optimizer timing rows, off=disabled (default)",
)
ap.add_argument(
    "--drift-run-frames",
    type=int,
    choices=DRIFT_RUN_FRAME_OPTIONS,
    default=10,
    help="consecutive frames required for one tyre at or above 9.3 deg",
)
ap.add_argument(
    "--drift-analysis",
    choices=["on", "off"],
    default="off",
    help="buffer 5/10/20-frame drift sensitivity rows and write CSV on exit",
)
args = ap.parse_args()
timing_optimizer_enabled = (args.timing == "on")
DRIFT_RUN_FRAMES = args.drift_run_frames
drift_analysis_enabled = (args.drift_analysis == "on")

def _vec3_to_list(v):
    return [
        float(getattr(v, "x", 0.0)),
        float(getattr(v, "y", 0.0)),
        float(getattr(v, "z", 0.0)),
    ]

_MAT_FIELDS = (
    "M11", "M12", "M13", "M14",
    "M21", "M22", "M23", "M24",
    "M31", "M32", "M33", "M34",
    "M41", "M42", "M43", "M44",
)


def _copy_mat44(src):
    dst = pd.mat44f()
    for field in _MAT_FIELDS:
        setattr(dst, field, float(getattr(src, field)))
    return dst


def _state_arr4(state, name):
    values = getattr(state, name)
    return [float(values[i]) for i in range(4)]


def _make_rollout_sethot(state, fallback_hot):
    base = fallback_hot.sethot
    wheel_speed = _state_arr4(state, "tyreAngularSpeed")

    return {
        "newrootVelocity": float(state.rootVelocity),
        "newengineRPM": float(state.engineRPM),
        "newcurrentGear": int(state.gear),
        "newoutShaftLvelocity": float(state.outShaftLvelocity),
        "newoutShaftRvelocity": float(state.outShaftRvelocity),
        "newbraketemp": float(base["newbraketemp"]),
        "newVelocity": _vec3_to_list(state.velocity),
        "newslipAngleRAD": _state_arr4(state, "slipAngleRAD"),
        "newslipRatio": _state_arr4(state, "tyreSlipRatio"),
        "newangularVelocity": wheel_speed,
        "newangularVelocityold": wheel_speed[:],
        "newMz": list(base["newMz"]),
        "newdirtyLevel": list(base["newdirtyLevel"]),
        "newcoretemp": list(base["newcoretemp"]),
        "newpatchtemp": list(base["newpatchtemp"]),
    }


def capture_dwb_rollout_s0(state, fallback_hot):
    return {
        "body_matrix": _copy_mat44(state.bodyMatrix),
        "fuel_matrix": _copy_mat44(state.fuelTankyMatrix),
        "body_velocity": _vec3_to_list(state.velocity),
        "body_angular_velocity": _vec3_to_list(state.angularVelocity),
        "hub_matrices": [
            _copy_mat44(state.dwbHubRawMatrix[i]) for i in range(4)
        ],
        "hub_velocities": [
            _vec3_to_list(state.dwbHubVelocity[i]) for i in range(4)
        ],
        "hub_angular_velocities": [
            _vec3_to_list(state.dwbHubAngularVelocity[i]) for i in range(4)
        ],
        "sethot": _make_rollout_sethot(state, fallback_hot),
    }


def restore_dwb_rollout_s0(sim, car, snapshot):
    S = snapshot["sethot"]

    pd.sethotstart(
        sim, car, False,
        S["newrootVelocity"], S["newengineRPM"], S["newcurrentGear"],
        S["newoutShaftLvelocity"], S["newoutShaftRvelocity"], S["newbraketemp"],
        S["newVelocity"],
        S["newslipAngleRAD"], S["newslipRatio"],
        S["newangularVelocity"], S["newangularVelocityold"],
        S["newMz"], S["newdirtyLevel"],
        S["newcoretemp"], S["newpatchtemp"],
    )

    pd.restoreDWBHotstartState(
        sim, car,
        snapshot["body_matrix"],
        snapshot["fuel_matrix"],
        snapshot["body_velocity"],
        snapshot["body_angular_velocity"],
        snapshot["hub_matrices"],
        snapshot["hub_velocities"],
        snapshot["hub_angular_velocities"],
    )

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
if timing_optimizer_enabled:
    print(f"[TIMING] optimizer CSV -> {timing_optimizer_csv}", flush=True)
else:
    print("[TIMING] optimizer timing disabled by default", flush=True)
if drift_analysis_enabled:
    print(
        f"[DRIFT] selected={DRIFT_RUN_FRAMES} frames, "
        f"threshold={DRIFT_SLIP_ANGLE_DEG} deg; CSV -> {drift_analysis_csv}",
        flush=True,
    )
else:
    print("[DRIFT] sensitivity CSV disabled", flush=True)

threading.Thread(target=data_udp_listener_loop, daemon=True).start()

try:
    while True:
        if not hotstart_request.wait(timeout=0.1):
            continue
        pickup_perf = time.perf_counter()
        with ext_lock:
            ext = dict(ext_data_latest) if ext_data_latest else None
        hotstart_request.clear()

        if ext:

            hotstart_counter += 1
            request_id = int(ext.get("request_id", -1))
            recv_perf = ext.get("_timing_recv_perf", pickup_perf)
            event_set_perf = ext.get("_timing_event_set_perf")
            timing_base = recv_perf if isinstance(recv_perf, float) else pickup_perf

            hotstart_start = time.perf_counter()
            hot = build_hotstart_from_ext(ext)
            hotstart_end = time.perf_counter()

            stabilize_start = time.perf_counter()
            stable_snapshot = stabilize_main_sim(hot, state, controls)
            stabilize_end = time.perf_counter()

            prediction_marks: Dict[int, float] = {}
            predict_start = time.perf_counter()
            prediction_results = run_all_predictions(hot, stable_snapshot, predict_units, prediction_marks)
            predict_end = time.perf_counter()

            latest_prediction_details = prediction_results

            matrix_start = time.perf_counter()
            matrix3x3 = [prediction_results[h]["summary"] for h in PREDICT_HORIZONS]
            matrix_end = time.perf_counter()

            # print(f"[RESULT] hotstart #{hotstart_counter}: {matrix3x3}")
            return_send_start = time.perf_counter()
            optimizer_processing_ms = (return_send_start - timing_base) * 1000.0
            send_result_matrix_udp(matrix3x3, request_id, optimizer_processing_ms)
            return_send_end = time.perf_counter()

            write_optimizer_timing_row(
                {
                    "request_id": request_id,
                    "recv_parse_ms": ext.get("_timing_parse_ms", ""),
                    "event_wait_ms": ((pickup_perf - event_set_perf) * 1000.0) if isinstance(event_set_perf, float) else "",
                    "hotstart_ms": (hotstart_end - hotstart_start) * 1000.0,
                    "stabilize_ms": (stabilize_end - stabilize_start) * 1000.0,
                    "predict_to_80_ms": ((prediction_marks[80] - predict_start) * 1000.0) if 80 in prediction_marks else "",
                    "predict_to_160_ms": ((prediction_marks[160] - predict_start) * 1000.0) if 160 in prediction_marks else "",
                    "predict_to_240_ms": ((prediction_marks[240] - predict_start) * 1000.0) if 240 in prediction_marks else "",
                    "predict_total_ms": (predict_end - predict_start) * 1000.0,
                    "matrix_build_ms": (matrix_end - matrix_start) * 1000.0,
                    "return_send_ms": (return_send_end - return_send_start) * 1000.0,
                    "optimizer_processing_ms": optimizer_processing_ms,
                    "optimizer_total_ms": (return_send_end - timing_base) * 1000.0,
                    "matrix": matrix3x3,
                }
            )
            write_drift_analysis_rows(request_id, prediction_results)

except KeyboardInterrupt:
    pass
finally:
    flush_optimizer_timing_rows()
    flush_drift_analysis_rows()
    try:
        return_sock.close()
    except Exception:
        pass
    pd.shutAll()

