# projectd_demo_original.py  — with Hotstart integration
import argparse
import hashlib
import json
import os
import sys
import site
import time
# >>> NEW
import threading
import socket
import struct
from dataclasses import dataclass
from typing import Dict, Any, Optional
import csv
import numpy as np  # 若文件顶部未导入

from collections import deque
import math

from ai_predictor_runtime import AINetworkPredictor

sim_hz = 333.0
sim_rate = 1.0 / sim_hz
smooth_controls = False
AI_ROLLOUT_STEPS = 150
AI_FEATURE_WINDOW_FLAG = 9727.0
AI_FEATURE_HEADER_FLOATS = 11
ACK_ADDR = ("127.0.0.1", 56060)

_parser = argparse.ArgumentParser(description="ProjectD AI-baseline consistency validation")
_parser.add_argument("--ai-push", type=int, choices=[80, 85, 90, 95], default=80)
_parser.add_argument("--baseline-mode", choices=["ai", "zoh"], default="ai")
_parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
_parser.add_argument("--model-dir", default=None)
_parser.add_argument("--output-root", default=r"C:\Users\itserv\OneDrive - Tshwane University of Technology\AC-simulator\code\FAMILIAR-TEST\py-g29-ac-simulator\Integratedcode\game\data\ai_model_validation")
ARGS = _parser.parse_args()
AI_PUSH = int(ARGS.ai_push)
BASELINE_MODE = str(ARGS.baseline_mode)

teleport_on_hit = False
teleport_off_track = False

auto_clutch = True
auto_shift = True
auto_blip = True

track_name = 'ks_silverstone1967'
car_model = 'ks_toyota_ae86_drift'
#car_model = 'ks_toyota_supra_mkiv_drift'

car_tunes = {
    "ks_toyota_ae86_drift" : {
        "FRONT_BIAS" : 55.0,
        "DIFF_POWER" : 30.0,
        "DIFF_COAST" : 30.0,
        "FINAL_RATIO" : 5.0,
        "PRESSURE_LF" : 28.0,
        "PRESSURE_RF" : 28.0,
        "PRESSURE_LR" : 28.0,
        "PRESSURE_RR" : 28.0,
    },
    "ks_toyota_supra_mkiv_drift" : {
        "FRONT_BIAS" : 55.0,
        "DIFF_POWER" : 90.0,
        "DIFF_COAST" : 90.0,
        "FINAL_RATIO" : 5.0,
        "TURBO_0" : 100.0,
        "TURBO_1" : 100.0,
        "PRESSURE_LF" : 28.0,
        "PRESSURE_RF" : 28.0,
        "PRESSURE_LR" : 28.0,
        "PRESSURE_RR" : 28.0,
    }
}

base_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
bin_dir = os.path.join(base_dir, 'bin')
site.addsitedir(bin_dir)
os.add_dll_directory(bin_dir)

# AI validation output is shared with the AC-side reference logger.
logs_dir = os.path.join(os.path.abspath(ARGS.output_root), f"push{AI_PUSH}")
os.makedirs(logs_dir, exist_ok=True)
run_timestamp = time.strftime('%Y%m%d_%H%M%S')
csv_stem = (
    f"model_push{AI_PUSH}_{run_timestamp}"
    if BASELINE_MODE == "ai"
    else f"model_zoh_push{AI_PUSH}_{run_timestamp}"
)
csv_path = os.path.join(logs_dir, f"{csv_stem}.csv")

import PyProjectD as pd
pd.setLogFile(os.path.join(base_dir, 'projectd.log'), True);

# -------------------- Hotstart integration (from optimizer_test.py) --------------------
# 这些列表用于解析 UDP 二进制 float 流（与 optimizer_test.py 保持一致）
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
MIN_BASE = ((1+len(graphics_cols))+(1+len(physics_cols))+(1+len(acti_cols))) if INCLUDE_TIMES else (len(graphics_cols)+len(physics_cols)+len(acti_cols))

DATA_UDP_IP, DATA_UDP_PORT = "0.0.0.0", 5005

model_dir = ARGS.model_dir or os.path.join(os.path.dirname(os.path.realpath(__file__)), "ai_models", f"push{AI_PUSH}")
ai_predictor = None
if BASELINE_MODE == "ai":
    ai_predictor = AINetworkPredictor(model_dir=model_dir, device=ARGS.device)
    if ai_predictor.horizon < AI_ROLLOUT_STEPS:
        raise ValueError(f"AI model horizon={ai_predictor.horizon}, need {AI_ROLLOUT_STEPS}")


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


manifest_path = os.path.join(logs_dir, f"{csv_stem}_manifest.json")
checkpoint_path = os.path.join(model_dir, "best.pt")
config_path = os.path.join(model_dir, "config.json")
manifest = {
    "baseline_mode": BASELINE_MODE,
    "ai_push": AI_PUSH,
    "rollout_steps": AI_ROLLOUT_STEPS,
    "mid_hotstart_steps": 60,
    "risk_rollout_steps": 90,
    "sim_hz": sim_hz,
    "model_csv": csv_path,
}
if ai_predictor is not None:
    manifest.update({
        "model_dir": os.path.abspath(model_dir),
        "checkpoint_sha256": _sha256_file(checkpoint_path),
        "config_sha256": _sha256_file(config_path),
        "input_len": ai_predictor.input_len,
        "feature_count": len(ai_predictor.feature_columns),
        "feature_columns": ai_predictor.feature_columns,
        "target_columns": ai_predictor.target_columns,
        "horizon": ai_predictor.horizon,
    })
with open(manifest_path, "w", encoding="utf-8") as manifest_file:
    json.dump(manifest, manifest_file, ensure_ascii=True, indent=2)
print(f"[VALIDATE] mode={BASELINE_MODE} manifest: {manifest_path}")
ack_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ext_lock = threading.Lock()
ext_data_latest: Optional[Dict[str, Any]] = None
hotstart_request = threading.Event()

@dataclass
class HotstartParams:
    pose: tuple
    sethot: dict
    driver: dict

def _f(d: Dict[str, Any], name: str, default=0.0) -> float:
    try:
        return float(d.get(name, default))
    except Exception:
        return float(default)

def build_hotstart_from_ext(ext: Dict[str, Any]) -> HotstartParams:
    # 位姿（含车体位置与欧拉角；heading/pitch/roll 与优化器侧保持一致）
    car_x = _f(ext, "carCoordinates_1", 0.0)
    car_y = _f(ext, "carCoordinates_2", 0.0)
    car_z = _f(ext, "carCoordinates_3", 0.0)
    heading = -_f(ext, "heading", 0.0)
    pitch   = -_f(ext, "pitch",   0.0)
    roll    = -_f(ext, "roll",    0.0)

    # 轮/胎温与滑移
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
    tc = [_f(ext, "tyreCoreTemperature_1",80.0), _f(ext, "tyreCoreTemperature_2",80.0),
          _f(ext, "tyreCoreTemperature_3",80.0), _f(ext, "tyreCoreTemperature_4",80.0)]
    ti = [_f(ext, "tyreTempI_1",75.0), _f(ext, "tyreTempI_2",75.0),
          _f(ext, "tyreTempI_3",75.0), _f(ext, "tyreTempI_4",75.0)]
    tm = [_f(ext, "tyreTempM_1",75.0), _f(ext, "tyreTempM_2",75.0),
          _f(ext, "tyreTempM_3",75.0), _f(ext, "tyreTempM_4",75.0)]
    to = [_f(ext, "tyreTempO_1",75.0), _f(ext, "tyreTempO_2",75.0),
          _f(ext, "tyreTempO_3",75.0), _f(ext, "tyreTempO_4",75.0)]
    patch = [(ti[i]+tm[i]+to[i])/3.0 for i in range(4)]

    engineRPM = _f(ext, "rpms", 4500.0)
    gear = int(round(ext.get("gear", 3)))
    vel = [_f(ext, "velocity_1", 14.0), _f(ext, "velocity_2", 0.0), _f(ext, "velocity_3", 0.0)]
    brakeTemp = sum([_f(ext,"brakeTemp_1",80.0), _f(ext,"brakeTemp_2",80.0),
                     _f(ext,"brakeTemp_3",80.0), _f(ext,"brakeTemp_4",80.0)]) / 4.0

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
        steer=_f(ext, "steerAngle", 0.0),
        gas  =_f(ext, "gas", 0.0),
        brake=_f(ext, "brake", 0.0),
        clutch=float(ext.get("clutch", 0.0))
    )
    return HotstartParams(
        pose=(car_x, car_y, car_z, heading, roll, pitch, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0),
        sethot=sethot, driver=driver
    )


def _safe_float(v, default=0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _safe_int(v, default=0) -> int:
    try:
        return int(v)
    except Exception:
        return int(default)


def _vec3_to_list(v):
    return [
        _safe_float(getattr(v, 'x', 0.0)),
        _safe_float(getattr(v, 'y', 0.0)),
        _safe_float(getattr(v, 'z', 0.0)),
    ]


def _arr4_to_list(arr):
    vals = []
    for idx in range(4):
        try:
            vals.append(_safe_float(arr[idx]))
        except Exception:
            vals.append(0.0)
    return vals


def _is_close_f32(value: float, target: float, tol: float = 0.25) -> bool:
    return abs(float(value) - float(target)) <= tol


def _parse_ai_feature_window(vals, start_idx: int, total_floats: int):
    if total_floats - start_idx < AI_FEATURE_HEADER_FLOATS:
        return None
    if not _is_close_f32(vals[start_idx], AI_FEATURE_WINDOW_FLAG):
        return None
    idx = start_idx
    flag = float(vals[idx]); idx += 1
    version = float(vals[idx]); idx += 1
    capture_id = int(round(float(vals[idx]))); idx += 1
    start_pid = int(round(float(vals[idx]))); idx += 1
    anchor_pid = int(round(float(vals[idx]))); idx += 1
    end_pid = int(round(float(vals[idx]))); idx += 1
    recovery_frames = int(round(float(vals[idx]))); idx += 1
    post_recovery_frames = int(round(float(vals[idx]))); idx += 1
    stable_skip_frames = int(round(float(vals[idx]))); idx += 1
    frame_count = int(round(float(vals[idx]))); idx += 1
    feature_count = int(round(float(vals[idx]))); idx += 1
    if frame_count <= 0 or feature_count <= 0:
        return None
    needed = frame_count * feature_count
    if idx + needed > total_floats:
        return None
    matrix = np.asarray(vals[idx:idx + needed], dtype=np.float32).reshape(frame_count, feature_count)
    return {
        "flag": flag,
        "version": version,
        "capture_id": capture_id,
        "start_pid": start_pid,
        "anchor_pid": anchor_pid,
        "end_pid": end_pid,
        "recovery_frames": recovery_frames,
        "post_recovery_frames": post_recovery_frames,
        "stable_skip_frames": stable_skip_frames,
        "frame_count": frame_count,
        "feature_count": feature_count,
        "matrix": matrix,
    }


def build_state_csv_header():
    return [
        'loop160_round_idx',
        'loop160_step_idx_1based',
        'loop160_step_idx_0based',
        'baseline_mode', 'ai_push', 'ai_capture_id',
        'ai_start_pid', 'ai_anchor_pid', 'ai_end_pid',
        'ai_feature_frames', 'ai_feature_count',
        'ai_infer_ms', 'ai_nan_count',
        'anchor_steer', 'anchor_gas', 'anchor_brake',
        'ai_predicted_steer',
        'start_car_x', 'start_car_y', 'start_car_z',
        'carId', 'simId', 'timestamp',
        'control_steer', 'control_gas', 'control_brake', 'control_clutch',
        'control_handBrake', 'control_isShifterSupported',
        'control_requestedGearIndex', 'control_gearUp', 'control_gearDn',
        'collisionFlag', 'outOfTrackFlag', 'trackPointId', 'lastTrackPointTimestamp',
        'trackLocation', 'kappa', 'driftNow',
        'trackLeft_x', 'trackLeft_y', 'trackLeft_z',
        'trackRight_x', 'trackRight_y', 'trackRight_z',
        'bodyVsTrack', 'velocityVsTrack',
        'finalRatio', 'diffPowerRamp', 'diffCoastRamp',
        'outShaftLvelocity', 'outShaftRvelocity', 'drivevelocity', 'rootVelocity', 'enginevelocity',
        'engineRPM', 'speedMS', 'gear', 'gearGrinding',
        'bodyPos_x', 'bodyPos_y', 'bodyPos_z',
        'bodyEuler_x', 'bodyEuler_y', 'bodyEuler_z',
        'accG_x', 'accG_y', 'accG_z',
        'velocity_x', 'velocity_y', 'velocity_z',
        'localVelocity_x', 'localVelocity_y', 'localVelocity_z',
        'angularVelocity_x', 'angularVelocity_y', 'angularVelocity_z',
        'localAngularVelocity_x', 'localAngularVelocity_y', 'localAngularVelocity_z',
        'tyreAngularSpeed_FL', 'tyreAngularSpeed_FR', 'tyreAngularSpeed_RL', 'tyreAngularSpeed_RR',
        'tyreSlipRatio_FL', 'tyreSlipRatio_FR', 'tyreSlipRatio_RL', 'tyreSlipRatio_RR',
        'tyreNdSlip_FL', 'tyreNdSlip_FR', 'tyreNdSlip_RL', 'tyreNdSlip_RR',
        'slipAngleRAD_FL', 'slipAngleRAD_FR', 'slipAngleRAD_RL', 'slipAngleRAD_RR',
        'fy_FL', 'fy_FR', 'fy_RL', 'fy_RR',
    ]


def build_state_csv_row(
    state,
    loop160_round_idx,
    loop160_step_idx_0based,
    start_car_xyz,
    active_controls,
    ai_meta,
):
    ctrl_obj = getattr(state, 'controls', None)

    def _ctrl(name, fallback=0.0):
        if ctrl_obj is not None and hasattr(ctrl_obj, name):
            return getattr(ctrl_obj, name)
        return getattr(active_controls, name, fallback)

    row = [
        _safe_int(loop160_round_idx),
        _safe_int(loop160_step_idx_0based + 1),
        _safe_int(loop160_step_idx_0based),
        str(ai_meta.get('baseline_mode', BASELINE_MODE)),
        _safe_int(ai_meta.get('ai_push', AI_PUSH)),
        _safe_int(ai_meta.get('capture_id', 0)),
        _safe_int(ai_meta.get('start_pid', 0)),
        _safe_int(ai_meta.get('anchor_pid', 0)),
        _safe_int(ai_meta.get('end_pid', 0)),
        _safe_int(ai_meta.get('frame_count', 0)),
        _safe_int(ai_meta.get('feature_count', 0)),
        _safe_float(ai_meta.get('infer_ms', 0.0)),
        _safe_int(ai_meta.get('nan_count', 0)),
        _safe_float(ai_meta.get('anchor_steer', 0.0)),
        _safe_float(ai_meta.get('anchor_gas', 0.0)),
        _safe_float(ai_meta.get('anchor_brake', 0.0)),
        _safe_float(ai_meta.get('predicted_steer', 0.0)),
        _safe_float(start_car_xyz[0]), _safe_float(start_car_xyz[1]), _safe_float(start_car_xyz[2]),
        _safe_int(getattr(state, 'carId', 0)),
        _safe_int(getattr(state, 'simId', 0)),
        _safe_float(getattr(state, 'timestamp', 0.0)),
        _safe_float(_ctrl('steer', 0.0)),
        _safe_float(_ctrl('gas', 0.0)),
        _safe_float(_ctrl('brake', 0.0)),
        _safe_float(_ctrl('clutch', 0.0)),
        _safe_float(_ctrl('handBrake', 0.0)),
        _safe_int(_ctrl('isShifterSupported', 1)),
        _safe_int(_ctrl('requestedGearIndex', -1)),
        _safe_int(_ctrl('gearUp', 0)),
        _safe_int(_ctrl('gearDn', 0)),
        _safe_int(getattr(state, 'collisionFlag', 0)),
        _safe_int(getattr(state, 'outOfTrackFlag', 0)),
        _safe_int(getattr(state, 'trackPointId', 0)),
        _safe_float(getattr(state, 'lastTrackPointTimestamp', 0.0)),
        _safe_float(getattr(state, 'trackLocation', 0.0)),
        _safe_float(getattr(state, 'kappa', 0.0)),
        _safe_int(getattr(state, 'driftNow', 0)),
    ]
    row.extend(_vec3_to_list(getattr(state, 'trackLeft', None)))
    row.extend(_vec3_to_list(getattr(state, 'trackRight', None)))
    row.extend([
        _safe_float(getattr(state, 'bodyVsTrack', 0.0)),
        _safe_float(getattr(state, 'velocityVsTrack', 0.0)),
        _safe_float(getattr(state, 'finalRatio', 0.0)),
        _safe_float(getattr(state, 'diffPowerRamp', 0.0)),
        _safe_float(getattr(state, 'diffCoastRamp', 0.0)),
        _safe_float(getattr(state, 'outShaftLvelocity', 0.0)),
        _safe_float(getattr(state, 'outShaftRvelocity', 0.0)),
        _safe_float(getattr(state, 'drivevelocity', 0.0)),
        _safe_float(getattr(state, 'rootVelocity', 0.0)),
        _safe_float(getattr(state, 'enginevelocity', 0.0)),
        _safe_float(getattr(state, 'engineRPM', 0.0)),
        _safe_float(getattr(state, 'speedMS', 0.0)),
        _safe_int(getattr(state, 'gear', 0)),
        _safe_int(getattr(state, 'gearGrinding', 0)),
    ])
    row.extend(_vec3_to_list(getattr(state, 'bodyPos', None)))
    row.extend(_vec3_to_list(getattr(state, 'bodyEuler', None)))
    row.extend(_vec3_to_list(getattr(state, 'accG', None)))
    row.extend(_vec3_to_list(getattr(state, 'velocity', None)))
    row.extend(_vec3_to_list(getattr(state, 'localVelocity', None)))
    row.extend(_vec3_to_list(getattr(state, 'angularVelocity', None)))
    row.extend(_vec3_to_list(getattr(state, 'localAngularVelocity', None)))
    row.extend(_arr4_to_list(getattr(state, 'tyreAngularSpeed', [0.0, 0.0, 0.0, 0.0])))
    row.extend(_arr4_to_list(getattr(state, 'tyreSlipRatio', [0.0, 0.0, 0.0, 0.0])))
    row.extend(_arr4_to_list(getattr(state, 'tyreNdSlip', [0.0, 0.0, 0.0, 0.0])))
    row.extend(_arr4_to_list(getattr(state, 'slipAngleRAD', [0.0, 0.0, 0.0, 0.0])))
    row.extend(_arr4_to_list(getattr(state, 'fy', [0.0, 0.0, 0.0, 0.0])))
    return row

def data_udp_listener_loop(ip=DATA_UDP_IP, port=DATA_UDP_PORT):
    """Receive the base hotstart block followed by the 9727 AI feature window."""
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

            # The validation sender always includes the 15-float geometry block,
            # but keep the no-geometry layout readable for protocol diagnostics.
            calc_feature_idx = MIN_BASE + CALC_BLOCK + 1
            plain_feature_idx = MIN_BASE + 1
            has_calc = (
                calc_feature_idx < total_floats
                and _is_close_f32(vals[calc_feature_idx], AI_FEATURE_WINDOW_FLAG)
            )
            if not has_calc and not (
                plain_feature_idx < total_floats
                and _is_close_f32(vals[plain_feature_idx], AI_FEATURE_WINDOW_FLAG)
            ):
                continue

            idx = 0
            parsed: Dict[str, float] = {}
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

            if has_calc:
                calc_names = [
                    "hub_FL_x","hub_FL_y","hub_FL_z",
                    "hub_FR_x","hub_FR_y","hub_FR_z",
                    "hub_RL_x","hub_RL_y","hub_RL_z",
                    "hub_RR_x","hub_RR_y","hub_RR_z",
                    "axle_center_x","axle_center_y","axle_center_z",
                ]
                for name in calc_names:
                    parsed[name] = vals[idx]; idx += 1

            if idx >= total_floats:
                continue
            parsed["hotstarttag"] = float(vals[idx]); idx += 1
            ai_window = _parse_ai_feature_window(vals, idx, total_floats)
            if ai_window is None:
                continue
            expected_input_len = ai_predictor.input_len if ai_predictor is not None else 20
            expected_feature_count = len(ai_predictor.feature_columns) if ai_predictor is not None else 60
            if ai_window["frame_count"] != expected_input_len:
                print(
                    f"[DATA] skip capture={ai_window['capture_id']}: "
                    f"frames={ai_window['frame_count']} expected={expected_input_len}"
                )
                continue
            if ai_window["feature_count"] != expected_feature_count:
                print(
                    f"[DATA] skip capture={ai_window['capture_id']}: "
                    f"features={ai_window['feature_count']} expected={expected_feature_count}"
                )
                continue
            parsed["ai_feature_window"] = ai_window
            with ext_lock:
                ext_data_latest = parsed
            if parsed.get("hotstarttag", 1.0) < 0.5:
                hotstart_request.set()
    except Exception as e:
        print(f"[DATA] recv error: {e}")
    finally:
        try: sock.close()
        except: pass
# -------------------- /Hotstart integration --------------------

sim = pd.createSimulator(base_dir)
pd.loadTrack(sim, track_name)
car = pd.addCar(sim, car_model)

pd.teleportCarToSpline(sim, car, 0.01)
pd.setCarAutoTeleport(sim, car, teleport_on_hit, teleport_off_track, 1) # 0:Start, 1:Nearest, 2:Random
pd.setCarAssists(sim, car, auto_clutch, auto_shift, auto_blip)

state = pd.CarState()
controls = pd.CarControls()
controls.gas = 0.0
controls.clutch = 1
#controls.requestedGearIndex = 2 # 0=R, 1=N, 2=H1, 3=H2, 4=H3, 5=H4, 6=H5, 7=H6

pd.writeLog('MAIN LOOP')

# >>> NEW: 打开 CSV 并写表头（逐帧记录每一轮 160 帧预测的完整状态）
csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(build_state_csv_header())
csv_file.flush()

# >>> NEW: 启动 UDP 热启动监听线程
threading.Thread(target=data_udp_listener_loop, daemon=True).start()

flush_every = 50  # >>> NEW: 每50行刷一次盘，降低IO开销
row_count = 0
prediction_round_idx = 0  # 第几轮 160 帧预测

try:
    while True:
    # >>> NEW: 若收到 hotstart 请求，则应用至当前 sim/car
        if hotstart_request.is_set():  
            t0 = time.perf_counter()    
            with ext_lock:
                ext = dict(ext_data_latest) if ext_data_latest else None
            if ext:
                ai_window = ext.get("ai_feature_window")
                if ai_window is None:
                    hotstart_request.clear()
                    continue
                hot = build_hotstart_from_ext(ext)   # from optimizer_test.py
                if ai_predictor is not None:
                    prediction = ai_predictor.predict(ai_window["matrix"])
                    predicted_steer = np.asarray(
                        prediction["pred_steer"], dtype=np.float32
                    )[:AI_ROLLOUT_STEPS]
                else:
                    matrix = np.asarray(ai_window["matrix"], dtype=np.float32)
                    prediction = {
                        "infer_ms": 0.0,
                        "nan_count": int(np.size(matrix) - np.isfinite(matrix).sum()),
                    }
                    predicted_steer = np.full(
                        AI_ROLLOUT_STEPS,
                        float(hot.driver["steer"]),
                        dtype=np.float32,
                    )
                if predicted_steer.size != AI_ROLLOUT_STEPS:
                    raise RuntimeError(
                        f"capture={ai_window['capture_id']} returned "
                        f"{predicted_steer.size} steer values"
                    )
                # 先快速定位位置与姿态             
                active_sim = pd.getActiveSimulator()
                active_car = pd.getActiveCar()
                pd.stepSimulator(sim, sim_rate)
                (x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az) = hot.pose
                pd.teleportCarToPose(sim, car, x,y,z, hd, rl, pt, flx,fly,flz, frx,fry,frz, ax,ay,az)
                
                # 应用 sethotstart（轮速、轮胎温度、齿轮、转速、车速等）
                
                for i in range(300):                                        
                    controls.steer  = 0
                    controls.clutch = 1.0
                    controls.brake  = 0
                    controls.gas    = 0
                    pd.setCarControls(sim, car, smooth_controls, controls)
                    pd.stepSimulator(sim, sim_rate)
                pd.getCarState(sim, car, state)

                m = state.bodyMatrix
                bodyM_snap = pd.mat44f()
                bodyM_snap.M11 = m.M11; bodyM_snap.M12 = m.M12; bodyM_snap.M13 = m.M13; bodyM_snap.M14 = m.M14
                bodyM_snap.M21 = m.M21; bodyM_snap.M22 = m.M22; bodyM_snap.M23 = m.M23; bodyM_snap.M24 = m.M24
                bodyM_snap.M31 = m.M31; bodyM_snap.M32 = m.M32; bodyM_snap.M33 = m.M33; bodyM_snap.M34 = m.M34
                bodyM_snap.M41 = m.M41; bodyM_snap.M42 = m.M42; bodyM_snap.M43 = m.M43; bodyM_snap.M44 = m.M44

                # 其他矩阵也一样做一份快照（否则也会“跟着变”）
                f = state.fuelTankyMatrix
                fuelM_snap = pd.mat44f()
                fuelM_snap.M11=f.M11; fuelM_snap.M12=f.M12; fuelM_snap.M13=f.M13; fuelM_snap.M14=f.M14
                fuelM_snap.M21=f.M21; fuelM_snap.M22=f.M22; fuelM_snap.M23=f.M23; fuelM_snap.M24=f.M24
                fuelM_snap.M31=f.M31; fuelM_snap.M32=f.M32; fuelM_snap.M33=f.M33; fuelM_snap.M34=f.M34
                fuelM_snap.M41=f.M41; fuelM_snap.M42=f.M42; fuelM_snap.M43=f.M43; fuelM_snap.M44=f.M44

                h1 = state.hub1Matrix; h2 = state.hub2Matrix
                hub1M_snap = pd.mat44f(); hub2M_snap = pd.mat44f()
                for dst,src in ((hub1M_snap,h1),(hub2M_snap,h2)):
                    dst.M11=src.M11; dst.M12=src.M12; dst.M13=src.M13; dst.M14=src.M14
                    dst.M21=src.M21; dst.M22=src.M22; dst.M23=src.M23; dst.M24=src.M24
                    dst.M31=src.M31; dst.M32=src.M32; dst.M33=src.M33; dst.M34=src.M34
                    dst.M41=src.M41; dst.M42=src.M42; dst.M43=src.M43; dst.M44=src.M44

                sb1 = state.strutBodyM1; sb2 = state.strutBodyM2
                strutBodyM1_snap = pd.mat44f(); strutBodyM2_snap = pd.mat44f()
                for dst,src in ((strutBodyM1_snap,sb1),(strutBodyM2_snap,sb2)):
                    dst.M11=src.M11; dst.M12=src.M12; dst.M13=src.M13; dst.M14=src.M14
                    dst.M21=src.M21; dst.M22=src.M22; dst.M23=src.M23; dst.M24=src.M24
                    dst.M31=src.M31; dst.M32=src.M32; dst.M33=src.M33; dst.M34=src.M34
                    dst.M41=src.M41; dst.M42=src.M42; dst.M43=src.M43; dst.M44=src.M44

                ax = state.axleMatrix
                axleM_snap = pd.mat44f()
                axleM_snap.M11=ax.M11; axleM_snap.M12=ax.M12; axleM_snap.M13=ax.M13; axleM_snap.M14=ax.M14
                axleM_snap.M21=ax.M21; axleM_snap.M22=ax.M22; axleM_snap.M23=ax.M23; axleM_snap.M24=ax.M24
                axleM_snap.M31=ax.M31; axleM_snap.M32=ax.M32; axleM_snap.M33=ax.M33; axleM_snap.M34=ax.M34
                axleM_snap.M41=ax.M41; axleM_snap.M42=ax.M42; axleM_snap.M43=ax.M43; axleM_snap.M44=ax.M44
                np.set_printoptions(suppress=True, precision=6)
                def mat44_to_np(m):
                    return np.array([
                        [m.M11, m.M12, m.M13, m.M14],
                        [m.M21, m.M22, m.M23, m.M24],
                        [m.M31, m.M32, m.M33, m.M34],
                        [m.M41, m.M42, m.M43, m.M44],
                    ], dtype=float)

                    # print("[DBG] bodyM =\n", mat44_to_np(bodyM_snap))

                pd.applyWorldMatricesFast(
                    sim, car,
                    bodyM_snap, fuelM_snap, hub1M_snap, hub2M_snap, strutBodyM1_snap, strutBodyM2_snap, axleM_snap, True, True)
                
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
                prediction_round_idx += 1
                start_car_xyz = (x, y, z)                
                ai_meta = {
                    "baseline_mode": BASELINE_MODE,
                    "ai_push": AI_PUSH,
                    "capture_id": ai_window["capture_id"],
                    "start_pid": ai_window["start_pid"],
                    "anchor_pid": ai_window["anchor_pid"],
                    "end_pid": ai_window["end_pid"],
                    "frame_count": ai_window["frame_count"],
                    "feature_count": ai_window["feature_count"],
                    "infer_ms": prediction["infer_ms"],
                    "nan_count": prediction["nan_count"],
                    "anchor_steer": float(hot.driver["steer"]),
                    "anchor_gas": float(hot.driver["gas"]),
                    "anchor_brake": float(hot.driver["brake"]),
                }
                p1=state.totalReward
                
                eY_seq, ePsi_seq, bvt_seq = [], [], []
                sa_seq, v_seq, gfm_seq = [], [], []
                rewardtrack_seq, rewardspeed_seq, rewardsmooth_seq = [], [], []
                costy_seq, costpsi_seq,costcross_seq,costbound_seq,costdrift_seq= [], [], [], [], []
                eY_last = None
                crossed = False
                cross_idx = None
                for i, steer_value in enumerate(predicted_steer):
                    controls.steer  = float(steer_value)
                    controls.clutch = float(hot.driver["clutch"])
                    controls.brake  = float(hot.driver["brake"])
                    controls.gas    = float(hot.driver["gas"])
                
                    pd.setCarControls(sim, car, smooth_controls, controls)
                    pd.stepSimulator(sim, sim_rate)
                    pd.getCarState(sim, car, state)
                    ai_meta["predicted_steer"] = float(steer_value)

                    csv_writer.writerow(
                        build_state_csv_row(
                            state=state,
                            loop160_round_idx=prediction_round_idx,
                            loop160_step_idx_0based=i,
                            start_car_xyz=start_car_xyz,
                            active_controls=controls,
                            ai_meta=ai_meta,
                        )
                    )
                    row_count += 1
                    if row_count % flush_every == 0:
                        csv_file.flush()
                    # time.sleep(0.0001)
                    
                    eY   = float(getattr(state, "eY", 0.0))
                    ePsi = float(getattr(state, "ePsi", 0.0))
                    bvt  = float(getattr(state, "bodyVsTrack", 0.0))
                    sa   = float(getattr(state, "driftNow", 0.0))
                    vms  = float(getattr(state, "speedMS", 0.0))
                    gfm   = float(getattr(state, "glastFramePenalty", 0.0))   
                    trackW = float(getattr(state, "trackW", 0.0)) 
                    rewardtrack  = float(getattr(state, "Ctrack", 0.0))
                    rewardspeed  = float(getattr(state, "Cspeed", 0.0))
                    rewardsmooth = float(getattr(state, "Csmooth", 0.0))  

                    costy  = float(getattr(state, "costy", 0.0))
                    costpsi  = float(getattr(state, "costpsi", 0.0))
                    costcross = float(getattr(state, "costcross", 0.0))
                    costbound  = float(getattr(state, "costbound", 0.0))
                    costdrift = float(getattr(state, "costdrift", 0.0))  
                    outOfTrackFlag = float(getattr(state, "outOfTrackFlag", 0.0))  
                    pleft=(state.trackLeft.x,state.trackLeft.y,state.trackLeft.z)
                    pright=(state.trackRight.x,state.trackRight.y,state.trackRight.z)
                    ppos=(state.bodyPos.x,state.bodyPos.y,state.bodyPos.z)
                    dleft = math.dist(pleft, ppos)
                    dright = math.dist(pright, ppos)

                    eY_seq.append(eY); ePsi_seq.append(ePsi); bvt_seq.append(bvt)
                    sa_seq.append(sa); v_seq.append(vms); gfm_seq.append(gfm)
                    rewardtrack_seq.append(rewardtrack); rewardspeed_seq.append(rewardspeed); rewardsmooth_seq.append(rewardsmooth)
                    costy_seq.append(costy);costpsi_seq.append(costpsi);costcross_seq.append(costcross)
                    costbound_seq.append(costbound);costdrift_seq.append(costdrift)
                    if eY_last is not None:
                        if (eY_last == 0.0) or (eY == 0.0) or (eY_last * eY < 0.0):
                            crossed = True
                            if cross_idx is None:
                                cross_idx = i
                    eY_last = eY
                    
                pd.getCarState(sim, car, state)
                p2=state.totalReward
                if crossed and cross_idx is not None and cross_idx > 0:
                    ey_avg   = float(np.mean(eY_seq[:cross_idx]))
                    epsi_avg = float(np.mean(ePsi_seq[:cross_idx]))
                else:
                    ey_avg   = float(np.mean(eY_seq)) if eY_seq else 0.0
                    epsi_avg = float(np.mean(ePsi_seq)) if ePsi_seq else 0.0

                # 连续 ≥10 帧 sa>0.1
                run = 0; max_run10 = 0
                for sa in sa_seq:
                    if sa > 0.1: run += 1
                    else: max_run10 = max(max_run10, run); run = 0
                    
                max_run10 = max(max_run10, run)

                # print(sum(rewardtrack_seq), sum(rewardspeed_seq), sum(rewardsmooth_seq),p2-p1)
                # print(sum(rewardtrack_seq), sum(costy_seq), sum(costpsi_seq),sum(costcross_seq), sum(costbound_seq), sum(costdrift_seq))
                t1 = time.perf_counter()
                csv_file.flush()
                elapsed_ms = (t1 - t0) * 1000.0
                ack = [0.0] * 6 + [float(ai_window["capture_id"]), float(elapsed_ms)]
                ack_socket.sendto(struct.pack("<8f", *ack), ACK_ADDR)
                print(
                    f"[VALIDATE] mode={BASELINE_MODE} push={AI_PUSH} capture={ai_window['capture_id']} "
                    f"pids={ai_window['start_pid']}->{ai_window['end_pid']} "
                    f"infer_ms={prediction['infer_ms']:.3f} "
                    f"rollout_ms={elapsed_ms:.3f}"
                )

            hotstart_request.clear()


        # 原有打印保留
        #print(state.totalReward)
except KeyboardInterrupt:
    pass
finally:
    # >>> NEW: 收尾
    try:
        csv_file.flush()
        csv_file.close()
    except Exception:
        pass
    try:
        ack_socket.close()
    except Exception:
        pass

    pd.shutAll()

print(f'CSV saved to: {csv_path}')
