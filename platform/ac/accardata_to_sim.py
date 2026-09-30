# accardata_to_sim.py  (updated for (c_float*3)*4 i.e. (4,3))

import ast, json
import numpy as np
from typing import Dict, List, Tuple, Any

# ── 与原脚本一致的列定义 ─────────────────────────────────
GRAPHICS_COLS = ["carCoordinates_1", "carCoordinates_2", "carCoordinates_3"]

PHYSICS_COLS_ORIG = [
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

ACTI_COLS_ORIG = [
    "SlipRatio_FL", "SlipRatio_FR", "SlipRatio_RL", "SlipRatio_RR",
    "SlipAngle_FL", "SlipAngle_FR", "SlipAngle_RL", "SlipAngle_RR",
]
ACTI_LOADED_RADIUS_COLS = ["TyreLoadedRadius_FL","TyreLoadedRadius_FR","TyreLoadedRadius_RL","TyreLoadedRadius_RR"]

CALC_COLS_ORDER = [
    "hub_FL_x","hub_FL_y","hub_FL_z",
    "hub_FR_x","hub_FR_y","hub_FR_z",
    "hub_RL_x","hub_RL_y","hub_RL_z",
    "hub_RR_x","hub_RR_y","hub_RR_z",
    "axle_center_x","axle_center_y","axle_center_z",
]

# ───────────────────────────────── helpers ─────────────────────────────────

def _parse_list_like(val):
    """把字符串/列表/元组/numpy数组 解析成 Python 列表（深拷贝）; 失败返回 None。"""
    if isinstance(val, (list, tuple)):
        return [ _parse_list_like(x) if isinstance(x, (list, tuple)) else x for x in val ]
    try:
        import numpy as _np  # 局部导入避免硬依赖
        if isinstance(val, _np.ndarray):
            return val.tolist()
    except Exception:
        pass
    if isinstance(val, str):
        s = val.strip()
        # 尝试 JSON → 再尝试 Python 字面量
        for loader in (json.loads, ast.literal_eval):
            try:
                obj = loader(s)
                return _parse_list_like(obj)
            except Exception:
                continue
    return None

def _coerce_4x3(arr_like) -> np.ndarray:
    """
    把任意二维/一维数组整理成形状 (4,3)：
      - 若是 (4,3) 直接返回
      - 若是 (3,4) 转置
      - 若是一维 12 个数，按 [FLx,FLy,FLz, FRx,FRy,FRz, RLx,RLy,RLz, RRx,RRy,RRz] 重排成 (4,3)
      - 其他形状报错
    """
    a = np.array(arr_like, dtype=float)
    if a.ndim == 2:
        if a.shape == (4, 3):   # 新格式：4轮×3轴（正确）
            return a
        if a.shape == (3, 4):   # 旧格式：3轴×4轮 → 转置
            return a.T
        # 其他二维形状，尝试平铺再重塑
        a = a.reshape(-1)
    if a.ndim == 1:
        if a.size == 12:
            return a.reshape(4, 3)
    raise ValueError(f"cannot coerce to (4,3), got shape={a.shape}")

def _getf(d: Dict[str, Any], k: str, *, allow_missing=False) -> float:
    """
    取标量：
      - 先直接取 d[k]
      - 若是形如 base_idx（如 velocity_1），尝试从向量 base 中取对应元素
    """
    if k in d:
        return float(d[k])
    if "_" in k:
        base, idxs = k.rsplit("_", 1)
        if idxs.isdigit():
            vec = _get_vec(d, base)
            if vec is not None:
                idx = int(idxs) - 1  # 1-based -> 0-based
                # 向量长度按语义判定
                if base in ("velocity",):  # 长度 3
                    if idx < 3: return float(vec[idx])
                elif base in ("wheelAngularSpeed","tyreCoreTemperature","tyreTempI","tyreTempM","tyreTempO","brakeTemp"):
                    if idx < 4: return float(vec[idx])
                elif base in ("tyreContactPoint","tyreContactNormal","tyreContactHeading"):
                    # 12 个（(4,3)展平）；此分支只用于兼容旧 flat 命名 _1.._12
                    if idx < 12: return float(vec[idx])
    if allow_missing:
        return float("nan")
    raise KeyError(f"Missing key: {k}")

def _get_vec(d: Dict[str, Any], base: str):
    """
    从 d[base] 解析向量/矩阵：
      - 一维向量: list/tuple/ndarray
      - 二维矩阵: (4,3)/(3,4)，会被展开成一维 12 个数（按 (4,3) 排）
      - 字符串: 支持 JSON / Python 字面量
    返回一维 list[float] 或 None
    """
    if base not in d:
        return None
    arr_like = _parse_list_like(d[base])
    if arr_like is None:
        return None
    try:
        a = np.array(arr_like, dtype=float)
    except Exception:
        return None

    if a.ndim == 1:
        return a.astype(float).tolist()
    if a.ndim == 2:
        try:
            a43 = _coerce_4x3(a)  # 统一成 (4,3)
            return a43.reshape(-1).tolist()  # 展平成 12 个
        except Exception:
            pass
        # 其他二维形状：平铺
        return a.reshape(-1).astype(float).tolist()
    # 其他维度：平铺
    return a.reshape(-1).astype(float).tolist()

def _get_4x3(physics: Dict[str, Any], base: str) -> np.ndarray:
    """
    返回 (4,3) 的数组（轮序 [FL,FR,RL,RR]），支持以下来源：
      1) physics[base] 为 (4,3) 或 (3,4) 或 一维 12 个数
      2) physics 里已有 flat 键 base_1..base_12
    """
    # 优先解析 physics[base]
    if base in physics:
        arr_like = _parse_list_like(physics[base])
        if arr_like is not None:
            return _coerce_4x3(arr_like)
    # 回退到 flat 12 键
    vals = [_getf(physics, f"{base}_{i}") for i in range(1, 13)]
    return np.array(vals, dtype=float).reshape(4, 3)

# ───────────────────────────────── 主函数 ─────────────────────────────────

def build_packet_from_live(
    physics: Dict[str, Any],
    graphics: Dict[str, Any],
    acti: Dict[str, Any],
    *,
    include_times: bool = True,
    trigger_hotstart: bool = False,
    include_calc: bool = True,
) -> Tuple[List[float], Dict[str, float]]:
    """
    从三份“实时字典”构建：
      - values: 按旧脚本完全一致顺序的一维 float 列表（默认含 3 个 time、15 个计算量、最后 1 个触发位）
      - calc  : 15 个几何量的字典（四轮 hub + 后桥中心）
    其中 tyreContactPoint/Normal 支持新结构 (4,3)（即 (c_float*3)*4），
    同时兼容旧结构 (3,4) 与扁平 12 键。
    """
    values: List[float] = []
    calc: Dict[str, float] = {}

    # graphics：carCoordinates_1..3 或 carCoordinates（字符串/数组）
    if include_times:
        values.append(float(graphics.get("time", float("nan"))))

    if "carCoordinates_1" in graphics:
        values.extend([float(graphics["carCoordinates_1"]),
                       float(graphics["carCoordinates_2"]),
                       float(graphics["carCoordinates_3"])])
    else:
        coords = _parse_list_like(graphics.get("carCoordinates"))
        if not (isinstance(coords, list) and len(coords) >= 3):
            raise KeyError("graphics missing carCoordinates or carCoordinates_1..3")
        values.extend([float(coords[0]), float(coords[1]), float(coords[2])])

    # physics：时间 + 原 33 列（向量自动拆分）
    if include_times:
        values.append(float(physics.get("time", float("nan"))))
    for k in PHYSICS_COLS_ORIG:
        values.append(_getf(physics, k))

    # acti：时间 + 8 列
    if include_times:
        values.append(float(acti.get("time", float("nan"))))
    for k in ACTI_COLS_ORIG:
        values.append(_getf(acti, k))

    # 15 个计算列：hub 与后桥中心（按 (4,3) 计算）
    if include_calc:
        P = _get_4x3(physics, "tyreContactPoint")    # (4,3)
        N = _get_4x3(physics, "tyreContactNormal")   # (4,3)

        norms = np.linalg.norm(N, axis=1, keepdims=True)
        N_safe = np.where(norms > 1e-12, N / norms, np.nan)

        R_loaded = [float(acti[c]) for c in ACTI_LOADED_RADIUS_COLS]  # [FL, FR, RL, RR]
        FL, FR, RL, RR = 0, 1, 2, 3

        hubs = {w: P[w, :] + N_safe[w, :] * R_loaded[w] for w in (FL, FR, RL, RR)}
        axle_center = 0.5 * (hubs[RL] + hubs[RR])

        calc_vals = {
            "hub_FL_x": hubs[FL][0], "hub_FL_y": hubs[FL][1], "hub_FL_z": hubs[FL][2],
            "hub_FR_x": hubs[FR][0], "hub_FR_y": hubs[FR][1], "hub_FR_z": hubs[FR][2],
            "hub_RL_x": hubs[RL][0], "hub_RL_y": hubs[RL][1], "hub_RL_z": hubs[RL][2],
            "hub_RR_x": hubs[RR][0], "hub_RR_y": hubs[RR][1], "hub_RR_z": hubs[RR][2],
            "axle_center_x": axle_center[0], "axle_center_y": axle_center[1], "axle_center_z": axle_center[2],
        }
        for name in CALC_COLS_ORDER:
            v = float(calc_vals[name])
            calc[name] = v
            values.append(v)

    # 末尾触发位（与旧脚本一致：0=触发热启动；1=不触发）
    values.append(0.0 if trigger_hotstart else 1.0)
    return values, calc
