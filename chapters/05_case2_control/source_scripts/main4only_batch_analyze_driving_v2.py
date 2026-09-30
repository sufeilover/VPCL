import os
import glob
import re
import numpy as np
import pandas as pd

# =========================================
# CONFIG（只需要改这里）
# =========================================
INPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "80-ai", "full_track_results")
INPUT_GLOB = "*_lap.xlsx"
OUTPUT_FILENAME = "full_track_driving.xlsx"

A_NEG3 = -3.0
A_NEG5 = -5.0
A_POS2 =  2.0

# =========================
# DESPIKE (short spikes <= 10 frames)
# =========================
DESPIKE_ENABLE = False     # <- 你要的标志
DESPIKE_MAX_LEN = 5      # <=10帧视为毛刺段，才会处理
DESPIKE_WIN = 11          # rolling窗口（建议奇数 15/21/31）
DESPIKE_K = 3.5           # 阈值系数（3~5常用，越大越不敏感）

# 输入过滤规则：跳过报表、Excel锁文件
SKIP_PREFIXES = ("report_", "~$")
SKIP_EXACT_NAMES = (OUTPUT_FILENAME,)  # 输出文件本身也跳过


# =========================================
# 工具函数
# =========================================

def _hampel_outlier_mask(x: pd.Series, win: int, k: float) -> np.ndarray:
    x = safe_numeric(x)
    med = x.rolling(win, center=True, min_periods=max(3, win // 3)).median()
    mad = (x - med).abs().rolling(win, center=True, min_periods=max(3, win // 3)).median()
    sigma = 1.4826 * mad
    thr = (k * sigma).replace(0, np.nan)
    out = (x - med).abs() > thr
    return out.fillna(False).to_numpy(dtype=bool)

def _replace_short_runs_by_interp(x: pd.Series, outlier: np.ndarray, max_len: int) -> pd.Series:
    x = safe_numeric(x).copy()
    n = len(x)
    if n == 0 or not np.any(outlier):
        return x

    to_fix = np.zeros(n, dtype=bool)
    i = 0
    while i < n:
        if not outlier[i]:
            i += 1
            continue
        j = i
        while j < n and outlier[j]:
            j += 1
        if (j - i) <= max_len:
            to_fix[i:j] = True
        i = j

    if not np.any(to_fix):
        return x

    x2 = x.copy()
    x2.iloc[to_fix] = np.nan
    x2 = x2.interpolate(method="linear", limit_direction="both")
    return x2

def despike_columns_inplace(df: pd.DataFrame, cols: list[str]) -> None:
    if not DESPIKE_ENABLE:
        return
    for c in cols:
        if c not in df.columns:
            continue
        mask = _hampel_outlier_mask(df[c], win=DESPIKE_WIN, k=DESPIKE_K)
        df[c] = _replace_short_runs_by_interp(df[c], mask, max_len=DESPIKE_MAX_LEN)

def find_col(df: pd.DataFrame, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None

def safe_numeric(s: pd.Series):
    return pd.to_numeric(s, errors="coerce").replace([np.inf, -np.inf], np.nan)

def rowwise_max_abs(df, cols):
    mat = df[cols].to_numpy(dtype=float)
    return np.nanmax(np.abs(mat), axis=1)

def rowwise_max(df, cols):
    mat = df[cols].to_numpy(dtype=float)
    return np.nanmax(mat, axis=1)

def rowwise_mean(df, cols):
    mat = df[cols].to_numpy(dtype=float)
    return np.nanmean(mat, axis=1)

def extract_wheels(df: pd.DataFrame, base_names):
    wheels = ["FL", "FR", "RL", "RR"]
    out = {w: None for w in wheels}
    cols = set(df.columns)
    for w in wheels:
        for base in base_names:
            cands = [
                f"{base}_{w}", f"{base}{w}",
                f"{base}_{w.lower()}", f"{base}{w.lower()}",
            ]
            for c in cands:
                if c in cols:
                    out[w] = c
                    break
            if out[w] is not None:
                break
    return out

def compute_longitudinal_accel(df, t_col, speed_col):
    t = safe_numeric(df[t_col]).to_numpy(dtype=float)
    v_kmh = safe_numeric(df[speed_col]).to_numpy(dtype=float)
    v = v_kmh / 3.6

    ok = np.isfinite(t) & np.isfinite(v)
    if ok.sum() < 5:
        return np.full(len(df), np.nan)

    t2, v2 = t, v  # Missing samples must not become fabricated acceleration.

    dt = np.diff(t2)
    dv = np.diff(v2)
    denom = np.where(dt <= 1e-9, np.nan, dt)

    a = np.empty_like(v2)
    a[:] = np.nan
    a[1:] = dv / denom
    # The first sample has no preceding interval; keep it missing.
    return a

def mode_of_series_numeric(s: pd.Series):
    v = safe_numeric(s).to_numpy(dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return np.nan, 0
    v2 = np.round(v, 6)
    vals, cnts = np.unique(v2, return_counts=True)
    return float(vals[int(np.argmax(cnts))]), int(v.size)

def compute_metrics(df, label, file_name,
                    pos_col, t_col, speed_col,
                    ws_cols4, sa_cols4,
                    sector_idx=None, sector_time_col=None):
    out = {
        "label": label,
        "file": file_name,
        "rows": int(len(df)),
        "sector_index": (int(sector_idx) if sector_idx is not None and np.isfinite(sector_idx) else np.nan),
        "sector_time_s": np.nan,
        "sector_time_samples": 0
    }

    if df is None or len(df) == 0:
        for k in ["pos_min","pos_max","speed_mean","speed_median","speed_p95","speed_max","speed_min",
                  "pct_a_lt_m3","pct_a_lt_m5","pct_a_gt_p2","a_min","a_max",
                  "wheelSlip_mean","wheelSlip_p95","wheelSlip_max",
                  "absSlipAngle_mean","absSlipAngle_p95","absSlipAngle_max"]:
            out[k] = np.nan
        return out

    if sector_time_col is not None and sector_time_col in df.columns:
        st, n_valid = mode_of_series_numeric(df[sector_time_col])
        out["sector_time_s"] = st
        out["sector_time_samples"] = n_valid

    pos = safe_numeric(df[pos_col]).to_numpy(float)
    out["pos_min"] = float(np.nanmin(pos))
    out["pos_max"] = float(np.nanmax(pos))

    sp = safe_numeric(df[speed_col]).to_numpy(float)
    out["speed_mean"] = float(np.nanmean(sp))
    out["speed_median"] = float(np.nanmedian(sp))
    out["speed_p95"] = float(np.nanpercentile(sp, 95))
    out["speed_max"] = float(np.nanmax(sp))
    out["speed_min"] = float(np.nanmin(sp))

    a = compute_longitudinal_accel(df, t_col, speed_col)
    out["pct_a_lt_m3"] = float(np.mean((a < A_NEG3)[np.isfinite(a)]) * 100.0) if np.isfinite(a).any() else np.nan
    out["pct_a_lt_m5"] = float(np.mean((a < A_NEG5)[np.isfinite(a)]) * 100.0) if np.isfinite(a).any() else np.nan
    out["pct_a_gt_p2"] = float(np.mean((a > A_POS2)[np.isfinite(a)]) * 100.0) if np.isfinite(a).any() else np.nan
    out["a_min"] = float(np.nanmin(a))
    out["a_max"] = float(np.nanmax(a))

    ws_valid = [c for c in ws_cols4 if c and c in df.columns]
    if ws_valid:
        ws_row_max  = rowwise_max(df, ws_valid)
        ws_row_mean = rowwise_mean(df, ws_valid)
        out["wheelSlip_mean"] = float(np.nanmean(ws_row_mean))
        out["wheelSlip_p95"]  = float(np.nanpercentile(ws_row_max, 95))
        out["wheelSlip_max"]  = float(np.nanmax(ws_row_max))
    else:
        out["wheelSlip_mean"] = np.nan
        out["wheelSlip_p95"] = np.nan
        out["wheelSlip_max"] = np.nan

    sa_valid = [c for c in sa_cols4 if c and c in df.columns]
    if sa_valid:
        asa_row_max  = rowwise_max_abs(df, sa_valid)
        asa_row_mean = np.nanmean(np.abs(df[sa_valid].to_numpy(float)), axis=1)
        out["absSlipAngle_mean"] = float(np.nanmean(asa_row_mean))
        out["absSlipAngle_p95"]  = float(np.nanpercentile(asa_row_max, 95))
        out["absSlipAngle_max"]  = float(np.nanmax(asa_row_max))
    else:
        out["absSlipAngle_mean"] = np.nan
        out["absSlipAngle_p95"] = np.nan
        out["absSlipAngle_max"] = np.nan

    return out

def analyze_one_file(xlsx_path: str, corners=None, straights=None):
    df = pd.read_excel(xlsx_path)
    file_name = os.path.basename(xlsx_path)

    pos_col   = find_col(df, ["normalizedCarPosition", "norm_pos", "pos_norm"])
    t_col     = find_col(df, ["t_sec", "time_s", "iCurrentTime_s"])
    speed_col = find_col(df, ["speedKmh", "speed_kmh", "speed"])

    sector_col = find_col(df, ["currentSectorIndex", "CurrentSectorIndex", "sectorIndex"])
    sector_time_col = find_col(df, ["CurrentSectorTime_s", "currentSectorTime_s", "sectorTime_s"])

    if pos_col is None or t_col is None or speed_col is None:
        raise KeyError(
            f"{file_name}: missing required columns. Need normalizedCarPosition/t_sec/speedKmh. Got: {list(df.columns)}"
        )

    ws_map = extract_wheels(df, ["wheelSlip"])
    sa_map = extract_wheels(df, ["SlipAngle", "slipAngle", "slip_angle"])
    nd_map = extract_wheels(df, ["NdSlip", "ndSlip", "nd_slip"])

    ws_cols4 = [ws_map[w] for w in ["FL","FR","RL","RR"] if ws_map[w] is not None]
    sa_cols4 = [sa_map[w] for w in ["FL","FR","RL","RR"] if sa_map[w] is not None]
    nd_cols4 = [nd_map[w] for w in ["FL","FR","RL","RR"] if nd_map[w] is not None]

    # ---- (建议)先按时间排序，保证“连续10帧”定义成立
    if t_col is not None and t_col in df.columns:
        df = df.sort_values(t_col).reset_index(drop=True)
    else:
        df = df.sort_values(pos_col).reset_index(drop=True)

    # ---- despike：只对 wheelSlip/SlipAngle/NdSlip 四轮做短毛刺处理
    despike_cols = list(dict.fromkeys(ws_cols4 + sa_cols4 + nd_cols4))  # 去重保序
    despike_columns_inplace(df, despike_cols)

    # Whole selected lap only; no geometric/sector classification.

    rows = []

    rows.append(
        compute_metrics(df, label="GLOBAL", file_name=file_name,
                        pos_col=pos_col, t_col=t_col, speed_col=speed_col,
                        ws_cols4=ws_cols4, sa_cols4=sa_cols4,
                        sector_idx=None, sector_time_col=None)
    )

    row = rows[0]
    row.pop("sector_index", None)
    row.pop("sector_time_s", None)
    row.pop("sector_time_samples", None)
    row["aggregation"] = "sample_weighted"
    row["despike_enabled"] = bool(DESPIKE_ENABLE)
    if "LapTime_s" in df:
        row["lap_time_s"], _ = mode_of_series_numeric(df["LapTime_s"])
    if "lap_complete" in df:
        row["lap_complete"] = bool(df["lap_complete"].all())
    row["time_span_s"] = float(safe_numeric(df[t_col]).max() - safe_numeric(df[t_col]).min())
    row["accel_valid_samples"] = int(np.isfinite(compute_longitudinal_accel(df, t_col, speed_col)).sum())

    return pd.DataFrame(rows)

# =========================================
# Main
# =========================================
def should_skip_file(path: str) -> bool:
    name = os.path.basename(path)
    if name in SKIP_EXACT_NAMES:
        return True
    for p in SKIP_PREFIXES:
        if name.startswith(p):
            return True
    return False

def main():
    print("[INFO] Whole-track analysis:", INPUT_DIR)
    pattern = os.path.join(INPUT_DIR, INPUT_GLOB)
    files_all = sorted(glob.glob(pattern))
    files = [f for f in files_all if not should_skip_file(f) and os.path.abspath(f) != os.path.abspath(os.path.join(INPUT_DIR, OUTPUT_FILENAME))]

    if not files:
        raise FileNotFoundError(f"No input files matched (after filtering): {pattern}")

    all_rows = []
    for f in files:
        try:
            df_rows = analyze_one_file(f)
            all_rows.append(df_rows)
        except PermissionError:
            print(f"[ERROR] {os.path.basename(f)}: Permission denied. Close Excel / OneDrive lock, then retry.")
        except Exception as e:
            print(f"[ERROR] {os.path.basename(f)}: {e}")

    if not all_rows:
        raise RuntimeError("No outputs produced. Check input files / locks / column names.")

    out_df = pd.concat(all_rows, ignore_index=True)

    out_path = os.path.join(INPUT_DIR, OUTPUT_FILENAME)
    with pd.ExcelWriter(out_path, engine="openpyxl") as w:
        out_df.to_excel(w, sheet_name="metrics_all", index=False)

    print("Saved:", out_path)

if __name__ == "__main__":
    main()
