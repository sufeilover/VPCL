import os
import re
import numpy as np
import pandas as pd

# =========================
# CONFIG
# =========================
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(ROOT_DIR, "80-ai", "full_track_results")
DRIVE_XLSX = os.path.join(DATA_DIR, "801_lap.xlsx")
SPLINE_CSV = os.path.join(ROOT_DIR, "spline_points.csv")
OUT_XLSX = os.path.join(DATA_DIR, "801_full_track_tracking.xlsx")

# 统计阈值（米）
CTE_THRESHOLDS_M = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]

# 用 XZ 做最近点匹配（推荐：避免 y 高度噪声影响）
USE_XZ_FOR_MATCH = True

# =========================
# Helpers
# =========================
def find_col(df: pd.DataFrame, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None

def safe_numeric(s: pd.Series):
    return pd.to_numeric(s, errors="coerce").replace([np.inf, -np.inf], np.nan)

def wrap_to_half(x):
    """wrap to [-0.5, 0.5)"""
    return (x + 0.5) % 1.0 - 0.5

def try_build_kdtree(ref_pts_2d: np.ndarray):
    """
    Return a callable query function:
      idx = query(pts_2d)  -> nearest ref index for each row in pts_2d
    Uses scipy cKDTree if available; otherwise uses a vectorized chunked brute force.
    """
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_pts_2d)
        def query(pts_2d: np.ndarray):
            _, idx = tree.query(pts_2d, k=1)
            return idx.astype(int)
        return query, "scipy.cKDTree"
    except Exception:
        # fallback: chunked brute force (slower but no extra deps)
        ref = ref_pts_2d.astype(np.float64)
        def query(pts_2d: np.ndarray, chunk=2000):
            pts = pts_2d.astype(np.float64)
            idx_out = np.empty(len(pts), dtype=int)
            for i in range(0, len(pts), chunk):
                p = pts[i:i+chunk]  # [M,2]
                # dist^2 = (p_x - r_x)^2 + (p_z - r_z)^2
                # compute via broadcasting: [M,1,2] - [1,N,2] -> [M,N,2]
                d2 = ((p[:, None, :] - ref[None, :, :]) ** 2).sum(axis=2)  # [M,N]
                idx_out[i:i+chunk] = np.argmin(d2, axis=1)
            return idx_out
        return query, "numpy_bruteforce_chunked"

def compute_ref_heading_xz(ref_x: np.ndarray, ref_z: np.ndarray):
    """
    heading angle of reference line at each point in XZ (radians), using forward difference with wrap.
    """
    dx = np.roll(ref_x, -1) - ref_x
    dz = np.roll(ref_z, -1) - ref_z
    return np.arctan2(dz, dx)

def angle_wrap_pi(a):
    """wrap to [-pi, pi)"""
    return (a + np.pi) % (2*np.pi) - np.pi

def summarize_tracking(df_seg: pd.DataFrame, label: str, file_name: str,
                       cte_xz: np.ndarray, cte_xyz: np.ndarray,
                       prog_err: np.ndarray,
                       heading_err: np.ndarray | None):
    out = {
        "label": label,
        "file": file_name,
        "rows": int(len(df_seg)),
    }

    # CTE (XZ)
    v = np.asarray(cte_xz)[np.isfinite(cte_xz)]
    if not len(v):
        v = np.array([np.nan])
    out["cte_xz_mean_m"] = float(np.nanmean(v))
    out["cte_xz_median_m"] = float(np.nanmedian(v))
    out["cte_xz_p95_m"] = float(np.nanpercentile(v, 95))
    out["cte_xz_max_m"] = float(np.nanmax(v))

    for th in CTE_THRESHOLDS_M:
        out[f"pct_cte_xz_le_{th}m"] = float(np.mean(v[np.isfinite(v)] <= th) * 100.0) if np.isfinite(v).any() else np.nan

    # CTE (XYZ) optional but provided
    v3 = cte_xyz
    out["cte_xyz_mean_m"] = float(np.nanmean(v3))
    out["cte_xyz_p95_m"] = float(np.nanpercentile(v3, 95))
    out["cte_xyz_max_m"] = float(np.nanmax(v3))

    # Progress error
    pe = np.abs(prog_err[np.isfinite(prog_err)])
    out["prog_err_abs_mean"] = float(np.mean(pe)) if len(pe) else np.nan
    out["prog_err_abs_p95"] = float(np.percentile(pe, 95)) if len(pe) else np.nan
    out["prog_err_abs_max"] = float(np.max(pe)) if len(pe) else np.nan

    # Heading error if available
    if heading_err is not None:
        he = np.abs(heading_err)
        out["heading_err_abs_mean_rad"] = float(np.nanmean(he))
        out["heading_err_abs_p95_rad"] = float(np.nanpercentile(he, 95))
        out["heading_err_abs_max_rad"] = float(np.nanmax(he))
    else:
        out["heading_err_abs_mean_rad"] = np.nan
        out["heading_err_abs_p95_rad"] = np.nan
        out["heading_err_abs_max_rad"] = np.nan

    return out

def main():
    # ---- read driving data
    df = pd.read_excel(DRIVE_XLSX)
    file_name = os.path.basename(DRIVE_XLSX)

    # Required columns
    pos_col = find_col(df, ["normalizedCarPosition", "norm_pos", "pos_norm"])
    if pos_col is None:
        raise KeyError(f"Missing normalizedCarPosition in driving file. Columns: {list(df.columns)}")

    x_col = find_col(df, ["x", "pos_x", "car_x"])
    y_col = find_col(df, ["y", "pos_y", "car_y"])
    z_col = find_col(df, ["z", "pos_z", "car_z"])
    if x_col is None or z_col is None:
        raise KeyError(f"Missing car position columns (x/z) in driving file. Columns: {list(df.columns)}")

    sector_col = find_col(df, ["currentSectorIndex", "CurrentSectorIndex", "sectorIndex"])

    # Optional yaw column for heading error
    yaw_col = find_col(df, ["yaw", "carYaw", "heading", "yawRad", "yaw_rad"])

    # ---- read spline
    sdf = pd.read_csv(SPLINE_CSV)
    if not all(c in sdf.columns for c in ["x", "y", "z"]):
        raise KeyError(f"spline_points.csv missing x/y/z columns. Columns: {list(sdf.columns)}")

    # Without explicit s_norm, normalized progress correspondence is unverified.
    if "s_norm" in sdf.columns:
        ref_s_norm = pd.to_numeric(sdf["s_norm"], errors="coerce").to_numpy(float)
    else:
        # fallback: uniform
        ref_s_norm = np.full(len(sdf), np.nan)

    ref_x = pd.to_numeric(sdf["x"], errors="coerce").to_numpy(float)
    ref_y = pd.to_numeric(sdf["y"], errors="coerce").to_numpy(float)
    ref_z = pd.to_numeric(sdf["z"], errors="coerce").to_numpy(float)

    if not np.isfinite(np.column_stack([ref_x, ref_y, ref_z])).all():
        raise ValueError("Reference spline contains invalid coordinates.")

    # KDTree matching points (XZ or XYZ)
    if USE_XZ_FOR_MATCH:
        ref_pts = np.column_stack([ref_x, ref_z])
    else:
        ref_pts = np.column_stack([ref_x, ref_y, ref_z])

    query_nn, nn_backend = try_build_kdtree(ref_pts)
    # Distances remain nearest sampled-point distances, not exact polyline CTE.

    # ---- compute nearest reference index for each driving sample
    car_x = safe_numeric(df[x_col]).to_numpy(float)
    car_y = safe_numeric(df[y_col]).to_numpy(float) if y_col is not None else np.full(len(df), np.nan)
    car_z = safe_numeric(df[z_col]).to_numpy(float)

    ok_pos = np.isfinite(car_x) & np.isfinite(car_z)
    car_query = np.column_stack([car_x, car_z]) if USE_XZ_FOR_MATCH else np.column_stack([car_x, car_y, car_z])
    ok_pos = np.isfinite(car_query).all(axis=1)
    nn_idx = np.full(len(df), -1, dtype=int)
    nn_idx[ok_pos] = query_nn(car_query[ok_pos])

    # ---- cte_xz / cte_xyz
    rx = ref_x[nn_idx.clip(0, len(ref_x)-1)]
    ry = ref_y[nn_idx.clip(0, len(ref_y)-1)]
    rz = ref_z[nn_idx.clip(0, len(ref_z)-1)]

    dx = car_x - rx
    dz = car_z - rz
    cte_xz = np.sqrt(dx*dx + dz*dz)
    dy = car_y - ry
    cte_xyz = np.sqrt(dx*dx + dy*dy + dz*dz)

    # ---- progress error: normalizedCarPosition vs ref s_norm at nn_idx
    car_pos_norm = safe_numeric(df[pos_col]).to_numpy(float) % 1.0
    ref_pos_norm = ref_s_norm[nn_idx.clip(0, len(ref_s_norm)-1)]
    prog_err = wrap_to_half(car_pos_norm - ref_pos_norm)
    prog_err[~ok_pos] = np.nan

    # ---- heading error (optional)
    heading_err = None
    if yaw_col is not None:
        car_yaw = safe_numeric(df[yaw_col]).to_numpy(float)
        # reference heading from spline tangent in XZ
        ref_head = compute_ref_heading_xz(ref_x, ref_z)
        ref_yaw = ref_head[nn_idx.clip(0, len(ref_head)-1)]
        m = ok_pos & np.isfinite(car_yaw) & np.isfinite(ref_yaw)
        heading_err = np.full(len(df), np.nan)
        heading_err[m] = angle_wrap_pi(car_yaw[m] - ref_yaw[m])

    # ---- build segments
    rows = []

    def add_segment(mask, label):
        dseg = df.loc[mask]
        if len(dseg) == 0:
            # still output an empty row with NaNs
            empty = summarize_tracking(dseg, label, file_name,
                                      np.array([np.nan]), np.array([np.nan]),
                                      np.array([np.nan]),
                                      None if heading_err is None else np.array([np.nan]))
            empty["rows"] = 0
            rows.append(empty)
            return

        idx = dseg.index.to_numpy()
        rows.append(
            summarize_tracking(
                dseg, label, file_name,
                cte_xz[idx], cte_xyz[idx],
                prog_err[idx],
                None if heading_err is None else heading_err[idx]
            )
        )

    # GLOBAL
    add_segment(np.ones(len(df), dtype=bool), "GLOBAL")

    out_df = pd.DataFrame(rows)

    # extra metadata: backend
    out_df.insert(2, "nn_backend", nn_backend)
    out_df["matching_plane"] = "XZ" if USE_XZ_FOR_MATCH else "XYZ"
    out_df["distance_definition"] = "nearest_sampled_reference_point"
    out_df["valid_position_rows"] = int(ok_pos.sum())
    out_df["aggregation"] = "sample_weighted"
    out_df["progress_mapping"] = "explicit_s_norm" if "s_norm" in sdf else "unavailable_no_s_norm"
    out_df["heading_convention"] = "legacy_raw_yaw_minus_atan2_dz_dx_unverified"
    os.makedirs(os.path.dirname(os.path.abspath(OUT_XLSX)), exist_ok=True)

    with pd.ExcelWriter(OUT_XLSX, engine="openpyxl") as w:
        out_df.to_excel(w, sheet_name="tracking_metrics", index=False)

    print("Saved:", OUT_XLSX)
    print("NN backend:", nn_backend)
    if yaw_col is None:
        print("[INFO] yaw column not found; heading error metrics are NaN.")

if __name__ == "__main__":
    main()
