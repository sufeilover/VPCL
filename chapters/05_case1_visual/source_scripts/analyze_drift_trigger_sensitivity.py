#!/usr/bin/env python3
"""Click Run in VS Code to analyze drift persistence sensitivity."""

from pathlib import Path
import math

import numpy as np
import pandas as pd


# =============================================================================
# USER SETTINGS
# =============================================================================
# None means: automatically analyze the newest matching CSV in INPUT_DIRECTORY.
# To lock the analysis to one experiment, replace None with Path(r"full path").
INPUT_CSV = None

INPUT_DIRECTORY = Path(
    r"E:\IEEE-TVT\new-similar\case study1_drift_analysis"
)
INPUT_PATTERN = "drift_sensitivity_*.csv"

# None means: create <CSV name>_report beside the input CSV.
OUTPUT_DIR = None

# None means: compare thresholds up to the shortest prediction horizon.
MAX_THRESHOLD = None

MINIMUM_COMMON_PLATEAU_LENGTH = 3

# None means: use the longest prediction horizon for the severity plot.
RISK_HORIZON = None

TREND_BIN_WIDTH = 20
OUTPUT_DPI = 300
SIMULATION_HZ = 333.0
# =============================================================================


RUN_COLUMNS = ["max_run_FL", "max_run_FR", "max_run_RL", "max_run_RR"]
SLIP_COLUMNS = [
    "max_abs_slip_deg_FL",
    "max_abs_slip_deg_FR",
    "max_abs_slip_deg_RL",
    "max_abs_slip_deg_RR",
]
REQUIRED_COLUMNS = [
    "request_id",
    "horizon_frames",
    "threshold_deg",
    *RUN_COLUMNS,
    *SLIP_COLUMNS,
]


def validate_data(df):
    missing = [column for column in REQUIRED_COLUMNS if column not in df.columns]
    if missing:
        raise ValueError("Missing required columns: " + ", ".join(missing))
    if df.empty:
        raise ValueError("The input CSV contains no data rows.")
    missing_values = df[REQUIRED_COLUMNS].isna().sum()
    missing_values = missing_values[missing_values > 0]
    if not missing_values.empty:
        raise ValueError(
            "Required columns contain missing values: "
            + str(missing_values.to_dict())
        )
    if (df[RUN_COLUMNS] < 0).any().any():
        raise ValueError("Run-length columns must be non-negative.")


def prepare_data(df):
    result = df.copy()
    result["persistence_frames"] = result[RUN_COLUMNS].max(axis=1).astype(int)
    result["max_slip_deg"] = result[SLIP_COLUMNS].max(axis=1).astype(float)
    return result


def check_request_alignment(df, horizons):
    expected = tuple(horizons)
    observed = df.groupby("request_id")["horizon_frames"].apply(
        lambda values: tuple(sorted(int(value) for value in values))
    )
    incomplete_ids = [int(value) for value in observed[observed != expected].index]
    duplicates = int(df.duplicated(["request_id", "horizon_frames"]).sum())
    return incomplete_ids, duplicates


def build_threshold_sweep(df, horizons, maximum_threshold):
    rows = []
    for horizon in horizons:
        group = df[df["horizon_frames"] == horizon]
        limit = min(maximum_threshold, horizon)
        for threshold in range(1, limit + 1):
            selected = group["persistence_frames"] >= threshold
            severity = group.loc[selected, "max_slip_deg"]
            rows.append(
                {
                    "horizon_frames": horizon,
                    "threshold_frames": threshold,
                    "threshold_ms": threshold / SIMULATION_HZ * 1000.0,
                    "trigger_count": int(selected.sum()),
                    "trigger_rate": float(selected.mean()),
                    "selected_slip_min_deg": (
                        float(severity.min()) if len(severity) else math.nan
                    ),
                    "selected_slip_median_deg": (
                        float(severity.median()) if len(severity) else math.nan
                    ),
                    "selected_slip_mean_deg": (
                        float(severity.mean()) if len(severity) else math.nan
                    ),
                    "total_requests": int(len(group)),
                }
            )
    return pd.DataFrame(rows)


def find_common_plateaus(sweep, horizons, minimum_length):
    pivot = sweep.pivot(
        index="threshold_frames",
        columns="horizon_frames",
        values="trigger_count",
    ).dropna(subset=horizons)
    if pivot.empty:
        return []

    plateaus = []
    thresholds = [int(value) for value in pivot.index]
    vectors = [
        tuple(int(row[horizon]) for horizon in horizons)
        for _, row in pivot.iterrows()
    ]
    start = thresholds[0]
    previous = thresholds[0]
    vector = vectors[0]

    for threshold, current_vector in zip(thresholds[1:], vectors[1:]):
        if threshold != previous + 1 or current_vector != vector:
            if previous - start + 1 >= minimum_length:
                plateaus.append((start, previous, vector))
            start = threshold
            vector = current_vector
        previous = threshold

    if previous - start + 1 >= minimum_length:
        plateaus.append((start, previous, vector))
    return plateaus


def spearman_correlation(x, y):
    if len(x) < 2 or x.nunique() < 2 or y.nunique() < 2:
        return math.nan
    return float(x.rank(method="average").corr(y.rank(method="average")))


def build_binned_trend(group, bin_width):
    maximum = int(group["persistence_frames"].max())
    edges = np.arange(0, maximum + bin_width + 1, bin_width)
    bins = pd.cut(
        group["persistence_frames"], edges, include_lowest=True, right=True
    )
    trend_source = group.assign(trend_bin=bins)
    return (
        trend_source.groupby("trend_bin", observed=True)
        .agg(
            persistence_center=("persistence_frames", "mean"),
            median_slip_deg=("max_slip_deg", "median"),
            samples=("request_id", "size"),
        )
        .reset_index(drop=True)
    )


def configure_plot_style():
    global plt
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "matplotlib is required to draw PNG files. Install it with: "
            "python -m pip install matplotlib"
        ) from exc

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.labelsize": 11,
            "axes.titlesize": 12,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )


def draw_sensitivity_curve(sweep, horizons, plateaus, output_path):
    colors = ["#2166AC", "#D6604D", "#1B7837", "#762A83"]
    fig, ax = plt.subplots(figsize=(7.2, 4.8), constrained_layout=True)

    for index, horizon in enumerate(horizons):
        group = sweep[sweep["horizon_frames"] == horizon]
        ax.plot(
            group["threshold_frames"],
            group["trigger_rate"] * 100.0,
            linewidth=2.0,
            color=colors[index % len(colors)],
            label=f"{horizon} frames",
        )

    for index, (start, end, _) in enumerate(plateaus):
        ax.axvspan(
            start,
            end,
            color="#F1C453",
            alpha=0.24,
            label="Common exact plateau" if index == 0 else None,
        )

    if plateaus:
        start, end, _ = plateaus[0]
        ax.annotate(
            f"First common plateau: {start}-{end} frames",
            xy=((start + end) / 2.0, ax.get_ylim()[1] * 0.77),
            xytext=(max(end + 5, 24), ax.get_ylim()[1] * 0.91),
            arrowprops={"arrowstyle": "->", "color": "#4D4D4D"},
            fontsize=9,
        )

    ax.set_title("Drift trigger sensitivity to persistence threshold")
    ax.set_xlabel("Consecutive frames above 9.3 degrees")
    ax.set_ylabel("Triggered requests (%)")
    ax.grid(color="#D9D9D9", linewidth=0.7, alpha=0.8)
    ax.legend(frameon=False, ncol=2)
    fig.savefig(output_path, dpi=OUTPUT_DPI, bbox_inches="tight")
    plt.close(fig)


def draw_persistence_risk_curve(
    df, risk_horizon, slip_threshold, primary_plateau, output_path
):
    group = df[
        (df["horizon_frames"] == risk_horizon)
        & (df["persistence_frames"] > 0)
    ].copy()
    if group.empty:
        raise ValueError(f"No above-threshold samples for horizon {risk_horizon}.")

    rho = spearman_correlation(
        group["persistence_frames"], group["max_slip_deg"]
    )
    trend = build_binned_trend(group, TREND_BIN_WIDTH)
    fig, ax = plt.subplots(figsize=(7.2, 4.8), constrained_layout=True)

    ax.scatter(
        group["persistence_frames"],
        group["max_slip_deg"],
        s=28,
        color="#2166AC",
        alpha=0.62,
        edgecolors="none",
        label=f"{risk_horizon}-frame requests",
    )
    ax.plot(
        trend["persistence_center"],
        trend["median_slip_deg"],
        color="#B2182B",
        linewidth=2.2,
        marker="o",
        markersize=4,
        label=f"Median in {TREND_BIN_WIDTH}-frame bins",
    )
    ax.axhline(
        slip_threshold,
        color="#4D4D4D",
        linewidth=1.2,
        linestyle="--",
        label=f"Slip threshold ({slip_threshold:g} deg)",
    )
    if primary_plateau is not None:
        start, end, _ = primary_plateau
        ax.axvspan(
            start,
            end,
            color="#F1C453",
            alpha=0.24,
            label=f"First common plateau ({start}-{end})",
        )

    ax.text(
        0.02,
        0.96,
        f"Spearman rho = {rho:.3f}",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontweight="bold",
    )
    ax.set_title("Persistence versus predicted slip severity")
    ax.set_xlabel("Longest consecutive run above threshold (frames)")
    ax.set_ylabel("Maximum absolute slip angle (degrees)")
    ax.grid(color="#D9D9D9", linewidth=0.7, alpha=0.8)
    ax.legend(frameon=False)
    fig.savefig(output_path, dpi=OUTPUT_DPI, bbox_inches="tight")
    plt.close(fig)
    return rho


def write_summary(
    output_path,
    input_path,
    df,
    horizons,
    incomplete_ids,
    duplicates,
    risk_horizon,
    rho,
    plateaus,
):
    lines = [
        "Drift trigger sensitivity summary",
        "=================================",
        f"Input: {input_path}",
        f"Rows: {len(df)}",
        f"Unique requests: {df['request_id'].nunique()}",
        f"Horizons: {', '.join(str(value) for value in horizons)} frames",
        f"Duplicate request/horizon rows: {duplicates}",
        f"Incomplete request IDs: {len(incomplete_ids)}",
        "",
        "Common exact plateaus",
        "---------------------",
    ]
    if plateaus:
        for start, end, counts in plateaus:
            count_text = ", ".join(
                f"{horizon}: {count}"
                for horizon, count in zip(horizons, counts)
            )
            lines.append(
                f"{start}-{end} frames "
                f"({start / SIMULATION_HZ * 1000.0:.1f}-"
                f"{end / SIMULATION_HZ * 1000.0:.1f} ms); {count_text}"
            )
        start, end, _ = plateaus[0]
        midpoint = int(round((start + end) / 2.0))
        lines.extend(
            [
                "",
                f"First plateau midpoint: {midpoint} frames "
                f"({midpoint / SIMULATION_HZ * 1000.0:.1f} ms)",
            ]
        )
    else:
        lines.append("No common exact plateau met the minimum length.")

    lines.extend(
        [
            "",
            "Persistence/severity relationship",
            "---------------------------------",
            f"Risk horizon: {risk_horizon} frames",
            f"Spearman rank correlation: {rho:.6f}",
            "",
            "Limitation",
            "----------",
            "Severity is the maximum predicted tyre slip angle in ProjectD.",
            "The input CSV does not contain future AC ground-truth risk labels.",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    if INPUT_CSV is None:
        input_directory = INPUT_DIRECTORY.expanduser().resolve()
        candidates = sorted(
            input_directory.glob(INPUT_PATTERN),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            raise FileNotFoundError(
                f"No files matching {INPUT_PATTERN!r} in {input_directory}"
            )
        input_path = candidates[0]
    else:
        input_path = Path(INPUT_CSV).expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(
            f"Input CSV not found: {input_path}\n"
            "Change INPUT_CSV or INPUT_DIRECTORY at the top of this script."
        )

    if OUTPUT_DIR is None:
        output_dir = input_path.parent / f"{input_path.stem}_report"
    else:
        output_dir = Path(OUTPUT_DIR).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(input_path, encoding="utf-8-sig")
    validate_data(df)
    df = prepare_data(df)
    horizons = sorted(int(value) for value in df["horizon_frames"].unique())
    maximum_threshold = MAX_THRESHOLD or min(horizons)
    risk_horizon = RISK_HORIZON or max(horizons)
    if risk_horizon not in horizons:
        raise ValueError(
            f"RISK_HORIZON={risk_horizon} is not present; available={horizons}"
        )

    incomplete_ids, duplicates = check_request_alignment(df, horizons)
    sweep = build_threshold_sweep(df, horizons, maximum_threshold)
    plateaus = find_common_plateaus(
        sweep, horizons, MINIMUM_COMMON_PLATEAU_LENGTH
    )
    primary_plateau = plateaus[0] if plateaus else None
    slip_threshold = float(df["threshold_deg"].median())

    sweep_path = output_dir / "drift_threshold_sweep.csv"
    sensitivity_path = output_dir / "drift_trigger_sensitivity_curve.png"
    risk_path = output_dir / "drift_persistence_risk_curve.png"
    summary_path = output_dir / "drift_sensitivity_summary.txt"

    sweep.to_csv(sweep_path, index=False, encoding="utf-8-sig")
    configure_plot_style()
    draw_sensitivity_curve(sweep, horizons, plateaus, sensitivity_path)
    rho = draw_persistence_risk_curve(
        df,
        risk_horizon,
        slip_threshold,
        primary_plateau,
        risk_path,
    )
    write_summary(
        summary_path,
        input_path,
        df,
        horizons,
        incomplete_ids,
        duplicates,
        risk_horizon,
        rho,
        plateaus,
    )

    print(f"Input rows: {len(df)}")
    print(f"Unique requests: {df['request_id'].nunique()}")
    print(f"Duplicate request/horizon rows: {duplicates}")
    print(f"Incomplete request IDs: {len(incomplete_ids)}")
    if plateaus:
        print("Common exact plateaus:")
        for start, end, counts in plateaus:
            print(f"  {start}-{end} frames; trigger counts={counts}")
    else:
        print("No common exact plateau found.")
    print(f"Spearman correlation ({risk_horizon} frames): {rho:.6f}")
    print(f"Output directory: {output_dir}")


if __name__ == "__main__":
    main()
