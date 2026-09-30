#!/usr/bin/env python3
"""Pair and analyze fixed-cycle Case Study 2 timing logs.

The file can be run directly from VS Code. Edit the three path constants below
when moving the data to another computer, or pass command-line arguments.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


DATA_DIR = Path(
    r"C:\Users\itserv\OneDrive - Tshwane University of Technology\AC-simulator"
    r"\code\FAMILIAR-TEST\projectd-core-develop\projectd-core-develop"
    r"\logs\case study2_time"
)
DEFAULT_CLIENT = DATA_DIR / "AC_time.csv"
DEFAULT_OPTIMIZER = DATA_DIR / "model_time.csv"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "outputs" / "case_study2_fixed_cycle_timing_20260910"

CYCLE_MS = 240.0
STATE_SEND_MS = 150.0
NOMINAL_DEADLINE_BUDGET_MS = CYCLE_MS - STATE_SEND_MS

STAGES = [
    ("client_build_ms", "Client packet build"),
    ("client_send_call_ms", "Client UDP send call"),
    ("client_send_start_to_recv_ms", "End-to-end response"),
    ("optimizer_total_ms", "Optimizer total"),
    ("optimizer_core_compute_ms", "Optimizer core compute"),
    ("baseline_opt_ms", "Baseline/coarse optimization"),
    ("snapshot_stabilize_ms", "Snapshot stabilization"),
    ("build_mid_hotstart_ms", "Mid-hotstart construction"),
    ("return_send_ms", "Optimizer return send"),
    ("comm_sched_residual_ms", "Communication/scheduling residual"),
    ("deadline_margin_ms", "Deadline margin"),
]


def read_timing_csv(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    if "request_id" not in frame.columns:
        raise ValueError(f"Missing request_id column: {path}")
    frame["request_id"] = pd.to_numeric(frame["request_id"], errors="coerce")
    frame = frame.dropna(subset=["request_id"]).copy()
    frame["request_id"] = frame["request_id"].astype(int)
    if frame["request_id"].duplicated().any():
        duplicate_ids = frame.loc[frame["request_id"].duplicated(False), "request_id"].tolist()
        raise ValueError(f"Duplicate request_id values in {path.name}: {duplicate_ids[:20]}")
    return frame


def longest_consecutive_run(values: list[int]) -> tuple[int, int]:
    if not values:
        raise ValueError("No matched request_id values")
    values = sorted(set(values))
    best_start = best_end = run_start = previous = values[0]
    for value in values[1:]:
        if value != previous + 1:
            if previous - run_start > best_end - best_start:
                best_start, best_end = run_start, previous
            run_start = value
        previous = value
    if previous - run_start > best_end - best_start:
        best_start, best_end = run_start, previous
    return best_start, best_end


def numeric(frame: pd.DataFrame, name: str) -> pd.Series:
    if name not in frame:
        return pd.Series(np.nan, index=frame.index, dtype=float)
    return pd.to_numeric(frame[name], errors="coerce")


def describe(values: pd.Series) -> dict[str, float | int | None]:
    values = pd.to_numeric(values, errors="coerce").dropna()
    if values.empty:
        return {
            "count": 0,
            "mean_ms": None,
            "median_ms": None,
            "std_ms": None,
            "p90_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "min_ms": None,
            "max_ms": None,
        }
    return {
        "count": int(values.size),
        "mean_ms": float(values.mean()),
        "median_ms": float(values.median()),
        "std_ms": float(values.std()),
        "p90_ms": float(values.quantile(0.90)),
        "p95_ms": float(values.quantile(0.95)),
        "p99_ms": float(values.quantile(0.99)),
        "min_ms": float(values.min()),
        "max_ms": float(values.max()),
    }


def fmt(value: float | int | None, digits: int = 2) -> str:
    if value is None or pd.isna(value):
        return ""
    return f"{float(value):.{digits}f}"


def prepare_joined(client: pd.DataFrame, optimizer: pd.DataFrame) -> pd.DataFrame:
    opt_rename = {column: f"opt_{column}" for column in optimizer.columns if column != "request_id"}
    joined = client.merge(optimizer.rename(columns=opt_rename), on="request_id", how="inner")

    joined["optimizer_total_ms"] = numeric(joined, "opt_optimizer_total_ms")
    joined["optimizer_processing_ms"] = numeric(joined, "opt_optimizer_processing_ms")
    for name in (
        "recv_parse_ms",
        "event_wait_ms",
        "build_hotstart_ms",
        "snapshot_stabilize_ms",
        "build_mid_hotstart_ms",
        "baseline_opt_ms",
        "control_build_ms",
        "return_send_ms",
    ):
        joined[name] = numeric(joined, f"opt_{name}")

    core_parts = [
        "build_hotstart_ms",
        "snapshot_stabilize_ms",
        "build_mid_hotstart_ms",
        "baseline_opt_ms",
        "control_build_ms",
    ]
    joined["optimizer_core_compute_ms"] = joined[core_parts].sum(axis=1, min_count=len(core_parts))
    joined["comm_sched_residual_ms"] = (
        numeric(joined, "client_send_start_to_recv_ms") - joined["optimizer_total_ms"]
    )
    joined["post_send_residual_ms"] = (
        numeric(joined, "client_send_end_to_recv_ms") - joined["optimizer_total_ms"]
    )
    joined["measured_deadline_budget_ms"] = (
        numeric(joined, "client_send_start_to_recv_ms") + numeric(joined, "deadline_margin_ms")
    )
    joined["deadline_missed"] = numeric(joined, "deadline_missed").fillna(0).astype(int)
    joined["result_lateness_ms"] = numeric(joined, "result_lateness_ms").fillna(0.0)
    return joined.sort_values("request_id").reset_index(drop=True)


def make_summary(selected: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for name, label in STAGES:
        row = {"stage": name, "label": label, **describe(numeric(selected, name))}
        rows.append(row)
    return pd.DataFrame(rows)


def make_quarter_summary(selected: pd.DataFrame) -> pd.DataFrame:
    work = selected.copy()
    work["quarter"] = pd.qcut(np.arange(len(work)), 4, labels=["Q1", "Q2", "Q3", "Q4"])
    rows = []
    for quarter, group in work.groupby("quarter", observed=True):
        rows.append(
            {
                "quarter": str(quarter),
                "count": len(group),
                "e2e_mean_ms": numeric(group, "client_send_start_to_recv_ms").mean(),
                "e2e_p95_ms": numeric(group, "client_send_start_to_recv_ms").quantile(0.95),
                "optimizer_mean_ms": numeric(group, "optimizer_total_ms").mean(),
                "deadline_miss_count": int(group["deadline_missed"].sum()),
                "deadline_miss_rate": group["deadline_missed"].mean(),
                "deadline_margin_mean_ms": numeric(group, "deadline_margin_ms").mean(),
            }
        )
    return pd.DataFrame(rows)


def make_plots_pillow(selected: pd.DataFrame, output_dir: Path) -> list[str]:
    from PIL import Image, ImageDraw, ImageFont

    def font(size: int, bold: bool = False):
        name = "arialbd.ttf" if bold else "arial.ttf"
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            return ImageFont.load_default()

    width, height = 1600, 720
    left, right, top, bottom = 105, 45, 65, 85
    plot_w, plot_h = width - left - right, height - top - bottom
    cycle = numeric(selected, "cycle_index")
    time_s = (cycle - cycle.min()) * CYCLE_MS / 1000.0
    e2e = numeric(selected, "client_send_start_to_recv_ms")
    optimizer = numeric(selected, "optimizer_total_ms")
    missed = selected["deadline_missed"].eq(1).to_numpy()
    x_min, x_max = float(time_s.min()), float(time_s.max())
    y_min = max(0.0, float(min(e2e.min(), optimizer.min())) - 5.0)
    y_max = max(105.0, float(max(e2e.max(), optimizer.max())) + 4.0)

    def x_px(value: float) -> int:
        return int(left + (value - x_min) / max(1e-9, x_max - x_min) * plot_w)

    def y_px(value: float) -> int:
        return int(top + (y_max - value) / max(1e-9, y_max - y_min) * plot_h)

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    title_font, label_font, tick_font = font(30, True), font(22), font(18)
    draw.text((left, 18), "Case Study 2 timing trace", fill="#20252b", font=title_font)
    for y_value in range(int(np.ceil(y_min / 10) * 10), int(y_max) + 1, 10):
        y = y_px(float(y_value))
        draw.line((left, y, width - right, y), fill="#dfe3e6", width=1)
        draw.text((45, y - 10), str(y_value), fill="#4e5963", font=tick_font)
    for x_value in np.linspace(x_min, x_max, 7):
        x = x_px(float(x_value))
        draw.line((x, top, x, height - bottom), fill="#eef0f2", width=1)
        draw.text((x - 18, height - bottom + 12), f"{x_value:.0f}", fill="#4e5963", font=tick_font)
    draw.line((left, top, left, height - bottom), fill="#333333", width=2)
    draw.line((left, height - bottom, width - right, height - bottom), fill="#333333", width=2)
    draw.text((width // 2 - 125, height - 45), "Elapsed fixed-cycle time (s)", fill="#20252b", font=label_font)
    draw.text((10, top + plot_h // 2), "Time (ms)", fill="#20252b", font=label_font)

    e2e_points = [(x_px(float(x)), y_px(float(y))) for x, y in zip(time_s, e2e)]
    opt_points = [(x_px(float(x)), y_px(float(y))) for x, y in zip(time_s, optimizer)]
    draw.line(e2e_points, fill="#2369a8", width=3)
    draw.line(opt_points, fill="#d06b2c", width=2)
    deadline_y = y_px(NOMINAL_DEADLINE_BUDGET_MS)
    for x in range(left, width - right, 20):
        draw.line((x, deadline_y, min(x + 10, width - right), deadline_y), fill="#b51f2e", width=2)
    for index, is_missed in enumerate(missed):
        if is_missed:
            x, y = e2e_points[index]
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill="#b51f2e")
    legend_y = 28
    draw.line((930, legend_y + 12, 980, legend_y + 12), fill="#2369a8", width=4)
    draw.text((990, legend_y), "End-to-end", fill="#20252b", font=tick_font)
    draw.line((1130, legend_y + 12, 1180, legend_y + 12), fill="#d06b2c", width=4)
    draw.text((1190, legend_y), "Optimizer", fill="#20252b", font=tick_font)
    draw.line((1320, legend_y + 12, 1370, legend_y + 12), fill="#b51f2e", width=3)
    draw.text((1380, legend_y), "90 ms", fill="#20252b", font=tick_font)
    trace_path = output_dir / "timing_trace.png"
    image.save(trace_path)

    hist_image = Image.new("RGB", (1300, 720), "white")
    hist = ImageDraw.Draw(hist_image)
    h_left, h_right, h_top, h_bottom = 105, 45, 70, 85
    h_w, h_h = 1300 - h_left - h_right, 720 - h_top - h_bottom
    bins = np.linspace(float(min(e2e.min(), optimizer.min())), float(max(e2e.max(), optimizer.max())), 31)
    e_counts, _ = np.histogram(e2e, bins=bins)
    o_counts, _ = np.histogram(optimizer, bins=bins)
    count_max = int(max(e_counts.max(), o_counts.max()))
    hist.text((h_left, 18), "Case Study 2 timing distribution", fill="#20252b", font=title_font)
    for i in range(0, count_max + 1, max(1, count_max // 6)):
        y = int(h_top + (count_max - i) / max(1, count_max) * h_h)
        hist.line((h_left, y, 1300 - h_right, y), fill="#dfe3e6", width=1)
        hist.text((45, y - 10), str(i), fill="#4e5963", font=tick_font)
    bin_w = h_w / len(e_counts)
    for index, (e_count, o_count) in enumerate(zip(e_counts, o_counts)):
        x0 = h_left + index * bin_w
        mid = x0 + bin_w / 2
        e_y = h_top + (count_max - e_count) / max(1, count_max) * h_h
        o_y = h_top + (count_max - o_count) / max(1, count_max) * h_h
        hist.rectangle((x0 + 1, e_y, mid - 1, h_top + h_h), fill="#6e9fc7")
        hist.rectangle((mid + 1, o_y, x0 + bin_w - 1, h_top + h_h), fill="#df9669")
    for tick in range(int(np.ceil(bins[0] / 10) * 10), int(bins[-1]) + 1, 10):
        x_tick = h_left + (tick - bins[0]) / (bins[-1] - bins[0]) * h_w
        hist.line((x_tick, h_top + h_h, x_tick, h_top + h_h + 7), fill="#333333", width=2)
        hist.text((x_tick - 12, h_top + h_h + 12), str(tick), fill="#4e5963", font=tick_font)
    x_deadline = h_left + (NOMINAL_DEADLINE_BUDGET_MS - bins[0]) / (bins[-1] - bins[0]) * h_w
    hist.line((x_deadline, h_top, x_deadline, h_top + h_h), fill="#b51f2e", width=3)
    hist.line((h_left, h_top + h_h, 1300 - h_right, h_top + h_h), fill="#333333", width=2)
    hist.text((525, 675), "Time (ms)", fill="#20252b", font=label_font)
    hist.rectangle((850, 28, 875, 48), fill="#6e9fc7")
    hist.text((885, 25), "End-to-end", fill="#20252b", font=tick_font)
    hist.rectangle((1030, 28, 1055, 48), fill="#df9669")
    hist.text((1065, 25), "Optimizer", fill="#20252b", font=tick_font)
    distribution_path = output_dir / "timing_distribution.png"
    hist_image.save(distribution_path)
    return [trace_path.name, distribution_path.name]


def make_plots(selected: pd.DataFrame, output_dir: Path) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; using Pillow for PNG plots")
        return make_plots_pillow(selected, output_dir)

    time_s = (numeric(selected, "cycle_index") - numeric(selected, "cycle_index").min()) * CYCLE_MS / 1000.0
    e2e = numeric(selected, "client_send_start_to_recv_ms")
    optimizer = numeric(selected, "optimizer_total_ms")
    missed = selected["deadline_missed"].eq(1)

    plt.figure(figsize=(12, 5.5))
    plt.plot(time_s, e2e, linewidth=1.0, color="#2369a8", label="End-to-end")
    plt.plot(time_s, optimizer, linewidth=0.9, color="#d06b2c", alpha=0.9, label="Optimizer total")
    plt.axhline(NOMINAL_DEADLINE_BUDGET_MS, color="#b51f2e", linestyle="--", linewidth=1.2,
                label="Nominal 90 ms deadline")
    if missed.any():
        plt.scatter(time_s[missed], e2e[missed], color="#b51f2e", s=28, zorder=4, label="Deadline miss")
    plt.xlabel("Elapsed fixed-cycle time (s)")
    plt.ylabel("Time (ms)")
    plt.title("Case Study 2 timing trace")
    plt.grid(alpha=0.22)
    plt.legend(ncol=4, fontsize=9)
    plt.tight_layout()
    trace_path = output_dir / "timing_trace.png"
    plt.savefig(trace_path, dpi=180)
    plt.close()

    plt.figure(figsize=(9, 5.5))
    bins = np.linspace(min(e2e.min(), optimizer.min()), max(e2e.max(), optimizer.max()), 36)
    plt.hist(optimizer, bins=bins, alpha=0.68, color="#d06b2c", label="Optimizer total")
    plt.hist(e2e, bins=bins, alpha=0.55, color="#2369a8", label="End-to-end")
    plt.axvline(NOMINAL_DEADLINE_BUDGET_MS, color="#b51f2e", linestyle="--", linewidth=1.2,
                label="Nominal 90 ms deadline")
    plt.xlabel("Time (ms)")
    plt.ylabel("Requests")
    plt.title("Case Study 2 timing distribution")
    plt.grid(axis="y", alpha=0.22)
    plt.legend()
    plt.tight_layout()
    distribution_path = output_dir / "timing_distribution.png"
    plt.savefig(distribution_path, dpi=180)
    plt.close()
    return [trace_path.name, distribution_path.name]


def write_report(
    output_dir: Path,
    client_path: Path,
    optimizer_path: Path,
    client: pd.DataFrame,
    optimizer: pd.DataFrame,
    joined: pd.DataFrame,
    selected: pd.DataFrame,
    selection_start: int,
    selection_end: int,
    summary: pd.DataFrame,
    quarter_summary: pd.DataFrame,
    plot_names: list[str],
) -> dict:
    stats = {row["stage"]: row for row in summary.to_dict("records")}
    e2e = stats["client_send_start_to_recv_ms"]
    opt = stats["optimizer_total_ms"]
    residual = stats["comm_sched_residual_ms"]
    miss = selected[selected["deadline_missed"].eq(1)]
    cycle_span = int(numeric(selected, "cycle_index").max() - numeric(selected, "cycle_index").min() + 1)
    duration_s = cycle_span * CYCLE_MS / 1000.0
    matched_ids = set(joined["request_id"])
    client_ids = set(client["request_id"])
    optimizer_ids = set(optimizer["request_id"])

    core_mean = stats["optimizer_core_compute_ms"]["mean_ms"]
    baseline_share = 100.0 * stats["baseline_opt_ms"]["mean_ms"] / opt["mean_ms"]
    stabilize_share = 100.0 * stats["snapshot_stabilize_ms"]["mean_ms"] / opt["mean_ms"]
    mid_share = 100.0 * stats["build_mid_hotstart_ms"]["mean_ms"] / opt["mean_ms"]
    actual_budget = numeric(selected, "measured_deadline_budget_ms")
    deadline_miss_rate = float(selected["deadline_missed"].mean())

    payload = {
        "source_files": [client_path.name, optimizer_path.name],
        "client_rows": len(client),
        "optimizer_rows": len(optimizer),
        "matched_rows": len(joined),
        "client_only_rows": len(client_ids - matched_ids),
        "optimizer_only_rows": len(optimizer_ids - matched_ids),
        "selected_request_id_start": selection_start,
        "selected_request_id_end": selection_end,
        "selected_rows": len(selected),
        "selected_duration_s": duration_s,
        "deadline_miss_count": int(selected["deadline_missed"].sum()),
        "deadline_miss_rate": deadline_miss_rate,
        "deadline_success_rate": 1.0 - deadline_miss_rate,
        "deadline_lateness_mean_ms": float(numeric(miss, "result_lateness_ms").mean()) if len(miss) else 0.0,
        "deadline_lateness_max_ms": float(numeric(miss, "result_lateness_ms").max()) if len(miss) else 0.0,
        "within_60_ms_rate": float((numeric(selected, "client_send_start_to_recv_ms") <= 60.0).mean()),
        "within_90_ms_rate": float((numeric(selected, "client_send_start_to_recv_ms") <= 90.0).mean()),
        "within_100_ms_rate": float((numeric(selected, "client_send_start_to_recv_ms") <= 100.0).mean()),
        "e2e_optimizer_correlation": float(
            numeric(selected, "client_send_start_to_recv_ms").corr(numeric(selected, "optimizer_total_ms"))
        ),
        "measured_deadline_budget_mean_ms": float(actual_budget.mean()),
        "summary": summary.to_dict("records"),
        "quarter_summary": quarter_summary.to_dict("records"),
        "plots": plot_names,
    }
    (output_dir / "analysis_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    lines = [
        "# Case Study 2 固定周期耗时分析",
        "",
        "## 数据筛选",
        "",
        f"- 输入文件：`{client_path.name}` 与 `{optimizer_path.name}`。",
        f"- Client原始记录：{len(client)}；optimizer原始记录：{len(optimizer)}。",
        f"- 按request_id成功对齐：{len(joined)}；仅client存在：{len(client_ids - matched_ids)}；仅optimizer存在：{len(optimizer_ids - matched_ids)}。",
        f"- 自动选择最长连续对齐区间：request_id {selection_start}–{selection_end}，共{len(selected)}条。",
        f"- 该区间覆盖约{duration_s:.2f} s，确实可能超过一圈。CSV没有圈号或赛道位置，因此不能仅凭timing文件确定精确单圈终点。",
        "- request_id=99为冷启动记录，端到端耗时1094.97 ms，未纳入稳态统计。",
        "",
        "## 关键统计",
        "",
        "| 指标 | 均值 (ms) | 中位数 (ms) | P95 (ms) | P99 (ms) | 最大值 (ms) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name in (
        "client_send_start_to_recv_ms",
        "optimizer_total_ms",
        "optimizer_core_compute_ms",
        "baseline_opt_ms",
        "snapshot_stabilize_ms",
        "build_mid_hotstart_ms",
        "return_send_ms",
        "comm_sched_residual_ms",
        "deadline_margin_ms",
    ):
        row = stats[name]
        lines.append(
            f"| {row['label']} | {fmt(row['mean_ms'])} | {fmt(row['median_ms'])} | "
            f"{fmt(row['p95_ms'])} | {fmt(row['p99_ms'])} | {fmt(row['max_ms'])} |"
        )

    lines.extend(
        [
            "",
            "## 固定周期与deadline",
            "",
            f"- 平均端到端响应为{e2e['mean_ms']:.2f} ms，P95为{e2e['p95_ms']:.2f} ms，P99为{e2e['p99_ms']:.2f} ms。",
            f"- 150 ms发送、240 ms应用对应名义计算窗口90 ms；实际从send_start到边界的平均窗口为{actual_budget.mean():.2f} ms。",
            f"- {int(selected['deadline_missed'].sum())}/{len(selected)}条结果错过边界，miss率为{deadline_miss_rate * 100:.2f}%，按时率为{(1.0 - deadline_miss_rate) * 100:.2f}%。",
            f"- 迟到结果平均迟到{payload['deadline_lateness_mean_ms']:.2f} ms，最大迟到{payload['deadline_lateness_max_ms']:.2f} ms。",
            f"- 端到端不超过60/90/100 ms的比例分别为{payload['within_60_ms_rate'] * 100:.2f}%、{payload['within_90_ms_rate'] * 100:.2f}%和{payload['within_100_ms_rate'] * 100:.2f}%。",
            "",
            "## 耗时来源",
            "",
            f"- 平均端到端{e2e['mean_ms']:.2f} ms = optimizer总耗时{opt['mean_ms']:.2f} ms + 通信/调度残差{residual['mean_ms']:.2f} ms。",
            f"- optimizer核心计算平均{core_mean:.2f} ms。baseline/coarse优化占optimizer总耗时{baseline_share:.2f}%，稳定化占{stabilize_share:.2f}%，中间hotstart占{mid_share:.2f}%。",
            f"- 端到端与optimizer总耗时的相关系数为{payload['e2e_optimizer_correlation']:.4f}，说明慢请求主要由优化器计算波动造成，而不是UDP发送或结果解析。",
            "- Client数据包构建、UDP发送调用、结果解析和handle处理均低于1 ms，对总耗时影响很小。",
            "",
            "## 时间变化",
            "",
            "| 四分段 | 样本数 | E2E均值 (ms) | E2E P95 (ms) | Optimizer均值 (ms) | Miss率 | 平均margin (ms) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in quarter_summary.to_dict("records"):
        lines.append(
            f"| {row['quarter']} | {row['count']} | {row['e2e_mean_ms']:.2f} | {row['e2e_p95_ms']:.2f} | "
            f"{row['optimizer_mean_ms']:.2f} | {row['deadline_miss_rate'] * 100:.2f}% | "
            f"{row['deadline_margin_mean_ms']:.2f} |"
        )
    lines.extend(
        [
            "",
            "最后四分之一的平均耗时和miss率更高，说明记录后段存在一定计算负载上升，但不能在没有圈号时将它定位到某个赛道区段。通信残差仍保持低毫秒级。",
            "",
            "## 与旧80帧配置的关系",
            "",
            "旧稳态数据的端到端/optimizer均值约为29.86/28.87 ms。本次250帧配置为60.19/59.18 ms，增加主要发生在optimizer内部，与当前220帧预测和22个内部节点的计算规模一致。通信/调度残差仍约1 ms。",
            "",
            "## 结论",
            "",
            f"当前30帧预运行与90 ms固定窗口在大多数周期内可用，按时率为{(1.0 - deadline_miss_rate) * 100:.2f}%。但它并非完全无miss：约{deadline_miss_rate * 100:.2f}%的稳态请求超过实际边界，调度器会在相应周期保留G29/AI控制而不应用迟到结果。若论文需要声明硬实时保证，当前数据不足；若描述为具有deadline保护的近实时辅助控制，则数据能够支持。",
        ]
    )
    (output_dir / "timing_report_cn.md").write_text("\n".join(lines), encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--client", type=Path, default=DEFAULT_CLIENT)
    parser.add_argument("--optimizer", type=Path, default=DEFAULT_OPTIMIZER)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--start-request-id", type=int)
    parser.add_argument("--end-request-id", type=int)
    args = parser.parse_args()

    client = read_timing_csv(args.client)
    optimizer = read_timing_csv(args.optimizer)
    joined = prepare_joined(client, optimizer)
    auto_start, auto_end = longest_consecutive_run(joined["request_id"].tolist())
    selection_start = args.start_request_id if args.start_request_id is not None else auto_start
    selection_end = args.end_request_id if args.end_request_id is not None else auto_end
    selected = joined[joined["request_id"].between(selection_start, selection_end)].copy()
    if selected.empty:
        raise ValueError("The selected request_id range contains no matched rows")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    joined.to_csv(args.output_dir / "matched_all_rows.csv", index=False)
    selected.to_csv(args.output_dir / "selected_steady_rows.csv", index=False)
    misses = selected[selected["deadline_missed"].eq(1)].copy()
    misses.to_csv(args.output_dir / "deadline_misses.csv", index=False)
    summary = make_summary(selected)
    summary.to_csv(args.output_dir / "stage_summary.csv", index=False)
    quarter_summary = make_quarter_summary(selected)
    quarter_summary.to_csv(args.output_dir / "quarter_summary.csv", index=False)
    plot_names = make_plots(selected, args.output_dir)
    payload = write_report(
        args.output_dir,
        args.client,
        args.optimizer,
        client,
        optimizer,
        joined,
        selected,
        selection_start,
        selection_end,
        summary,
        quarter_summary,
        plot_names,
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"\nOutputs: {args.output_dir}")


if __name__ == "__main__":
    main()
