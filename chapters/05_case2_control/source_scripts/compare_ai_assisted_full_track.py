"""Compare run-level GLOBAL results. Never reads raw telemetry or changes preprocessing."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
CONDITIONS = ('AI-only', 'optimizer-assisted')
SETTINGS = (80, 85, 90, 95, 100)
PRIMARY = {
    'lap_lap_time_s': ('Circuit time', 's'),
    'driving_speed_mean': ('Mean speed', 'km/h'),
    'driving_wheelSlip_p95': ('P95 of per-sample maximum wheelSlip', 'logged scale'),
    'driving_absSlipAngle_p95': ('P95 of per-sample maximum absolute slip angle', 'logged scale'),
    'tracking_cte_xz_mean_m': ('Mean nearest-reference-point distance', 'm'),
    'tracking_cte_xz_p95_m': ('P95 nearest-reference-point distance', 'm'),
}


def source_key(value):
    return str(value).replace('\\', '/').casefold()


def read_inputs(folder, allow_partial=False):
    summary_path = folder / 'run_summary.csv'
    status_path = folder / 'processing_status.csv'
    protocol_path = folder / 'analysis_protocol.json'
    if not summary_path.is_file() or not status_path.is_file():
        raise FileNotFoundError('Run run_pipeline_from_original.py first: summary and status are required.')
    paths = [summary_path, status_path] + ([protocol_path] if protocol_path.exists() else [])
    blobs = {p.name: p.read_bytes() for p in paths}
    if any(p.read_bytes() != blobs[p.name] for p in paths):
        raise RuntimeError('Input files changed while being read; wait for preprocessing to finish.')
    df = pd.read_csv(io.BytesIO(blobs[summary_path.name]), dtype={'run_id': str})
    status = pd.read_csv(io.BytesIO(blobs[status_path.name]))
    protocol = json.loads(blobs[protocol_path.name]) if protocol_path.name in blobs else None
    warnings = []
    if df.empty:
        raise ValueError('No successful runs in summary.')
    if protocol is None:
        warnings.append('No analysis_protocol.json: preprocessing completion is unconfirmed.')
    else:
        if protocol_path.stat().st_mtime_ns < max(summary_path.stat().st_mtime_ns, status_path.stat().st_mtime_ns):
            warnings.append('Protocol predates summary/status; preprocessing may still be running.')
        if protocol.get('successful_files') != len(df) or protocol.get('selected_files') != len(status):
            warnings.append('Protocol counts differ from current summary/status; possible incomplete rerun.')
        if protocol.get('folders') or protocol.get('limit'):
            warnings.append('Preprocessing selected a subset of inputs.')
    needed = ['source', 'folder', 'run_id', 'AIpush', 'condition', 'lap_lap_complete',
              'lap_selected_completedLaps', 'driving_aggregation', 'tracking_aggregation',
              'tracking_matching_plane', 'tracking_distance_definition', 'driving_despike_enabled']
    missing = set(needed) - set(df)
    if missing:
        raise ValueError(f'Missing required summary columns: {sorted(missing)}')
    if not {'source', 'status'}.issubset(status.columns):
        raise ValueError('Invalid processing_status.csv schema.')
    df['_key'] = df.source.map(source_key)
    status['_key'] = status.source.map(source_key)
    if df['_key'].duplicated().any() or status['_key'].duplicated().any():
        raise ValueError('Duplicate source records; comparison would double-count runs.')
    if df.duplicated(['folder', 'run_id']).any():
        raise ValueError('Duplicate folder/run_id.')
    if not status.status.isin(['ok', 'failed']).all():
        raise ValueError('Unknown processing status.')
    if set(df['_key']) != set(status.loc[status.status.eq('ok'), '_key']):
        raise ValueError('Summary and successful status entries do not match; wait for processing to finish.')
    if (status.status != 'ok').any():
        warnings.append(f'{int((status.status != "ok").sum())} preprocessing failures; successful runs only.')
    for row in df.itertuples():
        match = re.fullmatch(r'(80|85|90|95|100)-(ai|shared)', row.folder)
        if not match or float(row.AIpush) != int(match[1]):
            raise ValueError(f'Inconsistent folder/AIpush: {row.folder}')
        expected = CONDITIONS[0] if match[2] == 'ai' else CONDITIONS[1]
        if row.condition != expected:
            raise ValueError(f'Inconsistent condition: {row.folder}')
    if not df.lap_lap_complete.astype(str).str.lower().isin(['true', '1', '1.0']).all():
        raise ValueError('Incomplete laps in summary; do not silently compare them as full circuits.')
    for col in needed[6:]:
        if col == 'lap_lap_complete':
            continue
        if df[col].isna().any() or df[col].astype(str).nunique() != 1:
            raise ValueError(f'Mixed or missing analysis settings: {col}')
    df['AIpush'] = df.AIpush.astype(int)
    for push in SETTINGS:
        for condition in CONDITIONS:
            if not ((df.AIpush == push) & (df.condition == condition)).any():
                warnings.append(f'Missing condition: AIpush={push}, {condition}.')
    if warnings and not allow_partial:
        raise ValueError('\n'.join(warnings) + '\nFinish the full pipeline, or explicitly use --allow-partial.')
    hashes = {name: hashlib.sha256(blob).hexdigest() for name, blob in blobs.items()}
    return df.drop(columns='_key'), status.drop(columns='_key'), protocol, warnings, hashes


def metric_info(column):
    if column in PRIMARY:
        return PRIMARY[column]
    if column.startswith('driving_speed_'):
        return column, 'km/h'
    if column.startswith('driving_pct_') or column.startswith('tracking_pct_'):
        return column, '%'
    if column.startswith('driving_a_'):
        return column, 'm/s^2'
    if column.startswith('tracking_cte_') and column.endswith('_m'):
        return column, 'm'
    return column, 'logged scale'


def select_metrics(df):
    # No counters, duplicated lap-time fields, unverified heading or progress error.
    prefixes = ('driving_speed_', 'driving_pct_a_', 'driving_a_', 'driving_wheelSlip_',
                'driving_absSlipAngle_', 'tracking_cte_', 'tracking_pct_cte_')
    return [c for c in df if c == 'lap_lap_time_s' or c.startswith(prefixes)]


def compare(df):
    metrics = select_metrics(df)
    if not metrics:
        raise ValueError('No supported metrics found.')
    rows, differences = [], []
    for push in SETTINGS:
        for metric in metrics:
            groups = {}
            title, unit = metric_info(metric)
            for condition in CONDITIONS:
                part = df.loc[(df.AIpush == push) & (df.condition == condition), metric]
                numeric = pd.to_numeric(part, errors='coerce').to_numpy(float)
                values = numeric[np.isfinite(numeric)]
                n = len(values)
                record = dict(AIpush=push, condition=condition, metric=metric, title=title, unit=unit,
                              n_runs=len(part), n_valid=n, n_missing_or_invalid=len(part)-n,
                              mean=float(values.mean()) if n else np.nan,
                              std=float(values.std(ddof=1)) if n > 1 else np.nan,
                              median=float(np.median(values)) if n else np.nan,
                              minimum=float(values.min()) if n else np.nan,
                              maximum=float(values.max()) if n else np.nan)
                rows.append(record)
                groups[condition] = record
            ai, assisted = (groups[c] for c in CONDITIONS)
            valid = ai['n_valid'] > 0 and assisted['n_valid'] > 0
            delta = assisted['mean'] - ai['mean'] if valid else np.nan
            # Signed reference denominator retained; do not call this improvement.
            relative = delta / ai['mean'] * 100 if valid and ai['mean'] != 0 else np.nan
            differences.append(dict(AIpush=push, metric=metric, title=title, unit=unit,
                                    ai_n=ai['n_valid'], assisted_n=assisted['n_valid'],
                                    ai_mean=ai['mean'], ai_std=ai['std'],
                                    assisted_mean=assisted['mean'], assisted_std=assisted['std'],
                                    delta_assisted_minus_ai=delta, relative_change_pct=relative,
                                    delta_unit='percentage points' if unit == '%' else unit,
                                    comparison_status='available' if valid else 'missing_group_or_metric'))
    return pd.DataFrame(rows), pd.DataFrame(differences)


def make_plots(df, groups, output):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return 'matplotlib unavailable: CSV results retained; install matplotlib to create figures.'
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.2), constrained_layout=True)
    for ax, (metric, (title, unit)) in zip(axes.flat, PRIMARY.items()):
        for j, condition in enumerate(CONDITIONS):
            color = ('#2676b8', '#d97924')[j]
            offset = (-0.13, 0.13)[j]
            for k, push in enumerate(SETTINGS):
                part = df.loc[(df.AIpush == push) & (df.condition == condition)]
                if metric not in part:
                    continue
                values = pd.to_numeric(part[metric], errors='coerce').to_numpy(float)
                values = values[np.isfinite(values)]
                if not len(values):
                    continue
                spread = np.linspace(-0.05, 0.05, len(values)) if len(values) > 1 else [0]
                ax.scatter(k + offset + np.asarray(spread), values, color=color, alpha=.35, s=18)
                ax.errorbar(k + offset, values.mean(),
                            yerr=values.std(ddof=1) if len(values)>1 else None,
                            fmt='o', capsize=3, color=color,
                            label=condition if k == next(i for i, p in enumerate(SETTINGS)
                                                       if ((df.AIpush == p) & (df.condition == condition)).any()) else None)
        ax.set(title=title, xlabel='AIpush', ylabel=unit, xticks=range(5), xticklabels=SETTINGS)
        ax.grid(axis='y', alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    if handles:
        axes.flat[0].legend(handles, labels, fontsize=8)
    fig.suptitle('Whole-circuit run summaries: individual runs and mean ± sample SD', fontsize=12)
    fig.savefig(output/'ai_assisted_comparison.png', dpi=220)
    fig.savefig(output/'ai_assisted_comparison.pdf')
    plt.close(fig)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, default=ROOT/'full_track_results')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'ai_assisted_comparison')
    parser.add_argument('--allow-partial', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    if args.input_dir.resolve() == args.output_dir.resolve():
        parser.error('Keep comparison outputs separate from preprocessing inputs.')
    df, status, protocol, warnings, hashes = read_inputs(args.input_dir, args.allow_partial)
    groups, differences = compare(df)
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    df.to_csv(output/'included_run_summary.csv', index=False, encoding='utf-8-sig')
    groups.to_csv(output/'group_statistics.csv', index=False, encoding='utf-8-sig')
    differences.to_csv(output/'ai_assisted_differences.csv', index=False, encoding='utf-8-sig')
    plot_warning = None if args.no_plots else make_plots(df, groups, output)
    if plot_warning:
        warnings.append(plot_warning)
    meta = dict(input_dir=str(args.input_dir.resolve()), input_sha256=hashes,
                input_protocol=protocol, runs=len(df), warnings=warnings,
                allow_partial=args.allow_partial, figures_generated=not args.no_plots and not plot_warning,
                aggregation='equal-weight run-level means; sample SD across runs',
                comparison='unpaired descriptive differences, assisted minus AI',
                excluded_metrics='unverified heading/progress, counters, duplicate lap time',
                no_hypothesis_tests=True, no_composite_score=True)
    (output/'comparison_protocol.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# AI 与辅助驾驶：全赛道描述性对比', '',
             f'使用 {len(df)} 个成功运行记录。各运行等权；不将遥测行混池、不强制配对、不计算综合分数。', '',
             '差值 = 辅助驾驶均值 − AI 均值；相对变化 = 差值 / AI 均值 × 100%。AI 均值为零时相对变化留空。',
             '原指标为百分比时，直接差值单位是百分点。n=1 的标准差留空；缺组不计算差值。', '',
             '图中淡色点为各次运行结果，实心点为运行间均值，误差棒为样本标准差，不是置信区间。',
             'P95 指标对比的是各圈 P95 的运行间均值，不是合并所有遥测后的 P95。', '',
             '距离降低仅说明更接近给定参考采样点，不能直接证明安全提高。轮胎指标按日志原尺度解释。',
             'heading/progress 坐标对应未核实，因此不纳入对比。所有结果为描述性，不宣称显著性或独立重复已验证。', '',
             '## 数据提示', '']
    lines.extend(['- '+w for w in warnings] or ['无完整性提示。'])
    lines += ['', '## 主要指标', '', '|AIpush|指标|AI n|辅助 n|AI 均值|辅助均值|差值|变化 %|',
              '|---|---|---:|---:|---:|---:|---:|---:|']
    def fmt(value):
        return f'{value:.4g}' if np.isfinite(value) else 'NA'
    for row in differences[differences.metric.isin(PRIMARY)].itertuples():
        lines.append(f'|{row.AIpush}|{row.title} ({row.unit})|{row.ai_n}|{row.assisted_n}|'
                     f'{fmt(row.ai_mean)}|{fmt(row.assisted_mean)}|{fmt(row.delta_assisted_minus_ai)}|{fmt(row.relative_change_pct)}|')
    lines += ['', '每次运行覆盖同名输出。若本次未生成图，请勿使用输出目录中较早运行留下的图。']
    (output/'comparison_report.md').write_text('\n'.join(lines), encoding='utf-8')
    print(f'Compared {len(df)} runs -> {output.resolve()}')
    for warning in warnings:
        print('[WARN]', warning)


if __name__ == '__main__':
    main()
