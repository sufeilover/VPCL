"""Whole-track Case Study 2 pipeline. Overwrite derived outputs, never raw files."""
import argparse
import json
from pathlib import Path
import re
import sys
import pandas as pd
import Data_pre_process as dp
import main4only_batch_analyze_driving_v2 as driving
import main4only_track_analyze_driving_v2 as tracking

ROOT = Path(__file__).resolve().parent


def discover(root, folders=None):
    directories = [root / name for name in folders] if folders else sorted(
        p for p in root.iterdir() if p.is_dir() and re.fullmatch(r'(80|85|90|95|100)-(ai|shared)', p.name))
    paths = []
    for folder in directories:
        if not folder.is_dir() or not re.fullmatch(r'(80|85|90|95|100)-(ai|shared)', folder.name):
            raise ValueError(f'Invalid/missing condition folder: {folder}')
        expected = 'ai' if folder.name.endswith('-ai') else 'shared'
        paths.extend(sorted(p for p in folder.glob('*original.xlsx')
                            if re.fullmatch(r'\d+-' + expected + r'original\.xlsx', p.name)))
    return paths


def run_one(source, spline, lap_index):
    run_id = re.match(r'\d+', source.name).group()
    output = source.parent / 'full_track_results'
    lap = output / f'{run_id}_lap.xlsx'
    audit = dp.process_file(source, lap, lap_index)
    metrics = driving.analyze_one_file(str(lap))
    metrics.to_excel(output / f'{run_id}_full_track_driving.xlsx', index=False, sheet_name='metrics_all')
    tracking.DRIVE_XLSX = str(lap)
    tracking.SPLINE_CSV = str(spline)
    tracking.OUT_XLSX = str(output / f'{run_id}_full_track_tracking.xlsx')
    tracking.main()
    track = pd.read_excel(tracking.OUT_XLSX)
    if metrics['label'].tolist() != ['GLOBAL'] or track['label'].tolist() != ['GLOBAL']:
        raise AssertionError('Expected exactly one GLOBAL result per analysis.')
    record = dict(source=str(source.resolve()), folder=source.parent.name, run_id=run_id,
                  AIpush=int(source.parent.name.split('-')[0]),
                  condition='AI-only' if source.parent.name.endswith('-ai') else 'optimizer-assisted')
    record.update({f'lap_{k}': v for k, v in audit.items()})
    record.update({f'driving_{k}': v for k, v in metrics.iloc[0].items() if k not in ('file', 'label')})
    record.update({f'tracking_{k}': v for k, v in track.iloc[0].items() if k not in ('file', 'label')})
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--folders', nargs='+', help='For example: 80-ai 80-shared')
    parser.add_argument('--lap-index', type=int, default=1)
    parser.add_argument('--limit', type=int, help='Process only the first N discovered raw files')
    parser.add_argument('--list-only', action='store_true', help='List files without reading telemetry')
    args = parser.parse_args()
    if args.lap_index < 0 or (args.limit is not None and args.limit < 1):
        parser.error('lap-index must be >=0 and limit must be >=1')
    root = args.root.resolve()
    paths = discover(root, args.folders)
    if args.limit is not None:
        paths = paths[:args.limit]
    if not paths:
        raise FileNotFoundError('No matching original recordings found.')
    if args.list_only:
        for p in paths:
            print(p)
        print(f'Total: {len(paths)} original recordings')
        return
    spline = root / 'spline_points.csv'
    if not spline.is_file():
        raise FileNotFoundError(spline)
    records, status = [], []
    output = root / 'full_track_results'
    output.mkdir(exist_ok=True)
    for i, source in enumerate(paths, 1):
        print(f'[{i}/{len(paths)}] {source.parent.name}/{source.name}', flush=True)
        try:
            records.append(run_one(source, spline, args.lap_index))
            status.append(dict(source=str(source), status='ok', error=''))
        except Exception as exc:
            status.append(dict(source=str(source), status='failed', error=f'{type(exc).__name__}: {exc}'))
            print(f'[ERROR] {exc}', flush=True)
        pd.DataFrame(records).to_csv(output / 'run_summary.csv', index=False, encoding='utf-8-sig')
        pd.DataFrame(status).to_csv(output / 'processing_status.csv', index=False, encoding='utf-8-sig')
    meta = dict(selected_files=len(paths), successful_files=len(records),
                failed_files=sum(s['status'] != 'ok' for s in status), lap_index=args.lap_index,
                scope='GLOBAL only, no corners/straights/sectors',
                aggregation='sample-weighted within each run; no across-condition score',
                spline=str(spline), folders=args.folders, limit=args.limit,
                notes=['Historical shared folders are labeled optimizer-assisted.',
                       'Raw heading convention is unverified; heading error is not validated.',
                       'CTE fields retain historical names: nearest sampled-point distance.',
                       'No s_norm in spline means progress error is unavailable, not zero.',
                       'Summary covers ONLY files selected in this invocation.'])
    (output / 'analysis_protocol.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Completed: {len(records)}/{len(paths)}; summary: {output}')
    if meta['failed_files']:
        sys.exit(1)


if __name__ == '__main__':
    main()
