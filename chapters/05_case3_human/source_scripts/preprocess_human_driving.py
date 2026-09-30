"""Preprocess human trials without outcome comparisons or automatic incident labels.

Requires numpy, pandas, openpyxl. Raw logs are never modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
WHEELS = ('FL', 'FR', 'RL', 'RR')
SIGNALS = ['speedKmh', 'gas', 'brake', 'steerAngle', 'heading', 'pitch', 'roll',
           'x', 'y', 'z', 'normalizedCarPosition', 'iCurrentTime_s', 'iLastTime_s']
SIGNALS += [f'{base}_{w}' for base in ('wheelSlip', 'SlipAngle', 'NdSlip') for w in WHEELS]
DISTANCES = ['reference_point_distance_m', 'reference_lateral_abs_m', 'reference_lateral_signed_m']


def load_reference(path, closed=True):
    path = Path(path).resolve()
    raw = path.read_bytes()
    import io
    ref = pd.read_csv(io.BytesIO(raw))
    if not {'x', 'z'}.issubset(ref):
        raise ValueError('Reference CSV must contain ordered x,z coordinates in the same metre-based AC frame.')
    points = ref[['x', 'z']].apply(pd.to_numeric, errors='coerce').to_numpy(float)
    if len(points)<2 or not np.isfinite(points).all():
        raise ValueError('Reference requires at least two finite points; invalid points are not removed silently.')
    starts = points if closed else points[:-1]
    ends = np.roll(points,-1,axis=0) if closed else points[1:]
    vectors = ends-starts
    squared = np.sum(vectors*vectors,axis=1)
    valid = squared>1e-16
    if not valid.any():
        raise ValueError('Reference contains no nonzero-length segments.')
    return dict(points=points,starts=starts[valid],vectors=vectors[valid],squared=squared[valid],
                segment_ids=np.flatnonzero(valid),metadata=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),
                closed=closed,points=len(points),segments=int(valid.sum()),zero_length_segments=int((~valid).sum()),
                method='global nearest XZ polyline segment; clamped projection; all segments searched',
                sign='sign(dx_segment*dz_error - dz_segment*dx_error); reference file direction',
                alignment='user-specified reference; no translation/rotation or automatic track verification'))


def add_reference_distances(df, ref, chunk=128):
    if not {'x','z'}.issubset(df):
        raise ValueError('Telemetry x,z are required for reference distances.')
    car=df[['x','z']].apply(pd.to_numeric,errors='coerce').to_numpy(float)
    valid=np.flatnonzero(np.isfinite(car).all(axis=1))
    for col in DISTANCES+['reference_segment_id','reference_segment_fraction']:
        df[col]=np.nan
    starts,vectors,squared=ref['starts'],ref['vectors'],ref['squared']
    for offset in range(0,len(valid),chunk):
        ids=valid[offset:offset+chunk]
        p=car[ids]
        dx=p[:,None,0]-starts[None,:,0]
        dz=p[:,None,1]-starts[None,:,1]
        fraction=np.clip((dx*vectors[None,:,0]+dz*vectors[None,:,1])/squared,0,1)
        ex=dx-fraction*vectors[None,:,0]
        ez=dz-fraction*vectors[None,:,1]
        d2=ex*ex+ez*ez
        nearest=np.argmin(d2,axis=1)
        row=np.arange(len(ids))
        distance=np.sqrt(d2[row,nearest])
        cross=vectors[nearest,0]*ez[row,nearest]-vectors[nearest,1]*ex[row,nearest]
        sign=np.sign(cross)
        signed=distance*sign
        # At an endpoint a collinear displacement can have nonzero distance and undefined side.
        signed[(np.abs(cross)<1e-12)&(distance>1e-10)]=np.nan
        df.loc[ids,'reference_lateral_abs_m']=distance
        df.loc[ids,'reference_lateral_signed_m']=signed
        df.loc[ids,'reference_segment_id']=ref['segment_ids'][nearest]
        df.loc[ids,'reference_segment_fraction']=fraction[row,nearest]
        points=ref['points']
        point_d2=(p[:,None,0]-points[None,:,0])**2+(p[:,None,1]-points[None,:,1])**2
        df.loc[ids,'reference_point_distance_m']=np.sqrt(point_d2.min(axis=1))
    return df


def discover(root):
    rows = []
    for participant in range(1, 6):
        for suffix, condition in [('human', 'human-only'), ('ass', 'optimizer-assisted')]:
            folder = root / f'{participant}-{suffix}'
            files = sorted(p for p in folder.glob('telemetry_*.xlsx')
                           if re.fullmatch(r'telemetry_\d{8}_\d{6}\.xlsx', p.name))
            for run, path in enumerate(files, 1):
                rows.append(dict(participant=participant, condition=condition,
                                 run_order_in_condition=run, run_id=f'{folder.name}_{path.stem}',
                                 source=str(path.resolve())))
            if len(files) != 3:
                raise ValueError(f'{folder}: expected 3 telemetry files, found {len(files)}')
    return rows


def prepare(raw, gap_s=0.25):
    required = {'t_sec', 'completedLaps', 'normalizedCarPosition', 'speedKmh'}
    if required - set(raw):
        raise ValueError(f'Missing required columns: {sorted(required-set(raw))}')
    if len(raw) < 2:
        raise ValueError('Fewer than two samples.')
    df = raw.copy().reset_index(drop=True)
    df.insert(0, 'source_row_0based', np.arange(len(df)))
    invalid = {}
    for col in list(dict.fromkeys(['t_sec', 'completedLaps', 'packetId', 'udp_pid_used'] + SIGNALS)):
        if col in df:
            values = pd.to_numeric(df[col], errors='coerce').replace([np.inf, -np.inf], np.nan)
            invalid[col] = int(values.isna().sum())
            df[col] = values
    t = df.t_sec.to_numpy(float)
    if not np.isfinite(t).all() or np.any(np.diff(t) < 0):
        raise ValueError('Missing/decreasing t_sec: split/reset the recording explicitly; not silently sorted.')
    if t[-1] <= t[0]:
        raise ValueError('Recording has zero elapsed time.')
    laps = df.completedLaps.to_numpy(float)
    if not np.isfinite(laps).all() or np.any(laps != np.floor(laps)):
        raise ValueError('completedLaps must contain finite integers.')
    # Reset/jump boundaries are preserved and explicitly audited.
    df['lap_segment_id'] = np.r_[0, np.cumsum(np.diff(laps) != 0)]
    dt = np.r_[np.diff(t), np.nan]
    df['next_dt_s'] = dt
    df['valid_next_interval'] = np.isfinite(dt) & (dt > 0) & (dt <= gap_s)
    df['duplicate_timestamp'] = df.t_sec.duplicated(keep=False)
    if 'packetId' in df:
        df['repeated_packet'] = df.packetId.eq(df.packetId.shift())
    if 'packetId' in df and 'udp_pid_used' in df:
        df['packet_id_minus_udp_id'] = df.packetId - df.udp_pid_used
    for base, output in [('SlipAngle', 'max_abs_slip_angle'), ('wheelSlip', 'max_abs_wheel_slip'),
                         ('NdSlip', 'max_abs_nd_slip')]:
        cols = [f'{base}_{w}' for w in WHEELS]
        if all(c in df for c in cols):
            # Require all wheels for comparable four-wheel extrema.
            df[output] = df[cols].abs().max(axis=1).where(df[cols].notna().all(axis=1))
    a = np.full(len(df), np.nan)
    dv = np.diff(df.speedKmh.to_numpy(float) / 3.6)
    ok = df.valid_next_interval.to_numpy()[:-1] & np.isfinite(dv)
    a[1:][ok] = dv[ok] / dt[:-1][ok]
    df['longitudinal_accel_mps2'] = a
    qc = dict(rows=len(df), elapsed_s=float(t[-1]-t[0]),
              duplicate_timestamp_rows=int(df.duplicate_timestamp.sum()),
              gap_count=int(np.sum(dt > gap_s)), max_gap_s=float(np.nanmax(dt)),
              lap_counter_resets=int(np.sum(np.diff(laps)<0)),
              lap_counter_jumps=int(np.sum(np.diff(laps)>1)),
              valid_interval_duration_s=float(np.sum(dt[df.valid_next_interval])),
              invalid_by_column=invalid,
              missing_optional_columns=[c for c in SIGNALS if c not in df])
    return df, qc


def lap_inventory(df, gap_s=0.25):
    rows = []
    for seg, part in df.groupby('lap_segment_id', sort=False):
        first, last = int(part.index[0]), int(part.index[-1])
        lap = int(part.completedLaps.iloc[0])
        entry = first > 0 and df.completedLaps.iloc[first-1] == lap-1
        exit_ = last+1 < len(df) and df.completedLaps.iloc[last+1] == lap+1
        official = df.iLastTime_s.iloc[last+1] if exit_ and 'iLastTime_s' in df else np.nan
        if not np.isfinite(official) or official <= 0:
            official = np.nan
        rows.append(dict(lap_segment_id=int(seg), completedLaps=lap, rows=len(part),
                         start_t_sec=float(part.t_sec.iloc[0]), end_t_sec=float(part.t_sec.iloc[-1]),
                         entry_observed=bool(entry), exit_observed=bool(exit_),
                         counter_complete=bool(entry and exit_), official_lap_time_s=official,
                         gap_count=int((part.next_dt_s.iloc[:-1] > gap_s).sum())))
    return pd.DataFrame(rows)


def summary(df):
    out = {}
    channels = [c for c in SIGNALS + ['max_abs_slip_angle', 'max_abs_wheel_slip',
                'max_abs_nd_slip', 'longitudinal_accel_mps2'] + DISTANCES if c in df]
    weights = df.next_dt_s.where(df.valid_next_interval, 0).fillna(0)
    for col in channels:
        v = df[col].to_numpy(float)
        finite = np.isfinite(v)
        vals = v[finite]
        out[col+'_n_valid'] = len(vals)
        if len(vals):
            for label, value in [('mean', vals.mean()), ('std', vals.std(ddof=1) if len(vals)>1 else np.nan),
                                 ('median', np.median(vals)), ('p95', np.percentile(vals,95)),
                                 ('min', vals.min()), ('max', vals.max())]:
                out[col+'_'+label] = float(value)
            w = weights.to_numpy()[finite]
            out[col+'_time_mean'] = float(np.average(vals, weights=w)) if w.sum()>0 else np.nan
            if col in DISTANCES:
                out[col+'_rmse']=float(np.sqrt(np.mean(vals*vals)))
                out[col+'_time_rmse']=float(np.sqrt(np.average(vals*vals,weights=w))) if w.sum()>0 else np.nan
    if 'steerAngle' in df:
        diff = np.abs(np.diff(df.steerAngle.to_numpy(float)))
        good = df.valid_next_interval.to_numpy()[:-1] & np.isfinite(diff)
        out['steering_total_variation_logged_scale'] = float(diff[good].sum())
        out['steering_valid_interval_s'] = float(df.next_dt_s.to_numpy()[:-1][good].sum())
    # Euler-angle means are descriptive native-channel statistics, not tracking errors.
    return out


def resample(df, hz=20, max_age_s=0.1):
    unique = df.drop_duplicates('t_sec', keep='last')
    times = unique.t_sec.to_numpy(float)
    grid = times[0] + np.arange(int(np.floor((times[-1]-times[0])*hz))+1)/hz
    idx = np.searchsorted(times, grid, side='right')-1
    age = grid-times[idx]
    out = unique.iloc[idx].reset_index(drop=True).copy()
    out['source_t_sec'] = out.t_sec
    out['t_sec'] = grid
    out['source_age_s'] = age
    out['resample_valid'] = age <= max_age_s
    telemetry = [c for c in out if c not in ('t_sec', 'source_t_sec', 'source_age_s', 'resample_valid')]
    for col in telemetry:
        if pd.api.types.is_bool_dtype(out[col]):
            out[col] = out[col].astype('boolean')
    out.loc[~out.resample_valid, telemetry] = np.nan
    # These source-interval quantities do not describe the resampled grid.
    out = out.drop(columns=['next_dt_s', 'valid_next_interval', 'duplicate_timestamp'], errors='ignore')
    return out


def spatial_profile(df, bins=1000):
    pos = df.normalizedCarPosition
    valid = pos.notna() & pos.between(0, 1)
    part = df.loc[valid].copy()
    part['position_bin'] = np.minimum(np.floor(pos[valid]*bins).astype(int), bins-1)
    cols = [c for c in ['speedKmh', 'gas', 'brake', 'steerAngle', 'max_abs_slip_angle',
                       'max_abs_wheel_slip', 'x', 'y', 'z'] + DISTANCES if c in part]
    group = part.groupby(['lap_segment_id', 'position_bin'], sort=True)
    out = group[cols].median().add_suffix('_median')
    out['n_source_samples'] = group.size()
    out['first_t_sec'] = group.t_sec.min()
    out['last_t_sec'] = group.t_sec.max()
    # No filling unvisited bins; a reversing/stopped car may occupy a bin repeatedly.
    return out.reset_index()


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--list-only', action='store_true')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--gap-s', type=float, default=0.25)
    parser.add_argument('--hz', type=float, default=20)
    parser.add_argument('--max-age-s', type=float, default=0.1)
    parser.add_argument('--bins', type=int, default=1000)

    parser.add_argument(
        '--reference',
        type=Path,
        default=ROOT.parent / 'case study2_AI' / 'spline_points.csv',
        help='Reference trajectory CSV',
    )

    parser.add_argument(
        '--open-reference',
        action='store_true',
        help='Do not close the reference trajectory',
    )

    parser.add_argument(
        '--reuse-cleaned',
        type=Path,
        default=ROOT / 'human_preprocessed',
        help='Reuse existing cleaned data',
    )

    args = parser.parse_args()

    print(f'[CONFIG] Reference: {args.reference}', flush=True)
    print(f'[CONFIG] Reuse cleaned: {args.reuse_cleaned}', flush=True)
    if min(args.gap_s,args.hz,args.max_age_s,args.bins)<=0 or (args.limit is not None and args.limit<1):
        parser.error('Numeric settings and limit must be positive.')
    manifest = discover(args.root.resolve())
    if args.limit:
        manifest = manifest[:args.limit]
    if args.list_only:
        print(pd.DataFrame(manifest).to_string(index=False))
        return
    reference=load_reference(args.reference,not args.open_reference) if args.reference else None
    if reference is None:
        print('[INFO] No --reference supplied; reference distances will not be computed.',flush=True)
    output = args.output or args.root/'human_preprocessed'
    output = output.resolve()
    if output == args.root.resolve() or output in [Path(r['source']).parent for r in manifest]:
        raise ValueError('Use a separate output directory, not a raw-data folder.')
    output.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(manifest).to_csv(output/'manifest.csv', index=False, encoding='utf-8-sig')
    records, statuses, inventories = [], [], []
    protocol = dict(status='running', selected_files=len(manifest), settings=vars(args).copy(),
                    selection='all recorded rows; no warmup/low-speed/incident trimming',
                    resampling='previous sample, bounded age; no interpolation across gaps',
                    controls='logged vehicle commands; not independent raw human input',
                    units='native log scale except time(s), speed(km/h), acceleration(m/s^2)',
                    incidents='not automatically labeled; no safety threshold assumed',
                    trial_order='within-condition filename order; not paired trial identity')
    protocol['reference']=reference['metadata'] if reference else None
    def write_protocol():
        (output/'preprocessing_protocol.json').write_text(json.dumps(protocol,default=str,ensure_ascii=False,indent=2),encoding='utf-8')
    write_protocol()
    for i, meta in enumerate(manifest,1):
        print(f'[{i}/{len(manifest)}] {meta["run_id"]}',flush=True)
        try:
            path=Path(meta['source'])
            if args.reuse_cleaned:
                prior=args.reuse_cleaned/meta['run_id']
                qc=json.loads((prior/'qc.json').read_text(encoding='utf-8'))
                original=pd.read_csv(prior/'cleaned.csv.gz')
                # Rebuild derived columns using original telemetry, retaining all source rows.
                original=original.drop(columns=['source_row_0based']+DISTANCES+['reference_segment_id','reference_segment_fraction'],errors='ignore')
                df,new_qc=prepare(original,args.gap_s)
                source_hash=qc['source_sha256']
                qc=new_qc
                qc['source_hash_provenance']='copied from prior qc; raw Excel not reread'
            else:
                digest=hashlib.sha256()
                with path.open('rb') as stream:
                    for block in iter(lambda:stream.read(1024*1024),b''):
                        digest.update(block)
                source_hash=digest.hexdigest()
                df,qc=prepare(pd.read_excel(path),args.gap_s)
            if reference:
                df=add_reference_distances(df,reference)
                qc['reference']=reference['metadata']
                qc['reference_valid_position_rows']=int(df.reference_lateral_abs_m.notna().sum())
                qc['reference_side_undefined_rows']=int((df.reference_lateral_abs_m.notna() & df.reference_lateral_signed_m.isna()).sum())
            lap=lap_inventory(df,args.gap_s)
            aligned=resample(df,args.hz,args.max_age_s)
            run_dir=output/meta['run_id']
            run_dir.mkdir(exist_ok=True)
            df.to_csv(run_dir/'cleaned.csv.gz',index=False,compression='gzip')
            aligned.to_csv(run_dir/'resampled.csv.gz',index=False,compression='gzip')
            spatial_profile(df,args.bins).to_csv(run_dir/'spatial_profile.csv',index=False)
            for key in ['participant','condition','run_order_in_condition','run_id']:
                lap[key]=meta[key]
            lap.to_csv(run_dir/'lap_inventory.csv',index=False)
            qc.update(source_sha256=source_hash,resampled_rows=len(aligned),
                      resampled_invalid_rows=int((~aligned.resample_valid).sum()))
            (run_dir/'qc.json').write_text(json.dumps(qc,ensure_ascii=False,indent=2),encoding='utf-8')
            records.append(dict(**meta,source_sha256=source_hash,rows=len(df),elapsed_s=qc['elapsed_s'],
                                complete_counter_segments=int(lap.counter_complete.sum()),
                                gap_count=qc['gap_count'],**summary(df)))
            inventories.append(lap)
            statuses.append(dict(**meta,status='ok',error=''))
        except Exception as exc:
            statuses.append(dict(**meta,status='failed',error=f'{type(exc).__name__}: {exc}'))
            print(f'[ERROR] {exc}',flush=True)
        pd.DataFrame(records).to_csv(output/'run_summary.csv',index=False,encoding='utf-8-sig')
        pd.DataFrame(statuses).to_csv(output/'processing_status.csv',index=False,encoding='utf-8-sig')
        if inventories:
            pd.concat(inventories,ignore_index=True).to_csv(output/'lap_inventory_all.csv',index=False,encoding='utf-8-sig')
    protocol.update(status='complete' if len(records)==len(manifest) else 'completed_with_errors',
                    successful_files=len(records),failed_files=len(manifest)-len(records))
    write_protocol()
    print(f'Finished {len(records)}/{len(manifest)} -> {output}')
    if len(records)!=len(manifest):
        sys.exit(1)


if __name__=='__main__':
    main()
