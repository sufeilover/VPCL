"""Read processed AI products only; source files/manuscript remain unchanged."""
from pathlib import Path
import hashlib,json
import numpy as np
import pandas as pd
SOURCE=Path('E:/IEEE-TVT/new-similar/ai_driving_characteristics')
OUT=Path(__file__).resolve().parent.parent/'ai_characteristics_analysis'
OUT.mkdir(exist_ok=True)
def read(n,**kw):return pd.read_csv(SOURCE/n,**kw)
def short(s):return ('Barcelona' if 'barcelona' in s else 'Silverstone')+('/AE86' if 'ae86' in s else '/Supra')
manifest=json.loads((SOURCE/'run_manifest.json').read_text())
qc=read('lap_qc.csv');s=read('condition_summary.csv');f=read('run_features_long.csv')
for d in [qc,s,f]:d['configuration']=d.config.map(short)
keys=['config','track','vehicle','setting','stage','channel','statistic','unit']
re=f.groupby(keys,dropna=False).value.agg(n='count',mean='mean',sample_sd='std').reset_index()
joined=s.merge(re,on=keys,suffixes=('','_recomputed'),validate='one_to_one')
audit={'status':manifest['status'],'success':manifest['success'],'failed':manifest['failed'],'conditions':int(qc.groupby(['config','setting']).ngroups),
       'feature_rows':len(f),'condition_rows':len(s),'summary_count_mismatches':int((joined.n!=joined.n_recomputed).sum()),
       'max_summary_mean_discrepancy':float((joined['mean']-joined.mean_recomputed).abs().max()),
       'max_summary_sd_discrepancy':float((joined.sample_sd-joined.sample_sd_recomputed).abs().max()),
       'local_script_matches':hashlib.sha256(Path(__file__).with_name('preprocess_ai_driving_characteristics.py').read_bytes()).hexdigest()==manifest['script_sha256']}
for c in ['dropped_invalid_time_or_progress','duplicate_timestamp_rows','time_gap_count','progress_regression_rows','position_jump_count','gas_outside_0_1_rows','brake_outside_0_1_rows']:
    audit[c]=int(qc[c].sum())
for c in ['endpoint_interpolation_correction_s','max_dt_s','max_position_jump_m','pid_difference_mean','pid_difference_max']:
    audit[c]={'min':float(qc[c].min()),'max':float(qc[c].max())}
selected={
'lap_s':('lap','lap','duration_s'),
'speed_mean_kmh':('raw','speedKmh','time_mean'),
'speed_p95_kmh':('raw','speedKmh','q95'),
'gas_mean':('raw','gas','time_mean'),'gas_max':('raw','gas','max'),
'brake_mean':('raw','brake','time_mean'),
'brake_active_fraction':('events','brake','gt_0.05__active_fraction'),
'steer_abs_mean_native':('raw','steerAngle','time_mean_abs'),
'slip_abs_mean_deg':('regularized','SlipAngle_four_wheel_abs_mean','time_mean'),
'slip_abs_p95_deg':('regularized','SlipAngle_four_wheel_abs_mean','q95'),
'rear_front_slip_signed':('regularized','SlipAngle_rear_minus_front','time_mean'),
'accel_rms':('regularized','speed_derivative_m_s2','time_rms'),
'path_length_m':('lap','lap','path_length_xz_m'),
'speed_distance_m':('lap','lap','speed_integrated_distance_m'),
'path_curvature_mean':('regularized','path_curvature_abs_m_inv','time_mean'),
'yaw_rate_abs_mean':('regularized','yaw_rate_rad_s','time_mean_abs'),
'position_vs_speed_rms':('regularized','position_vs_recorded_speed_m_s','time_rms')}
tables=[];run_tables=[]
for label,(stage,channel,stat) in selected.items():
    for inp,dest in [(s,tables),(f,run_tables)]:
        x=inp[(inp.stage==stage)&(inp.channel==channel)&(inp.statistic==stat)].copy();x['metric']=label;dest.append(x)
t=pd.concat(tables);r=pd.concat(run_tables)
t.to_csv(OUT/'selected_condition_metrics.csv',index=False,encoding='utf-8-sig')
r.to_csv(OUT/'selected_run_metrics.csv',index=False,encoding='utf-8-sig')
wide=t.pivot(index=['configuration','setting'],columns='metric',values='mean').reset_index()
wide.to_csv(OUT/'condition_overview.csv',index=False,encoding='utf-8-sig')
cv=t[t.metric=='lap_s'].copy();cv['cv_pct']=100*cv.sample_sd/cv['mean']
audit['lap_sd_range_s']=[float(cv.sample_sd.min()),float(cv.sample_sd.max())]
audit['lap_cv_percent_range']=[float(cv.cv_pct.min()),float(cv.cv_pct.max())]
audit['selected_metric_min_runs']=int(t.n.min())
coverage=s[s.statistic=='coverage_fraction'].groupby(['stage','channel'])['mean'].agg(['min','max']).reset_index()
coverage.to_csv(OUT/'channel_coverage.csv',index=False,encoding='utf-8-sig')
with (OUT/'audit.json').open('w',encoding='utf-8') as stream:json.dump(audit,stream,ensure_ascii=False,indent=2)
print(json.dumps(audit,indent=2));print(wide.to_string(index=False,float_format=lambda v:f'{v:.4f}'))
print('LOWEST COVERAGE');print(coverage.sort_values('min').head(12).to_string(index=False))
print('LAP SD');print(cv[['configuration','setting','mean','sample_sd','cv_pct']].to_string(index=False))
channels=['speedKmh','gas','brake','steerAngle','SlipAngle_four_wheel_abs_mean']
parts=[]
for chunk in pd.read_csv(SOURCE/'condition_profiles.csv.gz',chunksize=100000):
    parts.append(chunk[chunk.channel.isin(channels)])
p=pd.concat(parts,ignore_index=True);p['configuration']=p.config.map(short)
ps=p.groupby(['configuration','setting','channel']).agg(points=('n','size'),min_runs=('n','min'),sd_median=('sample_sd','median'),sd_p95=('sample_sd',lambda x:x.quantile(.95)),sd_max=('sample_sd','max')).reset_index()
ps.to_csv(OUT/'spatial_repeatability_summary.csv',index=False,encoding='utf-8-sig')
print('SPATIAL SPEED SD');print(ps[ps.channel=='speedKmh'].to_string(index=False))
print('SILVERSTONE AE86 RUNS');print(r[(r.configuration=='Silverstone/AE86')&(r.setting>=95)&(r.metric=='lap_s')][['setting','run','value']].to_string(index=False))
print('GAS MAX RANGES');print(r[r.metric=='gas_max'].groupby('setting').value.agg(['min','max']).to_string())
q=qc.groupby('configuration').agg(pid_max=('pid_difference_max','max'),runs_pid_over10=('pid_difference_max',lambda x:(x>10).sum()))
print('PID QC');print(q.to_string());q.to_csv(OUT/'pid_qc_summary.csv',encoding='utf-8-sig')
pair=p[(p.configuration=='Silverstone/AE86')&p.setting.isin([95,100])].pivot(index=['channel','track_progress'],columns='setting',values='mean')
pair['delta_100_minus_95']=pair[100]-pair[95];pair.to_csv(OUT/'silverstone_ae86_95_100_profiles.csv',encoding='utf-8-sig')
v=pair.loc['speedKmh','delta_100_minus_95']
print('SA speed spatial difference',{'fraction_negative':float((v<0).mean()),'min':float(v.min()),'min_progress':float(v.idxmin()),'median':float(v.median()),'max':float(v.max())})
