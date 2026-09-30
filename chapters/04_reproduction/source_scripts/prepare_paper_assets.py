from pathlib import Path
import json,hashlib
import numpy as np
import pandas as pd
import analysis_engine as a
B=Path(__file__).resolve().parent
O=B/'results'
curves=pd.read_csv(O/'raw_curves.csv')
pool=curves[curves.scope=='pooled']
manifest=json.loads((O/'manifest.json').read_text(encoding='utf-8'))
if manifest.get('model_step_seconds') != a.MODEL_STEP_SECONDS:
    raise ValueError("Run rebuild_results.py with all four 333 Hz comparisons first.")
for source in manifest['sources']:
    if hashlib.sha256(Path(source['path']).read_bytes()).hexdigest() != source['sha256']:
        raise ValueError("Input changed since rebuild: " + source['path'])
checks=manifest['independent_summary_reconciliation']
for stat in ['mean','p95']:
    panels=[]
    for j,m in enumerate(a.METRICS):
        panels.append(dict(title=a.LABELS[j],xlabel='Prediction horizon (steps)',ylabel=a.UNITS[j],series=[(c,g.step,g[stat+'_'+m]) for c,g in pool.groupby('config',sort=False)]))
    panels.append(dict(title='Reference travel distance',xlabel='Prediction horizon (steps)',ylabel='m (mean)',series=[(c,g.step,g.distance_m) for c,g in pool.groupby('config',sort=False)]))
    a.panel_figure(panels,'Horizon-dependent '+stat.upper()+' errors and reference travel',O/f'paper_{stat}.png')
rows=[]
for r in pool[pool.step.isin([125,250,500])].itertuples():
    cells=[r.config.replace('Silverstone','Silverstone').replace(' + ','/'),str(r.step),f'{r.distance_m:.2f}']
    for m in a.METRICS:cells.append(f'{getattr(r,"mean_"+m):.3f}/{getattr(r,"p95_"+m):.3f}')
    rows.append(' & '.join(cells)+r' \\')
(B/'representative_rows.tex').write_text('\n'.join(rows)+'\n',encoding='utf-8')
s=pd.read_csv(O/'paper_preference_values.csv');rows=[]
for c in a.CONFIGS.values():
    cells=[c.replace(' + ','/')]
    for lam in [.5,1,2]:cells.append(str(int(s[(s.config==c)&(s.loss=='grouped')&(s.lambda_value==lam)].selected_step.iloc[0])))
    cells.append(str(int(s[(s.config==c)&(s.loss=='worst')&(s.lambda_value==1)].selected_step.iloc[0])))
    rows.append(' & '.join(cells)+r' \\')
(B/'preference_rows.tex').write_text('\n'.join(rows)+'\n',encoding='utf-8')
n=pd.read_csv(O/'normalization_sensitivity.csv')
f=n[(n.reference_cap==500)&(n.benefit_scale=='fixed500')&(n.statistic=='p95')&(n.loss=='grouped')&(n.lambda_value==1)]
print(pool[pool.step.isin([125,250,500])][['config','step','distance_m']+['p95_'+m for m in a.METRICS]].to_string(index=False))
print(f[['config','method','selected_step']].to_string(index=False))
print(pd.read_csv(O/'data_audit.csv').groupby('config').valid_rollouts.sum())
man=json.loads((O/'manifest.json').read_text())
man['independent_exclusion_reconciliation']=checks
man['asset_builder_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
man['engine_sha256']=hashlib.sha256((B/'analysis_engine.py').read_bytes()).hexdigest()
man['runner_sha256']=hashlib.sha256((B/'rebuild_results.py').read_bytes()).hexdigest()
man['outputs']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('*.csv')}
(O/'manifest.json').write_text(json.dumps(man,indent=2),encoding='utf-8')
