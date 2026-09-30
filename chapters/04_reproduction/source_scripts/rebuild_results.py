"""Rebuild Section IV results after the three explicitly selected exclusions.
Recomputes all four configurations at 333 Hz. No raw writes.
Run with the bundled Python runtime. Outputs are overwritten reproducibly.
"""
from pathlib import Path
import hashlib, json
import numpy as np
import pandas as pd
import analysis_engine as a

BASE=Path(__file__).resolve().parent
OLD=BASE.parent/'three_layer_results'
ROOT=Path('E:/IEEE-TVT/new-similar')
OUT=BASE/'results'
OUT.mkdir(parents=True,exist_ok=True)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    a.selftests()
    a.SOURCES.clear()
    # Validate every input before rebuilding: never mix 3 ms and 333 Hz results.
    for folder in a.CONFIGS:
        p=ROOT/folder/'similarity_500_results/similarity_analysis_manifest.json'
        m=json.loads(p.read_text(encoding='utf-8-sig'))
        if m['horizon_steps']!=500 or m['model_step_seconds']!=a.MODEL_STEP_SECONDS:
            raise ValueError(f'Rerun the 333 Hz comparator first: {p}')
    parts=[]; audits=[]; corrs=[]; checks=[]
    for folder,config in a.CONFIGS.items():
        current,qc,corr=a.load_config(ROOT,folder,OUT)
        parts.append(current);audits.extend(qc);corrs.extend(corr)
        # This configuration intentionally excludes three windows downstream.
        if config=='Silverstone + Supra':continue
        p=ROOT/folder/'similarity_500_results/horizon_summary_by_setting.csv'
        a.track_source(p)
        ref=pd.read_csv(p)
        ref=ref[ref.source_setting.astype(str)=='all'].sort_values('horizon_step')
        cur=current[current.scope=='pooled'].sort_values('step')
        for stat in ['mean','p95']:
            for metric in a.METRICS:
                col=stat+'_'+metric
                err=float(np.max(np.abs(ref[col].to_numpy()-cur[col].to_numpy())))
                assert err<1e-8,(config,col,err)
                checks.append(dict(config=config,column=col,max_abs_difference=err))
    curves=pd.concat(parts,ignore_index=True)
    audit=pd.DataFrame(audits)
    correlations=pd.DataFrame(corrs)
    assert len(curves)==14000 and not curves.duplicated(['config','scope','step']).any()
    a.save(curves,OUT/'raw_curves.csv');a.save(audit,OUT/'data_audit.csv')
    a.save(correlations,OUT/'within_horizon_correlations.csv')
    a.save(pd.DataFrame(checks),OUT/'exclusion_reconciliation.csv')
    result=a.make_analyses(curves,OUT)
    a.figures(curves,result,OUT)
    panels=[]
    for c,g in result['normalized_curves'].query("scope=='pooled' and statistic=='p95'").groupby('config',sort=False):
        panels.append(dict(title=c,xlabel='Prediction horizon (steps)',ylabel='Utility: B - lambda L',series=[(f'Lambda = {lam:g}',g.step,g.benefit-lam*g.grouped) for lam in [.5,1,2]]))
    a.panel_figure(panels,'Grouped P95 benefit-minus-loss utility',OUT/'04_grouped_utility_p95.png')
    pool=curves[curves.scope=='pooled']
    a.save(pool[pool.step.isin([125,250,500])],OUT/'paper_representative_values.csv')
    scans=result['lambda_scan']
    selected=scans[(scans.scope=='pooled')&(scans.statistic=='p95')&scans.loss.isin(['grouped','worst'])&scans.lambda_value.isin([.5,1,2])]
    a.save(selected,OUT/'paper_preference_values.csv')
    manifest=dict(model_step_seconds=a.MODEL_STEP_SECONDS,representative_horizons=[125,250,500],
        cohort='all four configurations recomputed; fixed three-window exclusion retained',
        excluded=dict(folder='ks_silverstone1967+ks_toyota_supra_mkiv_drift',setting=100,rollouts=[172,173,174]),
        valid_rollouts=int(audit.valid_rollouts.sum()),horizon_rows=int(audit.horizon_rows.sum()),
        sources=a.SOURCES,independent_summary_reconciliation=checks,
        engine_sha256=sha(BASE/'analysis_engine.py'),runner_sha256=sha(__file__),reuse='none',
        outputs={p.name:sha(p) for p in OUT.glob('*.csv')})
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(selected.to_string(index=False),flush=True)
    print('COMPLETE',flush=True)

if __name__=='__main__':main()
