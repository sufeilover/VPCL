"""Read-only validation of request summaries; regenerate paper sensitivity figure/table."""
from pathlib import Path
import json, hashlib
import numpy as np
import pandas as pd
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
ROOT=Path(__file__).resolve().parent
SRC=Path('E:/IEEE-TVT/new-similar/case study1_drift_analysis')
for folder in ['figures','results','tmp']:(ROOT/folder).mkdir(exist_ok=True)
sweeps=[];checks=[];summary=[]
for push in [80,85,90,95,100]:
    folder=SRC/f'{push}+ston+86';files=list(folder.glob('drift_sensitivity_*.csv'))
    assert len(files)==1
    f=files[0];d=pd.read_csv(f)
    assert not d.duplicated(['request_id','horizon_frames']).any()
    assert (d.groupby('request_id').horizon_frames.apply(lambda s:tuple(sorted(s)))==(80,160,240)).all()
    assert d.car_model.eq('ks_toyota_ae86_drift').all()
    assert d.threshold_deg.eq(9.3).all() and d.selected_run_frames.eq(10).all() and d.out_run_frames.eq(10).all()
    runcols=[f'max_run_{w}' for w in ['FL','FR','RL','RR']]
    assert np.isfinite(d[runcols]).all().all()
    assert (d[runcols]>=0).all().all() and d[runcols].le(d.horizon_frames,axis=0).all().all()
    assert (d[runcols]%1==0).all().all()
    d['longest']=d[runcols].max(axis=1)
    for n in [5,10,20]:assert d[f'drift_flag_{n}'].eq((d.longest>=n).astype(int)).all()
    assert d.selected_drift_flag.eq(d.drift_flag_10).all()
    assert np.allclose(d.horizon_seconds,d.horizon_frames/333)
    for col in runcols+['out_flag']:
        a=d.pivot(index='request_id',columns='horizon_frames',values=col)
        assert (np.diff(a[[80,160,240]],axis=1)>=0).all()
    old=pd.read_csv(folder/'drift_threshold_sweep.csv')
    for h,g in d.groupby('horizon_frames'):
        for n in range(1,81):
            count=int((g.longest>=n).sum());oldrow=old[(old.horizon_frames==h)&(old.threshold_frames==n)].iloc[0]
            assert count==int(oldrow.trigger_count) and np.isclose(count/len(g),oldrow.trigger_rate)
            sweeps.append(dict(aipush=push,horizon=h,persistence=n,count=count,requests=len(g),rate_pct=count/len(g)*100))
        summary.append(dict(aipush=push,horizon=h,requests=len(g),n5=int((g.longest>=5).sum()),n10=int((g.longest>=10).sum()),n20=int((g.longest>=20).sum()),out10=int(g.out_flag.sum())))
    checks.append(dict(aipush=push,requests=int(d.request_id.nunique()),rows=len(d),file=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
s=pd.DataFrame(sweeps);t=pd.DataFrame(summary)
s.to_csv(ROOT/'results/persistence_sweep.csv',index=False);t.to_csv(ROOT/'results/trigger_summary.csv',index=False)
(ROOT/'results/audit.json').write_text(json.dumps(checks,indent=2))
print(t.to_string(index=False));print('total requests',sum(x['requests'] for x in checks))
for h,g in t.groupby('horizon'):
    print('horizon',h,'5->10 pooled counts',int(g.n5.sum()),int(g.n10.sum()),'reduction',100*(1-g.n10.sum()/g.n5.sum()))
lines=[r'\begin{table}[!tb]',r'\caption{DRIFT-triggered requests at $N=10$. Entries are count (percentage); $n$ is the number of requests per horizon.}',r'\label{tab:v_exp1_counts}',r'\centering\footnotesize',r'\setlength{\tabcolsep}{3pt}',r'\begin{tabular}{rrrrr}\toprule',r'AIpush & $n$ & $h=80$ & $h=160$ & $h=240$\\\midrule']
for push,g in t.groupby('aipush'):
    vals=[f"{int(r.n10)} ({100*r.n10/r.requests:.2f}\\%)" for r in g.itertuples()]
    lines.append(f'{push} & {int(g.requests.iloc[0]):,} & '+' & '.join(vals)+r' \\')
lines += [r'\bottomrule\end{tabular}',r'\end{table}']
(ROOT/'results/trigger_table.tex').write_text('\n'.join(lines)+'\n')
c=canvas.Canvas(str(ROOT/'figures/persistence_sensitivity.pdf'),pagesize=(516,161))
colors=['#0072B2','#D55E00','#009E73','#8B51A5','#222222'];dashes=[(),(3,2),(1,1),(4,1,1,1),(6,2)]
def text(x,y,v,size=8):
    c.setFillColor(HexColor('#222222'));c.setFont('Times-Roman',size);c.drawString(x,y,v)
def line(pts,color,width=.7,dash=()):
    c.setStrokeColor(HexColor(color));c.setLineWidth(width);c.setDash(dash);p=c.beginPath();p.moveTo(*pts[0])
    for pt in pts[1:]:p.lineTo(*pt)
    c.drawPath(p);c.setDash()
for k,h in enumerate([80,160,240]):
    left=29+k*172;bottom=42;w=133;height=91
    xy=lambda x,y:(left+(x-1)/79*w,bottom+y/16*height)
    text(left,143,f'({chr(97+k)}) {h} steps',9)
    for v in [0,4,8,12,16]:
        y=xy(1,v)[1];line([(left,y),(left+w,y)],'#DDDDDD',.35);text(left-17,y-2,str(v),7)
    for n in [5,10,20]:line([xy(n,0),xy(n,16)],'#BBBBBB',.5,(1,2))
    line([(left,bottom+height),(left,bottom),(left+w,bottom)],'#333333')
    for n in [1,20,40,60,80]:text(xy(n,0)[0]-3,bottom-10,str(n),7)
    for push,col,dash in zip([80,85,90,95,100],colors,dashes):
        g=s[(s.aipush==push)&(s.horizon==h)]
        pts=[]
        for row in g.itertuples():
            xx,yy=xy(row.persistence,row.rate_pct)
            if pts:pts.append((xx,pts[-1][1]))
            pts.append((xx,yy))
        line(pts,col,.9,dash)
    text(left+3,19,'Persistence requirement N (steps)',7)
text(4,153,'Triggered requests (%)',7)
for i,(push,col,dash) in enumerate(zip([80,85,90,95,100],colors,dashes)):
    x=43+i*93;line([(x,5),(x+13,5)],col,.9,dash);text(x+17,3,f'AIpush {push}',7)
c.save()
