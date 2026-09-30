"""Extract supplied resource geometry; draw compact, vector publication figures."""
from pathlib import Path
import json, hashlib, csv, argparse
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
parser=argparse.ArgumentParser(description='Build Fig. 1, Fig. 2 and Table II from resource and processed files.')
parser.add_argument('--model-root',type=Path,default=Path('E:/IEEE-TVT/new-similar/code/model'))
parser.add_argument('--metrics',type=Path,default=Path(__file__).resolve().parent/'ai_characteristics_analysis/selected_condition_metrics.csv')
parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent)
args=parser.parse_args()
ROOT=args.output_dir;SOURCE=args.model_root
for folder in ['figures','evidence','tables']:(ROOT/folder).mkdir(parents=True,exist_ok=True)
def line(c,pts,color,width=.6,dash=()):
    c.setStrokeColor(HexColor(color));c.setLineWidth(width);c.setDash(dash)
    p=c.beginPath();p.moveTo(*pts[0])
    for pt in pts[1:]:p.lineTo(*pt)
    c.drawPath(p);c.setDash()
def text(c,x,y,t,size=8):
    c.setFillColor(HexColor('#222222'));c.setFont('Times-Roman',size);c.drawString(x,y,t)
tracks=[];evidence={}
for name in ['ks_barcelona','ks_silverstone1967']:
    b=SOURCE/name/'spline.bin';f=SOURCE/name/'spline.cache'
    a=np.fromfile(f,dtype='<f4').reshape(-1,15).astype(float)
    raw=np.fromfile(b,dtype='<f4').reshape(-1,5)
    assert len(a)==len(raw) and np.isfinite(a).all()
    # Cache ray-casting adjusts elevation; check horizontal point correspondence.
    assert np.max(np.abs(a[:,[0,2]]-raw[:,[0,2]]))<.01
    center=a[:,9:12]; ds=np.linalg.norm(np.roll(center,-1,axis=0)-center,axis=1)
    widths=np.linalg.norm(a[:,3:6]-a[:,6:9],axis=1)
    evidence[name]={'points':len(a),'center_closed_3d_length_m':float(ds.sum()),'elevation_range_m':float(np.ptp(center[:,1])),
      'boundary_width_median_m':float(np.median(widths)),'center_closure_gap_m':float(ds[-1]),'sha256_cache':hashlib.sha256(f.read_bytes()).hexdigest()}
    evidence[name]['reference_closed_3d_length_m']=float(np.linalg.norm(np.roll(a[:,:3],-1,axis=0)-a[:,:3],axis=1).sum())
    tracks.append(a)
c=canvas.Canvas(str(ROOT/'figures/track_layouts.pdf'),pagesize=(252,148))
scale=min(112/max(np.ptp(a[:,0]),np.ptp(a[:,2])) for a in tracks)
for i,(name,a) in enumerate(zip(evidence,tracks)):
    allxz=a[:,[0,2]];mid=(allxz.min(0)+allxz.max(0))/2
    def xy(p):return (p-mid)*scale+np.array([63+i*126,78])
    # The cache 'best' reference line, not the boundary midpoint or an AI run.
    pts=xy(allxz);line(c,np.vstack([pts,pts[0]]),'#0072B2',1.0)
    text(c,i*126+10,137,'(a) Barcelona' if i==0 else '(b) Silverstone 1967',9)
    text(c,i*126+10,9,f"Reference: {evidence[name]['reference_closed_3d_length_m']/1000:.3f} km",7.5)
line(c,[(10,24),(10+500*scale,24)],'#222222',1.2);text(c,10,29,'500 m',7)
c.save()
rows=list(csv.DictReader(args.metrics.open(encoding='utf-8-sig')))
configs=['Barcelona/AE86','Barcelona/Supra','Silverstone/AE86','Silverstone/Supra']
colors=['#0072B2','#D55E00','#009E73','#8B51A5'];dashes=[(),(3,2),(1,1),(4,1,1,1)]
# Table II is regenerated from the same run-level summary source, not typed numbers.
table_rows=[]
for conf in configs:
    cells=[]
    for setting in [80,85,90,95,100]:
        matches=[r for r in rows if r['configuration']==conf and r['metric']=='lap_s' and int(r['setting'])==setting]
        if len(matches)!=1 or int(matches[0]['n'])!=5:raise ValueError(f'Expected one n=5 summary: {conf}, {setting}')
        r=matches[0];cells.append(f"${float(r['mean']):.3f}\\pm{float(r['sample_sd']):.3f}$")
    table_rows.append(conf+' & '+' & '.join(cells)+r' \\')
(ROOT/'tables/aipush_rows.tex').write_text('\n'.join(table_rows)+'\n',encoding='utf-8')
table_header=r'''\begin{table*}[!t]
\caption{Complete-circuit time under the five AIpush settings. Entries are mean $\pm$ sample standard deviation in seconds over five recorded runs per condition.}
\label{tab:iv_aipush}
\centering\footnotesize
\setlength{\tabcolsep}{5pt}
\begin{tabular}{lccccc}\toprule
Configuration & 80 & 85 & 90 & 95 & 100\\\midrule
'''
(ROOT/'tables/aipush_table.tex').write_text(table_header+'\n'.join(table_rows)+'\n'+r'\bottomrule\end{tabular}\end{table*}'+'\n',encoding='utf-8')
c=canvas.Canvas(str(ROOT/'figures/ai_conditions.pdf'),pagesize=(252,224))
for k,(metric,title,lo,hi,ticks) in enumerate([
 ('speed_mean_kmh','(a) Mean speed (km/h)',100,175,[100,125,150,175]),
 ('steer_abs_mean_native','(b) Mean absolute steering (native)',.01,.06,[.01,.03,.05])]):
    bottom=119 if k==0 else 31;top=bottom+60
    text(c,2,top+10,title,8)
    def xy(x,y):return (30+(x-80)/20*213,bottom+(y-lo)/(hi-lo)*60)
    for v in ticks:
        yy=xy(80,v)[1];line(c,[(30,yy),(243,yy)],'#DDDDDD',.4);text(c,1,yy-2,f'{v:g}',7)
    line(c,[(30,top),(30,bottom),(243,bottom)],'#333333')
    for x in [80,85,90,95,100]:text(c,xy(x,lo)[0]-5,bottom-10,str(x),7)
    for conf,color,dash in zip(configs,colors,dashes):
        rr=sorted([r for r in rows if r['configuration']==conf and r['metric']==metric],key=lambda r:int(r['setting']))
        line(c,[xy(float(r['setting']),float(r['mean'])) for r in rr],color,.85,dash)
        for r in rr:
            x=float(r['setting']);m=float(r['mean']);s=float(r['sample_sd']);xx,yy=xy(x,m)
            line(c,[xy(x,m-s),xy(x,m+s)],color,.7)
            c.setFillColor(HexColor(color));c.circle(xx,yy,1.5,stroke=0,fill=1)
text(c,119,7,'AIpush',8)
for i,(conf,col,dash) in enumerate(zip(configs,colors,dashes)):
    x=2+(i%2)*126;y=218-(i//2)*9
    line(c,[(x,y),(x+12,y)],col,.8,dash);text(c,x+15,y-2,conf,7)
c.save()
(ROOT/'evidence/configuration_geometry.json').write_text(json.dumps(evidence,indent=2))
print(json.dumps(evidence,indent=2))
