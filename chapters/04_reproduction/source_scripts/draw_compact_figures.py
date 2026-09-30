"""Size-native publication layouts; original numerical curves are unchanged.

Raw metrics: 3 x 2 at two-column width. Preference figures: 2 x 2 at
single-column width. Fonts are 7.5--8 pt at the intended printed size.
"""
import csv, json, math, hashlib
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'figures'
CONFIGS=['Barcelona + AE86','Barcelona + Supra','Silverstone + AE86','Silverstone + Supra']
LABELS=['Barcelona/AE86','Barcelona/Supra','Silverstone/AE86','Silverstone/Supra']
COLORS=['#0072B2','#D55E00','#009E73','#8B51A5']
DASHES=[(),(3,1.6),(1,1),(4,1,1,1)]
pdfmetrics.registerFont(TTFont('EmbeddedTimesItalic','C:/Windows/Fonts/timesi.ttf'))
def read(name):
    with (ROOT/'results'/name).open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
raw=read('raw_curves.csv');norm=read('normalized_curves.csv');intervals=read('exact_lambda_intervals.csv')
def label(c,x,y,s,size=7.5,bold=False,align='left'):
    c.setFillColor(HexColor('#222222'));c.setFont('Times-Bold' if bold else 'Times-Roman',size)
    {'left':c.drawString,'right':c.drawRightString,'center':c.drawCentredString}[align](x,y,s)
def stroke(c,pts,color,dash=()):
    c.setStrokeColor(HexColor(color));c.setLineWidth(.8);c.setDash(dash)
    p=c.beginPath();p.moveTo(*pts[0])
    for pt in pts[1:]:p.lineTo(*pt)
    c.drawPath(p);c.setDash()
def axes(c,x,y,w,h,title,yticks,xticks,yrange,xrange=(0,500),log=False,show_x=True):
    left=x+25;right=x+w-7;bottom=y+15;top=y+h-15
    label(c,x+2,y+h-7,title,8,True)
    def xy(xx,yy):
        frac=(math.log10(xx)-math.log10(xrange[0]))/(math.log10(xrange[1])-math.log10(xrange[0])) if log else (xx-xrange[0])/(xrange[1]-xrange[0])
        return left+frac*(right-left),bottom+(yy-yrange[0])/(yrange[1]-yrange[0])*(top-bottom)
    for v in yticks:
        _,yy=xy(xrange[0],v)
        stroke(c,[(left,yy),(right,yy)],'#DEE2E6')
        label(c,left-4,yy-2.4,f'{v:g}',align='right')
    stroke(c,[(left,top),(left,bottom),(right,bottom)],'#444444')
    for v in xticks:
        xx,_=xy(v,yrange[0]);stroke(c,[(xx,bottom),(xx,bottom-2)],'#444444')
        if show_x:label(c,xx,bottom-10,f'{v:g}',align='center')
    return xy
def legend(c,labels,width,y,cols,lambda_labels=False):
    for i,s in enumerate(labels):
        col=i%cols;row=i//cols;x=4+col*width/cols;yy=y-row*11
        stroke(c,[(x,yy+2.5),(x+13,yy+2.5)],COLORS[i],DASHES[i])
        if lambda_labels:
            c.setFillColor(HexColor('#222222'));c.setFont('EmbeddedTimesItalic',8);c.drawString(x+17,yy,'λ')
            label(c,x+22,yy,'= '+s)
        else:label(c,x+17,yy,s)
def common_xlabel(c,width,y,lam=False):
    if lam:
        c.setFillColor(HexColor('#222222'));c.setFont('EmbeddedTimesItalic',8);c.drawString(width/2-24,y,'λ')
        label(c,width/2-18,y,'(log scale)',8)
    else:label(c,width/2,y,'Prediction horizon (steps)',8,align='center')

def raw_figure():
    width,height=516,238;c=canvas.Canvas(str(OUT/'five_metrics_compact.pdf'),pagesize=(width,height))
    c.setTitle('Five-metric P95 errors and mean reference distance')
    specs=[('pos3d_rmse_m','Position RMSE (m)',[0,2,4],(0,4)),('pos3d_finalae_m','Terminal error (m)',[0,5,10],(0,10)),('speedKmh_rmse','Speed RMSE (km/h)',[0,10,20],(0,20)),('attitude_mean_mae_deg','Attitude MAE (deg)',[0,2.5,5],(0,5)),('SlipAngle_mean_rmse','Slip RMSE (deg)',[0,6,12],(0,12)),('distance_m','Mean reference travel (m)',[0,35,70],(0,70))]
    for i,(metric,title,yticks,yrange) in enumerate(specs):
        xy=axes(c,(i%3)*172,36+(1-i//3)*98,172,98,f'({chr(97+i)}) '+title,yticks,[0,250,500],yrange,show_x=True)
        for ci,config in enumerate(CONFIGS):
            rows=sorted([r for r in raw if r['config']==config and r['scope']=='pooled'],key=lambda r:int(r['step']))
            key=metric if metric=='distance_m' else 'p95_'+metric
            vals=[(int(r['step']),float(r[key])) for r in rows]
            assert all(yrange[0]<=v<=yrange[1] for _,v in vals)
            stroke(c,[xy(h,v) for h,v in vals],COLORS[ci],DASHES[ci])
    common_xlabel(c,width,23);legend(c,LABELS,width,6,4);c.showPage();c.save()

def lambda_figure():
    width,height=252,244;c=canvas.Canvas(str(OUT/'lambda_overview_compact.pdf'),pagesize=(width,height))
    c.setTitle('Preference sensitivity over lambda 0.1 to 10')
    validation=json.loads((OUT/'lambda_overview_validation.json').read_text(encoding='utf-8'))
    specs=[('step','Horizon (steps)',[0,250,500],(0,500)),('distance_m','Distance (m)',[0,35,70],(0,70)),('benefit','Benefit B',[0,.5,1],(0,1)),('loss','Loss Lg',[0,.5,1],(-.08,1.05))]
    for i,(key,title,yticks,yrange) in enumerate(specs):
        xy=axes(c,(i%2)*126,50+(1-i//2)*95,126,95,f'({chr(97+i)}) '+title,yticks,[.1,1,10],yrange,(.1,10),True,True)
        # Auxiliary labels identify representative preferences, not major log ticks.
        for lam in [.5,2]:
            xx,bottom=xy(lam,yrange[0])
            stroke(c,[(xx,bottom),(xx,bottom-1.3)],'#666666')
            label(c,xx,bottom-10,f'{lam:g}',size=6.5,align='center')
        for lam in [.5,1,2]:stroke(c,[xy(lam,yrange[0]),xy(lam,yrange[1])],'#BBBBBB',(1,2))
        for ci,config in enumerate(CONFIGS):
            rows={int(r['step']):r for r in norm if r['config']==config and r['scope']=='pooled' and r['statistic']=='p95'}
            dist={int(r['step']):float(r['distance_m']) for r in raw if r['config']==config and r['scope']=='pooled'}
            spans=sorted((max(.1,float(r['lambda_low'])),min(10.,float(r['lambda_high'])),int(r['step'])) for r in intervals if r['config']==config and r['statistic']=='p95' and r['loss']=='grouped' and min(10.,float(r['lambda_high']))>max(.1,float(r['lambda_low']))+1e-12)
            def value(h):return h if key=='step' else dist[h] if key=='distance_m' else float(rows[h]['grouped' if key=='loss' else 'benefit'])
            assert all(yrange[0]<=value(h)<=yrange[1] for _,_,h in spans)
            stroke(c,[xy(lam,value(h)) for lo,hi,h in spans for lam in (lo,hi)],COLORS[ci],DASHES[ci])
            for lam in [.5,1.,2.]:
                state=validation['configs'][config]['examples'][str(lam)]
                xx,yy=xy(lam,state[key]);c.setFillColor(HexColor(COLORS[ci]));c.circle(xx,yy,1.3,stroke=0,fill=1)
    common_xlabel(c,width,37,True);legend(c,LABELS,width,20,2);c.showPage();c.save()

def utility_figure():
    width,height=252,224;c=canvas.Canvas(str(OUT/'utility_compact.pdf'),pagesize=(width,height))
    c.setTitle('Representative distance-error utility curves')
    for i,config in enumerate(CONFIGS):
        xy=axes(c,(i%2)*126,35+(1-i//2)*92,126,92,f'({chr(97+i)}) '+LABELS[i],[-1,0,.5],[0,250,500],(-1.03,.62),show_x=True)
        label(c,(i%2)*126+3,35+(1-i//2)*92+45,'U',8)
        rows=sorted([r for r in norm if r['config']==config and r['scope']=='pooled' and r['statistic']=='p95'],key=lambda r:int(r['step']))
        for ci,lam in enumerate([.5,1,2]):
            pts=[(int(r['step']),float(r['benefit'])-lam*float(r['grouped'])) for r in rows]
            assert all(-1.03<=v<=.62 for _,v in pts)
            stroke(c,[xy(h,v) for h,v in pts],COLORS[ci],DASHES[ci])
    common_xlabel(c,width,22);legend(c,['0.5','1','2'],width,6,3,True);c.showPage();c.save()

import sys
if '--horizon-only' in sys.argv:
    raw_figure();utility_figure()
    print('Updated raw-error and utility figures with x tick labels on every panel; numerical curves unchanged.')
    sys.exit(0)
if '--lambda-only' in sys.argv:
    lambda_figure()
    sys.exit(0)
raw_figure();lambda_figure();utility_figure()
(OUT/'compact_layout_manifest.json').write_text(json.dumps({'source_sha256':{n:hashlib.sha256((ROOT/'results'/n).read_bytes()).hexdigest() for n in ['raw_curves.csv','normalized_curves.csv','exact_lambda_intervals.csv']},'size_pt':{'five_metrics_compact.pdf':[516,238],'lambda_overview_compact.pdf':[252,244],'utility_compact.pdf':[252,224]},'font_size_pt':[7.5,8],'data_changed':False},indent=2),encoding='utf-8')
print('Created compact vector figures; all plotted values fit their axes.')
