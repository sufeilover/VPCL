"""Plot exact preference intervals; verify archived scan against normalized curves.

No raw driving files are read. The staircase changes at pairwise utility
intersections, not at the nearest sampled lambda. At an exact tie, the existing
analysis chooses the earliest horizon (tolerance 1e-12).
"""
import math, json, hashlib
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from draw_publication_figures import B, O, F, CONFIGS, COLORS, read, text, line, legend

norm = read('normalized_curves.csv')
raw = read('raw_curves.csv')
scan = read('lambda_scan.csv')
intervals = read('exact_lambda_intervals.csv')
curves, summaries = {}, {}
for config in CONFIGS:
    rows = sorted([r for r in norm if r['config']==config and r['scope']=='pooled' and r['statistic']=='p95'], key=lambda r:int(r['step']))
    dist = {int(r['step']):float(r['distance_m']) for r in raw if r['config']==config and r['scope']=='pooled'}
    states = {int(r['step']): {'step':int(r['step']), 'distance_m':dist[int(r['step'])], 'benefit':float(r['benefit']), 'loss':float(r['grouped'])} for r in rows}
    assert len(states)==500
    def select(lam):
        vals = {h:s['benefit']-lam*s['loss'] for h,s in states.items()}
        maximum=max(vals.values())
        return next(h for h in sorted(states) if vals[h]>=maximum-1e-12)
    checked=0
    for r in scan:
        if r['config']==config and r['scope']=='pooled' and r['statistic']=='p95' and r['loss']=='grouped':
            assert select(float(r['lambda_value']))==int(r['selected_step'])
            checked+=1
    spans=[]
    for r in intervals:
        if r['config']==config and r['statistic']=='p95' and r['loss']=='grouped':
            lo=max(.1,float(r['lambda_low']));hi=min(10.,float(r['lambda_high']))
            if hi>lo+1e-12:
                h=int(r['step']);assert select(math.sqrt(lo*hi))==h
                spans.append((lo,hi,h))
    spans.sort()
    assert abs(spans[0][0]-.1)<1e-8 and abs(spans[-1][1]-10)<1e-8
    for a,b in zip(spans,spans[1:]):assert abs(a[1]-b[0])<1e-8
    # These monotonicities are verified for this dataset, not assumed for raw errors.
    assert all(states[a[2]]['step']>=states[b[2]]['step'] for a,b in zip(spans,spans[1:]))
    assert all(states[a[2]]['loss']>=states[b[2]]['loss']-1e-10 for a,b in zip(spans,spans[1:]))
    curves[config]=(states,spans)
    summaries[config]={'verified_scan_points':checked,'intervals_in_range':len(spans),
        'examples':{str(lam):states[select(lam)] for lam in [.1,.5,1.,2.,10.]},
        'largest_step_drop':max(({'lambda':a[1],'from':a[2],'to':b[2],'drop':a[2]-b[2]} for a,b in zip(spans,spans[1:])),key=lambda r:r['drop'])}

width,height=516,326
c=canvas.Canvas(str(F/'lambda_overview_wide.pdf'),pagesize=(width,height))
c.setTitle('Continuous lambda sensitivity of selected prediction horizons')
panels=[('step','(a) Selected prediction horizon','steps',0,500),('distance_m','(b) Reference distance at selection','m',0,80),('benefit','(c) Distance benefit at selection','B(h*)',0,1),('loss','(d) Grouped P95 loss at selection','Lg(h*)',-.1,1.1)]
for pi,(key,title,unit,ymin,ymax) in enumerate(panels):
    x=(pi%2)*258;y=32+(1-pi//2)*146
    left=x+33;right=x+250;bottom=y+27;top=y+118
    def xy(lam,v):return left+(math.log10(lam)+1)/2*(right-left),bottom+(v-ymin)/(ymax-ymin)*(top-bottom)
    text(c,x+2,y+136,title,9.5,True);text(c,left,top+5,unit,8)
    for j in range(5):
        v=ymin+(ymax-ymin)*j/4;_,yy=xy(.1,v)
        c.setStrokeColor(HexColor('#E2E5E9'));c.setLineWidth(.35);c.line(left,yy,right,yy)
        c.setFillColor(HexColor('#333333'));c.setFont('Times-Roman',8);c.drawRightString(left-4,yy-2.5,f'{v:g}')
    for lam in [.5,1,2]:
        xx,_=xy(lam,0);c.setStrokeColor(HexColor('#AAAAAA'));c.setLineWidth(.5);c.setDash(1,2);c.line(xx,bottom,xx,top);c.setDash()
    c.setStrokeColor(HexColor('#333333'));c.setLineWidth(.6);c.line(left,bottom,right,bottom);c.line(left,bottom,left,top)
    for lam in [.1,.2,.5,1,2,5,10]:
        xx,_=xy(lam,0);c.line(xx,bottom,xx,bottom-3);c.setFillColor(HexColor('#333333'));c.setFont('Times-Roman',8);c.drawCentredString(xx,bottom-12,f'{lam:g}')
    for ci,config in enumerate(CONFIGS):
        states,spans=curves[config]
        pts=[xy(lam,states[h][key]) for lo,hi,h in spans for lam in (lo,hi)]
        assert all(ymin<=states[h][key]<=ymax for _,_,h in spans)
        line(c,pts,COLORS[ci],() if ci%2==0 else (4,2))
        for lam in [.5,1.,2.]:
            state=summaries[config]['examples'][str(lam)]
            xx,yy=xy(lam,state[key]);c.setFillColor(HexColor(COLORS[ci]));c.circle(xx,yy,1.6,stroke=0,fill=1)
    c.setFillColor(HexColor('#222222'));c.setFont('Times-Roman',8.5);c.drawCentredString((left+right)/2,y+2,'lambda (log scale)')
legend(c,[s.replace(' + ','/') for s in CONFIGS],COLORS,width,10)
c.showPage();c.save()
record={'protocol':'pooled P95; grouped loss; fixed 500-step endpoint normalization; 5718 retained rollouts',
    'source_sha256':{n:hashlib.sha256((O/n).read_bytes()).hexdigest() for n in ['raw_curves.csv','normalized_curves.csv','lambda_scan.csv','exact_lambda_intervals.csv']},
    'configs':summaries}
(F/'lambda_overview_validation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
print(json.dumps(summaries,indent=2))
