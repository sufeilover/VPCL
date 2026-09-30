"""Reproducible three-layer AC-referenced horizon trade-off analysis.
Run: python three_layer_analysis.py --root E:/IEEE-TVT/new-similar
Dependencies: numpy, pandas, openpyxl, Pillow. No breakpoint is required.
"""
from pathlib import Path
import argparse, hashlib, json, math, sys
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

CONFIGS = {
"ks_barcelona+ks_toyota_ae86_drift": "Barcelona + AE86",
"ks_barcelona+ks_toyota_supra_mkiv_drift": "Barcelona + Supra",
"ks_silverstone1967+ks_toyota_ae86_drift": "Silverstone + AE86",
"ks_silverstone1967+ks_toyota_supra_mkiv_drift": "Silverstone + Supra"}
METRICS = ["pos3d_rmse_m","pos3d_finalae_m","speedKmh_rmse","attitude_mean_mae_deg","SlipAngle_mean_rmse"]
LABELS = ["Position RMSE","Terminal position error","Speed RMSE","Attitude MAE","Slip-angle RMSE"]
UNITS = ["m","m","km/h","deg","deg"]
CN = ["位置RMSE","终点位置误差","速度RMSE","姿态MAE","轮胎滑移角RMSE"]
WEIGHTS = np.array([.125,.125,.25,.25,.25])
COLORS = ["#2563eb","#f97316","#16a34a","#9333ea","#e11d48","#0891b2","#475569","#a16207"]
SETTINGS = [80,85,90,95,100]
MODEL_STEP_SECONDS = 1.0 / 333.0
SOURCES = []
def track_source(p):
    p=Path(p); h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""): h.update(b)
    SOURCES.append(dict(path=str(p.resolve()),size=p.stat().st_size,sha256=h.hexdigest()))
def save(df,p):
    df.to_csv(p,index=False,encoding="utf-8-sig")
def weighted_quantile_columns(a,w,q=.95):
    order=np.argsort(a,axis=0)
    sorted_a=np.take_along_axis(a,order,axis=0)
    sorted_w=w[order]; cdf=np.cumsum(sorted_w,axis=0)/w.sum()
    k=(cdf>=q).argmax(axis=0)
    return sorted_a[k,np.arange(a.shape[1])]
def normalize(e,method,cap):
    if method=="endpoint":
        lo=e[0]; scale=e[cap-1]-lo
    elif method=="range":
        lo=e[:cap].min(axis=0); scale=e[:cap].max(axis=0)-lo
    elif method=="terminal_band":
        lo=np.median(e[:10],axis=0)
        scale=np.median(e[max(0,cap-25):cap],axis=0)-lo
    else: raise ValueError(method)
    if np.any(~np.isfinite(scale)) or np.any(scale<=1e-10):
        raise ValueError("Undefined normalization scale")
    return (e-lo)/scale,lo,scale
def loss_columns(z):
    return np.column_stack([z,z@WEIGHTS,z.max(axis=1),z.mean(axis=1)])
LOSS_NAMES=METRICS+["grouped","worst","equal5"]
def optimize(b,L,lambdas):
    # Earliest maximizing horizon only for numerical ties (1e-12).
    u=b[:,None,None]-L[:,:,None]*lambdas[None,None,:]
    maxu=u.max(axis=0)
    idx=(u>=maxu[None,:,:]-1e-12).argmax(axis=0)
    return idx,maxu
def exact_lambda_intervals(b,l,steps):
    rows=[]
    for i in range(len(b)):
        db=b[i]-b; dl=l[i]-l
        lower=0.; upper=float("inf"); good=True
        for j in range(len(b)):
            if abs(dl[j])<1e-12:
                if db[j]<-1e-12: good=False;break
            elif dl[j]>0: upper=min(upper,float(db[j]/dl[j]))
            else: lower=max(lower,float(db[j]/dl[j]))
        if good and upper>=max(0,lower)-1e-10 and upper>=0:
            rows.append(dict(step=int(steps[i]),lambda_low=max(0,lower),lambda_high=upper))
    return rows
def load_config(root,folder,out):
    config=CONFIGS[folder]; rd=root/folder/"similarity_500_results"
    manifest=json.loads((rd/"similarity_analysis_manifest.json").read_text(encoding="utf-8-sig"))
    assert manifest["horizon_steps"]==500 and manifest["model_step_seconds"]==MODEL_STEP_SECONDS
    assert "interpolated" in manifest["time_alignment_rule"]
    track_source(rd/"similarity_analysis_manifest.json")
    track_source(rd/"matching_qc.csv")
    cubes=[]; distances=[]; tags=[]; qc=[]; corr=[]
    for setting in SETTINGS:
        print(f"Reading {config} {setting}",flush=True)
        p=rd/f"setting_{setting}_rollout_horizon.csv";track_source(p)
        fields=["loop160_round_idx","horizon_step","ac_start_time_s","ac_elapsed_time_s"]+METRICS
        d=pd.read_csv(p,usecols=fields)
        if folder == "ks_silverstone1967+ks_toyota_supra_mkiv_drift" and setting == 100:
            removed = d[d.loop160_round_idx.isin([172,173,174])]
            assert len(removed) == 1500 and removed.loop160_round_idx.nunique() == 3
            d = d[~d.loop160_round_idx.isin([172,173,174])].copy()
        assert not d.duplicated(["loop160_round_idx","horizon_step"]).any()
        d=d.sort_values(["loop160_round_idx","horizon_step"])
        groups=d.groupby("loop160_round_idx")
        assert (groups.size()==500).all()
        n=groups.ngroups
        assert np.array_equal(d.horizon_step.to_numpy().reshape(n,500),np.tile(np.arange(1,501),(n,1)))
        arr=d[METRICS].to_numpy().reshape(n,500,5)
        assert np.isfinite(arr).all() and (arr>=0).all()
        starts=groups.ac_start_time_s.first().to_numpy()
        assert np.allclose(d.ac_elapsed_time_s.to_numpy().reshape(n,500),np.arange(1,501)*MODEL_STEP_SECONDS,atol=1e-7)
        # Use current AC content and verify the manifest hash, no stale cached distances.
        acpath=root/folder/f"{setting}_AC.xlsx";track_source(acpath)
        expected=next(v["sha256"] for v in manifest["input_files"] if v["setting"]==setting and v["kind"]=="AC")
        assert SOURCES[-1]["sha256"]==expected, f"AC source changed: {acpath}"
        ac=pd.read_excel(acpath,sheet_name="telemetry",usecols=["t_sec","x","y","z"])
        print(f"AC loaded: {len(ac)}",flush=True)
        assert np.isfinite(ac.to_numpy(float)).all()
        assert (np.diff(ac.t_sec)>=0).all()
        ac=ac.drop_duplicates("t_sec",keep="last")
        t=ac.t_sec.to_numpy(); xyz=ac[["x","y","z"]].to_numpy()
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xyz,axis=0),axis=1))]
        target=starts[:,None]+np.arange(1,501)*MODEL_STEP_SECONDS
        assert starts.min()>=t.min() and target.max()<=t.max()+1e-8
        dist=np.interp(target,t,arc)-np.interp(starts,t,arc)[:,None]
        assert (dist>=0).all() and (np.diff(dist,axis=1)>=-1e-9).all()
        cubes.append(arr);distances.append(dist);tags.extend([setting]*n)
        qc.append(dict(config=config,setting=setting,valid_rollouts=n,horizon_rows=len(d),
                       ac_rows=len(ac),distance_100_m=dist[:,99].mean(),distance_500_m=dist[:,-1].mean(),
                       max_raw_ac_position_jump_m=np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()))
        for h in [125,250,500]:
            matrix=pd.DataFrame(arr[:,h-1,:],columns=METRICS).corr(method="spearman")
            for j in range(5):
                for k in range(j+1,5):
                    corr.append(dict(config=config,setting=setting,step=h,metric_a=METRICS[j],metric_b=METRICS[k],spearman=matrix.iloc[j,k]))
        print(f"Loaded {config} AIpush={setting}: {n} rollouts",flush=True)
    a=np.concatenate(cubes);dist=np.concatenate(distances);tags=np.array(tags)
    weights=np.array([1/(5*np.sum(tags==s)) for s in tags])
    rows=[]
    for scope in ["pooled","balanced"]+[str(s) for s in SETTINGS]:
        if scope in ["pooled","balanced"]: aa=a; dd=dist
        else: aa=a[tags==int(scope)];dd=dist[tags==int(scope)]
        if scope=="balanced":
            mean=np.average(aa,axis=0,weights=weights)
            p95=weighted_quantile_columns(aa.reshape(len(aa),-1),weights).reshape(500,5)
            median=weighted_quantile_columns(aa.reshape(len(aa),-1),weights,.5).reshape(500,5)
            D=np.average(dd,axis=0,weights=weights)
        else:
            mean=aa.mean(axis=0);p95=np.quantile(aa,.95,axis=0);median=np.median(aa,axis=0);D=dd.mean(axis=0)
        for h in range(500):
            r=dict(config=config,folder=folder,scope=scope,step=h+1,time_s=(h+1)*MODEL_STEP_SECONDS,n_rollouts=len(aa),distance_m=D[h])
            for j,m in enumerate(METRICS):
                r["mean_"+m]=mean[h,j];r["p95_"+m]=p95[h,j];r["median_"+m]=median[h,j]
            rows.append(r)
    return pd.DataFrame(rows),qc,corr

def make_analyses(curves,out):
    scans=[];normrows=[];marginal=[];intervals=[];sensitivity=[];weightsens=[];anchors=[]
    lambdas=np.unique(np.r_[np.logspace(-1,1,241),[.25,.5,1,2,4]])
    for (config,scope),g in curves.groupby(["config","scope"],sort=False):
        g=g.sort_values("step");D=g.distance_m.to_numpy();steps=g.step.to_numpy()
        b=(D-D[0])/(D[-1]-D[0])
        for stat in ["mean","p95"]:
            e=g[[stat+"_"+m for m in METRICS]].to_numpy()
            z,lo,scale=normalize(e,"endpoint",500);L=loss_columns(z)
            idx,u=optimize(b,L,lambdas)
            for j,m in enumerate(LOSS_NAMES):
                for k,lam in enumerate(lambdas):
                    h=idx[j,k]
                    scans.append(dict(config=config,scope=scope,statistic=stat,loss=m,lambda_value=lam,
                        selected_step=int(steps[h]),time_s=steps[h]*MODEL_STEP_SECONDS,distance_m=D[h],benefit=b[h],loss_value=L[h,j],utility=u[j,k]))
                if scope=="pooled":
                    for r in exact_lambda_intervals(b,L[:,j],steps):
                        intervals.append(dict(config=config,statistic=stat,loss=m,**r))
            for j,m in enumerate(METRICS):
                anchors.append(dict(config=config,scope=scope,statistic=stat,metric=m,baseline=lo[j],endpoint=lo[j]+scale[j],scale=scale[j],
                    below_zero_steps=int((z[:,j]<0).sum()),above_one_steps=int((z[:,j]>1).sum())))
            for i,h in enumerate(steps):
                r=dict(config=config,scope=scope,statistic=stat,step=h,distance_m=D[i],benefit=b[i])
                for j,m in enumerate(LOSS_NAMES): r[m]=L[i,j]
                normrows.append(r)
            if scope=="pooled":
                for span in [10,25,50]:
                    for i in range(0,500-span):
                        ds=D[i+span]-D[i]
                        if ds<=1e-10: continue
                        for j,m in enumerate(METRICS):
                            marginal.append(dict(config=config,statistic=stat,metric=m,start_step=i+1,end_step=i+span+1,
                              delta_steps=span,error_per_extra_m=(e[i+span,j]-e[i,j])/ds))
                for method in ["endpoint","range","terminal_band"]:
                    for cap in [300,400,500]:
                        zz,_,_=normalize(e,method,cap)
                        # Keep 1..500 candidates: cap varies only the reference scale.
                        LL=loss_columns(zz)
                        for benefit_scale in ["fixed500","same_cap"]:
                            bb=b if benefit_scale=="fixed500" else (D-D[0])/(D[cap-1]-D[0])
                            ii,_=optimize(bb,LL,np.array([.5,1.,2.]))
                            for j,m in enumerate(LOSS_NAMES):
                                for k,lam in enumerate([.5,1.,2.]):
                                    sensitivity.append(dict(config=config,statistic=stat,method=method,reference_cap=cap,
                                      benefit_scale=benefit_scale,loss=m,lambda_value=lam,selected_step=int(steps[ii[j,k]])))
                groupweights=[("base",np.ones(4)/4)]
                for j in range(4):
                    w=np.ones(4);w[j]=2;w=w/w.sum();groupweights.append((f"double_group_{j}",w))
                for name,w in groupweights:
                    wm=np.r_[w[0]/2,w[0]/2,w[1:]]
                    ii,_=optimize(b,(z@wm)[:,None],lambdas)
                    for k,lam in enumerate(lambdas):
                        weightsens.append(dict(config=config,statistic=stat,weights=name,lambda_value=lam,selected_step=int(steps[ii[0,k]])))
    outputs=dict(lambda_scan=pd.DataFrame(scans),normalized_curves=pd.DataFrame(normrows),marginal_errors=pd.DataFrame(marginal),
      exact_lambda_intervals=pd.DataFrame(intervals),normalization_sensitivity=pd.DataFrame(sensitivity),
      group_weight_sensitivity=pd.DataFrame(weightsens),normalization_anchors=pd.DataFrame(anchors))
    for name,df in outputs.items():save(df,out/(name+".csv"))
    return outputs

def font(size=20,bold=False):
    return ImageFont.truetype("C:/Windows/Fonts/"+("arialbd.ttf" if bold else "arial.ttf"),size)
def panel_figure(panels,title,destination,cols=2):
    # Standalone PNGs: all coordinates and numerical tick labels generated from data.
    rows=math.ceil(len(panels)/cols);pw=780;ph=530
    im=Image.new("RGB",(pw*cols,ph*rows+70),"white");draw=ImageDraw.Draw(im)
    draw.text((35,20),title,fill="#111827",font=font(25,True))
    for k,p in enumerate(panels):
        x0=(k%cols)*pw;y0=(k//cols)*ph+70
        box=(x0+90,y0+55,x0+pw-35,y0+ph-160)
        allx=np.concatenate([np.asarray(s[1],float) for s in p["series"]])
        ally=np.concatenate([np.asarray(s[2],float) for s in p["series"]])
        islog=p.get("log",False)
        xmin,xmax=np.nanmin(allx),np.nanmax(allx)
        ymin,ymax=p.get("ylim",(min(0.,np.nanmin(ally)),np.nanmax(ally)))
        yr=max(ymax-ymin,1e-9);ymax+=yr*.04
        def xy(x,y):
            xx=(np.log10(x)-np.log10(xmin))/(np.log10(xmax)-np.log10(xmin)) if islog else (x-xmin)/max(xmax-xmin,1e-12)
            return (box[0]+xx*(box[2]-box[0]),box[3]-(y-ymin)/(ymax-ymin)*(box[3]-box[1]))
        draw.text((box[0],y0+10),p["title"],fill="#111827",font=font(21,True))
        for yy in np.linspace(ymin,ymax,5):
            _,yp=xy(xmin,yy);draw.line((box[0],yp,box[2],yp),fill="#e5e7eb")
            draw.text((box[0]-8,yp),f"{yy:.3g}",font=font(16),anchor="rm",fill="#374151")
        xticks=[.1,.2,.5,1,2,5,10] if islog else np.linspace(xmin,xmax,6)
        for xx in xticks:
            xp,_=xy(xx,ymin);draw.line((xp,box[1],xp,box[3]),fill="#edf0f3")
            draw.text((xp,box[3]+10),f"{xx:.3g}",anchor="ma",font=font(16),fill="#374151")
        draw.line((box[0],box[1],box[0],box[3],box[2],box[3]),fill="#374151",width=2)
        for j,(label,x,y) in enumerate(p["series"]):
            pts=[xy(xx,yy) for xx,yy in zip(x,y) if np.isfinite(xx) and np.isfinite(yy)]
            color=COLORS[j%len(COLORS)]
            if len(pts)>1:draw.line(pts,fill=color,width=3)
            lx=box[0]+(j%2)*320;ly=box[3]+64+(j//2)*21
            draw.line((lx,ly+9,lx+23,ly+9),fill=color,width=3)
            draw.text((lx+30,ly),label,fill="#374151",font=font(15))
        draw.text(((box[0]+box[2])/2,box[3]+35),p["xlabel"],anchor="ma",fill="#374151",font=font(17))
        draw.text((box[0],box[1]-24),p["ylabel"],fill="#374151",font=font(16))
    im.save(destination)
def figures(curves,results,out):
    pool=curves[curves.scope=="pooled"]
    for j,m in enumerate(METRICS):
        panels=[]
        for stat in ["mean","p95"]:
            series=[]
            for c,g in pool.groupby("config",sort=False): series.append((c,g.step,g[stat+"_"+m]))
            panels.append(dict(title=stat.upper(),xlabel="Prediction horizon (steps)",ylabel=UNITS[j],series=series))
        panel_figure(panels,LABELS[j]+": AC-referenced errors",out/f"01_raw_{j+1}.png")
        panels=[]
        for stat in ["mean","p95"]:
            series=[]
            for c,g in pool.groupby("config",sort=False):series.append((c,g.distance_m,g[stat+"_"+m]))
            panels.append(dict(title=stat.upper(),xlabel="Mean AC reference travel (m)",ylabel=UNITS[j],series=series))
        panel_figure(panels,LABELS[j]+": distance-error trade-off",out/f"01_distance_{j+1}.png")
    norm=results["normalized_curves"];scan=results["lambda_scan"]
    for stat in ["mean","p95"]:
        panels=[];aggregate=[];lambdap=[]
        for c,g in norm[(norm.scope=="pooled")&(norm.statistic==stat)].groupby("config",sort=False):
            panels.append(dict(title=c,xlabel="Prediction horizon (steps)",ylabel="Relative growth",series=
              [(LABELS[j],g.step,g[m]) for j,m in enumerate(METRICS)]+[("Distance benefit",g.step,g.benefit)]))
            aggregate.append(dict(title=c,xlabel="Prediction horizon (steps)",ylabel="Normalized benefit / loss",series=
              [("Distance benefit",g.step,g.benefit),("Grouped loss",g.step,g.grouped),("Worst growth",g.step,g.worst),("Equal-five loss",g.step,g.equal5)]))
            sg=scan[(scan.config==c)&(scan.scope=="pooled")&(scan.statistic==stat)]
            lambdap.append(dict(title=c,xlabel="Lambda (log scale)",ylabel="Selected horizon (steps)",log=True,ylim=(0,500),series=
              [(LABELS[j] if j<5 else ["Grouped","Worst"][j-5],ss.lambda_value,ss.selected_step)
               for j,m in enumerate(LOSS_NAMES[:7]) for ss in [sg[sg.loss==m]]]))
        panel_figure(panels,"Normalized five-metric curves: "+stat.upper(),out/f"02_normalized_{stat}.png")
        panel_figure(aggregate,"Benefit and aggregate losses: "+stat.upper(),out/f"02_aggregate_{stat}.png")
        panel_figure(lambdap,"Individual and aggregate lambda sensitivity: "+stat.upper(),out/f"03_lambda_{stat}.png")
    marginal=results["marginal_errors"]
    panels=[]
    for j,m in enumerate(METRICS):
        series=[]
        for c,g in marginal[(marginal.statistic=="p95")&(marginal.delta_steps==25)&(marginal.metric==m)].groupby("config",sort=False):
            series.append((c,(g.start_step+g.end_step)/2,g.error_per_extra_m))
        panels.append(dict(title=LABELS[j],xlabel="Window midpoint (steps)",ylabel=UNITS[j]+"/extra m",series=series))
    panel_figure(panels,"Marginal P95 error growth over 25-step windows",out/"01_marginal_p95.png")

def mdtable(df,columns=None,digits=3):
    if columns is not None:df=df[columns]
    def val(v):
        if isinstance(v,(float,np.floating)):return f"{v:.{digits}f}"
        return str(v).replace("|","/")
    return "| "+" | ".join(map(str,df.columns))+" |\n| "+" | ".join(["---"]*len(df.columns))+" |\n"+"\n".join("| "+" | ".join(val(v) for v in row)+" |" for row in df.itertuples(index=False,name=None))
def selftests():
    e=np.column_stack([np.linspace(j+1,j+3,500)**2 for j in range(5)])
    z,_,_=normalize(e,"endpoint",500)
    assert np.allclose(z[0],0) and np.allclose(z[-1],1)
    a=np.arange(1,501)/500
    ll=loss_columns(z)
    assert np.all(ll[:,6]>=ll[:,5]-1e-12)
    idx,_=optimize(a,ll,np.logspace(-1,1,101))
    assert (np.diff(idx,axis=1)<=0).all()
    # Exact weighted quantile uses inverse weighted empirical CDF.
    assert weighted_quantile_columns(np.array([[1.],[10.]]),np.array([.99,.01]))[0]==1
    r=exact_lambda_intervals(np.array([0.,1.]),np.array([0.,1.]),np.array([1,500]))
    assert len(r)==2 and r[0]["lambda_low"]==1 and r[1]["lambda_high"]==1

if __name__ == "__main__":
    raise SystemExit("Run rebuild_results.py, not analysis_engine.py, for the revised cohort.")
