"""Section IV reproduction timing; legacy filename retained.
Requires numpy/pandas. matplotlib is optional. Run directly in VS Code.
No fixed-cycle, deadline, elapsed-time or automatic steady-state assumptions.
"""
from pathlib import Path
import argparse, hashlib, json
import numpy as np
import pandas as pd

DATA_DIR=Path(__file__).resolve().parent
DEFAULT_CLIENT=DATA_DIR/'AC_time.csv'
DEFAULT_OPTIMIZER=DATA_DIR/'model_time.csv'
DEFAULT_OUTPUT=DATA_DIR/'outputs'/'section4_reproduction_timing'
CLIENT=['client_build_ms','client_send_call_ms','client_send_start_to_recv_ms','client_send_end_to_recv_ms','result_parse_ms']
MODEL=['recv_parse_ms','event_wait_ms','build_hotstart_ms','snapshot_stabilize_ms','state_restore_ms','rollout_prepare_ms','rollout_with_logging_ms','postprocess_ms','state_csv_flush_ms','optimizer_processing_ms','optimizer_total_ms']

def read_csv(path):
    d=pd.read_csv(path,encoding='utf-8-sig')
    if 'request_id' not in d:raise ValueError(f'{path}: missing request_id')
    x=pd.to_numeric(d.request_id,errors='coerce')
    if not (np.isfinite(x)&(x>=0)&(x%1==0)&(x<2**53)).all():raise ValueError(f'{path}: invalid request IDs')
    d['request_id']=x.astype('int64')
    if d.request_id.duplicated().any():raise ValueError(f'{path}: duplicate request IDs; check mixed sessions')
    return d

def numeric(d,key):
    x=d[key] if key in d else pd.Series(np.nan,index=d.index)
    return pd.to_numeric(x,errors='coerce').replace([np.inf,-np.inf],np.nan)

def summarize(x):
    v=x.dropna();r={'count':len(v),'missing_or_invalid_count':len(x)-len(v)}
    r.update({k:None for k in ['mean_ms','median_ms','std_ms','p90_ms','p95_ms','p99_ms','min_ms','max_ms']})
    if len(v):
        r.update(mean_ms=float(v.mean()),median_ms=float(v.median()),std_ms=float(v.std()) if len(v)>1 else None,
                 p90_ms=float(v.quantile(.90)),p95_ms=float(v.quantile(.95)),p99_ms=float(v.quantile(.99)),min_ms=float(v.min()),max_ms=float(v.max()))
    return r

def plots(d,out,warnings):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        warnings.append('未安装matplotlib，跳过图片；统计与报告仍正常生成。安装：python -m pip install matplotlib')
        return []
    result=[]
    for kind in ['trace','ecdf']:
        fig,ax=plt.subplots(figsize=(8,4),constrained_layout=True)
        for key,label in [('client_send_start_to_recv_ms','End-to-end'),('model_optimizer_total_ms','Model total')]:
            if kind=='trace':ax.plot(d.request_id,d[key],lw=.8,label=label)
            else:
                v=np.sort(d[key].dropna());ax.step(v,np.arange(1,len(v)+1)/len(v),where='post',label=label)
        ax.set(xlabel='Request ID (not elapsed time)' if kind=='trace' else 'Duration (ms)',ylabel='Duration (ms)' if kind=='trace' else 'Empirical cumulative probability',title='Section IV reproduction timing')
        ax.grid(alpha=.2);ax.legend();name=f'timing_{kind}.png';fig.savefig(out/name,dpi=180);plt.close(fig);result.append(name)
    return result

def fmt(x):return '不可用' if x is None or pd.isna(x) else f'{x:.3f}'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--client',type=Path,default=DEFAULT_CLIENT)
    p.add_argument('--optimizer','--model',dest='optimizer',type=Path,default=DEFAULT_OPTIMIZER)
    p.add_argument('--output-dir',type=Path,default=DEFAULT_OUTPUT)
    p.add_argument('--start-request-id',type=int);p.add_argument('--end-request-id',type=int)
    p.add_argument('--exclude-request-id',type=int,nargs='*',default=[])
    p.add_argument('--skip-first',type=int,default=0,help='Explicitly exclude first N matched requests after range/ID filtering')
    p.add_argument('--no-plots',action='store_true');a=p.parse_args()
    if a.skip_first<0:p.error('--skip-first must be nonnegative')
    if a.start_request_id is not None and a.end_request_id is not None and a.start_request_id>a.end_request_id:p.error('Invalid ID range')
    c=read_csv(a.client);m=read_csv(a.optimizer)
    if 'client_send_start_to_recv_ms' not in c or 'optimizer_total_ms' not in m:raise ValueError('Missing required end-to-end or model-total duration')
    j=c.merge(m.rename(columns={k:'model_'+k for k in m if k!='request_id'}),on='request_id',how='inner',validate='one_to_one').sort_values('request_id').reset_index(drop=True)
    if j.empty:raise ValueError('No matching requests; check recording session')
    # Client echoes provide independent evidence for request identity.
    consistency=[]
    for key in ['round_idx','prediction_steps','optimizer_total_ms','optimizer_processing_ms','rollout_with_logging_ms','state_csv_enabled']:
        mk='model_'+key
        if key not in j or mk not in j:continue
        if key=='state_csv_enabled':
            left=j[key].astype(str).str.strip().str.lower()
            right=j[mk].astype(str).str.strip().str.lower()
            bad=left.ne(right)
        else:
            left=numeric(j,key);right=numeric(j,mk)
            tol=0.05 if key.endswith('_ms') else 0
            bad=left.isna()|right.isna()|((left-right).abs()>tol)
        consistency.extend({'request_id':int(row.request_id),'field':key,'client_value':str(row[key]),'model_value':str(row[mk])} for _,row in j.loc[bad].iterrows())
    if consistency:
        a.output_dir.mkdir(parents=True,exist_ok=True)
        target=a.output_dir/'pairing_mismatches.csv'
        pd.DataFrame(consistency).to_csv(target,index=False,encoding='utf-8-sig')
        (a.output_dir/'pairing_validation.json').write_text(json.dumps({'valid':False,'mismatch_count':len(consistency),'sources':[str(a.client.resolve()),str(a.optimizer.resolve())]},indent=2),encoding='utf-8')
        raise ValueError(f'Request IDs overlap, but echoed model fields disagree ({len(consistency)} field mismatches). Check that AC_time.csv and model_time.csv belong to the same run. No new timing summary generated. Details: {target}')
    d=j.copy()
    if a.start_request_id is not None:d=d[d.request_id>=a.start_request_id]
    if a.end_request_id is not None:d=d[d.request_id<=a.end_request_id]
    d=d[~d.request_id.isin(a.exclude_request_id)].iloc[a.skip_first:].copy()
    if d.empty:raise ValueError('Selected range contains no requests')
    stats=[];warnings=[];invalid=[]
    model_stages=list(dict.fromkeys(MODEL+[k for k in m if k.endswith('_ms') and k!='prediction_sim_ms']))
    stages=[k for k in CLIENT if k in d]+['model_'+v for v in model_stages if 'model_'+v in d]
    for k in stages:
        x=numeric(d,k);bad=x.isna()|(x<0)
        if k not in d:warnings.append(f'缺少可选字段 {k}，标记为不可用，不填零。')
        else:
            invalid.extend({'request_id':int(rid),'column':k} for rid in d.loc[bad,'request_id'])
        d[k]=x.mask(x<0);stats.append({'stage':k,**summarize(d[k])})
    for k in ['client_send_start_to_recv_ms','model_optimizer_total_ms']:
        if not d[k].notna().any():raise ValueError(f'No valid values for {k}')
    d['comm_sched_residual_ms']=d.client_send_start_to_recv_ms-d.model_optimizer_total_ms
    stats.append({'stage':'comm_sched_residual_ms',**summarize(d.comm_sched_residual_ms)})
    negative=int((d.comm_sched_residual_ms<0).sum())
    if negative:warnings.append(f'{negative} 个有符号残差为负，保留原值；应核对计时边界，不当成纯网络时延。')
    if invalid:warnings.append(f'{len(invalid)} 个阶段值缺失/无效；按阶段有效样本数统计，未自动删除整个请求。')
    metadata={k:sorted(d['model_'+k].dropna().astype(str).unique().tolist()) for k in ['prediction_steps','full_horizon_steps','sim_hz','prediction_sim_ms','state_csv_enabled'] if 'model_'+k in d}
    if any(len(metadata.get(k,[]))>1 for k in ['prediction_steps','full_horizon_steps','sim_hz','state_csv_enabled']):
        raise ValueError(f'Mixed prediction/logging settings; analyze each configuration separately: {metadata}')
    out=a.output_dir;out.mkdir(parents=True,exist_ok=True)
    co=c[~c.request_id.isin(m.request_id)];mo=m[~m.request_id.isin(c.request_id)]
    for name,df in [('matched_all_rows',j),('selected_rows',d),('client_only_rows',co),('model_only_rows',mo),('stage_summary',pd.DataFrame(stats)),('invalid_stage_values',pd.DataFrame(invalid,columns=['request_id','column']))]:
        df.to_csv(out/(name+'.csv'),index=False,encoding='utf-8-sig')
    images=[] if a.no_plots else plots(d,out,warnings)
    selection=dict(start=a.start_request_id,end=a.end_request_id,exclude_ids=a.exclude_request_id,skip_first=a.skip_first)
    (out/'pairing_validation.json').write_text(json.dumps({'valid':True,'mismatch_count':0},indent=2),encoding='utf-8')
    payload=dict(experiment='Section IV reproduction',configuration=DATA_DIR.name,metadata=metadata,sources=[dict(path=str(f.resolve()),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in [a.client,a.optimizer]],client_rows=len(c),model_rows=len(m),matched_rows=len(j),selected_rows=len(d),client_only_rows=len(co),model_only_rows=len(mo),excluded_matched_rows=len(j)-len(d),selection=selection,negative_residual_count=negative,elapsed_duration_s=None,deadline_statistics=None,summary=stats,plots=images,warnings=warnings)
    (out/'analysis_summary.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    lines=['# 第四章：复现预测时延分析','',f'- 配置：{DATA_DIR.name}；日志参数：{metadata}。',f'- 客户端 {len(c)} 条；模型端 {len(m)} 条；匹配 {len(j)} 条；选用 {len(d)} 条。',f'- 仅客户端 {len(co)} 条；仅模型端 {len(mo)} 条；未匹配记录不能直接认定为丢包。',f'- 显式筛选：{selection}。默认保留全部匹配请求，不自动删除慢请求或认定稳态。','- 两份日志必须来自同一会话；请求编号仅用于配对与排序，不能推算总时长、周期或圈数。','','| 阶段 | 有效数 | 缺失/无效数 | 均值 ms | P95 ms | 最大值 ms |','|---|---:|---:|---:|---:|---:|']
    for r in stats:lines.append(f'| {r["stage"]} | {r["count"]} | {r["missing_or_invalid_count"]} | {fmt(r["mean_ms"])} | {fmt(r["p95_ms"])} | {fmt(r["max_ms"])} |')
    lines+=['','## 解释与限制','',
      '- 端到端为客户端发送开始至收到结果，不含发送前构建数据包或后续界面显示完成。',
      '- model_optimizer_total_ms 为模型日志中的 optimizer_total_ms；沿用字段名不代表本实验运行控制优化器。',
      '- 只统计日志实际提供的阶段。optimizer_processing_ms、optimizer_total_ms、elapsed_ms及子阶段可能重叠，不可直接相加。prediction_sim_ms是预测覆盖的模拟时间，不是程序执行耗时。',
      '- 端到端减模型总耗时是配对请求的有符号残差，受计时边界、通信与调度影响，不是独立网络延迟测量。',
      '- 不从历史文件名推定固定周期或截止时间；本日志未提供经确认的deadline协议，不计算deadline成功率。',
      '- 启动处理须由日志/协议确定；如有依据，可显式设置请求范围、排除ID或skip-first，并保留全量分析供比较。',
      '- 时延分布不单独证明硬实时、安全收益或人类反应改善。','','## 核查提示','']
    lines += ['- '+w for w in warnings] or ['- 未发现额外字段警告。']
    (out/'timing_report_cn.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(f'Matched={len(j)}, selected={len(d)}')
    for r in stats:
        if r['stage'] in ['client_send_start_to_recv_ms','model_optimizer_total_ms']:print(r)
    for w in warnings:print('WARNING:',w)
    print('Output:',out)

if __name__=='__main__':main()
