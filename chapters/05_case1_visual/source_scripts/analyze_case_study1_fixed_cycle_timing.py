"""Case Study 1 visual-feedback timing; legacy filename retained.
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
DEFAULT_OUTPUT=DATA_DIR/'outputs'/'case_study1_visual_feedback_timing'
CLIENT=['client_build_ms','client_send_call_ms','client_send_start_to_recv_ms','client_send_end_to_recv_ms','result_parse_ms']
MODEL=['recv_parse_ms','event_wait_ms','hotstart_ms','stabilize_ms','predict_to_80_ms','predict_to_160_ms','predict_to_240_ms','predict_total_ms','matrix_build_ms','return_send_ms','optimizer_processing_ms','optimizer_total_ms']

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
        ax.set(xlabel='Request ID (not elapsed time)' if kind=='trace' else 'Duration (ms)',ylabel='Duration (ms)' if kind=='trace' else 'Empirical cumulative probability',title='Case Study 1 timing')
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
    d=j.copy()
    if a.start_request_id is not None:d=d[d.request_id>=a.start_request_id]
    if a.end_request_id is not None:d=d[d.request_id<=a.end_request_id]
    d=d[~d.request_id.isin(a.exclude_request_id)].iloc[a.skip_first:].copy()
    if d.empty:raise ValueError('Selected range contains no requests')
    stats=[];warnings=[];invalid=[]
    for k in CLIENT+['model_'+v for v in MODEL]:
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
    chk=d[['model_predict_to_80_ms','model_predict_to_160_ms','model_predict_to_240_ms']].dropna()
    nonmono=int((np.diff(chk.to_numpy(),axis=1)<0).any(axis=1).sum())
    if nonmono:warnings.append(f'{nonmono} 条累计预测计时不单调。')
    out=a.output_dir;out.mkdir(parents=True,exist_ok=True)
    co=c[~c.request_id.isin(m.request_id)];mo=m[~m.request_id.isin(c.request_id)]
    for name,df in [('matched_all_rows',j),('selected_rows',d),('client_only_rows',co),('model_only_rows',mo),('stage_summary',pd.DataFrame(stats)),('invalid_stage_values',pd.DataFrame(invalid,columns=['request_id','column']))]:
        df.to_csv(out/(name+'.csv'),index=False,encoding='utf-8-sig')
    images=[] if a.no_plots else plots(d,out,warnings)
    selection=dict(start=a.start_request_id,end=a.end_request_id,exclude_ids=a.exclude_request_id,skip_first=a.skip_first)
    payload=dict(experiment='Case Study 1 visual feedback',sources=[dict(path=str(f.resolve()),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in [a.client,a.optimizer]],client_rows=len(c),model_rows=len(m),matched_rows=len(j),selected_rows=len(d),client_only_rows=len(co),model_only_rows=len(mo),excluded_matched_rows=len(j)-len(d),selection=selection,negative_residual_count=negative,nonmonotonic_checkpoint_count=nonmono,elapsed_duration_s=None,deadline_statistics=None,summary=stats,plots=images,warnings=warnings)
    (out/'analysis_summary.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    lines=['# 实验1：预测可视化反馈时延分析','',f'- 客户端 {len(c)} 条；模型端 {len(m)} 条；匹配 {len(j)} 条；选用 {len(d)} 条。',f'- 仅客户端 {len(co)} 条；仅模型端 {len(mo)} 条；未匹配记录不能直接认定为丢包。',f'- 显式筛选：{selection}。默认保留全部匹配请求，不自动删除慢请求或认定稳态。','- 两份日志必须来自同一会话；请求编号仅用于配对与排序，不能推算总时长、周期或圈数。','','| 阶段 | 有效数 | 缺失/无效数 | 均值 ms | P95 ms | 最大值 ms |','|---|---:|---:|---:|---:|---:|']
    for r in stats:lines.append(f'| {r["stage"]} | {r["count"]} | {r["missing_or_invalid_count"]} | {fmt(r["mean_ms"])} | {fmt(r["p95_ms"])} | {fmt(r["max_ms"])} |')
    lines+=['','## 解释与限制','',
      '- 端到端为客户端发送开始至收到结果，不含发送前构建数据包或后续界面显示完成。',
      '- model_optimizer_total_ms 为模型日志中的 optimizer_total_ms；沿用字段名不代表实验1运行控制优化器。',
      '- predict_to_80/160/240_ms 为累计计时，不能相加；predict_total_ms 与这些检查点有重叠。',
      '- 端到端减模型总耗时是配对请求的有符号残差，受计时边界、通信与调度影响，不是独立网络延迟测量。',
      '- 实验1不使用实验2的240 ms固定周期、150 ms发送点、90 ms deadline，故不计算deadline成功率。100 ms显示保持时间不是硬截止时间。',
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
