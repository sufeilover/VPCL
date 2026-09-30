"""Offline reproduction from packaged processed data. Never starts AC or sends commands."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import runpy
import shutil
import sys

ROOT=Path(__file__).resolve().parent

def run_source(path, replacements):
    text=path.read_text(encoding='utf-8-sig')
    for old,new in replacements:
        if old not in text: raise ValueError(f'Adapter no longer matches {path.name}: {old}')
        text=text.replace(old,new)
    exec(compile(text,str(path),'exec'),{'__name__':'__main__','__file__':str(path)})

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task',choices=['section4','case1','case2','human','all'])
    parser.add_argument('--output',type=Path,default=ROOT/'outputs')
    args=parser.parse_args(); out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    tasks=['section4','case1','case2','human'] if args.task=='all' else [args.task]
    for task in tasks:
        dest=out/task;dest.mkdir(parents=True,exist_ok=True)
        if task=='section4':
            base=ROOT/'chapters/04_reproduction'
            (dest/'figures').mkdir(exist_ok=True)
            # The upstream figure code expects results/ and a validation JSON.
            shutil.copytree(base/'data',dest/'results',dirs_exist_ok=True)
            shutil.copy2(base/'data/lambda_overview_validation.json',dest/'figures/lambda_overview_validation.json')
            import reportlab
            font=Path(reportlab.__file__).parent/'fonts/VeraIt.ttf'
            if not font.exists(): raise FileNotFoundError(font)
            run_source(base/'source_scripts/draw_compact_figures.py',[
                ('ROOT=Path(__file__).resolve().parent',f'ROOT=Path({str(dest)!r})'),
                ("'C:/Windows/Fonts/timesi.ttf'",repr(str(font)))])
        elif task=='case1':
            base=ROOT/'chapters/05_case1_visual'
            run_source(base/'source_scripts/build_exp1_evidence.py',[
                ('ROOT=Path(__file__).resolve().parent',f'ROOT=Path({str(dest)!r})'),
                ("SRC=Path('E:/IEEE-TVT/new-similar/case study1_drift_analysis')",f'SRC=Path({str(base/"data")!r})')])
        elif task=='case2':
            base=ROOT/'chapters/05_case2_control'
            path=base/'source_scripts/compare_ai_assisted_full_track.py'
            saved=sys.argv[:]
            try:
                sys.argv=[str(path),'--input-dir',str(base/'data/full_track_results'),'--output-dir',str(dest),'--no-plots']
                runpy.run_path(str(path),run_name='__main__')
            finally: sys.argv=saved
        else:
            base=ROOT/'chapters/05_case3_human/data/run_summary.csv'
            with base.open(encoding='utf-8',newline='') as f: rows=list(csv.DictReader(f))
            # The manuscript uses continuous-polyline distance, not sampled-point distance.
            metrics=['speedKmh_time_mean','reference_lateral_abs_m_time_mean','reference_lateral_abs_m_p95']
            records=[]
            for pid in sorted({r['participant'] for r in rows}):
                for condition in sorted({r['condition'] for r in rows}):
                    runs=[r for r in rows if r['participant']==pid and r['condition']==condition]
                    if len(runs)!=3: raise ValueError('Expected three runs per condition.')
                    record={'participant':pid,'condition':condition,'n_runs':len(runs)}
                    for m in metrics: record[m]=sum(float(r[m]) for r in runs)/len(runs)
                    records.append(record)
            with (dest/'participant_summary.csv').open('w',encoding='utf-8',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
        print(f'{task}: {dest}')

if __name__=='__main__': main()
