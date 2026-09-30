"""Export authorized participant summaries; do not export raw paths or timestamps."""
import argparse
import csv
import json
from pathlib import Path

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    with args.input.open(encoding='utf-8-sig',newline='') as f:
        reader=csv.DictReader(f); rows=list(reader); fields=reader.fieldnames
    if len(rows)!=30: raise ValueError('Expected the 30 recorded human runs.')
    remove={'source','source_sha256','run_id'}
    kept=[x for x in fields if x not in remove]
    groups={}
    for r in rows:
        key=(r['participant'],r['condition'])
        groups[key]=groups.get(key,0)+1
    if len(groups)!=10 or set(groups.values())!={3}: raise ValueError('Expected five participants, two conditions, three runs each.')
    output=[]
    for r in rows:
        record={k:r[k] for k in kept}
        record['run_id']=f"P{int(r['participant']):02d}_{r['condition']}_R{int(r['run_order_in_condition']):02d}"
        record['participant']=f"P{int(r['participant']):02d}"
        for k,v in record.items():
            if ':\\' in v or ':/' in v: raise ValueError(f'Unexpected local path in {k}')
        for k in kept:
            if k!='participant': assert record[k]==r[k]
        output.append(record)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['run_id']+kept);w.writeheader();w.writerows(output)
    print(json.dumps({'rows':len(output),'participant_conditions':len(groups),'numeric_cells_unchanged':True,'removed':sorted(remove)}))

if __name__=='__main__': main()
