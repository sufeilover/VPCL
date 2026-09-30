"""Regression checks against packaged comparison results and manuscript table values."""
from pathlib import Path
import csv
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]
for name in ['group_statistics.csv','ai_assisted_differences.csv']:
    expected=pd.read_csv(ROOT/'chapters/05_case2_control/data/comparison'/name)
    actual=pd.read_csv(ROOT/'outputs/case2'/name)
    pd.testing.assert_frame_equal(actual,expected,check_exact=False,rtol=1e-10,atol=1e-10)
with (ROOT/'outputs/human/participant_summary.csv').open() as f: rows=list(csv.DictReader(f))
# Source: revised manuscript participant table, paired as human-only/assisted.
values={
 'P01':[(91.37,1.180,3.420),(96.48,.694,2.276)],
 'P02':[(97.06,1.751,5.468),(97.91,.734,2.707)],
 'P03':[(84.90,1.263,3.883),(86.93,.603,1.723)],
 'P04':[(88.09,1.138,3.558),(88.62,.508,1.569)],
 'P05':[(107.09,2.051,6.270),(102.63,.764,2.547)]}
cols=['speedKmh_time_mean','reference_lateral_abs_m_time_mean','reference_lateral_abs_m_p95']
for r in rows:
    ref=values[r['participant']][0 if r['condition']=='human-only' else 1]
    for col,value,digits in zip(cols,ref,[2,3,3]):
        assert round(float(r[col]),digits)==value,(r['participant'],col)
curves=pd.read_csv(ROOT/'chapters/04_reproduction/data/paper_preference_values.csv')
selected=curves[(curves.statistic=='p95') & (curves.loss=='grouped') & (curves.lambda_value==1)]
assert selected.selected_step.tolist()==[261,248,256,253]
print('Case 2 numerical outputs match packaged references; human table and Section IV preference values match the manuscript.')
