# Case 2: control-oriented driving assistance

Run `python reproduce.py case2`. Inputs are `data/full_track_results/`; reference outputs are `data/comparison/`. Groups are compared within AIpush, with five AI and seven assisted runs per setting (60 runs total). The comparison is unpaired and run-weighted. All retained tracking summaries are whole-circuit, not straight/corner partitions.

Distance in this case is to the nearest sampled track-reference point in the XZ plane. The `cte` names are legacy field names, not evidence of a different geometric definition. P95 values summarized across runs are not P95 of all pooled samples.

The optimizer returns commands for direct application. Tracking/speed cost weights are not authority-blending weights. Runtime source resides in `platform/`; inspect its version discrepancies before attempting exact runtime replication. `timing/` includes the fixed-cycle timing evidence, including missed deadlines.

