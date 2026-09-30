# Case 1: predictive visual feedback

Run `python reproduce.py case1` from repository root. The task reads request-level summaries under `data/<AIpush>+ston+86/`, checks the stored flags and regenerates persistence curves, counts and the figure. Prefixes are 80, 160 and 240 model steps. The recorded slip reference is 9.3 degrees and selected persistence is 10 steps.

`max_run_FL/FR/RL/RR` are per-wheel maximum consecutive lengths; crossings of different wheels are not accumulated as one persistent event. `threshold_deg` is a configuration-related warning reference, not a validated universal safety limit. See the OUT implementation concern in the release checklist.

`timing/` contains visual-feedback stage summaries and processed selected rows. In-game screenshots and redistributable track resources are not bundled in this candidate.

