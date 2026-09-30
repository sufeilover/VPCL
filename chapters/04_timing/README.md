# Section IV: prediction timing

Each data subdirectory corresponds to one vehicle/track configuration. `stage_summary.csv` contains sample counts, missing-value counts and latency statistics in milliseconds. `selected_rows.csv` preserves the selected processed timing records.

Model processing and client send-to-receive duration are different intervals. Neither is the model's 500-step prediction duration, nor a measurement ending at physical actuation. Missing optional stages are unavailable rather than zero.

`source_scripts/` retains each configuration's analyzer. Reprocessing needs its raw client/model logs, not included here. The package does not substitute old Experiment 1 or 2 timing for this assessment.

