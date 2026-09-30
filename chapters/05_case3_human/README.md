# Case 3: human-in-the-loop assistance

The author confirmed authorization to share anonymous participant data. `data/run_summary.csv` includes all 30 run summaries: five participants, human-only and optimizer-assisted conditions, three runs in each condition. Run IDs use P01-P05 plus condition and within-condition order, with original paths, filename timestamps and source-workbook hashes removed. Numeric metrics are unchanged by the export.

Run `python reproduce.py human` to regenerate participant/condition averages for mean speed, mean distance and distance P95. This is a new packaging helper implementing the existing equal-run-weight summary, not a new experiment.

Human distance is the shortest XZ distance to a continuous closed reference polyline. Time-weighted means retain the recorded intersample durations; run P95 is sample-based. These definitions differ from Case 2's nearest-sample distance and should not be pooled across populations. Logged steering represents vehicle-side commands, not a separately measured raw human input.

Formal recordings followed human-only then assisted conditions, after familiarization. Informal participant feedback is subjective and is not present as a standardized questionnaire dataset. Raw traces and workbook filenames are not included in this lightweight release.

`source_scripts/preprocess_human_driving.py` is retained for processing new authorized data with a supplied track-reference CSV. No track-reference geometry is redistributed in this package.

