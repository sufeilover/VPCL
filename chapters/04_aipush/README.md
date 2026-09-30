# Section IV: AIpush and driving conditions

`data/` contains selected run-level and condition-level metrics, channel coverage and spatial-repeatability summaries. These are AI recordings, separate from the AC/model rollouts and from Case 2 assisted recordings.

There are two tracks, two vehicles, five AIpush settings and five repeated runs per condition. Condition summaries use run-level means and sample standard deviations. Unit and channel columns in the long-format tables identify the native measurement scale; normalized steering is not a steering-wheel angle in degrees.

`source_scripts/preprocess_ai_driving_characteristics.py` performs original preprocessing; `analyze_processed_results.py` consumes its larger processed output package, which is not fully included. The compact data here support inspecting the published driving-condition summaries without opening the raw workbooks.

