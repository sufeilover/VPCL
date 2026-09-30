# Processed-data conventions

CSV files retain original numerical precision. Empty numeric fields mean unavailable, not zero. The machine-readable `data_schemas.json` lists headers for each packaged CSV.

- `config` / `configuration`: vehicle-track combination; `setting` / `AIpush`: AI control setting.
- `step`: prediction integration steps, not milliseconds. Frequency is 333 Hz for the updated reproduction analysis.
- `mean_` / `p95_`: empirical aggregation of the named metric; retain its analysis scope.
- `pos3d_rmse_m`: position RMSE; `pos3d_finalae_m`: terminal position error; `speedKmh_rmse`: speed RMSE; `attitude_mean_mae_deg`: attitude MAE; `SlipAngle_mean_rmse`: four-wheel average slip-angle RMSE.
- `distance_m`: AC reference travel distance; `benefit`: normalized distance; `grouped`: grouped normalized loss. Normalization and lambda encode analysis preferences.
- `n` / `count`: count at the table's stated aggregation level. `sample_sd`: between-run sample standard deviation where specified.
- `*_ms`: milliseconds. `*_s`: seconds. `speedKmh_*`: km/h. Native steering commands must not be relabeled as degrees.
- Human `*_time_mean` / `*_time_rmse`: time-weighted within-run quantities; `*_p95`: within-run sample percentile. Participant summaries average three run-level summaries equally.
- Runtime keys containing `hotstart` or `shared` are retained historical API/log names; manuscript terms are state initialization and optimizer-assisted direct command application.

Absolute source paths in tabular metadata are replaced by `external/<basename>` in the release copy. Measurement cells are not recomputed in this step. The author's private source-to-package mapping is kept outside the repository.
