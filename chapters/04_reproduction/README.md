# Section IV: AC-referenced reproduction

Data are aggregate curves and analysis tables from the regenerated 333-Hz, 500-step assessment. Four configurations combine Barcelona/Silverstone1967 with AE86/Supra. AIpush settings are 80, 85, 90, 95 and 100.

Run `python reproduce.py section4` at repository root. `data/raw_curves.csv` stores mean/P95 errors and reference distance; `normalized_curves.csv` stores normalized growth and loss; `lambda_scan.csv` and `exact_lambda_intervals.csv` describe preference sensitivity. `paper_representative_values.csv` and `paper_preference_values.csv` provide tabulated values.

Position and terminal errors: m; speed error: km/h; attitude and tire slip-angle errors: degrees; reference distance: m. Step time is 1/333 s. `scope=pooled` and individual-setting curves are distinct analysis populations. The paper sample set is fixed over horizons; excluded windows are listed in `docs/REPRODUCING.md`.

The original per-rollout/per-horizon input tables and raw AC workbooks are not bundled in this lightweight release. Rebuilding alignment and distribution estimates requires those inputs. The source scripts retain their acquisition-time paths for traceability and are not all portable. Use the root runner for the included-data workflow.

