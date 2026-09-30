# Local packaging validation

- The four offline tasks completed using bundled processed inputs.
- Case 2 comparison and full-track unit tests: six tests passed.
- Regenerated Case 2 group statistics and differences match the packaged references within numerical tolerance.
- Anonymous human participant summaries match the revised manuscript table at the reported precision. The continuous-polyline distance fields are used, not the nearest-sampled-point fields.
- Grouped-P95 lambda=1 selections match 261, 248, 256 and 253 model steps.
- Python source syntax and packaged-file hashes are checked by `tools/verify_package.py`.

To repeat numerical checks after `python reproduce.py all`, run `python tools/check_reproduced_results.py`. Run original Case 2 tests from its `source_scripts` directory with `python -m unittest -v test_ai_assisted_comparison test_full_track_pipeline`.

The tested dependency versions are recorded in `tested_environment.json`. Optional missing libraries are stated rather than silently recorded as installed. No AC launch, plugin installation, PyProjectD build, hardware or network-control test was performed. Tests do not resolve runtime-version or redistribution questions in the release checklist.
