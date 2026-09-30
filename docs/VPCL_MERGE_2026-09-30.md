# VPCL model and documentation merge

This local update imports 204 source files from the first-submission VPCL `Model/code/projectd-core-develop-src/` directory into `platform/model/projectd-core-develop-src/`, and `Model/bin/PyProjectD.pyd` into `platform/bin/`. File contents are preserved. The original README and LICENSE are archived in `legacy_vpcl/`. No GitHub upload, simulator execution or model compilation is part of this update.

The root English README now includes setup prerequisites, the source-overlay destination (`src/`), rebuilding, binary compatibility, plugin backup instructions, external assets, execution order and acknowledgments. The Chinese overview, platform instructions and dependency notes were synchronized. The `.gitignore` permits the supplied PyProjectD binary while continuing to exclude other generated runtime files.

Existing chapter data and Python experiment scripts were not replaced. Before refreshing the manifest, one pre-existing mismatch was found: `platform/model/exp2_case_study2.py` already used `otf_run10 >= 10`, unlike the four-step condition recorded by the earlier release checklist. Its current contents are retained and included in the refreshed integrity baseline. This is not evidence of which executable or source version generated the recorded experiments. The pre-refresh hash discrepancy is recorded in `vpcl_merge_verification.json`.

The file manifest is an integrity baseline, not a scientific validation or a license grant. The source overlay, compiled interface and revised Python scripts still require a compatible local build and runtime check. The archived VPCL license is retained for provenance and is not applied automatically to the expanded package.
