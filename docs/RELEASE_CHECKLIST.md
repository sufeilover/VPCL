# Before public release

- Confirm licenses for author-owned code and processed data. ProjectD-Core's supplied license is restricted to non-profit educational use.
- Confirm the exact runtime files used for the recorded experiments. At the 2026-09-30 merge, the existing model Case 2 file already contained `prefer_brake_by_run10 = (otf_run10 >= 10)`, whereas the earlier package audit recorded four steps. This pre-existing edit was preserved, not made by the merge. Matching the paper's ten-step description does not identify the historical recording version.
- Confirm the implementation and provenance of OUT: the manuscript uses any-tire boundary crossing, while earlier code audits identified a body-reference-point/width criterion.
- Check compatibility of AC helper modules recovered from VPCL_V2 with the new-similar entry points and the installed plugins.
- Verify or remove remaining installation-specific paths in runtime and archival analysis source before advertising cross-machine runtime support.
- Specify exact AC/plugin/PyProjectD versions and build steps. No simulator or hardware-in-the-loop execution was performed during packaging.
- Check processed-data and paper-figure correspondence after final manuscript edits. The package deliberately excludes old score-based and straight/corner analyses as primary reproduction entry points.
- The anonymous human summary export removes source paths, raw file hashes and filename timestamps while preserving metric values and within-condition run order. It does not claim that rich driving data have zero re-identification risk.
- Choose a GitHub account and finalize this candidate locally before creating a public remote. No remote is configured by this package.
