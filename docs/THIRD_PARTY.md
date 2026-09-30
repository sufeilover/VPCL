# Third-party dependencies and release rights

The local ProjectD-Core LICENSE file states: `THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND FOR NON-PROFIT EDUCATIONAL USE ONLY!` A copy is retained in `projectd_local_license.txt` solely to document the supplied dependency terms. This is not the license for this repository.

The package includes the first-submission VPCL C++ source overlay at `platform/model/projectd-core-develop-src/` and its supplied runtime interface at `platform/bin/PyProjectD.pyd`. It does not include a complete upstream build environment, AC vehicle or track content, ACTI binaries, or other plugin installers. Preserve embedded third-party notices in the source overlay. The upstream project referred to by the manuscript is https://github.com/wongfei/projectd-core. Its current upstream state has not been used to replace the supplied local version.

The original VPCL README and license are archived in `docs/legacy_vpcl/` for provenance, not as a blanket license for this expanded package. Inclusion in the earlier repository does not establish redistribution rights for every dependency. Confirm the terms for the source overlay and compiled interface before public release; no new license has been assigned during this local merge.

Before publication, confirm the rights to each runtime script, helper module and the ScoringSystem.cpp snapshot, retain required notices, and choose a suitable license for author-owned code and data. Do not attach MIT/Apache/CC terms to third-party materials without authority. Public availability and a free download do not themselves grant unrestricted reuse.
