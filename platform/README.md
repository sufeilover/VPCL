# Real-time platform

`ac/` contains telemetry collection and control-return clients. `model/` contains independent model initialization, rollout and candidate-evaluation scripts. `ScoringSystem.cpp` is a supplied scoring implementation snapshot, not a complete model build.

## External installation

1. Install AC and obtain the selected vehicle/track resources under their applicable terms.
2. Install the compatible ACTI and control plugins. Configure the telemetry channels and ports to match the sender/receiver field order.
3. Obtain the matching upstream ProjectD-Core build tree. Copy the contents of `model/projectd-core-develop-src/` into its `src/` directory and rebuild with Visual Studio 2019. The source overlay and `bin/PyProjectD.pyd` (relative to this directory) were imported unchanged from the first-submission VPCL repository. The model scripts expect `platform/bin/` and `platform/content/`; content and additional required DLLs must be supplied separately. Check the Python ABI and architecture before using the supplied binary.
4. Install the Windows Python dependencies. The G29/vJoy pathway additionally requires the driver and suitable device configuration.
5. Check the entry-point configuration and packet mappings before starting a simulator. Do not import the runtime scripts as a smoke test: some initialize DLL paths, loggers or devices at import time.

| Application | AC entry | Model entry |
|---|---|---|
| Reproduction | main_show_paper1_exp1.py | exp1.py / validation variants; exact recording entry requires confirmation |
| Predictive visual feedback | main_show_paper1_exp1andexp2casestudy1.py | exp2_case_study1.py |
| Direct command evaluation | main_show_paper1_exp2casestudy2.py | exp2_case_study2.py |

The helper modules `accardata_to_sim.py`, `opt_vjoy.py`, `G29_vjoy.py` and `unbound/Unbound.lua` were recovered from a separate local VPCL_V2 tree. Their compatibility with the recording scripts has not been runtime-tested. The supplied PyProjectD binary is the model interface, not an ACTI/unbound installer. No vehicle configuration resources or track geometry are included.

Start AC first, then the AC entry in the table, then its matching model entry. AC entries are under `platform/ac/`; model entries are under `platform/model/`. Back up original plugin files before applying the supplied Lua modification. Model C++ changes require rebuilding the interface. The standalone `model/ScoringSystem.cpp` and the imported source tree are separate provenance snapshots; do not assume they were compiled together. See the root README for acknowledgments, installation guidance and licensing scope.
