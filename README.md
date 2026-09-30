This local release candidate organizes the platform and paper evidence by chapter and case study. The platform connects ongoing Assetto Corsa (AC) execution to telemetry-based initialization of a separate ProjectD-Core model and to prediction-informed application feedback.

## Start here

Python 3.10 or later is required for the offline scripts. From this directory:

```shell
python -m pip install -r requirements-analysis.txt
python tools/verify_package.py
python reproduce.py all
```

Offline outputs are written to `outputs/`; rerunning overwrites those outputs, not the packaged data. No simulator, network connection or steering device is started by this command. See [reproduction instructions](docs/REPRODUCING.md).

## Paper organization

| Paper content | Directory | Included material |
|---|---|---|
| I-II: motivation and related work | docs/ | Platform scope and dependency documentation |
| III: architecture | platform/ | AC interfaces, packet mapping, model-side experiment entry points |
| IV: reproduction assessment | chapters/04_reproduction/ | Five metrics, 1-500-step curves, normalization and preference sensitivity |
| IV: AI driving conditions | chapters/04_aipush/ | Run and condition summaries, preprocessing code |
| IV: processing time | chapters/04_timing/ | Four-configuration timing summaries and selected records |
| V-A: visual feedback | chapters/05_case1_visual/ | Persistence summaries, analysis and timing |
| V-B: control-oriented assistance | chapters/05_case2_control/ | Whole-circuit run summaries, comparison and timing |
| V-C: human-in-the-loop | chapters/05_case3_human/ | Anonymous run summaries and preprocessing code |
| VI: interpretation | docs/ | Reproduction scope and known implementation differences |

## What can be reproduced

The portable entry point redraws the Section IV metric/preference figures, rebuilds Case 1 persistence evidence, recomputes Case 2 group comparisons, and summarizes the 30 anonymous human runs. It uses packaged processed data, not the original high-volume telemetry workbooks. The original analysis scripts are retained under `source_scripts/` for provenance; some require external raw data or local installation settings and are not the portable entry points.

## Runtime and release status

The real-time platform requires Windows, a licensed AC installation, telemetry/control plugins, a compatible PyProjectD build and separately supplied vehicle/track resources. See [platform setup](platform/README.md). The current runtime files are candidate source snapshots; the recording-version and trigger discrepancies in [release checks](docs/RELEASE_CHECKLIST.md) must be resolved before claiming exact runtime replication.

## Model source and runtime interface

The model source and runtime interface from the first-submission VPCL repository are now included alongside the revised analysis package:

| Original VPCL directory | Directory in this package | Purpose |
|---|---|---|
| `Model/code/projectd-core-develop-src/` | `platform/model/projectd-core-develop-src/` | C++ source overlay, including ProjectD, PyProjectD and PlaygrounD |
| `Model/bin/` | `platform/bin/` | Supplied `PyProjectD.pyd` runtime interface |

These files are preserved unchanged. The overlay is not a complete standalone build tree: obtain the matching [ProjectD-Core project](https://github.com/wongfei/projectd-core), retain its dependencies and build files, then copy the overlay contents into its **`src/` directory**, not its root. For example, `ProjectD/Car/Car.cpp` belongs at `src/ProjectD/Car/Car.cpp`. Back up the upstream files first. Rebuild after changing C++ code; otherwise Python continues to use the old compiled implementation.

The original setup used Windows, Visual Studio 2019 and Python 3.10. The supplied `.pyd` must match the Python ABI, architecture and dependent DLLs on the target machine; its compatibility with the revised experiment scripts has not been runtime-tested here. If incompatible, build locally and put the resulting interface and required runtime DLLs in `platform/bin/`. Source and binary were copied from the same repository, but a source-to-binary build match has not been established.

## Running the real-time platform

1. Install Assetto Corsa and obtain vehicle/track resources legally. These assets are not included.
2. Install the original ACTI and unbound/control-plugin dependencies separately. Back up plugin files before applying corresponding modified files. This package supplies `platform/ac/unbound/Unbound.lua`; it does not include a complete ACTI installer or a modified ACTI directory.
3. Prepare ProjectD-Core and its runtime interface as described above. Supply the model content under `platform/content/`, following the folder names expected by the model and the selected entry point.
4. Install `requirements-runtime-windows.txt` in a suitable Windows environment. Configure vJoy and the steering device when using that input pathway.
5. Check vehicle/track identifiers, local paths, telemetry packet layouts, ports, trigger settings and model API compatibility before running. See [platform setup](platform/README.md) for the paired script names.
6. Start AC, start the selected AC-side script under `platform/ac/`, and then start its matching model-side script under `platform/model/`.

The offline `reproduce.py` workflow is separate: it does not load `PyProjectD.pyd`, start AC or control a device. Adding the original model files does not replace the newer Python entry points or the chapter-organized processed data.

## Acknowledgments

We thank the developers and maintainers of [ProjectD-Core](https://github.com/wongfei/projectd-core), ACTI, Custom Shaders Patch and unbound for the software and interfaces used in this research. We also acknowledge Assetto Corsa and the third-party libraries on which the model and its tools depend. Their authors retain ownership of their work; applicable notices and license terms must be preserved.

The first-submission README and license are retained in [docs/legacy_vpcl/](docs/legacy_vpcl/) as historical documentation. Its old paths, experiment descriptions and statements about excluded files are superseded by this README for this package. The archived license is not automatically applied to all newly packaged data or third-party code.

No GitHub remote has been created. No blanket open-source license is applied to this package yet: the supplied local ProjectD-Core license restricts use to non-profit education. See [third-party and licensing notes](docs/THIRD_PARTY.md). Code ownership and the license for the authors' code/data need confirmation before public release.

Human data are provided as run-level summaries with anonymous participant IDs. The author confirmed authorization to share anonymous data; raw workbooks and original filenames are not included. See [human data](chapters/05_case3_human/README.md).
