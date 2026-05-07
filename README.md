# Virtual–Physical Closed-Loop Shared Driving Platform

This repository provides the source code, modified experiment files, and representative experimental data for a virtual–physical closed-loop shared driving platform based on Assetto Corsa (AC) and `projectd-core`.

The platform is used for:

- AC-side real-time vehicle-state acquisition;
- model-side hot-start synchronization;
- short-horizon vehicle-state prediction;
- shared-driving control experiments;
- finite-strategy dual-preference / approximate-Nash analysis.

> This repository is intended as an academic reproducibility package. It does **not** redistribute third-party commercial assets, original vehicle / track assets, or original third-party plugin packages.

---

## 1. External Model Source

The vehicle physics model is based on the open-source `projectd-core` project.

Please download the original model source code from:

```text
https://github.com/wongfei/projectd-core
```

After downloading the source code, please build the project using **Visual Studio 2019**.

Recommended project root:

```text
projectd-core-develop/
```

---

## 2. Important Notice on Third-Party Files

To avoid redistributing third-party or potentially copyrighted files, this repository does **not** include the full original copies of:

- Assetto Corsa vehicle assets;
- Assetto Corsa track assets;
- original ACTI plugin package;
- original unbound plugin package;
- original `projectd-core` source repository;
- commercial game content, models, textures, audio, or other proprietary resources.

Users should obtain all third-party software, plugins, vehicle files, and track files from their own legal sources.

This repository only provides files required to reproduce the research workflow, such as:

- user-modified source files;
- AC-side Python scripts;
- model-side Python scripts;
- experiment scripts;
- representative experimental data;
- setup instructions.

---

## 3. Repository Folder Description

The recommended repository structure is:

```text
AC/
├─ code/
├─ ACTI_modified_files/
└─ unbound_modified_files/

model/
└─ code/

bin/
└─ [optional runtime files, if provided]

data/
├─ Section 4 Data/
└─ Section 5 Case Studty2 Data/

README.md
LICENSE
```

Folder descriptions:

- `AC/code/`: AC-side Python scripts.
- `AC/ACTI_modified_files/`: modified ACTI-related files only. The original ACTI plugin should be installed separately.
- `AC/unbound_modified_files/`: modified unbound-related files only. The original unbound plugin should be installed separately.
- `model/code/`: modified model-side C++ files and Python experiment scripts.
- `bin/`: optional runtime files, such as `PyProjectD.pyd`, if provided for convenience.
- `data/Section 4 Data/`: representative data used for Section / Chapter 4.
- `data/Section 5 Case Studty2 Data/`: representative data used for Section / Chapter 5 Case Study 2.

> Note: The folder name `Section 5 Case Studty2 Data` is kept as originally used in the experiment files. If this folder is renamed later, please update the README and any related scripts accordingly.

---

## 4. Required Software and Plugins

Before running the experiments, please prepare the following software and plugins:

1. Assetto Corsa.
2. `projectd-core`.
3. Visual Studio 2019.
4. Python environment compatible with `PyProjectD.pyd`.
5. ACTI plugin.
6. unbound plugin.
7. Required Assetto Corsa / projectd-core track and vehicle files prepared from legal sources.

The ACTI and unbound plugins should be installed from their original sources first. After installation, replace only the corresponding files using the modified files provided in this repository.

It is recommended to back up the original plugin files before replacement.

---

## 5. Model-Side File Replacement

The modified model-side files are located in:

```text
model/code/
```

Copy the files inside `model/code/` into their corresponding paths in the downloaded `projectd-core-develop` source tree and replace the original source files.

The replacement should follow the same file names and directory structure as the original `projectd-core` project.

After replacing the files, please **rebuild the project in Visual Studio 2019**.

This rebuild step is required. Otherwise, the modified C++ files will not take effect.

---

## 6. Runtime Binary File

If a runtime binary is provided, it will typically be placed under:

```text
bin/
```

For example:

```text
PyProjectD.pyd
```

If `PyProjectD.pyd` is not included, please build it locally from the modified `projectd-core` source code.

Please ensure that `PyProjectD.pyd` matches:

- your Python version;
- your system architecture;
- your local build configuration.

If import errors occur, rebuild `projectd-core` locally and use the locally generated `PyProjectD.pyd`.

---

## 7. Track and Vehicle Assets

This repository does **not** include track or vehicle asset folders.

To reproduce the experiments, please prepare the required track and vehicle content from your own legal Assetto Corsa / projectd-core installation and place them under the corresponding local `projectd-core` content directories.

Typical local paths are:

```text
projectd-core-develop/content/tracks/
projectd-core-develop/content/car/
```

If your local `projectd-core` directory uses a different vehicle-content folder, such as `content/cars/`, please follow your local project structure.

---

## 8. Experiment Source Code

### 8.1 AC-Side Code

The AC-side Python scripts are located in:

```text
AC/code/
```

These scripts should be run first. They are responsible for:

- reading AC-side vehicle data;
- sending real-time vehicle states;
- communicating with the model-side prediction / control process;
- supporting the virtual–physical closed-loop experiments.

### 8.2 Model-Side Code

The model-side Python experiment scripts should be placed under the corresponding `pyprojectd` folder after the replacement step, for example:

```text
projectd-core-develop/pyprojectd/
```

Main experiment scripts:

```text
exp1.py
exp2_case_study1.py
exp2_case_study2.py
```

Their usage:

- `exp1.py`: source code for Chapter / Section 4.
- `exp2_case_study1.py`: source code for Chapter / Section 5 Case Study 1.
- `exp2_case_study2.py`: source code for Chapter / Section 5 Case Study 2.

---

## 9. Experimental Data

Representative experimental data are provided for result verification.

### 9.1 Section / Chapter 4 Data

The data for Section / Chapter 4 are stored in:

```text
data/Section 4 Data/
```

This folder contains the data used for the Chapter / Section 4 experiments, including model-side prediction / validation outputs and related processed results.

### 9.2 Section / Chapter 5 Case Study 2 Data

The data for Section / Chapter 5 Case Study 2 are stored in:

```text
data/Section 5 Case Studty2 Data/
```

This folder contains the data used for Chapter / Section 5 Case Study 2, including shared-driving / strategy-evaluation outputs and related processed results.

### 9.3 Data Usage Notes

- The included data are representative experimental data, not a complete dump of all intermediate debugging files.
- These data are provided to support result verification and reproducibility.
- If new data folders are added, please keep their names consistent with the corresponding manuscript sections or case studies.
- When reproducing figures or tables, check the relevant section data folder first.

---

## 10. Recommended Setup Workflow

The recommended setup workflow is:

1. Download `projectd-core` from the official GitHub repository:

   ```text
   https://github.com/wongfei/projectd-core
   ```

2. Open and build `projectd-core` using **Visual Studio 2019**.

3. Install ACTI and unbound plugins in Assetto Corsa from their original sources.

4. Replace the corresponding ACTI and unbound plugin files using the modified files provided in:

   ```text
   AC/ACTI_modified_files/
   AC/unbound_modified_files/
   ```

5. Copy the modified model-side files from:

   ```text
   model/code/
   ```

   into their corresponding paths in:

   ```text
   projectd-core-develop/
   ```

6. Prepare the required track and vehicle content from your own legal installation and place them under the corresponding local `projectd-core` content directories.

7. Rebuild the modified project again using **Visual Studio 2019**.

8. If a compatible `PyProjectD.pyd` is generated or provided, place it in the corresponding runtime / build output directory.

9. Start Assetto Corsa.

10. Run the required AC-side Python script from:

   ```text
   AC/code/
   ```

11. Run the corresponding model-side Python script from:

   ```text
   projectd-core-develop/pyprojectd/
   ```

---

## 11. Running Order

The running order is important.

Please follow this order:

```text
Step 1: Start Assetto Corsa.
Step 2: Run the required AC-side Python script from AC/code/.
Step 3: Run the corresponding model-side Python script from projectd-core-develop/pyprojectd/.
Step 4: Start the target experiment or case study.
```

Examples:

### Chapter / Section 4

```text
Run AC-side script from AC/code/.
Run model-side script: projectd-core-develop/pyprojectd/exp1.py.
Use data folder: data/Section 4 Data/.
```

### Chapter / Section 5 Case Study 1

```text
Run AC-side script from AC/code/.
Run model-side script: projectd-core-develop/pyprojectd/exp2_case_study1.py.
```

### Chapter / Section 5 Case Study 2

```text
Run AC-side script from AC/code/.
Run model-side script: projectd-core-develop/pyprojectd/exp2_case_study2.py.
Use data folder: data/Section 5 Case Studty2 Data/.
```

---

## 12. Suggested Local Directory Structure

After setup, the local `projectd-core` directory may look like:

```text
projectd-core-develop/
├─ bin/
│  └─ PyProjectD.pyd
│
├─ content/
│  ├─ tracks/
│  │  └─ [required track folder]
│  └─ car/
│     └─ [required vehicle folder]
│
├─ pyprojectd/
│  ├─ exp1.py
│  ├─ exp2_case_study1.py
│  └─ exp2_case_study2.py
│
├─ [projectd-core source files]
└─ [modified model-side source files]
```

The repository-side files may be organized as:

```text
AC/
├─ code/
├─ ACTI_modified_files/
└─ unbound_modified_files/

model/
└─ code/

bin/
└─ [optional runtime files]

data/
├─ Section 4 Data/
└─ Section 5 Case Studty2 Data/

README.md
LICENSE
```

---

## 13. Notes

- The project was developed and tested under a Windows environment.
- Visual Studio 2019 is recommended for building the model source code.
- The modified model-side files must be copied into the correct locations before rebuilding.
- The project must be rebuilt after replacing the model-side C++ files.
- Required track and vehicle files are not included in this repository and should be obtained from legal sources.
- Python version compatibility should be checked carefully, especially for `PyProjectD.pyd`.
- The ACTI and unbound plugin files should be backed up before replacement.
- The AC-side Python script should be started before the corresponding model-side Python script.
- The provided scripts and data are intended to reproduce the experimental platform and case studies described in the associated research work.
- The included data folders are provided for result verification and should be kept with their corresponding manuscript sections.

---

## 14. License and Usage

This repository and the provided modified files are released for educational and non-commercial research purposes only.

The files may be used for:

- academic study;
- non-commercial research;
- reproduction of the associated experiments;
- educational demonstrations.

Commercial use, redistribution for profit, or integration into commercial simulation products is not permitted without explicit permission from the author.

Please also follow the license terms of the original `projectd-core` project and any third-party plugins, libraries, vehicle files, track files, and Assetto Corsa resources used with this platform.

Third-party assets and plugins remain the property of their respective owners and are not redistributed as part of this repository.
