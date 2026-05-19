# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

**First-time build** (builds Thirdparty libs + main library):
```bash
./build.sh
```

**Incremental rebuild** (after modifying `src/` or `include/`):
```bash
cd build && make -j4
```

**ROS nodes** (optional):
```bash
./build_ros.sh
```

The build produces `lib/libORB_SLAM3.so` and executables under `Examples/`.

## Running Examples

All executables take `Vocabulary/ORBvoc.txt` as first argument and a YAML config as second.

**Stereo-Inertial on EuRoC:**
```bash
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml \
    /path/to/EuRoC/MH_01_easy ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo_inertial
```

**Stereo on EuRoC:**
```bash
./Examples/Stereo/stereo_euroc \
    Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml \
    /path/to/EuRoC/MH_01_easy ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo
```

Dataset paths on this machine:
- EuRoC: `/media/lab405/Windows1/data/Euroc`
- TUM-VI: `/media/lab405/Windows1/data/TUM`

## Evaluation / Benchmarking

The unified benchmark script runs all sensor configurations and computes APE RMSE via `evo_ape`:
```bash
cd evaluation

# All algorithms and datasets
python3 run_stereo_inertial_benchmark.py

# Filter by algorithm or dataset
python3 run_stereo_inertial_benchmark.py --algo stereo --dataset euroc
python3 run_stereo_inertial_benchmark.py --algo stereo_inertial
python3 run_stereo_inertial_benchmark.py --algo mono_inertial --dataset tum
```

Results are saved to `evaluation/benchmark_results_<timestamp>.csv`. The script:
1. Runs the binary
2. Converts timestamps ns→s via `fix_time.py`
3. Computes APE RMSE with `evo_ape`

`evaluation/ate_3d.py` is a 3D ATE evaluation script.

## Enabling Timing Profiling

Uncomment `#define REGISTER_TIMES` in `include/Config.h` (note: currently the file is a stub — the actual flag was used in the old code path), then rebuild. Stats are printed to terminal and saved to `ExecTimeMean.txt`.

## Architecture

ORB-SLAM3 is a **multi-threaded SLAM system** built around three parallel threads:

### Core Threads
- **Tracking** (`src/Tracking.cc`): Main thread. Receives each frame, extracts ORB features, estimates camera pose via motion model or relocalization, and decides when to insert KeyFrames.
- **LocalMapping** (`src/LocalMapping.cc`): Processes new KeyFrames — creates new MapPoints via triangulation, culls redundant KeyFrames, and runs Local BA (via `Optimizer`).
- **LoopClosing** (`src/LoopClosing.cc`): Detects loop closures using DBoW2 place recognition, computes Sim3 correction, runs Essential Graph optimization, and triggers full Bundle Adjustment.

### Key Data Structures
- **Atlas** (`src/Atlas.cc`): Container for multiple Maps. Enables multi-map SLAM — when tracking is lost, a new Map is created; loop closure can merge maps.
- **Map** / **MapPoint** / **KeyFrame**: The 3D map representation. MapPoints store 3D position and observations from multiple KeyFrames.
- **Frame**: A single image capture with extracted ORB features. Short-lived; converted to KeyFrame when selected.

### Optimization (`src/Optimizer.cc`)
All non-linear optimization uses **g2o** (included in `Thirdparty/g2o`). Key functions:
- `PoseOptimization` — single-frame pose-only BA
- `LocalBundleAdjustment` — local map BA triggered by LocalMapping; the `num_iters` parameter has been added to this fork for monitoring
- `OptimizeEssentialGraph` — loop closure correction
- `GlobalBundleAdjustemnt` — full map BA after loop closure

### IMU Integration
`src/ImuTypes.cc` and `include/ImuTypes.h` define IMU preintegration. `src/G2oTypes.cc` defines the g2o edge/vertex types for visual-inertial optimization. IMU initialization happens in Tracking and is refined in LocalMapping.

### Camera Models
`include/CameraModels/`: `Pinhole` (standard) and `KannalaBrandt8` (fisheye). Both inherit from `GeometricCamera`. The active model is selected based on the YAML config `Camera.type` field.

### Settings / Config
`src/Settings.cc` parses YAML config files (OpenCV `FileStorage`). Each YAML in `Examples/*/` corresponds to a specific sensor+dataset combination. IMU noise parameters and camera intrinsics/extrinsics live there.

### Vocabulary
`Vocabulary/ORBvoc.txt` is the prebuilt DBoW2 vocabulary (decompressed from `ORBvoc.txt.tar.gz` by `build.sh`). Required at runtime — do not delete.

## Modified Files in This Fork

This fork adds monitoring/logging capabilities:
- `src/Tracking.cc` — instrumentation additions
- `src/Optimizer.cc` — `num_iters` parameter for `LocalBundleAdjustment`, monitoring output
- `Examples/Stereo-Inertial/TUM-VI_far.yaml` — tuned config for far-range TUM-VI
- `Examples/Stereo/EuRoC.yaml` — tuned stereo config

CSV logs (`localmapping_log.csv`, `tracking_log.csv`) are written during runs for analysis. `map_points_total` = current map point count; `new_map_points` = points attempted in a given frame.
