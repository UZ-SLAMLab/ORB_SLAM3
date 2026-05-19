# ORB-SLAM3 Trajectory Study Index

## Purpose

This index helps you read the two trajectory notes in a structured order:

- `trajectory_pure_visual.md`
- `trajectory_visual_inertial.md`

---

## Recommended Reading Order

1. Read this file first (global map of the code paths).
2. Read `trajectory_pure_visual.md` to lock down the baseline front-end and export logic.
3. Read `trajectory_visual_inertial.md` to understand what changes after IMU is introduced.
4. Return to `src/Tracking.cc` and trace one frame end-to-end.

---

## Big Picture (One Frame)

```text
Sensor Input
  -> Tracking::Track()
      -> initial pose guess
         - visual: RefKF / MotionModel
         - VI: PredictStateIMU (when initialized and outside reset window)
      -> TrackLocalMap() pose refinement
         - visual: PoseOptimization
         - VI: PoseOptimization or PoseInertialOptimization* (depends on state)
      -> store relative pose to reference KF
         - mlRelativeFramePoses / mlpReferences / mlFrameTimes / mlbLost
  -> System::SaveTrajectory*()
      -> reconstruct absolute trajectory from reference KFs
      -> output Twc (visual) or Twb (VI branch in EuRoC export)
```

---

## Side-by-Side Comparison

| Topic | Pure Visual | Visual-Inertial |
|---|---|---|
| Initial pose guess | RefKF / MotionModel | Can switch to `PredictStateIMU()` |
| Local pose refinement | `PoseOptimization` | `PoseOptimization` or `PoseInertialOptimizationLastFrame/LastKeyFrame` |
| State variables in optimization | pose only | pose + velocity + gyro bias + acc bias |
| Trajectory container | same 4 lists in `Tracking` | same 4 lists in `Tracking` |
| EuRoC export pose type | `Twc` branch | `Twb` branch |

---

## Code Entry Points You Should Revisit

- `src/Tracking.cc`
  - `Track()`
  - `TrackReferenceKeyFrame()`
  - `TrackWithMotionModel()`
  - `PredictStateIMU()`
  - `TrackLocalMap()`
  - `UpdateFrameIMU()`

- `src/Optimizer.cc`
  - `PoseOptimization()`
  - `PoseInertialOptimizationLastKeyFrame()`
  - `PoseInertialOptimizationLastFrame()`

- `src/System.cc`
  - `SaveTrajectoryTUM()`
  - `SaveTrajectoryEuRoC()`

- `src/LocalMapping.cc`
  - `InitializeIMU()`
  - `LocalInertialBA` / `LocalBundleAdjustment` branch

---

## Practical Validation Workflow

1. Run one pure visual sequence and one VI sequence.
2. Inspect `tracking_log.csv`:
   - `tracking_method`
   - `imu_initialized`
   - (optionally re-enable) `opt_type`, `imu_predicted`
3. Compare generated trajectory files and confirm expected branch behavior.

---

## Suggested Next Study Step

Create a small per-frame tracer table (frame_id -> tracking_method -> opt_type -> output pose type) for one short sequence. That will make branch switching behavior obvious.
