# Pose Scale Debugging Guide

Use this guide when the Field2d widget shows the robot moving farther or shorter
than its real-world motion, or when vision measurements don't snap the pose to the
correct field position.

---

## Background

The Field2d widget receives `state.Pose` directly from the Phoenix internal pose
estimator via the `Telemetry::Telemeterize()` callback registered in
`Container::configureBindingsInternal()`. There is no secondary WPILib estimator —
what Phoenix computes is exactly what you see on the dashboard.

The two main inputs to that estimator are:

1. **Wheel odometry** — driven by `kWheelRadius` and `kDriveGearRatio` in
   `TunerConstants.h`. A wrong wheel radius produces a **proportional** scale error
   that grows with distance.
2. **Vision corrections** — `AddVisionMeasurement()` calls fused in
   `RobotPeriodic()`. These should snap the pose toward the correct field position
   whenever a tag is visible.

---

## Test 1 — Isolate odometry scale (no vision needed)

**Goal:** Determine whether the scale error comes from the wheel radius / gear ratio.

1. Disable vision fusion temporarily (set `BOT_VISION 0` in `config.hpp`, or just
   watch `DriveState/Pose/x` in NetworkTables rather than the Field2d widget).
2. Place the robot at a known starting point — edge of a field tile works.
3. Zero the heading (heading reset button).
4. Drive the robot **exactly 1 metre** forward in a straight line. Measure with a
   tape measure on the floor.
5. Read `DriveState/Pose/x` from SmartDashboard / NetworkTables.

| Result | Conclusion |
|--------|------------|
| Reads ~1.00 m | Odometry scale is correct — skip to Test 3 |
| Reads < 1.00 m (e.g. 0.95) | Wheel radius in `TunerConstants.h` is **too large** |
| Reads > 1.00 m (e.g. 1.05) | Wheel radius in `TunerConstants.h` is **too small** |

**Current value:** `kWheelRadius = 2_in` (= 0.0508 m, i.e. 4-inch diameter wheel).
Measure your actual wheel diameter with calipers — wheels wear down over a season.

> **Proportional error formula:**
> `actual_radius = 2_in × (measured_pose_distance / real_distance)`

---

## Test 2 — Verify vision snap is firing

**Goal:** Confirm `AddVisionMeasurement()` is actually being called and is
correcting the pose.

1. Re-enable `BOT_VISION`.
2. Enable **Teleop** with at least one AprilTag visible to the FL or BL camera.
3. Watch these SmartDashboard values:
   - `Vision/FL/X (m)` and `Vision/FL/Y (m)` — should show plausible field
     coordinates (not 0.0, not wildly out of bounds).
   - `Vision/Accepted` — counter should be incrementing each cycle a tag is seen.
   - `DriveState/Pose/x` — should **jump** on the first teleop frame where a tag
     is accepted (this is `PoseResetOnce::tryReset()` firing).

| Observation | Conclusion |
|-------------|------------|
| `Vision/Accepted` not incrementing | Camera not connected or tags not detected |
| `Vision/FL/X` shows values but pose doesn't jump | `_poseReset` guard not firing — check `IsReal()` and `IsTeleop()` conditions in `RobotPeriodic()` |
| Pose jumps but to the **wrong** place | Camera transform `kRobotToCamera` in `vision.hpp` is incorrect |
| Pose jumps to the right place but drifts away | Wheel radius error (go to Test 1) or vision std devs too high |

---

## Test 3 — Constant offset vs. proportional drift

Drive 1 m, note pose error. Drive another 1 m, note pose error again.

| Pattern | Conclusion |
|---------|------------|
| Error doubles after 2 m (proportional) | Wheel radius / gear ratio is wrong → re-run Tuner X |
| Error stays the same regardless of distance | Initial pose is wrong, or camera transform offset is wrong |

---

## Known configuration discrepancy

`TunerConstants.h` and `pathplanner/settings.json` specify slightly different
wheel radii:

```
TunerConstants.h          kWheelRadius = 2_in   = 0.05080 m
pathplanner/settings.json driveWheelRadius       = 0.05100 m
```

This 0.4 mm difference is minor but means PathPlanner's feedforward model is
slightly inconsistent with what Phoenix is actually commanding. If you update
`kWheelRadius` after measuring the real wheel, update `settings.json` to match
(via the PathPlanner GUI or directly in the file).

---

## Re-running Tuner X

If Test 1 shows a scale error, regenerate `TunerConstants.h` via Tuner X:

1. Open Tuner X → Swerve Project Generator.
2. Re-measure the wheel diameter with calipers and enter the correct value.
3. Re-generate and replace `src/generated/TunerConstants.h` and
   `src/generated/TunerConstants.cpp`.
4. Update `pathplanner/settings.json` → `driveWheelRadius` to match.
5. Re-deploy and repeat Test 1 to confirm.

---

## Vision-only: camera transform verification

If odometry scale is correct (Test 1 passes) but vision snaps the pose to the
wrong location, the camera mounting transforms in `vision.hpp` are suspect:

```cpp
// vision.hpp — kRobotToCamera
frc::Transform3d{
    frc::Translation3d{13.74_in, 11.5_in, 15_in},   // FL
    frc::Rotation3d{0_deg, 0_deg, 0_deg}
},
frc::Transform3d{
    frc::Translation3d{-13.74_in, 2.24_in, 11.35_in}, // BL
    frc::Rotation3d{0_deg, -10_deg, 180_deg}
}
```

Verify these with a tape measure from the robot centre to each camera lens.
X = forward, Y = left, Z = up (WPILib robot frame convention).
