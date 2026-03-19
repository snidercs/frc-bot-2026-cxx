# Plan: Kalman Filter Tuning for Pose Estimation

## Background

The CTRE `SwerveDrivetrain` (which `CommandSwerveDrivetrain` extends via
`TunerSwerveDrivetrain`) contains an internal **Unscented Kalman Filter (UKF)**
that fuses wheel odometry, gyro heading, and vision measurements into a single
best-estimate robot pose.

Two noise matrices feed the filter:

| Matrix | Meaning | Where set |
|--------|---------|-----------|
| `odometryStandardDeviation` [x, y, θ] | How much to trust wheel odometry each cycle ($Q$ process noise) | `TunerConstants::CreateDrivetrain()` constructor |
| `visionMeasurementStdDevs` [x, y, θ] | How much to trust each individual vision measurement ($R$ measurement noise) | `drive.AddVisionMeasurement(pose, ts, stdDevs)` call site |

Currently `CreateDrivetrain()` uses the **default constructor** (no std-dev
arguments), so CTRE's built-in defaults are in effect. The per-measurement
`stdDevs` are computed dynamically by `Processor::computeStdDevs()`.

---

## Current State

### Odometry noise (process noise $Q$)
`TunerConstants.cpp` calls:
```cpp
return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
```
No explicit odometry std-dev array is passed → CTRE defaults apply (typically
`{0.1, 0.1, 0.1}`). These have **never been tuned** for this robot.

### Vision noise ($R$)
`Processor::computeStdDevs()` in `vision.cpp` returns:
```cpp
double xy = 0.2 + (distanceMeters * 0.07);   // ~0.2 m at 0 m, ~0.48 m at 4 m
// halved for >2 tags: xy *= 0.75
double theta = 9999.0;                         // never let vision override gyro
```
These were chosen conservatively and have not been validated against real match
data.

---

## What Needs to Be Done

### Step 1 — Pass explicit odometry std-devs to the drivetrain constructor

**File:** `src/generated/TunerConstants.cpp`

Switch from the default constructor to the 3-argument overload so we control
the odometry noise matrix explicitly:

```cpp
indy::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {
        DrivetrainConstants,
        250_Hz,                          // odometry update frequency (CAN FD)
        {0.1, 0.1, 0.01},               // odometry std-devs [x(m), y(m), θ(rad)]
        {0.9, 0.9, 9999.0},             // default vision std-devs (overridden per-call)
        FrontLeft, FrontRight, BackLeft, BackRight
    };
}
```

**Starting values to tune:**
- `odometry [x, y]` — `0.1 m` is a reasonable baseline for swerve with
  good wheel odometry. Increase if odometry drifts badly under defence or
  carpet scrub. Decrease to trust odometry more.
- `odometry [θ]` — `0.01 rad` (very tight) since the Pigeon 2 gyro is
  excellent. Widen only if yaw drift is observed.
- `vision [x, y]` (the constructor default) — `0.9 m` is deliberately loose
  because the real per-measurement values are supplied in `AddVisionMeasurement`
  calls; this default is only a fallback.
- `vision [θ]` — `9999.0` always, to prevent vision from overriding the gyro.

---

### Step 2 — Validate and tune `computeStdDevs()` in `vision.cpp`

The formula is:
```cpp
double xy = 0.2 + (distanceMeters * 0.07);
if (tagCount > 2) xy *= 0.75;
```

**Suggested tuning procedure:**

1. **Enable `BOT_TRACE_VISION`** in a test build. Log `Vision/<cam>/Distance (m)`,
   `Vision/<cam>/Tags`, and the accepted pose X/Y to Shuffleboard or `.hoot`.
2. Drive a known path on the field. Compare the fused pose to ground truth
   (tape-measure checkpoints).
3. If the fused pose over-corrects toward noisy vision measurements, **increase**
   the base offset (`0.2`) or slope (`0.07`).
4. If the fused pose is slow to converge after a pose reset, **decrease** them.
5. Multi-tag reward (`* 0.75`) is reasonable; consider tightening to `* 0.6`
   if 3-tag solves are consistently accurate.

**Candidate tuned formula (starting point):**
```cpp
double xy = 0.3 + (distanceMeters * 0.05);   // slightly more conservative base
if (tagCount >= 3) xy *= 0.65;               // stronger reward for 3+ tags
else if (tagCount == 2) xy *= 0.85;          // modest reward for 2-tag solve
```

---

### Step 3 — Enable `BOT_ADAPTIVE_STDDEVS` and validate

`BOT_ADAPTIVE_STDDEVS` (currently a compile-time flag) adds a rolling-variance
scale factor to `computeStdDevs()`:

```cpp
static constexpr double kVarianceThreshold = 0.05;   // m²
static constexpr double kMaxScale = 3.0;
const double scale = std::clamp(variance / kVarianceThreshold, 1.0, kMaxScale);
xy *= scale;
```

This automatically inflates stdDevs when recent vision poses are inconsistent —
effectively reducing vision trust when measurements are noisy.

**To enable:** add `-DBOT_ADAPTIVE_STDDEVS=1` to `build.gradle` or the
`compile_commands.json` flags.

**Tuning knobs:**
- `kVarianceThreshold` — the m² spread at which scaling begins. `0.05 m²`
  ≈ `0.22 m` positional spread. Tighten if you want inflation to start sooner.
- `kMaxScale` — caps inflation at `3×` the base stdDev. Increase for more
  aggressive suppression during noisy periods.
- `CameraAnchor::kWindowSize` (default 6 frames at 50 Hz = 120 ms) — increase
  to smooth over longer periods, decrease for faster adaptation.

---

### Step 4 — Log and verify the filter state on hardware

Add SmartDashboard telemetry to monitor the Kalman filter in action:

```cpp
// In RobotPeriodic(), after the vision fuse loop:
auto state = drive.GetState();
tkit::RecordOutput("Kalman/PoseX",    state.Pose.X().value());
tkit::RecordOutput("Kalman/PoseY",    state.Pose.Y().value());
tkit::RecordOutput("Kalman/PoseRot",  state.Pose.Rotation().Degrees().value());
```

Compare this against raw odometry (before vision corrections) to verify the
filter is actually moving toward accepted vision measurements.

---

## Summary Checklist

- [ ] **Step 1** — Pass explicit `odometryStandardDeviation` array to
      `CreateDrivetrain()` in `TunerConstants.cpp`
- [ ] **Step 2** — Run logged drive tests; tune `computeStdDevs()` slope and
      base in `vision.cpp`  
- [ ] **Step 3** — Enable `BOT_ADAPTIVE_STDDEVS`; tune `kVarianceThreshold`
      and `kMaxScale`
- [ ] **Step 4** — Add Kalman pose telemetry; verify convergence on hardware

---

## Key Constraints (do not change)

- `theta` stdDev is always `9999.0` — vision **must not** override the Pigeon 2
  gyro heading. This is intentional.
- `vision.process()` is called **exactly once** per `RobotPeriodic()` cycle.
  The measurement buffer is consumed on that call — do not call it again.
- Std-devs passed to `AddVisionMeasurement()` are per-measurement and override
  the constructor defaults for that call, so the constructor vision defaults are
  only a safety fallback.
