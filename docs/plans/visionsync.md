# Vision Pose Drift When Robot is Obstructed

## Problem

When the robot is physically stopped (e.g. driven into a wall), the wheel odometry
inside the CTRE swerve pose estimator still accumulates motion because the drivetrain
**reports commanded velocity, not actual velocity** — the motors are still being driven
even if the wheels aren't moving. Vision measurements fused via `AddVisionMeasurement()`
will eventually pull the pose back, but if the cameras are also blocked by the wall (or
have no AprilTags in view), vision measurements stop arriving and the pose drifts in the
direction of the obstructed motion until it jumps when vision resumes.

The symptom: Field2d widget moves the robot ghost through the wall while the physical
robot is stationary.

---

## Root Cause in Our Stack

Our pose estimator is the CTRE `SwerveDrivetrain` which internally uses a
`SwerveDrivePoseEstimator`. It fuses:
1. **Wheel odometry** (swerve module position deltas) — accumulates continuously
2. **Gyro heading** — accurate, never drifts
3. **Vision measurements** — only when cameras report valid AprilTag fixes

When the robot is against a wall:
- The gyro is fine (heading stays correct)
- X/Y odometry drifts because wheel slip or commanded-vs-actual velocity error
- Vision may be blocked (wall is too close for cameras to see tags) so the
  residual gate in `processResults()` (`kMaxResidual = 0.6m`) may reject any
  incoming measurements as "too far from current pose" — *because the pose itself
  has already drifted too far*

---

## Options

### Option A — Velocity-Based Odometry Rejection (Best Long-Term)

Detect when the robot is *commanded* to move but the wheels report near-zero
*actual* velocity (wheel slip / obstruction) and temporarily increase the
process noise on the estimator — effectively trusting vision more and odometry
less during that window.

**Implementation sketch:**
- In `robot.cpp`, pass `drive.GetState().Speeds` (actual measured chassis speeds)
  into `vision.read()` alongside `currentPose` — it's already read once before
  the vision block, so no extra `GetState()` call needed
- `VisionIO::read()` stores the incoming speeds as `_lastSpeeds` (same pattern
  as all other gate state in the base class) and computes a scalar obstruction
  multiplier — e.g. if commanded speed is high but `_lastSpeeds` is near-zero
  for N consecutive cycles, multiply the `stdDevs` passed to
  `AddVisionMeasurement()` by a scale factor inside `processResults()`
- `SetVisionMeasurementStdDevs()` is **not** used — it persists globally across
  cycles. We already pass per-measurement `stdDevs` via the 3-argument
  `AddVisionMeasurement()` overload; Option A just adds an obstruction multiplier
  on top of `computeStdDevs()`, keeping all gate logic self-contained in `VisionIO`

**Pros:** Addresses the root cause directly  
**Cons:** Requires access to commanded speeds (need to cache them in `robot.cpp`),
and the estimator still drifts during the gap before vision recovers

---

### Option B — Loosen the Residual Gate During Obstruction ✅ IMPLEMENTED

The residual gate in `processResults()` currently hard-rejects any vision pose
more than `kMaxResidual = 0.6m` from current odometry. If odometry has already
drifted 0.7m, valid vision measurements get rejected — making the drift
self-reinforcing.

**Implementation:** When the vision pipeline has been returning zero accepted
measurements for more than ~10 consecutive cycles (`_acceptedCount` staying at 0),
temporarily widen `kMaxResidual` to e.g. 1.5m to let vision "snap" the pose back.

```cpp
// In processResults() — already live:
const auto residualLimit = _dropouts > 10 ? kLooseResidual : kMaxResidual;
// kMaxResidual = 0.6m, kLooseResidual = 1.5m
```

**Pros:** Simple, contained change inside `VisionIO`  
**Cons:** A wider residual gate can let bad measurements through if the pose
drifted for a different reason (bad odometry, not obstruction)

---

### Option E — Velocity Gate on Accepted Measurements ✅ IMPLEMENTED

Reject a measurement if the implied velocity between the *last accepted pose*
and this one is physically impossible. This catches "slow" bad solves that
slip past the residual gate because odometry has already drifted to meet them.

**Implementation (inside `processResults()`, after residual gate):**
```cpp
auto& camState = _cameraState[cameraName];
if (camState.lastTime > 0_s) {
    auto dt = estimatedPose->timestamp - camState.lastTime;
    if (dt > 0_s) {
        auto displacement = pose2d.Translation().Distance(camState.lastPose.Translation());
        auto impliedVelocity = displacement / dt;
        if (impliedVelocity > kMaxImpliedVelocity) {  // kMaxImpliedVelocity = 5.0_mps
            _rejectedVelocity++;
            continue;
        }
    }
}
// State updated only after best candidate wins (not here) to avoid
// poisoning the next cycle with a superseded candidate.
```

Per-camera state lives in `VisionIO::_cameraState` (`std::unordered_map<std::string, CameraGateState>`).
State is committed only when a candidate is accepted as the best — never mid-loop.

**Pros:** No latency added; catches a failure mode the residual gate misses  
**Cons:** First measurement per camera per boot cycle is always passed (cold start)

---

### Option F — Adaptive `stdDevs` Based on Measurement Consistency ✅ IMPLEMENTED

Rather than filtering the pose directly, inflate the `stdDevs` passed to
`AddVisionMeasurement()` when recent measurements have been inconsistent.
This keeps the Kalman filter in control — instead of rejecting outright,
it just trusts the measurement less, allowing gradual convergence rather
than a hard jump or hard reject.

**Implementation (live in `VisionIO`, guarded by `BOT_ADAPTIVE_STDDEVS`):**
```cpp
// Per-camera rolling window (kWindowSize = 6) in CameraGateState.
// Variance computed via indy::math::rollingVariance2d().
// In computeStdDevs():
static constexpr double kVarianceThreshold = 0.05;
static constexpr double kMaxScale          = 3.0;
const double scale = std::clamp(variance / kVarianceThreshold, 1.0, kMaxScale);
xy *= scale;
```

**Pros:**
- No latency; Kalman filter does the gradual convergence naturally
- High-jitter camera still contributes, just with reduced weight
- Complements Option B — even a measurement that passes the loose residual
  gate gets appropriately down-weighted if it's inconsistent

**Cons:** Requires a small rolling history buffer per camera; adds complexity
to `computeStdDevs()`

---

### Option C — Field Boundary Clamping ✅ IMPLEMENTED

After every odometry update, clamp the estimated pose to within the field
boundaries. The robot cannot physically be outside the field, so any pose
that exits the field boundary is definitively wrong.

We already do this as a *rejection gate* for incoming vision measurements
(`kMaxResidual` + the out-of-bounds filter in `processResults()`). We could
additionally clamp the *drivetrain's own pose* if it exits the boundary.

**Implementation:**  
In `RobotPeriodic()`, after reading `currentPose`, check if it's outside
field bounds and call `drivetrain.ResetPose()` to snap back to the nearest
in-bounds position. Effectively a "wall constraint."

**Pros:** Zero drift outside the field ever  
**Cons:** `ResetPose()` hard-resets odometry history — if the pose was only
slightly out of bounds from noise, this causes a visible jump. Needs careful
threshold tuning.

---

### Option D — Add Cameras Facing Away from the Wall (Hardware)

The real fix at scale: more cameras with better field-of-view coverage. If
the robot is against one wall, the cameras on the *opposite* side of the
robot still have clear sightlines to the far AprilTags and will continue
reporting accurate measurements.

We already have FL and BL. Adding FR and BR (already stubbed in `vision.hpp`
as `kCameraNames` / `kRobotToCamera`) would mean that driving into any wall
only blocks *half* the cameras at most.

**Pros:** Solves the root cause at the sensor level, no software hacks  
**Cons:** Hardware cost, calibration time (see `docs/photonvision-calibration.md`)

---

## Recommendation

**Short term: Option B ✅ done** — residual gate loosens after 10 dropout cycles.
Already live in `VisionIO::processResults()`.

**Next up: Option E ✅ done** — velocity gate on accepted measurements is live in `VisionIO::processResults()`.

**Option C ✅ done** — field boundary clamp in `RobotPeriodic()`. Fires at `0.75_m`
outside the wall; clamp runs before the `currentPose` snapshot so the residual gate
benefits in the same cycle. Telemetry: `Vision/PoseClamped` on SmartDashboard.

**Option F ✅ done** — adaptive `stdDevs` scaling is live behind `BOT_ADAPTIVE_STDDEVS`.
A rolling window of 6 accepted positions is tracked per camera in `CameraGateState`.
`computeStdDevs()` calls `indy::math::rollingVariance2d()` and scales `xy` stdDevs
up to `3×` when recent measurements are inconsistent. Complements B and E: measurements
that slip through the gates are down-weighted by the Kalman filter rather than
hard-rejected.

**Medium term: Option A** — velocity discrepancy detection. Catches obstruction
before the pose drifts enough to trigger the rejection cycle in the first place.

**Long term: Option D** — finish mounting FR and BR cameras. This makes the
problem largely disappear at the sensor level and improves overall pose quality
everywhere on the field.

---

## Related Files

| File | Relevance |
|------|-----------|
| `src/vision.hpp` | `kMaxResidual`, `processResults()`, `_acceptedCount` |
| `src/vision.cpp` | Residual gate implementation |
| `src/robot.cpp` | `RobotPeriodic()` — where `getMeasurements()` and `AddVisionMeasurement()` are called |
| `docs/photonvision-calibration.md` | Camera calibration steps for adding FR/BR |
