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
- Compare `GetState().speeds` (actual measured chassis speeds) against the last
  commanded `ChassisSpeeds`
- If the discrepancy exceeds a threshold (e.g. >0.2 m/s commanded, <0.05 m/s
  actual) for more than N cycles, call `drivetrain.SetVisionMeasurementStdDevs()`
  with tighter values to increase vision weight

**Pros:** Addresses the root cause directly  
**Cons:** Requires access to commanded speeds (need to cache them in `robot.cpp`),
and the estimator still drifts during the gap before vision recovers

---

### Option B — Loosen the Residual Gate During Obstruction

The residual gate in `processResults()` currently hard-rejects any vision pose
more than `kMaxResidual = 0.6m` from current odometry. If odometry has already
drifted 0.7m, valid vision measurements get rejected — making the drift
self-reinforcing.

**Implementation:** When the vision pipeline has been returning zero accepted
measurements for more than ~10 consecutive cycles (`_acceptedCount` staying at 0),
temporarily widen `kMaxResidual` to e.g. 1.5m to let vision "snap" the pose back.

```cpp
// In processResults() / getMeasurements():
auto effectiveResidual = (_consecutiveDropouts > 10) ? 1.5_m : kMaxResidual;
```

**Pros:** Simple, contained change inside `VisionIO`  
**Cons:** A wider residual gate can let bad measurements through if the pose
drifted for a different reason (bad odometry, not obstruction)

---

### Option C — Field Boundary Clamping

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

**Short term: Option B** — loosen the residual gate after a dropout streak.
It's a 5-line change inside `VisionIO::processResults()`, is self-contained,
and directly targets the self-reinforcing rejection cycle.

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
