# Turret Rotation Range Update Plan

## Overview

The turret rotation range needs to be increased beyond its current limits.
This document outlines every location that must change and the gotchas to
watch for before making those changes.

---

## Current Range

| Direction | Soft Limit         | Approx. Angle |
|-----------|--------------------|---------------|
| Forward   | `+0.25_tr`         | ~90° left     |
| Reverse   | `-0.05542_tr`      | ~20° right    |

---

## What Needs to Change

### 1. Soft limits — `turret.cpp` → `configureMotors()`

```cpp
.WithForwardSoftLimitThreshold(0.25_tr)
.WithReverseSoftLimitThreshold(-0.05542_tr)
```

These are the primary hardware-enforced stops. Update both values to reflect
the new desired range. Units are **mechanism turns** (post gear-ratio, as
reported by Phoenix after `SensorToMechanismRatio` is applied).

### 2. Software clamp — `turret.cpp` → `setTargetPosition()`

```cpp
constexpr units::turn_t kMinPos = -0.05542_tr;
constexpr units::turn_t kMaxPos =  0.25_tr;
```

This is a software duplicate of the soft limits that clamps commanded
positions before they reach the motor. **Must be updated in sync with the
soft limits** or the two will fight each other.

### 3. Boot position seed — `turret.cpp` → `configureMotors()`

```cpp
_rotationMotor.SetPosition(0.25_tr);
```

On boot the motor is told it's at `0.25_tr` (the current forward soft limit,
representing physical home). If the range changes, verify this value still
correctly represents the physical home position.

---

## Gotchas ⚠️

### Gear ratio discrepancy — resolve before computing new limit values

There is a mismatch between two places:

- **Phoenix config** (`turret.cpp`): `SensorToMechanismRatio(10.0)`
- **Header constant** (`turret.hpp`): `kRotationGearRatio = 100.0` *(marked `// TODO: measure actual ratio`)*

All soft limit and `SetPosition` values are in **mechanism turns as Phoenix
sees them** (i.e. divided by `SensorToMechanismRatio`). If the wrong ratio is
in the config, the physical range produced by any new limit values will be
wrong. **Measure and confirm the actual gear ratio first.**

### `computeAimPosition()` mapping may need revisiting

The auto-aim geometry was designed around the original range:

```
 0_tr    = back
+0.25_tr = left  (forward soft limit)
~-0.05_tr = right (reverse soft limit)
```

If the range expands significantly, verify that the atan2 → motor position
mapping and its wrap logic (`while` loops into `(-0.5, +0.5]`) still produce
correct results across the new range. The comments in `computeAimPosition()`
will need to be updated to reflect the new calibration points.

### `calibrateRotationZero()` resets to `0_tr`

The zero-calibration command always resets to `0_tr`. The boot seed
`SetPosition(0.25_tr)` and this reset must both agree on what the physical
home position means. If the range or mounting changes, re-evaluate what
"zero" should represent mechanically.

### Manual rotation speed may feel slow across a wider arc

`updateRotationControl()` clamps duty cycle to `kMaxOutput = 0.3`. Over a
larger range this may feel sluggish. Consider increasing `kMaxOutput` after
validating the new limits are safe.

---

## Recommended Change Order

1. Confirm the actual gear ratio and resolve the `10.0` vs `100.0` discrepancy.
2. Calculate the new soft limit values in mechanism turns.
3. Update `WithForwardSoftLimitThreshold` / `WithReverseSoftLimitThreshold` in `configureMotors()`.
4. Update `kMinPos` / `kMaxPos` in `setTargetPosition()` to match exactly.
5. Verify `SetPosition(0.25_tr)` boot seed still represents physical home; update if needed.
6. Re-test `computeAimPosition()` across the new range and update its comments/mapping.
7. Tune `kMaxOutput` in `updateRotationControl()` if manual traversal feels too slow.
