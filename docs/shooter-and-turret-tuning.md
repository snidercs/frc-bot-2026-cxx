# Shooter and Turret Tuning Notes

This page should collect tuning knowledge for the turret and shooter so it does
not live only in comments or in someone's memory.

## Current known values

All constants are in `turret.hpp` and `turret.cpp` unless noted.

### Turret rotation

| Parameter | Value | Location |
|-----------|-------|----------|
| Forward soft limit | `+0.25_tr` (~90° left) | `configureMotors()` |
| Reverse soft limit | `-0.05542_tr` (~20° right) | `configureMotors()` |
| Boot position seed | `0.25_tr` | `_rotationMotor.SetPosition()` |
| Sensor-to-mechanism ratio | `10.0` | `FeedbackConfigs` |
| Angle tolerance | `2°` | `kAngleTolerance` |
| Manual rotation max output | `0.3` duty cycle | `updateRotationControl()` |
| PID (Slot 0) | P=24.0, I=0.0, D=0.2, V=0.12 | `configureMotors()` |

> See [`plans/turrentrange.md`](./plans/turrentrange.md) for turret range change planning notes.

### Shooter flywheel

| Parameter | Value | Location |
|-----------|-------|----------|
| Near calibration point | 45 tps @ 2.378 m (~7 ft 10 in) | `velocityFromDistance()` |
| Far calibration point | 64 tps @ 5.0 m (~16 ft 5 in) | `velocityFromDistance()` |
| Velocity tolerance | ±5 tps | `kShooterTolerance` |
| PID (Slot 0) | P=0.2, I=0.0, D=0.0, V=0.12, S=0.25 | `configureMotors()` |

The shooter speed is interpolated linearly between the two calibration points and
clamped at the ends. Adjust `kNearSpeed`, `kFarSpeed`, `kNearDist`, and `kFarDist`
in `velocityFromDistance()` as more data points are collected.

### Uptake

| Parameter | Value | Location |
|-----------|-------|----------|
| Uptake velocity | 100 tps | `kUptakeVelocity` |
| PID (Slot 0) | P=0.2, I=0.0, D=0.0, V=0.12, S=0.2 | `configureMotors()` |

## Good data to log here

- tested distances
- successful shot speeds
- misses and what changed them
- whether auto-aim or manual aim was used
- battery voltage during tuning

## Related docs

- [`plans/turrentrange.md`](./plans/turrentrange.md)
