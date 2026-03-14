# Vision Pipeline Overview

This page should explain the full vision path from camera frame to fused robot pose.

## High-level flow

1. PhotonVision cameras produce unread pipeline results
2. `VisionIO` implementations gather raw results
3. Shared logic in `VisionIO::processResults()` filters bad measurements
4. The best candidate from each camera is converted into a `VisionMeasurement`
5. `RobotPeriodic()` feeds accepted measurements into drivetrain pose fusion
6. Turret aiming uses the fused robot pose plus field geometry

## Important design rules

- `getMeasurements()` is not a cached read
- call it exactly once per periodic cycle
- filtering logic should live in `VisionIO` when shared by real and sim
- turret aiming should use fused pose, not raw camera angles

## Active cameras

Two cameras are currently active (`vision.hpp` → `kCameraNames`):

| Name | Position | Notes |
|------|----------|-------|
| `FL` | Front-left corner, facing forward | 13.74 in right, 11.5 in forward, 15 in up |
| `BL` | Back-left corner, facing backward | pitched up 10°, 180° yaw |

`FR` and `BR` are stubbed in the code but not yet physically mounted or calibrated.

## Gating rules (applied in `VisionIO::processResults()`)

| Check | Threshold | What happens on failure |
|-------|-----------|------------------------|
| Has targets | — | rejected (`_rejectedNoTargets`) |
| Latency | < 0.25 s | rejected (`_rejectedStale`) |
| Tag count | ≥ 2 tags | rejected (`_rejectedAmbiguous`) — single-tag solves have 180° ambiguity |
| Tag distance | < 4.0 m | rejected (`_rejectedOutOfBounds`) |
| Field bounds | within field + 0.5 m margin | rejected (`_rejectedOutOfBounds`) |
| Odometry residual | < 0.6 m (normal) / 1.5 m (after 10 dropouts) | rejected (`_rejectedResidual`) |

## Standard deviation strategy

`computeStdDevs()` scales trust by distance and tag count:

```cpp
double xy = 0.2 + (distanceMeters * 0.07);  // further away = less trust
if (tagCount > 2) xy *= 0.75;               // more tags = tighter std dev
double theta = 9999.0;                       // never override the gyro heading
```

## Dropout behavior

`_dropouts` counts consecutive cycles with zero accepted measurements.
After 10 dropouts the residual gate widens from `0.6 m` to `1.5 m` to allow
vision to pull the pose back if odometry has drifted. See [`plans/visionsync.md`](./plans/visionsync.md)
for the full discussion.

## Related docs

- [`photonvision-calibration.md`](./photonvision-calibration.md)
- [`plans/visionsync.md`](./plans/visionsync.md)
