# Vision Pose Coordinate System

## Rule: Always Use Blue-Origin Frame

All vision pose measurements fed into the drivetrain pose estimator via
`AddVisionMeasurement()` **must be in the WPILib standard blue-alliance-wall
origin frame**, regardless of the current alliance color.

This applies to:
- `VisionMulti::getMeasurements()`
- Any future `VisionIO` implementation
- Any direct calls to `drivetrain.AddVisionMeasurement()`

---

## Why SetOrigin() Must Not Be Used

`frc::AprilTagFieldLayout::SetOrigin()` changes the coordinate frame that
`PhotonPoseEstimator` uses to resolve tag positions. When called with
`kRedAllianceWallRightSide`, tag positions are reported relative to the
**red wall** (i.e. X = 0 at the red wall).

However, `AddVisionMeasurement()` in both WPILib's `SwerveDrivePoseEstimator`
and CTRE's `SwerveDrivetrain` always interprets poses in the **blue-origin
frame** (X = 0 at the blue wall, X ≈ 16.46 m at the red wall).

### What Goes Wrong

| Scenario | PhotonPoseEstimator output | Drivetrain interpretation |
|---|---|---|
| Red alliance, `SetOrigin(kRedAllianceWallRightSide)` | `X ≈ 0–2 m` (red-relative) | Robot is placed near the **blue wall** ❌ |
| Red alliance, blue origin (correct) | `X ≈ 14–16 m` (blue-relative) | Robot is correctly placed near the **red wall** ✅ |

The symptom is the Field2d dashboard widget jumping the robot indicator to
the wrong side of the field when vision measurements are accepted on the red
alliance. This also corrupts the fused pose estimate and causes erratic
field-relative driving.

---

## Correct Pattern

Keep `_fieldLayout` at its default origin (blue wall) at all times.
`PhotonPoseEstimator` will still resolve AprilTag poses correctly because
the official WPILib field JSON files store all tag positions in the
blue-origin frame.

```cpp
// CORRECT: construct once, never call SetOrigin()
VisionMulti::VisionMulti()
    : _fieldLayout(vision::getFieldLayout())  // blue-origin by default
{
    // cameras and estimators constructed from _fieldLayout as-is
}
```

```cpp
// WRONG: do NOT do this
_fieldLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
```

---

## Reference

- WPILib Pose Estimation docs: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
- PhotonVision coordinate system: https://docs.photonvision.org/en/latest/docs/programming/pose-estimation/index.html
- `SetOrigin()` is intended for **rendering/display** purposes only, not for
  changing the coordinate frame of pose estimates fed to the robot's state
  estimator.
