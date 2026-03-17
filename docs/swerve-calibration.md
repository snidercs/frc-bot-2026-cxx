# Swerve Calibration

Proper calibration is the foundation of accurate autonomous performance. In this robot, that means tuning swerve motor gains, selecting the right drive request type, preventing wheel slip, calibrating effective wheel radius, and measuring realistic top speed for feedforward scaling.

Calibration turns theoretical module constants into real-world accuracy. When these values are right, the robot tracks straighter, odometry drifts less, and autonomous routines become much more repeatable.

## Recommended order

Follow these steps in order:

1. Tune `steerGains` in `src/generated/TunerConstants.h`
2. Tune `driveGains` in `src/generated/TunerConstants.h`
3. Update drive request type for closed-loop driving
4. Find `kSlipCurrent` in `src/generated/TunerConstants.h`
5. Tune `kWheelRadius` in `src/generated/TunerConstants.h`
6. Find `kSpeedAt12Volts` in `src/generated/TunerConstants.h`
7. Re-check straight driving and odometry drift

## Motor calibration & tuning

### Tune `steerGains`

The steer motors are position-controlled. Treat them like a rotational mechanism and tune them for accurate angle tracking with minimal overshoot.

#### Goal

Get each module to snap to its target angle quickly and cleanly.

#### Procedure

1. Start from the current steer gains in `src/generated/TunerConstants.h`
2. Command module angle changes and watch for overshoot, hunting, or sluggishness
3. Increase `kP` until the module responds crisply
4. Add a small amount of `kD` only if you need to reduce overshoot
5. Leave `kI` at zero unless you have a clear reason to add it
6. Re-test with repeated angle changes across several headings

#### What good looks like

- Module reaches the commanded angle quickly
- No continuous oscillation around setpoint
- No obvious lag when switching directions

### Tune `driveGains`

The drive motors are velocity-controlled. Tune them in two phases so you separate pure velocity tracking from carpet load effects.

#### Phase 1: wheels off the ground

1. Put the robot securely on blocks
2. Command drive velocity using closed-loop control
3. Tune `kP` for responsive tracking
4. Confirm `kV` is in the right ballpark
5. Add `kS` only if needed to overcome static friction cleanly
6. Keep `kI` and `kD` minimal unless testing shows you need them

#### Phase 2: on the ground

1. Put the robot on carpet or the normal driving surface
2. Drive straight at a few different speeds
3. Fine-tune `kP` for real load conditions
4. Verify acceleration and deceleration are smooth
5. Make sure the robot does not feel lazy or oscillatory

#### What good looks like

- Velocity tracks commands consistently
- Robot accelerates smoothly
- No obvious pulsing or surging at steady speed

## Drive control configuration

### Use closed-loop drive requests

For accurate tuning and repeatable autonomous behavior, use velocity-based drive requests instead of open-loop voltage drive requests.

#### What to change

Where your driver control request is configured, prefer:

- `DriveRequestType::Velocity` for drive motors
- minimal or no extra deadband while calibrating

#### Why

Open-loop voltage is fine for quick bring-up, but velocity control gives much better speed consistency for testing odometry, feedforward, and path following.

## Find `kSlipCurrent`

`kSlipCurrent` is the stator current limit that helps prevent the wheels from breaking traction under heavy load.

#### Why it matters

Stator current is directly related to motor torque. If torque exceeds available traction, the wheels slip and odometry gets worse.

#### Procedure

1. Put the robot on carpet
2. Place it firmly against a wall
3. Plot drive motor stator current and velocity in Phoenix Tuner X
4. Slowly increase drive output
5. Watch for the point where wheel velocity becomes non-zero and stator current behavior changes sharply
6. Record that slip threshold
7. Set `kSlipCurrent` slightly below that value for margin

#### Notes

- Test on the real surface when possible
- Too high wastes traction and hurts repeatability
- Too low makes the robot feel soft and hurts acceleration

## Tune `kWheelRadius`

Use measured travel versus reported odometry travel to determine the effective wheel radius.

This is an *effective* radius, not just the raw physical tire radius. It captures tread compression, carpet interaction, and any remaining drivetrain scaling error.

#### Quick calibration procedure

1. Start with a reasonable nominal wheel radius in `src/generated/TunerConstants.h`
2. Drive slowly forward in a straight line to reduce slip
3. Measure the actual physical distance traveled
4. Record the reported odometry distance
5. Update the wheel radius with:

$$
\text{newWheelRadius} = \left(\frac{\text{actualDistance}}{\text{reportedDistance}}\right) \times \text{currentWheelRadius}
$$

Equivalent form:

$$
\text{newWheelRadius} = \frac{\text{currentWheelRadius}}{\left(\frac{\text{reportedDistance}}{\text{actualDistance}}\right)}
$$

#### Interpretation

- If reported distance is **greater** than actual distance, the configured wheel radius is too large
- If reported distance is **less** than actual distance, the configured wheel radius is too small

#### Current repo example

The current calibration comment in `src/generated/TunerConstants.h` was computed from:

- configured radius: `2.00_in`
- reported distance: `1.002587 m`
- measured distance: `0.98 m`

So:

$$
\text{scaleFactor} = \frac{1.002587}{0.98} = 1.02305
$$

$$
\text{newWheelRadius} = \frac{2.00\text{ in}}{1.02305} \approx 1.9551\text{ in}
$$

#### Best practices

- Drive slowly during this test
- Use a long straight path
- Repeat multiple runs and average the results
- Re-check after changing tread, wheel type, or gear ratio

## Find `kSpeedAt12Volts`

`kSpeedAt12Volts` should reflect the robot’s real achievable top speed, not just a catalog number.

#### Preferred measurement procedure

1. Put the robot on the ground on its normal driving surface
2. Command full forward speed in a straight line
3. Log the peak chassis velocity from drivetrain odometry
4. Repeat several times with a healthy battery
5. Use the representative peak value for `kSpeedAt12Volts`

#### Alternative quick check in Tuner X

You can also inspect drive motor velocity in Phoenix Tuner X and convert rotor velocity to linear speed:

$$
\text{speed} = \frac{\text{rotorRPS}}{\text{kDriveGearRatio}} \times 2\pi \times \text{kWheelRadius}
$$

For the current values in this repo:

- `kDriveGearRatio = 6.026785714285714`
- `kWheelRadius = 1.9551_in`

A setting of `5.12_mps` corresponds to about:

$$
	ext{rotorRPS} = \frac{5.12 \times 6.026785714285714}{2\pi \times 0.04966\text{ m}} \approx 98.9\text{ RPS}
$$

That is about `5934 RPM`, which is near the free speed of a Kraken X60 and suggests the current constant is a theoretical estimate.

#### Recommendation

Use the on-ground measured value as the final competition value.

## Zeroing procedure

Straight driving depends on modules being zeroed correctly.

#### Recommended process

1. Power down the robot
2. Use a straight edge across the wheels to align all modules perfectly straight
3. Power on and inspect module angles
4. Save zero positions in Phoenix Tuner X
5. Reboot and verify the wheels still come back to straight

## Encoder security

Protect anything that can shift calibration after impacts.

#### Recommendation

Make sure the steering encoder mounting is secure enough that hard hits do not move the zero reference. Even a small shift can introduce visible odometry and heading error.

## Re-check list after changes

Re-run the following after changing wheels, tread wear state, gearing, major current limits, or module hardware:

- `kSlipCurrent`
- `kWheelRadius`
- `kSpeedAt12Volts`
- module zero positions

## Practical notes for this robot

- Main constants live in `src/generated/TunerConstants.h`
- Wheel radius and top speed should be treated as measured values, not one-time theoretical values
- Recalibrate after swapping module ratio, wheel type, or heavily worn tread
- Use carpet for final validation whenever possible

## What to do next

Once these values are stable:

1. Validate straight-line odometry over longer distances
2. Validate rotation accuracy
3. Confirm path following and autonomous repeatability
4. Then move on to vision and localization tuning

## Source

This document was adapted from the Gray Matter Workshop swerve calibration guide:

- https://www.frc5712.com/swerve-calibration
