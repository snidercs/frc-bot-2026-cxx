[![Build](https://github.com/snidercs/frc-bot-2026-cxx/actions/workflows/build.yml/badge.svg)](https://github.com/snidercs/frc-bot-2026-cxx/actions/workflows/build.yml)

# Team 9431 "Indy" 2026 FRC Robot Firmware

FRC 2026 robot code for Team 9431, written in C++20 using WPILib and CTRE Phoenix 6.

## Tech Stack

| Component | Version |
|-----------|---------|
| WPILib / GradleRIO | 2026.2.1 |
| CTRE Phoenix 6 | 26.1.0 |
| LuaBot | 0.0.1 |
| LuaJIT (via sol2) | 2.1 |
| C++ Standard | C++20 |
| Target Platform | RoboRIO (Athena) |

## Subsystems

- **Drivetrain** — 4-module CTRE swerve (Kraken X60 drive + steer, CANcoder)
- **Turret** — rotation (position PID), shooter flywheel (velocity), uptake feeder (velocity)
- **Intake** — two TalonFX motors, voltage-controlled
- **Climber** — single TalonFX, software-limited travel, 180:1 gear ratio
- **Vision** — 4 PhotonVision cameras (FL, FR, BL, BR), pose fused into swerve estimator

## Build & Deploy

```bash
./gradlew build      # compile + run tests
./gradlew deploy     # build and deploy to roboRIO
```

Requires Java 17+ on your PATH (only used to run Gradle — the C++ cross-compiler is managed automatically by GradleRIO).

## Configuration

All tunable constants live in `robot/config.lua` and are loaded at runtime. Button indices, CAN IDs, drive gains, and input shaping values can be changed there without recompiling.

---

## Controls — Dual Flight Stick (Default)

Two flight sticks: **Stick 0** (left hand, USB 0) and **Stick 1** (right hand, USB 1).  
Set `gamepad = false` in `robot/config.lua` to use this mode (it is the default).

### Axes

| Stick | Axis | Function | Shaping |
|-------|------|----------|---------|
| Stick 0 | Y (axis 1) | Drive forward / backward | deadband 0.05, exponent 2.5 |
| Stick 0 | X (axis 0) | Drive strafe left / right | deadband 0.05, exponent 2.5 |
| Stick 1 | X (axis 0) | Rotate (yaw) | deadband 0.05, exponent 1.25 |
| Stick 0 | axis 3 | Manual turret rotation | gain 0.1 |

### Buttons — Stick 0 (Left Hand)

| Button | Trigger | Action | Subsystem |
|--------|---------|--------|-----------|
| 1 | While held | Lower climber | Climber |
| 5 | On press | Jitter front/back (intake agitator) | Drivetrain |
| 6 | On press | Jitter left/right (intake agitator) | Drivetrain |
| 8 | On press | Reset field-centric heading | Drivetrain |
| 16 | While held | Raise climber | Climber |
| 18 | While held | Run intake | Intake |

### Buttons — Stick 1 (Right Hand)

| Button | Trigger | Action | Subsystem |
|--------|---------|--------|-----------|
| 3 | On press | Zero turret rotation (calibrate) | Turret |
| 4 | While held | Disable climber soft limits (release to re-enable & zero) | Climber |
| 7 | — | Auto-aim (reserved) | Turret |
| 18 | While held | Shoot | Turret |
| 19 | While held | Eject intake | Intake |

### Default Commands

| Subsystem | Default Behavior |
|-----------|-----------------|
| Drivetrain | Field-centric drive from Stick 0 (X/Y) + Stick 1 (rotation) |
| Turret | Manual rotation from Stick 0 axis 3 |

> Button indices are defined in `robot/config.lua` and can be remapped without recompiling.
