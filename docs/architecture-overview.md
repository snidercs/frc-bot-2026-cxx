# Robot Architecture Overview

This page is the big-picture map of the robot code.

## What lives where

- `src/container.*` wires together subsystems, controls, and autonomous entry points
- `src/robot.*` owns the main robot lifecycle (`RobotInit`, `RobotPeriodic`, mode transitions)
- `src/drivetrain.*` handles swerve driving and odometry
- `src/vision.hpp` / `src/vision.cpp` define the `VisionIO` base class with shared filtering logic
- `src/visionmulti.hpp` / `src/visionmulti.cpp` implement `VisionIO` for real hardware (4× PhotonVision cameras)
- `src/visionsim.hpp` / `src/visionsim.cpp` implement `VisionIO` for simulation (`VisionSystemSim`)
- `src/turret.*` handles turret rotation, shooter flywheel control, and aiming behavior
- `src/intake.*`, `src/climber.*`, `src/shaker.*` hold mechanism-specific logic
- `robot/config.lua` stores field-tunable values and hardware configuration

## Architectural patterns

### Command-based structure

The robot uses WPILib's command-based model. Subsystems expose command factories,
and the container binds those commands to driver inputs.

### Hardware abstraction

The vision system uses a `VisionIO` base class so that real hardware and
simulation share the same higher-level filtering and measurement logic.

### Configuration-driven tuning

Many values live in `robot/config.lua` so tuning can happen without hardcoding
numbers throughout the codebase.

## What to add later

- subsystem ownership diagram
- control flow from joystick input to hardware output
- autonomous path and named command overview
- startup and simulation flow notes
