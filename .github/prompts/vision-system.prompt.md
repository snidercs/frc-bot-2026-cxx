# FRC Vision + Turret Aiming Architecture (Copilot Prompt)

We are building an FRC robot using **CTRE Phoenix 6 swerve**, generated with **Tuner X**, and programming in **C++**.

## Robot & Drivetrain
- CTRE Phoenix 6 swerve drivetrain
- Generated via Tuner X
- Standard WPILib coordinate frame (+X forward, +Y left)
- Drivetrain maintains a fused **field-relative robot pose** using odometry and vision

## Vision System
- **PhotonVision** for AprilTag detection and pose estimation
- **4× ThriftyCam** cameras (global shutter, ~55° FOV, default lens)
- Cameras are **fixed to the robot chassis**, not on the turret
- Mounted near each corner of the robot, angled approximately **45° outward**
- Vision processing is split across **2× Raspberry Pi 4s**
  - Each Pi runs PhotonVision with **2 cameras**
  - Same AprilTag pipeline on all cameras

## Vision Architecture
- Each camera produces **EstimatedRobotPose** from AprilTags
- Vision measurements are timestamped and filtered for quality
- Valid pose estimates are fused into the CTRE swerve pose estimator using
  `AddVisionMeasurement(...)`
- Turret control does **not** use raw camera yaw (`tx`)

## Turret Aiming
- Robot has a **turreted shooter** that must aim continuously
- Turret aiming is computed using:
  - Fused robot pose
  - Known AprilTag field layout
  - Known turret pivot location in robot coordinates
- Turret setpoint is the bearing from the turret pivot to the target, computed via geometry
- Turret PID drives angular error to zero
- Turret aiming continues through brief vision dropouts using odometry

## Design Goals
- Continuous turret tracking while driving and rotating
- Robust to camera dropouts and motion blur
- Scales cleanly from Pi 4 → Pi 5 without architectural changes
- Cameras only improve robot pose; all aiming logic is centralized and deterministic
