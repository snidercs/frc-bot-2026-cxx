# TelemetryKit Integration Plan

Port the AdvantageKit logging from the Java project (`org.littletonrobotics.junction.Logger`)
to the C++ project using TelemetryKit (`telemetrykit/TelemetryKit.h`). The vendordep
`TelemetryKit.json` is already installed.

The API maps almost 1:1 — this is a straightforward drop-in.

---

## API Mapping

| Java (AdvantageKit) | C++ (TelemetryKit) |
|---|---|
| `Logger.recordMetadata(key, value)` | `tkit::RecordOutput(key, value)` |
| `Logger.addDataReceiver(new WPILOGWriter())` | `logger.AddReceiver(std::make_unique<tkit::WPILogWriter>(path))` |
| `Logger.addDataReceiver(new NT4Publisher())` | `logger.AddReceiver(std::make_unique<tkit::NetworkTablesReceiver>())` |
| `Logger.start()` | `logger.Start()` |
| `Logger.recordOutput(key, value)` | `tkit::RecordOutput(key, value)` |
| `logger.Periodic()` (called each cycle) | `logger.Periodic()` |

---

## Changes Required

### 1. `robot.cpp` — Constructor (Logger Setup)

Replace the banner/init block in `Robot::Robot()` with TelemetryKit setup, mirroring the Java
`Robot()` constructor:

```cpp
#include <telemetrykit/TelemetryKit.h>

Robot::Robot() {
    DriverStation::SilenceJoystickConnectionWarning(true);

    auto& logger = tkit::Logger::GetInstance();

    // Record metadata (mirrors Java Logger.recordMetadata calls)
    tkit::RecordOutput("Metadata/ProjectName", std::string("frc-bot-2026-cxx"));
    tkit::RecordOutput("Metadata/ControllerType",
        std::string(config::boolean("gamepad") ? "Gamepad" : "Flightsticks"));

    if (frc::RobotBase::IsReal()) {
        logger.AddReceiver(std::make_unique<tkit::WPILogWriter>("/U/logs")); // USB stick
        logger.AddReceiver(std::make_unique<tkit::NetworkTablesReceiver>());
    } else {
        logger.AddReceiver(std::make_unique<tkit::NetworkTablesReceiver>());
    }

    logger.Start();

    _container = Container::create();
}
```

### 2. `robot.cpp` — `RobotPeriodic()`

Add `logger.Periodic()` at the top of `RobotPeriodic()`, and replace the
`SmartDashboard::PutNumber` call for `DistanceToHub` with `tkit::RecordOutput`:

```cpp
void Robot::RobotPeriodic() {
    tkit::Logger::GetInstance().Periodic();  // flush logger each cycle

    frc2::CommandScheduler::GetInstance().Run();

#if BOT_VISION
    for (const auto& measurement : _container->vision().getMeasurements()) {
        _container->drivetrain().AddVisionMeasurement(
            measurement.pose, measurement.timestamp, measurement.stdDevs);
    }

    auto robotPose = _container->drivetrain().GetState().Pose;
    units::meter_t distanceToHub = robotPose.Translation().Distance(landmarks::hubPosition());
    tkit::RecordOutput("Robot/DistanceToHub", distanceToHub.value());  // replaces SmartDashboard
#endif
}
```

### 3. `robot.hpp` — No changes needed

TelemetryKit uses a singleton (`tkit::Logger::GetInstance()`), so no member variable is required.

---

## What Stays the Same

- `Telemetry.cpp/hpp` — the existing swerve drive telemetry over NetworkTables/SignalLogger is
  separate and does not need to change
- `SmartDashboard` calls inside subsystems under `#if BOT_TRACE_SUBSYSTEMS` — leave those as-is,
  they serve a different purpose (live tuning dashboard)

---

## Suggested Implementation Order

1. Add `#include <telemetrykit/TelemetryKit.h>` to `robot.cpp`
2. Add logger setup to `Robot::Robot()` constructor
3. Add `logger.Periodic()` to `RobotPeriodic()`
4. Replace `SmartDashboard::PutNumber("Robot/DistanceToHub")` with `tkit::RecordOutput`
5. Build and deploy — verify logs appear on USB and/or NetworkTables
