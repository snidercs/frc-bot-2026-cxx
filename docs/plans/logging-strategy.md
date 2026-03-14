# Logging Strategy: Async & Real-Time Safe Telemetry

This doc answers the question: *does `SmartDashboard::PutNumber()` (or any
NetworkTables write) cost us real-time performance in the robot loop?*
Then it lays out a path to logging that is genuinely safe in a 60 Hz control
loop.

---

## TL;DR

| Method | RT-safe? | Notes |
|---|---|---|
| `SmartDashboard::PutNumber/Boolean/…` | **No** | Acquires an NT mutex, may serialise to wire inline |
| `nt::DoublePublisher::Set()` (raw NT4) | **Marginal** | Same mutex path, better batching, but still blocks RT thread |
| `wpi::DataLog` / `DataLogManager` | **Yes** | File I/O on a dedicated background thread; robot loop only touches a mutex + memcpy |
| `frc::DataLogManager` auto-NT mirror | **Acceptable** | NT values still written, but DataLog part is async |

**Short answer:** Every `SmartDashboard::Put*` call acquires a global NetworkTables
mutex and copies data into an internal queue.  Under normal conditions this takes
microseconds, but it is *not* bounded — if the NT background thread is busy
flushing or the lock is contended by a dashboard subscriber, the robot thread can
stall.  On a loaded RoboRIO 2 running 4-camera vision it is measurably worse.

---

## What actually happens inside NetworkTables

NT4 (WPILib ≥ 2023) publishes on a **producer/consumer** model:

1. `SmartDashboard::PutNumber(key, value)` resolves to an `nt::DoubleEntry::Set()`
   call.
2. That call acquires the NT instance's **internal mutex**, timestamps the value
   with the FPGA clock, and appends it to a **per-topic value ring-buffer**.
3. A **background NT thread** wakes up periodically (default ~100 ms, configurable
   down to ~5 ms with `FlushLocal()`) and serialises changed values over the
   network.

The robot loop itself **does** acquire the mutex on every `Put` call.  For a few
values this is fine.  For a subsystem that dumps 20–30 topics every 16.7 ms loop,
the cumulative lock time adds up.

Key facts from the WPILib docs and NT4 spec:

- By default NT **only transmits the most recent value** between flush intervals,
  so intermediate writes are silently dropped for dashboard purposes.
- NT is not a real-time transport — the spec makes no latency guarantees.
- String/array types involve heap allocations on every `Set()` call.
- `SmartDashboard` is just a thin wrapper around the `/SmartDashboard` NT
  sub-table; there is nothing special about it from a performance standpoint.

---

## What WPILib DataLog does differently

`wpi::DataLog` (via `frc::DataLogManager`) is designed from the ground up for
low-latency on-robot recording:

> *"In general, the data log facilities provided by WPILib have minimal overhead
> to robot code, as all file I/O is performed on a separate thread — the log
> operation consists of mainly a mutex acquisition and copying the data."*
> — WPILib docs

The write path on the robot thread is:
1. Acquire a **short critical section** (spinlock / lightweight mutex).
2. `memcpy` the value bytes into a **lock-free ring buffer**.
3. Release.

A dedicated **background writer thread** drains the ring buffer to disk (USB
drive or `/home/lvuser/logs`).  The robot loop never calls `write()` or `fsync()`.

This is genuinely RT-friendly.  The only overhead is the mutex + copy, which
is bounded and cache-local.

---

## Recommended logging architecture

### Tier 1 — Critical telemetry (every loop, 60 Hz)

Use `wpi::log::DoubleLogEntry` / `BooleanLogEntry` / `StructLogEntry` directly.
Pre-create entries at construction time.  Call `.Append(value)` in the periodic
method.  Zero heap allocation, zero network jitter.

```cpp
// In subsystem header:
wpi::log::DoubleLogEntry _logTurretAngle;
wpi::log::DoubleLogEntry _logShooterRPM;

// In constructor:
auto& log = frc::DataLogManager::GetLog();
_logTurretAngle = wpi::log::DoubleLogEntry(log, "/turret/angle_deg");
_logShooterRPM  = wpi::log::DoubleLogEntry(log, "/shooter/rpm");

// In Periodic():
_logTurretAngle.Append(_turretMotor.GetPosition().GetValue().value());
_logShooterRPM.Append(_shooterMotor.GetVelocity().GetValue().value());
```

Files are downloaded post-match via the WPILib **Log Viewer** or `wpilog` CLI
tool and are fully replayable.

### Tier 2 — Dashboard/tuning values (throttled, ~10 Hz)

Dashboard values are useful during development and tuning but should not be
published every loop.  Gate them with a counter or timer:

```cpp
// Publish to NT only every 6th loop (≈10 Hz)
if (++_dashboardCounter % 6 == 0) {
    frc::SmartDashboard::PutNumber("Turret/Angle", angleDeg);
    frc::SmartDashboard::PutNumber("Shooter/RPM", rpm);
}
```

Alternatively, use `frc::DataLogManager` with NT mirroring enabled (the default)
— it writes NT values **and** logs them.  Then you can remove `SmartDashboard`
calls entirely and use Elastic / Shuffleboard subscribed to the same NT topics.

### Tier 3 — Diagnostic / event logging (on-demand)

Use `frc::DataLogManager::Log(message)` for one-shot human-readable messages
(mode transitions, fault codes, vision dropout events).  These go to the
`messages` entry in the data log and are also printed to stdout.

```cpp
frc::DataLogManager::Log("Vision: camera 0 lost track, resetting");
```

---

## What to avoid in the robot loop

| Pattern | Problem |
|---|---|
| `SmartDashboard::PutString(key, std::string(…))` | Heap allocation + NT mutex every call |
| `SmartDashboard::PutNumberArray(key, vector)` | Vector copy + NT mutex, expensive |
| `fmt::format(…)` / `std::to_string(…)` in Periodic | Heap allocation every loop |
| NT Publisher created inside `Periodic()` | Topic lookup + allocation every call |
| `DataLogManager::Log(fmt::format(…))` every loop | String format + copy every 16.7 ms |

---

## Migration path

1. **Add `DataLogManager::Start()`** in `Robot::RobotInit()` (or `Container`
   constructor).  This starts the background writer thread and auto-logs all NT
   changes.  One line, immediate benefit.

2. **Audit `SmartDashboard` calls** — grep for `SmartDashboard::Put` and
   `frc::SmartDashboard`.  Categorise each as "tuning" (can be throttled or
   removed post-competition) vs. "diagnostic" (move to DataLog entry).

3. **Pre-declare `LogEntry` objects** for any value that is written every loop.
   One entry per subsystem file; initialised in the constructor.

4. **Throttle remaining NT puts** to 10 Hz or less using a loop counter.

5. **Consider a `Logger` utility class** — a thin wrapper that owns all
   `LogEntry` objects for a given subsystem and exposes typed `record*()`
   methods.  This keeps `Periodic()` clean and makes it easy to change the
   backing mechanism later.

```cpp
// Example sketch of a subsystem-local logger
struct TurretLogger {
    explicit TurretLogger(wpi::log::DataLog& log)
        : angle(log, "/turret/angle_deg"),
          setpoint(log, "/turret/setpoint_deg"),
          error(log, "/turret/error_deg"),
          atTarget(log, "/turret/at_target") {}

    wpi::log::DoubleLogEntry  angle;
    wpi::log::DoubleLogEntry  setpoint;
    wpi::log::DoubleLogEntry  error;
    wpi::log::BooleanLogEntry atTarget;
};
```

---

## Simulation note

`DataLogManager` works transparently in simulation — the same `LogEntry::Append`
calls write to a `.wpilog` file in the sim working directory.  No code changes
needed between real and sim.

---

## References

- [WPILib: On-Robot Telemetry Recording Into Data Logs](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html)
- [WPILib: What is NetworkTables](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html)
- [WPILib: Third-Party Telemetry Libraries](https://docs.wpilib.org/en/stable/docs/software/telemetry/3rd-party-libraries.html)
- NT4 wire protocol spec: `allwpilib/ntcore/doc/networktables4.adoc`
- Related: [`plans/realtime-performance.md`](./realtime-performance.md) — other hot-path concerns
