# Lifter Homing Command — Implementation Status

## Current State

The lifter/climber subsystem (`Climber` in `src/climber.hpp` / `src/climber.cpp`) is functional for
manual duty-cycle control and has soft limits configured, but **does not yet have a homing routine**.
The encoder position is currently assumed to be zero at startup via a hard-coded `SetPosition(0.0_tr)`
call in `configureMotor()`, which is not reliable if the robot is powered on with the arm at an
unknown position.

---

## What Already Exists

| Feature | Status |
|---|---|
| `Climber` subsystem class | ✅ Done |
| Duty-cycle climb / lower commands | ✅ Done |
| Brake neutral mode | ✅ Done |
| Current limits (40 A supply / 80 A stator) | ✅ Done |
| `SensorToMechanismRatio` set (180.0) | ✅ Done |
| Software forward soft limit (`0.0_tr`) | ✅ Done |
| Software reverse soft limit (`-3.109043_tr`) | ✅ Done |
| `disableSoftLimitsCommand()` | ✅ Done |
| `enableSoftLimitsAndResetCommand()` | ✅ Done |
| Homing / zeroing on startup | ❌ Not implemented |
| `homeCommand()` command factory | ❌ Not implemented |
| `GetStatorCurrent()` registered as status signal | ❌ Not registered |
| Motion Magic position control | ❌ Not configured |

---

## Homing Options

Two approaches are available. **Option A is strongly preferred** and should be implemented if at all
possible. Option B is a fallback only.

---

## ⚠️ Safety Warning

Homing commands **drive the arm into a physical stop**. This is inherently more dangerous than sensor-
guided motion:

- **Never home at high speed** — use the slowest duty cycle that still produces reliable motion
- **Always have a watchdog timeout** on the homing command (e.g. `WithTimeout(5_s)`) so a stuck or
  jammed arm does not hold full voltage indefinitely
- **Stall detection (Option B) is not foolproof** — a loose CAN connection, stale status signal, or
  threshold set too high means the motor will continue grinding against the stop until the command
  times out or current limits kick in
- The existing **stator current limit of 80 A** in `configureMotor()` provides a last-resort safety
  net, but sustained stall current will heat the motor and should not be relied upon
- **Do not schedule homing during a match** if the arm is already known to be at home — gate the
  command with a flag or only trigger it once per enable cycle

---

## Option A — Hardware Limit Switch (Recommended)

This is the safest and most reliable approach. A normally-open limit switch wired to the TalonFX's
reverse limit pin provides an unambiguous, hardware-level stop signal with no threshold tuning.

### A1. TalonFX Configuration

Add `HardwareLimitSwitchConfigs` to `Climber::configureMotor()`:

```cpp
.WithHardwareLimitSwitch(
    configs::HardwareLimitSwitchConfigs{}
        .WithReverseLimitEnable(true)
        .WithReverseLimitType(signals::ReverseLimitTypeValue::NormallyOpen)
        .WithReverseLimitAutosetPositionEnable(true)  // TalonFX auto-zeros on trigger
        .WithReverseLimitAutosetPositionValue(0.0_tr)
)
```

`WithReverseLimitAutosetPositionEnable(true)` means the TalonFX itself calls `SetPosition(0.0_tr)`
the instant the switch closes — no software polling required. The motor also **automatically stops**
when the reverse limit is active, providing a hardware-enforced cut.

### A2. `homeCommand()` Implementation

```cpp
/** Drives the lifter slowly in reverse until the hardware reverse limit switch triggers.
    The TalonFX automatically zeros the encoder position on limit switch activation.
    Soft limits are re-enabled after homing completes.

    Should be scheduled once on robot enable before any position-controlled moves.
*/
frc2::CommandPtr homeCommand();
```

```cpp
frc2::CommandPtr Climber::homeCommand() {
    return disableSoftLimitsCommand()
        .AndThen(
            Run([this] { setDutyCycle(kHomeDutyCycle); })
            .Until([this] {
                // TalonFX reports ForwardLimit/ReverseLimit as a signal
                return m_motor.GetReverseLimit().GetValue() ==
                       signals::ReverseLimitValue::ClosedToGround;
            })
        )
        .AndThen(RunOnce([this] { stop(); }))
        .AndThen(enableSoftLimitsAndResetCommand())
        .WithTimeout(5_s)  // Safety watchdog
        .WithName("ClimberHome");
}
```

### A3. Safety Properties

| Property | Value |
|---|---|
| Deterministic stop | ✅ Yes — switch is a hard signal |
| Motor auto-cut on limit | ✅ Yes — TalonFX enforces it |
| Requires tuning | ✅ None |
| Risk of grinding | ✅ Minimal — switch stops the motor immediately |
| Watchdog needed | Recommended but not critical |

---

## Option B — Stall Detection via Stator Current (No Limit Switch)

Use this only if a hardware limit switch cannot be fitted. The arm creeps into a mechanical hard stop
and the motor stall is inferred from a spike in stator current.

> **Why stator current, not supply current?**  
> Supply current is filtered by the battery and PDH and lags behind actual motor load. Stator current
> reflects the instantaneous electromagnetic torque and responds within a single 20 ms loop cycle.
> The current `configureMotor()` registers `GetSupplyCurrent()` for telemetry but **`GetStatorCurrent()`
> must also be registered** for homing to read it reliably (see B2 below).

### B1. `homeCommand()` Implementation

```cpp
/** Drives the lifter slowly in reverse until stator current exceeds the stall threshold,
    indicating contact with the mechanical hard stop. Then zeros the encoder and
    re-enables soft limits.

    ⚠️  Less safe than Option A — requires careful threshold tuning and a watchdog timeout.
    The motor will grind against the hard stop until stall is detected.
    Should be scheduled once on robot enable before any position-controlled moves.
*/
frc2::CommandPtr homeCommand();
```

```cpp
frc2::CommandPtr Climber::homeCommand() {
    return disableSoftLimitsCommand()
        .AndThen(
            Run([this] { setDutyCycle(kHomeDutyCycle); })
            .Until([this] {
                return m_motor.GetStatorCurrent().GetValue() > kHomeStallAmps;
            })
        )
        .AndThen(RunOnce([this] {
            stop();
            m_motor.SetPosition(0.0_tr);
        }))
        .AndThen(enableSoftLimitsAndResetCommand())
        .WithTimeout(5_s)  // Safety watchdog — REQUIRED for this approach
        .WithName("ClimberHome");
}
```

**New constants required in `climber.hpp`:**

```cpp
static constexpr double          kHomeDutyCycle  = -0.10;  // Slow creep into hard stop
static constexpr units::ampere_t kHomeStallAmps  = 25_A;   // Stator current stall threshold
```

Start at `25 A` and tune on the robot:
- **Too low** → triggers during normal motion, false home
- **Too high** → arm grinds too long before stopping, motor heats up

### B2. Register `GetStatorCurrent()` as a Status Signal

The current `configureMotor()` does not register `GetStatorCurrent()`. Add it so it updates at 50 Hz
and doesn't return a stale value during the homing loop:

```cpp
BaseStatusSignal::SetUpdateFrequencyForAll(
    50_Hz,
    m_motor.GetVelocity(),
    m_motor.GetSupplyCurrent(),
    m_motor.GetStatorCurrent(),   // <-- add this
    m_motor.GetMotorVoltage()
);
```

### B3. Safety Properties

| Property | Value |
|---|---|
| Deterministic stop | ⚠️ No — depends on threshold tuning |
| Motor auto-cut on limit | ❌ No — software must stop it |
| Requires tuning | ❌ Yes — `kHomeStallAmps` must be measured |
| Risk of grinding | ⚠️ Yes — motor stalls briefly before detection |
| Watchdog needed | **Mandatory** |

---

## Shared: `container.cpp` Integration

Regardless of which option is used, wire `homeCommand()` into `container.cpp` so it runs
automatically on every enable:

```cpp
#include <frc2/command/button/RobotModeTriggers.h>

// In configureBindings():
frc2::RobotModeTriggers::Disabled().OnFalse(
    climber().homeCommand()
);
```

---

## Config Values to Add

```lua
climber_home_duty_cycle = -0.10,   -- Slow reverse speed for homing
climber_home_stall_amps = 25.0,    -- (Option B only) Stator current stall threshold
```

---

## Suggested Implementation Order

**If pursuing Option A (recommended):**
1. Wire reverse limit switch to TalonFX reverse limit pin
2. Add `HardwareLimitSwitchConfigs` to `Climber::configureMotor()`
3. Add `kHomeDutyCycle` constant to `climber.hpp`
4. Implement `homeCommand()` using `GetReverseLimit()` signal
5. Integrate into `container.cpp` via `RobotModeTriggers::Disabled().OnFalse(...)`
6. Test — verify position reads `0.0_tr` after homing and soft limits engage cleanly

**If pursuing Option B (fallback):**
1. Add `GetStatorCurrent()` to `SetUpdateFrequencyForAll` in `configureMotor()`
2. Add `kHomeDutyCycle` and `kHomeStallAmps` constants to `climber.hpp`
3. Implement `homeCommand()` using stator current threshold
4. Integrate into `container.cpp` via `RobotModeTriggers::Disabled().OnFalse(...)`
5. Test on robot — log stator current during normal motion and at hard stop to find a safe threshold
6. Tune `kHomeStallAmps` — the gap between normal motion current and stall current should be large
6. **Test in sim / on robot** — verify position reads `0.0_tr` after homing completes
