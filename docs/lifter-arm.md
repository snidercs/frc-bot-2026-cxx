# Talon FX60 Climber (Winch) — Tuner X Settings & C++ Notes

## Tuner X Configuration (Motion Magic)

### Motor Output

| Setting                 | Value                             |
| ----------------------- | --------------------------------- |
| Neutral Mode            | Brake                             |
| Inverted                | As required (positive wraps cord) |
| Peak Forward Duty Cycle | 1.0                               |
| Peak Reverse Duty Cycle | -1.0                              |

---

### Current Limits

| Setting                  | Value                |
| ------------------------ | -------------------- |
| Supply Current Limit     | 50 A                 |
| Supply Current Threshold | 65 A                 |
| Supply Time Threshold    | 0.15 s               |
| Stator Current Limit     | Disabled (initially) |

---

### Feedback

| Setting                   | Value                  |
| ------------------------- | ---------------------- |
| Feedback Sensor Source    | Integrated Sensor      |
| Sensor to Mechanism Ratio | 1.0 (update if geared) |
| Rotor to Sensor Ratio     | Default                |

---

### Closed Loop (Slot 0 — Position)

| Setting      | Value |
| ------------ | ----- |
| kP           | 15.0  |
| kI           | 0.0   |
| kD           | 0.0   |
| kV           | 0.0   |
| kS           | 0.0   |
| Gravity Type | None  |

---

### Motion Magic

| Setting         | Value        |
| --------------- | ------------ |
| Cruise Velocity | 30 rps       |
| Acceleration    | 60 rps²      |
| Jerk            | 0 (disabled) |

---

### Soft Limits

| Setting                      | Value                    |
| ---------------------------- | ------------------------ |
| Forward Soft Limit Enable    | True                     |
| Reverse Soft Limit Enable    | True                     |
| Forward Soft Limit Threshold | TBD (measured rotations) |
| Reverse Soft Limit Threshold | TBD (measured rotations) |

---

### Limit Switch (Optional but Recommended)

| Setting                     | Value         |
| --------------------------- | ------------- |
| Reverse Limit Switch Source | Hardware      |
| Reverse Limit Switch Type   | Normally Open |
| Forward Limit Switch        | Disabled      |

---

### Control Mode Usage

| Mode         | Use Case                 |
| ------------ | ------------------------ |
| Motion Magic | Automatic / preset climb |
| Duty Cycle   | Manual override          |

---

## General C++ Implementation Notes (Phoenix 6)

### Initialization

* Configure **all Talon settings once** at robot startup
* Do not reconfigure settings periodically
* Use **Brake mode**
* Apply **current limits** before enabling

---

### Homing / Zeroing

* On robot init or enable:

  * Drive slowly in reverse using **Duty Cycle**
  * Stop when reverse limit switch is triggered
  * Call `SetPosition(0.0)`
* Do not trust encoder position until homed

---

### Motion Magic Usage

* Command Motion Magic with **position in rotations**
* Example targets:

  * `0.0` → fully unwound / home
  * `+X` → climb height
* Motion Magic handles acceleration and velocity onboard

---

### Manual Override

* Use **Duty Cycle** for driver-controlled climbing
* Always keep current limits active
* Respect soft limits in code if possible

---

### Safety Recommendations

* Use **mechanical hard stops**
* Never rely on Brake mode alone to hold full robot weight indefinitely
* Assume **cord stretch and slip** over time
* Re-home when possible

---

### Tuning Notes

* Increase **kP** if the lift stalls or feels weak
* Decrease **kP** if oscillation occurs
* Keep **kI = 0** for climbers
* Motion Magic works best with a **single-layer wrap**
