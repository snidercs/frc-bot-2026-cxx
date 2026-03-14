# Driver and Operator Controls Guide

This page explains what each control does in plain language.
The robot defaults to **flightstick mode** (`gamepad = false` in `config.lua`).
Two joysticks are used: **Stick 0** (driver, left) and **Stick 1** (operator, right).

## Flightstick controls (default)

### Stick 0 — Driver

| Button / Axis | Action | Behaviour |
|---------------|--------|-----------|
| Axis 0 (X) | Strafe left / right | WhileTrue default drive |
| Axis 1 (Y) | Drive forward / back | WhileTrue default drive |
| Button 4 | Reset pose to alliance corner | OnTrue — snaps to back-left (blue) or back-right (red) |
| Button 5 | Drive jitter front/back | OnTrue — short oscillation for intake agitation |
| Button 6 | Drive jitter left/right | OnTrue — short oscillation for intake agitation |
| Button 8 (`heading_button_index`) | Reset field-centric heading | OnTrue |
| Button 16 (`climber_climb_button_index`) | Climber climb | WhileTrue |
| Button 1 (`climber_lower_button_index`) | Climber lower | WhileTrue |
| Button 18 (`intake_trigger_index`) | Intake + shaker (0.1) | WhileTrue (parallel) |

### Stick 1 — Operator

| Button / Axis | Action | Behaviour |
|---------------|--------|-----------|
| Axis 0 (X, right stick) | Manual turret rotation | Default command, scaled by `turret_roation_gain = 0.1` |
| Button 3 | Zero turret rotation | OnTrue — resets motor position to 0 |
| Button 4 | Disable climber soft limits | OnTrue/OnFalse — hold to disable, release re-enables and resets |
| Button 16 | Auto-aim toggle | ToggleOnTrue — tracks hub using fused robot pose |
| Button 17 | Shaker oscillate toggle | ToggleOnTrue — oscillates at 0.4 duty cycle |
| Button 18 (`turret_shoot_button_index`) | Shoot + shaker (0.3) | WhileTrue (parallel) — adjusts speed by distance |
| Button 19 (`intake_eject_index`) | Eject intake | WhileTrue |

## Gamepad mode

Gamepad mode (`gamepad = true` in `config.lua`) uses a single Xbox controller (port 0).
The gamepad bindings are defined in `GamepadContainer` in `container.cpp`.
They are primarily for driving and intake; shooter controls are not fully bound in
gamepad mode. Check `container.cpp` for the current state.

## Notes

- Controls marked **WhileTrue** run while held and stop when released
- **ToggleOnTrue** toggles: first press enables, second press (or interruption) disables
- Shooter and intake run in parallel with the shaker using `.AlongWith()` — see [`architecture-overview.md`](./architecture-overview.md)
- The shaker subsystem uses `.AsProxy()` to avoid command conflicts when both are active simultaneously
