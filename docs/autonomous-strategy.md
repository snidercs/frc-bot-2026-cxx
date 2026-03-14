# Autonomous Strategy Notes

This page should explain how autonomous is structured and how to extend it.

## Current shape

Autonomous is built around PathPlanner. Named commands are registered in
`Container::Container()` in `container.cpp` and then referenced by path files
inside PathPlanner.

The default auto is set in `config.lua`:
```lua
auto_default_name = "Backup-Shoot-Left"
```

The active auto is selected via the SmartDashboard `AutoChooser` dropdown
(a `SendableChooser` registered in the container).

## Registered named commands

These are the commands PathPlanner paths can reference by name:

| Name | What it does |
|------|--------------|
| `shooterOn` | Spins up shooter and runs uptake; speed adjusts by distance to hub |
| `shooterOff` | Stops shooter flywheel and uptake |
| `turretAim` | Enables auto-aim, tracks hub using fused robot pose |
| `turretStop` | Stops all turret motors |
| `intakeStart` | Starts the intake |
| `intakeStutter` | Runs the intake in stutter mode (timed, for static-shot autos) |
| `intakeStop` | Stops the intake |
| `driveJitter` | Short front/back drive oscillation for intake agitation |

## Things to document here

- what each PathPlanner path file does
- assumptions each auto makes about starting pose
- when vision is used during auto
- how to test autos safely in sim and on carpet
- recovery behavior when a path fails or drifts
