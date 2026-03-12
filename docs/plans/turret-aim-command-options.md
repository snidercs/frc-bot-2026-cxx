# Turret Auto-Aim Command: PathPlanner Hang Fix

## Problem

`aimAtTargetCommand()` is built on `Run(...)`, which **never sets `IsFinished() = true`**.
It only stops when interrupted externally — which is what happens during teleop when the
button is released (`WhileTrue` cancels it). In PathPlanner, a named command just waits
for `IsFinished()` forever, so auto hangs indefinitely at the `turretAim` step.

---

## Option A — Timeout Wrapper (Simplest, No Code Changes)

Register the named command with a timeout:

```cpp
nc::registerCommand("turretAim", turret().aimAtTargetCommand(
    [this] { return drivetrain().GetState().Pose; },
    [this] { return frc::Pose2d{landmarks::hubPosition(), frc::Rotation2d{}}; }
).WithTimeout(2_s));
```

**Pros:**
- Zero changes to `turret.cpp` / `turret.hpp`
- Configurable per auto routine directly in PathPlanner via the timeout parameter

**Cons:**
- Auto always waits the full timeout even if the turret is on-target in 0.3s
- Doesn't give PathPlanner any signal that aiming actually succeeded

---

## Option B — Finish on `isAtTarget()` (Semantically Correct, Single Command)

Change `aimAtTargetCommand` to use `RunUntil` so it finishes when the turret settles:

```cpp
return RunOnce([this] { enableAutoAim(); })
    .AndThen(
        RunUntil([this] { return isAtTarget(); })
            .DeadlineFor(Run([this, robotPoseSupplier, targetPoseSupplier] {
                auto aimPos = computeAimPosition(robotPoseSupplier(), targetPoseSupplier());
                setTargetPosition(aimPos);
            }))
    )
    .FinallyDo([this] { disableAutoAim(); })
    .WithName("AimAtTarget");
```

Add a safety timeout at the call site:

```cpp
nc::registerCommand("turretAim", turret().aimAtTargetCommand(...).WithTimeout(3_s));
```

**Pros:**
- Auto moves on as soon as the turret is settled — no wasted time
- Single command, consistent behavior everywhere

**Cons:**
- Changes the teleop `WhileTrue` behavior: once on-target the command finishes and the
  scheduler re-runs `manualRotateCommand` as the default. The turret would drift back to
  hold position until the button is pressed again. This may or may not be desirable.
- More complex command composition

---

## Option C — Two Separate Commands (Recommended)

Keep the existing `aimAtTargetCommand()` for teleop exactly as-is. Add a new
`aimUntilOnTargetCommand()` for PathPlanner that finishes when `isAtTarget()` is true,
with a built-in safety timeout.

### `turret.hpp` addition
```cpp
/** Aims at a target and finishes once the turret is on-target (or timeout expires).
    Intended for use as a PathPlanner named command.

    @param robotPoseSupplier  Supplier returning current robot field pose.
    @param targetPoseSupplier Supplier returning target field pose.
    @param timeout            Maximum time to wait before giving up. Default 3s.
    @return CommandPtr that completes when isAtTarget() or timeout is reached.
*/
frc2::CommandPtr aimUntilOnTargetCommand(
    std::function<frc::Pose2d()> robotPoseSupplier,
    std::function<frc::Pose2d()> targetPoseSupplier,
    units::second_t timeout = 3_s);
```

### `turret.cpp` addition
```cpp
frc2::CommandPtr Turret::aimUntilOnTargetCommand(
    std::function<frc::Pose2d()> robotPoseSupplier,
    std::function<frc::Pose2d()> targetPoseSupplier,
    units::second_t timeout)
{
    return RunOnce([this] { enableAutoAim(); })
        .AndThen(
            Run([this, robotPoseSupplier, targetPoseSupplier] {
                auto aimPos = computeAimPosition(robotPoseSupplier(), targetPoseSupplier());
                setTargetPosition(aimPos);
            })
            .Until([this] { return isAtTarget(); })
            .WithTimeout(timeout)
        )
        .FinallyDo([this] { disableAutoAim(); })
        .WithName("AimUntilOnTarget");
}
```

### `container.cpp` change
```cpp
nc::registerCommand("turretAim", turret().aimUntilOnTargetCommand(
    [this] { return drivetrain().GetState().Pose; },
    [this] { return frc::Pose2d{landmarks::hubPosition(), frc::Rotation2d{}}; }
));
```

**Pros:**
- Teleop `WhileTrue` behavior is completely unchanged
- Auto finishes as soon as the turret settles, not after a fixed delay
- Built-in 3s safety timeout prevents infinite hang if something goes wrong
- Clear naming distinguishes intent at each call site

**Cons:**
- Two command factory methods to maintain (they share `computeAimPosition` so divergence
  risk is low)

---

## Recommendation

**Option C.** Preserves teleop behavior exactly, gives auto a natural finish condition,
and the safety timeout prevents the original hang in all failure cases.
