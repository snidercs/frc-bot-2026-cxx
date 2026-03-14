# FRC Bot 2026 C++ Docs

Welcome to the robot docs.

This folder is meant to be the team's shared reference book: setup notes,
calibration steps, subsystem ideas, and planning docs that explain not just
*what* the code does, but *why* we made certain choices.

If you're new to the project, start with the docs below before digging too far
into the code. It'll save you time.

## Start here

These are the most useful docs for understanding the current robot work.

- [`architecture-overview.md`](./architecture-overview.md) — the big-picture map of how the robot code is organized
- [`photonvision-calibration.md`](./photonvision-calibration.md) — how the PhotonVision cameras are calibrated and what to check when vision looks off
- [`posescaling.md`](./posescaling.md) — notes on pose scaling, field position interpretation, and related math ideas

## Handbook pages

These are the core reference pages that should grow into the team's day-to-day
documentation set.

- [`architecture-overview.md`](./architecture-overview.md) — overall code structure and major subsystem responsibilities
- [`can-and-electrical-map.md`](./can-and-electrical-map.md) — wiring, CAN IDs, bus names, and electrical reference notes
- [`controls-guide.md`](./controls-guide.md) — driver and operator controls in plain language
- [`shooter-and-turret-tuning.md`](./shooter-and-turret-tuning.md) — tuning notes, shot behavior, and aiming-related details
- [`autonomous-strategy.md`](./autonomous-strategy.md) — auto structure, named commands, and planning notes
- [`vision-pipeline-overview.md`](./vision-pipeline-overview.md) — camera-to-pose-fusion overview and vision design rules
- [`troubleshooting-checklist.md`](./troubleshooting-checklist.md) — quick match-day debugging checklist

## Working plans

These are design notes and scratchpad-style docs for changes we've been
thinking through or actively working on.

- [`plans/lifterhoming.md`](./plans/lifterhoming.md) — lifter homing behavior, assumptions, and implementation direction
- [`plans/turrentrange.md`](./plans/turrentrange.md) — turret rotation range thoughts and possible update work
- [`plans/visionsync.md`](./plans/visionsync.md) — ideas around vision sync, correction behavior, and estimator tuning

## How to use this folder

- Use one file per topic when possible
- Keep work-in-progress design notes under [`docs/plans/`](./plans/)
- Link related pages instead of repeating the same explanation everywhere
- Capture gotchas, tuning notes, and lessons learned while they're still fresh

## Recommended reading order

If you're trying to get up to speed quickly, a good order is:

1. [`architecture-overview.md`](./architecture-overview.md)
2. [`controls-guide.md`](./controls-guide.md)
3. [`vision-pipeline-overview.md`](./vision-pipeline-overview.md)
4. [`photonvision-calibration.md`](./photonvision-calibration.md)
5. subsystem- or feature-specific pages as needed
