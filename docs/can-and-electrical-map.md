# CAN IDs and Electrical Map

This page should become the quick-reference sheet for wiring and CAN layout.

## Purpose

Use this doc to answer questions like:

- Which device ID belongs to which motor?
- What bus is each controller on?
- What should we check when a motor will not come online?

## Current CAN device table

All values sourced from `robot/config.lua`.

| Subsystem | Role | CAN ID | Bus |
|-----------|------|--------|-----|
| Intake | Top motor | 14 | rio |
| Intake | Bottom motor | 15 | rio |
| Climber | Climber motor | 1 | rio |
| Turret | Rotation motor | 19 | rio |
| Turret | Shooter flywheel | 16 | rio |
| Turret | Uptake motor | 21 | rio |
| Shaker | Shaker motor | 35 | rio |
| Horizontal Shaker | Horizontal shaker motor | 22 | rio |

> **Note:** Swerve drive module CAN IDs are defined in `generated/TunerConstants.h`,
> not in `config.lua`. Update that table separately.

## Good things to also document here

- swerve drive module device IDs (from `TunerConstants.h`)
- PDP/PDH channel assignments
- breaker sizes if they matter for debugging
- any known flaky connectors or field-repair notes

## Common pitfall checklist

- duplicate CAN ID
- wrong CAN bus name in config
- loose power or CAN connector
- motor exists in code but was replaced physically
