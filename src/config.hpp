#pragma once

namespace config {
static constexpr bool USE_GAMEPAD = false;

// CAN IDs
static constexpr int INTAKE_TOP_MOTOR_ID = 14;
static constexpr int INTAKE_BOTTOM_MOTOR_ID = 15;

// The button index to use for resetting the heading direction.
static constexpr int HEADING_BUTTON_INDEX = 19;
// The button index to engage the intake.
static constexpr int INTAKE_TRIGGER_INDEX = 18;
} // namespace config
