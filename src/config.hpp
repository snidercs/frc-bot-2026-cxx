#pragma once

#include <string_view>

#include "scripting.hpp"

namespace config {
static constexpr bool USE_GAMEPAD = false;

// CAN IDs
static constexpr int INTAKE_TOP_MOTOR_ID = 14;
static constexpr int INTAKE_BOTTOM_MOTOR_ID = 15;

// The button index to use for resetting the heading direction.
static constexpr int HEADING_BUTTON_INDEX = 19;
// The button index to engage the intake.
static constexpr int INTAKE_TRIGGER_INDEX = 18;

/** Retrieves a numeric configuration value from Lua config.
 
    @tparam T The numeric type to retrieve (must be an integral or floating point type)
    @param key The configuration key to look up
    @return The configuration value as type T, or T(0) if the key is not found or has the wrong type
*/
template<typename T>
static double num (std::string_view key) {
    static_assert (std::is_integral_v<T> || std::is_floating_point_v<T>, 
        "T must be an integer or floating point type");
    auto val = lua::config::get (key);
    return val.is<T>() ? val.as<T>() : T(0);
}

/** Retrieves a floating-point configuration value from Lua config.
 
    @param key The configuration key to look up
    @return The configuration value as a double, or 0.0 if not found or has the wrong type
*/
static double number (std::string_view key) { return num<double> (key); }

/** Retrieves an integer configuration value from Lua config.
 
    @param key The configuration key to look up
    @return The configuration value as an int, or 0 if not found or has the wrong type
*/
static int integer (std::string_view key) { return num<int> (key); }

} // namespace config
