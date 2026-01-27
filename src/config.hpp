#pragma once

#include <iostream>
#include <string_view>

#include "scripting.hpp"

namespace config {

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

static bool boolean (std::string_view key, bool fallback = false) {
    auto val = lua::config::get (key);
    return val.is<bool>() ? val.as<bool>() : fallback;
}

/** Retrieves a string configuration value from Lua config. Note that
    this function will not convert non-strings to string.
 
    @param key The configuration key to look up
    @return The configuration value as a string, or an empty string if not found or has the wrong type
*/
static std::string str (std::string_view key) {
    auto val = lua::config::get (key);
    return val.is<std::string>() ? val.as<std::string>() : std::string();
}

/** Displays all configuration settings from the Lua config table.
 
    Iterates through all string keys in the config table and logs each one
    using lua::config::log(). Output is bracketed with begin/end markers.
*/
static void display() {
    auto& L { lua::state() };
    sol::table config = L["config"];

    std::cout << "[config] begin settings\n";
    for (const auto& [key, value] : config) {
        if (key.is<std::string>()) {
            lua::config::log(key.as<std::string>());
        }
    }
    std::cout << "[config] end settings\n";
}

} // namespace config
