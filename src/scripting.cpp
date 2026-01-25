
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>

#include <frc/Filesystem.h>

#include "scripting.hpp"

extern "C" {
#include "luajit.h"
}

// enable to log details about config queries
#define GS_TRACE_CONFIG 0

namespace fs = std::filesystem;

namespace lua {

namespace detail {

static sol::state* _state { nullptr };
static bool boostraped { false };
static std::string path;
static std::string search_dir;

static void init() {
    if (_state != nullptr)
        return;
    _state = new sol::state();
    _state->open_libraries();
}

static void destroy() {
    if (_state == nullptr)
        return;
    delete _state;
    _state = nullptr;
}

static bool hasCustomPath() { return ! path.empty(); }

static char separator() {
#ifdef _WIN32
    return '\\';
#else
    return '/';
#endif
}

/** Adds lua path qualifiers. e.g. ?.lua and /?/init.lua  to the input string.  
    Input does not get modified.*/
std::string withSearchQualifiers (std::string_view input) {
    std::stringstream out;
    out << input << detail::separator() << "?.lua;"
        << detail::separator() << "?" << detail::separator() << "init.lua";
    return out.str();
}

} // namespace detail

//==============================================================================
Lifecycle::Lifecycle() {
    if (detail::_state != nullptr) {
        detail::destroy();
        throw std::runtime_error ("lua::Lifecycle already exists.");
    }
    detail::init();
}

Lifecycle::~Lifecycle() { detail::destroy(); }

//==============================================================================
sol::state& state() {
    if (detail::_state == nullptr)
        throw std::runtime_error ("Lua state was not initialized");
    return *detail::_state;
}

void printVersion() {
    fputs (LUAJIT_VERSION " -- " LUAJIT_COPYRIGHT ". " LUAJIT_URL "\n", stdout);

    auto L = state().lua_state();
    int n;
    const char* s;
    lua_getfield (L, LUA_REGISTRYINDEX, "_LOADED");
    lua_getfield (L, -1, "jit"); /* Get jit.* module table. */
    lua_remove (L, -2);
    lua_getfield (L, -1, "status");
    lua_remove (L, -2);
    n = lua_gettop (L);
    lua_call (L, 0, LUA_MULTRET);
    fputs (lua_toboolean (L, n) ? "JIT: ON" : "JIT: OFF", stdout);
    for (n++; (s = lua_tostring (L, n)); n++) {
        putc (' ', stdout);
        fputs (s, stdout);
    }
    putc ('\n', stdout);
    lua_settop (L, 0); /* clear stack */
}

void setPath (std::string_view path) {
    if (path.empty())
        return;
    auto& L            = lua::state();
    detail::search_dir = path;
    detail::search_dir.shrink_to_fit();
    detail::path = detail::withSearchQualifiers (path);
    detail::path.shrink_to_fit();
    sol::table package = L["package"];
    package.set ("path", detail::path);
}

const std::string& searchDirectory() {
    return detail::search_dir;
}

bool bootstrap() {
    if (detail::boostraped)
        return true;

    if (! detail::hasCustomPath()) {
        setPath ([]() -> std::string {
            fs::path path;

            path = frc::filesystem::GetDeployDirectory();

            if (! fs::exists (path / "config.lua")) {
                path = frc::filesystem::GetOperatingDirectory();
                path /= "robot";
            }

            if (! fs::exists (path / "config.lua")) {
                path = frc::filesystem::GetLaunchDirectory();
                path /= "robot";
            }

            path.make_preferred();

            if (! fs::exists (path / "config.lua"))
                return "";

            std::clog << "[bot] bootstrap: lua path: " << path.string() << std::endl;
            return path.string();
        }());
    }

    auto& ls = state();
    sol::safe_function_result result;

    try {
        result = ls.script (R"(
            config = require ('config')
        )");
    } catch (const std::exception& e) {
        std::cerr << "[bot] " << e.what() << std::endl;
        return false;
    }

    if (! result.valid()) {
        sol::error err = result;
        std::cerr << "[bot] " << err.what() << std::endl;
        return false;
    }

    detail::boostraped = true;
    return detail::boostraped;
}

namespace config {

static void log (std::string_view key, const sol::object& val) {
    if (val.valid()) {
        std::string strVal = val.as<std::string>();
        if (val.is<bool>())
            strVal = val.as<bool>() ? "true" : "false";
        if (val.is<std::string>())
            strVal = "\"" + strVal + "\"";
        std::cout << "[config] " << key << " = " << strVal << std::endl;
    } else {
        std::cout << "[config] unknown key: " << key << std::endl;
    }
}

void log (std::string_view key) {
    auto& L { state() };
    sol::object val = L["config"][key];
    log (key, val);
}

sol::object get (std::string_view key) {
    auto& L { state() };
    sol::object obj = L["config"][key];
#if GS_TRACE_CONFIG
    log (key, obj);
#endif
    return obj;
}

sol::object get (std::string_view category, std::string_view symbol) {
    if (category.empty() || symbol.empty())
        return sol::object {};

    sol::object obj = get (category);
    if (obj.is<sol::table>()) {
        auto cat = obj.as<sol::table>();
        return cat[symbol];
    }

    return {};
}

} // namespace config
} // namespace lua
