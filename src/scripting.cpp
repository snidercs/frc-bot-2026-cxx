
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>

#include <frc/Filesystem.h>
#include <frc/RobotBase.h>

#include "scripting.hpp"

extern "C" {
#include "luajit.h"
}

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
        << input << detail::separator() << "?" << detail::separator() << "init.lua";
    return out.str();
}

static std::string luabotDir() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        return "/home/lvuser";
    }
    
    fs::path path (frc::filesystem::GetLaunchDirectory());
    path /= "build/luabot/share/luajit-2.1";
    path.make_preferred();
    return path.string();
}

static std::string robotDir() {

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

    if (! fs::exists (path / "config.lua")) {
        std::cout << "[bot] lua path doesn't exist: " << path.string() << std::endl;
        return "";
    }

    std::cout << "[bot] bootstrap: lua path: " << path.string() << std::endl;
    return path.string();
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
    std::cout << "PATH=" << path << std::endl;
    sol::table package = L["package"];
    package.set ("path", detail::path);
}

void setPath (const std::vector<std::string>& paths) {
    std::stringstream strm;
    
    bool first = true;
    for (const auto& entry : paths) {
        if (!first) {
            strm << (char) ';';
        }
        // Convert to fs::path to get correct directory separators
        strm << detail::withSearchQualifiers (fs::path(entry).string());
        first = false;
    }
    
    const std::string path { strm.str() };
    if (path.empty())
        return;
    auto& L            = lua::state();
    detail::search_dir = path;
    detail::search_dir.shrink_to_fit();
    detail::path = path;
    detail::path.shrink_to_fit();
    std::cout << "PATH=" << path << std::endl;
    sol::table package = L["package"];
    package.set ("path", detail::path);

    setPath(strm.str());
}

const std::string& searchDirectory() {
    return detail::search_dir;
}

bool bootstrap() {
    if (detail::boostraped)
        return true;

    if (! detail::hasCustomPath()) {
        std::vector<std::string> path;

        path.insert (path.begin(), {
            detail::luabotDir(),
            detail::robotDir()
        });

        setPath (path);
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

} // namespace config
} // namespace lua
