// Copyright (c) 2025 Michael Fisher @mfisher31
// SPDX-License-Identifier: MIT

#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <string_view>

#include <frc/RobotBase.h>
#include <hal/Main.h>
#include <lua.hpp>

#include <luabot/luabot.h>

#if defined(__GNUC__) || defined(__clang__)
#    define LUABOT_FUNCTION __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#    define LUABOT_FUNCTION __FUNCSIG__
#else
#    define LUABOT_FUNCTION __func__
#endif

#ifndef LUABOT_DEVELOPMENT
#    define LUABOT_DEVELOPMENT 0
#endif

namespace luabot {
namespace detail {

void run_lua_robot (std::mutex& m, lua_State** L_ptr, const char* lua_file) {
    const bool already_open = *L_ptr != nullptr;
    lua_State* L = already_open ? *L_ptr : luaL_newstate();

    if (! already_open) {
        luabot_set_default_paths (L);
        luaL_openlibs (L);
    }
    try {
#if LUABOT_DEVELOPMENT
        // Set LUA_PATH to include build/lua directory
        lua_getglobal (L, "package");
        lua_getfield (L, -1, "path");
        std::string current_path { lua_tostring (L, -1) };
        lua_pop (L, 1);
        std::string new_path = "./build/lua/?.lua;./build/lua/?/init.lua;" + current_path;
        lua_pushstring (L, new_path.c_str());
        lua_setfield (L, -2, "path");
        lua_pop (L, 1);
#endif

        if (! already_open) {
            std::scoped_lock lock { m };
            *L_ptr = L;
        }

        // Load and execute the Lua file as a module
        if (luaL_loadfile (L, lua_file) != 0) {
            const char* err     = lua_tostring (L, -1);
            std::string err_msg = err ? err : "unknown error";
            throw std::runtime_error (err_msg);
        }

        // Execute the file
        if (lua_pcall (L, 0, 1, 0) != 0) {
            const char* err     = lua_tostring (L, -1);
            std::string err_msg = err ? err : "unknown error";
            throw std::runtime_error (err_msg); // ("failed to create module");
        }

        // The module should return a table
        if (! lua_istable (L, -1)) {
            throw std::runtime_error ("Module did not return a table");
        }

        // Get the 'new' function from the returned table
        lua_getfield (L, -1, "new");
        if (! lua_isfunction (L, -1)) {
            throw std::runtime_error ("Module table missing 'new' function");
        }

        // Call module.new() to create the robot instance
        if (lua_pcall (L, 0, 1, 0) != 0) {
            const char* err     = lua_tostring (L, -1);
            std::string err_msg = err ? err : "unknown error";
            // std::cerr << "Error calling new(): " << err_msg << std::endl;
            throw std::runtime_error (err_msg);
        }

        // Store the robot instance in the registry to keep it alive
        lua_pushvalue (L, -1); // Duplicate the robot instance
        lua_setfield (L, LUA_REGISTRYINDEX, "robot_instance");

        // Now we have the robot instance on the stack
        // Get the startCompetition method
        lua_getfield (L, -1, "startCompetition");
        if (! lua_isfunction (L, -1)) {
            // std::cerr << "Error: Robot instance must have a startCompetition method" << std::endl;
            throw std::runtime_error ("Robot missing startCompetition method");
        }

        // Push the robot instance as 'self' for the method call
        lua_pushvalue (L, -2); // Duplicate the robot instance

        // Call instance:startCompetition()
        if (lua_pcall (L, 1, 0, 0) != 0) {
            const char* err     = lua_tostring (L, -1);
            std::string err_msg = err ? err : "unknown error";
            throw std::runtime_error (err_msg);
        }

        // startCompetition() has returned - call endCompetition before cleanup
        lua_getfield (L, LUA_REGISTRYINDEX, "robot_instance");
        if (lua_istable (L, -1)) {
            lua_getfield (L, -1, "endCompetition");
            if (lua_isfunction (L, -1)) {
                lua_pushvalue (L, -2);
                lua_pcall (L, 1, 0, 0);
            }
        }

        // Clean up Lua state
        if (! already_open) {
            lua_close (L);
            std::scoped_lock lock { m };
            *L_ptr = nullptr;
        }

    } catch (const std::exception& e) {
        auto hal_msg = std::string ("[luabot]: ") + std::string (e.what());
        HAL_SendError (1, frc::err::Error, 0, hal_msg.c_str(), "", "", 1);
        if (L && already_open) {
            lua_close (L);
            std::scoped_lock lock { m };
            *L_ptr = nullptr;
        }
    }
}

} // namespace detail

int start_robot (std::string_view lua_file, lua_State* state = nullptr) {
    int halInit = frc::RunHALInitialization();
    if (halInit != 0) {
        return halInit;
    }

    static std::mutex m;
    static std::condition_variable cv;
    static lua_State* L = state;
    static bool exited  = false;

    if (HAL_HasMain()) {
        std::thread thr ([lua_file] {
            try {
                detail::run_lua_robot (m, &L, lua_file.data());
            } catch (const std::exception& e) {
                std::cerr << "[luabot]: " << e.what() << std::endl;
                HAL_ExitMain();
                {
                    std::scoped_lock lock { m };
                    L      = nullptr;
                    exited = true;
                }
                cv.notify_all();
                return; // Don't re-throw from thread
            } catch (...) {
                std::cerr << "[luabot]: Unknown exception in robot thread" << std::endl;
                HAL_ExitMain();
                {
                    std::scoped_lock lock { m };
                    L      = nullptr;
                    exited = true;
                }
                cv.notify_all();
                return; // Don't re-throw from thread
            }

            HAL_ExitMain();
            {
                std::scoped_lock lock { m };
                L      = nullptr;
                exited = true;
            }
            cv.notify_all();
        });

        HAL_RunMain();

        // Prefer to join, but detach to exit if it doesn't exit in a timely manner
        using namespace std::chrono_literals;
        std::unique_lock lock { m };
        if (cv.wait_for (lock, 1s, [] { return exited; })) {
            thr.join();
        } else {
            thr.detach();
        }
    } else {
        detail::run_lua_robot (m, &L, lua_file.data());
    }

#ifndef __FRC_ROBORIO__
    frc::impl::ResetMotorSafety();
#endif
    HAL_Shutdown();

    return 0;
}

} // namespace luabot
