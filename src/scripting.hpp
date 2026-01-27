#pragma once

#include <string>
#include <string_view>

#include "sol/sol.hpp"

namespace lua {

/** Initialize and destroy Lua with RAII pattern. Instantiating this class more 
    than once will throw a runtime exception. Using lua::state() before the 
    Lifecycle is present will crash badly.
 */
struct Lifecycle final {
    Lifecycle();
    ~Lifecycle();

    // no copy
    Lifecycle (const Lifecycle&)            = delete;
    Lifecycle& operator= (const Lifecycle&) = delete;
    // no move
    Lifecycle (Lifecycle&&)            = delete;
    Lifecycle& operator= (Lifecycle&&) = delete;
};

/** Returns the global Lua context. */
sol::state& state();

/** Prints the Lua version to console. */
void printVersion();

/** Set the Lua search path.
    Does nothing if passed an empty string.
    @param path A non empty Lua path string.
*/
void setPath (std::string_view path);

/** Returns the search directory for Lua. */
const std::string& searchDirectory();

/** Bootstrap the interpreter (call once before robot init)
    @returns true if Lua could be bootstrapped.
*/
bool bootstrap();

} // namespace lua
