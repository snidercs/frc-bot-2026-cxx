
#include "config.hpp"

// enable to log details about config queries
#define GS_TRACE_CONFIG 0

namespace config {
namespace detail {
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
}

void log (std::string_view key) {
    auto& L { lua::state() };
    sol::object val = L["config"][key];
    detail::log (key, val);
}

sol::object get (std::string_view key) {
    auto& L { lua::state() };
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
}
