#include <filesystem>
#include <hal/HAL.h>
#include "gtest/gtest.h"
#include "scripting.hpp"

static lua::Lifecycle sLuaEngine;

namespace detail {
inline static void initLuaPath()
{
    std::filesystem::path path (__FILE__);
    path = path.parent_path()
               .parent_path()
               .parent_path();
    path /= "robot";
    path.make_preferred();
    lua::setPath (path.string());
}
} // namespace detail

int main (int argc, char** argv)
{
    detail::initLuaPath();
    if (! lua::bootstrap()) {
        std::cerr << "[bot] could not initialize lua for test suite." << std::endl;
        return 1;
    }

    HAL_Initialize (500, 0);
    ::testing::InitGoogleTest (&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
