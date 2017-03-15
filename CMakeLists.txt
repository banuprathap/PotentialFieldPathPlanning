PROJECT(Planner)

CMAKE_MINIMUM_REQUIRED(VERSION 3.2.1)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")
set(CMAKE_CXX_STANDARD 11)


#############################################################################
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)