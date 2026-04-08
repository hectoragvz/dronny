cmake_minimum_required(VERSION 3.20)
project(dronny LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(libs/math)
add_subdirectory(sim)
add_subdirectory(control)

add_executable(test_math tests/math.cpp)
add_executable(sim_main apps/sim_main.cpp)
target_link_libraries(test_math PRIVATE math_lib sim control)
target_link_libraries(sim_main PRIVATE math_lib sim control)
