project(ekf_tracker)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/ekf_tracker.cpp
    src/main.cpp)

add_executable(ekf_tracker ${sources})
