project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++14)
add_definitions(-stdlib=libc++)

set(sources
    src/FusionEKF.cpp
        src/ExtendedKalmanFilter.cpp
    src/main.cpp
    src/tools.cpp)

add_executable(ExtendedKF ${sources})
