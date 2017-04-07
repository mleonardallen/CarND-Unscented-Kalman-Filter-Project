project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp
   src/measurement_package.cpp
   src/laser_measurement_package.cpp
   src/radar_measurement_package.cpp)

add_executable(UnscentedKF ${sources})
