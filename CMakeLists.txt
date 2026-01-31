cmake_minimum_required(VERSION 3.10.0)
project(libmotion VERSION 0.1.0 LANGUAGES C CXX)

add_subdirectory(constant_velocity)
add_subdirectory(trapezoidal_ramp)
add_subdirectory(tests)
include(CTest)
enable_testing()

