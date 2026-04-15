cmake_minimum_required(VERSION 3.10.0)
project(libmotion VERSION 0.1.0 LANGUAGES C CXX)
set(CXX_STANDARD 22)

add_subdirectory(LinearProfiles)
#add_subdirectory(TrapezoidalRamp)
add_subdirectory(tests)
include(CTest)
enable_testing()

