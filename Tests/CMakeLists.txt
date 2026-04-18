include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

add_library(vcd_tracer vcd_tracer.cpp)

add_executable(test_motion test.cpp)
target_link_libraries(test_motion
    PRIVATE
        motion_linear_profiles
        GTest::gtest_main
        vcd_tracer
)

include(GoogleTest)
gtest_discover_tests(test_motion)
