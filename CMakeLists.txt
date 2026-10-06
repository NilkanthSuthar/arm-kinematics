cmake_minimum_required(VERSION 3.16)

project(arm_kinematics
    VERSION 0.1.0
    DESCRIPTION "Kinematics for a 2-link planar robot arm"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# clang-tidy reads the compile flags from build/compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ARM_ENABLE_COVERAGE "Instrument the build for gcov/lcov coverage (GCC/Clang only)" OFF)

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(arm src/arm.cpp)
target_include_directories(arm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(MSVC)
    target_compile_options(arm PRIVATE /W4 /WX)
else()
    target_compile_options(arm PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror)
endif()

if(ARM_ENABLE_COVERAGE)
    target_compile_options(arm PUBLIC --coverage -O0 -g)
    target_link_options(arm PUBLIC --coverage)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
include(CTest)

if(BUILD_TESTING)
    include(FetchContent)

    # Avoid the "DOWNLOAD_EXTRACT_TIMESTAMP" warning on newer CMake versions
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()

    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    )
    # Use the same runtime library as the rest of the project on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(test_arm tests/test_arm.cpp)
    target_link_libraries(test_arm PRIVATE arm GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(test_arm)
endif()
