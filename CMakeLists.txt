cmake_minimum_required(VERSION 3.22)
project(planar_robot_kinematics VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable compiler warnings and binary hardening flags
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -fstack-protector-strong -D_FORTIFY_SOURCE=2)
endif()

# Core Kinematics Library
add_library(kinematics_core STATIC
    src/manipulator2d.cpp
)

set_target_properties(kinematics_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(kinematics_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Catch2 Testing
find_package(Catch2 3 REQUIRED)

enable_testing()

add_executable(kinematics_tests
    tests/test_kinematics.cpp
)

target_link_libraries(kinematics_tests PRIVATE
    kinematics_core
    Catch2::Catch2WithMain
)

include(CTest)
add_test(NAME kinematics_tests COMMAND kinematics_tests)

# Fetch pybind11 using FetchContent
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        3e9dfa2866941655c56877882565e7577de6fc7b
)
FetchContent_MakeAvailable(pybind11)

# Pybind11 extension module
pybind11_add_module(kinematics_core_py
    src/bindings.cpp
)

target_link_libraries(kinematics_core_py PRIVATE
    kinematics_core
)

# Output compiled module .so into build directory
set_target_properties(kinematics_core_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
