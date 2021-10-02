cmake_minimum_required(VERSION 2.8)

project(sensor_hub)

add_definitions(-std=c++14 -fPIC)
set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC")
set(CMAKE_BUILD_TYPE "Release")

if (TEST_ENABLE)
    find_package(GTest MODULE REQUIRED)
    find_package(Threads)
    enable_testing()
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
    ${CMAKE_CURRENT_BINARY_DIR}/modules
    /usr/local/apollo/cyber/include/
)
add_subdirectory(modules)