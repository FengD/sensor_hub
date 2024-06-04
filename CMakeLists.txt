cmake_minimum_required(VERSION 3.8)

project(sensor_hub)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC")
set(CMAKE_BUILD_TYPE "Release")
set(DEPENDENCIES_PATH ${CMAKE_CURRENT_BINARY_DIR}/modules)

## af2.0
if (WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  if(NOT rclcpp_FOUND)
    message(WARNING "rclcpp Not Found")
  endif()
  find_package(beatles REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(af_interfaces REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)

  set(sensor_msg_DIR ${CMAKE_CURRENT_BINARY_DIR}/modules/crdc_airi_common/share/sensor_msg/cmake)
  find_package(sensor_msg REQUIRED)
  if(NOT sensor_msg_FOUND)
    message(WARNING "sensor_msg Not Found")
  endif()

  find_package(diagnostic_msgs REQUIRED)
  if(NOT diagnostic_msgs_FOUND)
    message(WARNING "diagnostic_msgs Not Found")
  endif()
  find_package(sensor_msgs REQUIRED)
  if(NOT sensor_msgs_FOUND)
    message(WARNING "sensor_msgs Not Found")
  endif()
endif()

## GTest
find_package(GTest)
if (NOT GTEST_FOUND)
    message(WARNING "GTest Not Found")
endif()

if (NOT DO_TEST)
    message(STATUS "DO_TEST OFF")
    set(GTEST_FOUND False)
else()
    message(STATUS "DO_TEST ON")
endif()


if (GTEST_FOUND)
  add_definitions(-DGTEST_FOUND)
endif()

# code converage
if (WITH_COV)
  message(STATUS "WITH_COV ON")
  add_compile_options(--coverage -fno-inline -fno-inline-small-functions -fno-default-inline -fprofile-arcs -ftest-coverage)
  link_libraries(gcov)
  add_definitions(-DWITH_TEST)
else()
  message(STATUS "WITH_COV OFF")
endif()

# enable ctest
if (DO_TEST)
enable_testing()
endif()

add_subdirectory(modules)
if(WITH_ROS2)
ament_package()
endif()