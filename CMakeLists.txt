cmake_minimum_required(VERSION 3.8)

project(sensor_hub)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC")
set(CMAKE_BUILD_TYPE "Release")
set(DEPENDENCIES_PATH ${CMAKE_CURRENT_BINARY_DIR}/modules)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ROS2 FLAGS
if (WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  if(NOT rclcpp_FOUND)
    message(WARNING "rclcpp Not Found")
  endif()
  find_package(class_loader REQUIRED)
  find_package(rosidl_runtime_cpp REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)

  find_package(sensor_msgs REQUIRED)
  if(NOT sensor_msgs_FOUND)
    message(WARNING "sensor_msgs Not Found")
  endif()
  find_package(diagnostic_msgs REQUIRED)
  if(NOT diagnostic_msgs_FOUND)
    message(WARNING "diagnostic_msgs Not Found")
  endif()

  include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/modules/sensor_msg/rosidl_generator_cpp/
  )
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
