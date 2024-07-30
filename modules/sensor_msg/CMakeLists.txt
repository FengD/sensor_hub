project(sensor_msg)
cmake_minimum_required(VERSION 3.8)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

FILE(GLOB msg_files
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/
  msg/*.msg
)

FILE(GLOB srv_files
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/
  srv/*.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES std_msgs
  DEPENDENCIES sensor_msgs
)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
