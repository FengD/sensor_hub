cmake_minimum_required(VERSION 2.8)

project(sensor)

file(GLOB SENSOR_SRCS *.cc)
if(WITH_ROS2)
    find_package(sensor_msgs REQUIRED)
    if(NOT sensor_msgs_FOUND)
        message(WARNING "sensor_msgs Not Found")
    endif()
    find_package(sensor_msg REQUIRED)
    if(NOT sensor_msg_FOUND)
        message(WARNING "sensor_msg Not Found")
    endif()
endif()
add_library(${PROJECT_NAME} STATIC ${SENSOR_SRCS})
if(WITH_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msgs
    sensor_msg
)
endif()
