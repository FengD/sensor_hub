project(lidar_dynamic_calibrate)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
FILE(GLOB SRCS dynamic_calibration.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})

target_link_libraries(${PROJECT_NAME}
    common
    glog
    gflags
    pthread
    fastrtps
)

if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msg
    sensor_msgs
    visualization_msgs
    geometry_msgs
)
endif()

target_link_libraries(${PROJECT_NAME}
    service_server
)