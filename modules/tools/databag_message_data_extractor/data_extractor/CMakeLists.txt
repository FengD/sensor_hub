cmake_minimum_required(VERSION 2.8)

project(data_extractor)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

file(GLOB CYBERRT_SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${CYBERRT_SRCS})
if(WITH_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msg
    sensor_msgs
)
else()
    target_link_libraries(${PROJECT_NAME}
    cyber
    fastrtps
)
endif()
target_link_libraries(${PROJECT_NAME}
    glog
    gflags    
    opencv_core
    opencv_highgui
    opencv_video
    opencv_imgproc
    opencv_imgcodecs
    ${PROTOBUF_LIBRARIES}
    ${PROTOBUF_PROTOC_LIBRARIES}
    sensor
)
