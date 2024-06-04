#cmake_minimum_required(VERSION 2.8.0)

project(databag_message_data_extractor)

find_package(PCL REQUIRED COMPONENT common io)
add_definitions(${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS})

include_directories(
    ${DEPENDENCIES_PATH}/
    ${CMAKE_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
if(WITH_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msg
)
endif()
target_link_libraries(${PROJECT_NAME}
    common
    data_extractor
    sensor
    boost_system
    boost_filesystem
    boost_thread
    pcl_common
    pcl_search
    pcl_io
    pcl_sample_consensus
    pcl_surface
    pcl_filters
    pcl_segmentation
    pcl_kdtree
    opencv_core
    opencv_highgui
    opencv_video
    opencv_imgproc
    ins_drivers_proto
    lidar_drivers_proto
    camera_drivers_proto
    OpenMP::OpenMP_CXX
)

add_subdirectory(data_extractor)
add_subdirectory(sensor)
add_subdirectory(scripts)

install(TARGETS ${PROJECT_NAME} DESTINATION databag_message_data_extractor/bin/)