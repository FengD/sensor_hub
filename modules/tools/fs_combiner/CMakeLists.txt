project(fs_combiner)

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
if (NOT WITH_ROS2)
    add_dependencies(${PROJECT_NAME} ins_drivers_input_cyber)
endif()
add_dependencies(${PROJECT_NAME}
    ins_drivers_proto
    ins_parser_pcap
    ins_parser_asensing_ins570d
    ins_drivers_parser
    ins_drivers_input_pcap
    lidar_drivers_proto
    lidar_drivers_parser
    lidar_drivers_compensator
    camera_drivers_proto
    camera_drivers_input
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
else()
target_link_libraries(${PROJECT_NAME}
    fastrtps
    cyber
    ins_drivers_input_cyber
)
endif()

target_link_libraries(${PROJECT_NAME}
    ins_drivers_proto
    ins_parser_pcap
    ins_parser_asensing_ins570d
    ins_drivers_parser
    ins_drivers_input_pcap
    lidar_drivers_proto
    lidar_drivers_parser
    lidar_drivers_compensator
    camera_drivers_proto
    camera_drivers_input
    glog
    gflags
    pthread
    swscale
    avcodec
    avutil
    avformat
    boost_system
    boost_filesystem
    boost_thread
    opencv_core
    opencv_highgui
    opencv_video
    opencv_imgproc 
    opencv_imgcodecs
    OpenMP::OpenMP_CXX
)


add_subdirectory(scripts)
install(DIRECTORY params DESTINATION fs_combiner/)
install(TARGETS ${PROJECT_NAME} DESTINATION fs_combiner/bin/)
