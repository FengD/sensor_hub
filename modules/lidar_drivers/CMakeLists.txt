project(LidarDrivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)
add_subdirectory(compensator)

if (DO_TEST)
add_subdirectory(test)
endif()

find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "pcl library found")
else()
    message(FATAL_ERROR "pcl library is needed but cant be found")
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})

set(LIB_LIDAR_DRIVERS_COMMON
    lidar_drivers_proto
    lidar_drivers_input_pcap
    lidar_drivers_parser
    lidar_drivers_compensator
    lidar_drivers_input_socket
)

set(LIB_COMMON
    common 
    glog
    gflags
    pthread
    module_diagnose
)

add_dependencies(${PROJECT_NAME} 
    ${LIB_LIDAR_DRIVERS_COMMON}
)

if(NOT WITH_ROS2)
    add_dependencies(${PROJECT_NAME} lidar_drivers_input_cyber)
else()
    add_dependencies(${PROJECT_NAME} lidar_drivers_input_ros2)
endif()
target_link_libraries(${PROJECT_NAME}
    module_util -Wl,--whole-archive
    module_diagnose_proto -Wl,--no-whole-archive
    ${LIB_LIDAR_DRIVERS_COMMON}
    ${LIB_COMMON}
    ${Protobuf_LIBRARIES}
)

if(WITH_ROS2)
    target_link_libraries(${PROJECT_NAME}
            lidar_drivers_input_ros2
    )
    add_dependencies(${PROJECT_NAME}
        sensor_msg
    )
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        visualization_msgs
    )

    rosidl_get_typesupport_target(cpp_typesupport_target
        sensor_msg "rosidl_typesupport_cpp"
    )
    
    target_link_libraries(${PROJECT_NAME}
        "${cpp_type_support_target}"
    )
    
else()
    target_link_libraries(${PROJECT_NAME} 
    fastrtps
    cyber
    lidar_drivers_input_cyber
)
endif()

install(DIRECTORY params DESTINATION ${PROJECT_NAME}/)
install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME}/bin/)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/etc/MANIFEST DESTINATION ${PROJECT_NAME}/etc/)
if(WITH_IPC)
    install(DIRECTORY params/drivers DESTINATION databag_message_type_convertor/params/)
endif()
