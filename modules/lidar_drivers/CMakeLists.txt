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
if (WITH_ROS2)
    file(GLOB SRC_CYBER lidar_main.cc)
    list(REMOVE_ITEM SRCS ${SRC_CYBER})
    add_library(${PROJECT_NAME} SHARED ${SRCS})
else()
    file(GLOB SRC_ROS2 lidar_main_af2.0.cc)
    list(REMOVE_ITEM SRCS ${SRC_ROS2})
    add_executable(${PROJECT_NAME} ${SRCS})
endif()




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
)

if(WITH_ROS2)
    target_link_libraries(${PROJECT_NAME}
            lidar_drivers_input_ros2)
    if(WITH_TDA4)
        target_link_libraries(${PROJECT_NAME}
            ${UTILS_LIB}
        )
    endif()
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msg
        sensor_msgs
        visualization_msgs
    )
else()
    target_link_libraries(${PROJECT_NAME} 
    fastrtps
    cyber
    lidar_drivers_input_cyber
)
endif()

if(WITH_TDA4 AND WITH_ROS2)
    target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        fastrtps
        lidars_calibrate
    )
endif()

install(DIRECTORY params DESTINATION ${PROJECT_NAME}/)
if(WITH_ROS2)
install(TARGETS ${PROJECT_NAME} LIBRARY  DESTINATION ${PROJECT_NAME}/)
install(
    FILES orchestration/manifest.yaml  orchestration/orchestration.yaml orchestration/af_args_config.prototxt
    DESTINATION ${PROJECT_NAME}/
)
else()
    install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME}/bin/)
endif()
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/etc/MANIFEST DESTINATION ${PROJECT_NAME}/etc/)
if(WITH_IPC)
    install(DIRECTORY params/drivers DESTINATION databag_message_type_convertor/params/)
endif()

if(WITH_ROS2 AND WITH_TDA4)
include(${CMAKE_SOURCE_DIR}/af_launcher/project.cmake)
install(CODE "message(\"Running my custom command\")
  execute_process(
  COMMAND
  ${CMAKE_COMMAND} -E chdir ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
  zip ../${PROJECT_NAME}.apk -r .
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
)")
endif()