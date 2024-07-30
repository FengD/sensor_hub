project(ins_drivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)

file(GLOB SRCS *.cc)

if (DO_TEST)
add_subdirectory(test)
endif()
if (WITH_ROS2)
    file(GLOB SRC_CYBER ins_main.cc)
    list(REMOVE_ITEM SRCS ${SRC_CYBER})
add_library(${PROJECT_NAME} SHARED ${SRCS})
else()
    file(GLOB SRC_ROS2 ins_main_afred.cc)
    list(REMOVE_ITEM SRCS ${SRC_ROS2})
    add_executable(${PROJECT_NAME} ${SRCS})
    add_dependencies(${PROJECT_NAME} ins_drivers_input_cyber)
endif()
add_dependencies(${PROJECT_NAME}
    ins_drivers_proto
    ins_drivers_input_socket
    ins_drivers_parser
    ins_parser_pcap
    ins_drivers_input_pcap
    )

target_link_libraries(${PROJECT_NAME}
    common
    ins_drivers_proto
    ins_drivers_input_socket
    ins_drivers_parser
    ins_parser_pcap
    ins_drivers_input_pcap
    glog
    gflags
    pthread -Wl,--whole-archive
    module_diagnose_proto -Wl,--no-whole-archive
    module_diagnose
)
if (WITH_ROS2)
    if(WITH_TDA4)
        target_link_libraries(${PROJECT_NAME}
            ${UTILS_LIB}
        )
    endif()
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msg
    )
else()
    target_link_libraries(${PROJECT_NAME}
        cyber
        fastrtps
        ins_drivers_input_cyber
    )
endif()

if(WITH_ROS2)
install(TARGETS ${PROJECT_NAME} LIBRARY  DESTINATION ins_drivers/)
install(
    FILES orchestration/manifest.yaml  orchestration/orchestration.yaml orchestration/af_args_config.prototxt
    DESTINATION ins_drivers/
)
else()
install(TARGETS ${PROJECT_NAME} DESTINATION ins_drivers/bin/)
endif()
install(DIRECTORY params DESTINATION ins_drivers/)

if(WITH_IPC)
    install(DIRECTORY params/drivers/ins/ DESTINATION databag_message_type_convertor/params/drivers/ins/)
endif()

