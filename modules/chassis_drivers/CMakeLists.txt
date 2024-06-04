project(AI_ChassisDrivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)

file(GLOB SRCS *.cc)

set(MODULE_UTILS_LIB module_util)
if (DO_TEST)
add_subdirectory(test)
endif()
add_library(${PROJECT_NAME} SHARED ${SRCS})

add_dependencies(${PROJECT_NAME}
    chassis_drivers_proto
    chassis_drivers_parser
    )

target_link_libraries(${PROJECT_NAME}
    common
    chassis_drivers_proto
    chassis_drivers_input_someip
    chassis_drivers_parser
    glog
    gflags
    pthread
    module_diagnose
)

if(WITH_TDA4)
    target_link_libraries(${PROJECT_NAME}
        ${MODULE_UTILS_LIB}
    )
endif()
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    beatles
    sensor_msg
    sensor_msgs
    af_interfaces
)


install(TARGETS ${PROJECT_NAME} LIBRARY  DESTINATION ${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/etc/MANIFEST DESTINATION ${PROJECT_NAME}/af_launcher)
install(
    FILES orchestration/manifest.yaml  orchestration/orchestration.yaml orchestration/af_args_config.prototxt
    DESTINATION ${PROJECT_NAME}
)

install(DIRECTORY params DESTINATION ${PROJECT_NAME})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/etc_product_release DESTINATION ${PROJECT_NAME})

if(WITH_TDA4)
    find_package(Utils REQUIRED)
    target_link_libraries(${PROJECT_NAME}
        ${UTILS_LIB}
    )
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
