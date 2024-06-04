project(camera_drivers_input_gstcamera)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} ${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES})
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
rclcpp
)
endif()

if(DO_TEST)
    add_subdirectory(test)
endif()