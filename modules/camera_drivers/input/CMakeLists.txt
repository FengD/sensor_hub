project(camera_drivers_input)

add_subdirectory(testing)
if (WITH_ROS2)
  add_subdirectory(ros2)
endif()

set(INPUT_GSTCAMERA_LIB "")
set(INPUT_TIOVX_CAMERA_LIB "")
set(INPUT_MXC_LIB "")

if (WITH_IPC)
  add_subdirectory(gstcamera)
  set(INPUT_GSTCAMERA_LIB camera_drivers_input_gstcamera)
endif()


file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} 
  camera_drivers_proto
  camera_drivers_input_testing
  ${INPUT_GSTCAMERA_LIB}
  ${INPUT_TIOVX_CAMERA_LIB}
  ${INPUT_MXC_LIB}
)
if (WITH_ROS2)
add_dependencies(${PROJECT_NAME}
  camera_drivers_input_ros2
)
endif()


target_link_libraries(${PROJECT_NAME}
  atomic
  camera_drivers_input_testing
  ${INPUT_GSTCAMERA_LIB}
  ${INPUT_TIOVX_CAMERA_LIB}
  ${INPUT_MXC_LIB}
)
if (WITH_ROS2)
target_link_libraries(${PROJECT_NAME}
  camera_drivers_input_ros2
)
endif()

if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
endif()