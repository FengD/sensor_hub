project(camera_drivers_convert)

if (WITH_IPC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
endif()

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_imgcodecs)
if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
endif()
if(DO_TEST)
    add_subdirectory(test)
endif()