project(service_server)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    calibration
    ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    beatles
    sensor_msg
    af_interfaces
)
