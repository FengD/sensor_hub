project(camera_drivers_diagnose)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB SRCS *.cc)
file(GLOB HEADERS *.h)

add_library(${PROJECT_NAME} STATIC ${SRCS})

target_link_libraries(${PROJECT_NAME}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    common
    camera_drivers_proto
)
if(WITH_ROS2)
add_dependencies(${PROJECT_NAME}
    sensor_msg
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
)
else()
target_link_libraries(${PROJECT_NAME}
    cyber
)
endif()

if (DO_TEST)
  add_subdirectory(test)
endif()