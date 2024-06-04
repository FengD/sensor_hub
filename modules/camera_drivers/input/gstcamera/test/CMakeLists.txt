project(gstcamera_drivers_input_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        camera_drivers_proto
        camera_drivers_input
        camera_drivers_input_gstcamera
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        gflags
        glog
        common
        cyber
        pthread
        camera_drivers_proto
        camera_drivers_input
        camera_drivers_input_gstcamera
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()