project(camera_diagnose_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        common
        glog
        gtest
        gflags
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        camera_drivers_proto
        camera_drivers_diagnose
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})

    add_dependencies(${PROJECT_NAME}
        ${OpenCV_LIBS}
        common
        glog
        gtest
        gflags
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        camera_drivers_proto
        camera_drivers_diagnose
    )

    if (TEST_PORT)
        ADD_DEFINITIONS(-D_SCENE="port")
    endif()
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()