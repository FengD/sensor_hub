project(drivers_camera_encoder_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        camera_drivers_encoder_cv
        camera_drivers_encoder_turbo
        camera_drivers_proto
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        gflags
        glog
        common
        cyber
        pthread
        camera_drivers_encoder_cv
        camera_drivers_encoder_turbo
        camera_drivers_proto
        turbojpeg
        opencv_core
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()
