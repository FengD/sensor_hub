project(camera_drivers_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc" "../camera.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        common
        camera_drivers_proto
        camera_drivers_input
        camera_drivers_convert
        camera_drivers_encoder_cv
        camera_drivers_encoder_turbo
        camera_drivers_undistortion_cv
        camera_drivers_diagnose
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        common
        camera_drivers_proto
        camera_drivers_input
        camera_drivers_convert
        camera_drivers_encoder_cv
        camera_drivers_encoder_turbo
        camera_drivers_undistortion_cv
        camera_drivers_diagnose
        glog
        gflags
        fastrtps
        cyber
        pthread -Wl,--whole-archive
        module_diagnose_proto -Wl,--no-whole-archive
        module_diagnose
        ${OpenCV_LIBS}
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()
