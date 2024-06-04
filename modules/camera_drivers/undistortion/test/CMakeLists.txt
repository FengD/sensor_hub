project(camera_undistortion_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME} camera_drivers_proto)
    target_link_libraries(${PROJECT_NAME}
      gtest
      common
      glog
      gflags
      cyber      
      pthread
      camera_drivers_proto
      opencv_core
      opencv_imgproc
      opencv_imgcodecs
      camera_drivers_undistortion_cv
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()
