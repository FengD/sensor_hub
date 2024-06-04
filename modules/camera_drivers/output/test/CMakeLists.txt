project(camera_cyber_output_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cpp")
    add_executable(${PROJECT_NAME} ${_SRCS})
    target_link_libraries(${PROJECT_NAME}
      gtest
      glog
      gflags
      common
      fastrtps
      cyber
      pthread
      ${PROTOBUF_LIBRARIES}
      opencv_highgui
      opencv_imgproc
      opencv_calib3d
      opencv_core
      opencv_features2d
      opencv_flann
      opencv_highgui
      opencv_imgcodecs
      opencv_imgproc
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} DESTINATION bin/)
