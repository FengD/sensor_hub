project(camera_convert_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cpp")
    add_executable(${PROJECT_NAME} ${_SRCS})
    target_link_libraries(${PROJECT_NAME}
      gtest
      common
      glog
      gflags
      cyber      
      camera_drivers_convert
      pthread
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()

