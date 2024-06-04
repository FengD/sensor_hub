project(camera_drivers_encoder)

add_subdirectory(cv_encoder)
add_subdirectory(turbo_encoder)

if(WITH_TDA4)
  add_subdirectory(h264_encoder)
endif()

if (DO_TEST)
add_subdirectory(test)
endif()