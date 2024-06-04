project(camera_drivers_undistortion)

add_subdirectory(cv)
if(DO_TEST)
    add_subdirectory(test)
endif()
