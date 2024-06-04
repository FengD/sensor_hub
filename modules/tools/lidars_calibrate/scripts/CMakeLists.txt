project(lidar_calibrate_scripts)

file(GLOB SCRIPTS *.sh)
if (WITH_ROS2)
    install(DIRECTORY afred/ DESTINATION lidars_calibration/)
else()
    install(DIRECTORY cyber/ DESTINATION lidars_calibration/bin/)
endif()
