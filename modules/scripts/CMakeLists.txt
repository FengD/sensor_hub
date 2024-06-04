project(sensor_hub_scripts)

file(GLOB SCRIPTS *.sh)

install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/execute_camera_drivers.sh DESTINATION camera_drivers/bin/)
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/execute_lidar_drivers.sh DESTINATION lidar_drivers/bin/)
install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/execute_ins_drivers.sh DESTINATION ins_drivers/bin/)
