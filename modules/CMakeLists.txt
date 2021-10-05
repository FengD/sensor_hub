cmake_minimum_required(VERSION 2.8)

project(sensor_hub_modules)

set(CYBER_PATH /usr/local/apollo/cyber)

link_directories(${CYBER_PATH}/lib/)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CYBER_PATH}/include/
)

add_subdirectory(common)
add_subdirectory(camera_drivers)
add_subdirectory(lidar_drivers)
add_subdirectory(radar_drivers)
add_subdirectory(imu_drivers)
add_subdirectory(scripts)