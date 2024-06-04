project(lidar_drivers_input)

add_subdirectory(socket)
add_subdirectory(pcap)
if(NOT WITH_ROS2)
    add_subdirectory(cyber)
else()
    add_subdirectory(ros2)
endif()


if (DO_TEST)
add_subdirectory(test)
endif()