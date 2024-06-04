project(ins_drivers_input)

add_subdirectory(socket)
add_subdirectory(pcap)
if (NOT WITH_ROS2)
add_subdirectory(cyber)
endif()

if (DO_TEST)
add_subdirectory(test)
endif()