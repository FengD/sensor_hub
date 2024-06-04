project(tools)

add_subdirectory(cyber_monitor)
add_subdirectory(cyber_recorder)

if (($ENV{PLATFORM} MATCHES "X86") AND ($ENV{TAG} MATCHES "1804"))
    add_subdirectory(cyber_visualizer) 
endif()

file(GLOB PYS cyber_channel/cyber_channel.py 
              cyber_launch/cyber_launch.py
              cyber_node/cyber_node.py
              cyber_service/cyber_service.py)
install(FILES ${PYS} DESTINATION python/apollo/cyber/)