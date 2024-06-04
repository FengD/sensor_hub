project(cyber_io)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_proto)
target_link_libraries(${PROJECT_NAME} cyber_proto)

add_executable(${PROJECT_NAME}_tcp_echo_client example/tcp_echo_client.cc)
add_dependencies(${PROJECT_NAME}_tcp_echo_client cyber)
target_link_libraries(${PROJECT_NAME}_tcp_echo_client cyber)

add_executable(${PROJECT_NAME}_tcp_echo_server example/tcp_echo_server.cc)
add_dependencies(${PROJECT_NAME}_tcp_echo_server cyber)
target_link_libraries(${PROJECT_NAME}_tcp_echo_server cyber)

add_executable(${PROJECT_NAME}_udp_echo_client example/udp_echo_client.cc)
add_dependencies(${PROJECT_NAME}_udp_echo_client cyber)
target_link_libraries(${PROJECT_NAME}_udp_echo_client cyber)

add_executable(${PROJECT_NAME}_udp_echo_server example/udp_echo_server.cc)
add_dependencies(${PROJECT_NAME}_udp_echo_server cyber)
target_link_libraries(${PROJECT_NAME}_udp_echo_server cyber)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/io/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_tcp_echo_client DESTINATION bin/apollo/cyber/examples/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_tcp_echo_server DESTINATION bin/apollo/cyber/examples/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_udp_echo_client DESTINATION bin/apollo/cyber/examples/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_udp_echo_server DESTINATION bin/apollo/cyber/examples/)
