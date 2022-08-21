project(ins_drivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    common
    ins_drivers_proto
    ins_drivers_input_socket
    ins_drivers_parser
)

target_link_libraries(${PROJECT_NAME}
    common 
    ins_drivers_proto
    ins_drivers_input_socket
    ins_drivers_parser
    glog
    gflags
    fastrtps
    cyber
    pthread
)

install(DIRECTORY params DESTINATION .)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)