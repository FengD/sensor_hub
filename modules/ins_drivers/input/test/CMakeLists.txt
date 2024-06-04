project(ins_drivers_input_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        ins_drivers_proto
        ins_drivers_input_socket
        ins_drivers_input_cyber
        ins_drivers_input_pcap
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        gflags
        glog
        common
        cyber
        ins_drivers_proto
        ins_drivers_input_socket
        ins_drivers_input_cyber
        ins_drivers_input_pcap
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()
