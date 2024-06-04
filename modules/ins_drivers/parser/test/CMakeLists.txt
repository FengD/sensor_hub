project(ins_drivers_parser_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        ins_drivers_proto
        ins_parser_asensing_ins570d
        ins_parser_pcap
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        gflags
        glog
        common
        cyber
        atomic
        pthread
        ins_drivers_proto
        ins_parser_asensing_ins570d
        ins_parser_pcap
        ins_drivers_parser
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()
