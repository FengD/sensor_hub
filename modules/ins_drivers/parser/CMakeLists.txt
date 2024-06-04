project(ins_drivers_parser)

add_subdirectory(asensing_ins570d)
add_subdirectory(pcap)

file(GLOB SRCS *.cc)

if (DO_TEST)
add_subdirectory(test)
endif()

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  ins_parser_asensing_ins570d
  ins_drivers_proto
)

target_link_libraries(${PROJECT_NAME}
  ins_parser_asensing_ins570d
  atomic
)

