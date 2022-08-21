project(ins_drivers_parser)

add_subdirectory(asensing_ins570d)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  ins_parser_asensing_ins570d
  ins_drivers_proto
)

target_link_libraries(${PROJECT_NAME}
  ins_parser_asensing_ins570d
  atomic
)