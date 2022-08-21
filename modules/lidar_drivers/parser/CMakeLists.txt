project(lidar_drivers_parser)

add_subdirectory(PandarXT32)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  lidar_drivers_parser_pxt32
  lidar_drivers_proto
)

target_link_libraries(${PROJECT_NAME}
  lidar_drivers_parser_pxt32
  atomic
)