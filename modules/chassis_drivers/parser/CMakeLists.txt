project(chassis_drivers_parser)

if (WITH_TDA4)
  add_subdirectory(someip)
endif()
file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  chassis_drivers_proto
)

target_link_libraries(${PROJECT_NAME}
  atomic
)

if (WITH_TDA4)
  target_link_libraries(${PROJECT_NAME}
    chassis_parser_someip
  )
endif()

