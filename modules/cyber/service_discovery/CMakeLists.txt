project(cyber_service_discovery)

add_subdirectory(communication)
add_subdirectory(container)
add_subdirectory(role)
add_subdirectory(specific_manager)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_service_discovery_specific_manager)
target_link_libraries(${PROJECT_NAME}
    cyber_service_discovery_specific_manager
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/service_discovery/)