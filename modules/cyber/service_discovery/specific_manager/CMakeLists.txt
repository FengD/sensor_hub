

project(cyber_service_discovery_specific_manager)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} 
    cyber_service_discovery_role
    cyber_service_discovery_container
    cyber_service_discovery_communication
)
target_link_libraries(${PROJECT_NAME} 
    cyber_service_discovery_role
    cyber_service_discovery_container
    cyber_service_discovery_communication
    fastrtps
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/service_discovery/specific_manager/)