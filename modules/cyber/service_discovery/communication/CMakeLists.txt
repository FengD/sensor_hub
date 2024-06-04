project(cyber_service_discovery_communication)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/service_discovery/communication/)