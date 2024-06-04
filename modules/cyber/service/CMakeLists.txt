project(cyber_service)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/service/)