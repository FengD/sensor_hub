project(cyber_base)

file(GLOB HEADERS *.h)

install(FILES ${HEADERS} DESTINATION include/cyber/base/)