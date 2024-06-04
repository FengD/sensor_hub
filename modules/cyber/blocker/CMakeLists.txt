project(cyber_blocker)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/blocker/)