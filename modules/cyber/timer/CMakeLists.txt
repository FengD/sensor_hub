

project(cyber_timer)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_time)
target_link_libraries(${PROJECT_NAME} 
    cyber_time
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/timer/)