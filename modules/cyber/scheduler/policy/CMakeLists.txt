project(cyber_scheduler_policy)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_scheduler)
target_link_libraries(${PROJECT_NAME}
    cyber_scheduler
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/scheduler/policy/)