project(cyber_component)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_proto)
target_link_libraries(${PROJECT_NAME}
    glog gflags cyber_proto
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/component/)