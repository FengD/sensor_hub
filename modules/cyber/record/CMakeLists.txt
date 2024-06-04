project(cyber_record)

add_subdirectory(file)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_record_file cyber_proto)
target_link_libraries(${PROJECT_NAME}
    cyber_record_file pthread
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/record/)