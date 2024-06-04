project(cyber_class_loader_utility)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME})

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/class_loader/utility/)