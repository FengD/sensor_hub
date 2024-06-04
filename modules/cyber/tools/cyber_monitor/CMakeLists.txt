

project(cyber_monitor)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} cyber)
target_link_libraries(${PROJECT_NAME} 
    cyber curses
    pthread
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/tools/cyber_monitor/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} DESTINATION bin/apollo/cyber/)
