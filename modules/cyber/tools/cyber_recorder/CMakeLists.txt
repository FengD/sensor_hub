

project(cyber_recorder)

add_subdirectory(player)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} cyber cyber_recorder_player)
target_link_libraries(${PROJECT_NAME} 
    cyber
    pthread
    cyber_recorder_player
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/tools/cyber_recorder/)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} DESTINATION bin/apollo/cyber/)
