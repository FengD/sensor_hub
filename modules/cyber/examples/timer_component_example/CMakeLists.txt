project(cyber_examples_timer_component_example)

add_executable(${PROJECT_NAME} timer_component_example.cc)
target_link_libraries(${PROJECT_NAME} cyber_example_proto cyber cyber_class_loader cyber_time)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} DESTINATION bin/apollo/cyber/examples/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/timer.dag DESTINATION bin/apollo/cyber/examplestimer_component_example/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/timer.launch DESTINATION bin/apollo/cyber/examplestimer_component_example/)
