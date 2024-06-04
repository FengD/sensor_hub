project(cyber_croutine)
add_subdirectory(detail)
file(GLOB SRCS *.cc detail/*.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} cyber_proto cyber_croutine_detail)
target_link_libraries(${PROJECT_NAME} 
    atomic cyber_croutine_detail
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/croutine/)