project(common_io)

if (DO_TEST)
    add_subdirectory(test)
endif()

file(GLOB SRCS *.cc)
file(GLOB HEADERS *.h)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} glog gflags)
install(FILES ${HEADERS} DESTINATION include/common/io)