project(module_util)

if (WITH_COV)
  message(STATUS "WITH_COV ON")
  add_compile_options(--coverage -fno-inline -fno-inline-small-functions -fno-default-inline -fprofile-arcs -ftest-coverage)
  link_libraries(gcov)
  add_definitions(-DWITH_TEST)
else()
  message(STATUS "WITH_COV OFF")
endif()

if (DO_TEST)
    add_subdirectory(test)
endif()

file(GLOB SRCS *.cc)
file(GLOB HEADERS *.h)
add_library(${PROJECT_NAME} STATIC ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${LIB_VERSION} SOVERSION 1)
target_link_libraries(${PROJECT_NAME} glog)
if (WITH_TDA4)
  find_package(Utils REQUIRED)
  target_link_libraries(${PROJECT_NAME}
      ${UTILS_LIB}
  )
endif()
install(FILES ${HEADERS} DESTINATION include/common/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)
