project(common)

find_package(Protobuf REQUIRED)
# code converage
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

# TODO(jinsong.wu): libcommon_io.a can not be compiled into libcommon if libcommon is static lib
# so add files in directory io here explicitly

add_subdirectory(io)

file(GLOB SRCS *.cc io/*.cc)
file(GLOB HEADERS *.h io/*.h)
add_library(${PROJECT_NAME} STATIC ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${LIB_VERSION} SOVERSION 1)
target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive
    pthread -Wl,--no-whole-archive
    ${PROTOBUF_LIBRARIES} ${PROTOBUF_PROTOC_LIBRARIES}
)
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
)
endif()
install(FILES ${HEADERS} DESTINATION include/common/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)