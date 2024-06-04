project(transform)

file(GLOB SRCS *.cpp)

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} transform_component_config_proto)
target_link_libraries(${PROJECT_NAME}
  gflags
  glog
  cyber
  common
  transform_component_config_proto
  fastcdr
  fastrtps
  pthread
  -Wl,--whole-archive
  lgsvl_proto  -Wl,--no-whole-archive
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${PROTOBUF_PROTOC_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION transform_apollo_rdc/bin/)
