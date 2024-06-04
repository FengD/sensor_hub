project(lgsvl_proto)

file(GLOB proto_srcs *.proto)

LIST(APPEND PROTO_FLAGS -I${CMAKE_SOURCE_DIR}/modules/)
set(PROTO_DIST_DIR ${CMAKE_BINARY_DIR}/modules/)

execute_process(
  COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_DIST_DIR} ${proto_srcs}
)

file(GLOB PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/*.pb.cc)
file(GLOB PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/*.pb.h)

add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARIES}
  ${PROTOBUF_PROTOC_LIBRARIES}
  pthread
)

install(FILES ${proto_srcs} DESTINATION proto/apollo/cyber/lgsvl_proto/)
install(FILES ${PROTO_HDRS} DESTINATION include/cyber/lgsvl_proto/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)