project(transform_component_config_proto)

file(GLOB proto_srcs *.proto)

set(Protobuf_USE_STATIC_LIBS on)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_srcs})
add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIE} ${PROTOBUF_PROTOC_LIBRARIES})