project(cyber_example_proto)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

SET(SYSROOT_DIR ${RECIPE_SYSROOT})
if(x${SYSROOT_DIR} STREQUAL "x")
    SET(SYSROOT_DIR $ENV{SDKTARGETSYSROOT})
endif()
set(Protobuf_INCLUDE_DIRS ${SYSROOT_DIR}/usr/include)
set(Protobuf_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotobuf.so)
set(Protobuf_PROTOC_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotoc.so)

file(GLOB proto_srcs *.proto)
set(Protobuf_USE_STATIC_LIBS on)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_srcs})

add_library(${PROJECT_NAME} STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES} ${PROTOBUF_PROTOC_LIBRARIES} pthread)

install(FILES ${PROTO_HDRS} DESTINATION include/cyber/example/proto/)