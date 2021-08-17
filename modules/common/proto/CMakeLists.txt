cmake_minimum_required(VERSION 3.4.1)

project(common_proto)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()
file(GLOB proto_srcs *.proto)

set(PROTOBUF_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
set(Protobuf_USE_STATIC_LIBS on)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../../ ${PROTOBUF_INCLUDE_DIRS})
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_srcs})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS EXPORT_MACRO DLL_EXPORT ${proto_srcs})
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS DESCRIPTORS PROTO_DESCS ${proto_srcs})

add_library(common_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(common_proto ${PROTOBUF_LIBRARIES} ${PROTOBUF_PROTOC_LIBRARIES} pthread)