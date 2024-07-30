project(common_io_test)

if (GTEST_FOUND)
    find_package(Protobuf REQUIRED)
    if(PROTOBUF_FOUND)
        message(STATUS "protobuf library found")
    else()
        message(FATAL_ERROR "protobuf library is needed but cant be found")
    endif()

    file(GLOB proto_srcs *.proto)
    set(Protobuf_USE_STATIC_LIBS on)

    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_srcs})

    add_executable(${PROJECT_NAME}_file file_test.cc ${PROTO_SRCS} ${PROTO_HDRS})
    target_link_libraries(${PROJECT_NAME}_file
        gtest
        gflags
        glog
        common_io
        pthread
        ${PROTOBUF_LIBRARIES} ${PROTOBUF_PROTOC_LIBRARIES}
    )
    add_test(${PROJECT_NAME}_file ${PROJECT_NAME}_file)

else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()
