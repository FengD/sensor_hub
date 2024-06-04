project(common_io_test)

if (GTEST_FOUND)
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
