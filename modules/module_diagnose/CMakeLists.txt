project(module_diagnose)

file(GLOB SRCS *.cc)
file(GLOB HEADERS *.h)

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
link_directories(${SYSROOT_DIR}/usr/lib)
include_directories(${SYSROOT_DIR}/usr/include)
set(Protobuf_INCLUDE_DIRS ${SYSROOT_DIR}/usr/include CACHE INTERNAL "Protobuf_include")
set(Protobuf_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotobuf.so CACHE INTERNAL "Protobuf_libraries")
set(Protobuf_PROTOC_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotoc.so CACHE INTERNAL "Protobuf_protoc_libraries")

add_subdirectory(proto)

add_library(${PROJECT_NAME} STATIC ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${LIB_VERSION} SOVERSION 1)
add_dependencies(${PROJECT_NAME} module_diagnose_proto)
target_link_libraries(${PROJECT_NAME} -Wl,--whole-archive
    module_diagnose_proto
    pthread -Wl,--no-whole-archive
)
if (WITH_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
    )
    add_dependencies(${PROJECT_NAME} sensor_msg)
    include_directories(${CMAKE_BINARY_DIR}/modules/af/rosidl_generator_cpp)
    rosidl_get_typesupport_target(cpp_typesupport_target
        sensor_msg "rosidl_typesupport_cpp"
    )
    target_link_libraries(${PROJECT_NAME}
        "${cpp_type_support_target}"
    )
endif()
install(FILES ${HEADERS} DESTINATION include/module_diagnose/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/)