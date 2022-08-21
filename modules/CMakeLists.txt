cmake_minimum_required(VERSION 2.8)

project(sensor_hub_modules)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

if(DEFINED ENV{SDKTARGETSYSROOT})
    link_directories($ENV{SDKTARGETSYSROOT}/usr/lib)
    include_directories($ENV{SDKTARGETSYSROOT}/usr/include)
    include_directories($ENV{SDKTARGETSYSROOT}/usr/include/opencv4)
    set(Protobuf_INCLUDE_DIRS $ENV{SDKTARGETSYSROOT}/usr/include CACHE INTERNAL "Protobuf_include")
    set(Protobuf_LIBRARIES $ENV{SDKTARGETSYSROOT}/usr/lib/libprotobuf.so CACHE INTERNAL "Protobuf_libraries")
    set(Protobuf_PROTOC_LIBRARIES $ENV{SDKTARGETSYSROOT}/usr/lib/libprotoc.so CACHE INTERNAL "Protobuf_protoc_libraries")
endif()

if (WITH_TDA4)
    set(TI_PSDK_RTOS_PATH $ENV{TI_PSDK_RTOS_PATH})
    link_directories(${TI_PSDK_RTOS_PATH}/targetfs/usr/lib/)
    add_definitions(-DWITH_TDA4)
endif()

if (WITH_A6)
    add_definitions(-DWITH_A6)
endif()


if (WITH_IPC)
    add_definitions(-DWITH_IPC)
endif()
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${DEPENDENCIES_PATH}/apollo/cyber/include
    ${DEPENDENCIES_PATH}/crdc_airi_common/include
    ${DEPENDENCIES_PATH}/tiovx_camera/include
)

link_directories(
    ${DEPENDENCIES_PATH}/apollo/cyber/lib
    ${DEPENDENCIES_PATH}/crdc_airi_common/lib
    ${DEPENDENCIES_PATH}/tiovx_camera/lib
)
add_subdirectory(common)
add_subdirectory(camera_drivers)
add_subdirectory(lidar_drivers)
add_subdirectory(radar_drivers)
add_subdirectory(ins_drivers)
add_subdirectory(scripts)
