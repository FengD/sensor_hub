cmake_minimum_required(VERSION 3.8)

project(sensor_hub_modules)
SET(LIB_VERSION 3.2.0)
if (WITH_TDA4)
    add_definitions(-DWITH_TDA4)
endif()

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

SET(SYSROOT_DIR ${RECIPE_SYSROOT})
if(x${SYSROOT_DIR} STREQUAL "x")
	SET(SYSROOT_DIR $ENV{SDKTARGETSYSROOT})
endif()

link_directories(${SYSROOT_DIR}/usr/lib)
include_directories(${SYSROOT_DIR}/usr/include)
include_directories(${SYSROOT_DIR}/usr/include/opencv4)

set(Protobuf_INCLUDE_DIRS ${SYSROOT_DIR}/usr/include CACHE INTERNAL "Protobuf_include")
set(Protobuf_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotobuf.so CACHE INTERNAL "Protobuf_libraries")
set(Protobuf_PROTOC_LIBRARIES ${SYSROOT_DIR}/usr/lib/libprotoc.so CACHE INTERNAL "Protobuf_protoc_libraries")

if (WITH_TDA4)
    if (($ENV{PLATFORM} MATCHES "TDA4") AND ($ENV{TAG} MATCHES "0703_HI"))
        set(TI_PSDK_RTOS_PATH ${SYSROOT_DIR})
    else()
        set(TI_PSDK_RTOS_PATH $ENV{TI_PSDK_RTOS_PATH})
    endif()
    
    link_directories(${TI_PSDK_RTOS_PATH}/targetfs/usr/lib/)
    add_definitions(-DWITH_TDA4)
endif()

if (WITH_A6)
    add_definitions(-DWITH_A6)
endif()

if (WITH_ROS2)
  add_definitions(-DWITH_ROS2)
endif()

if (WITH_IPC)
    add_definitions(-DWITH_IPC)
    link_directories(/install/fastrtps/lib/)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${DEPENDENCIES_PATH}/crdc_airi_common/include
    ${DEPENDENCIES_PATH}/perception_tiovx_pipelines/include/app_tiovx_camera
    ${DEPENDENCIES_PATH}/crdc_airi_common/include/sensor_msg
    ${DEPENDENCIES_PATH}/perception_algorithm_lib/include
    ${sensor_msg_INCLUDE_DIRS}
)

link_directories(
    ${DEPENDENCIES_PATH}/crdc_airi_common/lib
    ${DEPENDENCIES_PATH}/perception_tiovx_pipelines/lib
    ${DEPENDENCIES_PATH}/perception_algorithm_lib/lib
    ${sensor_msg_LIBS}
)
add_subdirectory(common)
add_subdirectory(camera_drivers)
add_subdirectory(lidar_drivers)
#add_subdirectory(ins_drivers)
#add_subdirectory(radar_drivers)
add_subdirectory(tools)
