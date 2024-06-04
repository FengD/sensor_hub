project(lidars_calibrate)
if(WITH_TDA4)
FILE(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
else()
FILE(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/lidars_calibration.cpp)
endif()

add_library(${PROJECT_NAME} STATIC ${SRCS})
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

find_package(PCL REQUIRED)
if(WITH_TDA4)
find_package(Framework REQUIRED)
endif()
if(PCL_FOUND)
    message(STATUS "pcl library found")
else()
    message(FATAL_ERROR "pcl library is needed but cant be found")
endif()

include_directories(${PCL_INCLUDE_DIRS}
                    ${FRAMEWORK_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${CMAKE_BINARY_DIR}/modules/tools/AF_server/generated/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(calibrate)

if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msg
    sensor_msgs
    visualization_msgs
    geometry_msgs
)
target_link_libraries(${PROJECT_NAME}
    glog
    gflags
    pthread
    fastrtps
    lidars_calibration
    lidar_dynamic_calibrate
    pcl_common
    pcl_search
    pcl_io
    pcl_sample_consensus
    pcl_surface
    pcl_filters
    pcl_segmentation
    pcl_kdtree
    ${OpenCV_LIBS}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    boost_filesystem
)
if(WITH_TDA4)
    target_link_libraries(${PROJECT_NAME}
    ${FRAMEWORK_LIB}
    service_server
    AF_server)
endif()
else()
target_link_libraries(${PROJECT_NAME}
    glog
    gflags
    pthread
    fastrtps
    lidars_calibration
    pcl_common
    pcl_search
    pcl_io
    pcl_sample_consensus
    pcl_surface
    pcl_filters
    pcl_segmentation
    pcl_kdtree
    cyber
)
endif()

if(WITH_IPC)
install(DIRECTORY orchestration/ DESTINATION lidars_calibration)
add_subdirectory(scripts)
endif()