project(camera_calibration)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})

find_package(PCL REQUIRED)
find_package(Framework REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} 
                    ${FRAMEWORK_INCLUDE_DIR}
                    ${CMAKE_BINARY_DIR}/modules/tools/AF_server/generated/
                    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
    protobuf
    ${OpenCV_LIBS}
    camera_base
    calibration
    camera_drivers_proto
    ${PCL_LIBRARIES}
    ${FRAMEWORK_LIB}
    AF_server
    service_server
    camera_drivers_input
)

