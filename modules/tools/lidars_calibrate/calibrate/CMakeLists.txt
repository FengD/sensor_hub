project(lidars_calibration)

add_subdirectory(proto)

file(GLOB SRCS *.cpp)
include_directories(
    include
)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} lidars_calibration_config_proto)

target_link_libraries(${PROJECT_NAME}
    lidars_calibration_config_proto
    ${PCL_LIBRARIES}
    gomp
)

