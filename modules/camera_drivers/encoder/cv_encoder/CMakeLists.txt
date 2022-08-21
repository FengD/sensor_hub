project(camera_drivers_encoder_cv)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} camera_drivers_proto)
target_link_libraries(${PROJECT_NAME}
    opencv_imgcodecs
)