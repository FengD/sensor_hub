project(camera_drivers_encoder_turbo)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})

target_link_libraries(${PROJECT_NAME}
    turbojpeg
)