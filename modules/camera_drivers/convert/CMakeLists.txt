project(camera_drivers_convert)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_imgcodecs)