project(camera_drivers)

add_subdirectory(input)
add_subdirectory(proto)
add_subdirectory(convert)
add_subdirectory(encoder)
add_subdirectory(undistortion)



# if(DEFINED ENV{SDKTARGETSYSROOT})
#     set(OpenCV_DIR $ENV{SDKTARGETSYSROOT}/usr/lib/cmake/opencv4/)
# endif()
# find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OpenMP Found")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    common
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
)

target_link_libraries(${PROJECT_NAME}
    common 
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
    glog
    gflags
    fastrtps
    cyber
    pthread
    ${OpenCV_LIBS}
)

install(DIRECTORY params DESTINATION .)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
