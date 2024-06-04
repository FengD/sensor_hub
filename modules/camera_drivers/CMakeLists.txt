project(AI_CameraDrivers)

# gstreamer
if(WITH_IPC)
find_package(PkgConfig)
pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()
include_directories(${GSTREAMER_INCLUDE_DIRS})
message(${GSTREAMER_INCLUDE_DIRS})
endif()

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/ ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_subdirectory(proto)
add_subdirectory(input)
add_subdirectory(convert)
add_subdirectory(encoder)
add_subdirectory(undistortion)
add_subdirectory(camera_diagnose)

if(WITH_ROS2)
find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "pcl library found")
else()
    message(FATAL_ERROR "pcl library is needed but cant be found")
endif()

endif()
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OpenMP Found")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB SRCS *.cc)
if(WITH_ROS2)
file(GLOB SRC_CYBER camera_main.cc)
list(REMOVE_ITEM SRCS ${SRC_CYBER})
add_library(${PROJECT_NAME} SHARED ${SRCS})
else()
file(GLOB SRC_ROS2 camera_main_af2.0.cc)
list(REMOVE_ITEM SRCS ${SRC_ROS2})
add_executable(${PROJECT_NAME} ${SRCS})
endif()
add_dependencies(${PROJECT_NAME}
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
    camera_drivers_diagnose
    ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}
    common
    module_util -Wl,--whole-archive
    module_diagnose_proto -Wl,--no-whole-archive
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
    camera_drivers_diagnose
    glog
    gflags
    pthread
    module_diagnose
    ${OpenCV_LIBS}
)
if(WITH_ROS2)
if(WITH_TDA4)
    add_subdirectory(camera_calibration)
    find_package(Utils REQUIRED)
    target_link_libraries(${PROJECT_NAME}
        ${UTILS_LIB}
        ${PCL_LIBRARIES}
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        camera_calibration
        camera_base
        calibration
        camera_tools_proto
        opencv_calib3d
        opencv_highgui
    )
endif()
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    beatles
    sensor_msg
    sensor_msgs
    diagnostic_msgs
    af_interfaces
)
else()
target_link_libraries(${PROJECT_NAME}
    fastrtps
    cyber
)
endif()

if(DO_TEST)
    add_subdirectory(test)
    add_subdirectory(output)
endif()

if(WITH_TDA4)
    add_dependencies(${PROJECT_NAME} camera_drivers_encoder_h264)
    target_link_libraries(${PROJECT_NAME} camera_drivers_encoder_h264)
endif()

if(WITH_ROS2)
install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${PROJECT_NAME}/
)

install(
    FILES orchestration/manifest.yaml  orchestration/orchestration.yaml orchestration/af_args_config.prototxt
    DESTINATION ${PROJECT_NAME}/
)
else()
install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME}/bin/)
endif()
install(DIRECTORY params DESTINATION ${PROJECT_NAME}/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/etc/MANIFEST DESTINATION ${PROJECT_NAME}/etc)
if(WITH_IPC)
    install(DIRECTORY params/drivers/camera/ DESTINATION databag_message_type_convertor/params/drivers/camera/)
endif()

if(WITH_ROS2 AND WITH_TDA4)
    include(${CMAKE_SOURCE_DIR}/af_launcher/project.cmake)
    install(CODE "message(\"Running my custom command\")
      execute_process(
      COMMAND
      ${CMAKE_COMMAND} -E chdir ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
      zip ../${PROJECT_NAME}.apk -r .
      WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
    )")
endif()