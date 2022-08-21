project(camera_drivers_input_tiovx_camera)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} 
    tiovx_camera
    app_utils_iss
    vx_app_modules
    app_utils_perf_stats
    app_utils_mem
    app_utils_ipc
    app_utils_console_io
    app_utils_remote_service
    app_utils_grpx
    app_tirtos_linux_mpu1_common
    vx_framework
    vx_utils
    vx_kernels_openvx_core
    vx_kernels_hwa
    vx_vxu
    vx_kernels_host_utils
    vx_platform_psdk_j7_linux
    vx_kernels_img_proc
    vx_kernels_fileio
    vx_kernels_imaging
    vx_target_kernels_fileio 
    vx_kernels_target_utils 
    app_utils_perf_stats
    ti_rpmsg_char
)
