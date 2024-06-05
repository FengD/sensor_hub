project(tools)

if(WITH_IPC)
    add_subdirectory(databag_message_data_extractor)
    add_subdirectory(databag_message_type_convertor)
    add_subdirectory(bag_to_image_to_video)
    if (NOT WITH_ROS2)
        add_subdirectory(transform_apollo_crdc)
    endif()
else()
    add_subdirectory(afred_data_collect)
endif()

