project(bag_to_image_to_video_convertor)

file(GLOB SCRIPTS *.sh *.py)
install(PROGRAMS ${SCRIPTS} DESTINATION bag_to_image_to_video/bin/)
