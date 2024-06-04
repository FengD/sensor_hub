project(extract_frames_scripts)

file(GLOB SCRIPTS *.sh)
install(PROGRAMS ${SCRIPTS} DESTINATION databag_message_data_extractor/bin/)
