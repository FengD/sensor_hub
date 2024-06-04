project(sensor_type_convertor_scripts)

file(GLOB SCRIPTS *.sh)
install(PROGRAMS ${SCRIPTS} DESTINATION databag_message_type_convertor/bin/)
