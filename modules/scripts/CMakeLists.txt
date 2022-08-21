project(sensor_hub_scripts)

file(GLOB SCRIPTS *.sh)

install(FILES ${SCRIPTS} DESTINATION bin/ PERMISSIONS WORLD_EXECUTE)
