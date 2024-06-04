project(cyber_conf)

file(GLOB CONFS *.conf)
install(FILES ${CONFS} DESTINATION apollo/cyber/conf/)