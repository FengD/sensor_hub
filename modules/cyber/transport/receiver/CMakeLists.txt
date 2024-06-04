

project(cyber_transport_receiver)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/transport/receiver/)