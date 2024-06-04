

project(cyber_transport_transmitter)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/transport/transmitter/)