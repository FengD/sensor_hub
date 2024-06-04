project(cyber_data_fusion)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/data/fusion/)