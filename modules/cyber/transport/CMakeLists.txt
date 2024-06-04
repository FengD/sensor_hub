

project(cyber_transport)

add_subdirectory(common)
add_subdirectory(dispatcher)
add_subdirectory(message)
add_subdirectory(qos)
add_subdirectory(receiver)
add_subdirectory(rtps)
add_subdirectory(shm)
add_subdirectory(transmitter)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
    cyber_transport_common
    cyber_transport_dispatcher
    cyber_transport_message
    cyber_transport_qos
    cyber_transport_rtps
    cyber_transport_shm
)
target_link_libraries(${PROJECT_NAME}
    cyber_transport_common
    cyber_transport_dispatcher
    cyber_transport_message
    cyber_transport_qos
    cyber_transport_rtps
    cyber_transport_shm
    fastcdr
)

file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION include/cyber/transport/)