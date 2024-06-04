project(py_cyber)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(py_cyber SHARED py_cyber.cc)
add_dependencies(py_cyber cyber)
target_link_libraries(py_cyber
  cyber
  fastrtps
)

add_library(py_record SHARED py_record.cc)
add_dependencies(py_record cyber)
target_link_libraries(py_record
  cyber
  fastrtps
)

add_library(py_time SHARED py_time.cc)
add_dependencies(py_time cyber)
target_link_libraries(py_time
  cyber
  fastrtps
)

add_library(py_timer SHARED py_timer.cc)
add_dependencies(py_timer cyber)
target_link_libraries(py_timer
  cyber
  fastrtps
)

add_library(py_parameter SHARED py_parameter.cc)
add_dependencies(py_parameter cyber)
target_link_libraries(py_parameter
  cyber
  fastrtps
)


file(GLOB HEADERS *.h)
install(FILES ${HEADERS} DESTINATION python/apollo/cyber/cyber/python/internal/)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpy_cyber.so 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION python/apollo/cyber/cyber/python/internal/ RENAME _cyber_wrapper.so)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpy_record.so 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION python/apollo/cyber/cyber/python/internal/ RENAME _cyber_record_wrapper.so)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpy_time.so 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION python/apollo/cyber/cyber/python/internal/ RENAME _cyber_time_wrapper.so)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpy_timer.so 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION python/apollo/cyber/cyber/python/internal/ RENAME _cyber_timer_wrapper.so)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libpy_parameter.so 
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION python/apollo/cyber/cyber/python/internal/ RENAME _cyber_parameter_wrapper.so)