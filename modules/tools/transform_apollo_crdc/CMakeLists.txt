project(modules)

find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "pcl library found")
else()
    message(FATAL_ERROR "pcl library is needed but cant be found")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "OpenCV library found")
else()
  message(FATAL_ERROR "OpenCV library is needed but not found")
endif()
link_directories(${OpenCV_LIBRARIES})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(proto)
add_subdirectory(transform)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/params/transform_config.prototxt DESTINATION transform_apollo_rdc/params/)
