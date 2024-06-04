project(AF_server)

find_package(Framework REQUIRED)
file(GLOB_RECURSE XML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gdbus/*.xml)
set(GENERATEDPATH "${CMAKE_BINARY_DIR}/modules/tools/AF_server/generated")

file(MAKE_DIRECTORY "${GENERATEDPATH}")

set(CODE_GENERATOR $ENV{GDBUS_CODEGENERATOR})
if("${CODE_GENERATOR}" STREQUAL "")
    set(CODE_GENERATOR gdbus-codegen-glibmm3)
endif()
message("XML_FILES is ${XML_FILES}")
message("CMAKE_CURRENT_SOURCE_DIR is ${CMAKE_CURRENT_SOURCE_DIR}")
foreach(v ${XML_FILES})
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/gdbus/" "" FILE_NAME ${v})
    string(REPLACE "\.xml" "" SERVICE_NAME ${FILE_NAME})
    message(STATUS "Service_name: ${SERVICE_NAME}") 
    set(GENERATEDNAME "${GENERATEDPATH}/${SERVICE_NAME}")
    message("GENERATEDNAME: ${GENERATEDNAME}") 
    execute_process(COMMAND ${CODE_GENERATOR} --generate-cpp-code=${GENERATEDNAME} ${v})
    execute_process(COMMAND sed -i "s:\"${GENERATEDNAME}_common.h\":\<tools/AF_server/generated/${SERVICE_NAME}_common.h\>:g" ${GENERATEDNAME}_stub.h)
    execute_process(COMMAND sed -i "s:\"${GENERATEDNAME}_common.h\":\<tools/AF_server/generated/${SERVICE_NAME}_common.h\>:g" ${GENERATEDNAME}_proxy.h)
    set(${PROJECT_NAME}_SRC_FILES ${${PROJECT_NAME}_SRC_FILES} ${GENERATEDNAME}_common.cpp ${GENERATEDNAME}_stub.cpp ${GENERATEDNAME}_proxy.cpp)
    set(${PROJECT_NAME}_HEADER_FILES ${${PROJECT_NAME}_HEADER_FILES} ${GENERATEDNAME}_common.h ${GENERATEDNAME}_stub.h ${GENERATEDNAME}_proxy.h)
endforeach()

set(INCLUDE_DIRS
    ${FRAMEWORK_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../../
    ${CMAKE_BINARY_DIR}/modules/tools/AF_server/generated/
)

aux_source_directory(${CMAKE_SOURCE_DIR}/ SOURCE_FILES)
aux_source_directory(${CMAKE_BINARY_DIR}/modules/tools/AF_server/generated/ SOURCE_FILES)

file(GLOB SRCS *.cc)
set(SOURCE_FILES ${SOURCE_FILES} ${SRCS})
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

include_directories(
    ${INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    pthread
    ${OpenCV_LIBS}
    ${FRAMEWORK_LIB}
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
)

