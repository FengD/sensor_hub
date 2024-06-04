project(cyber_visualizer)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

file(GLOB SRCS *.cc)
file(GLOB UIS uis/*.ui)
file(GLOB QRCS *.qrc)
QT5_WRAP_UI( UI_HEADERS ${UIS} )
QT5_ADD_RESOURCES(QRC_FILES ${QRCS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    .
)

add_executable(${PROJECT_NAME} ${UI_HEADERS} ${SRCS} ${QRC_FILES})
add_dependencies(${PROJECT_NAME} cyber)
target_link_libraries(${PROJECT_NAME}
    cyber
    GL
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    pthread
    fastrtps
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME} DESTINATION bin/apollo/cyber/)
