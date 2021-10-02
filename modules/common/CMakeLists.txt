project(common)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} ${SRCS})