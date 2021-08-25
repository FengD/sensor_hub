cmake_minimum_required(VERSION 2.8)

project(sensor_common)

add_subdirectory(proto)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} ${SRCS})