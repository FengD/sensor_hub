project(ins_parser_asensing_ins570d)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})