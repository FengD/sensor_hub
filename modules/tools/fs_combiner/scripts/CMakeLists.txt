project(fs_combiner)

file(GLOB SCRIPTS *.sh)
install(PROGRAMS ${SCRIPTS} DESTINATION fs_combiner/bin/)
