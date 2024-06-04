project(cyber_python)

file(GLOB PYS *.py)
file(GLOB EXAMPLE_PYS examples/*.py)
file(GLOB TEST_PYS test/*.py)
install(FILES ${PYS} DESTINATION python/apollo/cyber/cyber/python/cyber_py3/)
install(FILES ${EXAMPLE_PYS} DESTINATION python/apollo/cyber/cyber/python/cyber_py3/examples/)
install(FILES ${TEST_PYS} DESTINATION python/apollo/cyber/cyber/python/cyber_py3/test/)
