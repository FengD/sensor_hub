project(module_util_test)

if (GTEST_FOUND)
    add_executable(${PROJECT_NAME}_util_test util_test.cc)
    add_dependencies(${PROJECT_NAME}_util_test module_util)
    target_link_libraries(${PROJECT_NAME}_util_test
        gtest pthread
        module_util
    )
    add_test(${PROJECT_NAME}_util_test ${PROJECT_NAME}_util_test)

else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()
