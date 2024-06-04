project(common_test)

if (GTEST_FOUND)
    add_executable(${PROJECT_NAME}_factory factory_test.cc)
    add_dependencies(${PROJECT_NAME}_factory
        common
    )
    target_link_libraries(${PROJECT_NAME}_factory
        gtest
        gflags
        glog
        common
        common_io
        pthread
    )
    add_test(${PROJECT_NAME}_factory ${PROJECT_NAME}_factory)

    # add_executable(${PROJECT_NAME}_concurrent_object_pool concurrent_object_pool_test.cc)
    # add_dependencies(${PROJECT_NAME}_concurrent_object_pool
    #     common
    # )
    # target_link_libraries(${PROJECT_NAME}_concurrent_object_pool
    #     gtest
    #     gflags
    #     glog
    #     common
    #     common_io
    #     pthread
    # )
    # add_test(${PROJECT_NAME}_concurrent_object_pool ${PROJECT_NAME}_concurrent_object_pool)

    add_executable(${PROJECT_NAME}_concurrent_queue concurrent_queue_test.cc)
    add_dependencies(${PROJECT_NAME}_concurrent_queue
        common
    )
    target_link_libraries(${PROJECT_NAME}_concurrent_queue
        gtest
        gflags
        glog
        common
        common_io
        pthread
    )
    add_test(${PROJECT_NAME}_concurrent_queue ${PROJECT_NAME}_concurrent_queue)

    add_executable(${PROJECT_NAME}_for_each for_each_test.cc)
    add_dependencies(${PROJECT_NAME}_for_each
        common
    )
    target_link_libraries(${PROJECT_NAME}_for_each
        gtest
        gflags
        glog
        common
        common_io
        pthread
    )
    add_test(${PROJECT_NAME}_for_each ${PROJECT_NAME}_for_each)

    add_executable(${PROJECT_NAME}_singleton singleton_test.cc)
    add_dependencies(${PROJECT_NAME}_singleton
        common
    )
    target_link_libraries(${PROJECT_NAME}_singleton
        gtest
        gflags
        glog
        common
        common_io
        pthread
    )
    add_test(${PROJECT_NAME}_singleton ${PROJECT_NAME}_singleton)

    # add_executable(${PROJECT_NAME}_thread_safe_queue thread_safe_queue_test.cc)
    # add_dependencies(${PROJECT_NAME}_thread_safe_queue
    #     common
    # )
    # target_link_libraries(${PROJECT_NAME}_thread_safe_queue
    #     gtest
    #     gflags
    #     glog
    #     common
    #     common_io
    #     pthread
    # )
    # add_test(${PROJECT_NAME}_thread_safe_queue ${PROJECT_NAME}_thread_safe_queue)

    add_executable(${PROJECT_NAME}_thread thread_test.cc)
    add_dependencies(${PROJECT_NAME}_thread
        common
    )
    target_link_libraries(${PROJECT_NAME}_thread
        gtest
        gflags
        glog
        common
        common_io
        pthread
    )
    add_test(${PROJECT_NAME}_thread ${PROJECT_NAME}_thread)
else()
    message(WARNING "Gtest not Found. test_tf_op will not build")
endif()
