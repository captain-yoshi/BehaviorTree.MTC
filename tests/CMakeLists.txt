######################################################
# TESTS

set(BT_MTC_TESTS
  # gtest_xxx.cpp
)

set(TEST_DEPENDECIES
    ${PROJECT_NAME}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_add_gtest(${PROJECT_NAME}_test ${BT_MTC_TESTS})
    target_link_libraries(${PROJECT_NAME}_test
        ${TEST_DEPENDECIES}
        ${catkin_LIBRARIES})

endif()
