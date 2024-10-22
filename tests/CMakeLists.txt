######################################################
# TESTS

set(BT_MTC_TESTS
  gtest_mtc_properties.cpp
  gtest_mtc_modify_planning_scene.cpp
)

set(TEST_DEPENDENCIES
    ${PROJECT_NAME}
)

# Enable testing and find necessary packages
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_test ${BT_MTC_TESTS})
    if(TARGET ${PROJECT_NAME}_test)
        target_link_libraries(${PROJECT_NAME}_test
            ${TEST_DEPENDENCIES}
        )
    endif()
endif()
