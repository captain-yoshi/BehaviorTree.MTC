######################################################
# TESTS

set(BT_MTC_TESTS
  gtest_mtc_modify_planning_scene.cpp
  gtest_mtc_properties.cpp
  gtest_plan_mtc_task.cpp
)

set(TEST_DEPENDENCIES
    gtest_utils
    ${PROJECT_NAME}
)

add_library(gtest_utils models.cpp stage_mockups.cpp)
target_link_libraries(gtest_utils ${PROJECT_NAME} ${moveit_task_constructor_LIBRARIES})

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