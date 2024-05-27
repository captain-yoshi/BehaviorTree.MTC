cmake_minimum_required(VERSION 3.5)
project(behaviortree_mtc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    behaviortree_cpp
    geometry_msgs
    moveit_task_constructor_core
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    behaviortree_cpp
    geometry_msgs
    moveit_task_constructor_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/geometry_msgs.cpp
  src/initialize_mtc_task.cpp
  src/plan_mtc_task.cpp
  src/create_mtc_current_state.cpp
  src/create_mtc_pipeline_planner.cpp
  src/move_mtc_stage_to_container.cpp
)

add_executable(bt_demo examples/bt_demo.cpp)

target_include_directories(bt_demo PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_demo ${PROJECT_NAME} ${catkin_LIBRARIES})
