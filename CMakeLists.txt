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
  # utilities
  src/geometry_msgs.cpp

  # mtc
  src/create_mtc_connect.cpp
  src/create_mtc_current_state.cpp
  src/create_mtc_move_relative.cpp
  src/create_mtc_move_to.cpp
  src/create_mtc_pipeline_planner.cpp
  src/create_mtc_cartesian_path.cpp
  src/create_mtc_joint_interpolation.cpp
  src/create_mtc_serial_container.cpp
  src/get_mtc_raw_stage.cpp
  src/initialize_mtc_task.cpp
  src/move_mtc_stage_to_container.cpp
  src/move_mtc_container_to_parent_container.cpp
  src/plan_mtc_task.cpp
)

add_executable(bt_demo examples/bt_demo.cpp)

target_include_directories(bt_demo PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bt_move_relative examples/bt_move_relative.cpp)
add_executable(bt_move_to examples/bt_move_to.cpp)

target_include_directories(bt_move_relative PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_move_relative ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_move_to PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_move_to ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bt_joint_interpolation_solver examples/bt_joint_interpolation_solver.cpp)

target_include_directories(bt_joint_interpolation_solver PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_joint_interpolation_solver ${PROJECT_NAME} ${catkin_LIBRARIES})
add_executable(bt_serial_container examples/bt_serial_container.cpp)

target_include_directories(bt_serial_container PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_serial_container ${PROJECT_NAME} ${catkin_LIBRARIES})


