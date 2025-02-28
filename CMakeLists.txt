cmake_minimum_required(VERSION 3.5)
project(behaviortree_mtc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    behaviortree_cpp
    geometry_msgs
    moveit_msgs
    moveit_task_constructor_core
    moveit_ros_planning_interface
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    behaviortree_cpp
    geometry_msgs
    moveit_msgs
    moveit_task_constructor_core
    moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/container.cpp
  src/stage.cpp
  src/task.cpp

  src/moveit/planning_scene_interface.cpp

  src/msgs/geometry_msgs.cpp
  src/msgs/moveit_msgs.cpp

  src/solvers/pipeline_planner.cpp
  src/solvers/cartesian_path.cpp
  src/solvers/joint_interpolation.cpp

  src/stages/compute_ik.cpp
  src/stages/connect.cpp
  src/stages/current_state.cpp
  src/stages/modify_planning_scene.cpp
  src/stages/move_relative.cpp
  src/stages/move_to.cpp
  src/stages/generate_grasp_pose.cpp
  src/stages/generate_place_pose.cpp
)

add_executable(bt_pick_place examples/bt_pick_place.cpp)
target_include_directories(bt_pick_place PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_pick_place ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(generate_xml_model src/bt/generate_xml_model.cpp)
target_include_directories(generate_xml_model PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(generate_xml_model ${PROJECT_NAME} ${catkin_LIBRARIES})


add_subdirectory(tests)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
