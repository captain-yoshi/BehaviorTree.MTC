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
  # utilities
  src/geometry_msgs.cpp

  # mtc
  src/add_object_to_planning_scene.cpp
  src/create_mtc_compute_ik.cpp
  src/create_mtc_connect.cpp
  src/create_mtc_current_state.cpp
  src/create_mtc_modify_planning_scene.cpp
  src/create_mtc_move_relative.cpp
  src/create_mtc_move_to.cpp
  src/create_mtc_pipeline_planner.cpp
  src/create_mtc_cartesian_path.cpp
  src/create_mtc_generate_grasp_pose.cpp
  src/create_mtc_generate_place_pose.cpp
  src/create_mtc_joint_interpolation.cpp
  src/create_mtc_serial_container.cpp
  src/create_planning_scene_interface.cpp
  src/get_mtc_raw_stage.cpp
  src/initialize_mtc_task.cpp
  src/moveit_msgs.cpp
  src/move_mtc_stage_to_container.cpp
  src/move_mtc_container_to_parent_container.cpp
  src/plan_mtc_task.cpp
)

add_executable(bt_allow_collisions examples/bt_allow_collisions.cpp)
add_executable(bt_compute_ik examples/bt_compute_ik.cpp)
add_executable(bt_create_collision_object examples/bt_create_collision_object.cpp)
add_executable(bt_demo examples/bt_demo.cpp)
add_executable(bt_generate_place_pose examples/bt_generate_place_pose.cpp)
add_executable(bt_joint_interpolation_solver examples/bt_joint_interpolation_solver.cpp)
add_executable(bt_move_relative examples/bt_move_relative.cpp)
add_executable(bt_move_to examples/bt_move_to.cpp)
add_executable(bt_serial_container examples/bt_serial_container.cpp)

target_include_directories(bt_allow_collisions PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_allow_collisions ${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(bt_compute_ik PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_compute_ik ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_compute_ik PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_compute_ik ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_create_collision_object PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_create_collision_object ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_demo PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_generate_place_pose PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_generate_place_pose ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_joint_interpolation_solver PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_joint_interpolation_solver ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_move_relative PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_move_relative ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_move_to PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_move_to ${PROJECT_NAME} ${catkin_LIBRARIES})

target_include_directories(bt_serial_container PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_serial_container ${PROJECT_NAME} ${catkin_LIBRARIES})


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
