cmake_minimum_required(VERSION 3.5)
project(behaviortree_mtc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include 
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
  src/plan_mtc_task.cpp
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp
  behaviortree_ros2
  geometry_msgs
  moveit_task_constructor_core
  rclcpp
)

add_executable(bt_allow_collisions examples/bt_allow_collisions.cpp)
add_executable(bt_compute_ik examples/bt_compute_ik.cpp)
add_executable(bt_create_collision_object examples/bt_create_collision_object.cpp)
add_executable(bt_demo examples/bt_demo.cpp)
add_executable(bt_generate_place_pose examples/bt_generate_place_pose.cpp)
add_executable(bt_joint_interpolation_solver examples/bt_joint_interpolation_solver.cpp)
add_executable(bt_move_relative examples/bt_move_relative.cpp)
add_executable(bt_move_to examples/bt_move_to.cpp)
add_executable(bt_pick_place examples/bt_pick_place.cpp)
add_executable(bt_serial_container examples/bt_serial_container.cpp)
add_executable(generate_xml_model src/generate_xml_model.cpp)

target_link_libraries(bt_allow_collisions ${PROJECT_NAME})
target_link_libraries(bt_compute_ik ${PROJECT_NAME})
target_link_libraries(bt_create_collision_object ${PROJECT_NAME})
target_link_libraries(bt_demo ${PROJECT_NAME})
target_link_libraries(bt_generate_place_pose ${PROJECT_NAME} )
target_link_libraries(bt_joint_interpolation_solver ${PROJECT_NAME})
target_link_libraries(bt_move_relative ${PROJECT_NAME})
target_link_libraries(bt_move_to ${PROJECT_NAME})
target_link_libraries(bt_pick_place ${PROJECT_NAME})
target_link_libraries(bt_serial_container ${PROJECT_NAME})
target_link_libraries(generate_xml_model ${PROJECT_NAME})


add_subdirectory(tests)


install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  bt_allow_collisions
  bt_compute_ik
  bt_create_collision_object
  bt_demo
  bt_generate_place_pose
  bt_joint_interpolation_solver
  bt_move_relative
  bt_move_to
  bt_pick_place
  bt_serial_container
  generate_xml_model
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()