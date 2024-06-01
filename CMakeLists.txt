cmake_minimum_required(VERSION 3.5)
project(behaviortree_mtc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(bt_demo_lib
  src/create_mtc_current_state.cpp
  src/create_mtc_move_relative.cpp
  src/create_mtc_pipeline_planner.cpp
  src/initialize_mtc_task.cpp
  src/move_mtc_stage_to_container.cpp
  src/plan_mtc_task.cpp
)

ament_target_dependencies(bt_demo_lib
  behaviortree_cpp
  moveit_task_constructor_core
  rclcpp
)

# Create executable
add_executable(bt_demo examples/bt_demo.cpp)
target_link_libraries(bt_demo bt_demo_lib)

# Install targets
install(TARGETS
  bt_demo
  bt_demo_lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()
