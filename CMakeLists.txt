cmake_minimum_required(VERSION 3.5)
project(behaviortree_mtc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    behaviortree_cpp
    moveit_task_constructor_core
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    behaviortree_cpp
    moveit_task_constructor_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/initialize_mtc_task.cpp
  src/plan_mtc_task.cpp
  src/create_mtc_current_state.cpp
)

add_executable(btcpp_sample main.cpp)

target_include_directories(btcpp_sample PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(btcpp_sample ${catkin_LIBRARIES})
