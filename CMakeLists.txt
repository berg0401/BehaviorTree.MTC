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
  src/geometry_msgs.cpp
  src/get_mtc_raw_stage.cpp
  src/initialize_mtc_task.cpp
  src/plan_mtc_task.cpp
  src/create_mtc_current_state.cpp
  src/create_mtc_move_relative.cpp
  src/create_mtc_pipeline_planner.cpp
  src/move_mtc_stage_to_container.cpp
  src/create_mtc_cartesian_path.cpp
  src/create_mtc_generate_grasp_pose.cpp
  src/create_planning_scene_interface.cpp
  src/moveit_msgs.cpp
  src/set_up_scene.cpp
  src/create_mtc_move_to.cpp
  src/create_mtc_connect.cpp
)

add_executable(bt_demo examples/bt_demo.cpp)

target_include_directories(bt_demo PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_demo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bt_move_relative examples/bt_move_relative.cpp)

target_include_directories(bt_move_relative PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_move_relative ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bt_create_collision_object examples/bt_create_collision_object.cpp)

target_include_directories(bt_create_collision_object PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_create_collision_object ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bt_set_ik_frame examples/bt_set_ik_frame.cpp)

target_include_directories(bt_set_ik_frame PRIVATE ${catkin_INCLUDE_DIRS} include)
target_link_libraries(bt_set_ik_frame ${PROJECT_NAME} ${catkin_LIBRARIES})