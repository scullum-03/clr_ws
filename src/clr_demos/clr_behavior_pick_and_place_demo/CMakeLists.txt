cmake_minimum_required(VERSION 3.8)
project(clr_behavior_pick_and_place_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies_pkgs
    ament_cmake
    rclcpp
    drt_behavior
    moveit_behavior
    color_tools_behavior
    geometry_behavior
    moveit_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
)

foreach(pkg IN LISTS dependencies_pkgs)
    find_package(${pkg} REQUIRED)
endforeach()

add_executable(acm_modifier_node src/acm_modifier_node.cpp )
ament_target_dependencies(acm_modifier_node
    rclcpp 
    moveit_msgs 
    moveit_core
    moveit_ros_planning
    moveit_visual_tools
    moveit_ros_planning_interface
)

add_executable(clr_constrained_planner src/clr_constrained_planner.cpp)
ament_target_dependencies(clr_constrained_planner
    rclcpp 
    moveit_msgs 
    moveit_core
    moveit_ros_planning
    moveit_visual_tools
    moveit_ros_planning_interface
)

install(TARGETS
    acm_modifier_node
    clr_constrained_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    launch
    behavior_trees_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

