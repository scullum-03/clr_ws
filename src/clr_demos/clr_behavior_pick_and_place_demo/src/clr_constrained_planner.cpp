#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/srv/get_motion_plan.hpp> 
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("clr_constrained_planner", node_options);

  // uses a multi-threaded executor to handle service calls and visualization
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread run_thread([&executor]() { executor.spin(); });

  // initializes MoveGroup for configuration
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "clr");

  // initializes Visualization Tools
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools(
      node, move_group_interface.getRobotModel()->getModelFrame(), "constrained_motion_planning", move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

// initializes the MotionPlanRequest message
moveit_msgs::msg::MotionPlanRequest request;
request.group_name = "clr"; 
request.allowed_planning_time = 30.0;
request.num_planning_attempts = 5;

// sets coordinate frame and end-effector link names 
std::string reference_frame = "world"; 
std::string ee_link = "gripper_base_link"; 

// moveit_msgs::msg::RobotState start_state;
// moveit::core::robotStateToRobotStateMsg(*move_group_interface.getCurrentState(), start_state);
// request.start_state = start_state;

//creates the box constraint object
moveit_msgs::msg::PositionConstraint box_constraint;
box_constraint.header.frame_id = reference_frame;
box_constraint.link_name = ee_link;

shape_msgs::msg::SolidPrimitive box;
box.type = shape_msgs::msg::SolidPrimitive::BOX;
box.dimensions = { 0.5, 2.0, 0.7 }; 
box_constraint.constraint_region.primitives.emplace_back(box);

geometry_msgs::msg::Pose box_pose;
box_pose.position.x = 0.25;
box_pose.position.y = 0.0;
box_pose.position.z = 0.5;
box_pose.orientation.w = 1.0;
box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
box_constraint.weight = 1.0; 

request.path_constraints.position_constraints.push_back(box_constraint);
request.path_constraints.name = "front_of_rail_constraint";

// visualizes the box constraint
  Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                              box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                              box_pose.position.z + box.dimensions[2] / 2);
  moveit_visual_tools.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

// sets target position
geometry_msgs::msg::Pose goal_pose;
goal_pose.position.x = 0.3; 
goal_pose.position.y = 1.25;
goal_pose.position.z = 0.5;
goal_pose.orientation.w = 1.0; 

// sets target position constraints
moveit_msgs::msg::PositionConstraint goal_pos;
goal_pos.header.frame_id = reference_frame;
goal_pos.link_name = ee_link;

shape_msgs::msg::SolidPrimitive tol_sphere;
tol_sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
tol_sphere.dimensions = { 0.01 }; // 1 centimeter tolerance radius

goal_pos.constraint_region.primitives.push_back(tol_sphere);
goal_pos.constraint_region.primitive_poses.push_back(goal_pose);
goal_pos.weight = 1.0;

// sets target orientation constraints
moveit_msgs::msg::OrientationConstraint goal_ori;
goal_ori.header.frame_id = reference_frame;
goal_ori.link_name = ee_link;
goal_ori.orientation = goal_pose.orientation;
goal_ori.absolute_x_axis_tolerance = 0.01; 
goal_ori.absolute_y_axis_tolerance = 0.01;
goal_ori.absolute_z_axis_tolerance = 0.01;
goal_ori.weight = 1.0;

// packs position and orientation into the goal container, then onto the request
moveit_msgs::msg::Constraints goal_constraint;
goal_constraint.position_constraints.push_back(goal_pos);
goal_constraint.orientation_constraints.push_back(goal_ori);
request.goal_constraints.push_back(goal_constraint);

// sends the request via service request
rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr client =
    node->create_client<moveit_msgs::srv::GetMotionPlan>("/get_kinematic_path");

while (!client->wait_for_service(std::chrono::seconds(1))) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Waiting for /get_kinematic_path service...");
}

auto srv_request = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
srv_request->motion_plan_request = request;

RCLCPP_INFO(node->get_logger(), "Sending target-pose service request to /get_kinematic_path...");
auto result_future = client->async_send_request(srv_request);

// waits for the response and parse the output trajectory
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    if (response->motion_plan_response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Plan found successfully! Trajectory length: %zu points",
                  response->motion_plan_response.trajectory.joint_trajectory.points.size());
      
      // visualizes the successful trajectory path line in RViz (extra)
      moveit_visual_tools.publishTrajectoryLine(response->motion_plan_response.trajectory, 
                                                move_group_interface.getRobotModel()->getJointModelGroup("clr"));
      moveit_visual_tools.trigger();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed with code: %d", response->motion_plan_response.error_code.val);
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_kinematic_path");
  }

// shutdown structure
  rclcpp::shutdown();
  if (run_thread.joinable()) {
    run_thread.join();
  }
  return 0;
}

