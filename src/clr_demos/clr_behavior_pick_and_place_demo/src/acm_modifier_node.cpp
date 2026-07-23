#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>

using namespace std::chrono_literals;

class AcmModifierNode : public rclcpp::Node
{
public: 
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;



  AcmModifierNode() : Node("acm_modifier_node")
  {

    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    //sets default gripper parameters
    this->declare_parameter<std::vector<std::string>>(
        "gripper_links", 
        {"finger_1_link", "finger_2_link", "gripper_base_link"});
    this->declare_parameter<std::string>("target_part", "default_part_name");

    //creates service clients
    get_planning_scene_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_planning_scene_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

    // Exposes service via standard std_srvs::srv::SetBool
    toggle_collision_service = this->create_service<std_srvs::srv::SetBool>(
        "~/toggle_collision",
        std::bind(&AcmModifierNode::handle_toggle_service, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default,
                                                                client_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Service server initialized on: ~/toggle_collision");
  }

  // waits for services to become available before accepting requests
  bool init_services()
  {
    while (!get_planning_scene_client->wait_for_service(1s) || !apply_planning_scene_client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for MoveIt services.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt planning scene services to start...");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to MoveIt! Node is ready to service requests.");
    return true;
  }


private:  

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_collision_service;

  //handles toggle logic
  void handle_toggle_service(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::string target = this->get_parameter("target_part").as_string();
    
    //request-> data == true -> Turns collision checking OFF
    //request->data == false -> Turns (re-enables) collision checking ON
    bool allow_collision = request->data;

    //If target is "all", converts it to the global wildcard ""
    if (target == "all" || target.empty() || target == "default_part_name") {
      target = ""; // Empty string tells MoveIt to target ALL links/objects in the environment
      
      if (allow_collision) {
        RCLCPP_INFO(this->get_logger(), "Service call received: Disabling collision monitoring globally for ALL objects!");
      } else {
        RCLCPP_INFO(this->get_logger(), "Service call received: Re-enabling collision monitoring globally for ALL objects!");
      }
    } else {
      if (allow_collision){
        RCLCPP_INFO(this->get_logger(), "Service call received: Disabling collision monitoring with [%s]", target.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Service call received: Re-enabling collision monitoring with [%s]", target.c_str());
      }
    }

    bool success = execute_acm_update(target, allow_collision);
    
    response->success = success;
    response->message = success ? "Collision matrix successfully updated." : "Failed to update matrix in MoveIt.";
  }

  //updates acm
  bool execute_acm_update(const std::string& target_object, bool allow_collision)
  {
    std::vector<std::string> gripper_links = this->get_parameter("gripper_links").as_string_array();

    //gets planning scene
    RCLCPP_INFO(this->get_logger(), "Rquesting planning scene");

    auto get_planning_scene = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_planning_scene->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    
    auto response = this->request_response<rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr,
                                           std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>,
                                           std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response>>(
        get_planning_scene_client, get_planning_scene);

    //modifies acm
    collision_detection::AllowedCollisionMatrix acm(response->scene.allowed_collision_matrix);
    for (const auto& link : gripper_links)
    {
      acm.setEntry(link, target_object, allow_collision);
    }

    auto apply_planning_scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
    acm.getMessage(acm_msg);
    
    apply_planning_scene->scene.allowed_collision_matrix = acm_msg;
    apply_planning_scene->scene.is_diff = true;

    auto apply_response = this->request_response<rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr,
                                                 std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request>,
                                                 std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response>>(
        apply_planning_scene_client, apply_planning_scene);

    // Check success state
    if (!apply_response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "MoveIt rejected the matrix update request.");
      return false;
    }

    return true;
  }

  template <typename Client, typename Request, typename Response>
  Response request_response(Client client, Request request)
  {
    // Sends the asynchronous request
    auto future = client->async_send_request(request);
    
    // Safely wait using standard C++ futures instead of spinning an active node executor
    if (future.wait_for(5s) == std::future_status::ready)
    {
      return future.get();
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed or timed out waiting for service: %s", client->get_service_name());
    return Response(); 
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AcmModifierNode>();

  if (node->init_services()) {
    // istantiates a MultiThreadedExecutor instead of standard single-threaded rclcpp::spin
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // loops indefinitely using multiple threads to prevent callback deadlocks
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}