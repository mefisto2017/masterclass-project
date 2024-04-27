#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "hole_detector/srv/hole_coordinates.hpp"

#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class GetPoseClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;

  explicit GetPoseClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) : Node("get_pose_client", node_options)
  {
    // Callback group
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = callback_group_;

    // move it
    move_group_node_ = rclcpp::Node::make_shared("move_group_interface", node_options);

    // Hole detector client
    client_ptr_ = this->create_client<hole_detector::srv::HoleCoordinates>("/holes_coordinates");

    // Wait for the service to be available
    while (!client_ptr_->wait_for_service(std::chrono::seconds(5))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(LOGGER, "Service not available, waiting again...");
    }

    // Call the service
    auto request = std::make_shared<hole_detector::srv::HoleCoordinates::Request>();
    client_ptr_->async_send_request(request, std::bind(&GetPoseClient::pick_and_place_callback, this, std::placeholders::_1));
  }


private:
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<hole_detector::srv::HoleCoordinates>::SharedPtr client_ptr_;

  
  void pick_and_place_callback(rclcpp::Client<hole_detector::srv::HoleCoordinates>::SharedFuture future_result) 
  {
    // Get the response
    auto response = future_result.get();

    // Check for success
    if (!response->success) RCLCPP_ERROR(LOGGER, "Failed to get hole coordinates. Exiting.");

    RCLCPP_INFO(LOGGER, "Result received");
    RCLCPP_INFO(LOGGER, "X: %f, Y: %f, Z: %f", response->coordinates[0].x,
                                               response->coordinates[0].y,
                                               response->coordinates[0].z);


    float x_pos = response->coordinates[0].x;
    float y_pos = response->coordinates[0].y;
    float z_pos = response->coordinates[0].z;

    /* Coordinates returned by the camera
       X: 0.327950 , Y: -0.012054
       My coordinates
       x = 0.335 , y = -0.016; // replace them  in line 
           target_pose1.position.x = 0.335;
           target_pose1.position.y = -0.016;

       then new coordinates:
       x_pos += 0.335 - 0.327950
       y_pos += -0.016 + 0.012054
    */
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node_);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Moving the arm
    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    static const std::string PLANNING_GROUP_GRIPPER = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node_, PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node_, PLANNING_GROUP_GRIPPER);

    const moveit::core::JointModelGroup *joint_model_group_arm =  move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    const moveit::core::JointModelGroup *joint_model_group_gripper =  move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

    // Get Current State
    moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
    moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);

    std::vector<double> joint_group_positions_arm;
    std::vector<double> joint_group_positions_gripper;
    
    current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
    current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);

    move_group_arm.setStartStateToCurrentState();
    move_group_gripper.setStartStateToCurrentState();

    // Go Home
    RCLCPP_INFO(LOGGER, "Going Home");
    move_group_arm.setNamedTarget("home");
    move_group_arm.move();

    RCLCPP_INFO(LOGGER, "Rotating Arm");
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.875;
    target_pose1.orientation.y = 0.483;
    target_pose1.orientation.z = 0.027;
    target_pose1.orientation.w = -0.013;
    target_pose1.position.x = -0.312;
    target_pose1.position.y = 0.457;
    target_pose1.position.z = -0.010;
    move_group_arm.setPoseTarget(target_pose1);
    move_group_arm.move();
    RCLCPP_INFO(LOGGER, "Finish Arm");

    /*// Pregrasp
    RCLCPP_INFO(LOGGER, "Pregrasp Position");
    move_group_arm.setNamedTarget("pregrasp");
    move_group_arm.move();

    // Open Gripper
    RCLCPP_INFO(LOGGER, "Open Gripper!");
    move_group_gripper.setNamedTarget("gripper_open");
    move_group_gripper.move();

    // Approach
    RCLCPP_INFO(LOGGER, "Approach to object!");
    geometry_msgs::msg::Pose target_pose1;
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    target_pose1.position.x = x_pos;
    target_pose1.position.y = y_pos;
    target_pose1.position.z = 0.254;
    target_pose1.orientation.x = 0.721;
    target_pose1.orientation.y = -0.693;
    target_pose1.orientation.z = -0.018;
    target_pose1.orientation.w = -0.006;

    target_pose1.position.z -= 0.04;
    approach_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.04;
    approach_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    move_group_arm.execute(trajectory_approach);
    
    // Close Gripper
    RCLCPP_INFO(LOGGER, "Close Gripper!");
    move_group_gripper.setNamedTarget("gripper_close");
    move_group_gripper.move();

    // Retreat
    RCLCPP_INFO(LOGGER, "Retreat from object!");
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose1.position.z += 0.0;
    retreat_waypoints.push_back(target_pose1);

    target_pose1.position.z += 0.03;
    retreat_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;

    fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

    move_group_arm.execute(trajectory_retreat);

    // Place
    RCLCPP_INFO(LOGGER, "Rotating Arm");
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.343;
    target_pose1.position.y = 0.132;
    target_pose1.position.z = 0.264;
    move_group_arm.setPoseTarget(target_pose1);

    // Open Gripper
    RCLCPP_INFO(LOGGER, "Release Object!");
    move_group_gripper.setNamedTarget("gripper_open");
    move_group_gripper.move();*/
  }
}; // class GetPoseClient


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_client = std::make_shared<GetPoseClient>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);
  executor.spin();
  rclcpp::shutdown();

}