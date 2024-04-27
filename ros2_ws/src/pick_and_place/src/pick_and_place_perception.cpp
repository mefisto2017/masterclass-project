#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

class GetPoseClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetPoseClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("get_pose_client", node_options), goal_done_(false) 
      {
        // Callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;

        // move it
        move_group_node_ = rclcpp::Node::make_shared("move_group_interface", node_options);

        
      
        this->client_ptr_ = rclcpp_action::create_client<Find>(this->get_node_base_interface(), 
                                                               this->get_node_graph_interface(),
                                                               this->get_node_logging_interface(),
                                                               this->get_node_waitables_interface(), "find_objects");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GetPoseClient::send_goal, this), callback_group_);
      }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() 
  {
    using namespace std::placeholders;
    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&GetPoseClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } 
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFind::SharedPtr, const std::shared_ptr<const Find::Feedback> feedback) 
  {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }

  void result_callback(const GoalHandleFind::WrappedResult &result) 
  {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received");
    RCLCPP_INFO(this->get_logger(), "X: %f", result.result->objects[0].object.primitive_poses[0].position.x);
    RCLCPP_INFO(this->get_logger(), "Y: %f", result.result->objects[0].object.primitive_poses[0].position.y);

    float x_pos = result.result->objects[0].object.primitive_poses[0].position.x;
    float y_pos = result.result->objects[0].object.primitive_poses[0].position.y;

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

    // Pregrasp
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

    target_pose1.position.z -= 0.02;
    approach_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.02;
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
    move_group_arm.setNamedTarget("place");
    move_group_arm.move();

    // Open Gripper
    RCLCPP_INFO(LOGGER, "Release Object!");
    move_group_gripper.setNamedTarget("gripper_open");
    move_group_gripper.move();
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