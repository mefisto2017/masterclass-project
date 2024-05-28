#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "hole_detector/srv/hole_coordinates.hpp"

#include "std_msgs/msg/int16.hpp"

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

    // Create subscribers
    webpage_sub_ = this->create_subscription<std_msgs::msg::Int16>("/webpage", 10, std::bind(&GetPoseClient::webpage_callback, this, std::placeholders::_1), options1); 

    // Move it
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
    timer_ = create_wall_timer(std::chrono::milliseconds(250), std::bind(&GetPoseClient::pick_and_place_service, this), callback_group_);
  }


private:
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<hole_detector::srv::HoleCoordinates>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr webpage_sub_;
  bool go_ = false;

  
  void pick_and_place(rclcpp::Client<hole_detector::srv::HoleCoordinates>::SharedFuture future_result) 
  {
    // Get the response
    auto response = future_result.get();

    // Check for success
    if (!response->success) RCLCPP_ERROR(LOGGER, "Failed to get hole coordinates. Exiting.");

    RCLCPP_INFO(LOGGER, "Result received");
    RCLCPP_INFO(LOGGER, "X: %f, Y: %f, Z: %f", response->coordinates[0].x,
                                               response->coordinates[0].y,
                                               response->coordinates[0].z);

    int len = sizeof(response->coordinates) / sizeof(response->coordinates[0]);
    float z_pos = response->coordinates[0].z + 0.35;
    
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

    // Set Reference frame and end effector
    // https://docs.ros.org/en/groovy/api/moveit_ros_planning_interface/html/group__set__pose__goal.html
    move_group_arm.setPoseReferenceFrame("base_link");
    std::string rf_string = move_group_arm.getPoseReferenceFrame();
    char* char_arr_rf = &rf_string[0];
    RCLCPP_INFO(LOGGER,"Reference frame set to: %s", char_arr_rf);

    move_group_arm.setEndEffectorLink("tool0");
    std::string ee_string = move_group_arm.getEndEffectorLink();
    char* char_arr_ee = &ee_string[0];
    RCLCPP_INFO(LOGGER,"End effector link set to: %s", char_arr_ee);
    
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
    target_pose1.position.x = 0.199;
    target_pose1.position.y = 0.365;
    target_pose1.position.z = 0.397;
    target_pose1.orientation.x = 0.917;
    target_pose1.orientation.y = -0.399;
    target_pose1.orientation.z = 0.007;
    target_pose1.orientation.w = -0.016;

    target_pose1.position.z -= 0.05;
    approach_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.05;
    approach_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1;

    double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    // https://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#ad6b02d15000d5b17c89b15a0f744b47c
    // Compute a Cartesian path that follows specified waypoints with a step size of at most eef_step meters between end effector configurations of consecutive points
    // in the result trajectory. The reference frame for the waypoints is that specified by setPoseReferenceFrame(). No more than jump_threshold is allowed as change 
    // in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions). Collisions are avoided if avoid_collisions is set to true. 
    // If collisions cannot be avoided, the function fails. Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints. 
    // Return -1.0 in case of error.
    RCLCPP_INFO(LOGGER, "fraction %f", fraction);

    move_group_arm.execute(trajectory_approach);
    
    // Close Gripper
    RCLCPP_INFO(LOGGER, "Close Gripper!");
    move_group_gripper.setNamedTarget("gripper_close");
    move_group_gripper.move();

    // Retreat
    RCLCPP_INFO(LOGGER, "Retreat from object!");
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose1.position.z += 0.05;
    retreat_waypoints.push_back(target_pose1);

    target_pose1.position.z += 0.05;
    retreat_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;
    fraction = move_group_arm.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
    move_group_arm.execute(trajectory_retreat);

    // Preplace
    RCLCPP_INFO(LOGGER, "Rotating Arm");
    move_group_arm.setNamedTarget("preplace");
    move_group_arm.move();

    // Hover
    RCLCPP_INFO(LOGGER, "Hovering");
    move_group_arm.setNamedTarget("hover");
    move_group_arm.move(); 

    // Place
    RCLCPP_INFO(LOGGER, "Placing");
    auto current_pose = move_group_arm.getCurrentPose();
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.orientation = current_pose.pose.orientation;
    target_pose2.position.x = response->coordinates[0].x;
    target_pose2.position.y = response->coordinates[0].y - 0.02;
    target_pose2.position.z = z_pos;
    
    // Define orientation constraint
    // https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html
    // TIENE PROBLEMAS LA POSE QUEDA RANDOM
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group_arm.getPoseReferenceFrame();
    orientation_constraint.link_name = move_group_arm.getEndEffectorLink();
    // Create pose orientation constrant as the current orientation
    orientation_constraint.orientation = current_pose.pose.orientation;
    orientation_constraint.absolute_x_axis_tolerance = 0.2; // +-10 deg
    orientation_constraint.absolute_y_axis_tolerance = 0.2; // +-10 deg
    orientation_constraint.absolute_z_axis_tolerance = 3.14159; // +-10 deg
    orientation_constraint.weight = 1.0;
    // We need to use a generic Constraints message, but this time we add it to the orientation_constraints
    moveit_msgs::msg::Constraints orientation_constraints;
    orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
    // Set constraints
    move_group_arm.setPathConstraints(orientation_constraints);
    move_group_arm.setPoseTarget(target_pose2);


    // Plan and move
    move_group_arm.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // Fix wn, pq aveces dice q encontro una solucion pero en verdad no
    bool plan_success = false;
    int i = 1;
    while (!plan_success)
    {
      RCLCPP_INFO(LOGGER, "PLANNING FOR X: %f, Y: %f, Z: %f", target_pose2.position.x,
                                                              target_pose2.position.y,
                                                              target_pose2.position.z);
      rclcpp::Time t1 = this->now();
      bool success = (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      rclcpp::Time t2 = this->now();
      // Check if the hole is in range
      if (!success)
      {
        // If not in range move to the next point
        target_pose2.position.x = response->coordinates[i].x;
        target_pose2.position.y = response->coordinates[i].y - 0.02;
        move_group_arm.setPoseTarget(target_pose2);
        i++;
        if (i >= len) i = 0;
      }
      // Only if time was less 10 seconds a real solution was found
      if (t2.seconds()-t1.seconds() < 10.0)
      {
        plan_success = true;
      }
    }
    move_group_arm.execute(plan);
    
    // Open Gripper
    RCLCPP_INFO(LOGGER, "Release Object!");
    move_group_gripper.setNamedTarget("gripper_open");
    move_group_gripper.move();

    // https://moveit.picknik.ai/main/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface_1_1MoveGroupInterfaceImpl.html#a80bf5d4f466b9d8edbc197a7e8e2a691
    move_group_arm.clearPathConstraints();

    // Hover
    // Hovering again easier to calculate path to home
    RCLCPP_INFO(LOGGER, "Hovering");
    move_group_arm.setNamedTarget("hover");
    move_group_arm.move();

    // Go Home
    RCLCPP_INFO(LOGGER, "Going Home");
    move_group_arm.setNamedTarget("home");
    move_group_arm.move();
  }

  void pick_and_place_service()
  {
    /*
        Calls the hole detector service and start
        the arm movement
    */
    if (go_)
    {
      // Reset flag
      go_ = false;
      // Call the service
      auto request = std::make_shared<hole_detector::srv::HoleCoordinates::Request>();
      client_ptr_->async_send_request(request, std::bind(&GetPoseClient::pick_and_place, this, std::placeholders::_1));
    }
  }

  void webpage_callback(const std_msgs::msg::Int16::SharedPtr msg) 
  {
    /*
        Check if the Start button was pressed on the website
    */
    if (msg->data == 1)
    {
      go_ = true;
    }
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
