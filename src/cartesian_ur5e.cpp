#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Code reference from below link:
// https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_logger");

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_manipulator";

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(move_group_node, PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* Create collision object for the robot to avoid */
  auto const collision_object = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "table1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.0;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  /* Starting position */
  // Initial position
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  RCLCPP_INFO(LOGGER, "Going home pose.");

  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -1.57;
  joint_group_positions[2] = 1.57;
  joint_group_positions[3] = -1.57;
  joint_group_positions[4] = -1.57;
  joint_group_positions[5] = 0.0;
  move_group_interface.setJointValueTarget(joint_group_positions);
  
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the plan
  if (success) {
    RCLCPP_INFO(LOGGER, "Executing to initial position.");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  /* Target goal poses */
  RCLCPP_INFO(LOGGER, "Going to target pose 1.");

  tf2::Quaternion tcp_orientation;
  tcp_orientation.setRPY(M_PI, 0.0, 0.0);
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation = tf2::toMsg(tcp_orientation);
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.3;
  target_pose1.position.z = 0.3;
  move_group_interface.setPoseTarget(target_pose1);

  success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Execute the plan
  if (success) {
      RCLCPP_INFO(LOGGER, "Executing to target pose 1.");
      move_group_interface.execute(plan);
  } else {
      RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  /* Cartesian path: Approach */
  RCLCPP_INFO(LOGGER, "Start approach to object.");
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  geometry_msgs::msg::Pose target_pose2 = target_pose1;
  target_pose2.position.z -= 0.05;
  approach_waypoints.push_back(target_pose2);
  target_pose2.position.z -= 0.05;
  approach_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory approach_trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, approach_trajectory);

  RCLCPP_INFO(LOGGER, "Approaching to object...");
  move_group_interface.execute(approach_trajectory);

  /* Cartesian path: Retreat */
  RCLCPP_INFO(LOGGER, "Start retreat from object.");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  geometry_msgs::msg::Pose target_pose3 = target_pose2;
  target_pose3.position.z += 0.05;
  retreat_waypoints.push_back(target_pose3);
  target_pose3.position.z += 0.05;
  retreat_waypoints.push_back(target_pose3);

  moveit_msgs::msg::RobotTrajectory retreat_trajectory;
  fraction = move_group_interface.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, retreat_trajectory);

  RCLCPP_INFO(LOGGER, "Approaching to object...");
  move_group_interface.execute(retreat_trajectory);


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}