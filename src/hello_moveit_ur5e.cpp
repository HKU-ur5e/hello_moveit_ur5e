#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("my_logger");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting basic information
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(
    move_group_interface.getJointModelGroupNames().begin(),
    move_group_interface.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", ")
  );

  // Initial to ZERO position
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = 1.57;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = 0.0;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;
  move_group_interface.setJointValueTarget(joint_group_positions);
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // // Set a target Pose
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();
  // move_group_interface.setPoseTarget(target_pose);

  // Set a list of target Poses
  std::vector<geometry_msgs::msg::Pose> target_poses;
  auto const target_pose1 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  target_poses.push_back(target_pose1);

  auto const target_pose2 = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = -0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  target_poses.push_back(target_pose2);

  // Iterate through the list of target Poses
  for (auto const& target_pose : target_poses) {

    // Plan to the target pose
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}