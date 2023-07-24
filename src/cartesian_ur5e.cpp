#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/move_group_interface/src/move_group_interface_tutorial.cpp

int main(int argc, char * argv[])
{
  static const std::string PLANNING_GROUP = "ur_manipulator";
  
//   bool success;

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("my_logger");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          PLANNING_GROUP)](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // [WIP] Getting basic information
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::stringstream ss;
  std::copy(
    move_group_interface.getJointModelGroupNames().begin(),
    move_group_interface.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(ss, ", ")
  );
  RCLCPP_INFO_STREAM(logger, ss.str());

  // Create collision object for the robot to avoid
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
  // Initial to ZERO position
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -1.57;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = -1.57;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;
  move_group_interface.setJointValueTarget(joint_group_positions);
  
  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Execute the plan
  if (success) {
    draw_title("Executing");
    RCLCPP_INFO(logger, "Executing to initial position.");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

/* Target goal poses */
tf2::Quaternion tcp_orientation;
tcp_orientation.setRPY(M_PI, 0.0, 0.0);
auto const target_pose1 = [&tcp_orientation]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = tf2::toMsg(tcp_orientation);
    msg.position.x = 0.5;
    msg.position.y = 0.5;
    msg.position.z = 0.5;
    return msg;
}();
move_group_interface.setPoseTarget(target_pose1);

// Create a plan to that target pose
prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
draw_title("Planning");
moveit_visual_tools.trigger();
success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(logger, "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

// Execute the plan
if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    RCLCPP_INFO(logger, "Executing to target pose.");
    move_group_interface.execute(plan);
} else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
}

// Cartesian path
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(target_pose1);
geometry_msgs::msg::Pose target_pose2 = target_pose1;
target_pose2.position.z -= 0.2;
waypoints.push_back(target_pose2);  // down
target_pose2.position.y -= 0.2;
waypoints.push_back(target_pose2);  // right
target_pose2.position.z += 0.2;
target_pose2.position.y += 0.2;
target_pose2.position.x -= 0.2;
waypoints.push_back(target_pose2);  // up and left

moveit_msgs::msg::RobotTrajectory trajectory;
const double jump_threshold = 0.0;
const double eef_step = 0.01;
double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
RCLCPP_INFO(logger, "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
draw_title("Planning Cartesian path.");
moveit_visual_tools.trigger();
success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(logger, "Visualizing plan 3 (cartesian) %s", success ? "" : "FAILED");

// Execute the plan
if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    RCLCPP_INFO(logger, "Executing to target pose.");
    move_group_interface.execute(plan);
} else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}