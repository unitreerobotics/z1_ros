#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setMaxVelocityScalingFactor(0.15);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  const moveit::core::JointModelGroup* jmg = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* Move to target position */
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0;
  target_pose1.position.z = 0.4;
  move_group_interface.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(plan1);
  }

  // Cartesinan Paths
  // ^^^^^^^^^^^^^^^^
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);
  
  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.x += 0.1;
  waypoints.push_back(target_pose2); // forward

  target_pose2.position.z -= 0.1;
  waypoints.push_back(target_pose2); // up

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threhold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threhold, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  plan2.trajectory_ = trajectory;
  move_group_interface.execute(plan2);


  /* Move to saved position */
  move_group_interface.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan_home;
  success = (move_group_interface.plan(plan_home) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(plan_home);
  }

  ros::shutdown();

  return 0;
}