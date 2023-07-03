#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    std::cerr << "Usage: gripper_ctrl <gripper_goal_angle> <option: max_effort> \n"
              << "gripper_goal_angle: [-1.57, 0] rad, negative direction indicates open.\n"
              << "max_effort : [3, 20] N·m, default: 10 N·m"
              << std::endl;
    return -1;
  }

  ros::init(argc, argv, "gripper_ctrl");
  ros::NodeHandle nh;

  /* Create Gripper Action client */
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> z1_gripper_client("z1_gripper", true);

  ROS_INFO("Waiting for action server to start...");
  z1_gripper_client.waitForServer();

  /* Move Gripper */
  ROS_INFO("Move Gripper.");
  control_msgs::GripperCommandGoal goal_open;
  goal_open.command.position = std::stod(argv[1]); 
  goal_open.command.max_effort = argc == 3 ? std::stod(argv[2]) : 10.;

  z1_gripper_client.sendGoal(goal_open);

  z1_gripper_client.waitForResult();
  if(z1_gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Gripper move successfully.\n\
              Current state. Angle: %f, Effort: %f",
              z1_gripper_client.getResult()->position, z1_gripper_client.getResult()->effort);
  }else{
    ROS_INFO("Gripper move successfully.\n\
              Current state. Angle: %f, Effort: %f",
              z1_gripper_client.getResult()->position, z1_gripper_client.getResult()->effort);
  }

  return 0;
}