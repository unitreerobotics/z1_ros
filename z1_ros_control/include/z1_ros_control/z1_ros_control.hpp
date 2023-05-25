#ifndef Z1_ROS_CONTROL
#define Z1_ROS_CONTROL

#include "unitree_arm_sdk/control/unitreeArm.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

class Z1Robot : public hardware_interface::RobotHW
{
public:
  Z1Robot(ros::NodeHandle& nh);
  void init();
  void dinit();
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  
  void gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg);

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface arm_pos_interface;
  hardware_interface::PositionJointInterface grip_pos_interface;
  
  bool has_gripper;

  double cmd[6];
  
  double* pos;
  double* vel;
  double* eff;

  UNITREE_ARM::CtrlComponents* ctrlComp;
  UNITREE_ARM::unitreeArm* arm;
  std::vector<double> KP, KW;

  std::string hostname;
  int controller_port_sdk;

  ros::NodeHandle* _nh;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* gripper_as;
  control_msgs::GripperCommandFeedback gripper_feedback;
  control_msgs::GripperCommandResult gripper_result;

  double gripper_position_cmd{0};
  double gripper_effort_cmd{0};

  bool gripper_as_active{false};
};

#endif
