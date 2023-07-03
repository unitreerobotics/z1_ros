#ifndef Z1_ROS_CONTROL
#define Z1_ROS_CONTROL

#include "unitree_arm_sdk/unitree_arm.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

/**
 * @brief This class wraps the functions of unitree-arm-sdk 
 * for controlling Z1 robots into the ros_control framework.
 */
class Z1HW : public hardware_interface::RobotHW
{
public:
  /**
   * @brief Default constructor
   * 
   * @note Be sure to call the init() method before operation.
   */
  Z1HW(ros::NodeHandle& nh);

  /**
   * @brief Initializes the model informations.
   * 
   */
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

  UNITREE_ARM_SDK::UnitreeArm* arm;

  ros::NodeHandle* _nh;

  /* Gripper Action */
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* gripper_as;
  control_msgs::GripperCommandFeedback gripper_feedback;
  control_msgs::GripperCommandResult gripper_result;

  double gripper_epsilon = 0.01;
  double gripper_position_cmd{};
  double gripper_effort_cmd{};
};

#endif // Z1_ROS_CONTROL
