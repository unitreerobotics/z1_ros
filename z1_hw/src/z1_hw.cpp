#include "z1_hw.hpp"

Z1HW::Z1HW(ros::NodeHandle& nh)
{
  /* Setting parameters */
  _nh = &nh;
  nh.param<bool>("UnitreeGripperYN", has_gripper, true);
  std::string controller_ip;
  nh.param<std::string>("udp_to_controller/controller_ip", controller_ip, "127.0.0.1");
  int sdk_own_port, controller_port;
  nh.param<int>("udp/port_to_sdk", controller_port, 8872);
  nh.param<int>("udp_to_controller/own_port", sdk_own_port, 8872);

  int njoints = has_gripper ? 7 : 6;
  pos = new double[njoints];
  vel = new double[njoints];
  eff = new double[njoints];

  /* Communicate wit arm */
  arm = new UNITREE_ARM_SDK::UnitreeArm(controller_ip, sdk_own_port, controller_port);
  this->init();

  /* Define hardware interface */
  hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_1);
  hardware_interface::JointStateHandle state_handle_2("joint2", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_2);
  hardware_interface::JointStateHandle state_handle_3("joint3", &pos[2], &vel[2], &eff[2]);
  jnt_state_interface.registerHandle(state_handle_3);
  hardware_interface::JointStateHandle state_handle_4("joint4", &pos[3], &vel[3], &eff[3]);
  jnt_state_interface.registerHandle(state_handle_4);
  hardware_interface::JointStateHandle state_handle_5("joint5", &pos[4], &vel[4], &eff[4]);
  jnt_state_interface.registerHandle(state_handle_5);
  hardware_interface::JointStateHandle state_handle_6("joint6", &pos[5], &vel[5], &eff[5]);
  jnt_state_interface.registerHandle(state_handle_6);
  if(has_gripper) {
    hardware_interface::JointStateHandle state_handle_g("jointGripper", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_g);
  }
  
  registerInterface(&jnt_state_interface);

  hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
  arm_pos_interface.registerHandle(pos_handle_1);
  hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
  arm_pos_interface.registerHandle(pos_handle_2);
  hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint3"), &cmd[2]);
  arm_pos_interface.registerHandle(pos_handle_3);
  hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("joint4"), &cmd[3]);
  arm_pos_interface.registerHandle(pos_handle_4);
  hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("joint5"), &cmd[4]);
  arm_pos_interface.registerHandle(pos_handle_5);
  hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("joint6"), &cmd[5]);
  arm_pos_interface.registerHandle(pos_handle_6);

  registerInterface(&arm_pos_interface);
  

  /* Set UnitreeArm SDK Class */
  gripper_as = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>("z1_gripper", boost::bind(&Z1HW::gripperCB, this, _1), false);
  gripper_as->start();
}

void Z1HW::init()
{
  ROS_INFO("Unitree Arm Initializing...");

  arm->init();

  // get initial pos
  for(int i(0); i<6; i++) {
    pos[i] = arm->armState.q[i];
    cmd[i] = pos[i];
  }
  gripper_position_cmd = arm->armState.gripperState.angle;
}

void Z1HW::dinit()
{
  arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  arm->sendRecv();
}

void Z1HW::read(const ros::Time& time, const ros::Duration& period)
{
  for(int i(0); i<6; i++) {
    pos[i] = arm->armState.q[i];
    vel[i] = arm->armState.dq[i];
    eff[i] = arm->armState.tau[i];
  }

  if(has_gripper)
  {
    pos[6] = arm->armState.gripperState.angle;
    vel[6] = arm->armState.gripperState.speed;
    eff[6] = arm->armState.gripperState.tau;
  }
}

void Z1HW::write(const ros::Time& time, const ros::Duration& period)
{
  arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;

  for(int i(0); i<6; i++) {
    arm->armCmd.q_d[i] = cmd[i];
  }

  if(has_gripper) {
    arm->armCmd.gripperCmd.angle = gripper_position_cmd;
    arm->armCmd.gripperCmd.maxTau = gripper_effort_cmd;
    arm->armCmd.gripperCmd.epsilon_inner = gripper_epsilon;
    arm->armCmd.gripperCmd.epsilon_outer = gripper_epsilon;
  }

  arm->sendRecv();
}

void Z1HW::gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg)
{
  gripper_position_cmd = msg->command.position;
  gripper_effort_cmd = msg->command.max_effort;

  ros::Rate r(5);

  Timer timer(2.0);
  while (true)
  {
    gripper_feedback.position = arm->armState.gripperState.angle;
    gripper_feedback.effort = arm->armState.gripperState.tau;
    gripper_feedback.reached_goal = arm->armState.gripperState.reached_goal;
    gripper_feedback.stalled = arm->armState.gripperState.stalled;

    if(gripper_as->isPreemptRequested() || !ros::ok() || (timer.wait_time() < 0))
    {
      gripper_result.position = gripper_feedback.position;
      gripper_result.effort = gripper_feedback.effort;
      gripper_result.reached_goal = gripper_feedback.reached_goal;
      gripper_result.stalled = gripper_feedback.stalled;
      gripper_as->setPreempted(gripper_result);
      break;
    }

    gripper_as->publishFeedback(gripper_feedback);

    if(std::fabs(gripper_position_cmd - arm->armState.gripperState.angle)<0.01)
    {
      if(gripper_feedback.stalled)
      {
        ROS_INFO("[Gripper] Reach goal.");
        gripper_result.position = gripper_feedback.position;
        gripper_result.effort = gripper_feedback.effort;
        gripper_result.reached_goal = gripper_feedback.reached_goal;
        gripper_result.stalled = gripper_feedback.stalled;
        gripper_as->setSucceeded(gripper_result);
        break;
      }
    }

    r.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "z1_ros_controller");
  ros::NodeHandle nh("~");

  Z1HW robot(nh);
  
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(300.0);

  while(ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    robot.read(time, period);
    cm.update(time, period);
    robot.write(time, period);

    rate.sleep();
  }

  robot.dinit();

  return 0;
}