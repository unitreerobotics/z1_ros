#include "unitree_arm_sdk/unitree_arm.h"

/**
 * @example example_lowcmd.cpp
 * An example showing how to control the arm motors directly.
 * 
 * Run `roslaunch z1_bringup **_ctrl.launch` first.
 * @note The gripper is not allowed to contrl in this way.
 */
int main(int argc, char** argv)
{
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  if(z1.armState.mode != (mode_t)UNITREE_ARM_SDK::ArmMode::Passive) {
    std::cout << "[ERROR] Please repower the arm to use lowcmd mode.\n";
    exit(-1);
  }

  /* Change to State_Lowcmd mode */
  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::LowCmd;
  // Initialize position
  z1.armCmd.q_d = z1.armState.q;
  z1.armCmd.gripperCmd.angle = z1.armState.gripperCmd.angle;
  z1.armCmd.dq_d.fill(0);
  z1.armCmd.tau_d.fill(0);

  // Set control gain
  // The gain of kd & kd has been modified and is now the normal joint factor.
  z1.armCmd.Kp = {500, 600, 500, 400, 300, 200};
  z1.armCmd.Kd = {5, 5, 5, 5, 5, 5};
  z1.sendRecv();

  Timer timer(z1.dt);
  double joint_speed = 0.2;
  for(size_t i(0); i<300; i++)
  {
    z1.armCmd.q_d[0] += joint_speed*z1.dt;
    z1.armCmd.setTau(Vec6::Zero());
    z1.armCmd.gripperCmd.angle -= joint_speed*z1.dt;
    z1.sendRecv();
    timer.sleep();
  }

  /* Set to State_Passive mode */
  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  z1.sendRecv();
  return 0;
}