#include "unitree_arm_sdk/unitree_arm.h"

/**
 * @example example_joint_ctrl.cpp
 * An example showing how to control joint velocity and position.
 * 
 * Run `roslaunch z1_bringup **_ctrl.launch` first.
 */
int main(int argc, char** argv)
{
  /* Connect to z1_controller */
  std::string controller_IP = argc > 1 ? argv[1] : "127.0.0.1";
  UNITREE_ARM_SDK::UnitreeArm z1(controller_IP);
  z1.init();

  Timer timer(z1.dt);

  /* Joint velocity control */
  double jntSpeed = 0.3;
  Vec6 dq;
  dq << jntSpeed, 0, 0, 0, 0, 0;
  for (size_t i = 0; i < 400; i++)
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
    z1.armCmd.setDq(dq);
    z1.armCmd.gripperCmd.angle -= jntSpeed*z1.dt;
    z1.sendRecv();
    timer.sleep();
  }

  /* Joint position control */
  // Get current desired q
  Vec6 q = z1.armState.getQ_d();
  for (size_t i = 0; i < 400; i++)
  {
    z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    q(0) -= jntSpeed * z1.dt;
    z1.armCmd.setQ(q);
    z1.armCmd.dq_d[0] = -jntSpeed;
    z1.armCmd.gripperCmd.angle += jntSpeed*z1.dt;
    z1.sendRecv();
    timer.sleep();
  }

  z1.armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  z1.sendRecv();

  return 0;
}