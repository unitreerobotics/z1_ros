#ifndef UNITREE_ARM_H
#define UNITREE_ARM_H

#include "unitree_arm_sdk/UdpPort.h"
#include "unitree_arm_sdk/unitree_arm_common.h"

namespace UNITREE_ARM_SDK {

/**
 * @brief Maintain a connection to z1_controller.
 */
class UnitreeArm
{
public:
  /**
   * @brief Construct a new Unitree Arm object
   * 
   * @param controllerIP IP/hostname of the z1_controller
   * @param ownPort Udp port to bind for this program
   * @param toPort Udp port to z1_controller. See z1_controller/config/config.yaml
   */
  UnitreeArm(std::string controllerIP, uint ownPort = 0, uint toPort = 8871)
  {
    armCmd.version = {3, 0, 1};
    udp_ = std::make_shared<UdpPort>("z1", ownPort, controllerIP, toPort, 20);
    udp_->setCRC32(true);
  }

  /**
   * @brief Send an initialization message.
   * Verify that the connection is valid.
   */
  void init()
  {
    std::cout << "Wait for connection with z1_controller.\n";
    // Set default max tarque of gripper.
    armCmd.gripperCmd.maxTau = 10;

    /**
     * If the user wants to add a gripper of their own to the end, 
     * please modify this part of the code.
     */
    armCmd.mass_ee = 0;
    armCmd.com_ee = {0, 0, 0};
    armCmd.set_inertia_ee(Mat3::Zero());

    /**
     * If the user wants to add a load of to the end, 
     * please modify this part of the code.
     */
    armCmd.mass_load = 0;
    armCmd.com_load = {0, 0, 0};
    armCmd.inertia_load.fill(0);

    /**
     * Configured the end effector in flange frame.
     */
    armCmd.setF_T_EE(Mat4::Identity());

    /* Wait for initializing... */
    Timer timer(5.0);
    while (armState.mode == 0 || armState.mode == 99) {// Invalid & Init
      armCmd.mode = 0;
      udp_->send((uint8_t*)&armCmd, ARM_CMD_LENGTH);
      if(IOPortStatus::ok == udp_->recv((uint8_t*)&armState, ARM_STATE_LENGTH))
      {
        if(!(armState.version[0] == armCmd.version[0] && armState.version[1] == armCmd.version[1]))
        {
          std::cout << "[ERROR] Incompatiable version. Controller version: "
                    << armState.version[0]<<"."<<armState.version[1]<<"."<<armState.version[2]
                    << std::endl;
          exit(-1);
        }
      }
      if(timer.wait_time() < 0) {
        std::cout << "[ERROR] Connect with z1_controller failed.\n";
        exit(-1);
      }
    }

    armCmd.q_d = armState.q_d;
    armCmd.gripperCmd.angle = armState.gripperState.angle;
  }

  void sendRecv()
  {
    udp_->send((uint8_t*)&armCmd, ARM_CMD_LENGTH);
    udp_->recv((uint8_t*)&armState, ARM_STATE_LENGTH);
    // printlog();
  }

  /**
   * @brief Clear the running error.
   * 
   * Currently available for clear collision detection signs.
   */
  void clearErrors()
  {
    armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::ClearError;
    sendRecv();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
    sendRecv();
  }

  void printlog()
  {
    if(armState.errors[(int)UNITREE_ARM_SDK::Error::joint_position_limits_violation]) {
      std::cout << "[WARNING] joint position limits violation." << std::endl;
    }
    if(armState.errors[(int)UNITREE_ARM_SDK::Error::joint_velocity_violation]) {
      std::cout << "[WARNING] joint velocity violation." << std::endl;
    }
    if(armState.hasMotorError()) {
      std::cout << "[ERROR] Motor error occured." << std::endl;
    }
  }

  ArmCmd armCmd{};
  ArmState armState{};
  
  const double dt = 0.004;

private:
  /* communication */
  UdpPortPtr udp_;
};

} // namespace UNITREE_ARM_SDK

#endif // UNITREE_ARM_H