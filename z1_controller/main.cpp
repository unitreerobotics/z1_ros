#include <csignal>
#include <sched.h>
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Init.h"
#include "FSM/State_JointCtrl.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Passive.h"
#include "FSM/State_LowCmd.h"
#include "FSM/State_ClearError.h"
#include "IOROS.h"
#include <ros/package.h>
#include "pinocchio/parsers/urdf.hpp"

/**
 * @brief main.cpp
 */

bool running = true;

int main(int argc, char **argv){
  /* set print format */
  std::cout << std::fixed << std::setprecision(4);

  ros::init(argc, argv, "unitree_arm_controller");
  ros::NodeHandle nh("~");

  /* running parameters */
  // 1. communication
  std::string mcu_ip; // The real robot address
  nh.param<std::string>("udp/mcu_ip", mcu_ip, "192.168.123.110");
  int port_to_mcu;
  nh.param<int>("udp/port_to_mcu", port_to_mcu, 8881);
  int port_to_sdk;
  nh.param<int>("udp/port_to_sdk", port_to_sdk, 8871);

  // 2. collision
  bool collisionOpen;
  nh.param<bool>("collision/open", collisionOpen, true);
  double collisionLimitT;
  nh.param<double>("collision/limitT", collisionLimitT, 10.0);
  collisionLimitT = clamp(collisionLimitT, 5, 30);
  // 3. control method
  std::string communication;
  nh.param<std::string>("communication", communication, "UDP");
  bool cmdCache;
  nh.param<bool>("cmdCache", cmdCache, false);

  /* set parameters */
  auto ctrlComp = std::make_shared<CtrlComponents>();
  ctrlComp->projectPath = ros::package::getPath("z1_controller");
  pin::Model model;
  pin::urdf::buildModel(ctrlComp->projectPath+std::string("/config/z1.urdf"), model);
  ctrlComp->arm = std::make_shared<ArmModel>(model);
  ctrlComp->cmdSdk = std::make_shared<UdpSdk>(port_to_sdk, ctrlComp->lowState, cmdCache);
  // Real robot or simulation
  if(communication.compare("UDP") == 0) {
    ctrlComp->ioInter = std::make_shared<IOUDP>(mcu_ip.c_str(), port_to_mcu);
  }else if(communication.compare("ROS") == 0) {
    ctrlComp->ioInter = std::make_shared<IOROS>();
  }else{ // default
    ROS_INFO("[WARNING] Unknown communication. Set to default: UDP. ");
    ctrlComp->ioInter = std::make_shared<IOUDP>(mcu_ip.c_str(), 0);
  }

  std::cout << "Wait for connection with Arm." << std::endl;
  signal(SIGINT, [](int signum){ running = false; printf("Exit z1_controller.\n");} );
  while (running) {
    if(ctrlComp->ioInter->init()) {
      std::cout << "Connected with Arm.\n[STATE] Current arm " 
                << (ctrlComp->ioInter->hasGripper ? "has " : "doesn't have ")
                << "Unitree Gripper.\n";
      break;
    }
  }

  if(!ctrlComp->ioInter->initialized()) {
    return -1;
  }

  std::string csvFilePath = ctrlComp->projectPath + std::string("/config");
  std::vector<std::shared_ptr<FSMState>> states;
  states.push_back(std::make_shared<State_Init>(ctrlComp));
  states.push_back(std::make_shared<State_Passive>(ctrlComp));
  states.push_back(std::make_shared<State_JointCtrl>(ctrlComp));
  states.push_back(std::make_shared<State_Teach>(ctrlComp, csvFilePath));
  states.push_back(std::make_shared<State_TeachRepeat>(ctrlComp, csvFilePath));
  states.push_back(std::make_shared<State_Calibration>(ctrlComp));
  states.push_back(std::make_shared<State_LowCmd>(ctrlComp));
  states.push_back(std::make_shared<State_ClearError>(ctrlComp));

  auto fsm = std::make_shared<FiniteStateMachine>(states, ctrlComp);

  while (running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  ros::shutdown();
  return 0;
}