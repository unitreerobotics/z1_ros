#pragma once

#include <memory>
#include "sdk/unitree_arm_common.h"
#include "UnitreeArmModule/IOPort/UdpPort.h"
#include "IOInterface/LowLevelMsg/LowLevelState.h"

class CmdSdk
{
public:
  CmdSdk(){};
  ~CmdSdk(){};

  virtual bool init() = 0;
  virtual bool isConnected() = 0;
  virtual void sendRecv() = 0;
  mode_t getTargetFSMState();

  const std::array<uint8_t, 3>  controller_version = { {3, 0, 0} };
  UNITREE_ARM_SDK::ArmCmd armCmd{};
  UNITREE_ARM_SDK::ArmState armState{};
protected:
  UNITREE_ARM_SDK::ArmCmd armCmdTemp{};
};

class UdpSdk : public CmdSdk
{
public:
  UdpSdk(uint ownPort, std::shared_ptr<LowLevelState> lowState, bool cmdCache = false);
  ~UdpSdk(){}

  bool init() override;
  bool isConnected() override;
  void sendRecv() override;
private:
  void getCmd();

  UdpPortPtr _udp;
  std::shared_ptr<LowLevelState> _lowState;
  bool _cmdCache;
};