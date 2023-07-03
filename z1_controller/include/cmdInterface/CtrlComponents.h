# pragma once

#include "cmdInterface/cmdSdk.h"
#include "model/manipulator.h"
#include "IOInterface/LowLevelMsg/LowLevelCmd.h"
#include "IOInterface/LowLevelMsg/LowLevelState.h"
#include "IOInterface/IOInterface.h"
#include "IOInterface/IOUDP/IOUDP.h"
#include "UnitreeArmModule/error/Color.h"
#include <thread>
#include "UnitreeArmModule/file/CSVTool.h"

typedef struct
{
  Vec6 q{};
  Vec6 qlast{};
  Vec6 dq{};
  Vec6 tau{};
  Vec6 ftip{};
} FsmArmCmd;

struct CtrlComponents
{
  CtrlComponents()
  {
    lowCmd = std::make_shared<LowLevelCmd>();
    lowState = std::make_shared<LowLevelState>(dt);
    fsmArmCmd = std::make_shared<FsmArmCmd>();
  };
  ~CtrlComponents(){};

  double dt = 0.004;

  std::string projectPath;

  std::shared_ptr<ArmModel> arm;

  std::shared_ptr<IOInterface> ioInter;
  std::shared_ptr<CmdSdk> cmdSdk;

  std::shared_ptr<LowLevelCmd> lowCmd;
  std::shared_ptr<LowLevelState> lowState;
  std::shared_ptr<FsmArmCmd> fsmArmCmd;

  bool collisionOpen;
  double collisionLimitT;
};