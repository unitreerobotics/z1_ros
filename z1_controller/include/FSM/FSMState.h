#ifndef FSMSTATE_H
#define FSMSTATE_H

#include "FSM/BaseState.h"
#include "cmdInterface/CtrlComponents.h"
#include "sdk/unitree_arm_common.h"
#include "UnitreeArmModule/math/robotics.h"

namespace unitree = UNITREE_ARM_SDK;

class FSMState : public BaseState
{
public:
  FSMState(std::shared_ptr<CtrlComponents> ctrlComp, mode_t stateMode, std::string stateString);
  virtual ~FSMState(){};

  virtual void enter() = 0;
  void run();
  virtual void run_impl() = 0;
  virtual void exit() = 0;
  mode_t checkChange(mode_t cmd);
  virtual mode_t checkChange_impl(mode_t cmd) { return 0; }

protected:
  bool _collisionTest();
  void _modifySDKState(UNITREE_ARM_SDK::ArmState& state);
  Vec6 computeDeltaQ();

  std::shared_ptr<ArmModel> _arm;
  std::shared_ptr<IOInterface> _ioInter;
  std::shared_ptr<CmdSdk> _cmdSdk;

  std::shared_ptr<LowLevelCmd> _lowCmd;
  std::shared_ptr<LowLevelState> _lowState;
  
  std::shared_ptr<FsmArmCmd> _fsmArmCmd;
  std::shared_ptr<CtrlComponents> _ctrlComp;

  bool _enableSetUp = true;
  bool _enableTearDown = true;
  bool _enableCheckChange = true;

  double _gripperQ;

private:
  void SetUp();
  void TearDown();

  void _tauAddFriction();
  void _gripperCtrl();



  Vec6 _mLinearFriction, _mCoulombFriction;

  size_t _collisionCnt;

};

#endif // FSMSTATE_H