#ifndef STATE_CALIBRATION_H
#define STATE_CALIBRATION_H

#include "FSM/FSMState.h"

class State_Calibration : public FSMState
{
public:
  State_Calibration(std::shared_ptr<CtrlComponents> ctrlComp)
    : FSMState(ctrlComp, (mode_t)unitree::ArmMode::Calibration, "Calibration"){}
  ~State_Calibration(){};

  void enter() {
    _ioInter->calibration();
  }

  void exit(){};
  void run_impl(){}

  mode_t checkChange_impl(mode_t cmd) {
    return (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
  }
};

#endif // STATE_CALIBRATION_H
