#ifndef STATE_CLEAR_ERROR_H
#define STATE_CLEAR_ERROR_H

#include "FSM/FSMState.h"

class State_ClearError : public FSMState
{
public:
  State_ClearError(std::shared_ptr<CtrlComponents> ctrlComp)
    : FSMState(ctrlComp, (mode_t)UNITREE_ARM_SDK::ArmMode::ClearError, "ClearError"){};
  ~State_ClearError(){}

  void enter(){}
  void run_impl(){
    _lowState->errors.fill(false);
  }
  void exit(){}
  mode_t checkChange_impl(mode_t cmd){
    return (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
  };
};

#endif // STATE_CLEAR_ERROR_H