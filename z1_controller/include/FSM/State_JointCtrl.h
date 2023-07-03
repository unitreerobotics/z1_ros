#ifndef STATE_JOINT_CONTROL_H
#define STATE_JOINT_CONTROL_H

#include "FSM/FSMState.h"

class State_JointCtrl : public FSMState
{
public:
  State_JointCtrl(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_JointCtrl(){};
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:
  void jointPositionCtrl();
  void jointSpeedCtrl();
  
  VecX lowerPositionLimit_old, lowerPositionLimit_new;
};

#endif // STATE_JOINT_CONTROL_H
