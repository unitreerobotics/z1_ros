#ifndef STATE_LOW_COMMAND_H
#define STATE_LOW_COMMAND_H

#include "FSM/FSMState.h"

class State_LowCmd : public FSMState
{
public:
  State_LowCmd(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_LowCmd();
  
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:

};

#endif // STATE_LOW_COMMAND_H