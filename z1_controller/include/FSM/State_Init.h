#ifndef STATE_INIT_H
#define STATE_INIT_H

#include "FSM/FSMState.h"

class State_Init : public FSMState
{
public:
  State_Init(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_Init(){};
  
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:
  bool _hasConnectedWithSdk{};
};

#endif // STATE_INIT_H