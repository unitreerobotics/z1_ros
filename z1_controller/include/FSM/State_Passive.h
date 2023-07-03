#ifndef STATE_PASSIVE_H
#define STATE_PASSIVE_H

#include "FSM/FSMState.h"

/**
 * @file State_Passive.h
 * 
 * 当sdk未连接成功时，在该状态等待，或者执行一些简单任务
 */

class State_Passive : public FSMState
{
public:
  State_Passive(std::shared_ptr<CtrlComponents> ctrlComp);
  ~State_Passive(){};
  
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:
};

#endif // STATE_PASSIVE_H