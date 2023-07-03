#ifndef _FINITE_STATE_MACHINE_H
#define _FINITE_STATE_MACHINE_H

#include "FSM/FSMState.h"
#include <vector>
#include "UnitreeArmModule/loop/loop.h"

class FiniteStateMachine
{
public:
  FiniteStateMachine( std::vector<std::shared_ptr<FSMState>> states, 
                      std::shared_ptr<CtrlComponents> ctrlComp);
  ~FiniteStateMachine();

private:
  void _run();
  std::vector<std::shared_ptr<FSMState>> _states;
  FSMMode _mode;
  std::shared_ptr<FSMState> _currentState;
  std::shared_ptr<FSMState> _nextState;
  mode_t _nextStateNum;

  std::shared_ptr<CtrlComponents> _ctrlComp;
  std::unique_ptr<Loop> _runThread;
};

#endif // _FINITE_STATE_MACHINE_H