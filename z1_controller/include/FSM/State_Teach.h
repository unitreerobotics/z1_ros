#ifndef STATE_TEACH_H
#define STATE_TEACH_H

#include "FSM/FSMState.h"

class State_Teach : public FSMState
{
public:
  State_Teach(std::shared_ptr<CtrlComponents> ctrlComp, std::string directoryPath);
  ~State_Teach(){};
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:
  std::string _directoryPath;

  CSVTool* _trajCSV;
  size_t _stateID;
  double _errorBias{0.001};
};

#endif // STATE_TEACH_H
