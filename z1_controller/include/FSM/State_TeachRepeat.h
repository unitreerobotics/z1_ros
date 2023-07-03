#ifndef STATE_TEACH_REPEAT_H
#define STATE_TEACH_REPEAT_H

#include "FSM/FSMState.h"
#include "model/jointTraj.h"

class State_TeachRepeat : public FSMState
{
public:
  State_TeachRepeat(std::shared_ptr<CtrlComponents> ctrlComp, std::string directoryPath);
  ~State_TeachRepeat(){};
  void enter() override;
  void run_impl() override;
  void exit() override;
  mode_t checkChange_impl(mode_t cmd) override;
private:
  std::string _directoryPath;

  bool _setCorrectly, _reachStart, _finihedRepeat;
  CSVTool *_trajCSV;
  std::shared_ptr<JointTraj> _toStartTraj;
  
  Vec6 _startQ;
  double _startGripperQ, _deltaGripperStartQ;
  size_t _index;
};

#endif // STATE_TEACH_REPEAT_H
