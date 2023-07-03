#ifndef BASESTATE_H
#define BASESTATE_H

#include <string>

enum class FSMMode{
    NORMAL,
    CHANGE
};

class BaseState{
public:
    BaseState(mode_t stateNameEnum, std::string stateNameString)
        : _stateNameEnum(stateNameEnum), _stateNameString(stateNameString){}
    virtual ~BaseState() = default;

    virtual void enter() = 0;
    virtual void run_impl() = 0;
    virtual void exit() = 0;
    virtual mode_t checkChange(mode_t mode) = 0;

    bool isState(mode_t stateEnum) { return _stateNameEnum == stateEnum; }
    std::string getStateName() { return _stateNameString; }
    mode_t getStateNameEnum() { return _stateNameEnum; };
protected:
    mode_t _stateNameEnum;
    std::string _stateNameString;
};

#endif // BASESTATE_H