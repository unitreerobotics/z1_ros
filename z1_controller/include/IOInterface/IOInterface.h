#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include <string>
#include <memory>
#include "LowLevelMsg/LowLevelCmd.h"
#include "LowLevelMsg/LowLevelState.h"

class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){};

    virtual bool init() = 0;
    bool initialized() { return initialized_; }

    virtual bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) = 0;
    virtual bool calibration(){return false;};
    virtual bool isConnected() { return false; }

    bool hasGripper = false;
protected:
    bool initialized_ = false;
};

#endif  //IOINTERFACE_H