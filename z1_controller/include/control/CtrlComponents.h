#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include <string>
#include <iostream>
#include "common/utilities/loop.h"
#include "message/arm_common.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/utilities/CSVTool.h"
#include "model/ArmModel.h"
#include "interface/IOUDP.h"
#include "interface/IOROS.h"
#include "control/armSDK.h"

using namespace std;

struct CtrlComponents{
public:
    CtrlComponents(int argc, char**argv);
    ~CtrlComponents();

    std::string armConfigPath;
    CmdPanel *cmdPanel;
    IOInterface *ioInter;
    Z1Model *armModel;
    CSVTool *stateCSV;

    SendCmd sendCmd{}; // cmd that receive from SDK 
    RecvState recvState{};// state that send to SDK
    
    //config
    double dt;
    bool *running;
    Control ctrl = Control::SDK;
    bool hasGripper{};
    bool isCollisionOpen{};
    double collisionTLimit{};
    bool isPlot{};
    int trajChoose = 1;
    size_t armType = 36;

    const std::string controller_version = "2.2.5";

    void geneObj();
    void writeData();
};

#endif  // CTRLCOMPONENTS_H