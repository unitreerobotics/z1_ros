#include <csignal>
#include <sched.h>
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Passive.h"
#include "FSM/State_BackToStart.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Cartesian.h"
#include "FSM/State_JointSpace.h"
#include "FSM/State_MoveJ.h"
#include "FSM/State_MoveL.h"
#include "FSM/State_MoveC.h"
#include "FSM/State_ToState.h"
#include "FSM/State_SaveState.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Trajectory.h"
#include "FSM/State_LowCmd.h"

bool running = true;

//set real-time program
void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        // std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv){
    /* set real-time process */
    setProcessScheduler();

    ros::init(argc, argv, "z1_controller");
    ros::NodeHandle nh("~");

    std::string mcu_ip;
    nh.param<std::string>("udp_to_mcu/mcu_ip", mcu_ip, "192.168.123.110");

    std::string sdk_ip;
    nh.param<std::string>("udp_to_sdk/sdk_ip", sdk_ip, "127.0.0.1");

    int controller_port_mcu;
    nh.param<int>("udp_to_mcu/controller_port", controller_port_mcu, 8881);

    int controller_port_sdk;
    nh.param<int>("udp_to_sdk/controller_port", controller_port_sdk, 8072);

    bool collision_open;
    nh.param<bool>("collision/open", collision_open, false);

    double torque_threhold;
    nh.param<double>("collision/torque_threhold", torque_threhold, 10.0);

    double payload;
    nh.param<double>("collision/payload", payload, 0.03);

    bool has_gripper;
    nh.param<bool>("UnitreeGripper", has_gripper, true);

    EmptyAction emptyAction((int)ArmFSMStateName::INVALID);
    std::vector<KeyAction*> events;
    CtrlComponents *ctrlComp = new CtrlComponents(argc, argv);

    torque_threhold = saturation(torque_threhold, 1., 33.);
    payload = saturation(payload, 0., 3.);

    /* control parameters settings */
    ctrlComp->dt = 1.0/250.;

    ctrlComp->ioInter = new IOUDP(mcu_ip.c_str(), controller_port_mcu);

    ctrlComp->isCollisionOpen = collision_open;
    ctrlComp->collisionTLimit = torque_threhold;

    if(has_gripper) {
        ctrlComp->hasGripper = true;
        ctrlComp->armModel = new Z1Model(ctrlComp->armType, Vec3(0.0382, 0.0, 0.0), 0.80225,
            Vec3(0.0037, 0.0014, -0.0003), Vec3(0.00057593, 0.00099960, 0.00106337).asDiagonal());
    } else {
        ctrlComp->hasGripper = false;
        ctrlComp->armModel = new Z1Model(ctrlComp->armType);
    }
    
    ctrlComp->armModel->addLoad(payload);

    ctrlComp->geneObj();

    ctrlComp->cmdPanel = new ARMSDK(events, emptyAction, sdk_ip.c_str(), controller_port_sdk, 0.002);

    std::vector<FSMState*> states;
    states.push_back(new State_Passive(ctrlComp));
    states.push_back(new State_BackToStart(ctrlComp));
    states.push_back(new State_JointSpace(ctrlComp));
    states.push_back(new State_Cartesian(ctrlComp));
    states.push_back(new State_MoveJ(ctrlComp));
    states.push_back(new State_MoveL(ctrlComp));
    states.push_back(new State_MoveC(ctrlComp));
    states.push_back(new State_LowCmd(ctrlComp));
    states.push_back(new State_SaveState(ctrlComp));
    states.push_back(new State_Teach(ctrlComp));
    states.push_back(new State_TeachRepeat(ctrlComp));
    states.push_back(new State_ToState(ctrlComp));
    states.push_back(new State_Trajectory(ctrlComp));
    states.push_back(new State_Calibration(ctrlComp));

    FiniteStateMachine *fsm;
    fsm = new FiniteStateMachine(states, ctrlComp);

    ctrlComp->running = &running;
    std::signal(SIGINT, [](int signum){ running = false; });
    while(running){
        sleep(1);
    }

    delete fsm;
    delete ctrlComp;
    return 0;
}