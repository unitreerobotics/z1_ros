#include "IOROS.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

IOROS::IOROS(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;

    /* start subscriber */
    _initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    /* initialize publisher */
    _initSend();   

    signal(SIGINT, [](int signum){
        ROS_INFO("ROS interface shutting down!");
        ros::shutdown();
    });
    
    initialized_ = true;

    _nm.param<bool>("z1_ctrl/UnitreeGripperYN", hasGripper, false);
}

bool IOROS::isConnected()
{
    return true;
}

bool IOROS::init()
{
    _joint_cmd[0].mode = 10;
    _servo_pub[0].publish(_joint_cmd[0]);

    //check for the presence of gripper
    _gripperCmd.mode = 10;
    _gripper_pub.publish(_gripperCmd);

    ros::spinOnce();
    return _isConnected;
}

bool IOROS::sendRecv(const LowLevelCmd *cmd, LowLevelState *state){
    _sendCmd(cmd);
    _recvState(state);
    return true;
}

void IOROS::_sendCmd(const LowLevelCmd *lowCmd){
    for(size_t i(0); i < 6; ++i){
        _joint_cmd[i].mode = 10;
        _joint_cmd[i].q    = lowCmd->q[i];
        _joint_cmd[i].dq   = lowCmd->dq[i];
        _joint_cmd[i].tau  = lowCmd->tau[i];
        _joint_cmd[i].Kd   = lowCmd->Kd[i];
        _joint_cmd[i].Kp   = lowCmd->Kp[i];

        _servo_pub[i].publish(_joint_cmd[i]);
    }

    if(hasGripper)
    {
        _gripperCmd.q = lowCmd->q.back();
        _gripperCmd.dq = lowCmd->dq.back();
        _gripperCmd.tau = lowCmd->tau.back();
        _gripperCmd.Kp = lowCmd->Kp.back();
        _gripperCmd.Kd = lowCmd->Kd.back();
        _gripper_pub.publish(_gripperCmd);
    }
    
    ros::spinOnce();
}

void IOROS::_recvState(LowLevelState *state){
    for(size_t i(0); i < 6; ++i){
        state->q[i]   = _joint_state[i].q;
        state->dq[i]  = _joint_state[i].dq;
        state->tau[i] = _joint_state[i].tauEst;
    }
    state->q.back() = _gripperState.q;
    state->dq.back() = _gripperState.dq;
    state->tau.back() = _gripperState.tauEst;
}

void IOROS::_initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint01_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint02_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint03_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint04_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint05_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint06_controller/command", 1);
    _gripper_pub  = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/gripper_controller/command", 1);
}

void IOROS::_initRecv(){
    _servo_sub[0] = _nm.subscribe("/z1_gazebo/Joint01_controller/state", 1, &IOROS::_joint00Callback, this);
    _servo_sub[1] = _nm.subscribe("/z1_gazebo/Joint02_controller/state", 1, &IOROS::_joint01Callback, this);
    _servo_sub[2] = _nm.subscribe("/z1_gazebo/Joint03_controller/state", 1, &IOROS::_joint02Callback, this);
    _servo_sub[3] = _nm.subscribe("/z1_gazebo/Joint04_controller/state", 1, &IOROS::_joint03Callback, this);
    _servo_sub[4] = _nm.subscribe("/z1_gazebo/Joint05_controller/state", 1, &IOROS::_joint04Callback, this);
    _servo_sub[5] = _nm.subscribe("/z1_gazebo/Joint06_controller/state", 1, &IOROS::_joint05Callback, this);
    _gripper_sub  = _nm.subscribe("/z1_gazebo/gripper_controller/state", 1, &IOROS::_gripperCallback, this);
}

void IOROS::_joint00Callback(const unitree_legged_msgs::MotorState& msg){
    _isConnected = true;
    _joint_state[0].mode = msg.mode;
    _joint_state[0].q = msg.q;
    _joint_state[0].dq = msg.dq;
    _joint_state[0].ddq = msg.ddq;
    _joint_state[0].tauEst = msg.tauEst;
}

void IOROS::_joint01Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[1].mode = msg.mode;
    _joint_state[1].q = msg.q;
    _joint_state[1].dq = msg.dq;
    _joint_state[1].ddq = msg.ddq;
    _joint_state[1].tauEst = msg.tauEst;
}

void IOROS::_joint02Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[2].mode = msg.mode;
    _joint_state[2].q = msg.q;
    _joint_state[2].dq = msg.dq;
    _joint_state[2].ddq = msg.ddq;
    _joint_state[2].tauEst = msg.tauEst;
}

void IOROS::_joint03Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[3].mode = msg.mode;
    _joint_state[3].q = msg.q;
    _joint_state[3].dq = msg.dq;
    _joint_state[3].ddq = msg.ddq;
    _joint_state[3].tauEst = msg.tauEst;
}

void IOROS::_joint04Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[4].mode = msg.mode;
    _joint_state[4].q = msg.q;
    _joint_state[4].dq = msg.dq;
    _joint_state[4].ddq = msg.ddq;
    _joint_state[4].tauEst = msg.tauEst;
}

void IOROS::_joint05Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[5].mode = msg.mode;
    _joint_state[5].q = msg.q;
    _joint_state[5].dq = msg.dq;
    _joint_state[5].ddq = msg.ddq;
    _joint_state[5].tauEst = msg.tauEst;
}

void IOROS::_gripperCallback(const unitree_legged_msgs::MotorState& msg){
    _gripperState.mode = msg.mode;
    _gripperState.q = msg.q;
    _gripperState.dq = msg.dq;
    _gripperState.ddq = msg.ddq;
    _gripperState.tauEst = msg.tauEst;
}