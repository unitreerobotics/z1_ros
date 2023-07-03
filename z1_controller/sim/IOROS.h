#ifndef IOROS_H
#define IOROS_H

#include <ros/ros.h>
#include "IOInterface/IOInterface.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

class IOROS : public IOInterface{
public:
    IOROS();
    ~IOROS(){};

    bool init() override;
    bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
    bool isConnected() override;
private:
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[6], _gripper_sub;
    ros::Publisher _servo_pub[6], _gripper_pub;
    unitree_legged_msgs::MotorState _joint_state[6]{}, _gripperState{};
    unitree_legged_msgs::MotorCmd _joint_cmd[6]{}, _gripperCmd{};
    bool _isConnected = false;

    void _sendCmd(const LowLevelCmd *cmd);
    void _recvState(LowLevelState *state);
    void _initRecv();
    void _initSend();

    void _joint00Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint01Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint02Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint03Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint04Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint05Callback(const unitree_legged_msgs::MotorState& msg);
    void _gripperCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H