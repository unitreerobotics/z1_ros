#!/usr/bin/env python3

import time
import rospkg
z1_sdk_path = rospkg.RosPack().get_path('z1_sdk')
import sys
sys.path.append(z1_sdk_path+"/lib")
import unitree_arm_interface


'''
example_joint_ctrl.py
An example showing how to control joint velocity and position in python.

Run `roslaunch z1_bringup **_ctrl.launch` first.
'''

# Connect to z1_controller
z1 =  unitree_arm_interface.UnitreeArm("127.0.0.1", 0)
z1.init()

# Joint velocity control
jntSpeed = 0.5
dq = [jntSpeed, 0., 0., 0., 0., 0.]
for i in range(0, 400):
    z1.armCmd.mode = unitree_arm_interface.ArmMode.JointSpeedCtrl
    z1.armCmd.dq_d = [jntSpeed, 0, 0, 0, 0, 0] 
    # It cannot modify the elements of a bound C++ std::array using direct index assignment.
    # z1.armCmd.dq_d[0] = 0.5
    z1.sendRecv()
    time.sleep(z1.dt)

# Joint position control
# Get current desired q
q_d = z1.armState.q_d
for i in range(0, 400):
    z1.armCmd.mode = unitree_arm_interface.ArmMode.JointPositionCtrl
    q_d[0] -= jntSpeed * z1.dt
    z1.armCmd.q_d = q_d
    z1.sendRecv()
    time.sleep(z1.dt)

z1.armCmd.mode = unitree_arm_interface.ArmMode.Passive
z1.sendRecv()
