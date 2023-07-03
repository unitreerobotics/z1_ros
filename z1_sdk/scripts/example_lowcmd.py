#!/usr/bin/env python3

import time
import rospkg
z1_sdk_path = rospkg.RosPack().get_path('z1_sdk')
import sys
sys.path.append(z1_sdk_path+"/lib")
import unitree_arm_interface
import numpy as np

'''
example_low_cmd.py
An example showing how to control arm motors directly.

Run `roslaunch z1_bringup **_ctrl.launch` first.
'''

# Connect to z1_controller
z1 =  unitree_arm_interface.UnitreeArm("127.0.0.1", 0)
z1.init()


if z1.armState.mode != int(unitree_arm_interface.ArmMode.Passive):
  print("ERROR] Please repower the arm to use lowcmd mode.")
  sys.exit(-1)

# Change to State_lowcmd mode
z1.armCmd.mode = int(unitree_arm_interface.ArmMode.LowCmd)
z1.armCmd.q_d = z1.armState.q
z1.armCmd.gripperCmd.angle = z1.armState.gripperCmd.angle
z1.armCmd.dq_d = np.zeros(6)
z1.armCmd.tau_d = np.zeros(6)

z1.armCmd.Kp = [500, 600, 500, 400, 300, 200]
z1.armCmd.Kd = [5, 5, 5, 5, 5, 5]
z1.sendRecv()

jnt_speed = 0.2

q_d = z1.armState.q_d
for i in range(0, 300):
  q_d[0] += jnt_speed * z1.dt
  z1.armCmd.q_d = q_d
  z1.armCmd.tau_d = np.zeros(6)
  z1.sendRecv()
  time.sleep(z1.dt)

z1.armCmd.mode = int(unitree_arm_interface.ArmMode.Passive)
z1.sendRecv()



