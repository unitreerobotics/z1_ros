## Usage

## 1. Run with moveit

### Source Noetic if not in .bashrc
```bash
source /opt/ros/noetic/setup.bash
source ~/z1_ws/devel/setup.bash
```

### Real Robot Arm

```bash
roslaunch z1_bringup real_arm.launch rviz:=true
```

### Simulated Robot Arm

If you aren't using a real robot arm, you can simulate an arm in Gazebo.

```bash
roslaunch z1_bringup sim_arm.launch UnitreeGripperYN:=true rviz:=true
```

![gazebo](image/gazebo.png)

### Moveit

MoveIt provides a lot of functionality that can greatly speed up development-time:

* Motion Planning
* Manipulation
* Inverse Kinematics
* Control
* 3D Perception
* Collision Checking

An example MoveIt configuration is included in the *z1_moveit_config* package.

There are many ways to interact with MoveIt.  The most interactive way is to use RVIZ.  RVIZ can be started with the above launch scripts using the launch parameter, or can be started after using:

```bash
roslaunch z1_rviz view_robot.launch
```

![rviz_moveit](image/rviz_moveit.png)

### Gripper

The gripper is controlled using an action server. To send a gripper position, publish to the goal topic and set the command/position value to the angle you want the gripper to open to. Zero is closed, and negative values are open. You can monitor the status of the action through the feedback or result topics

```bash
rosrun z1_examples ros_gripper_ctrl -1
```

## 2. Run with udp


### Real robot arm

```bash
roslaunch z1_bringup real_ctrl.launch
```

### Simulates robot arm

This will open the z1_controller and gazebo.

```bash
roslaunch z1_bringup sim_ctrl.launch
```

### z1_sdk

Both cpp and python examples are provided here.

```bash
rosrun z1_sdk example_joint_ctrl
rosrun z1_sdk example_joint_ctrl.py
```

The user can also easily control the motors directly.

```bash
rosrun z1_sdk example_lowcmd
rosrun z1_sdk example_lowcmd.py
```
