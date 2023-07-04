# Z1_ROS

![structure](doc/image/structure.png)

## Packages

+ **z1_bingup** - Simple launching package for starting necessary components of the MoveIt driver.
+ **z1_controller** - For direct control of the robot arm and connection to the SDK.
+ **z1_hw** - Hardware interface between robot and MoveIt. Creates a joint_trjectory_controller for the arm and an action server for the gripper.
+ **z1_moveit_config** - Example configuration for using MoveIt with a Z1 arm.
+ **z1_rviz** - Simple RVIZ launches and configurations for displaying a currently running arm, or the URDF to verify the setup.
+ **z1_examples** - Examples for using moveit.
+ **z1_sdk** - Examples for using udp communication.


Welcome to the ROS driver for the Unitree Z1!

[Setup](doc/setup.md) - Configure the environment

[Usage](doc/usage.md) - Contrl the z1 arm

**Attention:** The `z1_controller` package in `z1_ros` is different from [https://github.com/unitreerobotics/z1_controller](https://github.com/unitreerobotics/z1_controller), and [https://github.com/unitreerobotics/z1_sdk](https://github.com/unitreerobotics/z1_sdk) is not compatible with this package.

See details [z1_controller](doc/controller.md)