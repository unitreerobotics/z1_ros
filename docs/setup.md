# Setup

## Software

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    source /opt/ros/noetic/setup.bash
    catkin_init_workspace

    git clone --recursive https://github.com/unitreerobotics/z1_sdk.git
    git clone --recursive https://github.com/davesarmoury/unitree_z1_ros.git
    git clone --recursive https://github.com/unitreerobotics/unitree_ros_to_real.git
    git clone --recursive https://github.com/davesarmoury/unitree_ros.git
    git clone --recursive https://github.com/unitreerobotics/unitree_legged_sdk.git

    cd ..
    rosdep install --from-paths src --ignore-src -yr
    catkin_make

    echo "source ~/catkin_ws/devel/setup.bash
    source ~/.bashrc

If you are going to be using a simulation, you can move on to [Usage](usage.md), otherwise, finish setting up your hardware below

## Positioning the Arm

The arm needs to be in its zero position before starting the ROS driver. 

![start_position](./images/start.jpg)

## Power and networking

Plug-in the include power supply with the arm if you are using it on a rigid table.  Ensure it is securely mounted down.  If you are using it on a mobile platform or using a non-standard power-supply, ensure the arm has 24v DC power.

Connect the ethernet cable from the left ethernet port to your ethernet switch or directly to your computer.  Set the IP address of your computer to *192.168.123.99* and verify you can communicate with the robot with the command below

    ping 192.168.123.110

Next step - [Usage](./usage.md)