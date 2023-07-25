## Setup

### Dependencies

+ gcc-7 or newer
+ libboost-all-dev
+ libeigen3-dev
+ liburdfdom-dev

```shell
sudo apt install -y libboost-all-dev libeigen3-dev liburdfdom-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
sudo ln -s /usr/include/eigen3/unsupported /usr/local/include/unsupported
```
+ moveit

① build from source: [moveit noetic](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
② Binary install
```
# install noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
sudo apt install ros-noetic-catkin python3-catkin-tools
sudo apt install python3-wstool

sudo apt install -y ros-noetic-moveit-*
sudo apt install -y ros-noetic-joint-trajectory-controller ros-noetic-trac-ik-kinematics-plugin
```

+ [pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

If you want to install python interface, please check the offical page.
```
# Install pinocchio
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF 

make
sudo make install
```

Configure Path, add those lines to `~/.bashrc`
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
```

+ [pybind11](https://github.com/pybind/pybind11) 

If you want to use python interface, please execute the following commands.

```bash
# Install pybind11
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build && cd build
cmake .. -DPYBIND11_TEST=OFF
make -j
sudo make install
```

### catkin_ws

```bash
mkdir -p ~/z1_ws/src
cd ~/z1_ws/src
git clone --recursive https://github.com/unitreerobotics/z1_ros.git
cd ..


source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -yr
catkin_make
source devel/setup.bash
```
