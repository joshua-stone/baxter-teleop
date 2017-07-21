# Overview

## Install

### 1. Install ROS

This project requires [Ubuntu 14.04 LTS](http://releases.ubuntu.com/14.04/) to set up [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu):

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Set up Hydra controller support

The Razer Hydra controller must be installed and configured:

```bash
sudo apt-get install ros-indigo-razer-hydra
roscd razer_hydra
sudo config/99-hydra-indigo.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules
```

### 3. Test controller

Once Hydra controller support is configured, test it by plugging in a controller and running this command:

```bash
lsusb -d 1532:0300 && [ -e /dev/hydra ] && echo "Controller works" || echo "Controller failed"
```

### 4. Install Baxter

From your catkin workspace, run these commands:

```bash
sudo apt-get -y install git gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser
mkdir -p src/
cd src/
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
wstool update
cd -
source /opt/ros/indigo/setup.bash
catkin_make
```

### 5. Sourcing environment variables

Next, append the generated setup.bash file to ~/.bashrc so ROS environment variables are loaded every time you open a terminal:

```bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "${PWD}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Also be sure that the roscore is set to the one in Baxter:

```bash
echo "export ROS_MASTER_URI=http://baxter.lan:11311" >> ~/.bashrc
source ~/.bashrc
```

### 6. Starting the controller

The controller should now be able to run over ROS:

```bash
roslaunch razer_hydra hydra.launch
```

### 7. Running the Hydra teleop program

Be sure that the base station is placed on a stable surface and positioned like in the [reference manual](https://www.manualslib.com/manual/513849/Razer-Hydra.html?page=3#manual).

When ready, run this:

```bash
python razer.py
```
