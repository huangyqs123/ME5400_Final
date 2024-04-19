# Perception of Autonomous Driving Platform with TrackingFunction
## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Turtlebot Simulation](#turtlebot-simulation)
  - [Multi Turtlebot Simulation](#multi-turtlebot-simulation)
  - [Download Scripts](#download-scripts)
- **[Examples](#examples)**
  - [Single Turtlebot Simulation](#single-turtlebot-simulation)
  - [Multi Turtlebot Simulation](#multi-turtlebot-simulation)
  - [Scripts For objectfollowing](#scripts-for-objectfollowing)


</br>
</br>

> Authors: [Chen](https://github.com/Chronobreakk) and [David](https://github.com/huangyqs123)

Here is the demo video we apply this Project to the NUS ARC Whill
<!-- > Video Presentation -->
[![ROS SLAM, Perception, and Navigation based on Gazebo simulation](https://s2.loli.net/2024/04/20/I2sQSBXOrcJR5VP.jpg)](https://b23.tv/3KfQj2k)


## About
This project is based on the NUS ME5400A Robotics Project 1, it aims make one turtlebot following another.

This repo is the python scripts file repo of these project. This project also includes two other repos: [Turtlebot Simulation](https://github.com/Chronobreakk/5400turtlebot) and [Multiturtlebot Simulation](https://github.com/Chronobreakk/ff_ros1_ws).


</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)

Install all non-ROS prerequisite packages,

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-server ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro  ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation  ros-noetic-interactive-markers rviz

sudo apt update && sudo apt install \
  git wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./free_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>


### Turtlebot Simulation
Start a new ROS 1 workspace, and pull it in your default ROS workspace,

```bash
mkdir -p ~/catkin_ws/src/turtlebot
cd ~/catkin_ws/src/turtlebot
git clone https://github.com/Chronobreakk/5400turtlebot.git
cd ~/catkin_ws
catkin_make
```
</br>

### Multi Turtlebot Simulation

Start a new ROS 1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/ff_ros1_ws
cd ~/ff_ros1_ws
git clone https://github.com/Chronobreakk/ff_ros1_ws.git
cd ~/ff_ros1_ws/src
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.7.x
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS 1 and build,

```bash
cd ~/ff_ros1_ws
source /opt/ros/noetic/setup.bash
colcon build
```

</br>

### Download Scripts

```bash
mkdir -p ~/object-following-scripts
cd ~/object-following-scripts
git clone https://github.com/Chronobreakk/Object-following-with-move_base.git
```
</br>
</br>

## Examples

Please remember to source first!!!
```bash
source ~/ff_ros1_ws/install/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Single turtlebot Simulation:
```bash
# Open the gazebo world
roslaunch turtlebot3_gazebo turtlebot3_world.launch 

# Navigate with pre-build map
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 

# Navigate without pre-build map
roslaunch turtlebot3_navigation withoutmap_nav.launch
```

</br>

### Multi turtlebot Simulation:
The default turtlebot3 model is set as waffle

You can run this file by follow steps:
```bash
# Navigate with pre-build map
roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch

# Navigate without pre-build map
roslaunch ff_examples_ros1 multi_nomap.launch 
```

</br>


### Scripts for objectfollowing:
You need to launch multi turtlebot environment first

You can run this file by follow steps:
```bash
cd ~/object-following-scripts
# Following use pure cmd_vel control
python3 Cmdvel_following.py

# Following using move_base
python3 movebase_following.py
```
For the ARC ROS Whill，please run:
```bash
cd ~/object-following-scripts
# Following use pure cmd_vel control
python3 Cmdvel_following.py

# Following using move_base
python3 ROS_whill_following.py
```

