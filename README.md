# Autonomous Driving Platform with Tracking Function
## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Build the workspace](#build-the-workspace)
  - [Multi turtlebot Gazebo world](#multi-turtlebot-gazebo-world)
- **[Examples](#examples)**
  - [Multi turtlebot Tracking Stimulation](#multi-turtlebot-tracking-stimulation)
  - [Running on ROS WHILL](running-on-ros-whill)



</br>
</br>

> Authors: [Chen](https://github.com/Chronobreakk) and [David](https://github.com/huangyqs123)

Here is the demo video we apply this Project to the NUS ARC Whill
<!-- > Video Presentation -->
[![ROS SLAM, Perception, and Navigation based on Gazebo simulation](https://s2.loli.net/2024/04/20/I2sQSBXOrcJR5VP.jpg)](https://b23.tv/3KfQj2k)


## About
This project is based on the NUS ME5400A Robotics Project 1, it aims to develope and Autonomous Driving Platform with Tracking Function.


</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)
* [Pysot](https://github.com/STVIR/pysot)

Install Pysot,
```bash
cd
git clone https://github.com/STVIR/pysot.git
```

Setup Pysot environment,
```bash
pip install numpy==1.21.0
pip install pytorch==2.2.1
pip install opencv-python
pip install pyyaml yacs tqdm colorama matplotlib cython tensorboardX
```

Build Pysot extensions,
```bash
cd ~/pysot
python setup.py build_ext --inplace
```

Run this command every time before you run the tracking node, or add it in to ~/.bashrc
```bash
export PYTHONPATH=~/pysot:$PYTHONPATH
```

Download the Pysot model from [Here](https://drive.google.com/drive/folders/1lOOTedwGLbGZ7MAbqJimIcET3ANJd29A)
And put it into ~/ME5400_Final/src/tracking/scripts/pysot/siamrpn_r50_l234_dwxcorr_lt


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

### Build the workspace
```bash
cd
cd ME5400_Final
catkin_make
```
</br>


### Multi turtlebot Gazebo world

Move ff_ros1_ws out

```bash
cd ~/ME5400_Final
mv ff_ros1_ws ..
```

Install all the dependencies through `rosdep`,

```bash
cd ~/ff_ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS 1 and build (Run colcon build twice if you had error output for the first time),

```bash
cd ~/ff_ros1_ws
source /opt/ros/noetic/setup.bash
colcon build
colcon build
```

</br>
</br>

## Examples

Please remember to source first!!!
```bash
source ~/ff_ros1_ws/install/setup.bash
source ~/ME5400_Final/devel/setup.bash
export PYTHONPATH=~/pysot:$PYTHONPATH
```

### Multi turtlebot Tracking Stimulation
The default turtlebot3 model is set as waffle
```bash
export TURTLEBOT3_MODEL=waffle
```

Launch the Gazebo World:
```bash
# Navigate with pre-build map
roslaunch ff_examples_ros1 multi_turtlebot3_ff.launch

# Navigate without pre-build map
roslaunch ff_examples_ros1 multi_nomap.launch 
```

Run the Tracking Node:
```bash
rosrun tracking tracking.py
```

Run the Following scripts:
```bash
cd ~/ME5400_Final/following

# Following use pure cmd_vel control
python3 Cmdvel_following.py

# Following using move_base
python3 movebase_following.py

```

### Running on ROS WHILL
Setup the ROS_WHILL drive accroding to [ros_whill_arc](https://github.com/legubiao/ros_whill_arc).
Setup RealSense D435i drive. [Help](https://zhuanlan.zhihu.com/p/456408345).

Launch ROS WHILL
```bash
roslaunch ros_whill ros_whill.launch
```

Launch RealSense
```bash
roslaunch realsense2_camera rs_camera.launch
```

Run the Tracking Node with RealSense
```bash
rosrun tracking tracking_realsense.py
```

Run the Following scripts:
```bash
cd ~/ME5400_Final/following

# Following use pure cmd_vel control
python3 Cmdvel_following.py

# Following using move_base
python3 movebase_following.py

```






