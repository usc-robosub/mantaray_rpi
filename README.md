# mantaray_rpi
ROS package that handles control on the Raspberry Pi.

## Installation

Install the required ROS packages

```
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-pid
```

Install Eigen3
```
sudo apt-get install libeigen3-dev
```

To install our mantaray library:

```
cd ~/catkin_ws/src
git clone git@github.com:USCAUVTeam/mantaray_rpi.git
cd ..
catkin_make
```

## Launching

```
roslaunch mantaray_rpi mantaray.launch
```

## Dependencies:
* [serialib](https://github.com/imabot2/serialib) - Cross-platform lightweight serial library for C++
* [NGIMU C++ Example](https://github.com/xioTechnologies/NGIMU-C-Cpp-Example) - A wrapper for decoding NGIMU messages
* [OSC99](https://github.com/xioTechnologies/OSC99) - Used for decoding OSC packets
* [robot-localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
* [pid](https://wiki.ros.org/pid)
* [libeigen](https://gitlab.com/libeigen/eigen)