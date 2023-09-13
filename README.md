# mantaray_rpi
ROS package that handles control on the Raspberry Pi.

## Installation

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